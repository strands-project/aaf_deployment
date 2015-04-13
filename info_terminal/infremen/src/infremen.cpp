#include <stdlib.h>
#include "ros/ros.h"
#include <mongodb_store/message_store.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <infremen/infremenConfig.h>
#include <strands_navigation_msgs/NavStatistics.h>
#include <strands_navigation_msgs/TopologicalMap.h>
#include <strands_navigation_msgs/TopologicalNode.h>
#include <strands_executive_msgs/AddTask.h>
#include <strands_executive_msgs/CancelTask.h>
#include <strands_executive_msgs/SetExecutionStatus.h>
#include <strands_executive_msgs/CreateTask.h>
#include <mongodb_store_msgs/StringPair.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <strands_navigation_msgs/GetTaggedNodes.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <scitos_msgs/BatteryState.h>
#include <info_task/Clicks.h>
#include <infremen/InfremenResult.h>
#include <infremen/AtomicInteraction.h>
#include "CFrelementSet.h"
#include <time.h> 
#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"
#include <boost/foreach.hpp>
#include <sstream>
#include <cassert>

using namespace mongodb_store;
using namespace std;

//FIXED parameters
int windowDuration = 300;
int rescheduleInterval = 86400;

//standard parameters
string collectionName;
string scheduleDirectory;

//runtine parameters
float explorationRatio = 0.5;
int8_t   minimalBatteryLevel = 80;
int32_t   minimalBatteryLevelTime = 0;
int interactionTimeout = 30;
int maxTaskNumber = 5;
int taskDuration = 180;	
int taskPriority = 1;	
bool debug = false;

//ROS communication
ros::NodeHandle *n;
MessageStoreProxy *messageStore;
ros::Subscriber robotPoseSub;
ros::Subscriber currentNodeSub;
ros::Subscriber interfaceSub; 
ros::Subscriber batterySub;
ros::Subscriber infoTaskSub;
ros::Subscriber guiSub; 
ros::ServiceClient nodeListClient;
ros::ServiceClient taskAdder;
ros::ServiceClient taskCreator;
ros::ServiceClient taskCancel;

const mongo::BSONObj EMPTY_BSON_OBJ;

//fremen component
CFrelementSet frelementSet;

//state variables
int lastTimeSlot = -1;
int currentTimeSlot = -1;
int numCurrentTasks = 0; 
string nodeName = "ChargingPoint";
string closestNode = "ChargingPoint";
int32_t lastInteractionTime = -1;
geometry_msgs::Pose lastPose;
bool forceCharging = false;
int timeOffset = 0;

//times and nodes of schedule
uint32_t timeSlots[10000];
int nodes[10000];
int taskIDs[10000];
int numNodes = 0;

uint32_t getMidnightTime(uint32_t givenTime)
{
	return ((givenTime+timeOffset)/rescheduleInterval)*rescheduleInterval-timeOffset;
}

//parameter reconfiguration
void reconfigureCallback(infremen::infremenConfig &config, uint32_t level) 
{
	ROS_INFO("Reconfigure Request: %lf %d %d %d %d", config.explorationRatio, config.minimalBatteryLevel, config.interactionTimeout, config.maxTaskNumber, config.taskDuration);
	explorationRatio = config.explorationRatio;
	minimalBatteryLevel = config.minimalBatteryLevel;
	interactionTimeout = config.interactionTimeout;
	maxTaskNumber = config.maxTaskNumber;
	taskDuration = config.taskDuration;
	taskPriority = config.taskPriority;
	debug = config.verbose;
}

//listen to battery and set forced charging if necessary
void batteryCallBack(const scitos_msgs::BatteryState &msg)
{
	/*TODO learn from experience about energy consumption, plan ahead*/
	ROS_DEBUG("Infremen: Battery level %i %i",msg.lifePercent,minimalBatteryLevel);
	if (minimalBatteryLevel > msg.lifePercent) forceCharging = true; else forceCharging = false;
}	

/*get robot pose*/
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	lastPose = *msg;
	//if (debug) ROS_INFO("Infremen: Robot at %lf %lf %lf.",lastPose.position.x,lastPose.position.y,lastPose.position.z);
}

/*detailed info on interation*/
void interacted(const std_msgs::Int32::ConstPtr& msg)
{
	char testTime[1000];
	ros::Time currentTime = ros::Time::now();
	lastInteractionTime = currentTime.sec;

	time_t timeInfo = currentTime.sec;
	strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
	if (debug) ROS_INFO("Infremen: Screen %i clicked at %s close to %s on %s.",msg->data,nodeName.c_str(),closestNode.c_str(),testTime);

	infremen::AtomicInteraction lastInteraction;

	lastInteraction.time = currentTime.sec;
	lastInteraction.screen = msg->data;
	lastInteraction.waypoint = closestNode;
	lastInteraction.infoWaypoint = nodeName;

	lastInteraction.robotPosePhi = tf::getYaw(lastPose.orientation);
	lastInteraction.robotPoseX = lastPose.position.x;
	lastInteraction.robotPoseY = lastPose.position.y;
	lastInteraction.robotPoseZ = lastPose.position.z;
	string id(messageStore->insertNamed(collectionName, lastInteraction));
	//if (debug) cout<<"Interaction \""<<lastInteraction<<"\" inserted with id "<<id<<endl;
}

/*updates the closest node*/
void getCurrentNode(const std_msgs::String::ConstPtr& msg)
{
	closestNode = msg->data;
	if (frelementSet.find(msg->data.c_str())>-1){
		nodeName = msg->data;
		if (debug) ROS_INFO("Closest InfoTerminal node switched to %s.",nodeName.c_str());
	}else{
		if (debug) ROS_INFO("Closest node %s - however, it's not an Infoterminal node.",msg->data.c_str());
	}
}

/*loads relevant nodes from the map description*/
int getRelevantNodes()
{
	int result = -1;
	uint32_t times[1];
	unsigned char signal[1];
	strands_navigation_msgs::GetTaggedNodes srv;
	srv.request.tag = "InfoTerminal";
	if (nodeListClient.call(srv))
	{
		for (int i=0;i<srv.response.nodes.size();i++) frelementSet.add(srv.response.nodes[i].c_str(),times,signal,0,true);
		numNodes = frelementSet.numFrelements;
		ROS_INFO("Number of infoterminal nodes: %d",numNodes);
		for (int i=0;i<numNodes;i++) ROS_INFO("Infoterminal waypoint %i: %s.",i,frelementSet.frelements[i]->id);
		result = numNodes;
	}
	else
	{
		ROS_ERROR("Failed to obtain InfoTerminal-relevant nodes");
	}
	return result;	
}

/*get closest node*/
void getClosestNode()
{
	return;//TODO Nodes will be ordered in the future
	strands_navigation_msgs::GetTaggedNodes srv;
	srv.request.tag = "InfoTerminal";
	if (nodeListClient.call(srv))
	{
		if (srv.response.nodes.size() > 0 ) nodeName = srv.response.nodes[0];
		if (debug) ROS_INFO("The closest InfoTerminal node is %s.",nodeName.c_str());
	}
	else
	{
		ROS_ERROR("Failed to obtain InfoTerminal-relevant nodes");
	}	
}

void addResult(const char *name,unsigned char state,uint32_t time)
{
	uint32_t times[1];
	unsigned char signal[1];
	signal[0] = state;
	times[0] = time;
	int statesAdded = frelementSet.add(name,times,signal,1,false);
	if (statesAdded ==1){ 
		ROS_INFO("Infremen: added interation at %s:%i-%i.",name,times[0],signal[0]);
	}else if (statesAdded == 0){
		ROS_WARN("Infremen: interation at %s:%i-%i indicated, but it seems to be already in the model.",name,times[0],signal[0]);
	}else{
		ROS_WARN("Infremen: interation at %s:%i-%i indicated, but node is not tagged as InfoTerminal.",name,times[0],signal[0]);
	}
}

/*retrieve interactions from the database and build the FreMen model*/
void retrieveInteractions(uint32_t lastTime)
{
	char testTime[1000];
	vector< boost::shared_ptr<infremen::InfremenResult> > results;
	messageStore->query<infremen::InfremenResult>(results);
	BOOST_FOREACH( boost::shared_ptr<infremen::InfremenResult> p,  results)
	{
		time_t timeInfo = p->time;
		strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
		ROS_INFO("There were %d interaction at %s at waypoint %s.",p->number,testTime,p->waypoint.c_str());
		if (lastTime > p->time){ 
			if (p->number>1) addResult(p->waypoint.c_str(),1,p->time); else addResult(p->waypoint.c_str(),0,p->time); 
		}
	}
}

/*generates a schedule and saves it in a file*/
int generateNewSchedule(uint32_t givenTime)
{
	/*establish relevant time frame*/
	int numSlots = 24*3600/windowDuration;
	uint32_t timeSlots[numSlots];
	uint32_t midnight = getMidnightTime(givenTime);
	retrieveInteractions(midnight);

	/*create timeslots*/
	for (int i = 0;i<numSlots;i++) timeSlots[i] = midnight+3600*24/numSlots*i; 
	numNodes = frelementSet.numFrelements;

	/*evaluate the individual nodes*/
	int p = 0;
	float entropy[1],probability[1];
	uint32_t times[1];
	char dummy[1000];
	float wheel[numNodes];
	float lastWheel;	
	for (int s=0;s<numSlots;s++)
	{
		/*evaluate nodes for the given time*/
		lastWheel = 0;
		for (int i=0;i<numNodes;i++)
		{
			times[0] = timeSlots[s]; 
			frelementSet.frelements[i]->estimateEntropy(times,entropy,1,1);
			frelementSet.frelements[i]->estimate(times,probability,1,1);
			lastWheel += explorationRatio*entropy[0]+(1-explorationRatio)*probability[0];
			wheel[i] = lastWheel;
		}

		/*some debug prints*/
		if (debug)
		{
			sprintf(dummy, "Cumulative node evaluations: ");
			for (int i=0;i<numNodes;i++) sprintf(dummy,"%s %.3f ",dummy,wheel[i]);
			sprintf(dummy,"%s %.3f",dummy,lastWheel);
			ROS_INFO("%s",dummy);
		}
		/*random seletion of a particular node*/
		int node = 0;
		float randomNum = 0;
		randomNum = (float)rand()/RAND_MAX*lastWheel;
		p = 0;
		while (p <numNodes && randomNum > wheel[p]) p++;
		nodes[s] = p;
	}

	/*save a file with a schedule TODO - save to mongo + interface calendar*/
	time_t timeInfo = midnight;
	char fileName[1000];
	strftime(dummy, sizeof(dummy), "InfoTerminal-Schedule-%Y-%m-%d.txt",localtime(&timeInfo));
	sprintf(fileName,"%s/%s",scheduleDirectory.c_str(),dummy);
	printf("Generating schedule from %s\n",fileName);
	FILE* file = fopen(fileName,"w+");
	if (file == NULL)ROS_ERROR("Could not open Schedule file %s.",fileName);
	for (int s=0;s<numSlots;s++)
	{
		sprintf(dummy,"%i ",s);
		timeInfo = timeSlots[s];
		strftime(dummy, sizeof(dummy), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
		fprintf(file,"%ld %s %s\n",timeInfo,dummy,frelementSet.frelements[nodes[s]]->id);
	}
	fclose(file);
}

int generateSchedule(uint32_t givenTime)
{
	char dummy[1000];
	int numSlots = 24*3600/windowDuration;
	uint32_t midnight =  getMidnightTime(givenTime);
	if (debug)
	{
		ROS_INFO("TIME: %i",givenTime);
		ROS_INFO("MIDNIGHT: %i %i",midnight,midnight-givenTime);
	}

	/*create timeslots*/
	for (int i = 0;i<numSlots;i++) timeSlots[i] = midnight+3600*24/numSlots*i; 
	
	/*open a schedule file - create a new one if it does not exist*/
	char fileName[1000];
	time_t timeInfo = midnight;
	strftime(dummy, sizeof(dummy), "InfoTerminal-Schedule-%Y-%m-%d.txt",localtime(&timeInfo));
	sprintf(fileName,"%s/%s",scheduleDirectory.c_str(),dummy);
	printf("Retrieving schedule from %s\n",fileName);
	FILE* file = fopen(fileName,"r");
	if (file == NULL){
		printf("Schedule file not found %s\n",fileName);
		generateNewSchedule(givenTime);
		file = fopen(fileName,"r");
	}

	/*read schedule file*/
	int checkReturn;
	uint64_t slot;
	char nodeName[1000];
	char testTime[1000];
	for (int s=0;s<numSlots;s++){
		checkReturn = fscanf(file,"%ld %s %s\n",&slot,dummy,nodeName);
		timeInfo = slot;
		strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
		if (checkReturn != 3) 			ROS_ERROR("Infoterminal schedule file %s is corrupt at line %i (wrong number of entries %i)!",dummy,s,checkReturn);
		else if (slot != timeSlots[s]) 		ROS_ERROR("Infoterminal schedule file %s is corrupt at line %i (ROS time mismatch)!",dummy,s);
		else if (strcmp(testTime,dummy)!=0)	ROS_ERROR("Infoterminal schedule file %s is corrupt at line %i (time in seconds does not match string time)!",dummy,s);
		else {
			nodes[s] = frelementSet.find(nodeName);
			if (nodes[s] < 0) ROS_ERROR("Infoterminal schedule file %s is corrupt at line %i (node %s is not tagged as InfoTerminal in topoogical map)!",dummy,s,nodeName);
		}
	}
	fclose(file);
}

int getNextTimeSlot(int lookAhead)
{
	char dummy[1000];
	char testTime[1000];
	time_t timeInfo;
	int numSlots = 24*3600/windowDuration;
	ros::Time currentTime = ros::Time::now();
	uint32_t givenTime = currentTime.sec+lookAhead*windowDuration;
	uint32_t midnight = getMidnightTime(givenTime);
	if (timeSlots[0] != midnight)
	{
		ROS_INFO("Generating new schedule!");
		generateSchedule(givenTime);
	} 
	int currentSlot = (givenTime-timeSlots[0])/windowDuration;
	if (debug)
	{
		timeInfo = givenTime;
		strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
		sprintf(dummy,"time %i %s - slot %i: going to node %i(%s).",givenTime,testTime,currentSlot,nodes[currentSlot],frelementSet.frelements[nodes[currentSlot]]->id);
		//ROS_INFO("%s",dummy);
	}
	if (currentSlot >= 0 && currentSlot < numSlots) return currentSlot;
	ROS_ERROR("Infoterminal schedule error: attempting to get task in a non-existent time slot.");
	return -1;	
}

/*creates a task for the given slot*/
int createTask(int slot)
{
	char dummy[1000];
	char testTime[1000];
	time_t timeInfo = timeSlots[slot];
	strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));

	strands_executive_msgs::CreateTask srv;
	if (taskCreator.call(srv))
	{
		strands_executive_msgs::Task task=srv.response.task;
		task.start_node_id = frelementSet.frelements[nodes[slot]]->id;
		task.end_node_id = frelementSet.frelements[nodes[slot]]->id;
		task.priority = taskPriority;

  		task.start_after =  ros::Time(timeSlots[slot]+5,0);
		task.end_before = ros::Time(timeSlots[slot]+windowDuration - 2,0);
		task.max_duration = ros::Duration(taskDuration,0);
		if (slot > 1 && nodes[slot]==nodes[slot-1]) task.max_duration = ros::Duration(windowDuration-10,0);
		strands_executive_msgs::AddTask taskAdd;
		taskAdd.request.task = task;
		if (taskAdder.call(taskAdd))
		{
			sprintf(dummy,"%s for timeslot %i on %s.",frelementSet.frelements[nodes[slot]]->id,slot,testTime);
			ROS_INFO("Task %ld created %s at ", taskAdd.response.task_id,dummy);
			taskIDs[slot] = taskAdd.response.task_id;
		}
	}else{
		sprintf(dummy,"Could not create task for timeslot %i: At %s go to %s.",slot,testTime,frelementSet.frelements[nodes[slot]]->id);
		ROS_ERROR("%s",dummy);
		taskIDs[slot] = -1;
	}
}

/*drops and reschedules the following task on special conditions*/
int modifyNextTask(int slot)
{
	int lastNodeID = frelementSet.find(nodeName.c_str());
	int chargeNodeID = frelementSet.find("ChargingPoint");
	bool changeTaskFlag = false;
	/*charge when low on battery*/
	if (chargeNodeID != -1 && forceCharging){
		 nodes[slot]=chargeNodeID;
		 changeTaskFlag = true;
		 ROS_INFO("Task %i should be changed to charging.",taskIDs[slot]);
	}
	/*do not run away during interactions*/
	if (lastNodeID != -1 && (timeSlots[slot] - lastInteractionTime) < interactionTimeout)
	{
		 nodes[slot]=lastNodeID;
		 changeTaskFlag = true;
		 ROS_INFO("Task %i should be changed to last waypoint.",taskIDs[slot]);
	}
	/*do not run away during interactions*/
	if (changeTaskFlag)
	{
		strands_executive_msgs::CancelTask taskCanc;
		taskCanc.request.task_id = taskIDs[slot];
		if (taskCancel.call(taskCanc)){
			if (taskCanc.response.cancelled){
				ROS_INFO("Task %i cancelled, replanning.",taskIDs[slot]);
				createTask(slot);
			}else{
				ROS_INFO("Scheduler refuses to cancel task %i.",taskIDs[slot]);
			}
		}
	}
}

/*records interaction to mongodb*/
void guiCallBack(const info_task::Clicks &msg)
{
	static bool firstGuiCallBack = true;
	if (firstGuiCallBack==false)
	{
		ROS_INFO("Infremen: there were %ld interactions.",msg.page_array.size());
		getClosestNode();
		infremen::InfremenResult itr;
		ros::Time currentTime = ros::Time::now();
		itr.time = currentTime.sec-windowDuration/2;
		itr.number = msg.page_array.size();
		itr.waypoint = nodeName;

		messageStore->insertNamed(collectionName, itr);
	}else{
		firstGuiCallBack = false;
	}
}

/*retrieve interactions from the database and build the FreMen model*/
void printAllInteractions(uint32_t lastTime)
{
	char testTime[1000];
	vector< boost::shared_ptr<infremen::InfremenResult> > results;
	messageStore->queryNamed<infremen::InfremenResult>(collectionName,results,false);
	BOOST_FOREACH( boost::shared_ptr<infremen::InfremenResult> p,  results)
	{
		time_t timeInfo = p->time;
		strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
		ROS_INFO("There were %d interaction at %s at waypoint %s.",p->number,testTime,p->waypoint.c_str());
	}

	vector< boost::shared_ptr<infremen::AtomicInteraction> > atoms;
	string id;
	messageStore->queryNamed<infremen::AtomicInteraction>(collectionName,atoms,false);
	BOOST_FOREACH( boost::shared_ptr<infremen::AtomicInteraction> a,  atoms)
	{
		time_t timeInfo = a->time;
		strftime(testTime, sizeof(testTime), "%Y-%m-%d_%H:%M:%S",localtime(&timeInfo));
		ROS_INFO("Screen switched to %d interaction at %s at waypoint %s(%s) - robot pose %f %f %f.",a->screen,testTime,a->infoWaypoint.c_str(),a->waypoint.c_str(),a->robotPoseX,a->robotPoseY,a->robotPosePhi);
	}
}

int main(int argc,char* argv[])
{
	//determine ROS time to local dignight offset - used to replan at midnight
	tzset();
	timeOffset = timezone+daylight*3600;
	if (debug) ROS_DEBUG("Local time offset %i",timeOffset);

	//initialize ros and datacentre
	ros::init(argc, argv, "infremen");
	n = new ros::NodeHandle();
        messageStore = new MessageStoreProxy(*n,"message_store");
	//load parameters
	n->param<std::string>("/infremen/collectionName", collectionName, "WharfTest2");
	n->param<std::string>("/infremen/scheduleDirectory", scheduleDirectory, "/localhome/strands/schedules");
	n->param("/infremen/taskPriority", taskPriority,1);
	n->param("/infremen/verbose", debug,false);
	//debug prints
	printAllInteractions(-1);

	//initialize dynamic reconfiguration feedback
	dynamic_reconfigure::Server<infremen::infremenConfig> server;
	dynamic_reconfigure::Server<infremen::infremenConfig>::CallbackType dynSer;
	dynSer = boost::bind(&reconfigureCallback, _1, _2);
	server.setCallback(dynSer);

	//to get the robot position
	robotPoseSub = n->subscribe("/robot_pose", 1, poseCallback); 
	//to get the current node 
	currentNodeSub = n->subscribe("/closest_node", 1, getCurrentNode);
	//to determine if charging is required
	batterySub = n->subscribe("battery_state", 1, batteryCallBack);
	//to receive feedback from task_info
	infoTaskSub = n->subscribe("/info_terminal/task_outcome", 1, guiCallBack);
	//to receive feedback about the task outcome 
	infoTaskSub = n->subscribe("/info_terminal/task_outcome", 1, guiCallBack);
	//to receive feedback from the gui itself 
	guiSub = n->subscribe("/info_terminal/active_screen", 1, interacted);
	//to get relevant nodes
	nodeListClient = n->serviceClient<strands_navigation_msgs::GetTaggedNodes>("/topological_map_manager/get_tagged_nodes");
	//to create task objects
	taskCreator = n->serviceClient<strands_executive_msgs::CreateTask>("/info_task_server_create");
	//to add tasks to the schedule
	taskAdder = n->serviceClient<strands_executive_msgs::AddTask>("/task_executor/add_task");
	//to remove tasks from the schedule
	taskCancel = n->serviceClient<strands_executive_msgs::CancelTask>("/task_executor/cancel_task");
	//get nodes tagged as InfoTerminal	
	if (getRelevantNodes() < 0)
	{
		 ROS_ERROR("Topological navigation does not report about tagged nodes. Is it running?");
		 return -1;
	}
	if (getRelevantNodes() == 0)
	{
		 ROS_ERROR("There are no Info-Terminal relevant nodes in the topological map ");
		 return -1;
	}
	//generate schedule
	ros::Time currentTime = ros::Time::now();
	//buildModels(currentTime.sec);
	generateSchedule(currentTime.sec);
	//to start scheduler - for standalone testing 
	ros::ServiceClient taskStart;
	taskStart = n->serviceClient<strands_executive_msgs::SetExecutionStatus>("/task_executor/set_execution_status");

	//TODO for testing only - remove later 
	/*strands_executive_msgs::SetExecutionStatus runExec;
	runExec.request.status = true;
	if (taskStart.call(runExec)) ROS_INFO("Task execution enabled.");*/

	while (ros::ok())
	{
		ros::spinOnce();
		sleep(1);
		if (debug) ROS_INFO("Infremen tasks: %i %i",numCurrentTasks,maxTaskNumber);
		currentTimeSlot = getNextTimeSlot(0);
		if (currentTimeSlot!=lastTimeSlot){
			modifyNextTask(currentTimeSlot);
			numCurrentTasks--;
			if (numCurrentTasks < 0) numCurrentTasks = 0;
		}
		if (numCurrentTasks < maxTaskNumber)
		{
			lastTimeSlot=currentTimeSlot;
			int a=getNextTimeSlot(numCurrentTasks);
			if ( a >= 0){
				createTask(a);
				numCurrentTasks++;
			}
		}
	}
	return 0;
}

