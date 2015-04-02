#include <stdlib.h>
#include "ros/ros.h"
#include <mongodb_store/message_store.h>
#include <boost/foreach.hpp>
#include <strands_navigation_msgs/NavStatistics.h>
#include <strands_navigation_msgs/TopologicalMap.h>
#include <strands_navigation_msgs/TopologicalNode.h>
#include <strands_executive_msgs/AddTask.h>
#include <strands_executive_msgs/Task.h>
#include <strands_executive_msgs/SetExecutionStatus.h>
#include <infremen/CreateInFremenTask.h>
#include <mongodb_store_msgs/StringPair.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include "CFrelementSet.h"
#include <time.h> 

#define MAX_EDGES 10000

const mongo::BSONObj EMPTY_BSON_OBJ;
int waitingInterval = 60;
int taskLength = 120;
int numTasks = 5;

using namespace mongodb_store;
using namespace std;

ros::NodeHandle *n;
ros::Publisher  taskPub;
bool debug = false;
ros::ServiceClient taskAdder;
ros::ServiceClient taskClear;
ros::ServiceClient taskStart;
std_srvs::Empty dummySrv;

CFrelementSet frelementSet;
string ide;
string nodeName;
uint32_t 	times[100000];
unsigned char 	state[100000];
float wheel[1000];
int numNodes = 0;
float lastWheel = 0;

CTimer timer;
CTimer interactionTimer;

void recalculateModels()
{
	numNodes = frelementSet.numFrelements;
	float explorationRatio = 0.5;
	float entropy[1],probability[1];
	uint32_t times[1];
	times[0] = 0;//TODO current time
	lastWheel = 0;
	for (int i=0;i<numNodes;i++)
	{
		frelementSet.frelements[i]->estimateEntropy(times,entropy,1,1);
		frelementSet.frelements[i]->estimate(times,probability,1,1);
		lastWheel += explorationRatio*entropy[0]+(1-explorationRatio)*probability[0];
		wheel[i] = lastWheel;
	}
	printf("Wheel: ");
	for (int i=0;i<numNodes;i++) printf("%.3f ",wheel[i]);
	printf("%.3f \n",lastWheel);
}

/*loads nodes from the map description*/
void loadMap(const strands_navigation_msgs::TopologicalMap::ConstPtr& msg)
{
	
  	for (int i=0;i<msg->nodes.size();i++)
	{
		printf("MAP: %s.\n",msg->nodes[i].name.c_str());
		frelementSet.add(msg->nodes[i].name.c_str(),times,state,0);
	}
	numNodes = frelementSet.numFrelements;
  	for (int i=0;i<numNodes;i++)
	{
		printf("Final: %s.\n",frelementSet.frelements[i]->id);
	}
	recalculateModels();
}

/*loads nodes from the map description*/
void getCurrentNode(const std_msgs::String::ConstPtr& msg)
{
	nodeName = msg->data;
	printf("Current node %s.\n",nodeName.c_str());
	timer.reset(waitingInterval);
}




void addResult(const char *name,unsigned char state)
{
	uint32_t times[1];
	unsigned char signal[1];
	signal[0] = state;
	printf("%s-%i-%i\n",nodeName.c_str(),times[0],signal[0]);
	times[0] = (uint32_t)ros::Time::now().toSec();
	frelementSet.add(nodeName.c_str(),times,signal,1);
}

/*loads nodes from the map description*/
void getCommand(const std_msgs::String::ConstPtr& msg)
{
	printf("Current command %s.\n",msg->data.c_str());
	if (msg->data == "INFO_TERMINAL"){
		timer.reset(waitingInterval);	
		printf("Info request at %s.",nodeName.c_str());
		if (interactionTimer.timeOut())
		{
			printf(" - FreMEnized.");
			interactionTimer.reset(waitingInterval/2);
			addResult(msg->data.c_str(),1);
		}
		printf("\n");
	}
}

void interacted(const std_msgs::Int32::ConstPtr& msg)
{
	printf("Infoterminal %i clicked at %s.",msg->data,nodeName.c_str());
	if (interactionTimer.timeOut())
	{
		printf(" - FreMEnized.");
		interactionTimer.reset(waitingInterval/2);
		addResult(nodeName.c_str(),1);
	}
	printf("\n");
}

void timeOut()
{
	timer.reset(waitingInterval);	
	addResult(nodeName.c_str(),0);
}

/*recalculates models*/
void recalculate(const std_msgs::Empty::ConstPtr& msg)
{
	recalculateModels();
}





/*generates a task - invoked every minute*/
bool generateTasks(int num, std::vector< ::strands_executive_msgs::Task > & tasks)	
{
	time_t t;
	srand((unsigned int)time(&t));
	for (int i=0;i<num;i++)
	{
		ROS_INFO("generating info_terminal task");
		int node = 0;
		float randomNum = 0;
		if (numNodes > 0){
			randomNum = (float)rand()/RAND_MAX*lastWheel;
			int p = 0;
			while (p <numNodes && randomNum > wheel[p]) p++;
			node = p;
		}

		//printf("Throw: %lf %i %s\n",randomNum,node,frelementSet.frelements[node]->id);

		strands_executive_msgs::Task task;
		task.start_node_id = frelementSet.frelements[node]->id;
		task.action = "/go_to_person_action";
		task.start_after = ros::Time::now()+ros::Duration(taskLength*i);
		task.end_before  = ros::Time::now()+ros::Duration(taskLength*(i+1)-1);
		task.max_duration = ros::Duration(waitingInterval);

		mongodb_store_msgs::StringPair pair;

		pair.first = "geometry_msgs/PoseStamped";
		pair.second = ide; 
		task.arguments.push_back(pair); 

		pair.first = "Task.BOOL_TYPE";
		pair.second = "True";
		task.arguments.push_back(pair);

		pair.first = task.BOOL_TYPE;
		pair.second = "False";
		task.arguments.push_back(pair);

		pair.first = task.FLOAT_TYPE;
		pair.second = "60";
		task.arguments.push_back(pair);

		tasks.push_back(task);
	}
	return true;
}

/*generates a task - invoked every minute*/
bool generateTasks(infremen::CreateInFremenTask::Request& req, infremen::CreateInFremenTask::Response& resp)	
{
	return generateTasks(req.num, resp.tasks);
}

void submitTasks() 
{
	std::vector< ::strands_executive_msgs::Task > tasks;
	strands_executive_msgs::SetExecutionStatus runExec;	
	generateTasks(numTasks, tasks);
	for (unsigned int i=0; i<tasks.size(); i++ ) 
	{
		strands_executive_msgs::AddTask taskAdd;
		taskAdd.request.task = tasks[i];
		if (taskAdder.call(taskAdd))
		{
			ROS_INFO("Task ID: %ld", taskAdd.response.task_id);
		}
	}
	runExec.request.status = true;
	if (taskStart.call(runExec)) ROS_INFO("Task execution enabled.");
}


int main(int argc,char* argv[])
{
	ros::init(argc, argv, "infremen");
	n = new ros::NodeHandle();
    //MessageStoreProxy messageStore(*n,"message_store");
	geometry_msgs::PoseStamped dummyPose;	
	//ide = messageStore.insert(dummyPose);

	ros::Subscriber  topoMapSub = n->subscribe("/topological_map", 1000, loadMap);
	ros::Subscriber  currentNodeSub = n->subscribe("/closest_node", 1000, getCurrentNode);
	ros::Subscriber  getCommandSub = n->subscribe("/socialCardReader/commands", 1000, getCommand);
	ros::Subscriber  interfaceSub = n->subscribe("/info_terminal/active_screen", 1000, interacted);
	ros::ServiceServer service = n->advertiseService("create_infoterminal_tasks", generateTasks);

	timer.start();
	timer.reset(waitingInterval);
	interactionTimer.start();
	interactionTimer.reset(waitingInterval);
	//ros::Subscriber  recalculateSub = n->subscribe("/recalculate", 1000, recalculate);
//	taskPub = n->advertise<strands_executive_msgs::Task>("/taskTopic", 1000, recalculate);
	taskAdder = n->serviceClient<strands_executive_msgs::AddTask>("/task_executor/add_task");
	//taskClear = n->serviceClient<std_msgs::Empty>("/task_executor/clear_schedule");
	taskStart = n->serviceClient<strands_executive_msgs::SetExecutionStatus>("/task_executor/set_execution_status");
	int periodicity = 20*(taskLength*numTasks+5);
	int iii = periodicity-5*20;
	while (ros::ok())
	{
		ros::spinOnce();
		usleep(50000);
		if (iii++>periodicity){
			recalculateModels();
//	generateTasks();
			iii=0;
		}
		if (timer.timeOut())timeOut();
	}
	frelementSet.save("latest.fre");
	return 0;
}

