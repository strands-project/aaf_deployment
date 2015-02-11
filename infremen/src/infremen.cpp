#include <stdlib.h>
#include "ros/ros.h"
#include <mongodb_store/message_store.h>
#include <boost/foreach.hpp>
#include <strands_navigation_msgs/NavStatistics.h>
#include <strands_navigation_msgs/TopologicalMap.h>
#include <strands_navigation_msgs/TopologicalNode.h>
#include <strands_executive_msgs/AddTask.h>
#include <std_msgs/Empty.h>
#include "CFrelementSet.h"
#include <time.h> 

#define MAX_EDGES 10000

const mongo::BSONObj EMPTY_BSON_OBJ;

using namespace mongodb_store;
using namespace std;

ros::NodeHandle *n;
ros::Publisher  taskPub;
bool debug = false;
ros::ServiceClient taskAdder;

CFrelementSet frelementSet;
string nodeName;
uint32_t 	times[100000];
unsigned char 	state[100000];
float wheel[1000];
int numNodes = 0;
float lastWheel = 0;

void recalculateModels()
{
	numNodes = frelementSet.numFrelements;
	float explorationRatio = 0.5;
	float entropy[1],probability[1];
	uint32_t times[1];
	times[0] = 0;//TODO current time
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
	recalculateModels();
}

/*loads nodes from the map description*/
void getCurrentNode(const std_msgs::String::ConstPtr& msg)
{
	nodeName = msg->name;
	printf("Current %s.\n",nodeName.c_str());
}

void addResult(const char *name,unsigned char state)
{
	uint32_t times[1];
	unsigned char signal[1];
	signal[0] = state;
	frelementSet.add(nodeName.c_str(),times,signal,1);
}


void interacted(const std_msgs::Empty::ConstPtr& msg)
{
	addResult(nodeName.c_str(),1);
}

void timeOut(const std_msgs::Empty::ConstPtr& msg)
{
	addResult(nodeName.c_str(),0);
}

/*recalculates models*/
void recalculate(const std_msgs::Empty::ConstPtr& msg)
{
	recalculateModels();
}

/*generates a task - invoked every minute*/
int generateTasks()
{
 time_t t;
 srand((unsigned int)time(&t));
	for (int i=0;i<2;i++)
	{
		int node = 0;
		if (numNodes > 0){
			float randomNum = (float)rand()/RAND_MAX*lastWheel;
			for (int p = 0;p<numNodes && randomNum > wheel[p];p++) node = p;
		}
		strands_executive_msgs::AddTask taskAdd;
		strands_executive_msgs::Task task;
		task.start_node_id = frelementSet.frelements[node]->id;
		task.action = "wait_action";
		task.start_after = ros::Time::now()+ros::Duration(180*i);
		task.end_before  = ros::Time::now()+ros::Duration(180*(i+1)-1);
		task.max_duration = ros::Duration(10);
		taskAdd.request.task = task;
		if (taskAdder.call(taskAdd))
		{
			ROS_INFO("Task ID: %ld", taskAdd.response.task_id);
		}
		printf("Time slot: %i-%i %i %s \n",180*i,180*(i+1)-1,numNodes,frelementSet.frelements[node]->id);
	}
}


int main(int argc,char* argv[])
{
	ros::init(argc, argv, "infremen");
	n = new ros::NodeHandle();
	ros::Subscriber  topoMapSub = n->subscribe("/topological_map", 1000, loadMap);
	ros::Subscriber  currentNodeSub = n->subscribe("/current_node", 1000, getCurrentNode);
	//ros::Subscriber  interfaceSub = n->subscribe("/info_terminal", 1000, interacted);
	//ros::Subscriber  recalculateSub = n->subscribe("/recalculate", 1000, recalculate);
//	taskPub = n->advertise<strands_executive_msgs::Task>("/taskTopic", 1000, recalculate);
	taskAdder = n->serviceClient<strands_executive_msgs::AddTask>("/task_executor/add_task");
	int periodicity = 20*60*7;
	int iii = periodicity-5*20;
	while (ros::ok())
	{
		ros::spinOnce();
		usleep(50000);
		if (iii++>periodicity){
			 generateTasks();
			 iii=0;
		}
	}
	return 0;
}
