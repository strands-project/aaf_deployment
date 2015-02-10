#include <stdlib.h>
#include "ros/ros.h"
#include <mongodb_store/message_store.h>
#include <boost/foreach.hpp>
#include <strands_navigation_msgs/NavStatistics.h>
#include "CFrelementSet.h"

#define MAX_EDGES 10000

const mongo::BSONObj EMPTY_BSON_OBJ;

using namespace mongodb_store;
using namespace std;

ros::NodeHandle *n;
bool debug = false;

CFrelementSet frelementSet;
char mapName[1000];

/*loads nodes from the map description*/
void loadMap(const strands_navigation_msgs::::ConstPtr& msg)
{
}

/*generates a task - invoked every minute*/
int generateTask()
{
}

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "infremen");
	n = new ros::NodeHandle();
	//retrieveData();
	//loadData(argv[1]);
	//printEdges(argv[2]);
	//debugPrint(argv[2]);
	ros::Subscriber sub_octo = n.subscribe("/topological_map", 1000, loadMap);

	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
	}
	return 0;
}
