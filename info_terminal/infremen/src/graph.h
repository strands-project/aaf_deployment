/*
 * File name: graph.h
 * Date:      2014-08-20 09:33:53 +0200
 * Author:    Miroslav Kulich
 */

#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <string>
#include <vector>
#include <map>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "strands_navigation_msgs/TopologicalMap.h"

#define MAX_GRAPH_SIZE 200
namespace imr {

/// class
class Node;
class Graph {
public:
  typedef std::vector<Node> Nodes;
  typedef std::vector<double> Probabilities;
  typedef std::map<std::string,int> NodeMap;
  typedef std::pair<int,int> Link;
  typedef std::vector<int> Path;
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property,
  boost::property< boost::edge_weight_t, int > > BoostGraph;

  enum {
    EXPLORATION_METHOD,
    EXPLOITATION_METHOD,
    ARTIFICIAL_METHOD,
    PROBENTROPY_METHOD,
  };


  struct PathSolution {
    double benefit;
    Path path;
    std::vector<int> pathTimes;
  };

  Nodes nodes;
  NodeMap nodeMap;

  BoostGraph g;
  int dist[MAX_GRAPH_SIZE][MAX_GRAPH_SIZE];
  int discredisedDist[MAX_GRAPH_SIZE][MAX_GRAPH_SIZE];

  int next[MAX_GRAPH_SIZE][MAX_GRAPH_SIZE];
//   std::vector<double> length;
//   std::vector<double> probability;


  int addNode(const std::string name);
  void addUndirectedLink(int from, int to, int weight);
  void generateMessage(std::string name, strands_navigation_msgs::TopologicalMap &map);
  std::string getNodeName(int id);
  int getNodeId(std::string name);


  void display();
  void loadMap(std::string name);
  void load(const strands_navigation_msgs::TopologicalMapConstPtr &msg);
  void loadFile(std::string fileName); 
  void permuteNodes(int id);
  PathSolution getPath(int start);
  int setPlanningHorizon(int horizon);
  void setMethod(std::string method);
  double getBenefit(double p);
  Graph();
  ~Graph();

  const int timeSlotDurationInMinutes;
  const int interactionSlotDurationInMinutes;
  int planningHorizonInMinutes;
  const int timeSlotDuration;
  const int interactionSlotDuration;
  int planningHorizon;
  std::string methodName;
private:

   Path path;
   std::vector<int> pathTimes;
   PathSolution solution;//
   int pathTime;
   double pathBenefit;//, bestPathBenefit;
   int method;

};

/// class
class Node {
public:
  int id;
  std::string name;
  std::vector<Graph::Link> links;
  std::vector<double> probs; // probabilities for given times (stored somewhere else)
  Node(int id, std::string name);
  void addLink(int id, int weight);
  ~Node();
};

}

#endif

/* end of graph.h */
