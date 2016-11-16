/*
 * File name: graph.cc
 * Date:      2015-08-21
 * Author:    Miroslav Kulich
 */


#include <string>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "ros/ros.h"
#include "CGraph.h"

using namespace imr;


/// =============================
/// = Node ======================
/// =============================

/// - constructor --------------------------------------------------------------
Node::Node(int id, std::string name) :
  id(id),
  name(name) {
	  infoTerminal = false;
  };

/// - destructor --------------------------------------------------------------
Node::~Node() {
}

/// - public method --------------------------------------------------------------
void Node::addLink(int id, int weight) {
  links.push_back(Graph::Link(id,weight));
}



/// =============================
/// = Graph =====================
/// =============================


//Graph() timeSlotDuration(5*60), interactionSlotDuration(3*60)  {};


/// - constructor --------------------------------------------------------------
Graph::Graph():
timeSlotDurationInMinutes(5),
interactionSlotDurationInMinutes(3),
planningHorizonInMinutes(30),
timeSlotDuration(timeSlotDurationInMinutes*60),
interactionSlotDuration(interactionSlotDurationInMinutes*60),
planningHorizon(planningHorizonInMinutes*60) {
}

/// - destructor --------------------------------------------------------------
Graph::~Graph() {
}

/// - public method --------------------------------------------------------------
int Graph::setPlanningHorizon(int horizon) {
  planningHorizonInMinutes = horizon;
  planningHorizon = planningHorizonInMinutes*60;
}


/// - public method --------------------------------------------------------------
int Graph::addNode(std::string name) {
  static int num = 0;
  int result;
  NodeMap::iterator it = nodeMap.find(name);
  if ( it == nodeMap.end() ) {
    nodes.push_back(Node(num,name));
    result = num;
    add_vertex(g);
    nodeMap[name] = num++;
  } else {
    result = it->second;
  }
  return result;
}

/// - public method --------------------------------------------------------------
void Graph::addUndirectedLink(int from, int to, int weight) {
    nodes[to].addLink(from, weight);
    nodes[from].addLink(to, weight);
    add_edge(from,to,weight,g);
}


/// - public method --------------------------------------------------------------
void Graph::display() {
  for(auto node:nodes) {
    std::cout << node.name << " (" << node.id << ")"  << std::endl;
    for(auto link:node.links) {
      std::cout << "   " << link.first << " " << nodes[link.first].name;
      std::cout << " w: " << link.second;
//       std::cout << " pp: " << link.second * nodes[link.first].getProbability() << "  " << nodes[link.first].getProbability();
      std::cout << std::endl;
    }
  }

}

void Graph::loadFile(std::string fileName) 
{
std::ifstream fin;
  std::string type, node, from, to;
  double weight;
  int idFrom, idTo;
  fin.open(fileName.c_str());
  if (!fin.good()) {
    throw std::runtime_error("File " + fileName + " cann't be opened.");
  }

  fin >> type;
  while (!fin.eof())
  {
    if (type.compare("node")==0) {
      fin >> node;
      addNode(node);
    } else if (type.compare("link")==0) {
      fin >> from;
      fin >> to;
      fin >> weight;
      idFrom = addNode(from);
      idTo = addNode(to);
      std::cout << "adding " << idFrom <<  " " << idTo << " " << weight <<  std::endl;
      addUndirectedLink(idFrom, idTo, weight);
    }
    fin >> type;
  }
  
  display();
}

void Graph::load(const strands_navigation_msgs::TopologicalMapConstPtr &msg,CFrelementSet *set) 
{
	std::ifstream fin;
	std::string type, node, from, to;
	int weight;
	int idFrom, idTo;

	nodes.clear();
	nodeMap.clear();
	float x[msg->nodes.size()];
	float y[msg->nodes.size()];
	for(auto n:msg->nodes) {
		int a = addNode(n.name);
		ROS_INFO("  Node %i: %s %f %f",a, n.name.c_str(),n.pose.position.x,n.pose.position.y);
		x[a] = n.pose.position.x;
		y[a] = n.pose.position.y;
	}

	for(auto n:msg->nodes) {
		idFrom = addNode(n.name);
		for(auto e:n.edges) {
			idTo = addNode(e.node);
			if (idFrom > idTo) {
				addUndirectedLink(idFrom, idTo, e.top_vel);
			}
			ROS_INFO("    -> %s", e.node.c_str());
		}
	}
	display();


	std::vector <int> d(100, (std::numeric_limits <int>::max)());
	boost::johnson_all_pairs_shortest_paths(g, dist, boost::distance_map(&d[0]));

	int minDist;
	int dd;
	for (unsigned int i = 0; i < nodes.size(); ++i) {
		if (set->find(nodes[i].name.c_str()) >=0) nodes[i].infoTerminal = true; else nodes[i].infoTerminal = false;
		for (unsigned int j = 0; j < nodes.size(); ++j) {
			if (i!=j) {
				minDist = std::numeric_limits <int>::max();
				for(auto link:nodes[i].links) {
					dd = link.second + dist[link.first][j];
					if (dd < minDist) {
						minDist = dd;
						next[i][j] = link.first;
					}
				}
			} else {
				next[i][j] = i;
			}
		}
	}

	int rest;
	for (unsigned int i = 0; i < nodes.size(); ++i) {
		for (unsigned int j = 0; j < nodes.size(); ++j) {
			if (i==j) {
				discredisedDist[i][j] = 2*planningHorizon;
			} else {
				rest = timeSlotDuration - ((dist[i][j]+interactionSlotDuration) % timeSlotDuration);
				discredisedDist[i][j] = dist[i][j] + interactionSlotDuration + rest;
			}
		}
	}

	std::cout << "Distances" << std::endl << "       ";
	for (unsigned int k = 0; k < nodes.size(); ++k) {
		std::cout << std::setw(8) << k;
	}
	std::cout << std::endl;

	for (unsigned int i = 0; i < nodes.size(); ++i) {
		std::cout << std::setw(3) << i << " -> ";
		for (unsigned int j = 0; j < nodes.size(); ++j) {
			if (dist[i][j] == (std::numeric_limits<int>::max)())
				std::cout << std::setw(8) << "inf";
			else
				std::cout << std::setw(8) << dist[i][j];
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;


	std::cout << "Discretized distances" << std::endl << "       ";
	for (unsigned int k = 0; k < nodes.size(); ++k) {
		std::cout << std::setw(8) << k;
	}
	std::cout << std::endl;

	for (unsigned int i = 0; i < nodes.size(); ++i) {
		std::cout << std::setw(3) << i << " -> ";
		for (unsigned int j = 0; j < nodes.size(); ++j) {
			if (discredisedDist[i][j] == (std::numeric_limits<int>::max)())
				std::cout << std::setw(8) << "inf";
			else
				std::cout << std::setw(8) << discredisedDist[i][j];
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;




	std::cout << "Nexts"<< std::endl << "       ";
	for (unsigned int k = 0; k < nodes.size(); ++k) {
		std::cout << std::setw(5) << k;
	}
	std::cout << std::endl;

	for (unsigned int i = 0; i < nodes.size(); ++i) {
		std::cout << std::setw(3) << i << " -> ";
		for (unsigned int j = 0; j < nodes.size(); ++j) {
			if (dist[i][j] == (std::numeric_limits<int>::max)())
				std::cout << std::setw(5) << "inf";
			else
				std::cout << std::setw(5) << next[i][j];
		}
		std::cout << std::endl;
	}



	// print boost graph structure
	std::ofstream fout("fig.dot");
	fout << "graph A {\n"
		<< "  rankdir=LR\n"
		<< "size=\"5,3\"\n"
		<< "ratio=\"fill\"\n"
		<< "edge[style=\"bold\" fontsize=21, fontcolor=\"blue\",fontname=\"Times-Roman bold\"]\n"
		<< "node[shape=\"circle\" , width=1.6, fontsize=21, fillcolor=\"yellow\", style=filled]\n";

	boost::graph_traits < BoostGraph >::edge_iterator ei, ei_end;
	for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei)
		fout << nodes[boost::source(*ei, g)].name << " -- " << nodes[boost::target(*ei, g)].name
			//    fout << boost::source(*ei, g) << " -> " << boost::target(*ei, g)
			<< "[label=" << boost::get(boost::edge_weight, g)[*ei] << "]\n";

	fout << "}\n";
	fout.close();
	system("dot -Tpdf  -n  fig.dot > fig.pdf");

}
	/// - public method --------------------------------------------------------------
void Graph::load(const strands_navigation_msgs::TopologicalMapConstPtr &msg) {
  std::ifstream fin;
  std::string type, node, from, to;
  int weight;
  int idFrom, idTo;

  nodes.clear();
  nodeMap.clear();
  for(auto n:msg->nodes) {
    addNode(n.name);
  }

  for(auto n:msg->nodes) {
    ROS_INFO("  Node: %s", n.name.c_str());
    idFrom = addNode(n.name);
    for(auto e:n.edges) {
      idTo = addNode(e.node);
      if (idFrom > idTo) {
        addUndirectedLink(idFrom, idTo, e.top_vel);
      }
      ROS_INFO("    -> %s", e.node.c_str());
    }
  }
  display();



  std::vector <int> d(100, (std::numeric_limits <int>::max)());
  boost::johnson_all_pairs_shortest_paths(g, dist, boost::distance_map(&d[0]));

  int minDist;
  int dd;
  for (unsigned int i = 0; i < nodes.size(); ++i) {
    for (unsigned int j = 0; j < nodes.size(); ++j) {
      if (i!=j) {
        minDist = std::numeric_limits <int>::max();
        for(auto link:nodes[i].links) {
          dd = link.second + dist[link.first][j];
          if (dd < minDist) {
            minDist = dd;
            next[i][j] = link.first;
          }
        }
      } else {
        next[i][j] = i;
      }
    }
  }

  int rest;
  for (unsigned int i = 0; i < nodes.size(); ++i) {
    for (unsigned int j = 0; j < nodes.size(); ++j) {
      if (i==j) {
        discredisedDist[i][j] = 2*planningHorizon;
      } else {
        rest = timeSlotDuration - ((dist[i][j]+interactionSlotDuration) % timeSlotDuration);
        discredisedDist[i][j] = dist[i][j] + interactionSlotDuration + rest;
      }
    }
  }

  std::cout << "Distances" << std::endl << "       ";
  for (unsigned int k = 0; k < nodes.size(); ++k) {
    std::cout << std::setw(8) << k;
  }
  std::cout << std::endl;

  for (unsigned int i = 0; i < nodes.size(); ++i) {
    std::cout << std::setw(3) << i << " -> ";
    for (unsigned int j = 0; j < nodes.size(); ++j) {
      if (dist[i][j] == (std::numeric_limits<int>::max)())
        std::cout << std::setw(8) << "inf";
      else
        std::cout << std::setw(8) << dist[i][j];
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;


  std::cout << "Discretized distances" << std::endl << "       ";
  for (unsigned int k = 0; k < nodes.size(); ++k) {
    std::cout << std::setw(8) << k;
  }
  std::cout << std::endl;

  for (unsigned int i = 0; i < nodes.size(); ++i) {
    std::cout << std::setw(3) << i << " -> ";
    for (unsigned int j = 0; j < nodes.size(); ++j) {
      if (discredisedDist[i][j] == (std::numeric_limits<int>::max)())
        std::cout << std::setw(8) << "inf";
      else
        std::cout << std::setw(8) << discredisedDist[i][j];
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;




  std::cout << "Nexts"<< std::endl << "       ";
  for (unsigned int k = 0; k < nodes.size(); ++k) {
    std::cout << std::setw(5) << k;
  }
  std::cout << std::endl;

  for (unsigned int i = 0; i < nodes.size(); ++i) {
    std::cout << std::setw(3) << i << " -> ";
    for (unsigned int j = 0; j < nodes.size(); ++j) {
      if (dist[i][j] == (std::numeric_limits<int>::max)())
        std::cout << std::setw(5) << "inf";
      else
        std::cout << std::setw(5) << next[i][j];
    }
    std::cout << std::endl;
  }



  // print boost graph structure
  std::ofstream fout("fig.dot");
  fout << "graph A {\n"
  << "  rankdir=LR\n"
  << "size=\"5,3\"\n"
  << "ratio=\"fill\"\n"
  << "edge[style=\"bold\" fontsize=21, fontcolor=\"blue\",fontname=\"Times-Roman bold\"]\n"
  << "node[shape=\"circle\" , width=1.6, fontsize=21, fillcolor=\"yellow\", style=filled]\n";

  boost::graph_traits < BoostGraph >::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei)
    fout << nodes[boost::source(*ei, g)].name << " -- " << nodes[boost::target(*ei, g)].name
//    fout << boost::source(*ei, g) << " -> " << boost::target(*ei, g)
    << "[label=" << boost::get(boost::edge_weight, g)[*ei] << "]\n";

  fout << "}\n";
  fout.close();
  system("dot -Tpdf  -n  fig.dot > fig.pdf");
}

void Graph::generateMessage(std::string name, strands_navigation_msgs::TopologicalMap &map) {
  map.name = name;
  map.map = name;
  map.pointset = name;
  map.last_updated = "0";
  int edgeId = 1;
  for(auto n:nodes) {
    strands_navigation_msgs::TopologicalNode node;
    node.name = n.name;
    node.map = map.map;
    node.pointset = map.map;
    //     geometry_msgs/Pose pose
    //     float64 yaw_goal_tolerance  #The tolerance in radians for the waypoint position
    //     float64 xy_goal_tolerance   #The tolerance in meters for the waypoint position
    //    Vertex[] verts
    //    Edge[] edges
    ///    string localise_by_topic  #The configuration for localisation by topic
    for(auto l:n.links) {
      strands_navigation_msgs::Edge edge;
      edge.edge_id = edgeId++;
      edge.node = nodes[l.first].name;
//       string action
//       float64 top_vel
//       string map_2dd
//       float64 inflation_radius
//       string recovery_behaviours_config
//       l.second
      node.edges.push_back(edge);
    }
    map.nodes.push_back(node);
  }
}

/// - public method --------------------------------------------------------------
std::string Graph::getNodeName(int id) {
  return nodes[id].name;
}

/// - public method --------------------------------------------------------------
int Graph::getNodeId(std::string name) {
  return nodeMap[name];
}



int perNum;

double Graph:: getBenefit(double p) {
  double result;
  switch(method) {
    case EXPLOITATION_METHOD:
      result = p;
      break;
    case EXPLORATION_METHOD:
      result = -(p*log2(p) + (1-p)*log2(1-p));
      break;
    case PROBENTROPY_METHOD:
      result = 0.5*p - (1-0.5)*(p*log2(p) + (1-p)*log2(1-p));
      break;
    case ARTIFICIAL_METHOD:
      result = 2.0/(1-p) + 1./(1+100*(0.5-p)*(0.5-p));
      break;
  }
  return result;
}


void Graph::permuteNodes(int id) {
	if (pathBenefit > solution.benefit) {
		solution.benefit = pathBenefit;
		solution.pathTimes.resize(pathTimes.size());
		solution.path.resize(path.size());
		std::copy(pathTimes.begin(),pathTimes.end(),solution.pathTimes.begin());
		std::copy(path.begin(),path.end(),solution.path.begin());

		//     for(auto n: path) {
		//       std::cout << nodes[n].name << " -> ";
		//     }
		//     std::cout << "   [ " << pathBenefit  << " " <<  pathTime << " ]";
		//     std::cout << std::endl;
	}
	int *dd =  discredisedDist[id];
	for(auto n: nodes) {
		if (n.infoTerminal){
			pathTime += dd[n.id];
			if (pathTime <= planningHorizon) {
				double p = n.probs[pathTime/timeSlotDuration - 1] + rand()/ (100000.0*RAND_MAX); //TODO: wouldn't be better to use shuffle?
				double benefit = getBenefit(p);
				//       double benefit = 200/(1-p) + 1./(1+100*(0.5-p)*(0.5-p));
				//       double benefit = 0.5*p - (1-0.5)*(p*log2(p) + (1-p)*log2(1-p));
				//      double benefit = -(p*log2(p) + (1-p)*log2(1-p));
				//        std::cout << (pathTime/60 - interactionSlotDurationInMinutes) << " " << benefit << " " << n.probs.size()  << std::endl;
				//        std::cout << " " << n.probs[(int) benefit]<< std::endl;
				pathBenefit+=benefit;
				path.push_back(n.id);
				pathTimes.push_back(pathTime);
				permuteNodes(n.id);
				path.pop_back();
				pathTimes.pop_back();
				pathBenefit-=benefit;
			}
			pathTime -= dd[n.id];
		}
	}
}




/// - public method --------------------------------------------------------------
Graph::PathSolution Graph::getPath(int start) {
  path.clear();
//   pathTimes.clear();
//      int start = nodeMap["Corridor"];
//   std::string START_NODE = "Kindergarten";
//  std::string START_NODE = "WP8";
//   int start = nodeMap[START_NODE];
//  path.push_back(start);
//   pathTimes.push_back(0);


  pathTime = 0;
  pathBenefit = 0;
  pathTimes.clear();
//   best = std::numeric_limits<double>::max();
  solution.benefit = -1;
  solution.path.clear();
  solution.pathTimes.clear();


  boost::posix_time::ptime startT(boost::posix_time::microsec_clock::local_time());
  permuteNodes(start);
  boost::posix_time::ptime finishT(boost::posix_time::microsec_clock::local_time());

  int i=0;
//   ROS_INFO("FOUND SOLUTION:");
//   std::cout << nodes[start].name << " -> ";
//   for(auto n: solution.path) {
//     std::cout << nodes[n].name << "(" << solution.pathTimes[i++] << ") -> ";
//   }
//   std::cout << "   [ " << solution.benefit << " ]";
//   std::cout << std::endl;
  std::cout << "Time: " << (finishT - startT).total_microseconds()/1000.0 << std::endl;

  /*
  result.path.resize(bestPath.size());
  std::copy(bestPath.begin(), bestPath.end(), result.path.begin());
  result.pathTimes.resize(bestPath.size());
  std::copy(bestPathTimes.begin(), bestPathTimes.end(), result.pathTimes.begin());
  result.expectedTime = best;
*/


//  std::cout << "Time spent in estimation: " << estTime/1000000.0 << "s" << std::endl;
  return solution;
}

void Graph::setMethod(std::string m) {
  std::map<std::string, int> methodType;
  methodType["exploitation"] = EXPLOITATION_METHOD;
  methodType["exploration"] = EXPLORATION_METHOD;
  methodType["probentropy"] = PROBENTROPY_METHOD;
  methodType["artificial"] = ARTIFICIAL_METHOD;

  methodName = m;
  method = methodType[m];
}
