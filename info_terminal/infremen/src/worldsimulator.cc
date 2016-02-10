/*
 * File name: worldsimulator.cc
 * Date:      2015-08-28 12:05:48 +0200
 * Author:    Miroslav Kulich
 */

#include "ros/ros.h"
#include "worldsimulator.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <chrono>
#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>

using namespace imr;

const unsigned int secondsInWeek = 7*24*3600;
const unsigned int secondsInDay = 24*3600;

/// - constructor --------------------------------------------------------------
CWorldSimulator::CWorldSimulator(Graph &map,CFrelementSet* fremeni) :
map(map),
time(0),
dayTime(0),
finished(false),
detectionCount (0),
allDetectionsCount(0),
order (4) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  expDistGenerator = std::default_random_engine(seed);
  fremen = fremeni;
}



/*
/// - constructor --------------------------------------------------------------
CWorldSimulator::CWorldSimulator(std::string modelName, Graph &map) :
map(map),
time(0),
dayTime(0),
// isHumanDetected(false),
finished(false),
detectionCount (0),
allDetectionsCount(0),
order(4) {
  loadModel(modelName);
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  expDistGenerator = std::default_random_engine(seed);
}
*/




/// - destructor --------------------------------------------------------------
CWorldSimulator::~CWorldSimulator() {
  delete[] allTimes;
}



/// - public --------------------------------------------------------------
void CWorldSimulator::loadModel(std::string modelName) {
  groundTruth.load(modelName);
  detectionCount = 0;
  int delta = 300;
  int i=0;
  allTimesCount = groundTruth.finalTime()/delta;
  allTimes = new uint32_t[allTimesCount];
  for(unsigned int t=120;t<=groundTruth.finalTime();i++,t+=delta) {
    allTimes[i] = t;
  }
}

/// - public --------------------------------------------------------------
void CWorldSimulator::setMethod(std::string m, std::string sm, double iA, double iB) {
  std::map<std::string, int> methodType;
  methodType["exploitation"] = EXPLOITATION_METHOD;
  methodType["exploration"] = EXPLORATION_METHOD;
  methodType["montecarlo"] = MONTECARLO_METHOD;
  methodType["artificial"] = ARTIFICIAL_METHOD;
  methodType["probentropy"] = PROBENTROPY_METHOD;
  methodType["randomwalk"] = RANDOMWALK_METHOD;
  methodType["oraculum"] = ORACULUM_METHOD;
  methodType["superoraculum"] = SUPERORACULUM_METHOD;
  methodType["mcexploration"] = MC_EXPLORATION_METHOD;
  methodType["mcartificial"] = MC_ARTIFICIAL_METHOD;
  methodType["mcexploitation"] = MC_EXPLOITATION_METHOD;
  methodType["mcprobentropy"] = MC_PROBENTROPY_METHOD;
  methodType["horizon"] = HORIZON_METHOD;

  methodName = m;
  method = methodType[m];
  ROS_INFO("STOD (%lf) (%lf) (%s)",iA, iB, sm.c_str());
  A = iA;
  B = iB;
  map.setMethod(sm);
}


/// - public --------------------------------------------------------------
void CWorldSimulator::goTo(std::string node) {

if (false) {
  topological_navigation::GotoNodeGoal goal;
  goal.target = node;
  navigator->sendGoal(goal);
  if (navigator->waitForResult(ros::Duration(10.0))) {
//     topological_navigation::GotoNodeResultConstPtr result = (topological_navigation::GotoNodeResultConstPtr) (navigator->getResult());
//     ROS_INFO("Action finished: %s",result->success ? "true" : "false" );
//     ROS_INFO("Message: %s",result->message.c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }
} else {
  setCurrentNode(node);
}

}

/*
void CWorldSimulator::rememberMeasurement(std::string id, uint32_t time, unsigned char state) {
  SMeasurementData data;
  data.id = id;
  data.time = time;
  data.state = state;
  measurements.push_back(data);
}

void CWorldSimulator::addRememberedMeasurements() {
  for(auto m: measurements) {
    addMeasurementToFremen(m.id,m.time,m.state);
  }
  ROS_INFO("%i measurements added.",(int)measurements.size());
  measurements.clear();
}

void CWorldSimulator::addMeasurementToFremen(std::string id, uint32_t time, unsigned char state) {
  uint32_t times[1];
  unsigned char states[1];
  times[0] = time;
  states[0] = state;
  int result = fremen->add(id.c_str(),times,states,1);
//   ROS_INFO("FREMEN: %d elements added    time: %d state:  %s value: %d", result,times[0], id.c_str(), (int) states[0]);
}
*/

void CWorldSimulator::initFremen()
{

  uint32_t times[0];
  unsigned char states[0];
  for(auto n: map.nodes) {
//	  std::cout <<"Init    "<< n.name << " " << fremen->add(n.name.c_str(),times,states,0) << std::endl;
  }
}

void CWorldSimulator::getProbablitesFromFremen(std::vector<std::string> &ids, std::vector<uint32_t> &times, std::vector<float> &probs) {
  uint32_t tms[1];
  float pps[1];

  int i=0;
  probs.clear();
  for(auto id: ids) {
    tms[0] = times[i];
    fremen->estimate(id.c_str(), tms, pps, 1, order);
    probs.push_back(pps[0]);
    i++;
  }
//   std::copy(times.begin(), times.end(), tms);
//   fremen->estimate(names,tms,pps,ids.size(), 2);
//   probs.resize(ids.size());
//   std::copy(pps, pps+ids.size(), probs);


}

void CWorldSimulator::getProbablitesFromFremen(std::vector<uint32_t> &times) {
  int n = times.size();
  uint32_t tms[n];
  float pps[n];

  int i=0;
  for(auto t: times) {
    tms[i++] = t;
  }

  for(auto &node: map.nodes) {
    fremen->estimate(node.name.c_str(), tms, pps, n, order);
    node.probs.clear();
//      std::cout << "Probs for " << node.name << ": ";
    for(int k=0;k<n;k++) {
//        std::cout << pps[k] << " ";
      node.probs.push_back(pps[k]);
    }
//      std::cout << std::endl;
  }
}




//  int estimate(const char *name,uint32_t times[],float probs[],int length,int order);

// http://stackoverflow.com/questions/2140787/select-random-k-elements-from-a-list-whose-elements-have-weights
int CWorldSimulator::MonteCarloSelection(std::vector<float> &probs) {
  int result;
  double min = std::numeric_limits<double>::max();
  double number;
  float sum = 0;
  for(auto p:probs) { sum+=p; }
//   double sum = std::accumulate(probs.begin(), probs.end(), 0);


  int i =0;
//    std::cout << "PROBS: ";
  for(auto p: probs) {
    if (p==0) {
      number = 0;
    } else {
      std::exponential_distribution<double> distribution(p/sum);
      number = distribution(expDistGenerator);
    }
//     std::cout << p << " (" << number << ") ";
    if (number < min) {
      min = number;
      result = i;
    }
    i++;
  }
//    std::cout << std::endl;
//    ROS_INFO("Chosen %d %lf", result, min);
//    ROS_INFO("Chosen %d %lf", result, probs[result]);
  return result;
}

int CWorldSimulator::MonteCarloExploitationSelection(std::vector<float> &probs) {
  float sum = 0;
  for(auto p:probs) { sum+=p; }

  float r = (float) std::rand();
  r =  sum* r/RAND_MAX;

  int result = 0;
  float acc = probs[0];


  while(acc < r) {
    result++;
    acc += probs[result];
  }
  return result;
}


int CWorldSimulator::MonteCarloExplorationSelection(std::vector<float> &probs) {
  float sum = 0;
  float p;
  std::vector<double> f;

  for(uint i=0;i<probs.size();i++) {
    p = probs[i];
    f.push_back(-(p*log2(p) + (1-p)*log2(1-p)));
//     ROS_INFO("    VAL: %d %lf %lf       %s",i, p,-(p*log2(p) + (1-p)*log2(1-p)), map.getNodeName(i).c_str());
  }

  for(auto p:f) { sum+=p; }

  float r = (float) std::rand();
  r =  sum* r/RAND_MAX;
// ROS_INFO("R: %f",r);
  int result = 0;
  float acc = f[0];


  while(acc < r) {
    result++;
    acc += f[result];
  }
  return result;
}


int CWorldSimulator::MonteCarloProbEntropySelection(std::vector<float> &probs) {
  float sum = 0;
  float p;
  std::vector<double> f;

  for(uint i=0;i<probs.size();i++) {
    p = probs[i];
    f.push_back(p-(p*log2(p) + (1-p)*log2(1-p)));
    //     ROS_INFO("    VAL: %d %lf %lf       %s",i, p,-(p*log2(p) + (1-p)*log2(1-p)), map.getNodeName(i).c_str());
  }

  for(auto p:f) { sum+=p; }

  float r = (float) std::rand();
  r =  sum* r/RAND_MAX;
  // ROS_INFO("R: %f",r);
  int result = 0;
  float acc = f[0];


  while(acc < r) {
    result++;
    acc += f[result];
  }
  return result;
}


int CWorldSimulator::MonteCarloArtificialSelection(double A, double B, std::vector<float> &probs) {
  float sum = 0;
  std::vector<double> f;
  float p,pp;

  for(uint i=0;i<probs.size();i++) {
    p = probs[i];
    pp = A/(1-p) + 1./(1+B*(0.5-p)*(0.5-p));
//     ROS_INFO("PP %d %lf %lf", i, p, pp);
    f.push_back(pp);
  }

  for(auto p:f) { sum+=p; }

  float r = (float) std::rand();
  r =  sum* r/RAND_MAX;

  int result = 0;
  float acc = f[0];


  while(acc < r) {
    result++;
    acc += f[result];
  }
  return result;
}



int CWorldSimulator::RandomWalkSelection(std::vector<float> &probs) {
  return std::rand() % probs.size();
}

int CWorldSimulator::ExploitationSelection(std::vector<float> &probs) {
  double max = std::numeric_limits<double>::lowest();
  int maxIndex = -1;
  for(uint i=0;i<probs.size();i++) {
    if (probs[i]>max) {
      max = probs[i];
      maxIndex = i;
    }
  }
  return maxIndex;
//   return std::distance(probs.begin(),std::max_element(probs.begin(), probs.end()));
}

int CWorldSimulator::ExplorationSelection(std::vector<float> &probs) {
  double max = std::numeric_limits<double>::lowest();
  int maxIndex = -1;
  double f,p;
//  fremen->print(true);
  for(uint i=0;i<probs.size();i++) {
    p = probs[i];
    f = - (p*log2(p) + (1-p)*log2(1-p));
//     ROS_INFO("    VAL: %d %lf %lf       %s",i, p,f, map.getNodeName(i).c_str());
    if (f>max) {
      max = f;
      maxIndex = i;
    }
  }
//   ROS_INFO("result %d %d",maxIndex,probs.size());
  return maxIndex;
  //   return std::distance(probs.begin(),std::max_element(probs.begin(), probs.end()));
}



int CWorldSimulator::ProbEntropySelection(double A, std::vector<float> &probs) {
  double max = std::numeric_limits<double>::lowest();
  int maxIndex = -1;
  for(uint i=0;i<probs.size();i++) {
    double p = probs[i];
    double f = A*p - (1-A)*(p*log2(p) + (1-p)*log2(1-p));
//    ROS_INFO("A %lf %lf %lf %lf",A, p, f, -(1-A)*(p*log2(p) + (1-p)*log2(1-p)));
    if (f>max) {
      max = f;
      maxIndex = i;
    }
//     probs[i] = f;
  }
//   int maxIndex = std::distance(probs.begin(),std::max_element(probs.begin(), probs.end()));
  return maxIndex;
}


int CWorldSimulator::ArtificialFuncSelection(double A, double B, std::vector<float> &probs) {
  double max = std::numeric_limits<double>::lowest();
  int maxIndex = -1;
  for(uint i=0;i<probs.size();i++) {
    double p = probs[i];
    double f = A/(1-p) + 1./(1+B*(0.5-p)*(0.5-p));
//     ROS_INFO("pp %lf %lf %s",p,f,ids[i].c_str());
    if (f>max) {
      max = f;
      maxIndex = i;
    }
  }
//   ROS_INFO("ArtificialFuncSelection %d %d %s",probs.size(),maxIndex, ids[maxIndex].c_str());
//   int maxIndex = std::distance(probs.begin(),std::max_element(probs.begin(), probs.end()));
  return maxIndex;
}


int CWorldSimulator::OraculumSelection(unsigned int time) {
//   int result = groundTruth.humanAt(time);
//   if (result < 0 || result >= map.nodes.size()) { result = 0; }
//   return result;
  return -1;
}

int CWorldSimulator::SuperOraculumSelection(unsigned int time) {
//   int result = groundTruth.humanAt(time);
//   if (result < 0 || result >= map.nodes.size()) { result = 0; }
//   return result;
  return -1;
}

int CWorldSimulator::HorizonSelection(unsigned int time) {
  imr::Graph::PathSolution ps = map.getPath(currentNodeId);
  int delta = ps.pathTimes[0];
  return ps.path[0];
}


std::string CWorldSimulator::nextGoal(uint32_t time, uint32_t &delta) 
{
	std::vector<int> ids;
	std::vector<float> probs;
	int maxIndex;

	std::vector<uint32_t> times;
	static  std::vector<uint32_t> deltas = { 2, 7, 12, 17, 22, 27, 32, 37, 42, 47 }; //TODO: initialize based on planning horizon

	for(auto d: deltas) {
		times.push_back(time+d*60);
	}

	cout << currentNode << "Node" << currentNodeId << endl;
	getProbablitesFromFremen(times);
	if (method == HORIZON_METHOD) { // horizon does not need shuffeling and current node removal (it solves it internally)
		maxIndex = HorizonSelection(time);
		delta = map.discredisedDist[currentNodeId][maxIndex];
		return map.nodes[maxIndex].name;
	} else {
		// put neighbours in a random order to prevent chosing the first one in case of ties
		std::vector<int> randIndex;
		for (auto n: map.nodes) {
			if (n.id!=currentNodeId) {
				randIndex.push_back(n.id);
			}
		}
		std::random_shuffle (randIndex.begin(),randIndex.end());

		for(auto i: randIndex) {
			//    ROS_INFO ("Node %s %d %d %f", map.nodes[i].name.c_str(), map.discredisedDist[currentNodeId][i], map.discredisedDist[currentNodeId][i]/map.timeSlotDuration-1, map.nodes[i].probs[map.discredisedDist[currentNodeId][i]/map.timeSlotDuration-1]);

			probs.push_back(map.nodes[i].probs[map.discredisedDist[currentNodeId][i]/map.timeSlotDuration-1]);
			ids.push_back(i);
		}


		/*
		   std::vector<int> randIndex;
		   for (int i=0; i<map.nodes[currentNodeId].links.size(); ++i) {
		   randIndex.push_back(i);
		   }
		   std::random_shuffle (randIndex.begin(),randIndex.end());


		   for(auto i: randIndex) {
		   nodeName = map.getNodeName(map.nodes[currentNodeId].links[i].first);
		   ids.push_back(nodeName);
		   times.push_back(time);
		   }
		//   ids.push_back(currentNode); //TODO:should I stay on the spot?
		//   times.push_back(time);


		getProbablitesFromFremen(ids,times,probs);
		 */
		switch (method) {
			case EXPLOITATION_METHOD:
				maxIndex = ExploitationSelection(probs);
				break;
			case MONTECARLO_METHOD:
				maxIndex = MonteCarloSelection(probs);
				break;
			case MC_EXPLOITATION_METHOD:
				maxIndex = MonteCarloExploitationSelection(probs);
				break;
			case MC_EXPLORATION_METHOD:
				maxIndex = MonteCarloExplorationSelection(probs);
				break;
			case MC_PROBENTROPY_METHOD:
				maxIndex = MonteCarloProbEntropySelection(probs);
				break;
			case MC_ARTIFICIAL_METHOD:
				maxIndex = MonteCarloArtificialSelection(A,B,probs);
				break;
			case ARTIFICIAL_METHOD:
				maxIndex =  ArtificialFuncSelection(A, B, probs);
				break;
			case EXPLORATION_METHOD:
				maxIndex =  ExplorationSelection(probs);
				break;
			case PROBENTROPY_METHOD:
				maxIndex = ProbEntropySelection(A, probs);
				break;
			case RANDOMWALK_METHOD:
				maxIndex = RandomWalkSelection(probs);
				break;
			case ORACULUM_METHOD:
				maxIndex = OraculumSelection(time);
				break;
			case SUPERORACULUM_METHOD:
				maxIndex = SuperOraculumSelection(time);
				break;
		}

		//   return ids[maxIndex];
		delta = map.discredisedDist[currentNodeId][ids[maxIndex]];
		return map.nodes[ids[maxIndex]].name;
	}
}

bool CWorldSimulator::isInteractionDetected(int id, uint32_t time) {
  return groundTruth.isHumanAt(id,time);
}

/*Tom TODO*/
void CWorldSimulator::step() 
{
	uint32_t delta = 0;
	/*
	   std::vector<uint32_t> times;
	   std::vector<uint32_t> deltas = { 2, 7, 12, 17, 22, 27, 32, 37, 42, 47 }; //TODO: initialize based on planning horizon
	   for(auto d: deltas) {
	   times.push_back(time+d*60);
	   }
	   getProbablitesFromFremen(times);
	   imr::Graph::PathSolution ps = map.getPath(currentNodeId);
	   int delta = ps.pathTimes[0];
	 */
	std::string goal = nextGoal(time, delta);

	//   ROS_INFO("Next goal is %s at %d",goal.c_str(), time + delta);
	/*  if (time+delta <=  groundTruth.finalTime()) {
	    uint32_t interactionTime  = time + delta - map.interactionSlotDuration;
	    bool interactionDetected = isInteractionDetected(map.getNodeId(goal),interactionTime);
	    if (interactionDetected) {
	    detectionCount++;
	    }
	    rememberMeasurement(goal, interactionTime, interactionDetected);
	    goTo(goal);
	    }
	    time += delta;
	    dayTime +=delta;
	    finished = time > groundTruth.finalTime();
	    if (dayTime >=secondsInDay || finished) {
	    addRememberedMeasurements();
	    SReportData record;
	    record.time = time;
	    record.detectionCount = detectionCount;
	    record.allDetectionsCount = allDetectionsCount;
	    evaluate(record.evaluation);
	    record.humanCount = humanCount();
	    ROS_INFO("Time: %d    %d  %d        %f %lf",time, detectionCount, allDetectionsCount, record.evaluation[record.evaluation.size()-1], record.humanCount);
	    reportData.push_back(record);
	    dayTime-=secondsInDay;
	    }*/
	//   ROS_INFO("time: %d final time %d",time, groundTruth.finalTime());
}

/// - public --------------------------------------------------------------
void CWorldSimulator::setCurrentNode(std::string node) 
{
	currentNode = node;
	currentNodeId = map.getNodeId(node) ;
}


/// - public --------------------------------------------------------------
void  CWorldSimulator::setNavigator(actionlib::SimpleActionClient<topological_navigation::GotoNodeAction> * nav) {
  navigator = nav;
}


bool CWorldSimulator::isFinish() {
  return finished;
}

int CWorldSimulator::getDetectionCount() {
  return detectionCount;
}

int CWorldSimulator::getAllDetectionsCount() {
  return allDetectionsCount;
}


int CWorldSimulator::createDir(const std::string rootPath, std::string path, mode_t mode=0777) {
  struct stat st;

  for( std::string::iterator iter = path.begin() ; iter != path.end(); )
  {
    std::string::iterator newIter = std::find( iter, path.end(), '/' );
    std::string newPath = std::string( path.begin(), newIter);

    if( stat( newPath.c_str(), &st) != 0)
    {
      if( mkdir( newPath.c_str(), mode) != 0 && errno != EEXIST )
      {
        std::cout << "cannot create folder [" << newPath << "] : " << strerror(errno) << std::endl;
        return -1;
      }
    }
    else
      if( !S_ISDIR(st.st_mode) )
      {
        errno = ENOTDIR;
        std:: cout << "path [" << newPath << "] not a dir " << std::endl;
        return -1;
      }
      else
        std::cout << "path [" << newPath << "] already exists " << std::endl;


      iter = newIter;
      if( newIter != path.end() )
        ++ iter;
  }
  return 0;
}

/// - public --------------------------------------------------------------
void CWorldSimulator::report() {
  std::stringstream ss;
  ss << "results-aaf/order2-" << order << "/";
  std::string dir = ss.str();
  createDir("",dir);

  ss.str(std::string());
  if (methodName.compare("horizon") == 0) {
    ss << dir << methodName << "_" << std::setw(3) << std::setfill('0') << map.planningHorizonInMinutes << "_" << map.methodName << ".txt";
  } else {
    ss << dir << methodName << "_" << std::setw(3) << std::setfill('0') << A*100 << "_";
    ss << std::setw(3) << std::setfill('0') << B << ".txt";
  }
  std::ofstream ofs(ss.str(), std::ofstream::out);
  for(auto rec: reportData) {
    ofs <<  std::setw(15) << std::setfill('0') << rec.time << " ";
    ofs << std::setw(8) << std::setfill(' ') << rec.detectionCount << " ";
    ofs << std::setw(8) << std::setfill(' ') << rec.allDetectionsCount << " ";
    for(auto e: rec.evaluation) {
      ofs << std::setw(10) << std::setfill(' ') << e << " ";
    }
    ofs << std::setw(10) << std::setfill(' ') << rec.humanCount;
    ofs << std::endl;
  }
  ofs.close();
  SReportData rec = reportData[reportData.size()-1];
  ss.str(std::string());
  ss << dir << "results.txt";
  std::ofstream ofs2(ss.str(), std::ofstream::out | std::ofstream::app);
  ofs2 << std::setw(20) << std::setfill(' ') << std::left << methodName << " ";
  ofs2 << std::setw(5) << std::setfill(' ') << std::right << A << " ";
  ofs2 << std::setw(5) << std::setfill(' ') << std::right << B << " ";
  ofs2 << std::setw(5) << std::setfill(' ') << std::right << rec.detectionCount << " ";
  ofs2 << std::setw(8) << std::setfill(' ') << std::right << rec.allDetectionsCount << " ";
  ofs2 << std::setw(5) << std::setfill(' ') << std::right << rec.evaluation[rec.evaluation.size()-1] << " ";
  ofs2 << std::setw(5) << std::setfill(' ') << std::right << rec.humanCount;
  ofs2 << std::endl;
  ofs2.close();
}

void CWorldSimulator::evaluate(std::vector<float> &evals) {
  evals.clear();
  int i=0;
  float total = 0;
  for(auto n: map.nodes) {
    float ev[5];
    unsigned char states[allTimesCount];
    i=0;
    unsigned int t;
//     for(t=0;t<=groundTruth.finalTime();i++,t+=delta) {
    for(i=0;i<allTimesCount;i++) {
        states[i] = isInteractionDetected(n.id, allTimes[i]) ? 1 : 0;
    }
//     ROS_INFO("COUNT %d %d %d", count, i, t);

    int res = fremen->evaluate(n.name.c_str(),allTimes,states,allTimesCount,4,ev);
//     ROS_INFO("RESULT %d %f" ,res, ev[order]);
    evals.push_back(ev[order]);
    total += ev[order];
  }
  evals.push_back(total);

//   result.success = frelements.evaluate(goal->id.c_str(),(uint32_t*)goal->times.data(),(unsigned char*)goal->states.data(),(int)goal->times.size(),goal->order,evaluations);
}



double CWorldSimulator::humanCount() {
  int i=0;
  double total = 0;
  float pps[allTimesCount];
  for(auto n: map.nodes) {
    fremen->estimate(n.name.c_str(), allTimes, pps, allTimesCount, order);
    for(int i=0;i<allTimesCount;i++) {
      if (pps[i] > 0.5) {
        total++;
      }
    }
  }
  return total/(double)allTimesCount;
}




/* end of worldsimulator.cc */
