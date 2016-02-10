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
CWorldSimulator::CWorldSimulator(Graph &map) :
map(map),
time(0),
isHumanDetected(false),
finished(false),
detectionCount (0),
order (1) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  expDistGenerator = std::default_random_engine(seed);
}

/// - constructor --------------------------------------------------------------
CWorldSimulator::CWorldSimulator(std::string modelName, Graph &map) :
map(map),
time(0),
isHumanDetected(false),
finished(false),
detectionCount (0),
allDetectionsCount(0),
order(2) {
  loadModel(modelName);
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  expDistGenerator = std::default_random_engine(seed);
}

/// - destructor --------------------------------------------------------------
CWorldSimulator::~CWorldSimulator() {
  delete[] allTimes;
}



/// - public --------------------------------------------------------------
void CWorldSimulator::loadModel(std::string modelName) {
  groundTruth.load(modelName);
  detectionCount = 0;
  delta = 600;
  int i=0;
  allTimes = new uint32_t[groundTruth.finalTime()/delta+1];
  for(unsigned int t=0;t<=groundTruth.finalTime();i++,t+=delta) {
    allTimes[i] = t;
  }
}

/// - public --------------------------------------------------------------
void CWorldSimulator::setMethod(std::string m, double iA, double iB) {
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

  methodName = m;
  method = methodType[m];
  ROS_INFO("STOD (%lf) (%lf)",iA, iB);
  A = iA;
  B = iB;
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

void CWorldSimulator::rememberMeasurement(std::string id, uint32_t time, unsigned char state) {
  SMeasurementData data;
  data.id = id;
  data.time = time;
  data.state = state;
  measurements.push_back(data);
}

/*void CWorldSimulator::addRememberedMeasurements() {
  for(auto m: measurements) {
    AddMeasurementToFremen(m.id,m.time,m.state);
  }
  measurements.clear();
}*/

//TTT remove - mine from mongodb
/*void CWorldSimulator::AddMeasurementToFremen(std::string id, uint32_t time, unsigned char state) {
  uint32_t times[1];
  unsigned char states[1];
  times[0] = time;
  states[0] = state;
  int result = fremen.add(id.c_str(),times,states,1);
//   ROS_INFO("%d elements added    time: %d state:  %s value: %d", result,times[0], id.c_str(), (int) states[0]);
}*/

void CWorldSimulator::initFremen() {
  uint32_t times[0];
  unsigned char states[0];

  for(auto n: map.nodes) {
    std::cout <<"Init    "<< n.name << " " << fremen->add(n.name.c_str(),times,states,0) << std::endl;
  }
}

//TTT externalize FreMen
void CWorldSimulator::getProbablitesFromFremen(std::vector<std::string> ids, std::vector<uint32_t> times, std::vector<float> &probs) {
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
//   fremen.estimate(names,tms,pps,ids.size(), 2);
//   probs.resize(ids.size());
//   std::copy(pps, pps+ids.size(), probs);


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
//  fremen.print(true);
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
    if (f>max) {
      max = f;
      maxIndex = i;
    }
  }
//   int maxIndex = std::distance(probs.begin(),std::max_element(probs.begin(), probs.end()));
  return maxIndex;
}


int CWorldSimulator::OraculumSelection(unsigned int time) {
  int result = groundTruth.humanAt(time);
  if (result < 0 || result >= map.nodes.size()) { result = 0; }
  return result;
}

int CWorldSimulator::SuperOraculumSelection(unsigned int time) {
  int result = groundTruth.humanAt(time);
  if (result < 0 || result >= map.nodes.size()) { result = 0; }
  return result;
}


std::string CWorldSimulator::nextGoal(unsigned int time) {
  std::vector<std::string> ids;
  std::vector<uint32_t> times;
  std::vector<float> probs;
  std::string nodeName;


//  for(auto link: map.nodes[currentNodeId].links) {
//  nodeName = map.getNodeName(link.first);
  for(auto n: map.nodes) { //TODO: use graph
      nodeName = n.name;
    ids.push_back(nodeName);
    times.push_back(time);
  }
//   ids.push_back(currentNode); //TODO:
//   times.push_back(time);


  getProbablitesFromFremen(ids,times,probs);
  int maxIndex;
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

  return ids[maxIndex];
}


/// - public --------------------------------------------------------------
void CWorldSimulator::step() {
  static int day = 1;
  int gtNode = groundTruth.humanAt(time);
//   ROS_INFO("Ground truth: %d   current posisition %d    %s",gtNode,currentNodeId, (gtNode == currentNodeId ? "yeah" : ":-(("));
  std::string goal;
  static int iter = 0;
//   ROS_INFO("Time %d Current node: %s  human at %s", time, currentNode.c_str(), (gtNode == 9 ? "Outside" : map.getNodeName(gtNode).c_str()));
  iter++;
  if (iter%100==0) {
    SReportData record;
    record.time = time;
    record.detectionCount = detectionCount;
    record.allDetectionsCount = allDetectionsCount;
    evaluate(record.evaluation);
    record.humanCount = humanCount();
    ROS_INFO("Time: %d    %d  %d        %f %lf",time, detectionCount, allDetectionsCount, record.evaluation[record.evaluation.size()-1], record.humanCount);
    reportData.push_back(record);
  }

//   AddMeasurementToFremen(currentNode, time, (gtNode == currentNodeId ? 1 : 0));

if (!isHumanDetected || !gtNode == currentNodeId) {
  if (method == SUPERORACULUM_METHOD) {
    for(auto n:map.nodes) {
      rememberMeasurement(n.name, time, (gtNode == n.id ? 1 : 0));
    }
  } else {
      rememberMeasurement(currentNode, time, (gtNode == currentNodeId ? 1 : 0));
  }
}
//create Tasksk here TTT
  if (isHumanDetected) {
    allDetectionsCount++;
    if (gtNode == currentNodeId) { // human still at the node
      goTo(currentNode);
      // do nothing return;
    } else { //human disappeared  ==> planning needed
      isHumanDetected=false;
      goal = nextGoal(time+delta);
      goTo(goal);
    }
  } else {
    if (gtNode == currentNodeId) { //human found
//      ROS_INFO("Time %d:  Human found at %s ", time, currentNode.c_str());
      detectionCount++;
      isHumanDetected = true;
      goTo(currentNode);
    } else { //human still not found ==> planning needed
      goal = nextGoal(time+delta);
      goTo(goal);
    }
  }


  time += delta; //increase simulation time
  if (time >= groundTruth.finalTime()) {
    finished = true;
  } else if (time % secondsInDay == 0) {
    ROS_INFO("DAY %d finished",day++);
    addRememberedMeasurements();
  }

  //  finished = time > groundTruth.finalTime()/2; //
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
  std::string dir = "resultsNB600-1/";
  createDir("",dir);

  ss << dir << methodName << "_" << std::setw(3) << std::setfill('0') << A*100 << "_";
  ss << std::setw(3) << std::setfill('0') << B << ".txt";

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
  int count = groundTruth.finalTime()/delta+1;
  int i=0;
  float total = 0;
  for(auto n: map.nodes) {
    float ev[5];
    unsigned char states[count];
    i=0;
    unsigned int t;
    for(t=0;t<=groundTruth.finalTime();i++,t+=delta) {
      states[i] = groundTruth.humanAt(t) == n.id ? 1 : 0;
//      if (groundTruth.humanAt(t) != n.id) states[i]=1;
    }
//     ROS_INFO("COUNT %d %d %d", count, i, t);

    int res = fremen->evaluate(n.name.c_str(),allTimes,states,count,4,ev);
//     ROS_INFO("RESULT %d %f" ,res, ev[order]);
    evals.push_back(ev[order]);
    total += ev[order];
  }
  evals.push_back(total);

//   result.success = frelements.evaluate(goal->id.c_str(),(uint32_t*)goal->times.data(),(unsigned char*)goal->states.data(),(int)goal->times.size(),goal->order,evaluations);
}



double CWorldSimulator::humanCount() {
  int count = groundTruth.finalTime()/delta+1;
  int i=0;
  double total = 0;
  float pps[count];
  for(auto n: map.nodes) {
    fremen->estimate(n.name.c_str(), allTimes, pps, count, order);
    for(int i=0;i<count;i++) {
      if (pps[i] > 0.5) {
        total++;
      }
    }
  }
  return total/(double)count;
}




/* end of worldsimulator.cc */
