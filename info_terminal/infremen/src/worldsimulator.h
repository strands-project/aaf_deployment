/*
 * File name: worldsimulator.h
 * Date:      2015-08-28 12:05:48 +0200
 * Author:    Miroslav Kulich
 */

#ifndef __WORLDSIMULATOR_H__
#define __WORLDSIMULATOR_H__

#include "modelmultigt.h"
#include "graph.h"
#include <actionlib/client/simple_action_client.h>
#include "topological_navigation/GotoNodeAction.h"
#include "CFrelementSet.h"
#include <string>
#include <random>

namespace imr {

class CWorldSimulator {
public:
  enum {
    EXPLOITATION_METHOD,
    MONTECARLO_METHOD,
    ARTIFICIAL_METHOD,
    PROBENTROPY_METHOD,
    RANDOMWALK_METHOD,
    ORACULUM_METHOD,
    EXPLORATION_METHOD,
    SUPERORACULUM_METHOD,
    MC_EXPLORATION_METHOD,
    MC_ARTIFICIAL_METHOD,
    MC_EXPLOITATION_METHOD,
    MC_PROBENTROPY_METHOD,
    HORIZON_METHOD,
  };
  struct SReportData {
    unsigned int time;
    int detectionCount;
    double humanCount;
    int allDetectionsCount;
    std::vector<float> evaluation;
  };
  struct SMeasurementData {
    std::string id;
    uint32_t time;
    unsigned char state;
  };

  CWorldSimulator(Graph &map,CFrelementSet *fremenIn);
  CWorldSimulator(std::string modelName, Graph &map);
  ~CWorldSimulator();
  void loadModel(std::string modelName);
  void step();
  void setCurrentNode(std::string node);
  void goTo(std::string node);
  void setNavigator(actionlib::SimpleActionClient<topological_navigation::GotoNodeAction> * navigator);
  bool isFinish();
  int getDetectionCount();
  int getAllDetectionsCount();
  //void AddMeasurementToFremen(std::string id, uint32_t time, unsigned char state);
  void getProbablitesFromFremen(std::vector<std::string> &ids, std::vector<uint32_t> &times, std::vector<float> &probs);
  void getProbablitesFromFremen(std::vector<uint32_t> &times);
  std::string nextGoal(uint32_t time, uint32_t &delta);
  void initFremen();
  void setMethod(std::string m, std::string sm, double iA, double iB);
  int MonteCarloSelection(std::vector<float> &probs);
  int ExploitationSelection(std::vector<float> &probs);
  int ExplorationSelection(std::vector<float> &probs);
  int ArtificialFuncSelection(double A, double B, std::vector<float> &probs);
  int ProbEntropySelection(double A, std::vector<float> &probs);
  int RandomWalkSelection(std::vector<float> &probs);
  int OraculumSelection(unsigned int time);
  int SuperOraculumSelection(unsigned int time);
  int MonteCarloExploitationSelection(std::vector<float> &probs);
  int MonteCarloExplorationSelection(std::vector<float> &probs);
  int MonteCarloProbEntropySelection(std::vector<float> &probs);
  int MonteCarloArtificialSelection(double A, double B,std::vector<float> &probs);
  int HorizonSelection(unsigned int time);

  void report();
  void evaluate(std::vector<float> &evals);
  //void rememberMeasurement(std::string id, uint32_t time, unsigned char state);
  //void addRememberedMeasurements();
  int createDir(const std::string rootPath, std::string path, mode_t mode);
  double humanCount();
  bool isInteractionDetected(int id, uint32_t time);
  void setOrder(int ord) { order=ord; };
private:
  imr::CModelMultiGT groundTruth;
  Graph &map;
  uint32_t time;
  uint32_t dayTime;

  bool finished;
  std::string currentNode;
  int currentNodeId;
  int detectionCount;
  int allDetectionsCount;
  actionlib::SimpleActionClient<topological_navigation::GotoNodeAction> *navigator;
  CFrelementSet* fremen;
  int method;
  std::string methodName;
  double A;
  double B;
  std::vector<SReportData> reportData;
  std::default_random_engine expDistGenerator;
  uint32_t *allTimes;
  int allTimesCount;
  int order; //fremen order
  std::vector<SMeasurementData> measurements;
};

}

#endif

/* end of worldsimulator.h */
