/*
 * File name: modelgt.cc
 * Date:      2015-08-26 19:23:01 +0200
 * Author:    Miroslav Kulich
 */

#include "ros/ros.h"
#include "modelgt.h"


#include<algorithm>
#include<iterator>
#include <fstream>

using namespace imr;


/// - constructor --------------------------------------------------------------
CModelGT::CModelGT(std::string fileName) {
  load(fileName);
}

/// - constructor --------------------------------------------------------------
CModelGT::CModelGT() : count(0) {
}

/// - constructor --------------------------------------------------------------
void CModelGT::load(std::string fileName) {
  std::ifstream ifs(fileName);
  if (!ifs.is_open()) {
    ROS_ERROR("File %s  can nont be open",fileName.c_str());
  }
//  count =std::count(std::istreambuf_iterator<char>(ifs),  std::istreambuf_iterator<char>(), '\n');
//  ROS_INFO("num lines %d",count);
//  ifs.seekg (0, ifs.beg);

  int token;
//  unsigned int i = 0;
  data.clear();
  while (ifs >> token) {
//   while (!ifs.eof()) {
//     ifs >> token;
    data.push_back((char) token);
//    std::cout << token << " (" << ++i << " )" ;
  }
  count = data.size();
//   std::cout << data.size() << std::endl;
  ROS_INFO("Groundtruth file loaded.");
}

/// - destructor --------------------------------------------------------------
CModelGT::~CModelGT() {
}


/* end of modelgt.cc */
