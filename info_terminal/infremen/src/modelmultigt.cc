/*
 * File name: modelgt.cc
 * Date:      2015-08-26 19:23:01 +0200
 * Author:    Miroslav Kulich
 */

#include "ros/ros.h"
#include "modelmultigt.h"


#include<algorithm>
#include<iterator>
#include <fstream>

using namespace imr;


/// - constructor --------------------------------------------------------------
CModelMultiGT::CModelMultiGT(std::string fileName) {
  load(fileName);
}

/// - constructor --------------------------------------------------------------
CModelMultiGT::CModelMultiGT() : count(0) {
}

/// - constructor --------------------------------------------------------------
void CModelMultiGT::load(std::string fileName) {
  std::ifstream ifs(fileName);
  if (!ifs.is_open()) {
    ROS_ERROR("File %s  can nont be open",fileName.c_str());
  }
//  count =std::count(std::istreambuf_iterator<char>(ifs),  std::istreambuf_iterator<char>(), '\n');
//  ROS_INFO("num lines %d",count);
//  ifs.seekg (0, ifs.beg);

  int token;
  count = 0;
unsigned int i = 1;
  data.clear();
  std::string line;
  while(std::getline(ifs, line)) {
//     std::cout << "line " << i++ << "    ";
    std::istringstream iss(line);
    std::vector<char> d;
    while (iss >> token) {
      d.push_back((char)token);

//       std::cout << token << " ";
    }
    count++;
    data.push_back(d);
//     std::cout << std::endl;
  }

  ROS_INFO("Groundtruth file loaded (%d records).",count+1);
}

/// - destructor --------------------------------------------------------------
CModelMultiGT::~CModelMultiGT() {
}


/* end of modelgt.cc */
