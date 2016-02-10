/*
 * File name: modelmultigt.h
 * Date:      2015-08-26 19:23:01 +0200
 * Author:    Miroslav Kulich
 */

#ifndef __MODELGT_H__
#define __MODELGT_H__
#include <string>
#include <vector>
namespace imr {

class CModelMultiGT {
public:
  CModelMultiGT(std::string filename);
  CModelMultiGT();
  ~CModelMultiGT();
  void load(std::string filename);
  unsigned int finalTime() { return 60*count; }; /// ground truth has time in minutes
  bool  isHumanAt(int id, unsigned int time) { return data[time/60][id] == 1; };// ground truth has time in minutes
private:
  unsigned int count;
  std::vector<std::vector<char> >data;
};

}

#endif

/* end of modelgt.h */
