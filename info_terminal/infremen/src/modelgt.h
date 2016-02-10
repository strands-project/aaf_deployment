/*
 * File name: modelgt.h
 * Date:      2015-08-26 19:23:01 +0200
 * Author:    Miroslav Kulich
 */

#ifndef __MODELGT_H__
#define __MODELGT_H__
#include <string>
#include <vector>
namespace imr {

class CModelGT {
public:
  CModelGT(std::string filename);
  CModelGT();
  ~CModelGT();
  void load(std::string filename);
  unsigned int finalTime() { return count; };
  char  humanAt(unsigned int time) { return data[time]; };
private:
  unsigned int count;
  std::vector<char> data;
};

}

#endif

/* end of modelgt.h */
