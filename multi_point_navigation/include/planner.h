
#ifndef PLANNER_H_
#define PLANNER_H_
#include "navigation_options.h"
#include "ros/ros.h"
using namespace std; 

class Planner {
 public:
  Planner() = delete;
  explicit Planner(const NavigationOption &option);

 private:
  NavigationOption option_;
};

#endif  
