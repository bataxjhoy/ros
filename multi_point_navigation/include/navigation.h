/**
 * Copyright (c) 2020 COTEK Inc. All rights reserved.
 */
#ifndef NAVIGATION_H_
#define NAVIGATION_H_
#include "ros/ros.h"
#include "planner.h"
#include "navigation_options.h"
#include <mutex>
#include <string>
#include <thread>
using namespace std; 


class Navigation {
 public:
  Navigation();
  ~Navigation() {
    if (executor_) {
      executor_->join();
      executor_ = nullptr;
    }
  }

  bool Init(const NavigationOption &option);

  void Run();


 private:
    std::shared_ptr<std::thread> executor_;
    std::shared_ptr<Planner> planner_;
    void Runner();
    NavigationOption option_;
};

#endif  