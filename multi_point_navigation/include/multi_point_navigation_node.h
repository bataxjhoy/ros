#ifndef MUTIL_POINT_NAVIGATION_NODE_H_
#define MUTIL_POINT_NAVIGATION_NODE_H_
#include "ros/ros.h"
#include "navigation.h"
#include "navigation_options.h"
using namespace std; 

NavigationOption ConfigFileToNavigationOption( ){
    NavigationOption option = {};
    option.controller_frequency =50;

    return option;   
}


class multi_point_navigation_node final { 
   
private:
    std::shared_ptr<Navigation> navigation_ptr_=std::make_shared<Navigation>();     
    ros::NodeHandle* nh_;
 public:
    multi_point_navigation_node() = delete;
    explicit multi_point_navigation_node(std::shared_ptr<Navigation> navigation, ros::NodeHandle* nh) : navigation_ptr_(navigation), nh_(nh){}

    bool Init() {
        return navigation_ptr_->Init(ConfigFileToNavigationOption());
    }
     void Run() { navigation_ptr_->Run(); }

};
#endif