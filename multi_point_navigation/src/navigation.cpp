#include "navigation.h"
using namespace std; 
Navigation::Navigation(): executor_(nullptr) {
  ros::NodeHandle nh;
}

bool Navigation::Init(const NavigationOption &option) {
  // 配置初始化
   option_ = option;
   planner_ = std::make_shared<Planner>(option_);
  return true;
}

void Navigation::Run() {
  executor_ =
      std::make_shared<std::thread>(std::bind(&Navigation::Runner, this));
}

void Navigation::Runner() {
    ROS_INFO("cotek navigation start.");
     ros::Rate rate(option_.controller_frequency);
     while (ros::ok()) {
    //   // 此处加锁是为了确认 is_waiting 是否被篡改影响
    //   std::unique_lock<std::mutex> lock(mutex_);

    //   // 错误处理
    //   HandleError();

    //   if (planner_->Update(state_manager_, goal_manager_)) {
    //     cotek_msgs::move_cmd move_cmd =
    //         planner_->Plan(state_manager_, goal_manager_);
    //     cmd_vel_pub_.publish(move_cmd);
    //   }
    //   lock.unlock();

    //   if (rate.cycleTime() > ros::Duration(1.0 / option_.controller_frequency)) {
    //     LOG_WARN(
    //         "Control loop missed its desired rate of %.2fHz... the heartbeat "
    //         "actually took %.2f seconds",
    //         option_.controller_frequency, rate.cycleTime().toSec());
    //   }
    //   rate.sleep();
     }
}