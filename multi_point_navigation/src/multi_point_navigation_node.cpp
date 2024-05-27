#include "multi_point_navigation_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_point_navigation_node");
  ros::NodeHandle nh;

  auto navigation = std::make_shared<Navigation>();

  multi_point_navigation_node node(navigation, &nh);

  if (!node.Init()) {
    return -1;
  }
  node.Run();
  ros::spin();
  return 0;
}
