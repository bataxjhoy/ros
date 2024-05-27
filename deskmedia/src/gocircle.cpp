#include "ros/ros.h"

#include <signal.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmd_vel;

void shutdown(int sig)
{
	cmd_vel.publish(geometry_msgs::Twist());
	ROS_INFO("go circles finish!!!");
	ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gocircle");
	std::string topic = "/cmd_vel";
	ros::NodeHandle node;
	cmd_vel = node.advertise<geometry_msgs::Twist>(topic, 1);
	ros::Rate loopRate(10);

	signal(SIGINT, shutdown);

	geometry_msgs::Twist speed;
	while(ros::ok())
	{
		speed.linear.x = 0;
		speed.angular.z = 0.5;
		cmd_vel.publish(speed);
		loopRate.sleep();
	}

	return 0;
}