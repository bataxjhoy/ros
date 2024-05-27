#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <signal.h>

#include <tf/transform_datatypes.h>
#include <fstream>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <xjh_learn/DMVel.h>
#include <xjh_learn/DMPose.h>
#include <xjh_learn/DMGoal.h>

#include <xjh_learn/DMDema.h>
#include "xjh_learn/DMDemarcateAction.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#define PI 3.141592
#define RADIAN_TO_DEGREE 57.324840
ros::Publisher cmd_vel;//发布速度
ros::Publisher pub_error;
ros::Publisher pub_goal;

ros::Publisher pub_move;
ros::Publisher pub_pose;

ros::Publisher set_x_scale;
ros::Publisher set_z_scale;

std::vector<float> g_ranges;//雷达数组

float head_space = 0.0;
float tail_space = 0.0;
float left_space = 0.0;
float right_space = 0.0;
float closest_distance = 0.0;
xjh_learn::DMPose g_goal_a;
xjh_learn::DMPose g_goal_b;
xjh_learn::DMPose g_goal_t;

float move_speed = 0.0;
float spin_speed = 0.0;
float move_target = 0.0;
float spin_target = 0.0;
float scale_factor_x = 1.0;
float scale_factor_z = 1.0;

float g_yaw = 0.0;
float locat_move = 0.0;
float locat_spin = 0.0;
float locat_start_yaw = 0.0;
geometry_msgs::PoseStamped t_goal;
geometry_msgs::PoseStamped g_pose;

typedef enum {
	GOAL_FREE = 0,
	GOAL_A = 1,
	GOAL_B = 2,

	GOAL_SET_A = 11,
	GOAL_SET_B = 12
}GOAL_E; //目标位置or设置目标

typedef enum {
	ROS_STATUS_FREE = 0,
	ROS_STATUS_MAPPING = 1,
	ROS_STATUS_NAVIGATION_FREE = 2,
	ROS_STATUS_NAVIGATION_MOVE = 3,
}ROS_STATUS_E;//建图or导航

typedef enum {
	LOCAT_UNKNOW = -1,
	LOCAT_INIT = 0,
	LOCAT_SEARCH = 1,
	LOCAT_IN_RANGE = 2,
	LOCAT_SPIN = 3,
	LOCAT_MOVE = 4,
	LOCAT_PAUSE = 5,
	LOCAT_FINISH = 6
}LOCAT_E;//机器人状态

LOCAT_E g_locat = LOCAT_UNKNOW;
LOCAT_E last_locat_status = LOCAT_UNKNOW;
ROS_STATUS_E g_ros_status = ROS_STATUS_FREE;


typedef actionlib::SimpleActionClient<xjh_learn::DMDemarcateAction> DemarcateC;//客户端
typedef actionlib::SimpleActionServer<xjh_learn::DMDemarcateAction> DemarcateS;//服务端

void shutdown(int sig)
{
	cmd_vel.publish(geometry_msgs::Twist());
	ROS_INFO("common node quit!");
	ros::shutdown();
}

//发布错误
void publishErrorCmd(const char *msgs)
{
	std_msgs::String msg;
    std::stringstream msgStr;
    msgStr << msgs;
    msg.data = msgStr.str();
    pub_error.publish(msg);
}
//发布速度
void publishMotorCmd(float v, float w)
{
	geometry_msgs::Twist speed;
	
	speed.linear.x = v;
	speed.angular.z = w;
	cmd_vel.publish(speed);	
}
//发布位置
void publishPoseCmd(float x, float y, float a)
{
	xjh_learn::DMPose pose;
	
	pose.x = x;
	pose.y = y;
	pose.angle = a;
	pub_pose.publish(pose);	
}
//发布目标位置
void publishGoalCmd(float x, float y, float z)
{
	geometry_msgs::PoseStamped goal;

	goal.header.frame_id = "map";
	goal.pose.position.x = x;
	goal.pose.position.y = y;
	goal.pose.orientation.z = sin(z / 2);
	goal.pose.orientation.w = cos(z);

	pub_goal.publish(goal);
}
//发布完成
void publishMoveFinish(const char *msgs)
{
	std_msgs::String msg;
    std::stringstream msgStr;
    msgStr << msgs;
    msg.data = msgStr.str();
    pub_move.publish(msg);
}
//发布设置比
void publishSetScale(int type, float scale)
{
	std_msgs::Float32 scale_factor; 

    scale_factor.data = scale; 
	if (type) {
		set_x_scale.publish(scale_factor);
	} else {
		set_z_scale.publish(scale_factor);
	} 
}
//判断 Odom数据更新
bool updatePose(geometry_msgs::Pose pose)
{
	if (g_pose.pose.position.x != pose.position.x
	 || g_pose.pose.position.y != pose.position.y
	 || g_pose.pose.orientation.w != pose.orientation.w) {
		return true;
	} else {
		return false;
	}
}
//odom数据更新
void getOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	if (updatePose(odom->pose.pose)) {
		g_pose.pose.position = odom->pose.pose.position;
		g_pose.pose.orientation = odom->pose.pose.orientation; //更新位置
		tf::Quaternion quat;
		tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//更新欧拉角
		g_yaw = yaw;
		publishPoseCmd(odom->pose.pose.position.x, odom->pose.pose.position.y, yaw * RADIAN_TO_DEGREE);
		ROS_WARN("get odom pose update[x: %.3f, y: %.3f, z: %.3f], [x: %.3f, y: %.3f, z: %.3f, w: %.3f] => yaw[%.3f(%.3f)]", 
			odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z,
			odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w, 
			yaw, yaw * RADIAN_TO_DEGREE);

	}
}
//雷达数据更新
void getScanDataCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
	g_ranges = laser_msg->ranges;
	int range_size = g_ranges.size();
	float *c_range = std::min_element(&g_ranges[0], &g_ranges[range_size - 1]);
	head_space = g_ranges[0];
	tail_space = g_ranges[359];
	left_space = g_ranges[179];
	right_space = g_ranges[539];
	closest_distance = *c_range;
	
}
//订阅app的控制命令
void getVelCallback(const xjh_learn::DMVel::ConstPtr& msg)
{
	if (0.0 == msg->x) {//旋转
		publishMotorCmd(msg->x, msg->yaw);
	} else {
		if (msg->x > 0) {//前进
			float *c_range1 = std::min_element(&g_ranges[0], &g_ranges[59]);
			float *c_range2 = std::min_element(&g_ranges[660], &g_ranges[719]);
			float nearleast = *c_range1 < *c_range2 ? *c_range1 : *c_range2;
			ROS_WARN("f: %f, t: %f, l: %f", g_ranges[0], g_ranges[359], nearleast);
			if (nearleast <= 0.20) {
				publishErrorCmd("前进方向有障碍物");
				publishMotorCmd(0, msg->yaw);
			} else {
				publishMotorCmd(msg->x, msg->yaw);
			}
		} else {//后退
			float *c_range = std::min_element(&g_ranges[300], &g_ranges[419]);
			ROS_WARN("f: %f, t: %f, l: %f", g_ranges[0], g_ranges[359], *c_range);
			if (*c_range <= 0.60) {
				publishErrorCmd("后退方向有障碍物");
				publishMotorCmd(0, msg->yaw);
			} else {
				publishMotorCmd(msg->x, msg->yaw);
			}
		}
	} 
}

//保存目标到文件
bool saveGoalPoseToFile(int goal, float x, float y, float z)
{	
	char filePath[128] = {0};
	if (goal) {
		sprintf(filePath, "/home/ximei/robot/cfg/goal_a.txt");
	} else {
		sprintf(filePath, "/home/ximei/robot/cfg/goal_b.txt");
	}
	std::ofstream storeFile(filePath);
	if (!storeFile) {
		ROS_WARN("can not open store goal file!");
	} else {
		storeFile << x << " " << y << " " << z ;
		ROS_WARN("store goal %d to file: x: %.3f, y: %.3f, a: %.3f success!!", goal, x, y, z);
		return true;
	}
	return false;
}

//获取目标的位置
bool getGoalPoseFromFile() 
{
	double storePose[3] = {0};
	
	std::ifstream aFile("/home/ximei/robot/cfg/goal_a.txt");
	if (!aFile) {
		memset(&g_goal_a, 0x00, sizeof(g_goal_a));
	} else {
		int i = 0;
		double tmp = 0.0;
		while(aFile >> tmp) {
			storePose[i++] = tmp;
		}
		g_goal_a.x = storePose[0];
		g_goal_a.y = storePose[1];
		g_goal_a.angle = storePose[2];
		ROS_INFO("load goal a pose success!");
		aFile.close();
  	}

	std::ifstream bFile("/home/ximei/robot/cfg/goal_b.txt");
	if (!bFile) {
		memset(&g_goal_b, 0x00, sizeof(g_goal_b));
		return false;
	} else {
		int i = 0;
		double tmp = 0.0;
		while(bFile >> tmp) {
			storePose[i++] = tmp;
		}
		g_goal_b.x = storePose[0];
		g_goal_b.y = storePose[1];
		g_goal_b.angle = storePose[2];
		ROS_INFO("load goal b pose success!");
		bFile.close();
	}

	ROS_WARN("a[%f, %f, %f], b[%f, %f, %f]", g_goal_a.x, g_goal_a.y, g_goal_a.angle, g_goal_b.x, g_goal_b.y, g_goal_b.angle);
	return true;
}
//获取目标回调
void getGoalCallback(const xjh_learn::DMGoal::ConstPtr& msg)
{
	g_locat = LOCAT_INIT;
	if (g_ros_status == ROS_STATUS_NAVIGATION_FREE)
		g_ros_status = ROS_STATUS_NAVIGATION_MOVE;
	if (GOAL_FREE == msg->type) {  //其他目标
		g_goal_t.x = msg->goal_x;
		g_goal_t.y = msg->goal_y;
		g_goal_t.angle = msg->goal_z;
		publishGoalCmd(msg->goal_x, msg->goal_y, msg->goal_z);
	} else if (GOAL_A == msg->type) { //去A点
		g_goal_t= g_goal_a;
		publishGoalCmd(g_goal_a.x, g_goal_a.y, g_goal_a.angle);
	} else if (GOAL_B == msg->type) { //去B点
		g_goal_t = g_goal_b;
		publishGoalCmd(g_goal_b.x, g_goal_b.y, g_goal_b.angle);
	} else if (GOAL_SET_A == msg->type) {  //设置A点
		saveGoalPoseToFile(1, msg->goal_x, msg->goal_y, msg->goal_z);
		g_goal_a.x = msg->goal_x;
		g_goal_a.y = msg->goal_y;
		g_goal_a.angle = msg->goal_z;
	} else if (GOAL_SET_B == msg->type) { //设置B点
		saveGoalPoseToFile(0, msg->goal_x, msg->goal_y, msg->goal_z);
		g_goal_b.x = msg->goal_x;
		g_goal_b.y = msg->goal_y;
		g_goal_b.angle = msg->goal_z;
	} else {
		publishErrorCmd("未定义的目标类型");
		g_ros_status = ROS_STATUS_FREE;
	}
}
//检查是否直线运动完成
//Point c：起点
//Point g：终点
//float t：目标距离
bool checkMoveFinish(geometry_msgs::Point c, geometry_msgs::Point g, float t)
{
	float  distance=sqrt((g.x - c.x)*(g.x - c.x)+(g.y - c.y)*(g.y - c.y));
	if (distance- abs(t) > 0.001) {
		return true;
	} else {
		return false;
	}
}
//标定完成回调
void demarcateDoneCb(const actionlib::SimpleClientGoalState& state, const xjh_learn::DMDemarcateResultConstPtr& result)
{
	ROS_WARN("demarcateDoneCb done!");
	// publishMoveFinish("done");
}
//活跃回调
void demarcateActiveCb()
{
	ROS_WARN("demarcateActiveCb...");
}
//反馈回调
void demarcateFeedbackCb(const xjh_learn::DMDemarcateFeedbackConstPtr& feedback)
{
	ROS_WARN("demarcateFeedbackCb feedback[%f]%%", feedback->percent * 100);
}
//标定初始化
void getDemarcateCallback(const xjh_learn::DMDema::ConstPtr& msg)
{	 
	float dema_target = 0.0;
	xjh_learn::DMDemarcateGoal goal;
	if (strstr(msg->type.c_str(), "move") != NULL) { //距离标定
		goal.type = 1;
		move_target = msg->target;
		dema_target = move_target;
	} else if (strstr(msg->type.c_str(), "spin") != NULL) {//旋转标定
		goal.type = 2;
		spin_target = msg->target;
		dema_target = spin_target / RADIAN_TO_DEGREE;
	} else {
		return ;
	}

	goal.target = dema_target;
	DemarcateC dc("demarcate", true);
	dc.sendGoal(goal, &demarcateDoneCb, &demarcateActiveCb, &demarcateFeedbackCb);
	ros::spin(); 
}
//标定操作
void demarcateExecute(const xjh_learn::DMDemarcateGoalConstPtr& goal, DemarcateS *as)
{
	ros::Rate r(20);
	float tm_speed = 0;
	float ts_speed = 0;
	xjh_learn::DMDemarcateFeedback fb;
	ROS_WARN("doing type %d, target = %f", goal->type, goal->target);
	if (goal->type == 1) {    			//1直线运动
		if (goal->target > 0)
			tm_speed = move_speed;
		else
			tm_speed = -move_speed;
	} else if (goal->type == 2) {		//2旋转运动
		if (goal->target > 0)
			ts_speed = spin_speed;
		else 
			ts_speed = -spin_speed;
	} else {
		as->setSucceeded();
		return ;
	}

	ROS_WARN("move: %.03f, spin: %.03f", tm_speed, ts_speed);

	geometry_msgs::Pose t_pose;
	t_pose.position.x = g_pose.pose.position.x; //起始位置t_pose 当前位置g_pose 目标距离goal->target
	t_pose.position.y = g_pose.pose.position.y;
	t_pose.orientation.w = g_pose.pose.orientation.w;

	float t_yaw = g_yaw;//起始角度t_yaw 当前角度g_yaw 目标角度goal->target
	float turn_angle = 0;//累计旋转角度
	ros::NodeHandle n;
	while(n.ok()) {
		if (goal->type == 1) {//直线运动
			float  current_distance=sqrt((g_pose.pose.position.x - t_pose.position.x)*(g_pose.pose.position.x - t_pose.position.x)+
										(g_pose.pose.position.y - t_pose.position.y)*(g_pose.pose.position.y - t_pose.position.y));
			ROS_WARN(" target_distance:%f, current_distance:%f", goal->target, current_distance);//显示目标距离 与当前运动距离

			if (!checkMoveFinish(t_pose.position, g_pose.pose.position, goal->target)) {
				publishMotorCmd(tm_speed, 0);
			} else {
				publishMotorCmd(0, 0);
				as->setSucceeded();
				publishMoveFinish("move"); 
				break;
			}
		}
		else if (goal->type == 2) {//旋转运动

			float delta_angle=0;//单周期运动角度
			if( goal->target > 0) //逆时针
			{
				if(g_yaw >= t_yaw)
					delta_angle = g_yaw-t_yaw; 
				else
					delta_angle = g_yaw+2*PI-t_yaw;  //PI--->(-PI)

			}else if (goal->target < 0)//顺时针
			{
				if(g_yaw <= t_yaw)
					delta_angle = t_yaw-g_yaw; 
				else
					delta_angle = t_yaw-g_yaw+2*PI;	//(-PI)--->PI
			}
			t_yaw = g_yaw;//重新赋值
			turn_angle +=delta_angle;//累计旋转角度
			ROS_WARN(" target_angle:%f, turn_angle:%f", goal->target, turn_angle);
			//ROS_WARN(" c:%f, t:%f, g:%f", t_yaw, goal->target, g_yaw);
			//if (!checkSpinFinish(t_yaw, g_yaw, goal->target)) 
			if(turn_angle-abs(goal->target) < 0.0001){
				// fb.percent = (abs(t_pose.orientation.w) - abs(g_pose.pose.orientation.w)) / goal->target;
				// as->publishFeedback(fb);
				publishMotorCmd(0, ts_speed);
			} else {
				publishMotorCmd(0, 0);
				// fb.percent = 1;
				// as->publishFeedback(fb);
				as->setSucceeded();
				publishMoveFinish("spin");
				break;
			}
			
		} 
		ros::spinOnce();
		r.sleep();
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xjh_node");
   
 	ros::NodeHandle private_nh("~");
	private_nh.getParam("move_speed", move_speed);//读取移动速度
 	ROS_WARN("move_speed=%f",move_speed);
	private_nh.param<float>("spin_speed", spin_speed, 0.120);//旋转速度
	private_nh.param<float>("scale_factor_x", scale_factor_x, 1.0);//直线运动比例
	private_nh.param<float>("scale_factor_z", scale_factor_z, 1.0);//旋转运动比例
	// private_nh.setParam("move_speed", 100);//设置移动速度
	// private_nh.getParam("move_speed", move_speed);//查看移动速度
 	// ROS_WARN("move_speed=%f",move_speed);

	getGoalPoseFromFile();//获取AB目标点

   	ros::NodeHandle nh;

    //发布话题
    cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //发布速度    
	pub_error = nh.advertise<std_msgs::String>("/dm_error", 1);//发布错误
	
	pub_move = nh.advertise<std_msgs::String>("/move_stop", 1);
	
	set_x_scale = nh.advertise<std_msgs::Float32>("/set_scale_x", 1);
	set_z_scale = nh.advertise<std_msgs::Float32>("/set_scale_z", 1);
	pub_pose = nh.advertise<xjh_learn::DMPose>("/dm_pose", 1);
	pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);//发布目标话题

    //订阅话题
	ros::Subscriber vel_sub = nh.subscribe("/dm_vel", 1, getVelCallback);//前进后退速度回调
    ros::Subscriber laser_sub = nh.subscribe("/scan", 1, getScanDataCallback);	
	ros::Subscriber goal_sub = nh.subscribe("/dm_goal", 1, getGoalCallback);//获取目标回调
	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, getOdomCallback);
	ros::Subscriber demarcate_sub = nh.subscribe("/dm_demarcate", 1, getDemarcateCallback); //标定回调
	DemarcateS ds(nh, "demarcate", boost::bind(&demarcateExecute, _1, &ds), false);
	ds.start();

    signal(SIGINT, shutdown);
    ros::spin();
    
    return 0;

}