/**
 * @brief 席媒控制节点文件 
 * @author D 
 * @version 1.0
 * @date 2021-10-25
 * */

#include "deskmedia_node.h"

/**
 * @brief 席媒控制节点入口函数 
 * */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "common_node");
	ROS_INFO("params: %d", argc);

	//环境参数
	tf2_ros::Buffer buffer(ros::Duration(20));
  	tf2_ros::TransformListener tf(buffer);

	//实例化对象
	deskmedia_node common_node(buffer);
	ros::NodeHandle nh;
	common_node.InitNode(nh);
	ros::spin();
	return 0;
}

/**
 * @brief 系统中断入口函数 
 * @param sig 中断类型
 * 
 * */
void Shutdown(int sig)
{
	// cmd_vel_.publish(geometry_msgs::Twist());
	ROS_INFO("common node quit!");
	ros::shutdown();
}

/**
 * @brief 四元素转欧拉角 
 * @param orientation-四元素
 * @return 欧拉角
 * */
float deskmedia_node::QuaternionTransToYaw(geometry_msgs::Quaternion orientation)
{
	tf::Quaternion quat;
	tf::quaternionMsgToTF(orientation, quat);

	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	return (float)yaw;
}

/**
 * @brief 保存指定坐标到文件 
 * @param set_pose-指定坐标
 * @return true/false 成功/失败
 * */
bool deskmedia_node::SaveAppoitPoseToFile(geometry_msgs::PoseWithCovarianceStamped set_pose)
{
	char file_path[128] = {0};
	sprintf(file_path, "%spose.txt", DATA_STORE_PATH);
	std::ofstream store_file(file_path);
	if (!store_file) {
		ROS_WARN("can not open store pose file!");
	} else {
		float yaw = QuaternionTransToYaw(set_pose.pose.pose.orientation);
		store_file << set_pose.pose.pose.position.x << " " 
				   << set_pose.pose.pose.position.y << " " 
				   << yaw << " "
				   << set_pose.pose.covariance[6*0+0] << " "
				   << set_pose.pose.covariance[6*1+1] << " " 
				   << set_pose.pose.covariance[6*5+5];
		store_file.close();
		ROS_WARN("store pose to file: x: %.3f, y: %.3f, a: %.3f success!! --- xx: %.3f, yy: %.3f, aa: %.3f", 
		  set_pose.pose.pose.position.x, set_pose.pose.pose.position.y, yaw,
		  set_pose.pose.covariance[6*0+0], set_pose.pose.covariance[6*1+1], set_pose.pose.covariance[6*5+5]);
		  return true;
	}
	return false;
}

/**
 * @brief 保存当前EKF坐标到文件 
 * @param 
 * @return true/false 成功/失败
 * */
bool deskmedia_node::SaveOdomCombPoseToFile()
{
	char file_path[128] = {0};
	sprintf(file_path, "%spose.txt", DATA_STORE_PATH);
	std::ofstream store_file(file_path);
	if (!store_file) {
		ROS_WARN("can not open store pose file!");
	} else {
		store_file << g_ekf_pose_.pose.pose.position.x << " " 
				   << g_ekf_pose_.pose.pose.position.y << " " 
				   << g_ekf_yaw_ << " "
				   << g_ekf_pose_.pose.covariance[6*0+0] << " "
				   << g_ekf_pose_.pose.covariance[6*1+1] << " " 
				   << g_ekf_pose_.pose.covariance[6*5+5];
		store_file.close();
		ROS_WARN("store pose to file: x: %.3f, y: %.3f, a: %.3f success!! --- xx: %.3f, yy: %.3f, aa: %.3f", 
		  g_ekf_pose_.pose.pose.position.x, g_ekf_pose_.pose.pose.position.y, g_ekf_yaw_,
		  g_ekf_pose_.pose.covariance[6*0+0], g_ekf_pose_.pose.covariance[6*1+1], g_ekf_pose_.pose.covariance[6*5+5]);
		  return true;
	}
	return false;
}

/**
 * @brief 保存当前carto坐标到文件 
 * @param 
 * @return true/false 成功/失败
 * */
bool deskmedia_node::SaveCartographPoseToFile()
{
	char file_path[128] = {0};
	sprintf(file_path, "%scarto_pose.txt", DATA_STORE_PATH);
	std::ofstream store_file(file_path);
	if (!store_file) {
		ROS_WARN("can not open carto store pose file!");
	} else {
		store_file << g_carto_pose_.pose.position.x << " " 
				   << g_carto_pose_.pose.position.y << " " 
				   << g_carto_pose_.pose.orientation.x << " " 
				   << g_carto_pose_.pose.orientation.y << " " 
				   << g_carto_pose_.pose.orientation.z << " " 
				   << g_carto_pose_.pose.orientation.w; 
				   
		store_file.close();
		return true;
	}
	return false;
}

/**
 * @brief 保存禁行区域参数到文件 
 * @param 
 * @return true/false 成功/失败
 * */
bool deskmedia_node::SaveForbidAreaInfoToFile(int sx, int sy, int fx, int fy)
{
	char file_path[128] = {0};
	sprintf(file_path, "%sforbid.txt", DATA_STORE_PATH);
	std::ofstream store_file(file_path, ios::app);
	if (!store_file) {
		ROS_WARN("can not open store forbid area file!");
	} else {
		store_file << sx << " " 
				   << sy << " " 
				   << fx << " "
				   << fy << "\n";
		store_file.close();
		return true;
	}
	return false;
}

/**
 * @brief 从文件获取禁行区域参数 
 * @param 
 * @return true/false 成功/失败
 * */
bool deskmedia_node::GetForbidAreaInfoFromFile()
{
	
	return false;
}


/**
 * @brief 保存坐标到文件 
 * @param goal-坐标类型 1-A点；0-B点
 * 		  x,y,z-坐标实际的x,y及角度参数
 * @return true/false 成功/失败
 * */
bool deskmedia_node::SaveGoalPoseToFile(int goal, float x, float y, float z)
{	
	char filePath[128] = {0};
	if (1 == goal) {
		sprintf(filePath, "%sgoal_a.txt", DATA_STORE_PATH);
	} else if (2 == goal){
		sprintf(filePath, "%sgoal_b_amcl.txt", DATA_STORE_PATH); 
	} else {
		sprintf(filePath, "%sgoal_b.txt", DATA_STORE_PATH);
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

/**
 * @brief 从文件获取A/B点坐标 
 * @param goal_a-A点坐标 
 * 		  goal_b-B点坐标
 * @return true/false 成功/失败
 * */
bool deskmedia_node::GetGoalPoseFromFile(deskmedia::DMPose *goal_a, deskmedia::DMPose *goal_b) 
{
	char file_path[128] = {0};
	double store_pose[3] = {0};
	
	sprintf(file_path, "%s%s", DATA_STORE_PATH, "goal_a.txt");
	// ROS_WARN("goal a file path: %s", file_path);
	std::ifstream a_point_file(file_path);
	if (!a_point_file) {
		memset(goal_a, 0x00, sizeof(deskmedia::DMPose));
	} else {
		int i = 0;
		double tmp = 0.0;
		while(a_point_file >> tmp) {
			store_pose[i++] = tmp;
		}
		goal_a->x = store_pose[0];
		goal_a->y = store_pose[1];
		goal_a->angle = store_pose[2];
		ROS_INFO("load goal a pose success!");
		a_point_file.close();
  	}

	sprintf(file_path, "%s%s", DATA_STORE_PATH, "goal_b.txt");
	// ROS_WARN("goal b file path: %s", file_path);
	std::ifstream b_point_file(file_path);
	if (!b_point_file) {
		memset(&goal_b, 0x00, sizeof(deskmedia::DMPose));
		return false;
	} else {
		int i = 0;
		double tmp = 0.0;
		while(b_point_file >> tmp) {
			store_pose[i++] = tmp;
		}
		goal_b->x = store_pose[0];
		goal_b->y = store_pose[1];
		goal_b->angle = store_pose[2];
		ROS_INFO("load goal b pose success!");
		b_point_file.close();
	}

	ROS_ERROR("a[%f, %f, %f], b[%f, %f, %f]", g_goal_a_.x, g_goal_a_.y, g_goal_a_.angle, g_goal_b_.x, g_goal_b_.y, g_goal_b_.angle);
	return true;
}

/**
 * @brief 触发amcl模块中的位置更新 
 * @param x,y,z - 想要更新的位置坐标
 * @return 
 * */
void deskmedia_node::SyncPoseToAmcl(float x, float y, float z)
{
	geometry_msgs::PoseWithCovarianceStamped syncPose;

	syncPose.header.frame_id = "map";

	syncPose.pose.pose.position.x = x;
	syncPose.pose.pose.position.y = y;

	syncPose.pose.pose.orientation.z = sin(z / 2);
	syncPose.pose.pose.orientation.w = cos(z / 2);

	syncPose.pose.covariance[6*0+0] = 0.25;
	syncPose.pose.covariance[6*1+1] = 0.25;
	syncPose.pose.covariance[6*5+5] = 0.068539;

	pub_update_.publish(syncPose);
}

void deskmedia_node::SyncGetDataFlagToTurnOnEmb(bool flag)
{
	std_msgs::UInt8 sync;

	if (flag)
    	sync.data = 1; 
	else
		sync.data = 0; 
    
	pub_sync_flag_.publish(sync);
}

void deskmedia_node::SetPoseCallback(const deskmedia::DMPose::ConstPtr& msg)
{
	ROS_WARN("try to set pose to amcl(%f, %f, %f)", msg->x, msg->y, msg->angle / RADIAN_TO_DEGREE);
	SyncPoseToAmcl(msg->x, msg->y, msg->angle / RADIAN_TO_DEGREE);
}

void deskmedia_node::GetPoseCallback(const std_msgs::String::ConstPtr & msg)
{
	if (g_ros_status_ == ROS_STATUS_MAPPING) {
//		PublishPoseCmd(g_pose_.pose.position.x, g_pose_.pose.position.y, g_odom_yaw_ * RADIAN_TO_DEGREE);
		PublishPoseCmd(g_ekf_pose_.pose.pose.position.x, g_ekf_pose_.pose.pose.position.y, g_ekf_yaw_ * RADIAN_TO_DEGREE);
	} else {
		PublishPoseCmd(g_amcl_pose_.pose.pose.position.x, g_amcl_pose_.pose.pose.position.y, g_amcl_yaw_ * RADIAN_TO_DEGREE);
	}
}

/**
 * @brief 清除所有缓存位置信息 
 * @param 
 * @return 
 * */
void deskmedia_node::clearAllPoseData()
{
	PublishClearCmd("");
	system("rm -f /home/ximei/robot_robot/cfg/carto_pose.txt");
	memset(&g_pose_, 0x00, sizeof(geometry_msgs::PoseStamped));
	memset(&g_carto_pose_, 0x00, sizeof(geometry_msgs::PoseStamped));
	memset(&g_ekf_pose_, 0x00, sizeof(geometry_msgs::PoseWithCovarianceStamped));
	memset(&g_amcl_pose_, 0x00, sizeof(geometry_msgs::PoseWithCovarianceStamped));
}

/**
 * @brief 清除禁行区域参数文件 
 * @param 
 * @return 
 * */
void deskmedia_node::clearForbidAreaInfoFile()
{
	char cmd[128] = {0};
	sprintf(cmd, "rm -rf %sforbid.txt", DATA_STORE_PATH);
	system(cmd);
}

/**
 * @brief 获取底盘尾部与面板的倾向
 * @param degree - 检测角度
 * @return -1/0/1/2 - 无效参数/垂直/右倾/左倾
 * */
int deskmedia_node::GetDeviceTiltDirection(int degree, bool type = true)
{
	int tail_array = 359;
	int degree_to_point = degree * 2;
	float mid_left = 0;
	float mid_right = 0;
	if (type) {
		mid_left = FindValidDistance(tail_array + degree_to_point - 1, false);
		mid_right = FindValidDistance(tail_array - degree_to_point + 1, true);
	} else {
		mid_left = g_ranges_[tail_array + degree_to_point - 1];
		mid_right = g_ranges_[tail_array - degree_to_point + 1];
	}
	ROS_WARN("get tilt mid [%.4f], left [%.4f], right [%.4f]", g_ranges_[tail_array], mid_left, mid_right);
	if (isinf(mid_left) || isinf(mid_right))
		return -1;
	if (abs(mid_left - mid_right) < 0.005)
		return 0;
	else {
		if (mid_left - mid_right > 0)
			return 1;
		else {
			return 2;
		}
	}
	return -1;
}

/**
 * @brief 获取尾部空间距离
 * @param 
 * @return 尾部空间距离
 * */
float deskmedia_node::GetTailSpaceDistance()
{
	int tail_array = 359;
	int degree_to_point = 20;
	float mid_left = FindValidDistance(tail_array + degree_to_point - 1, false);
	float mid_right = FindValidDistance(tail_array - degree_to_point + 1, true);
	float tail_space = (mid_left + mid_right) * cos(10 / RADIAN_TO_DEGREE) / 2;
	ROS_WARN("calc tail space: %f", tail_space);
	return tail_space;
}

/**
 * @brief 获取指定方向上左右两边夹角偏差
 * @param dirt - 指定方向角度; degree - 检测角度
 * @return 夹角距离偏差
 * */
float deskmedia_node::GetDeviceTiltOffsetValue(int dirt, int degree)
{
	int tail_array = dirt * 2 - 1;
	int degree_to_point = degree * 2;
	float mid_left = FindValidDistance(tail_array + degree_to_point, false);
	float mid_right = FindValidDistance(tail_array - degree_to_point, true);
	return (mid_left - mid_right);
}

/**
 * @brief 获取移动至水平状态的速度 
 * @param action - 执行速度结构体
 * @return 
 * */
bool deskmedia_node::GetMoveToVerticalAction(ACTION_DATA_S* action)
{
	static float last_w = 0;
	int ret = GetDeviceTiltDirection(20, false);
	if (ret == 0) {	
		action->w = 0;
	} else if (1 == ret) {
		action->w = -minw_;
	} else if (2 == ret) {
		action->w = minw_;
	} else {
		action->w = last_w;
		return false;
	}
	last_w = action->w;
	return true;
}

void deskmedia_node::GetMoveToVerticalSingleWheelAction(ACTION_DATA_S* action)
{
	int ret = GetDeviceTiltDirection(10);
	if (ret == 0) {	
		action->w = 0;
	} else if (1 == ret) {
		action->w = -minw_;
	} else if (2 == ret) {
		action->w = minw_;
	}
	action->v = -action->w * 0.287 / 2;
	
	//左轮不动，逆时针
//	float w = 0.2;
//	float v = w * 0.287 / 2;
//	PublishDMMotorCmd(v, w);

	//右轮不动，逆时针
//	float w = 0.2;
//	float v = -w * 0.287 / 2;
//	PublishDMMotorCmd(v, w);

	//左轮不动，顺时针
//	float w = -0.2;
//	float v = w * 0.287 / 2;
//	PublishDMMotorCmd(v, w);

	//右轮不动，顺时针
//	float w = -0.2;
//	float v = -w * 0.287 / 2;
//	PublishDMMotorCmd(v, w);
}

/**
 * @brief 获取红外对接信号的居中值 
 * @param 
 * @return true/false - 计算完成/计算未完成
 * */
bool deskmedia_node::GetCenteredDirection()
{
	static int count = 0;

	count++;
	g_dock_goal_.y = (g_left_edge_.y + g_right_edge_.y) / 2;
	g_dock_goal_.angle = (g_left_edge_.angle + g_right_edge_.angle) / 2 - 1;

	if (count >= 2) {
		count = 0;
		ROS_WARN("move to mid line: %f, %f", g_dock_goal_.y, g_dock_goal_.angle);
		return true;
	} else {
		return false;
	}
}

/**
 * @brief 判断odom pose 是否更新 
 * @param pose-记录点坐标 
 * @return true/false 已更新/未更新
 * */
bool deskmedia_node::IsUpdateOdomPose(geometry_msgs::Pose pose)
{
	if (g_pose_.pose.position.x != pose.position.x
	 || g_pose_.pose.position.y != pose.position.y
	 || g_pose_.pose.orientation.w != pose.orientation.w) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief 判断amcl pose 是否更新 
 * @param pose-记录点坐标 
 * @return true/false 已更新/未更新
 * */
bool deskmedia_node::IsUpdateAmclPose(geometry_msgs::Pose pose)
{
	if (g_amcl_pose_.pose.pose.position.x != pose.position.x
	 || g_amcl_pose_.pose.pose.position.y != pose.position.y
	 || g_amcl_pose_.pose.pose.orientation.w != pose.orientation.w) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief 判断odom comb pose 是否更新 
 * @param pose-记录点坐标 
 * @return true/false 已更新/未更新
 * */
bool deskmedia_node::IsUpdateOdomCombPose(geometry_msgs::Pose pose)
{
	// ROS_ERROR("g[%f, %f, %f], c[%f, %f, %f] %f", 
	// 	g_ekf_pose_.pose.pose.position.x, g_ekf_pose_.pose.pose.position.y, g_ekf_pose_.pose.pose.orientation.w,
	// 	pose.position.x,pose.position.y, pose.orientation.w, abs(g_ekf_pose_.pose.pose.orientation.w - pose.orientation.w));
	if ((abs(g_ekf_pose_.pose.pose.position.x - pose.position.x) > 0.001)
	 || (abs(g_ekf_pose_.pose.pose.position.y - pose.position.y) > 0.001)
	 || (abs(g_ekf_pose_.pose.pose.orientation.w - pose.orientation.w) > 0.001)) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief 判断carto pose 是否更新 
 * @param pose-记录点坐标 
 * @return true/false 已更新/未更新
 * */
bool deskmedia_node::IsUpdateCartographPose(geometry_msgs::Pose pose)
{
	return true;
}

/**
 * @brief 填充地图上点位数据 
 * @param map-地图数据指针 
 * 		  width*height-地图大小
 * 		  (sx, sy) - 填充起始点坐标
 * 		  (fx, fy) - 填充结束点坐标
 * @return 
 * */
void deskmedia_node::UpdateMapArea(unsigned char *map, int width, int height, int sx, int sy, int fx, int fy)
{
	if (fy >= sy) {
		for (unsigned int y = sy; y < fy; y++)
		{
//			for (unsigned int x = sx; x < fx; x++)
//			{
//				unsigned int i = x + y * width;
//				map[i] = 0x00;
//			} 
			if (sx < fx) {
				for (unsigned int x = sx; x < fx; x++)
				{
					unsigned int i = x + y * width;
					map[i] = 0x00;
				}
			} else {
				for (unsigned int x = sx; x >= fx; x--)
				{
					unsigned int i = x + y * width;
					map[i] = 0x00;
				}
			}
		}
	} else {
		for (unsigned int y = sy; y >= fy; y--)
		{
			if (sx < fx) {
				for (unsigned int x = sx; x < fx; x++)
				{
					unsigned int i = x + y * width;
					map[i] = 0x00;
				}
			} else {
				for (unsigned int x = sx; x >= fx; x--)
				{
					unsigned int i = x + y * width;
					map[i] = 0x00;
				}
			}
		}
	}
}

/**
 * @brief 检查直线移动是否完成 
 * @param c - 起点 
 * 		  g - 终点
 * 		  t - 目标距离
 * @return true/false 结束/未结束
 * */
bool deskmedia_node::CheckMoveFinish(geometry_msgs::Point c, geometry_msgs::Point g, float t)
{
	float  distance=sqrt((g.x - c.x)*(g.x - c.x)+(g.y - c.y)*(g.y - c.y));
	if (distance- abs(t) > 0.001) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief 查找拐点特征值，从起始角度扫描90° 
 * @param from - 起始点角度 
 * 		  order - true/false 顺序/倒序
 * @return 拐点索引
 * */
int deskmedia_node::FindTurningPoint(int from, bool order, float *distance)
{
	int start = 0;
	int finish = 0;
	int degree = 0;
	float last_degree_distance = 0.0;

	if (order) {
		start = from * 2 - 1;
		finish = start + 180;
		last_degree_distance = g_ranges_[start];
		for (int i = start; i <= finish; i++) {
//			ROS_WARN("[%d]-%f", i, g_ranges_[i]);
			if (isinf(g_ranges_[i]))
				continue;
			if (abs(last_degree_distance - g_ranges_[i]) > 0.15) {
//				ROS_WARN("get turning point:{last:%f, %f, now:%f }", last_degree_distance, g_ranges_[i - 1], g_ranges_[i]);
				*distance = last_degree_distance;
				return i - 1;
			}
			last_degree_distance = g_ranges_[i];
		}
	} else {
		start = from * 2 - 1;
		finish = start - 180;
		last_degree_distance = g_ranges_[start];
		for (int i = start; i >= finish; i--) {
//			ROS_WARN("[%d]-%f", i, g_ranges_[i]);
			if (isinf(g_ranges_[i]))
				continue;
			if (abs(last_degree_distance - g_ranges_[i]) > 0.15) {
//				ROS_WARN("get turning point:{last:%f, %f, now:%f }", last_degree_distance, g_ranges_[i + 1], g_ranges_[i]);
				*distance = last_degree_distance;
				return i + 1;
			}
			last_degree_distance = g_ranges_[i];
		}
	}
	
	return -1;
}

/**
 * @brief 查找有效距离数据 
 * @param index - 起始索引 
 * 		  order - true/false 顺序/倒序
 * @return 索引对应的
 * */
float deskmedia_node::FindValidDistance(int index, bool order)
{
	do {
		if (!isinf(g_ranges_[index]))
			return g_ranges_[index];
	}while(order ? (index++) : (index--));
}

/**
 * @brief 查找对接特征值 
 * @param 
 * @return true/false 匹配/未匹配
 * */
bool deskmedia_node::FindDockingFeature()
{
	int left_point = 0;
	int right_point = 0;
	float left_distance = 0;
	float right_distance = 0;

	left_point = FindTurningPoint(180, true, &left_distance);
	right_point = FindTurningPoint(180, false, &right_distance);

	// ROS_WARN("%d, %d, %f, %f", 
	// 	left_point, right_point, 
	// 	g_ranges_[left_point], g_ranges_[right_point]);

	float *nearest = std::min_element(&g_ranges_[right_point], &g_ranges_[left_point]);
	float feature = sqrt(pow(left_distance, 2) - pow(*nearest, 2)) + sqrt(pow(right_distance, 2) - pow(*nearest, 2));
	float mid_to_left = sqrt(pow(left_distance, 2) - pow(*nearest, 2));
	float mid_to_right = sqrt(pow(right_distance, 2) - pow(*nearest, 2));
	ROS_WARN("l: %f, r: %f, v: %f, f: %f (mtl: %f, mtr: %f)", left_distance, right_distance, *nearest, feature, mid_to_left, mid_to_right);
	// ROS_WARN("feature distance: %f", sqrt(pow(left_distance, 2) - pow(*nearest, 2)) + sqrt(pow(right_distance, 2) - pow(*nearest, 2)));

	// ROS_WARN("%d, %d, %f, %f, %f, %f", 
	// 	left_point, right_point, 
	// 	g_ranges_[left_point << 1], g_ranges_[right_point << 1], *nearest,
	// 	sqrt(pow(g_ranges_[left_point << 1], 2) - pow(*nearest, 2)) + sqrt(pow(g_ranges_[right_point << 1], 2) - pow(*nearest, 2)));
	if (feature >= 0.36 && feature <= 0.50)
		return true;
	else
		return false;
}

/**
 * @brief 判断设备是否在对接范围内
 * @param offset - 距离中心偏差
 * @return true/false - 范围内/范围外
 * */
bool deskmedia_node::IsDeviceInArea(float *mid_offset, bool *side, float *vert_offset, float *hori_offset)
{
	int left_point = 0;
	int right_point = 0;
	float left_distance = 0;
	float right_distance = 0;
	float long_hypotenuse = 0;
	float short_hypotenuse = 0;

	left_point = FindTurningPoint(180, true, &left_distance);
	right_point = FindTurningPoint(180, false, &right_distance);

	if (left_distance >= right_distance) {
		long_hypotenuse = left_distance;
		short_hypotenuse = right_distance;
	} else {
		long_hypotenuse = right_distance;
		short_hypotenuse = left_distance;
	}

	// ROS_WARN("%d, %d, %f, %f", 
	// 	left_point, right_point, 
	// 	g_ranges_[left_point], g_ranges_[right_point]);

	float *nearest = std::min_element(&g_ranges_[right_point], &g_ranges_[left_point]);
	if ((abs(left_distance - *nearest) < 0.015) || (abs(right_distance - *nearest) < 0.015)) {
		*vert_offset = (pow(long_hypotenuse, 2) - pow(short_hypotenuse, 2) - pow(DOCKING_FEATRUE, 2)) / (DOCKING_FEATRUE * 2);
		if (*vert_offset < 0) {
			*hori_offset = short_hypotenuse;
		} else {
			*hori_offset = sqrt(pow(short_hypotenuse, 2) - pow(*vert_offset, 2));
		}
		ROS_WARN("device vert-offset: %f, hori_offset: %f", *vert_offset, *hori_offset);
		if ((abs(left_distance - *nearest) < 0.015))
			*side = true; //in left
		else
			*side = false; //in right
		return false;
	}
	
	float feature = sqrt(pow(left_distance, 2) - pow(*nearest, 2)) + sqrt(pow(right_distance, 2) - pow(*nearest, 2));
	float mid_to_left = sqrt(pow(left_distance, 2) - pow(*nearest, 2));
	float mid_to_right = sqrt(pow(right_distance, 2) - pow(*nearest, 2));
	ROS_WARN("l: %f, r: %f, v: %f, f: %f (mtl: %f, mtr: %f)", left_distance, right_distance, *nearest, feature, mid_to_left, mid_to_right);
	*mid_offset = (mid_to_left - mid_to_right);
	
	return true;
}


/**
 * @brief 计算设备离对接中心的偏差
 * @param 
 * @return 实际偏差的距离
 * */
float deskmedia_node::FindCenterOffset()
{
	int left_point = 0;
	int right_point = 0;
	float left_distance = 0;
	float right_distance = 0;

	left_point = FindTurningPoint(180, true, &left_distance);
	right_point = FindTurningPoint(180, false, &right_distance);

	// ROS_WARN("%d, %d, %f, %f", 
	// 	left_point, right_point, 
	// 	g_ranges_[left_point], g_ranges_[right_point]);

	float *nearest = std::min_element(&g_ranges_[right_point], &g_ranges_[left_point]);
	float feature = sqrt(pow(left_distance, 2) - pow(*nearest, 2)) + sqrt(pow(right_distance, 2) - pow(*nearest, 2));
	float mid_to_left = sqrt(pow(left_distance, 2) - pow(*nearest, 2));
	float mid_to_right = sqrt(pow(right_distance, 2) - pow(*nearest, 2));
	ROS_WARN("l: %f, r: %f, v: %f, f: %f (mtl: %f, mtr: %f)", left_distance, right_distance, *nearest, feature, mid_to_left, mid_to_right);
	return (mid_to_left - mid_to_right);
}

/**
 * @brief 获取对接前的位姿及状态、趋势等信息
 * @param 
 * @return 实际偏差的距离
 * */
int deskmedia_node::FindDockingLocation()
{
	int direction = 0;
	bool side = false;
	bool in_area = true;
	float mid_offset = 0;
	float vert_offset = 0;
	float hori_offset = 0;
	ROS_WARN("----------------------------------------------------------------------------------");

	//基础条件
	ROS_WARN("docking leds signal: 0x%02x - (%d, %d, %d)", g_infrared_, g_infrared_ & 0x01, (g_infrared_ & 0x02) >> 1, (g_infrared_ & 0x04) >> 2);
	if (!FindDockingFeature())
		ROS_ERROR("can not find docking feature");

	//位置信息
	in_area = IsDeviceInArea(&mid_offset, &side, &vert_offset, &hori_offset);
	if(in_area) {
		if (abs(mid_offset) <= 0.01)
			ROS_WARN("device just in middle");
		else
			ROS_WARN("device offset to middle near %s, %f", (mid_offset < 0) ? "left" : "right", mid_offset);

		direction = GetDeviceTiltDirection(10);
		if (!direction)
			ROS_WARN("device vertical");
		else
			ROS_WARN("device face to %s", (direction == 1) ? "left" : "right");
	} else {
		//TODO 外部的就近特征判断
		ROS_WARN("out side of docking area, v: %f, h: %f", vert_offset, hori_offset);
	}

	if (vert_offset < 0)
		back_target_ = (DOCKING_FEATRUE / 2) / sqrt(pow(tail_space_, 2) - pow(hori_offset, 2)) * tail_space_ - vert_offset;
	else
		back_target_ = (DOCKING_FEATRUE / 2) / sqrt(pow(tail_space_, 2) - pow(hori_offset, 2)) * tail_space_;
	float angle = atan(hori_offset / (DOCKING_FEATRUE + vert_offset)) * RADIAN_TO_DEGREE;
	if (angle > 62)
		ROS_WARN("can docking with once");
	//((pow(vert_offset, 2) + pow(tail_space_, 2) - pow(hori_offset, 2)) / (2 * vert_offset * tail_space_));
	ROS_WARN("tail left space: %f, target: %f, angle: %f", tail_space_, back_target_, angle);
	ROS_WARN("----------------------------------------------------------------------------------");

	if ((direction == 1) && mid_offset > 0) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * @brief 校验设备针对对接端是否居中 
 * @param 
 * @return true/false 居中/未居中
 * */
bool deskmedia_node::IsDockingCentered()
{
	return false;
}

/**
 * @brief 通过角度判断执行动作是否需要结束 
 * @param goal - 目标角度
 * 		  curt - 当前角度
 * @return 0/1/2 完成/小幅偏差/大幅偏离
 * */
int  deskmedia_node::IsMoveFinishByAngle(float goal, float curt)
{
	if (abs(curt - goal) < 0.5)
		return 0;
	else if (abs(curt - goal) > 5)
		return 2;
	return 1;
}

/**
 * @brief 通过距离判断执行动作是否需要结束 
 * @param goal - 目标位置
 * 		  curt - 当前位置
 * @return true/false 完成/未完成
 * */
bool deskmedia_node::IsMoveFinishByDistance(geometry_msgs::Pose goal, geometry_msgs::Pose curt)
{
	return false;
}

/**
 * @brief 通过pose判断设备是否移动 
 * @param s - 起始位置
 *		  s_yaw - 起始角度
 * 		  c - 当前坐标
 * 		  c_yaw - 当前角度
 * @return true/false 已移动/未移动
 * */
bool deskmedia_node::IsDeviceMoved(geometry_msgs::PoseWithCovarianceStamped s, float s_yaw, geometry_msgs::PoseWithCovarianceStamped c, float c_yaw)
{
	if ((abs(s.pose.pose.position.x - c.pose.pose.position.x) > 0.01)
	 || (abs(s.pose.pose.position.y - c.pose.pose.position.y) > 0.01)
	 || (abs(s_yaw - c_yaw) > 0.01)) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief 进行坐标变换后的位置获取
 * @param global_pose - 里程计坐标
 * @return true/false 正常/异常
 * */
bool deskmedia_node::GetDevicePose(geometry_msgs::PoseStamped& global_pose)
{
	tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
	robot_pose.header.frame_id = "base_link"; 
  	robot_pose.header.stamp = ros::Time();
  	ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, "map");
    }
	catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

//	ROS_WARN("tf_pose(%f, %f, %f)", global_pose.pose.position.x, global_pose.pose.position.y, global_pose.pose.position.z);

    return true;
}

/**
 * @brief 二次定位对接初始化处理，调整角度和距离 
 * @param action - 反馈计算出的v和w
 * @return true/false 正常/异常
 * */
bool deskmedia_node::DockingTaskInit(ACTION_DATA_S* action)
{
	action->v = 0;
	action->w = 0;
	#if INFRARED_MODE
	if (0x0d != g_infrared_) {
		if (0x08 == g_infrared_ || 0x0c == g_infrared_) {
			action->w = -0.20;
		} else if (0x01 == g_infrared_ || 0x05 == g_infrared_) {
			action->w = 0.20;
		}
	} else {
		GetMoveToVerticalAction(action);
		if (0 == action->w) {
			if(!isinf(tail_space_)) {
//				float offset = FindCenterOffset();
//				ROS_WARN("center offset: %f, v = %f, w = %f", offset, action->v, action->w);
				if (tail_space_ < (0.24 + LOCATION_SPACE)) { // 0.265
					action->v = 0.1;
	//				action->w = offset * 10;
				} else if (tail_space_ > (0.35 + LOCATION_SPACE)) { //0.305
					action->v = -0.05;
	//				action->w = offset * 10;
				} else {
					GetMoveToVerticalAction(action);
					if (0 == action->w) {
						g_docking_status_ = DOCKING_FIND_LEFT;
					}
				}
			}
		}
		
//		if(!isinf(tail_space_)) {
//			float offset = FindCenterOffset();
//			ROS_WARN("center offset: %f", offset);
//			if (tail_space_ < (0.24 + LOCATION_SPACE)) { // 0.265
//				action->v = 0.1;
////				action->w = offset * 10;
//			} else if (tail_space_ > (0.35 + LOCATION_SPACE)) { //0.305
//				action->v = -0.05;
////				action->w = offset * 10;
//			} else {
////				GetMoveToVerticalAction(action);
////				if (0 == action->w) {
////					g_docking_status_ = DOCKING_FINISH;//DOCKING_FIND_LEFT;
////				}
//			}
//		}
	}
	#else
	if (0x04 != g_infrared_) {
		ROS_WARN("Do not match docking condition! - 1");
	} else {
		if(!isinf(tail_space_)) {
			float offset = FindCenterOffset();
			ROS_WARN("center offset: %f", offset);
			if (tail_space_ < (0.24 + LOCATION_SPACE)) { // 0.265
				action->v = 0.1;
			} else if (tail_space_ > (0.35 + LOCATION_SPACE)) { //0.305
				action->v = -0.05;
			} else {
				GetMoveToVerticalAction(action);
				if (0 == action->w) {
					g_docking_status_ = DOCKING_FIND_LEFT;
				}
			}
		}
	}
	#endif
	// ROS_WARN("action v: %f, w: %f", action->v, action->w);
	return true;
}

/**
 * @brief 二次定位，后退对接
 * @param action - 反馈计算出的v和w
 * @return true/false 正常/异常
 * */
bool deskmedia_node::DockingGoBackAction(ACTION_DATA_S* action)
{
	action->v = 0;
	action->w = 0;
	static int count = 0;
	static float last_yaw = 0.0;
	static float last_space = 0.0;
	static geometry_msgs::PoseWithCovarianceStamped last_pose;

	if (!g_docking_) {
		if (count >= 40) {
			count = 0;
			g_docking_status_ = DOCKING_INIT;
			return false;
		}

		action->v = -0.1;
		action->w = (g_dock_goal_.angle - g_odom_yaw_ * RADIAN_TO_DEGREE) / docking_factor_;

		if (IsDeviceMoved(last_pose, last_yaw, g_ekf_pose_, g_odom_yaw_ * RADIAN_TO_DEGREE)) {
			if (tail_space_ < 0.47) {
				if (abs(last_space - tail_space_) < 0.004) {
					if (count >= 30)
						ROS_WARN("device has stucked![%f]-[%f](%d)", last_space, tail_space_, count);
					if (tail_space_ > last_space) {
						last_space = tail_space_;
						count = 0;
					}
					count++;
					return true;
				} else {
					last_space = tail_space_;
					count = 0;
				}
			} else {
				last_pose.pose.pose.position.x = g_ekf_pose_.pose.pose.position.x;
				last_pose.pose.pose.position.y = g_ekf_pose_.pose.pose.position.y;
				last_yaw = g_odom_yaw_ * RADIAN_TO_DEGREE;
				count = 0;
			}
		} else {
			count++;
			return true;
		}
	} else {
		g_docking_status_ = DOCKING_INIT;
	}
	
	return true;
}

/**
 * @brief 二次定位，查找对接信号中心 
 * @param action - 反馈计算出的v和w
 * @return true/false 正常/异常
 * */
bool deskmedia_node::DockingFindMiddleAction(ACTION_DATA_S* action)
{
	action->v = 0;
	action->w = 0;
	ROS_WARN("target: %f, current:%f", g_dock_goal_.angle, g_odom_yaw_ * RADIAN_TO_DEGREE);
	int ret = IsMoveFinishByAngle(g_dock_goal_.angle, g_odom_yaw_ * RADIAN_TO_DEGREE);
	if (!ret) {
		g_docking_status_ = DOCKING_GO_BACK;
	} else {
		if (1 == ret) {
			action->w = -0.15;
		} else {
			action->w = -0.25;
		}
	}
	return true;
}

/**
 * @brief 二次定位，查找左侧边界 
 * @param action - 反馈计算出的v和w
 * @return true/false 正常/异常
 * */
bool deskmedia_node::DockingFindLeftEdgeAction(ACTION_DATA_S* action)
{
	#if INFRARED_MODE
	if (g_infrared_ == 0x05) {
		ROS_WARN("left edge [%f, %f, %f]", g_ekf_pose_.pose.pose.position.x, g_ekf_pose_.pose.pose.position.y, g_odom_yaw_ * RADIAN_TO_DEGREE);
		g_left_edge_.x = g_ekf_pose_.pose.pose.position.x;
		g_left_edge_.y = g_ekf_pose_.pose.pose.position.y;
		g_left_edge_.angle = g_odom_yaw_ * RADIAN_TO_DEGREE;
		g_docking_status_ = DOCKING_FIND_RIGHT;
		action->v = 0;
		action->w = 0;
	} else {
		action->v = -0.04;
		action->w = -0.25;
	}
	#else
	if (g_infrared_ == 0x00) {
		ROS_WARN("left edge [%f, %f, %f]", g_ekf_pose_.pose.pose.position.x, g_ekf_pose_.pose.pose.position.y, g_odom_yaw_ * RADIAN_TO_DEGREE);
		g_left_edge_.x = g_ekf_pose_.pose.pose.position.x;
		g_left_edge_.y = g_ekf_pose_.pose.pose.position.y;
		g_left_edge_.angle = g_odom_yaw_ * RADIAN_TO_DEGREE;
		g_last_docking_status_ = DOCKING_FIND_LEFT;
		g_docking_status_ = DOCKING_FIND_SIGNAL;
		action->v = 0;
		action->w = 0;
	} else {
		action->v = -0.04;
		action->w = -0.25;
	}
	#endif
	return true;						
}

/**
 * @brief 二次定位，查找右侧边界 
 * @param action - 反馈计算出的v和w
 * @return true/false 正常/异常
 * */
bool deskmedia_node::DockingFindRightEdgeAction(ACTION_DATA_S* action)
{
	#if INFRARED_MODE
	if (g_infrared_ == 0x0C) {
		ROS_WARN("right edge [%f, %f, %f]", g_ekf_pose_.pose.pose.position.x, g_ekf_pose_.pose.pose.position.y, g_odom_yaw_ * RADIAN_TO_DEGREE);
		g_right_edge_.x = g_ekf_pose_.pose.pose.position.x;
		g_right_edge_.y = g_ekf_pose_.pose.pose.position.y;
		g_right_edge_.angle = g_odom_yaw_ * RADIAN_TO_DEGREE;
		if (GetCenteredDirection())
			g_docking_status_ = DOCKING_FIND_MID;
		else
			g_docking_status_ = DOCKING_FIND_LEFT;
		action->v = 0;
		action->w = 0;	
	} else {
		action->v = -0.04;
		action->w = 0.25;
	}
	#else
	if (g_infrared_ == 0x00) {
		ROS_WARN("right edge [%f, %f, %f]", g_ekf_pose_.pose.pose.position.x, g_ekf_pose_.pose.pose.position.y, g_odom_yaw_ * RADIAN_TO_DEGREE);
		g_right_edge_.x = g_ekf_pose_.pose.pose.position.x;
		g_right_edge_.y = g_ekf_pose_.pose.pose.position.y;
		g_right_edge_.angle = g_odom_yaw_ * RADIAN_TO_DEGREE;
		g_last_docking_status_ = DOCKING_FIND_RIGHT;
		if (GetCenteredDirection())
			g_docking_status_ = DOCKING_FIND_MID;
		else
			g_docking_status_ = DOCKING_FIND_SIGNAL;
		action->v = 0;
		action->w = 0;	
	} else {
		action->v = -0.04;
		action->w = 0.25;
	}
	#endif
	return true;						
}

/**
 * @brief 二次定位，找回信号 
 * @param action - 反馈计算出的v和w
 * @return true/false 正常/异常
 * */
bool deskmedia_node::DockingFindSignalAction(ACTION_DATA_S* action)
{
	if (g_infrared_ == 0x04) {
		if (g_last_docking_status_ == DOCKING_FIND_LEFT) {
			g_docking_status_ = DOCKING_FIND_RIGHT;
		} else if (g_last_docking_status_ == DOCKING_FIND_RIGHT) {
			g_docking_status_ = DOCKING_FIND_LEFT;
		} 
		action->v = 0;
		action->w = 0;
	} else {
		if (g_last_docking_status_ == DOCKING_FIND_LEFT) {
			action->v = -0.04;
			action->w = 0.25;
		} else if (g_last_docking_status_ == DOCKING_FIND_RIGHT) {
			action->v = -0.04;
			action->w = -0.25;
		} else {
			action->v = 0;
			action->w = 0;
		}
	}
	return true;						
}


/**
 * @brief 二次定位对接入口 
 * @param action - 反馈计算出的v和w
 * @return true/false 正常/异常
 * */
bool deskmedia_node::DockingTaskAction(ACTION_DATA_S* action)
{
	static int last_signal = 0;
	static int last_action_type = 0;
	if (last_action_type != g_docking_status_) {
		last_action_type = g_docking_status_;
		ROS_WARN("action type(%d)", g_docking_status_);
	}

	#if INFRARED_MODE
	switch (g_docking_status_) 
	{
		case DOCKING_INIT:
			return DockingTaskInit(action);
		break;
		case DOCKING_GO_BACK:
			return DockingGoBackAction(action);
		break;
		case DOCKING_FIND_MID:
			return DockingFindMiddleAction(action);
		break;
		case DOCKING_FIND_LEFT:
			return DockingFindLeftEdgeAction(action);
		break;
		case DOCKING_FIND_RIGHT:
			return DockingFindRightEdgeAction(action);
		break;
		case DOCKING_FIND_SIGNAL:
			return DockingFindSignalAction(action);
		break;
		case DOCKING_FINISH:
			g_docking_ = 1;
		break;
		default:
			ROS_WARN("undefined action type(%d)", g_docking_status_);
			return false;
	}
	#else
	//if (0x04 != g_infrared_) {

	//} else {
		if(!isinf(tail_space_)) {
			float offset = FindCenterOffset();
			ROS_WARN("center offset: %f, signal:%d", offset, g_infrared_);
			if (tail_space_ > (0.10 + LOCATION_SPACE)) { // 0.265
				action->v = -0.03;
				if (abs(offset) > 0.01) {
					if (abs(offset * 10 > maxw_)) {
						action->w = -(maxw_ * (offset/abs(offset)));
					} else {
						if (abs(offset) < minw_)
							action->w = -(minw_ * (offset/abs(offset)));
						else
							action->w = -(offset);
					}
				}
			} else {
				GetMoveToVerticalAction(action);
				if (0 == action->w) {
					if (0x04 == g_infrared_)
						action->v = -0.1;
					else
						action->v = 0;
				} 
			}
		}
	//}
	#endif
	return true;
}


bool deskmedia_node::DockingTaskActionEx(ACTION_DATA_S* action)
{
	int ret = 0;
	int index = 0;
	action->v = 0;
	action->w = 0;
	static int count = 0;
	static int count2 = 0;
	static int target = 0;
	static int condition = 0;
	static int oscillation = 0;
	static int action_type = 1;
	static int last_signal = 0;
	static int last_action = 0; //0;1-v;2-w
	static int last_action_type = 0;
	static bool wait_flag = false;
	static float last_v = 0;
	static float last_w = 0;
	static float last_yaw = 0;
	static float last_space = 0;
	static geometry_msgs::PoseWithCovarianceStamped last_pose;

	if (last_action_type != g_action_type_) {//显示步骤
		ROS_ERROR("action type changed: %d -> %d", last_action_type, g_action_type_);
		last_action_type = g_action_type_;
	}

	if (wait_flag) { //反馈速度大于EMB_FB_DATA_RANGE
		ROS_WARN("wait for stop: x:%f, z:%f", last_x_fb_, last_z_fb_);
		if (abs(last_x_fb_) > EMB_FB_DATA_RANGE || abs(last_z_fb_) > EMB_FB_DATA_RANGE) {
			return true;
		} else {
			wait_flag = false;
			last_action = 0;
		}
	}

	if (abs(tail_space_ - last_space) < 0.005) {
		last_space = tail_space_;
		oscillation = 0;
	} else {
		if (DOCKING_DOCKING != g_action_type_) {
			oscillation++;
			if (oscillation >= 60) { //出现震荡让倒退一点
				ROS_ERROR("in docking mid device has trucked!");
				if (tail_space_ > 0.4)
					MoveTargetDistanceBackford(0.02);
				else
					MoveTargetDistanceBackford(0.01);
				oscillation = 0;
			}
		}
	}
	
	switch(g_action_type_)
	{
		case DOCKING_READY:  //对接准备
			if (g_docking_times_++ >= TRY_TIMES) {
				g_docking_times_ = 0;
				return false;
			}
			ROS_WARN("start %d times docking", g_docking_times_);
			
			if((g_infrared_ !=IR_NONE)&& tail_space_ > 0.5)//有信号尾部距离足够
			{
					g_action_type_ = DOCKING_BACK;
			}
			else if((g_infrared_ !=IR_NONE) && tail_space_ <= 0.5)//有信号尾部距离不足
			{
				MoveTargetDistanceForward(0.8-tail_space_);
			}
			else 
			{				
				if (!DockingTaskFindSignalAction(action)) {//TODO查找信号
					ROS_WARN("can not find docking signal!");
					PublishErrorCmd("无法找到对接信号");
					return false;
				} else {
					MoveTargetDistanceForward(0.3);
				}
			}
			break;
		case DOCKING_BACK: //后退对接
			if (tail_space_ <= 0.28) { //TODO 距离有效且信号全1且左右水平
				ROS_ERROR("space: %f, signal: %02x", tail_space_, g_infrared_);
				if (g_infrared_ == IR_ALL || g_infrared_ == IR_MID || g_infrared_ == IR_SIDE)
					g_action_type_ = DOCKING_WATI;
				else {
					if (g_infrared_ == IR_LEFT || g_infrared_ == IR_MMID_LEFT) {
						action->w = -dock_w_;
					} else if (g_infrared_ == IR_RIGHT || g_infrared_ == IR_MID_RIGHT) {
						action->w = dock_w_;
					} else if (g_infrared_ == IR_NONE) {
						
					}
				}
			} else {
				if (g_infrared_ == IR_ALL || g_infrared_ == IR_MID || g_infrared_ == IR_SIDE) {
					action->v = -dock_v_;
				} else if (g_infrared_ == IR_LEFT || g_infrared_ == IR_MMID_LEFT) {
					action->w = -dock_w_;
				} else if (g_infrared_ == IR_RIGHT || g_infrared_ == IR_MID_RIGHT) {
					action->w = dock_w_;
				} else if (g_infrared_ == IR_NONE) {
					action->v = last_v;
					action->w = last_w;
				}
			}
			if ((1 == last_action && 0 != action->w) || (2 == last_action && 0 != action->v)) {
				ROS_WARN("action type has changed! wait for stop");
				wait_flag = true;
			}
			ROS_WARN("tail(%.3f), g_infrared_(%d), pub cmd vel: %.3f(%.3f), %.3f(%.3f) -- %d, %d", 
				tail_space_, g_infrared_, action->v, last_x_fb_, action->w, last_z_fb_,
				g_action_type_, oscillation);
			last_v = action->v;
			last_w = action->w;
			if (action->v != 0)
				last_action = 1;
			else if (action->w != 0)
				last_action = 2;

			if (wait_flag) {
				action->v = 0;
				action->w = 0;
			}
			
			break;
		case DOCKING_WATI:
			if (count2 < 3) {
				count2++; 
			} else {
				count = 0;
				count2 = 0;
				g_action_type_ = DOCKING_DOCKING;
			}
			break;
		case DOCKING_DOCKING:
			if (tail_space_ != last_space) {
				last_space = tail_space_;
				action->v = -dock_v_;
			} else {
				count++;
				if (count >= 50) {
					ROS_WARN("device has trucked!");
					g_action_type_ = DOCKING_AHEAD;
					action->v = 0;
					count = 0;
				}				
				action->v = -dock_v_;
			}
			ROS_WARN("%.3f, g_infrared_(%d), pub cmd vel: %.3f(%.3f), %.3f(%.3f)", tail_space_, g_infrared_, action->v, last_x_fb_, action->w, last_z_fb_);
			break;
		case DOCKING_AHEAD: //出来重新对接
			// MoveTargetDistanceForward(0.45);
			// g_action_type_ = DOCKING_READY;
			// break;
			return false; //直接结束--测试用
			break;
		default:
			ROS_WARN("undefined action type(%d)", action_type);
			break;
	}
	
	return true;
}
bool deskmedia_node::DockingTaskFindSignalAction(ACTION_DATA_S* action)
{
	ros::Rate r(20);
	ros::NodeHandle n;
	bool finish = false;
	float target = 3.141593*2;
	geometry_msgs::Pose t_pose;
	t_pose.position.x = g_pose_.pose.position.x;
	t_pose.position.y = g_pose_.pose.position.y;
	t_pose.orientation.w = g_pose_.pose.orientation.w;

	float t_yaw = QuaternionTransToYaw(g_pose_.pose.orientation);//起始角度t_yaw 当前角度g_yaw 目标角度goal->target
	float turn_angle = 0;//累计旋转角度
	while(n.ok() && (IR_NONE == g_infrared_)) {
		float delta_angle=0;//单周期运动角度
		if( target > 0) {
			if(g_odom_yaw_ >= t_yaw)
				delta_angle = g_odom_yaw_- t_yaw; 
			else
				delta_angle = g_odom_yaw_+ 2 * PI - t_yaw;  //PI--->(-PI)
		} else if (target < 0) {
			if(g_odom_yaw_ <= t_yaw)
				delta_angle = t_yaw - g_odom_yaw_; 
			else
				delta_angle = t_yaw - g_odom_yaw_+2*PI;	//(-PI)--->PI
		}
		t_yaw = g_odom_yaw_;//重新赋值
		turn_angle += delta_angle;//累计旋转角度
		ROS_WARN("target_angle: %f, turn_angle: %f", target, turn_angle);
		if(turn_angle - abs(target) < 0.0001){
			PublishDMMotorCmd(0, 0.15,true);
		} else {
			PublishDMMotorCmd(0, 0,true);
			return false;
			break;
		}

		ros::spinOnce();
		r.sleep();
	}
	
	return true;
}

/**
 * @brief 发布地图上的目标位置 
 * @param (x, y) - 坐标 
 * 		  z-角度
 * @return 
 * */
void deskmedia_node::PublishGoalCmd(float x, float y, float z)
{
	geometry_msgs::PoseStamped goal;

	goal.header.frame_id = "map";
	goal.pose.position.x = x;
	goal.pose.position.y = y;
	goal.pose.orientation.z = sin(z / 2);
	goal.pose.orientation.w = cos(z / 2);

	pub_goal_.publish(goal);
}

/**
 * @brief 发布导航任务取消指令
 * @param 
 * @return 
 * */
void deskmedia_node::PublishCancelGoalCmd()
{
	actionlib_msgs::GoalID goal;
	goal.id = "";
	pub_cancel_.publish(goal);
}

/**
 * @brief 发布需要执行的速度到采集板 
 * @param v-线速度 
 * 		  w-角速度
 * @return 
 * */
void deskmedia_node::PublishMotorCmd(float v, float w)
{
	geometry_msgs::Twist speed;
	
	speed.linear.x = v;
	speed.angular.z = w;
	cmd_vel_.publish(speed);	
}

/**
 * @brief 二次封装后的速度发布，避免控制时底盘碰撞 
 * @param v-线速度 
 * 		  w-角速度
 * 		  check_tail_space_-是否检测尾部距离
 * @return true/false 可以发布/禁止发布
 * */
bool deskmedia_node::PublishDMMotorCmd(float v, float w,bool check_tail_space_ )
{
	#if 1
	if (0.0 == v) {
		PublishMotorCmd(0, w);
	} else {
		if (v > 0) {
			float *c_range1 = std::min_element(&g_ranges_[0], &g_ranges_[59]);
			float *c_range2 = std::min_element(&g_ranges_[660], &g_ranges_[719]);
			float nearleast = *c_range1 < *c_range2 ? *c_range1 : *c_range2;
			// ROS_WARN("f: %f, t: %f, l: %f", g_ranges_[0], g_ranges_[359], nearleast);
			if (nearleast <= 0.20) {
				PublishMotorCmd(0, 0);
				return false;
			} else {
				PublishMotorCmd(v, w);
			}
		} else {
//			float *c_range = std::min_element(&g_ranges_[330], &g_ranges_[390]); //0.349
//			ROS_WARN("f: %f, t: %f, l: %f", g_ranges_[0], g_ranges_[359], *c_range);
			if (tail_space_ <= 0.365 && check_tail_space_==true) {
				PublishMotorCmd(0, 0);
				return false;
			} else {
				PublishMotorCmd(v, w);
			}
		}
	} 
	#else
	if (0.0 == v) {
		PublishMotorCmd(0, w);
	} else {
		if (v > 0) {
			float c_range1 = GetObstacleToBaselinkDistance(0,59);;
			float c_range2 = GetObstacleToBaselinkDistance(660,719);;
			float nearleast = c_range1 < c_range2 ? c_range1 : c_range2;
			if (nearleast <= 0.20) {
				PublishMotorCmd(0, 0);
				return false;
			} else {
				PublishMotorCmd(v, w);
			}
		} else {
			float c_range = GetObstacleToBaselinkDistance(330,390);
			if (c_range <= 0.349) {
				PublishMotorCmd(0, 0);
				return false;
			} else {
				PublishMotorCmd(v, w);
			}
		}
	} 
	#endif

	return true;
}

/**
 * @brief 发布地图更新信息 
 * @param map-地图描述数据 
 * 		  index-当前可以获取的地图索引
 * @return 
 * */
void deskmedia_node::PublishMapInfo(nav_msgs::MapMetaData map, int index)
{
	deskmedia::DMMap map_info;
	
	map_info.resolution = map.resolution;
	map_info.width = map.width;
	map_info.height = map.height;
	map_info.origin_x = map.origin.position.x;
	map_info.origin_y = map.origin.position.y;
	map_info.index = index;

	float yaw = QuaternionTransToYaw(map.origin.orientation);
	map_info.origin_z = yaw;

	map_info.car_x = g_pose_.pose.position.x;
	map_info.car_y = g_pose_.pose.position.y;
	map_info.car_z = yaw * RADIAN_TO_DEGREE;

	pub_map_.publish(map_info);
}

/**
 * @brief 发布设备所在位置信息
 * @param location-当前所在位置
 * @return 
 * */
void deskmedia_node::PublishLocationCmd(int location)
{
	std_msgs::UInt8 location_msgs;
    
    location_msgs.data = location; 
    pub_location_.publish(location_msgs); 
}

/**
 * @brief 发布红外对接信号 
 * @param infrared-红外对接信号状态值
 * @return 
 * */
void deskmedia_node::PublishInfraredCmd(int infrared)
{
	std_msgs::UInt8 infrared_msgs;
    
    infrared_msgs.data = infrared; 
    pub_infrared_.publish(infrared_msgs); 
}

/**
 * @brief 发布位置信息 
 * @param (x, y) - 坐标
 * 		  a - 角度
 * @return 
 * */
void deskmedia_node::PublishPoseCmd(float x, float y, float a)
{
	deskmedia::DMPose pose;
	
	pose.x = x;
	pose.y = y;
	pose.angle = a;
	pub_pose_.publish(pose);	
}

/**
 * @brief 发布命令执行反馈信息 
 * @param msgs-反馈信息描述字符串 
 * @return 
 * */
void deskmedia_node::PublishMsgsCmd(const char *msgs)
{
	std_msgs::String msg;
    std::stringstream msgStr;
    msgStr << msgs;
    msg.data = msgStr.str();
    pub_msgs_.publish(msg);
}

/**
 * @brief 发布错误信息 
 * @param msgs-异常状态描述字符串 
 * @return 
 * */
void deskmedia_node::PublishErrorCmd(const char *msgs)
{
	std_msgs::String msg;
    std::stringstream msgStr;
    msgStr << msgs;
    msg.data = msgStr.str();
    pub_error_.publish(msg);
}

/**
 * @brief 发布清除缓存指令 
 * @param msgs-空 
 * @return 
 * */
void deskmedia_node::PublishClearCmd(const char *msgs)
{
	std_msgs::String msg;
    std::stringstream msgStr;
    msgStr << msgs;
    msg.data = msgStr.str();
    pub_clear_.publish(msg);
}

/**
 * @brief 发布标定执行完成 
 * @param msgs-标定类型字符串 
 * @return 
 * */
void deskmedia_node::PublishMoveFinish(const char *msgs)
{
	std_msgs::String msg;
    std::stringstream msgStr;
    msgStr << msgs;
    msg.data = msgStr.str();
    pub_move_.publish(msg);
}

/**
 * @brief 发布比例系数更新信息 
 * @param type-系数类型
 * 		  scale-比例系数
 * @return 
 * */
void deskmedia_node::PublishSetScale(int type, float scale)
{
	std_msgs::Float32 scale_factor; 

    scale_factor.data = scale; 
	if (!type) {
		set_x_scale_.publish(scale_factor);
	} else {
		set_z_scale_.publish(scale_factor);
	} 
}

/**
 * @brief 发布底盘当前状态信息 
 * @param status-状态
 * @return 
 * */
void deskmedia_node::PublishRosStatus(int status)
{
	std_msgs::UInt32 ros_status; 

    ros_status.data = status; 

	pub_status_.publish(ros_status);
}

void deskmedia_node::PublishDebugParamsCmd(const char *msgs)
{
	std_msgs::String msg;
    std::stringstream msgStr;
    msgStr << msgs;
    msg.data = msgStr.str();
    pub_debug_params_.publish(msg); 
}

void deskmedia_node::DemarcateDoneCb(const actionlib::SimpleClientGoalState& state, const deskmedia::DMDemarcateResultConstPtr& result)
{
	ROS_WARN("demarcateDoneCb done!");
	// publishMoveFinish("done");
}

void deskmedia_node::DemarcateActiveCb()
{
	ROS_WARN("demarcateActiveCb...");
}

void deskmedia_node::DemarcateFeedbackCb(const deskmedia::DMDemarcateFeedbackConstPtr& feedback)
{
	ROS_WARN("demarcateFeedbackCb feedback[%f]%%", feedback->percent * 100);
}

/**
 * @brief 执行标定任务进程 
 * @param goal-任务目标
 * 		  as-标定服务对象
 * @return 
 * */
void deskmedia_node::DemarcateExecute(const deskmedia::DMDemarcateGoalConstPtr& goal)
{
	ros::Rate r(20);
	float tm_speed = 0;
	float ts_speed = 0;
	deskmedia::DMDemarcateFeedback fb;
	ROS_WARN("doing type %d, target = %f", goal->type, goal->target);
	if (goal->type == 1) {
		if (goal->target > 0)
			tm_speed = move_speed_;
		else
			tm_speed = -move_speed_;
	} else if (goal->type == 2) {
		if (goal->target > 0)
			ts_speed = spin_speed_;
		else 
			ts_speed = -spin_speed_;
	} else {
		ds_->setSucceeded();
		return ;
	}

	ROS_WARN("move: %.03f, spin: %.03f", tm_speed, ts_speed);

	geometry_msgs::Pose t_pose;
	t_pose.position.x = g_pose_.pose.position.x;
	t_pose.position.y = g_pose_.pose.position.y;
	t_pose.orientation.w = g_pose_.pose.orientation.w;

	float t_yaw = QuaternionTransToYaw(g_pose_.pose.orientation);//起始角度t_yaw 当前角度g_yaw 目标角度goal->target
	float turn_angle = 0;//累计旋转角度
	ros::NodeHandle n;
	while(n.ok()) {
		if (goal->type == 1) {//直线运动
			float  current_distance = sqrt((g_pose_.pose.position.x - t_pose.position.x)*(g_pose_.pose.position.x - t_pose.position.x)+
										(g_pose_.pose.position.y - t_pose.position.y)*(g_pose_.pose.position.y - t_pose.position.y));
			ROS_WARN(" target_distance:%f, current_distance:%f", goal->target, current_distance);//显示目标距离 与当前运动距离

			if (!CheckMoveFinish(t_pose.position, g_pose_.pose.position, goal->target)) {
				PublishDMMotorCmd(tm_speed, 0,true);
			} else {
				PublishDMMotorCmd(0, 0,true);
				ds_->setSucceeded();
				PublishMoveFinish("move"); 
				break;
			}
		}
		else if (goal->type == 2) {//旋转运动

			float delta_angle=0;//单周期运动角度
			if( goal->target > 0) //逆时针
			{
				if(g_odom_yaw_ >= t_yaw)
					delta_angle = g_odom_yaw_-t_yaw; 
				else
					delta_angle = g_odom_yaw_+2*PI-t_yaw;  //PI--->(-PI)

			}else if (goal->target < 0)//顺时针
			{
				if(g_odom_yaw_ <= t_yaw)
					delta_angle = t_yaw-g_odom_yaw_; 
				else
					delta_angle = t_yaw-g_odom_yaw_+2*PI;	//(-PI)--->PI
			}
			t_yaw = g_odom_yaw_;//重新赋值
			turn_angle +=delta_angle;//累计旋转角度
			ROS_WARN(" target_angle:%f, turn_angle:%f", goal->target, turn_angle);
			//ROS_WARN(" c:%f, t:%f, g:%f", t_yaw, goal->target, g_yaw_);
			//if (!checkSpinFinish(t_yaw, g_yaw_, goal->target)) 
			if(turn_angle-abs(goal->target) < 0.0001){
				PublishDMMotorCmd(0, ts_speed,true);
			} else {
				PublishDMMotorCmd(0, 0,true);
				ds_->setSucceeded();
				PublishMoveFinish("spin");
				break;
			}
		} 
		ros::spinOnce();
		r.sleep();
	}

	// as->setSucceeded();
}

/**
 * @brief 接收原生导航结束消息入口，进行异常报警或是二次定位逻辑处理 
 * @param msg-消息类型字符串
 * @return 
 * */
void deskmedia_node::GetErrorCallback(const std_msgs::String::ConstPtr& msg)
{
	bool ret = false;
	float target = 0;
	ACTION_DATA_S action;
	// geometry_msgs::PoseStamped nearest_goal;
	ROS_WARN("get error[%s]", msg->data.c_str());
	//TODO 打印目标地址、小车位置、可用的最近的局部目标地址，考虑自行移动出去
	if (true) {
		ROS_ERROR("----------------------------------%s---------------------------------------", msg->data.c_str());
		ROS_WARN("This task goal @(%.3f, %.3f) - %.3f", 
			t_goal_.pose.position.x, t_goal_.pose.position.y, QuaternionTransToYaw(t_goal_.pose.orientation) * RADIAN_TO_DEGREE);
		GetDevicePose(g_tf_pose_);
		ROS_WARN("device current pose @((%.3f, %.3f) - %.3f)",
			g_tf_pose_.pose.position.x, g_tf_pose_.pose.position.y, QuaternionTransToYaw(g_tf_pose_.pose.orientation) * RADIAN_TO_DEGREE);
		ROS_ERROR("----------------------------------%s---------------------------------------", msg->data.c_str());
	}
	if (strstr(msg->data.c_str(), "OSCILLATION_R") != NULL) {
		PublishErrorCmd("无法自动复位，请检查小车位置");
//		ROS_ERROR("try to move");
//		geometry_msgs::PoseStamped nearest_goal;
//		CalcNearestPathPoint(g_tf_pose_, path_, nearest_goal);
//		ComputeVelocityCommands(g_tf_pose_, nearest_goal);
	} 
	// 全局路线无法生成，进入控制器，试探运行
	else if (strstr(msg->data.c_str(), "PLANNING_R") != NULL) {
		if (g_planning_times_++ >= TRY_TIMES) {
			g_planning_times_ = 0;
			ROS_ERROR("can not plan path, count max");
			PublishErrorCmd("计算路径失败，已达最大尝试次数，请检查小车位置");
			return ;
		}
#if 1
		if (path_.size() > 0) {
			ROS_ERROR("try to rectify in %s", msg->data.c_str());
			int times = 0;
			ros::Rate r(20);
			ros::NodeHandle n;
			geometry_msgs::PoseStamped nearest_goal;
			CalcNearestPathPoint(g_tf_pose_, path_, nearest_goal);
			while(n.ok() && (abs(ComputeMinAngle(QuaternionTransToYaw(g_tf_pose_.pose.orientation), QuaternionTransToYaw(nearest_goal.pose.orientation))) >= 0.02f 
				|| abs(g_tf_pose_.pose.position.x - nearest_goal.pose.position.x) >= 0.02f 
				|| abs(g_tf_pose_.pose.position.y - nearest_goal.pose.position.y) >= 0.02f)) {
				if (times > 20) {
					ROS_ERROR("try enough times over 200");
					break;
				}
				ComputeVelocityCommands(g_tf_pose_, nearest_goal);
				ros::spinOnce();
				r.sleep();
				times++;
			}
		} else {
			ROS_ERROR("no path to rectify pose %s", msg->data.c_str());
		} 
		PublishGoalCmd(g_goal_t_.x, g_goal_t_.y, g_goal_t_.angle);
#else
//		ROS_ERROR("can not plan path, move to useage range?");
//		if (head_space_ > 0.2) {
//			target = 0.1;
//			MoveTargetDistanceForward(target);
			PublishGoalCmd(g_goal_t_.x, g_goal_t_.y, g_goal_t_.angle);
//		} else {
//			PublishErrorCmd("空间不足，无法正常导航，请检查小车位置");
//			return ; 
//		}
#endif	
	}
	// 无法进行导航，进入控制器，试探运行 
	else if(strstr(msg->data.c_str(), "CONTROLLING_R") != NULL){
		g_planning_times_=0;
		ROS_ERROR("CONTROLLING_R!!!!");
	
//		ROS_ERROR("try to move");
//		geometry_msgs::PoseStamped nearest_goal;
//		CalcNearestPathPoint(g_tf_pose_, path_, nearest_goal);
//		ComputeVelocityCommands(g_tf_pose_, nearest_goal);
		
		// SpinTargetAngle(0.2);

		PublishGoalCmd(g_goal_t_.x, g_goal_t_.y, g_goal_t_.angle);
	}
// 	else if (strstr(msg->data.c_str(), "reached") != NULL) {
		
// 		ros::Rate r(20);
// 		ros::NodeHandle n;
// 		bool docking_failed = false;
// 		g_docking_status_ = DOCKING_INIT;
// 		while(n.ok() && (!g_docking_)) {
// //				ret = DockingTaskAction(&action); 
// 			ret = DockingTaskActionEx(&action);
// 			if (ret) {
// 				PublishDMMotorCmd(action.v, action.w,false);//不检测尾部距离
// 			} else {
// 				PublishDMMotorCmd(0, 0,true);
// 				docking_failed = true;
// 				break;
// 			}
// 			ros::spinOnce();
// 			r.sleep();
// 		}

// 		if (docking_failed) {
// 			PublishErrorCmd("docking failed");
// 		} else {
// 			if (g_docking_)
// 				SyncGetDataFlagToTurnOnEmb(false);
			
// 			if (GOAL_A == g_goal_type_)
// 				SyncPoseToAmcl(0, 0, 0);
// 			g_action_type_ = 1;
// 			g_docking_times_ = 0;
// 			PublishDMMotorCmd(0, 0,true);
// 			if (1 == g_side_)
// 				PublishMsgsCmd("reach goal in bed area");
// 			else
// 				PublishMsgsCmd("reach goal in toilet area");
// 		}
// 		g_ros_status_ = ROS_STATUS_NAVIGATION_FREE;
// 		ROS_WARN("docking finished! %f", tail_space_);
// 	}
}

/**
 * @brief 获取上位机发布的速度指令话题，并进行响应
 * @param msg-速度话题消息类型数据
 * @return 
 * */
void deskmedia_node::GetVelCallback(const deskmedia::DMVel::ConstPtr& msg)
{
	#if 1
	// ROS_WARN("get vel cmd [x: %f, z: %f]", msg->x, msg->yaw);
	if (0.0 == msg->x) {
		PublishMotorCmd(msg->x, msg->yaw);
	} else {
		if (msg->x > 0) {
			float *c_range1 = std::min_element(&g_ranges_[0], &g_ranges_[59]);
			float *c_range2 = std::min_element(&g_ranges_[660], &g_ranges_[719]);
			float nearleast = *c_range1 < *c_range2 ? *c_range1 : *c_range2;
			ROS_WARN("f: %f, t: %f, l: %f", g_ranges_[0], g_ranges_[359], nearleast);
			if (nearleast <= 0.20) {
				PublishErrorCmd("前进方向有障碍物");
				PublishMotorCmd(0, msg->yaw);
			} else {
				PublishMotorCmd(msg->x, msg->yaw);
			}
		} else {
//			float *c_range = std::min_element(&g_ranges_[300], &g_ranges_[419]);
//			ROS_WARN("f: %f, t: %f, l: %f", g_ranges_[0], g_ranges_[359], *c_range);
			ROS_WARN("f: %f, t: %f", g_ranges_[0], tail_space_);
			if (tail_space_ <= 0.37) {
				PublishErrorCmd("后退方向有障碍物");
				PublishMotorCmd(0, msg->yaw);
			} else {
				PublishMotorCmd(msg->x, msg->yaw);
			}
		}
	} 
	ROS_WARN("vel:(v: %f(%f), w: %f(%f)", msg->x, last_x_fb_, msg->yaw, last_z_fb_);
	#else 
	if (0.0 == msg->x) {//旋转
		PublishMotorCmd(msg->x, msg->yaw);
	} else {
		if (msg->x > 0) {//前进
			float c_range1 = GetObstacleToBaselinkDistance(0, 180);
			float c_range2 = GetObstacleToBaselinkDistance(540, 719);			
			float nearleast = c_range1 < c_range2 ? c_range1 : c_range2;
			if (nearleast <= 0.30) {
				PublishErrorCmd("前进方向有障碍物");
				PublishMotorCmd(0, msg->yaw);
			} else {
				PublishMotorCmd(msg->x, msg->yaw);
			}
		} else {//后退
			float c_range = GetObstacleToBaselinkDistance(180, 540);
			if (c_range <= 0.30 + LOCATION_SPACE) {
				PublishErrorCmd("后退方向有障碍物");
				PublishMotorCmd(0, msg->yaw);
			} else {
				PublishMotorCmd(msg->x, msg->yaw);
			}
		}
	} 
	#endif
}

void deskmedia_node::GetGoalCallback(const deskmedia::DMGoal::ConstPtr& msg)
{	
	g_goal_type_ = msg->type;
	ROS_WARN("set goal type: %d", g_goal_type_);
	if (GOAL_FREE == g_goal_type_) { //自由目标
		g_goal_t_.x = msg->goal_x;
		g_goal_t_.y = msg->goal_y;
		g_goal_t_.angle = msg->goal_z;
		PublishGoalCmd(msg->goal_x, msg->goal_y, msg->goal_z);
		ROS_WARN("goal (%f, %f, %f)", msg->goal_x, msg->goal_y, msg->goal_z);
	} else if (GOAL_A == g_goal_type_) { //A点
		SyncGetDataFlagToTurnOnEmb(true);
		g_goal_t_.x = 0.4;
		g_goal_t_.y = 0;
		g_goal_t_.angle = 0;
		PublishInfraredCmd(0);
		t_goal_.pose.position.x = g_goal_t_.x;
		t_goal_.pose.position.y = g_goal_t_.y;
		// 移动出对接位置
		if (g_docking_ || tail_space_ < 0.7) {
			MoveTargetDistanceForward(0.75 - tail_space_);
		}
		PublishGoalCmd(0.4, 0, 0);
	} else if (GOAL_B == g_goal_type_) { //B点
		SyncGetDataFlagToTurnOnEmb(true);
		g_goal_t_ = g_goal_b_;
		PublishInfraredCmd(0);
		// 移动出对接位置
		if (g_docking_ || tail_space_ < 0.7) {
			MoveTargetDistanceForward(0.75 - tail_space_);
		}
		PublishGoalCmd(g_goal_b_.x, g_goal_b_.y, g_goal_b_.angle / RADIAN_TO_DEGREE);
		ROS_WARN("goal (%f, %f, %f)", g_goal_b_.x, g_goal_b_.y, g_goal_b_.angle);
	} else if (GOAL_SET_A == g_goal_type_) {
		SaveGoalPoseToFile(1, msg->goal_x, msg->goal_y, msg->goal_z);
		g_goal_a_.x = msg->goal_x;
		g_goal_a_.y = msg->goal_y;
		g_goal_a_.angle = msg->goal_z;
		return ;
	} else if (GOAL_SET_B == g_goal_type_) {
		SaveGoalPoseToFile(0, msg->goal_x, msg->goal_y, msg->goal_z);
		g_goal_b_.x = msg->goal_x;
		g_goal_b_.y = msg->goal_y;
		g_goal_b_.angle = msg->goal_z;
		return ;
	} else if (GOAL_CANCEL == g_goal_type_) {
		PublishCancelGoalCmd();
	} else {
		PublishErrorCmd("未定义的目标类型");
		g_ros_status_ = ROS_STATUS_FREE;
		return ;
	}

	if (g_ros_status_ == ROS_STATUS_NAVIGATION_FREE)
		g_ros_status_ = ROS_STATUS_NAVIGATION_MOVE;
}

/**
 * @brief 获取采集板DI输入状态
 * @param msg-DI状态值
 * @return 
 * */
void deskmedia_node::GetDiStatusCallback(const std_msgs::UInt8::ConstPtr& msg)
{
	int diStatus = msg->data;
	int collisionDetection=(diStatus >> 3)& 0x01;//碰撞
	int dockFlag = (diStatus >> 4) & 0x08;  //对接结束检测信号
	int dockLeds = (diStatus >> 4) & 0x07; //三路红外
	static int count = 0;
	static float last_yaw = 0.0;
	static geometry_msgs::PoseWithCovarianceStamped last_pose;
	float distance = sqrt(pow(g_amcl_pose_.pose.pose.position.x - g_goal_t_.x, 2) + pow(g_amcl_pose_.pose.pose.position.y - g_goal_t_.y, 2));

	////碰撞
	if (collisionDetection != g_collision_) {
		ROS_WARN("Collision Detection signal from: %d to %d", g_collision_, collisionDetection);
		g_collision_ = collisionDetection;
	}
	//对接结束检测信号
	if (dockFlag != g_docking_) {
		ROS_WARN("docking signal from: %d to %d", g_docking_ >> 3, dockFlag >> 3);
		g_docking_ = dockFlag;
	}
	//三路红外
	if (g_test_for_infrared_print_)
		ROS_WARN("infrared signal (0x%02x)0x%02x, (%d, %d, %d)", diStatus, dockLeds, (dockLeds & 0x04) >> 2 ,(dockLeds & 0x02) >> 1, dockLeds & 0x01);

	//过滤异常数据 
	//如果底盘未移动，但信号变化，可认为是异常信号，不做处理
	//如果移动过程中，短暂出现2次以内的全0信号，可认为是异常信号，不做处理
	if (IsDeviceMoved(last_pose, last_yaw * RADIAN_TO_DEGREE, g_ekf_pose_, g_odom_yaw_ * RADIAN_TO_DEGREE)) {
		if (IR_NONE == dockLeds && (g_infrared_ != dockLeds))
			count++;
		else {
			count = 0;
			g_infrared_ = dockLeds;
		}
		
		if (count > 2) {
			count = 0;
//			ROS_ERROR("move infrared signal changed: 0x%02x -> 0x%02x, (%d, %d, %d)", g_infrared_, dockLeds, dockLeds & 0x01, (dockLeds & 0x04) >> 2, (dockLeds & 0x08) >> 3);
			g_infrared_ = dockLeds;
		}
		
		last_pose.pose.pose.position.x = g_ekf_pose_.pose.pose.position.x;
		last_pose.pose.pose.position.y = g_ekf_pose_.pose.pose.position.y;
		last_yaw = g_odom_yaw_;
	} else {
		if (IR_NONE != dockLeds && dockLeds != g_infrared_ && !g_docking_) {
//			ROS_ERROR("1.stay infrared signal changed: 0x%02x -> 0x%02x, (%d, %d, %d)", g_infrared_, dockLeds, dockLeds & 0x01, (dockLeds & 0x04) >> 2, (dockLeds & 0x08) >> 3);
			g_infrared_ = dockLeds;
		} else if (IR_NONE == dockLeds && IR_NONE != g_infrared_) {
			count++;
		} else {
			count = 0;
		}

		if (count > 2) {
			count = 0;
//			ROS_ERROR("2.stay infrared signal changed: 0x%02x -> 0x%02x, (%d, %d, %d)", g_infrared_, dockLeds, dockLeds & 0x01, (dockLeds & 0x04) >> 2, (dockLeds & 0x08) >> 3);
			g_infrared_ = dockLeds;
		}
	}

	if ((g_ros_status_ == ROS_STATUS_NAVIGATION_MOVE) && 
		(t_goal_.pose.position.x != g_amcl_pose_.pose.pose.position.x) && 
		(t_goal_.pose.position.y != g_amcl_pose_.pose.pose.position.y) &&
		distance < 0.6 && (IR_ALL == g_infrared_ || IR_SIDE == g_infrared_)) {
			PublishInfraredCmd(g_infrared_);
	}
}

/**
 * @brief 获取采集板IR输入状态
 * @param msg-IR状态值
 * @return 
 * */
void deskmedia_node::GetIrStatusCallback(const std_msgs::UInt32::ConstPtr& msg)
{
	static int count = 0;
	int irStatus = msg->data;
	if (irStatus != g_side_) {
		if (0 == irStatus && count < 2) {
			count++;
			if (count >= 2) {
				count = 0;
				ROS_WARN("ir status change from: %d to %d", g_side_, irStatus);	
				g_side_ = irStatus;
			} else {
				return ;
			}
		} else {
			ROS_WARN("ir status change from: %d to %d", g_side_, irStatus);	
			g_side_ = irStatus;
		}	
	} else {
		count = 0;
	}
}

/**
 * @brief 获取采集板反馈水路状态回调
 * @param msg-水路状态信息数据
 * @return 
 * */
void deskmedia_node::GetWaterStatusCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	Json::Value value;
	Json::Reader reader;	
	ROS_WARN("get water status[%s]", msg->data.c_str()); 

	if(reader.parse(msg->data, value)) {
        if(!value["terminal"].isNull()) {
			if (!value["terminal"].isNull())
				sprintf(g_water_info.terminal, "%s", value["terminal"].asString().c_str());
			if (!value["cleanTop"].isNull())
            	g_water_info.cleanTop = value["cleanTop"].asBool();
			if (!value["sewageTop"].isNull())
				g_water_info.sewageTop = value["sewageTop"].asBool();
			if (!value["cleanBottom"].isNull())
				g_water_info.cleanBottom = value["cleanBottom"].asBool();
			if (!value["sewageBottom"].isNull())
				g_water_info.sewageBottom = value["sewageBottom"].asBool();
			if (!value["toiletHasWater"].isNull())
				g_water_info.toiletHasWater = value["toiletHasWater"].asBool();
			if (!value["inNavigation"].isNull())
				g_water_info.navigation = value["inNavigation"].asBool();
        }
    }
	
	if ((strstr(g_water_info.terminal, "CAR") != NULL) && (g_water_info.navigation == false)) {
		//if (g_water_info.cleanTop == true && g_water_info.sewageTop == true){
		if (g_water_info.cleanTop == true && g_water_info.sewageTop == true) {
			dock_w_ = minw_ * 2;
			dock_v_ = minv_ + 0.02;
			system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.5");
		} else if ((g_water_info.cleanTop == true && g_water_info.sewageBottom == true) || (g_water_info.sewageTop == true && g_water_info.cleanBottom == true)) {
			dock_w_ = minw_ * 1.5;
			dock_v_ = minv_ + 0.01;
			system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.4");
		} else {
			dock_w_ = minw_;
			dock_v_ = minv_;
			system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.3");
		}
		ROS_INFO("need change docking params: v -> %.3f, w -> %.3f", dock_v_, dock_w_);
	} else if (strstr(g_water_info.terminal, "BED") != NULL) {

	} else if (strstr(g_water_info.terminal, "TOILET") != NULL) {

	}
//	ROS_WARN("terminal: %s", g_water_info.terminal);
//	ROS_WARN("water info:[%d, %d, %d, %d]", g_water_info.cleanTop, g_water_info.cleanBottom, g_water_info.sewageTop, g_water_info.sewageBottom);
}

/**
 * @brief 请求获取机器人参数回调入口
 * @param msg-请求的参数类型
 * @return 
 * */
void deskmedia_node::GetRobotParamsCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	Json::Value root;
	Json::FastWriter fast_writer;
	ROS_WARN("get robot params cmd[%s]", msg->data.c_str());
	if (strstr(msg->data.c_str(), "debug") != NULL) {
		Json::Value dwa;
//		dwa["module"] = Json::Value("dwa");  
		dwa["max_vel_x"] = Json::Value(g_reconfig_info.dwa_params.max_vel_x); 
		dwa["min_vel_x"] = Json::Value(g_reconfig_info.dwa_params.min_vel_x); 
		dwa["max_vel_theta"] = Json::Value(g_reconfig_info.dwa_params.max_vel_theta); 
		dwa["min_vel_theta"] = Json::Value(g_reconfig_info.dwa_params.min_vel_theta); 
		root["dwa"] = dwa;
	}
	ROS_INFO("get robot params data: %s", fast_writer.write(root).c_str());

//	std_msgs::String msg;
//    std::stringstream msgStr;
//    msgStr << fast_writer.write(root).c_str();
//    msg.data = msgStr.str();
//    pub_debug_params_.publish(msg); 

	PublishDebugParamsCmd(fast_writer.write(root).c_str());
}

/**
 * @brief 获取需要重新设置的参数回调
 * @param msg-需要修改的参数描述信息
 * @return 
 * */
void deskmedia_node::GetDynamicReconfigureCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	std::string cmd;
	char str_temp[64];
	Json::Value value;
	Json::Reader reader;
	bool not_first = false;
	
	cmd += "rosrun dynamic_reconfigure dynparam set ";
//	 /move_base/DWAPlannerROS "{'max_vel_theta': 0.8, 'max_vel_trans': 0.2}"
	ROS_INFO("get reconfigure data[%s]", msg->data.c_str()); 

	if(reader.parse(msg->data, value)) {
        if(!value["module"].isNull()) {
			sprintf(g_reconfig_info.module, "%s", value["module"].asString().c_str());
			if (strstr(g_reconfig_info.module, "dwa") != NULL) {
				cmd += "/move_base/DWAPlannerROS \"{";
				if (!value["max_vel_x"].isNull()) {
					g_reconfig_info.dwa_params.max_vel_x = value["max_vel_x"].asDouble();
					if (not_first) cmd += ", ";
					sprintf(str_temp, "'max_vel_x':%.03f", g_reconfig_info.dwa_params.max_vel_x);
					not_first = true;
					cmd += str_temp;
				}

				if (!value["min_vel_x"].isNull()) {
					g_reconfig_info.dwa_params.min_vel_x = value["min_vel_x"].asDouble();
					if (not_first) cmd += ", ";
					sprintf(str_temp, "'min_vel_x':%.03f", g_reconfig_info.dwa_params.min_vel_x);
					not_first = true;
					cmd += str_temp;
				}

				if (!value["max_vel_theta"].isNull()) {
					g_reconfig_info.dwa_params.max_vel_x = value["max_vel_theta"].asDouble();
					if (not_first) cmd += ", ";
					sprintf(str_temp, "'max_vel_theta':%.03f", g_reconfig_info.dwa_params.max_vel_x);
					not_first = true;
					cmd += str_temp;
				}

				if (!value["min_vel_theta"].isNull()) {
					g_reconfig_info.dwa_params.max_vel_x = value["min_vel_theta"].asDouble();
					if (not_first) cmd += ", ";
					sprintf(str_temp, "'min_vel_theta':%.03f", g_reconfig_info.dwa_params.max_vel_x);
					not_first = true;
					cmd += str_temp;
				}

				cmd += "}\"";
			}else if (strstr(g_reconfig_info.module, "move") != NULL){
				cmd += "/move_base \"{";
//				if (!value["max_vel_x"].isNull()) {
//					g_reconfig_info.dwa_params.max_vel_x = value["max_vel_x"].asDouble();
//					if (not_first) cmd += ", ";
//					sprintf(str_temp, "'max_vel_x':%.03f", g_reconfig_info.dwa_params.max_vel_x);
//					not_first = true;
//					cmd += str_temp;
//				}
//
//				if (!value["min_vel_x"].isNull()) {
//					g_reconfig_info.dwa_params.min_vel_x = value["min_vel_x"].asDouble();
//					if (not_first) cmd += ", ";
//					sprintf(str_temp, "'min_vel_x':%.03f", g_reconfig_info.dwa_params.min_vel_x);
//					not_first = true;
//					cmd += str_temp;
//				}
//
//				if (!value["max_vel_theta"].isNull()) {
//					g_reconfig_info.dwa_params.max_vel_x = value["max_vel_theta"].asDouble();
//					if (not_first) cmd += ", ";
//					sprintf(str_temp, "'max_vel_theta':%.03f", g_reconfig_info.dwa_params.max_vel_x);
//					not_first = true;
//					cmd += str_temp;
//				}
//
//				if (!value["min_vel_theta"].isNull()) {
//					g_reconfig_info.dwa_params.max_vel_x = value["min_vel_theta"].asDouble();
//					if (not_first) cmd += ", ";
//					sprintf(str_temp, "'min_vel_theta':%.03f", g_reconfig_info.dwa_params.max_vel_x);
//					not_first = true;
//					cmd += str_temp;
//				}

				cmd += "}\"";
			}
        } else {
			ROS_WARN("unknow reconfigure module");
		}
    }
	ROS_INFO("get reconfigure cmd [%s]", cmd.c_str());
	system(cmd.c_str());
}

/**
 * @brief 获取建地图指令回调响应
 * @param msg-指令字符串
 * @return 
 * */
void deskmedia_node::GetMappingCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_WARN("get mapping cmd[%s]", msg->data.c_str());
	if (strstr(msg->data.c_str(), "mapping") != NULL && (g_map_ != MAP_START_BUILD)) { //开始建地图
		if (g_ros_status_ >= ROS_STATUS_NAVIGATION_FREE) {
			system("rostopic pub /dnavigation std_msgs/String \"data: \'finish\'\"");
			WaitForExec(2);
		}
		ROS_WARN("start mapping!");
		g_map_ = MAP_START_BUILD;
		g_map_index_ = 0;
		g_ros_status_ = ROS_STATUS_MAPPING;
		clearAllPoseData();
		SyncGetDataFlagToTurnOnEmb(true);
		system("roslaunch turn_on_wheeltec_robot dm_mapping.launch &");
		system("cd /home/ximei/robot_robot;python -m SimpleHTTPServer 8000 &");
	} else if (strstr(msg->data.c_str(), "save") != NULL && (g_map_ != MAP_SAVE)) {	//地图保存
		g_map_ = MAP_SAVE;
		g_ros_status_ = ROS_STATUS_FREE;
		char map_store[128] = {0};
		//保存pbstream文件 
		sprintf(map_store, "rosservice call /write_state \"filename: \'/home/ximei/robot_robot/src/cartographer_ros/cartographer_ros/mapfiles/map.pbstream\'\" ");
		system(map_store);
		
		sprintf(map_store, "rm -f %smap.bmp;cp %stemp.bmp %smap.bmp &", MAP_STORE_PATH, MAP_STORE_PATH, MAP_STORE_PATH);
		system(map_store);
		sprintf(map_store, "rm -f %smap.jpeg;cp %smap%d.jpeg %smap.jpeg &", MAP_STORE_PATH, MAP_STORE_PATH, g_map_index_, MAP_STORE_PATH);
		system(map_store);
		system("roslaunch turn_on_wheeltec_robot map_saver.launch");
		system("ps -ef | grep dm_mapping | grep -v \"color\" | cut -c 9-15 | xargs kill -2");//杀死不用的进程
		system("ps -ef | grep slam_gmapping | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		system("ps -ef | grep algorithm_gmapping | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		PublishMsgsCmd("map save finish");
		ROS_WARN("map save finish");
		SaveOdomCombPoseToFile();
		g_save_amcl_ = true;
	} else if (strstr(msg->data.c_str(), "clear") != NULL && (g_map_ != MAP_START_BUILD)) {
		system("rm -rf /home/ximei/robot_robot/src/turn_on_wheeltec_robot/map/WHEELTEC.pgm");
		system("rm -rf /home/ximei/robot_robot/src/turn_on_wheeltec_robot/map/WHEELTEC.yaml");
	}
}

/**
 * @brief 获取节点控制逻辑回调响应
 * @param msg-指令字符串
 * @return 
 * */
void deskmedia_node::GetControlCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_WARN("get control cmd[%s]", msg->data.c_str());
	if (strstr(msg->data.c_str(), "stop") != NULL ) {
		system("ps -ef | grep control | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		system("ps -ef | grep rplidarNode | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		system("ps -ef | grep common_node | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		system("ps -ef | grep wheeltec_robot_node | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		system("ps -ef | grep rosbridge_websocket | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
	}
}

/**
 * @brief 进入导航控制逻辑回调入口
 * @param msg-指令字符串
 * @return 
 * */
void deskmedia_node::GetNavigationCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_WARN("get navigation cmd[%s]", msg->data.c_str());
	if (strstr(msg->data.c_str(), "start") != NULL && (g_nav_ != NAV_START)) {
		g_nav_ = NAV_START;
		g_ros_status_ = ROS_STATUS_NAVIGATION_FREE;
		system("ps -ef | grep SimpleHTTPServer | grep -v \"color\" | cut -c 9-15 | xargs kill -9");
		system("roslaunch turn_on_wheeltec_robot dm_navigation.launch &");
	} else if (strstr(msg->data.c_str(), "finish") != NULL && (g_nav_ != NAV_FINISH)) {
		g_nav_ = NAV_FINISH;
		g_ros_status_ = ROS_STATUS_FREE;
		system("ps -ef | grep amcl | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		system("ps -ef | grep move_base | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		system("ps -ef | grep map_server | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		system("ps -ef | grep dm_navigation | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		system("ps -ef | grep dwa_local_planner | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		system("ps -ef | grep robot_state_publisher | grep -v \"color\" | cut -c 9-15 | xargs kill -2");
		PublishDMMotorCmd(0, 0,true);
	}
}

/**
 * @brief 底盘旋转控制逻辑回调响应，通过次数估算
 * @param msg-指令字符串
 * @return 
 * */
void deskmedia_node::GetSpinCallback(const std_msgs::Float32::ConstPtr& msg)
{
	float speed = spin_speed_;
	float radian = (msg->data) / RADIAN_TO_DEGREE;
	ROS_WARN("getSpinCallback need spin:%.3f", radian);
	int i = 0;
	int times = 20;
	int count = abs(radian) / speed;
	ros::Rate r(20);
	ros::NodeHandle n;
	while(n.ok() && (i++ < (count * times))) {
		if (radian > 0)
			PublishDMMotorCmd(0, speed,true);
		else
			PublishMotorCmd(0, -speed);
		ros::spinOnce();
		r.sleep();
	}	
	PublishMotorCmd(0, 0);
	ROS_WARN("spin finish!");
}

/**
 * @brief 底盘执行控制逻辑回调响应，通过次数估算
 * @param msg-指令字符串
 * @return 
 * */
void deskmedia_node::GetMoveCallback(const std_msgs::Float32::ConstPtr& msg)
{
	float speed = move_speed_;
	float distance = (msg->data);
	int i = 0;
	int times = 20;
	int count = abs(distance) / speed;
	ROS_WARN("getMoveCallback need move:%.3f m, with speed:%f about %d times", distance, speed, count * times);
	ros::Rate r(20);
	ros::NodeHandle n;
	while(n.ok() && (i++ < (count * times))) {
		if (distance > 0)		
			PublishDMMotorCmd(speed, 0,true);
		else
			PublishDMMotorCmd(-speed, 0,true);
		ros::spinOnce();
		r.sleep();
	}	
	PublishDMMotorCmd(0, 0,true);
	ROS_WARN("move finish!");
}

/**
 * @brief 系统控制指令回调入口
 * @param msg-指令字符串
 * @return 
 * */
void deskmedia_node::GetControlCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	const char *cmd = msg->data.c_str();
	ROS_WARN("get test cmd[%s]", cmd);
	if (strstr(cmd, "reboot") != NULL) {
		system("echo 123456 | sudo -S reboot");
	}
}

/**
 * @brief 调试指令回调入口
 * @param msg-指令字符串
 * @return 
 * */
void deskmedia_node::GetTestCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	int ret = 0;
	float spin = 0.0;
	ACTION_DATA_S action;
	ROS_WARN("get test cmd[%s]", msg->data.c_str());
	if (strstr(msg->data.c_str(), "vert") != NULL) {
		
	} else if (strstr(msg->data.c_str(), "pose") != NULL) {
		ROS_WARN("amcl pose[x: %.3f, y: %.3f, z: %.3f], [x: %.3f, y: %.3f, z: %.3f, w: %.3f]",
			g_amcl_pose_.pose.pose.position.x, g_amcl_pose_.pose.pose.position.y, g_amcl_pose_.pose.pose.position.z,
			g_amcl_pose_.pose.pose.orientation.x, g_amcl_pose_.pose.pose.orientation.y, g_amcl_pose_.pose.pose.orientation.z, g_amcl_pose_.pose.pose.orientation.w);
		ROS_WARN("odom pose[x: %.3f, y: %.3f, z: %.3f], [x: %.3f, y: %.3f, z: %.3f, w: %.3f]",
			g_pose_.pose.position.x, g_pose_.pose.position.y, g_pose_.pose.position.z,
			g_pose_.pose.orientation.x, g_pose_.pose.orientation.y, g_pose_.pose.orientation.z, g_pose_.pose.orientation.w);
		ROS_WARN("comb pose[x: %.3f, y: %.3f, z: %.3f], [x: %.3f, y: %.3f, z: %.3f, w: %.3f]",
			g_ekf_pose_.pose.pose.position.x, g_ekf_pose_.pose.pose.position.y, g_ekf_pose_.pose.pose.position.z,
			g_ekf_pose_.pose.pose.orientation.x, g_ekf_pose_.pose.pose.orientation.y, g_ekf_pose_.pose.pose.orientation.z, g_amcl_pose_.pose.pose.orientation.w);
	} else if (strstr(msg->data.c_str(), "laser") != NULL) {
		ROS_WARN("head = %f, left = %f, right = %f, tail = %f", g_ranges_[0], g_ranges_[179], g_ranges_[539], g_ranges_[359]);
	} else if (strstr(msg->data.c_str(), "docking_led") != NULL) {
		ROS_WARN("leds status[0x%02x] -- [%d]", g_docking_, g_docking_ >> 1);
	} else if (strstr(msg->data.c_str(), "infrared") != NULL) {
		if (strstr(msg->data.c_str(), "on") != NULL) {
			g_test_for_infrared_print_ = true;
		} else if (strstr(msg->data.c_str(), "off") != NULL) {
			g_test_for_infrared_print_ = false;
		} else {
			ROS_WARN("leds status[0x%02x] -- [%d,%d,%d]", g_infrared_, g_infrared_ & 0x01, (g_infrared_ & 0x02) >> 1, (g_infrared_ & 0x04) >> 2);

		}
	} else if (strstr(msg->data.c_str(), "cmd_vel") != NULL) {
		if (strstr(msg->data.c_str(), "on") != NULL) {
			g_test_for_vel_print_ = true;
		} else if (strstr(msg->data.c_str(), "off") != NULL) {
			g_test_for_vel_print_ = false;
		}
	} else if (strstr(msg->data.c_str(), "single_spin") != NULL) {
		float w = -0.2;
		float v = -w * 0.287 / 2;
		PublishDMMotorCmd(v, w,true);
	} else if (strstr(msg->data.c_str(), "docking_test") != NULL) {
		
		ros::Rate r(20);
		ros::NodeHandle n;
		g_docking_times_ = 0;//对接次数
		while(n.ok() && (!g_docking_)) {
			
			ret = DockingTaskActionEx(&action);
			if (ret)
				PublishDMMotorCmd(action.v, action.w,false);//不检测尾部距离
			else
				break;
			ros::spinOnce();
			r.sleep();
		}
		g_action_type_ = 1;
		PublishDMMotorCmd(0, 0,true);
		ROS_WARN("docking finished!");
	} else if (strstr(msg->data.c_str(), "ros_status") != NULL) {
		PublishRosStatus(0x08);
	}
}

/**
 * @brief 原生电机控制指令下发数据查看
 * @param msg-指令字符串
 * @return 
 * */
void deskmedia_node::GetCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	if (g_test_for_vel_print_)
		ROS_WARN("cmd vel[x: %.3f, z: %.3f]", msg->linear.x, msg->angular.z);
}

/**
 * @brief 获取节点注册结果，用于控制节点间的前后逻辑
 * @param msg-指令字符串
 * @return 
 * */
void deskmedia_node::GetRegisterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_WARN("get register module[%s]", msg->data.c_str());
	if (strstr(msg->data.c_str(), "robot") != NULL) {
		PublishSetScale(SCALE_X, scale_factor_x_);
		PublishSetScale(SCALE_Z, scale_factor_z_);
		PublishRosStatus(g_emb_status_);
	} else if (strstr(msg->data.c_str(), "navigation") != NULL) {
		GetDynamicReconfigureInit();
		PublishMsgsCmd("navigation ready");
	}
}

/**
 * @brief 获取实际标定移动的距离，并重新计算放大系数
 * @param msg-实际移动距离
 * @return 
 * */
void deskmedia_node::GetActualMoveCallback(const std_msgs::Float32::ConstPtr& msg)
{
	//calc new scale factor
	float scale = scale_factor_x_ * (move_target_ / msg->data);
	ROS_WARN("move target = %f, actual move: %f => scale need: %f", move_target_, msg->data, scale);
	PublishSetScale(SCALE_X, scale);

	//write to file
	char cmd[256] = {0};
	sprintf(cmd, "sed -i 's/param name=\"scale_factor_x_\" value=\"%f\"/param name=\"scale_factor_x_\" value=\"%f\"/g' /home/ximei/robot_robot/src/deskmedia/launch/control.launch", scale_factor_x_, scale);
	ROS_WARN("replace :%s", cmd);
	system(cmd);
	scale_factor_x_ = scale;
}

/**
 * @brief 获取实际标定旋转的距离，并重新计算放大系数
 * @param msg-实际旋转角度
 * @return 
 * */
void deskmedia_node::GetActualSpinCallback(const std_msgs::Float32::ConstPtr& msg)
{
	//calc new scale factor
	float scale = scale_factor_z_ * (spin_target_ / msg->data);
	ROS_WARN("spin target = %f, actual move: %f => scale need: %f", spin_target_, msg->data, scale);
	PublishSetScale(SCALE_Z, scale);

	//write to file
	char cmd[256] = {0};
	sprintf(cmd,  "sed -i 's/param name=\"scale_factor_z_\" value=\"%f\"/param name=\"scale_factor_z_\" value=\"%f\"/g' /home/ximei/robot_robot/src/deskmedia/launch/control.launch", scale_factor_z_, scale);
	ROS_WARN("replace :%s", cmd);
	system(cmd);
	scale_factor_z_ = scale;
}

/**
 * @brief 获取地图数据，并进行格式压缩转换
 * @param map-原生地图数据
 * @return 
 * */
void deskmedia_node::GetMapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	char flag = 0;
	char color = 0;
	int row = map->info.width;
	int line = map->info.height;
	static int compress_flag = 0;  //压缩标志
	char map_name[64] = {0};
	char map_format[128] = {0};
	RGB_QUAD rgbquad[256] = {0};
	BIT_MAP_FILE_HEADER fileHeader = {0};
	BIT_MAP_INFO_HEADER infoHeader = {0};
	if (!compress_flag) {
		if (g_map_index_ >= 4) {
			g_map_index_ = 1;
		} else {
			g_map_index_++;
		}
		compress_flag = 1;
		ROS_WARN("compress map data start, %d * %d = %ld", row, line, map->data.size());
		std::string mapdatafile = "/home/ximei/robot_robot/temp.bmp";
		
		ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
		FILE* out = fopen(mapdatafile.c_str(), "w");
		if (!out) {
			ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
			return;
		}

		ROS_WARN("save date to file");
		// fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
		// 		map->info.resolution, map->info.width, map->info.height);
		fileHeader.bfType = 0x4D42;
		fileHeader.bfResv1 = 0x0000;
		fileHeader.bfResv2 = 0x0000;
		fileHeader.bfOffBits = 54 + sizeof(RGB_QUAD) * 256;

		infoHeader.biSize = 40;
		infoHeader.biWidth = map->info.width;
		infoHeader.biHeight = map->info.height;
		infoHeader.biPlanes = 0x0001;
		infoHeader.biBitCount = 0x0008;
		infoHeader.biCompression = 0x0000;
		infoHeader.biSizeImage = infoHeader.biWidth * infoHeader.biHeight;

		fileHeader.bfSize = fileHeader.bfOffBits + infoHeader.biSizeImage;
		// ROS_ERROR("file size: %d, info size: %d, rgb: %d", sizeof(BITMAPFILEHEADER), sizeof(BITMAPINFOHEADER), sizeof(RGBQUAD) * 256);
		// ROS_ERROR("head:%d, image:%d, total:%d", fileHeader.bfOffBits, infoHeader.biSizeImage, fileHeader.bfSize);
		
		for (int i = 0; i < 256; i++) {
			rgbquad[i].rgbBlue = i;
			rgbquad[i].rgbGreen = i;
			rgbquad[i].rgbRed = i;
			rgbquad[i].rgbReserved = 0;
		}

		fwrite(&fileHeader, 8, 1, out);
		fwrite(&fileHeader.bfResv2, 2, 1, out);
		fwrite(&fileHeader.bfOffBits, 4, 1, out);
		fwrite(&infoHeader, sizeof(BIT_MAP_INFO_HEADER), 1, out);
		fwrite(&rgbquad, sizeof(RGB_QUAD) * 256, 1, out);

		for(unsigned int y = 0; y < map->info.height; y++) {
			for(unsigned int x = 0; x < map->info.width; x++) {
				// unsigned int i = x + (map->info.height - y - 1) * map->info.width;
				unsigned int i = x + y * map->info.width;
				if (map->data[i] >= 0 && map->data[i] <= 25) { // [0,free) 254 
					fputc(0xfe, out);
				} else if (map->data[i] >= 65) { // (occ,255] 0
					fputc(0x00, out);
				} else { //occ [0.25,0.65] 205
					fputc(0xcd, out);
				}
			}
		}
		fclose(out);

		ROS_WARN("formt bmp to jpeg[%d]", g_map_index_);
		sprintf(map_name, "/home/ximei/robot_robot/map%d.jpeg", g_map_index_);
		sprintf(map_format, "cjpeg -outfile %s /home/ximei/robot_robot/temp.bmp", map_name);
		ROS_WARN("compress map data finish");
		system(map_format);
		compress_flag = 0;

		//触发更新地图数据
		if (g_ros_status_ == ROS_STATUS_MAPPING)
			PublishMapInfo(map->info, g_map_index_);
	}
}

void deskmedia_node::GetMapDataCallbackEx(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	char flag = 0;
	char color = 0;
	int row = map->info.width;
	int line = map->info.height;
	static int last_row = 0;
	static int last_line = 0;
	#if 0
	if (last_row == row && last_line == line) {
		ROS_ERROR("map do not update!");
		return ;
	} else if (row % 4 != 0) {
		ROS_ERROR("map date do not align...(%d * %d)!", row, line);
		last_line = line;
		last_row = row;
		return ;
	} else {
		last_line = line;
		last_row = row;
	}
	#else
	if (row % 4 != 0) {
		ROS_ERROR("map date do not align...(%d * %d)!", row, line);
		return ;
	}
	#endif
	
	int skip = 0;
	char map_name[64] = {0};
	char map_temp[64] = {0};
	char map_clean[128] ={0};
	char map_format[128] = {0};
	RGB_QUAD rgbquad[256] = {0};
	BIT_MAP_FILE_HEADER fileHeader = {0};
	BIT_MAP_INFO_HEADER infoHeader = {0};	
	
	if (!g_compress_flag) {
		if (g_map_index_ >= 5) {
			g_map_index_ = 1;
		} else {
			g_map_index_++;
		}
		g_compress_flag = 1;
		ROS_WARN("compress map data start, %d * %d = %ld", row, line, map->data.size());
		#if 1
//		std::string mapdatafile = "/home/ximei/robot_robot/temp.bmp";
//		ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
		sprintf(map_name, "/home/ximei/robot_robot/temp.bmp");
		ROS_INFO("Writing map src data to %s", map_name);
		#else
		sprintf(map_name, "/home/ximei/robot_robot/map%d.jpeg", g_map_index_);
		ROS_INFO("Writing map occupancy data to %s", map_name);
		#endif
		sprintf(map_clean, "rm -rf %s", map_name);
		system(map_clean);
		FILE* out = fopen(map_name, "w");
		if (!out) {
			ROS_ERROR("Couldn't save map file to %s", map_name);
			return;
		}

		ROS_WARN("save date to file");
		// fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
		// 		map->info.resolution, map->info.width, map->info.height);
		fileHeader.bfType = 0x4D42;
		fileHeader.bfResv1 = 0x0000;
		fileHeader.bfResv2 = 0x0000;
		fileHeader.bfOffBits = 54 + sizeof(RGB_QUAD) * 256;

		infoHeader.biSize = 40;
		infoHeader.biWidth = map->info.width;
		infoHeader.biHeight = map->info.height;
		infoHeader.biPlanes = 0x0001;
		infoHeader.biBitCount = 0x0008;
		infoHeader.biCompression = 0x0000;
		infoHeader.biSizeImage = infoHeader.biWidth * infoHeader.biHeight;

		fileHeader.bfSize = fileHeader.bfOffBits + infoHeader.biSizeImage;
		// ROS_ERROR("file size: %d, info size: %d, rgb: %d", sizeof(BITMAPFILEHEADER), sizeof(BITMAPINFOHEADER), sizeof(RGBQUAD) * 256);
		// ROS_ERROR("head:%d, image:%d, total:%d", fileHeader.bfOffBits, infoHeader.biSizeImage, fileHeader.bfSize);
		
		for (int i = 0; i < 256; i++) {
			rgbquad[i].rgbBlue = i;
			rgbquad[i].rgbGreen = i;
			rgbquad[i].rgbRed = i;
			rgbquad[i].rgbReserved = 0;
		}

		fwrite(&fileHeader, 8, 1, out);
		fwrite(&fileHeader.bfResv2, 2, 1, out);
		fwrite(&fileHeader.bfOffBits, 4, 1, out);
		fwrite(&infoHeader, sizeof(BIT_MAP_INFO_HEADER), 1, out);
		fwrite(&rgbquad, sizeof(RGB_QUAD) * 256, 1, out);

		for(unsigned int y = 0; y < map->info.height; y++) {
			for(unsigned int x = 0; x < map->info.width; x++) {
				unsigned int i = x + (map->info.height - y - 1) * map->info.width;
//				fputc(map->data[i], out); 
				if (map->data[i] >= 0 && map->data[i] <= 55) { // [0,free) 254 
					fputc(0xfe, out);
				} else if (map->data[i] >= 80) { // (occ,255] 0
					fputc(0x00, out);
				} else { //occ [0.25,0.65] 205
					fputc(0xcd, out);
				}
			}
		}
		fclose(out);

		ROS_WARN("formt bmp to jpeg[%d]", g_map_index_);
		sprintf(map_temp, "/home/ximei/robot_robot/map%d.jpeg", g_map_index_);
		sprintf(map_format, "cjpeg -outfile %s /home/ximei/robot_robot/temp.bmp", map_temp);
		system(map_format);
		g_compress_flag = 0; 
		ROS_WARN("compress map data finish");

		//触发更新地图数据
		if (g_ros_status_ == ROS_STATUS_MAPPING)
			PublishMapInfo(map->info, g_map_index_);
	} else {
		ROS_ERROR("last map is compressing");
	}
}

/**
 * @brief 获取激光雷达距离数据
 * @param laser_msg-雷达原生数据
 * @return 
 * */
void deskmedia_node::GetScanDataCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{	
	g_ranges_ = laser_msg->ranges;
	float *temp=std::min_element(&g_ranges_[349], &g_ranges_[369]);//保证不为inf

	head_space_ = g_ranges_[0];
	tail_space_ = *temp;
	left_space_ = g_ranges_[179];
	right_space_ = g_ranges_[539];

}

/**
 * @brief 获取采集板反馈得线速度
 * @param msg-反馈速度数据
 * @return 
 * */
void deskmedia_node::GetSrcXVelDataCallback(const std_msgs::Float32::ConstPtr& msg)
{
	last_x_fb_ = msg->data;
}

/**
 * @brief 获取采集板反馈得角速度
 * @param msg-反馈速度数据
 * @return 
 * */
void deskmedia_node::GetSrcZVelDataCallback(const std_msgs::Float32::ConstPtr& msg)
{
	last_z_fb_ = msg->data;
}


/**
 * @brief 获取底盘系统当前状态回调
 * @param msg-查询指令字符串
 * @return 
 * */
void deskmedia_node::GetEmbRosStatusCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
	int emb_status = msg->data;
	if (emb_status != g_emb_status_) {
		ROS_WARN("ros status changed: 0x%02x -> 0x%02x", g_emb_status_, emb_status);
		g_emb_status_ = emb_status;
	}
}

/**
 * @brief 获取底盘系统当前状态回调
 * @param msg-查询指令字符串
 * @return 
 * */
void deskmedia_node::GetRosStatusToUpperCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	
	ROS_WARN("get ros status cmd: %s", msg->data.c_str());
	if (strstr(msg->data.c_str(), "model") != NULL)
		PublishRosStatus(g_ros_status_);
	else if (strstr(msg->data.c_str(), "position") != NULL) {
		if (g_docking_) {
			PublishLocationCmd(g_side_ + 10); 
		} else {
			PublishLocationCmd(g_side_);
		}
	}
}

/**
 * @brief 设置地图修改回调入口
 * @param msg-指令参数
 * @return 
 * */
void deskmedia_node::GetModifyMapCmdCallback(const deskmedia::DMSetMap::ConstPtr& msg)
{
	float resolution = 0.01;
	RGB_QUAD rgbquad[256] = {0};
	BIT_MAP_FILE_HEADER fileHeader = {0};
	BIT_MAP_INFO_HEADER infoHeader = {0};
	clearForbidAreaInfoFile();
	
	ROS_WARN("set points to map [%ld]", msg->areas.size());

	//读取地图文件
	std::string srcFile = "/home/ximei/robot_robot/map.bmp";
	std::string modifyFile = "/home/ximei/robot_robot/modify.bmp";
	
	ROS_INFO("read src map data from %s", srcFile.c_str());
	FILE* out = fopen(srcFile.c_str(), "r");
	if (!out) {
		ROS_ERROR("Couldn't read map data from %s", srcFile.c_str());
		return;
	}
	// fread(&fileHeader, 1, sizeof(fileHeader), out);
	fread(&fileHeader, 8, 1, out);
	fread(&fileHeader.bfResv2, 2, 1, out);
	fread(&fileHeader.bfOffBits, 4, 1, out);
	fread(&infoHeader, 1, sizeof(infoHeader), out);
	fread(&rgbquad, sizeof(RGB_QUAD) * 256, 1, out);

	ROS_WARN("map size %d, [%d * %d]", infoHeader.biSizeImage, infoHeader.biWidth, infoHeader.biHeight);
	unsigned char *map = (unsigned char *)malloc(infoHeader.biSizeImage);
	fseek(out, fileHeader.bfOffBits, SEEK_SET);
	fread(map, 1, infoHeader.biSizeImage, out);
	fclose(out);

	//填充数据
	for (int i = 0; i < msg->areas.size(); i++) {
		ROS_WARN("set areas to [%d] - [%d, s(%f, %f), e(%f, %f)]", i, msg->areas[i].type, msg->areas[i].start_x, msg->areas[i].start_y, msg->areas[i].end_x, msg->areas[i].end_y);
		if (msg->areas[i].type == 1 && (abs(msg->areas[i].start_x - msg->areas[i].end_x) > 1 || abs(msg->areas[i].start_y - msg->areas[i].end_y) > 1)) {
			ROS_WARN("draw data to map: %d", i);
			UpdateMapArea(map, infoHeader.biWidth, infoHeader.biHeight, msg->areas[i].start_x, msg->areas[i].start_y, msg->areas[i].end_x, msg->areas[i].end_y);
		} else { 
			if (!SaveForbidAreaInfoToFile(msg->areas[i].start_x, msg->areas[i].start_y, msg->areas[i].end_x, msg->areas[i].end_y))
				PublishErrorCmd("禁行区参数保存失败");
		}
	}

	FILE* in = fopen(modifyFile.c_str(), "w");
	if (!in) {
		ROS_ERROR("Couldn't write map to %s", modifyFile.c_str());
		return;
	}
	
	fwrite(&fileHeader, 8, 1, in);
	fwrite(&fileHeader.bfResv2, 2, 1, in);
	fwrite(&fileHeader.bfOffBits, 4, 1, in);
	fwrite(&infoHeader, 1, sizeof(infoHeader), in);
	fwrite(&rgbquad, sizeof(RGB_QUAD) * 256, 1, in);	
	fwrite(map, 1, infoHeader.biSizeImage, in);
	fclose(in);
	free(map);

	//保存成pgm图片
	std::string map_data_file = "/home/ximei/robot_robot/src/turn_on_wheeltec_robot/map/WHEELTEC.pgm";
	ROS_INFO("Writing map occupancy data to %s", map_data_file.c_str()); 
	FILE* modify = fopen(map_data_file.c_str(), "w");
	if (!modify) {
		ROS_ERROR("Couldn't save map file to %s", map_data_file.c_str());
		return;
	}

	fprintf(modify, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
			resolution, infoHeader.biWidth, infoHeader.biHeight);
	for(unsigned int y = 0; y < infoHeader.biHeight; y++) {
		for(unsigned int x = 0; x < infoHeader.biWidth; x++) {
			unsigned int i = x + (infoHeader.biHeight - y - 1) * infoHeader.biWidth;
			fputc(map[i], modify);
		}
	}

	fclose(modify);

	PublishMsgsCmd("map modify finish");
	ROS_WARN("map modify finish");	
}

/**
 * @brief 控制标定入口回调
 * @param msg-指令参数字符串
 * @return 
 * */
void deskmedia_node::GetDemarcateCallback(const deskmedia::DMDema::ConstPtr& msg)
{	 
	float dema_target = 0.0;
	deskmedia::DMDemarcateGoal goal;
	ROS_WARN("get demarcate cmd: %s", msg->type.c_str());
	if (strstr(msg->type.c_str(), "move") != NULL) {
		goal.type = 1;
		move_target_ = msg->target;
		dema_target = move_target_;
	} else if (strstr(msg->type.c_str(), "spin") != NULL) {
		goal.type = 2;
		spin_target_ = msg->target;
		dema_target = spin_target_ / RADIAN_TO_DEGREE;
	} else {
		return ;
	}

	goal.target = dema_target;
	DemarcateClient_ dc("demarcate", true);
//	dc.sendGoal(goal, &deskmedia_node::DemarcateDoneCb, &deskmedia_node::DemarcateActiveCb, &deskmedia_node::DemarcateFeedbackCb, this);
	ros::spin(); 
}

/**
 * @brief 获取原始odom计算里程计数据
 * @param odom-里程计信息
 * @return 
 * */
void deskmedia_node::GetOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	if (IsUpdateOdomPose(odom->pose.pose)) {
		g_pose_.pose = odom->pose.pose;
		g_odom_yaw_ = QuaternionTransToYaw(odom->pose.pose.orientation);
//		ROS_WARN("odom pose (%f, %f) yaw: %f, degree:%f", odom->pose.pose.position.x, odom->pose.pose.position.y, g_odom_yaw_, g_odom_yaw_ * RADIAN_TO_DEGREE);
//		if (g_ros_status_ == ROS_STATUS_MAPPING)
//			PublishPoseCmd(odom->pose.pose.position.x, odom->pose.pose.position.y, g_odom_yaw_ * RADIAN_TO_DEGREE);
	}
}

/**
 * @brief 获取amcl计算出的坐标数据
 * @param pose-位置信息
 * @return 
 * */
void deskmedia_node::GetAMCLPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
	if (g_save_amcl_) {
		g_save_amcl_ = false;
		g_amcl_pose_.pose = pose->pose;
		g_amcl_yaw_ = QuaternionTransToYaw(pose->pose.pose.orientation);
		SaveGoalPoseToFile(2, pose->pose.pose.position.x, pose->pose.pose.position.y, g_amcl_yaw_ * RADIAN_TO_DEGREE);
	}
	if (IsUpdateAmclPose(pose->pose.pose)) {
		g_amcl_pose_.pose = pose->pose;
		g_amcl_yaw_ = QuaternionTransToYaw(pose->pose.pose.orientation);
		// ROS_WARN("cur_x= %f, cur_y= %f, cur_yaw= %f", pose->pose.pose.position.x, pose->pose.pose.position.y, g_amcl_yaw_);
		ROS_WARN("amcl pose (%f, %f) yaw: %f, degree: %f)", pose->pose.pose.position.x, pose->pose.pose.position.y, g_amcl_yaw_, g_amcl_yaw_ * RADIAN_TO_DEGREE);
		if (g_ros_status_ != ROS_STATUS_MAPPING)
			PublishPoseCmd(pose->pose.pose.position.x, pose->pose.pose.position.y, g_amcl_yaw_ * RADIAN_TO_DEGREE);
	}
}

/**
 * @brief 获取ekf融合计算出的坐标数据
 * @param pose-融合位置信息
 * @return 
 * */
void deskmedia_node::GetOdomCombPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
	if (IsUpdateOdomCombPose(pose->pose.pose)) {
		g_ekf_pose_.pose = pose->pose;
		g_ekf_yaw_ = QuaternionTransToYaw(pose->pose.pose.orientation);
		GetDevicePose(g_tf_pose_);
//		ROS_WARN("ekf pose (%f, %f) yaw: %f, degree: %f)", pose->pose.pose.position.x, pose->pose.pose.position.y, g_ekf_yaw_, g_ekf_yaw_ * RADIAN_TO_DEGREE);
		if (g_ros_status_ == ROS_STATUS_MAPPING) {
			// GetDevicePose(g_tf_pose_);
			PublishPoseCmd(g_tf_pose_.pose.position.x, g_tf_pose_.pose.position.y, QuaternionTransToYaw(g_tf_pose_.pose.orientation) * RADIAN_TO_DEGREE); //和ekf的角度有偏差
//			PublishPoseCmd(pose->pose.pose.position.x, pose->pose.pose.position.y, g_ekf_yaw_ * RADIAN_TO_DEGREE);
		}
	}
}

/**
 * @brief 获取导航目标数据
 * @param goal-目标对象
 * @return 
 * */
void deskmedia_node::GetMoveBaseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	// ROS_WARN("get move base goal[%f]", goal->header.stamp.sec + 1e-9 * goal->header.stamp.nsec);
	ROS_WARN("get move base goal[x: %.3f, y: %.3f, z: %.3f], [x: %.3f, y: %.3f, z: %.3f, w: %.3f]", 
		goal->pose.position.x, goal->pose.position.y, goal->pose.position.z,
		goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
	t_goal_.pose.position = goal->pose.position;
	t_goal_.pose.orientation = goal->pose.orientation;
}

/**
 * @brief 获取carto位置信息
 * @param pose-当前坐标
 * @return 
 * */
void deskmedia_node::GetCartographPoseCmdCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	g_carto_pose_.pose = pose->pose;
	g_carto_yaw_ = QuaternionTransToYaw(pose->pose.orientation);
	SaveCartographPoseToFile();
}


/**
 * @brief 获取全局路径
 * @param path-路径
 * @return 
 * */
void deskmedia_node::GetGlobalPlannerCallback(const nav_msgs::Path path)
{
//	geometry_msgs::PoseStamped nearest_goal;

	if(path.poses.empty())
	{
		return;
	}
	path_.clear();

	for (unsigned int i = 0; i < path.poses.size(); i++) {
        path_.push_back(path.poses[i]);
    }	

//	CalcNearestPathPoint(g_tf_pose_, path_, nearest_goal);
//	ComputeVelocityCommands(g_tf_pose_, nearest_goal);
}

/**
 * @brief 获取当前导航目标数据
 * @param goal-目标对象
 * @return 
 * */
void deskmedia_node::GetCurrentGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	ROS_ERROR("get move base current goal[x: %.3f, y: %.3f, z: %.3f], [x: %.3f, y: %.3f, z: %.3f, w: %.3f]", 
		goal->pose.position.x, goal->pose.position.y, goal->pose.position.z,
		goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
}

/**
 * @brief 获取障碍物到baselink的距离
 * @param start_index - 开始雷达激光光束
 * 		  end_index - 结束雷达激光光束
 * @return 返回障碍物到baselink的距离
 * */
float deskmedia_node::GetObstacleToBaselinkDistance(int start_index,int end_index)
{
	float min_distance = 0;
	float *c_range = std::min_element(&g_ranges_[start_index], &g_ranges_[end_index]);		//获取最小值
	float index = std::distance(&g_ranges_[start_index], std::min_element(&g_ranges_[start_index], &g_ranges_[end_index]) ) + start_index; 	//获取index
	float angle = index / 2;		//得到角度
	float angleToRadian = (angle + 180) / RADIAN_TO_DEGREE;		//得到弧度 //laser坐标系 x在180度方向上 y在270度方向上
	geometry_msgs::Quaternion quart;
 	quart = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angleToRadian);	//欧拉角转四元数
	float xx = *c_range * cos(angleToRadian);
	float yy = *c_range * sin(angleToRadian);	//获取坐标

	tf::Stamped<tf::Pose> laser_pos(tf::Pose(tf::Quaternion(quart.x, quart.y, quart.z, quart.w), tf::Vector3(xx, yy, 0.0)) , ros::Time(0), "laser"); 	//最近位置在laser的坐标
	tf::TransformListener transform_listener;
	tf::Stamped<tf::Pose> base_link_pos;
	ros::Time timeout(0.1);
    ros::Rate rate(2);
	try {   
		rate.sleep();
		transform_listener.transformPose("base_link", laser_pos, base_link_pos);//换算到base_link下的坐标
    } catch (tf::TransformException &ex) {
        ROS_WARN("transformPose base_link error,%s", ex.what());
        return 0;
    }
	min_distance = sqrt(pow(base_link_pos.getOrigin().x(), 2) + pow(base_link_pos.getOrigin().y(), 2)); //计算障碍物 到原点距离
	ROS_WARN("*c_range= %f, angle = %f, laser_xx = %f, laser_yy = %f, min_distance = %f", *c_range, angle, xx, yy, min_distance);

	ROS_WARN("base_link_pos.x()= %f, base_link_pos.y() = %f", base_link_pos.getOrigin().x(), base_link_pos.getOrigin().y());
	return min_distance;
}

/**
 * @brief 从launch配置文件获取初始化数据
 * @param nh-节点句柄
 * @return 
 * */
void deskmedia_node::LoadParameters(ros::NodeHandle& nh)
{
	v_kp_ = 1;
	w_kp_ = 1.5;
	minv_ = 0.05;
	minw_ = 0.15; 
	maxv_ = 0.1;
	maxw_ = 0.1;
	dock_v_ = minv_;
	dock_w_ = minw_;
	last_x_fb_ = 0;
	last_z_fb_ = 0;
	g_infrared_ = 0;
	g_map_index_ = 0;
	g_emb_status_ = 1;
	min_distance_ = 0.4;
	max_distance_ = 0.8;
	g_action_type_ = 1;
	docking_factor_ = 8;
	g_save_amcl_ = false;
	g_compress_flag = 0;
	g_docking_times_ = 0;
	g_planning_times_ = 0;
	g_test_for_vel_print_ = false;
	g_test_for_infrared_print_ = false;
	memset(&g_reconfig_info, 0x00, sizeof(RECONFIGTURE_PARAMS));
	nh.param<float>("move_speed_", move_speed_, 0.025);//移动速度
	nh.param<float>("spin_speed_", spin_speed_, 0.120);//旋转速度
	nh.param<float>("scale_factor_x_", scale_factor_x_, 1.0);//直线运动比例
	nh.param<float>("scale_factor_z_", scale_factor_z_, 1.0);//旋转运动比例
	ROS_ERROR("get default speed move: %f, spin: %f, scale_factor[x: %f, z: %f]", move_speed_, spin_speed_, scale_factor_x_, scale_factor_z_);
}

/**
 * @brief 从系统获取可动态配置参数的初始化值
 * @param 
 * @return true/false：成功/失败
 * */
bool deskmedia_node::GetDynamicReconfigureInit()
{
	std::string dwa_params = GetSysCmdExecResult("rosrun dynamic_reconfigure dynparam get /move_base/DWAPlannerROS");
//	ROS_INFO("get dwa params: %s", dwa_params.c_str());
	std::string dwa_json = ReplaceString(dwa_params, "'", "\"");
//	ROS_INFO("get dwa params: %s", dwa_json.c_str());
	transform(dwa_json.begin(), dwa_json.end(), dwa_json.begin(), ::tolower);
//	ROS_INFO("get dwa params: %s", dwa_json.c_str());

	Json::Value value;
	Json::Reader reader;
	if (reader.parse(dwa_json, value)) {
		if (!value["max_vel_x"].isNull()) {
			g_reconfig_info.dwa_params.max_vel_x = (float)value["max_vel_x"].asDouble();
		}
		if (!value["min_vel_x"].isNull()) {
			g_reconfig_info.dwa_params.min_vel_x = (float)value["min_vel_x"].asDouble();
		}
		if (!value["max_vel_theta"].isNull()) {
			g_reconfig_info.dwa_params.max_vel_theta = (float)value["max_vel_theta"].asDouble();
		}
		if (!value["min_vel_theta"].isNull()) {
			g_reconfig_info.dwa_params.min_vel_theta = (float)value["min_vel_theta"].asDouble();
		}
	} else {
		ROS_WARN("dwa_params analysis failed");
		return false;
	}
	return true;
}


/**
 * @brief 旋转固定角度
 * @param target-角度
 * @return 
 * */
void deskmedia_node::SpinTargetAngle(float target)
{
	ros::Rate r(20);
	ros::NodeHandle n;
	geometry_msgs::Pose t_pose;
	t_pose.position.x = g_pose_.pose.position.x;
	t_pose.position.y = g_pose_.pose.position.y;
	t_pose.orientation.w = g_pose_.pose.orientation.w;

	float t_yaw = QuaternionTransToYaw(g_pose_.pose.orientation);//起始角度t_yaw 当前角度g_yaw 目标角度goal->target
	float turn_angle = 0;//累计旋转角度
	while(n.ok()) {
		float delta_angle=0;//单周期运动角度
		if( target > 0) {
			if(g_ekf_yaw_ >= t_yaw)
				delta_angle = g_ekf_yaw_- t_yaw; 
			else
				delta_angle = g_ekf_yaw_+ 2 * PI - t_yaw;  //PI--->(-PI)
		} else if (target < 0) {
			if(g_ekf_yaw_ <= t_yaw)
				delta_angle = t_yaw - g_ekf_yaw_; 
			else
				delta_angle = t_yaw - g_ekf_yaw_+2*PI;	//(-PI)--->PI
		}
		t_yaw = g_ekf_yaw_;//重新赋值
		turn_angle += delta_angle;//累计旋转角度
		ROS_WARN(" target_angle:%f, turn_angle:%f", target, turn_angle);
		//ROS_WARN(" c:%f, t:%f, g:%f", t_yaw, goal->target, g_yaw_);
		//if (!checkSpinFinish(t_yaw, g_yaw_, goal->target)) 
		if(turn_angle-abs(target) < 0.0001){
			PublishDMMotorCmd(0, 0.15,true);
		} else {
			PublishDMMotorCmd(0, 0,true);
			break;
		}

		ros::spinOnce();
		r.sleep();
	}
}

/**
 * @brief 向前移动固定距离
 * @param target-距离
 * @return 
 * */
void deskmedia_node::MoveTargetDistanceForward(float target)
{
	ros::Rate r(20);
	ros::NodeHandle n;
	geometry_msgs::Pose t_pose;
	t_pose.position.x = g_pose_.pose.position.x;
	t_pose.position.y = g_pose_.pose.position.y;
	while(n.ok() && sqrt(pow(g_pose_.pose.position.x - t_pose.position.x, 2) + pow(g_pose_.pose.position.y - t_pose.position.y, 2)) < target) {			
		if(!PublishDMMotorCmd(0.1, 0,true))
			break;
		ros::spinOnce();
		r.sleep();
	}
	PublishMotorCmd(0, 0);
}

/**
 * @brief 向后移动固定距离
 * @param target-距离
 * @return 
 * */
void deskmedia_node::MoveTargetDistanceBackford(float target)
{
	ros::Rate r(20);
	ros::NodeHandle n;
	geometry_msgs::Pose t_pose;
	t_pose.position.x = g_pose_.pose.position.x;
	t_pose.position.y = g_pose_.pose.position.y;
	while(n.ok() && sqrt(pow(t_pose.position.x - g_pose_.pose.position.x, 2) + pow(t_pose.position.y - g_pose_.pose.position.y, 2)) < target) {			
		PublishDMMotorCmd(-0.05, 0,false); //不检测尾部距离
		ros::spinOnce();
		r.sleep();
	}
	PublishMotorCmd(0, 0);
}


void deskmedia_node::CalcNearestPathPoint(geometry_msgs::PoseStamped &cur_pose, const std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::PoseStamped& goal)
{
	std::vector<float> dx;
    std::vector<float> dy;
    float d[path.size()];

    int index = 0;
	if(path.empty())
	{
		return;
	}

    // 计算当前点到目标点距离
    for (int i = 0; i < path.size(); i++)
    {
        dx.push_back(path[i].pose.position.x - cur_pose.pose.position.x);
        dy.push_back(path[i].pose.position.y - cur_pose.pose.position.y);
        d[i] = std::sqrt(std::pow(dx[i], 2) + std::pow(dy[i], 2));
    }

    // 最短距离索引
    index = std::abs(d - min_element(d, d + path.size())); 

    goal = path[index];
	ROS_INFO("cur pos(%.3f, %.3f), target pos(%.3f, %.3f)", cur_pose.pose.position.x, cur_pose.pose.position.y, goal.pose.position.x, goal.pose.position.y);
}

float deskmedia_node::ComputeMinAngle(const float curyaw, const float goalyaw)
{
	float t_yaw;
	if(abs(goalyaw - curyaw) > PI)
		t_yaw = goalyaw - curyaw - 2 * PI;
	else
		t_yaw = goalyaw - curyaw;

	return t_yaw;
}
void deskmedia_node::ComputeVelocityCommands(geometry_msgs::PoseStamped &cur_pose, geometry_msgs::PoseStamped &goal_pose)
{
//	ROS_WARN(" ComputeVelocityCommands");
	float dx = cur_pose.pose.position.x - goal_pose.pose.position.x;
	float dy = cur_pose.pose.position.y - goal_pose.pose.position.y;
	float tar_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
	float cur_yaw = QuaternionTransToYaw(cur_pose.pose.orientation);
	float goal_yaw = QuaternionTransToYaw(goal_pose.pose.orientation);
	float tar_yaw = ComputeMinAngle(cur_yaw, goal_yaw);
	float v_direction = fabs(tar_yaw) > ( PI /2) ? (-cos(tar_yaw)) : cos(tar_yaw);
	// float v_direction = -cos(tar_yaw) * dx - sin(tar_yaw) * dy;
	float cmd_v = 0;
	float cmd_w = 0;

	if( tar_distance < 0.015f && abs(tar_yaw) < 0.0174f) {
		cmd_v = 0.0f;
		cmd_w = 0.0f;
	} else {	
		if(abs(tar_yaw) >= 0.0174f) {
			cmd_w = tar_yaw * w_kp_ ;
			if(abs(cmd_w) < minw_) cmd_w = SIGN(cmd_w) * minw_;//限制
			if(abs(cmd_w) > maxw_) cmd_w = SIGN(cmd_w) * maxw_;//限制
		} else {
			cmd_w = 0.0f;
		}

		if(tar_distance >= 0.015f) {
			cmd_v = SIGN(v_direction) * tar_distance * v_kp_;
			if(abs(cmd_v) < minv_) cmd_v = SIGN(cmd_v) * minv_;//限制
			if(abs(cmd_v) > maxv_) cmd_v = SIGN(cmd_v) * maxv_;//限制
		} else {
			cmd_v = 0.0f;
		}
	}
	ROS_ERROR("cur (%.3f, %.3f, %.3f) - goal (%.3f, %.3f, %.3f) -> distance = %.3f, tar_yaw = %.3f, v = %.3f, w = %.3f",
		cur_pose.pose.position.x, cur_pose.pose.position.y, cur_yaw,
		goal_pose.pose.position.x, goal_pose.pose.position.y, goal_yaw,
		tar_distance, tar_yaw, cmd_v, cmd_w);
//	ROS_ERROR("ComputeVelocityCommands tar_distance=%f,tar_yaw=%f",tar_distance, tar_yaw);
//	ROS_ERROR("ComputeVelocityCommands cmd_v=%f,cmd_w=%f",cmd_v, cmd_w);
	PublishDMMotorCmd(cmd_v, cmd_w,true);
}

void deskmedia_node::WaitForExec(int times)
{
	ros::Rate r(1);
	ros::NodeHandle n;
	
	while(n.ok() && times--) {
		ros::spinOnce();
		r.sleep();
	}
}

/**
 * @brief 执行系统程序获取对应的返回值
 * @param cmd-需要执行的命令
 * @return 获取到的结果字符串
 * */
std::string deskmedia_node::GetSysCmdExecResult(const std::string & cmd)
{
	FILE *pf = NULL;
	char buffer[10240] = {0};
	std::string strResult;

	if ((pf = popen(cmd.c_str(), "r")) == NULL)
		return "";

	while(fgets(buffer, sizeof(buffer), pf)){
		strResult += buffer;
	}

	pclose(pf);

	return strResult;
}

/**
 * @brief 字符串替换方法
 * @param str-原始字符串 old-待替换的字符 replace-需要替换上的字符
 * @return 新字符串
 * */
std::string& deskmedia_node::ReplaceString(std::string &str, const std::string & old, const std::string & replace)
{
  
	for(string::size_type pos(0); pos != string::npos; pos += replace.length())   
	{
		pos = str.find(old, pos);
		if(pos!=string::npos)     
			str.replace(pos, old.length(), replace);     
		else  
			break;       
	}     
	return str;     
}

/**
 * @brief 控制节点构造函数
 * @param 
 * @return
 * */
deskmedia_node::deskmedia_node(tf2_ros::Buffer& tf) : tf_(tf)
{
	//todo
}

//cartographer位置初始化
int traj_id = 1;
void deskmedia_node::init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = tf::getYaw(msg->pose.pose.orientation);
    ros::NodeHandle nh;

    ros::ServiceClient client_traj_finish = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("finish_trajectory");
    cartographer_ros_msgs::FinishTrajectory srv_traj_finish;
    srv_traj_finish.request.trajectory_id = traj_id;
    if (client_traj_finish.call(srv_traj_finish))
    {
        ROS_INFO("Call finish_trajectory %d success!", traj_id);
    }
    else
    {
        ROS_INFO("Failed to call finish_trajectory service!");
    }

    ros::ServiceClient client_traj_start = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("start_trajectory");
    cartographer_ros_msgs::StartTrajectory srv_traj_start;
    srv_traj_start.request.configuration_directory = "/home/ximei/robot_robot/src/cartographer_ros/cartographer_ros/configuration_files";//.lua文件所在路径
    srv_traj_start.request.configuration_basename = "backpack_2d_localization.lua";//lua文件
    srv_traj_start.request.use_initial_pose = 1;
    srv_traj_start.request.initial_pose = msg->pose.pose;
    srv_traj_start.request.relative_to_trajectory_id = 0;
    printf("&&&&&: %f__%f\n",srv_traj_start.request.initial_pose.position.x,srv_traj_start.request.initial_pose.position.y);
    if (client_traj_start.call(srv_traj_start))
    {
        // ROS_INFO("Status ", srv_traj_finish.response.status)
        ROS_INFO("Call start_trajectory %d success!", traj_id);
        traj_id++;
    }
    else
    {
        ROS_INFO("Failed to call start_trajectory service!");
    }
}


/**
 * @brief 控制节点析构函数
 * @param 
 * @return
 * */
deskmedia_node::~deskmedia_node()
{
	//todo
}

/**
 * @brief 初始化控制节点，进行配置参数的读取、对应话题的订阅和发布话题的创建
 * @param nh-节点句柄
 * @return true/false 成功/失败
 * */
bool deskmedia_node::InitNode(ros::NodeHandle& nh)
{
	//加载参数
	ros::NodeHandle private_nh("~");
	LoadParameters(private_nh);

	// //加载位置信息
	GetGoalPoseFromFile(&g_goal_a_, &g_goal_b_);
	system("ps -ef | grep SimpleHTTPServer | grep -v \"color\" | cut -c 9-15 | xargs kill -9");
	
	//原生发布节点的创建
	cmd_vel_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	pub_cancel_ = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
	pub_update_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
	plan_pub_  =  nh.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 1);

	//自定义发布节点的创建
	pub_map_ = nh.advertise<deskmedia::DMMap>("/dm_map", 1);
	pub_move_ = nh.advertise<std_msgs::String>("/move_stop", 1);
	pub_pose_ = nh.advertise<deskmedia::DMPose>("/dm_pose", 1);
	pub_msgs_ = nh.advertise<std_msgs::String>("/dm_msgs", 1);
	pub_error_ = nh.advertise<std_msgs::String>("/dm_error", 1);
	pub_clear_ = nh.advertise<std_msgs::String>("/dm_clear_cache", 1);
	pub_status_ = nh.advertise<std_msgs::UInt32>("/dm_ros_status", 1);
	set_x_scale_ = nh.advertise<std_msgs::Float32>("/set_scale_x", 1);
	set_z_scale_ = nh.advertise<std_msgs::Float32>("/set_scale_z", 1);
	pub_location_ = nh.advertise<std_msgs::UInt8>("/dm_location", 1);
	pub_infrared_ = nh.advertise<std_msgs::UInt8>("/infrared_signal", 1);
	pub_sync_flag_ = nh.advertise<std_msgs::UInt8>("/dm_sync_data", 1);
	pub_debug_params_ = nh.advertise<std_msgs::String>("/pub_debug_params", 1);
	
	//原生话题的订阅
	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &deskmedia_node::GetOdomCallback, this);
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &deskmedia_node::GetAMCLPoseCallback, this); 
	ros::Subscriber carto_sub = nh.subscribe<geometry_msgs::PoseStamped>("/tracked_pose", 1, &deskmedia_node::GetCartographPoseCmdCallback, this); 
	ros::Subscriber cur_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base/current_goal", 1, &deskmedia_node::GetCurrentGoalCallback, this);
	ros::Subscriber ros_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &deskmedia_node::GetMoveBaseGoalCallback, this);
	ros::Subscriber odom_comb_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined", 1, &deskmedia_node::GetOdomCombPoseCallback, this);
	ros::Subscriber global_planner_sub = nh.subscribe<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 1, &deskmedia_node::GetGlobalPlannerCallback, this);
	ros::Subscriber _pose_init_sub = nh.subscribe("/initialpose", 1, &deskmedia_node::init_pose_callback,this);//cartographer rviz定位

	//自定义话题的订阅
	ros::Subscriber di_sub = nh.subscribe("/di_status", 1, &deskmedia_node::GetDiStatusCallback, this);
	ros::Subscriber ir_sub = nh.subscribe("/ir_status", 1, &deskmedia_node::GetIrStatusCallback, this);
	ros::Subscriber water_sub = nh.subscribe("/water_status", 1, &deskmedia_node::GetWaterStatusCmdCallback, this);
	ros::Subscriber debug_sub = nh.subscribe("/debug_params", 1, &deskmedia_node::GetDynamicReconfigureCmdCallback, this);
	
	ros::Subscriber vel_sub = nh.subscribe("/dm_vel", 1, &deskmedia_node::GetVelCallback, this);
	ros::Subscriber map_sub = nh.subscribe("/dm_mapping", 1, &deskmedia_node::GetMappingCmdCallback, this);
	ros::Subscriber ctl_sub = nh.subscribe("/dm_control", 1, &deskmedia_node::GetControlCallback, this);
	ros::Subscriber spin_sub = nh.subscribe("/move_spin", 1, &deskmedia_node::GetSpinCallback, this);	
	ros::Subscriber move_sub = nh.subscribe("/move_distance", 1, &deskmedia_node::GetMoveCallback, this);
	ros::Subscriber acts_sub = nh.subscribe("/actual_spin", 1, &deskmedia_node::GetActualSpinCallback, this);
	ros::Subscriber actm_sub = nh.subscribe("/actual_distance", 1, &deskmedia_node::GetActualMoveCallback, this);
	ros::Subscriber goal_sub = nh.subscribe("/dm_goal", 1, &deskmedia_node::GetGoalCallback, this);
	ros::Subscriber test_sub = nh.subscribe("/dm_test", 1, &deskmedia_node::GetTestCmdCallback, this);			
	ros::Subscriber speed_sub = nh.subscribe("/cmd_vel", 1, &deskmedia_node::GetCmdVelCallback, this);	
	ros::Subscriber laser_sub = nh.subscribe("/scan", 1, &deskmedia_node::GetScanDataCallback, this);	
	ros::Subscriber src_x_sub = nh.subscribe("/dm_src_x", 1, &deskmedia_node::GetSrcXVelDataCallback, this);
	ros::Subscriber src_z_sub = nh.subscribe("/dm_src_z", 1, &deskmedia_node::GetSrcZVelDataCallback, this);	
	ros::Subscriber control_sub = nh.subscribe("/dm_control", 1, &deskmedia_node::GetControlCmdCallback, this);
	ros::Subscriber set_map_sub = nh.subscribe("/dm_set_map", 1, &deskmedia_node::GetModifyMapCmdCallback, this);	
	ros::Subscriber mapping_sub = nh.subscribe("/map", 1, &deskmedia_node::GetMapDataCallbackEx, this); 
	ros::Subscriber pose_get_sub = nh.subscribe("/dm_pose_get", 1, &deskmedia_node::GetPoseCallback, this);
	ros::Subscriber pose_set_sub = nh.subscribe("/dm_pose_set", 1, &deskmedia_node::SetPoseCallback, this);
	ros::Subscriber register_sub = nh.subscribe("/dm_register", 1, &deskmedia_node::GetRegisterCallback, this);
	ros::Subscriber demarcate_sub = nh.subscribe("/dm_demarcate", 1, &deskmedia_node::GetDemarcateCallback, this); 
	ros::Subscriber rob_status_sub = nh.subscribe("/robot_status", 1, &deskmedia_node::GetEmbRosStatusCmdCallback, this);	
	ros::Subscriber get_params_sub = nh.subscribe("/dm_get_params", 1, &deskmedia_node::GetRobotParamsCmdCallback, this);	
	ros::Subscriber get_status_sub = nh.subscribe("/dm_get_status", 1, &deskmedia_node::GetRosStatusToUpperCmdCallback, this);	
	ros::Subscriber navigation_sub = nh.subscribe("/dm_navigation", 1, &deskmedia_node::GetNavigationCallback, this);
	ros::Subscriber navigation_result_sub = nh.subscribe("/msg_error", 1, &deskmedia_node::GetErrorCallback, this);

	ds_ = new DemarcateServer_(ros::NodeHandle(), "demarcate", boost::bind(&deskmedia_node::DemarcateExecute, this, _1), false);
	ds_->start();
	g_ros_status_ = ROS_STATUS_FREE;
	//判断地图文件存在，则自动进入导航模式
	// if (access("/home/ximei/robot_robot/src/turn_on_wheeltec_robot/map/WHEELTEC.pgm", F_OK ) != -1) {
	// 	g_nav_ = NAV_START;
	// 	g_ros_status_ = ROS_STATUS_NAVIGATION_FREE;
	// 	system("ps -ef | grep SimpleHTTPServer | grep -v \"color\" | cut -c 9-15 | xargs kill -9");
	// 	system("roslaunch turn_on_wheeltec_robot dm_navigation.launch &");
	// } else {
	// 	ROS_WARN("map file does ont exist!"); 
	// }
	
	signal(SIGINT, Shutdown);
	ros::spin();
}

