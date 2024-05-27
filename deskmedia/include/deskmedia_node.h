#ifndef __DESKMEDIA_NODE_H_
#define __DESKMEDIA_NODE_H_

#include "ros/ros.h"
#include <json/json.h>
#include <deskmedia/DMVel.h>
#include <deskmedia/DMMap.h>
#include <deskmedia/DMPose.h>
#include <deskmedia/DMDema.h>
#include <deskmedia/DMGoal.h>
#include <deskmedia/DMSetMap.h>
#include <deskmedia/DMCtrlAction.h>
#include "deskmedia/DMDemarcateAction.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <jpeglib.h>
#include <fstream>
#include <signal.h>
#include <iostream>
#include <unistd.h>
#include <algorithm>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/StartTrajectory.h"

using namespace std; 

//Macro definition
//宏定义
#define PI 3.141592
#define MAP_EDGE 10
#define TRY_TIMES 5
#define INFRARED_MODE 1
#define DOCKING_FEATRUE 0.42
#define LOCATION_SPACE 0.4
#define RADIAN_TO_DEGREE 57.324840
#define EMB_FB_DATA_RANGE 0.01
#define SIGN(x) ((x < 0 ) ? -1 : (x > 0))
#define MAP_STORE_PATH "/home/ximei/robot_robot/"
#define DATA_STORE_PATH "/home/ximei/robot_robot/cfg/"

//物理定义

//枚举，结构体
typedef enum _DIR_E_
{
	DIR_UNKNOW = -1,
	DIR_STRAIGHT = 0,
	DIR_BIAS_LEFT = 1,
	DIR_BIAS_RIGHT = 2
}DIR_E;

typedef enum _GOAL_E_ 
{
	GOAL_FREE = 0,
	GOAL_A = 1,
	GOAL_B = 2,

	GOAL_SET_A = 11,
	GOAL_SET_B = 12,

	GOAL_CANCEL = 100,
}GOAL_E;

typedef enum _MAP_E_ 
{
	MAP_UNKNOW = -1,
	MAP_START_BUILD = 0,
	MAP_START_REBUILD = 1,
	MAP_SAVE = 2
}MAP_E;

typedef enum _NAV_E_ 
{
	NAV_UNKNOW = -1,
	NAV_START = 0,
	NAV_FINISH = 1
}NAV_E;

typedef enum _SCALE_TYPE_
{
    SCALE_X = 0,
    SCALE_Z = 1,
}SCALE_TYPE_E;

typedef enum _LOCAT_E_
{
	LOCAT_UNKNOW = -1,
	LOCAT_INIT = 0,
	LOCAT_SEARCH = 1,
	LOCAT_IN_RANGE = 2,
	LOCAT_SPIN = 3,
	LOCAT_MOVE = 4,
	LOCAT_PAUSE = 5,
	LOCAT_FINISH = 6
}LOCAT_E;

typedef enum _IR_SIGNAL_E
{
	IR_NONE = 0,
	IR_RIGHT = 1,
	IR_MID = 2,
	IR_MID_RIGHT = 3,
	IR_LEFT = 4,
	IR_SIDE = 5,
	IR_MMID_LEFT = 6,
	IR_ALL = 7
}IR_SIGNAL_E;

typedef enum _DOCKING_E_ 
{
	DOCKING_INIT = 1,
	DOCKING_GO_AHEAD,
	DOCKING_GO_BACK,
	DOCKING_FIND_MID,
	DOCKING_FIND_LEFT,
	DOCKING_FIND_RIGHT,
	DOCKING_FIND_SIGNAL,
	DOCKING_PAUSE,
	DOCKING_FINISH
}DOCKING_E;

typedef enum _DOCKING_EX_E_ 
{
	DOCKING_READY = 1,
	DOCKING_BACK = 2,
	DOCKING_WATI = 3,
	DOCKING_DOCKING = 4,
	DOCKING_AHEAD = 5,
}DOCKING_EX_E;

typedef enum _ROS_STATUS_E_
{
	ROS_STATUS_FREE = 0,
	ROS_STATUS_MAPPING = 1,
	ROS_STATUS_NAVIGATION_FREE = 2,
	ROS_STATUS_NAVIGATION_MOVE = 3,
	ROS_STATUS_NAVIGATION_CTRL = 4
}ROS_STATUS_E;

// typedef enum _INFRARED_STATUS_E_
// {
// 	INFRARED_STATUS_LEFT = 0,
// 	INFRARED_STATUS_MID = 0,
//     INFRARED_STATUS_RIGHT = 0
// }INFRARED_STATUS_E;

typedef enum _ROBOT_POSITION_E_
{
	ROBOT_POSITION_LEFT = 0,
	ROBOT_POSITION_MID = 0,
    ROBOT_POSITION_RIGHT = 0
}ROBOT_POSITION_E;

typedef enum _INFRARED_STATUS_E_
{
    INFRARED_STATUS_NULL = 0x00,
    INFRARED_STATUS_RIGHTMOST = 0x01,
    INFRARED_STATUS_RIGHT = 0x05,
    INFRARED_STATUS_MID = 0x0d,
    INFRARED_STATUS_LEFT = 0x0c,
	INFRARED_STATUS_LEFTMOST = 0x08
}INFRARED_STATUS_E;

typedef struct _WATER_STATUS_
{
	char terminal[16];
	bool cleanTop;
	bool cleanBottom;
	bool sewageTop;
	bool sewageBottom;
	bool toiletHasWater;
	bool navigation;
}WATER_STATUS_S;

typedef struct _DWA_PARAMS_
{
	float max_vel_x;
	float min_vel_x;
	float max_vel_theta;
	float min_vel_theta;
}DWA_PARAMS;

typedef struct _MOVE_BASE_PARAMS_
{
	
}MOVE_BASE_PARAMS;

typedef struct _RECONFIGTURE_PARAMS_
{
	char module[16];
	DWA_PARAMS dwa_params;
	MOVE_BASE_PARAMS move_params;
	
}RECONFIGTURE_PARAMS;

typedef struct _RGB_QUAD_
{
    char rgbBlue;
    char rgbGreen;
    char rgbRed;
    char rgbReserved;
}RGB_QUAD;

typedef struct _BIT_MAP_FILE_HEADER_  
{
	unsigned short bfType;
	unsigned int   bfSize;
	unsigned short bfResv1;
	unsigned short bfResv2;
	unsigned int   bfOffBits;
}BIT_MAP_FILE_HEADER;

typedef struct _BIT_MAP_INFO_HEADER_
{
	unsigned int   biSize;
	unsigned int   biWidth;
	unsigned int   biHeight;
	unsigned short biPlanes;
	unsigned short biBitCount;
	unsigned int   biCompression;
	unsigned int   biSizeImage;
	unsigned int   biXPelPerMeter;
	unsigned int   biYPelPerMeter;
	unsigned int   biClrUsed;
	unsigned int   biClrImporttant;
}BIT_MAP_INFO_HEADER;

typedef struct _ACTION_DATA_
{
    float v;
    float w;
}ACTION_DATA_S;

typedef struct _DOCKING_INFO_
{
	int left_edge_index;
	int right_edge_index;

	float head_space;
	float left_space;
	float tail_space;
	float right_space;
	
	float left_edge_data;
	float right_edge_data;
	
	float tail_target;
	float edge_target;

	DOCKING_E status;
}DOCKING_INFO_S;

int infratedred[] = {0x00,0x01,0x05,0x0d,0x0c,0x08};

void Shutdown(int sig);

class deskmedia_node
{
    public:
        deskmedia_node(tf2_ros::Buffer& tf);  //Constructor //构造函数
		~deskmedia_node(); //Destructor //析构函数
        bool InitNode(ros::NodeHandle& nh);

    private:
        // DIR_E g_dirct_;
        MAP_E g_map_;
        NAV_E g_nav_;
        // LOCAT_E g_locat_;
        // LOCAT_E last_locat_status_;
        DOCKING_E g_docking_status_;
		DOCKING_E g_last_docking_status_;
        ROS_STATUS_E g_ros_status_;

		WATER_STATUS_S g_water_info;
		DOCKING_INFO_S g_docking_info;
		RECONFIGTURE_PARAMS g_reconfig_info;

		int g_side_;
        int g_edge_x_;
        int g_edge_y_;
        int g_collision_;
        int g_docking_;
        int g_infrared_;
		int g_goal_type_;
        int g_map_index_;
		int g_emb_status_;
		int g_action_type_;//对接类型
		int g_compress_flag;
        int g_docking_times_;
		int g_planning_times_;

		bool g_save_amcl_;

		//打印信息开关
        bool g_test_for_vel_print_;
		bool g_test_for_infrared_print_;

        float last_x_fb_;
        float last_z_fb_;
        float move_speed_;
        float spin_speed_;
        float move_target_;
        float spin_target_;
        float scale_factor_x_;
        float scale_factor_z_;
        float docking_factor_;

        float head_space_;
        float tail_space_;
        float left_space_;
        float right_space_;
		float last_space_;
		float back_target_;

        float g_ekf_yaw_;
        float g_odom_yaw_;
        float g_amcl_yaw_;
		float g_carto_yaw_;
        // float locat_move_;
        // float locat_spin_;
        // float locat_start_yaw_;

        // std::queue<int> infrared_queue_;
        std::vector<float> g_ranges_;
        float minv_;
        float minw_;  
		float maxv_;
		float maxw_;
        float min_distance_;
        float max_distance_;
        float v_kp_;
        float w_kp_;
		float dock_v_;
        float dock_w_;  

		tf2_ros::Buffer& tf_;

        deskmedia::DMPose g_goal_a_;
        deskmedia::DMPose g_goal_b_;
        deskmedia::DMPose g_goal_t_;
        deskmedia::DMPose g_dock_goal_;
        deskmedia::DMPose g_left_edge_;
        deskmedia::DMPose g_right_edge_;
        ros::Publisher plan_pub_;
        ros::Publisher cmd_vel_;
        ros::Publisher pub_map_;
        ros::Publisher pub_pose_;
        ros::Publisher pub_move_;
        ros::Publisher pub_msgs_;
        ros::Publisher pub_goal_;
        ros::Publisher pub_error_;
		ros::Publisher pub_clear_;
		ros::Publisher pub_update_;
        ros::Publisher pub_status_;
		ros::Publisher pub_cancel_;
        ros::Publisher set_x_scale_;
        ros::Publisher set_z_scale_;
		ros::Publisher pub_location_;
        ros::Publisher pub_infrared_;
		ros::Publisher pub_sync_flag_;
		ros::Publisher pub_debug_params_;
		ros::NodeHandle g_nh_;
		ros::Subscriber g_mapping_sub_;
        geometry_msgs::PoseStamped t_goal_;
        geometry_msgs::PoseStamped g_pose_;
		geometry_msgs::PoseStamped g_tf_pose_;
		geometry_msgs::PoseStamped g_carto_pose_;
        // geometry_msgs::PoseStamped g_left_edge;
        // geometry_msgs::PoseStamped g_right_edge;
        geometry_msgs::PoseWithCovarianceStamped g_ekf_pose_;
        geometry_msgs::PoseWithCovarianceStamped g_amcl_pose_;
        std::vector<geometry_msgs::PoseStamped> path_;
        

        typedef actionlib::SimpleActionClient<deskmedia::DMCtrlAction> Client_;
        typedef actionlib::SimpleActionServer<deskmedia::DMCtrlAction> Server_;

        typedef actionlib::SimpleActionClient<deskmedia::DMDemarcateAction> DemarcateClient_;
        typedef actionlib::SimpleActionServer<deskmedia::DMDemarcateAction> DemarcateServer_;

        DemarcateServer_ *ds_;

        //cur pose 文件操作 使用一个函数，传入不同的pose
        bool SaveOdomCombPoseToFile();
		bool SaveCartographPoseToFile();
        bool SaveAppoitPoseToFile(geometry_msgs::PoseWithCovarianceStamped set_pose);
        // // bool formatPointToFile();

        //保存/获取目标位置 a,b
        bool SaveGoalPoseToFile(int goal, float x, float y, float z);
        bool GetGoalPoseFromFile(deskmedia::DMPose *goal_a, deskmedia::DMPose *goal_b);

		//保存/获取禁行区域参数
		bool SaveForbidAreaInfoToFile(int sx, int sy, int fx, int fy);
		bool GetForbidAreaInfoFromFile();

		//缓存数据清除
		void clearAllPoseData();
		void clearForbidAreaInfoFile();

        //判断信息是否更新
        bool IsDeviceInArea(float *mid_offset, bool *side, float *vert_offset, float *hori_offset);
        bool IsUpdateOdomPose(geometry_msgs::Pose pose);
        bool IsUpdateAmclPose(geometry_msgs::Pose pose);
        bool IsUpdateOdomCombPose(geometry_msgs::Pose pose);
		bool IsUpdateCartographPose(geometry_msgs::Pose pose);

        //点更新， 区域更新，用于地图更新
        // inline void UpdateUsefullPoint(int x, int y);
        void UpdateMapArea(unsigned char *map, int width, int height, int sx, int sy, int fx, int fy);

        //辅助条件判断
        int  GetDeviceTiltDirection(int degree, bool type);
        int  FindTurningPoint(int from, bool order, float *distance);
		int  FindDockingLocation();
        bool FindDockingFeature();
        bool IsDockingCentered();
        int  IsMoveFinishByAngle(float goal, float curt);
        bool IsMoveFinishByDistance(geometry_msgs::Pose goal, geometry_msgs::Pose curt);
        bool IsDeviceMoved(geometry_msgs::PoseWithCovarianceStamped s, float s_yaw, geometry_msgs::PoseWithCovarianceStamped c, float c_yaw);
        bool GetCenteredDirection();
        bool GetMoveToVerticalAction(ACTION_DATA_S* action);
		void GetMoveToVerticalSingleWheelAction(ACTION_DATA_S* action);
        float FindCenterOffset();
        float FindValidDistance(int index, bool order);
		float GetTailSpaceDistance();
		float GetDeviceTiltOffsetValue(int dirt, int degree);
        float GetObstacleToBaselinkDistance(int start_index, int end_index);

		//坐标变换
		bool GetDevicePose(geometry_msgs::PoseStamped& global_pose);

        //二次定位 状态机处理
        bool DockingTaskInit(ACTION_DATA_S* action);
        bool DockingGoBackAction(ACTION_DATA_S* action);
		bool DockingFindSignalAction(ACTION_DATA_S* action);
        bool DockingFindMiddleAction(ACTION_DATA_S* action);
        bool DockingFindLeftEdgeAction(ACTION_DATA_S* action);
        bool DockingFindRightEdgeAction(ACTION_DATA_S* action);
        //对接入口
        bool DockingTaskAction(ACTION_DATA_S* action);
		//测试入口
		bool DockingTaskActionEx(ACTION_DATA_S* action);
		bool DockingTaskFindSignalAction(ACTION_DATA_S* action);
        

        // //二次定位 action 抽象
        // int CheckMoveToMidFinish(float goal, float curt);
        // bool CheckMoved(geometry_msgs::PoseWithCovarianceStamped s, float s_yaw, geometry_msgs::PoseWithCovarianceStamped c, float c_yaw);
        // bool CheckMidDirt();
        // int CheckVertical(int degree);
        // void MoveToVertical(int result);
        // void MoveToDockingCtrl();
        // float GetNearleastDistanceInRange(float start, float end);
        // int JudgeNextActionEx(const int infraredstatus, const float headscanarea, const float tailscanarea, const float leftscanarea, const float rightscanarea);

        // float MinAngle(float curyaw, float goalyaw);
        // bool RotateController();
        // //新增


        // //todo
        // // int actionFindRange();
        // // bool actionSearch();
        // // bool actionSpin();
        // // int actionMove();
        // // void judgeNextAction();

        // queue<int> getInfraredQueue();
        // int checkInfraredTrend(std::queue<int> infraredstatus, float curw);
        // int findInfrared(std::queue<int> infraredstatus);
        // int judgeNextActionLast(ACTION_DATA_S* action);
        // bool isAscendingOrder(int *array, int size);
        // bool fNearby(float left, float right, float diff);
        // void getInfraredStatus(int infraredstatus, bool clearflag);
        // void InfraredDecision(std::queue<int> infraredstatus);
        // float checkVertical(int degree);
        
        // 标定
        void DemarcateDoneCb(const actionlib::SimpleClientGoalState& state, const deskmedia::DMDemarcateResultConstPtr& result);
        void DemarcateExecute(const deskmedia::DMDemarcateGoalConstPtr& goal);
        void DemarcateActiveCb();
        void DemarcateFeedbackCb(const deskmedia::DMDemarcateFeedbackConstPtr& feedback);
        
        //条件判断
        bool CheckMoveFinish(geometry_msgs::Point c, geometry_msgs::Point g, float t);
        // bool checkSpinFinish(float c, float g, float t);

        //发布
        void PublishMsgsCmd(const char *msgs);
        void PublishPoseCmd(float x, float y, float a);
        void PublishGoalCmd(float x, float y, float z);
		void PublishCancelGoalCmd();
        void PublishMapInfo(nav_msgs::MapMetaData map, int index);
        void PublishSetScale(int type, float scale);
        void PublishErrorCmd(const char *msgs);
		void PublishClearCmd(const char *msgs);
        void PublishMotorCmd(float v, float w);
        void PublishRosStatus(int status);
        void PublishMoveFinish(const char *msgs);
        bool PublishDMMotorCmd(float v, float w,bool check_tail_space_);
		void PublishLocationCmd(int location);
        void PublishInfraredCmd(int infrared);
		void PublishDebugParamsCmd(const char *msgs);

        //数据转换
        float QuaternionTransToYaw(geometry_msgs::Quaternion orientation);

		//数据同步
		void SyncPoseToAmcl(float x, float y, float z);
		void SyncGetDataFlagToTurnOnEmb(bool flag);

        //回调函数
        void GetVelCallback(const deskmedia::DMVel::ConstPtr& msg);
        void GetGoalCallback(const deskmedia::DMGoal::ConstPtr& msg);
		void SetPoseCallback(const deskmedia::DMPose::ConstPtr& msg);
		void GetPoseCallback(const std_msgs::String::ConstPtr& msg);
        void GetErrorCallback(const std_msgs::String::ConstPtr& msg);
        void GetDiStatusCallback(const std_msgs::UInt8::ConstPtr& msg);
		void GetIrStatusCallback(const std_msgs::UInt32::ConstPtr& msg); 
//        void GetRosStatusCmdCallback(const std_msgs::String::ConstPtr& msg);
		void GetWaterStatusCmdCallback(const std_msgs::String::ConstPtr& msg);
		void GetEmbRosStatusCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
		void GetRosStatusToUpperCmdCallback(const std_msgs::String::ConstPtr& msg);
		void GetRobotParamsCmdCallback(const std_msgs::String::ConstPtr& msg);
		void GetDynamicReconfigureCmdCallback(const std_msgs::String::ConstPtr& msg);
        void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
        //控制逻辑回调
        void GetControlCallback(const std_msgs::String::ConstPtr& msg);
        void GetMappingCmdCallback(const std_msgs::String::ConstPtr& msg);
        void GetNavigationCallback(const std_msgs::String::ConstPtr& msg);

        //节点间注册回调
        void GetRegisterCallback(const std_msgs::String::ConstPtr& msg);
        
        // //标定回调
        void GetSpinCallback(const std_msgs::Float32::ConstPtr& msg);
        void GetMoveCallback(const std_msgs::Float32::ConstPtr& msg);
        void GetDemarcateCallback(const deskmedia::DMDema::ConstPtr& msg);
        void GetActualMoveCallback(const std_msgs::Float32::ConstPtr& msg);
        void GetActualSpinCallback(const std_msgs::Float32::ConstPtr& msg);
       
        //修改地图数据
        void GetModifyMapCmdCallback(const deskmedia::DMSetMap::ConstPtr& msg);

        //原生数据查看
        void GetOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void GetCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void GetMapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
        void GetScanDataCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
        void GetSrcXVelDataCallback(const std_msgs::Float32::ConstPtr& msg);
        void GetSrcZVelDataCallback(const std_msgs::Float32::ConstPtr& msg);
        void GetAMCLPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
        void GetOdomCombPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
        void GetMoveBaseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
		void GetCurrentGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
        bool GetRobotPose(geometry_msgs::PoseStamped& global_pose);
        void GetGlobalPlannerCallback(const nav_msgs::Path path);
        void GetMapDataCallbackEx(const nav_msgs::OccupancyGrid::ConstPtr& map);
		void GetCartographPoseCmdCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);

		//控制指令入口
		void GetControlCmdCallback(const std_msgs::String::ConstPtr& msg);

        //调试指令入口
        void GetTestCmdCallback(const std_msgs::String::ConstPtr& msg);

        //配置参数读取入口
        void LoadParameters(ros::NodeHandle& nh);

		//动态配置参数初始化入口
		bool GetDynamicReconfigureInit();

		//移动指令入口
		void SpinTargetAngle(float target);
		void MoveTargetDistanceForward(float target);
		void MoveTargetDistanceBackford(float target);

        //搜索最邻近车体路径上的点geometry_msgs::PoseStamped& global_pose PoseStamped
        void CalcNearestPathPoint(geometry_msgs::PoseStamped& cur_pose, const std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::PoseStamped& goal);
        //最小角度
        float ComputeMinAngle(const float curyaw, const float goalyaw);
        //运行至goal 控制器
        void ComputeVelocityCommands(geometry_msgs::PoseStamped& cur_pose, geometry_msgs::PoseStamped& goal_pose);

		//辅助工具
		void WaitForExec(int times);
		std::string GetSysCmdExecResult(const std::string &cmd);
		std::string& ReplaceString(std::string &str, const std::string &old, const std::string &replace);
    
}; //class deskmedia_node

#endif