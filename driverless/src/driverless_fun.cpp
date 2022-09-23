#include "driverless/driverless_node.h"
#define __NAME__ "driverless"

/* @function 在此文件中定义AutoDrive初始化成员函数、功能函数等
 *           在driverless_node.cpp对关键函数进行定义，分布存储以提高可读性
 */

AutoDrive::AutoDrive():
	AutoDriveBase(__NAME__),
    task_running_(false),
	system_state_(State_Idle),
	last_system_state_(State_Idle),
	has_new_task_(false),
	request_listen_(false),
	as_(nullptr)
{
	controlCmd1_.set_driverlessMode = true;
	controlCmd1_.set_handBrake = false;
	controlCmd2_.set_gear = controlCmd2_.GEAR_DRIVE;
	controlCmd2_.set_speed = 0.0;
	controlCmd2_.set_brake = 0.0;
	controlCmd2_.set_roadWheelAngle = 0.0;
}

bool AutoDrive::init(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
	if(is_initialed_) return true; // 避免重复初始化

	nh_ = nh;
	nh_private_ = nh_private;

	// 获取参数
	std::string odom_topic = nh_private_.param<std::string>("odom_topic", "/gps_odom");
	
	// 初始化故障诊断发布器
	initDiagnosticPublisher(nh_, __NAME__);

	if(!loadVehicleParams())
	{
		ROS_ERROR("[%s] Failed to load vehicle parameters!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to load vehicle parameters!");
		return false;
	}

	// 订阅公用传感器数据
	sub_odom_ = nh_.subscribe(odom_topic, 1, &AutoDrive::odom_callback, this);
	sub_vehicleState1_ = nh_.subscribe("/vehicleState1", 1, &AutoDrive::vehicleState1_callback, this); // 档位
	sub_vehicleState2_ = nh_.subscribe("/vehicleState2", 1, &AutoDrive::vehicleState2_callback, this); // 轮速
	sub_vehicleState4_ = nh_.subscribe("/vehicleState4", 1, &AutoDrive::vehicleState4_callback, this); // 转角

	// 发布控制信号与诊断信息
	pub_cmd1_ = nh_.advertise<ant_msgs::ControlCmd1>("/controlCmd1", 1);
	pub_cmd2_ = nh_.advertise<ant_msgs::ControlCmd2>("/controlCmd2", 1);
	pub_diagnostic_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("/driverless/diagnostic", 1);
	
	// 定时器
	cmd1_timer_ = nh_.createTimer(ros::Duration(0.02), &AutoDrive::sendCmd1_callback, this, false, false);
	cmd2_timer_ = nh_.createTimer(ros::Duration(0.01), &AutoDrive::sendCmd2_callback, this, false, false);
	
	// 车辆状态检查，等待初始化
	while(ros::ok())
	{
		bool ok = true;
		if(!vehicle_state_.isSpeedValid())
		{
			ROS_ERROR("[%s] Vehicle speed invalid!", __NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Vehicle speed invalid!");
			ok = false;
		}
		if(!vehicle_state_.isSteerAngleValid())
		{
			ROS_ERROR("[%s] Vehicle steer angle invalid!", __NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Vehicle steer angle invalid!");
			ok = false;
		}
		if(!vehicle_state_.isPoseValid())
		{
			ROS_ERROR("[%s] Vehicle pose invalid!", __NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Vehicle pose invalid!");
			ok = false;
		}
		if(!ok)
		{
			ROS_INFO("[%s] Retrying...", __NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Retrying...");
			ros::Duration(0.5).sleep();
		}
		else
		{
			ROS_INFO("[%s] Vehicle is ready.", __NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Vehicle is ready.");
			break;
		}
	}

	// 初始化自动驾驶请求服务器
	as_  = new DoDriverlessTaskServer(nh_, "do_driverless_task", 
    	boost::bind(&AutoDrive::executeDriverlessCallback, this, _1), false);
    as_->start();

    // 初始化局部路径规划控制器
    if(!planner_.init(nh_, nh_private_))
	{
		ROS_ERROR("[%s] Failed to init path planner!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to init path planner!");
		return false;
	}
	else
	{
        ROS_INFO("[%s] Successfully init path planner.", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Successfully init path planner.");
	}

	// 初始化路径跟踪控制器
    if(!tracker_.init(nh_, nh_private_))
	{
		ROS_ERROR("[%s] Failed to init path tracker!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to init path tracker!");
		return false;
	}
	else
	{
        ROS_INFO("[%s] Successfully init path tracker.", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Successfully init path tracker.");
	}

	// 启动工作线程，等待新任务唤醒
	switchSystemState(State_Idle);
	std::thread t(&AutoDrive::workingThread, this);
	t.detach();
	
	is_initialed_ = true;
	return true;
}


/* @brief 切换系统状态
 *        根据系统状态设置档位，直到档位设置成功
 */
void AutoDrive::switchSystemState(int state)
{
	ROS_INFO("[%s] Switch system state: %s.", __NAME__, StateName[state].c_str());
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Switch system state: " + StateName[state] + ".");

	if(system_state_ == state) return; // 防止重复操作
	
	last_system_state_ = system_state_;
    system_state_ = state;

    // 状态为前进，自动驾驶模式开，档位置D
	if(state == State_Drive) 
	{
		if(isDriveGear())
		{
			cmd2_mutex_.lock(); // 确保指令正确，防止指令已更改状态未更新
			controlCmd2_.set_gear = controlCmd2_.GEAR_DRIVE;
			cmd2_mutex_.unlock();
			return;
		}

		cmd1_mutex_.lock();
		controlCmd1_.set_driverlessMode = true;
		controlCmd1_.set_handBrake = false;
		cmd1_mutex_.unlock();

		cmd2_mutex_.lock();
		controlCmd2_.set_gear = controlCmd2_.GEAR_DRIVE;
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		controlCmd2_.set_roadWheelAngle = 0.0;
		controlCmd2_.set_emergencyBrake = false;
		cmd2_mutex_.unlock();

        // 启动控制指令发送
		setSendControlCmdEnable(true);

		// 等待档位切换成功
        waitGearOk(ant_msgs::State1::GEAR_DRIVE);
	}
    // 状态为后退，自动驾驶模式开，档位置R
	else if(state == State_Reverse) 
	{
		if(isReverseGear()) 
		{
			cmd2_mutex_.lock(); // 确保指令正确，防止指令已更改状态未更新
			controlCmd2_.set_gear = controlCmd2_.GEAR_REVERSE;
			cmd2_mutex_.unlock();
			return;
		}

		cmd1_mutex_.lock();
		controlCmd1_.set_driverlessMode = true;
		controlCmd1_.set_handBrake = false;
		cmd1_mutex_.unlock();

		cmd2_mutex_.lock();
		controlCmd2_.set_gear = controlCmd2_.GEAR_REVERSE;
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		controlCmd2_.set_roadWheelAngle = 0.0;
		controlCmd2_.set_emergencyBrake = false;
		cmd2_mutex_.unlock();

        // 启动控制指令发送
		setSendControlCmdEnable(true);

		// 等待档位切换成功
        waitGearOk(ant_msgs::State1::GEAR_REVERSE);
	}
    // 状态为空闲，停止发送控制指令
	else if(state == State_Idle)
	{
		cmd2_mutex_.lock();
		controlCmd2_.set_gear = controlCmd2_.GEAR_INITIAL;
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		controlCmd2_.set_roadWheelAngle = 0.0;
		controlCmd2_.set_emergencyBrake = false;
		cmd2_mutex_.unlock();

		setSendControlCmdEnable(true);
	}
    // 状态为停止，自动驾驶模式开, 速度置零，拉手刹
    // 车辆停止后，切换为空挡
	else if(state == State_Stop)  
	{
		cmd1_mutex_.lock();
		controlCmd1_.set_driverlessMode = true;
		controlCmd1_.set_handBrake = false; // true拉手刹
		cmd1_mutex_.unlock();

		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0; 
		// controlCmd2_.set_brake = 60;
		controlCmd2_.set_brake = 100;
		controlCmd2_.set_roadWheelAngle = 0.0;
		controlCmd2_.set_emergencyBrake = false;
		cmd2_mutex_.unlock();
		setSendControlCmdEnable(true);

		waitSpeedZero(); // 等待汽车速度为0
		
        cmd2_mutex_.lock(); // 指令预设为N档
		controlCmd2_.set_gear = controlCmd2_.GEAR_NEUTRAL;
        cmd2_mutex_.unlock();
		
        ROS_INFO("[%s] Set gear: GEAR_NEUTRAL.", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Set gear: GEAR_NEUTRAL.");

		// 等待正在执行的任务彻底退出后，将系统置为空闲
		while(task_running_) 
		{
			ROS_INFO("Waiting %s exit...", StateName[system_state_].c_str());
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Waiting " + StateName[system_state_] + " exit...");
			ros::Duration(0.05).sleep();
		}
        switchSystemState(State_Idle); //递归调用，状态置为空闲!!!!
	}
    // 准备切换到前进状态
    else if(state == State_SwitchToDrive)
    {
    	setSendControlCmdEnable(true);
        // 已经为D档，直接返回
        if(isDriveGear())
        {
        	cmd2_mutex_.lock(); // 确保指令正确，防止指令已更改状态未更新
			controlCmd2_.set_gear = controlCmd2_.GEAR_DRIVE;
			cmd2_mutex_.unlock();
        	system_state_ = State_Drive;
        	return;
        }
        
        // 非D档，速度置0，然后切D档
        cmd1_mutex_.lock();
		controlCmd1_.set_driverlessMode = true;
		controlCmd1_.set_handBrake = false;
		cmd1_mutex_.unlock();

		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		cmd2_mutex_.unlock();

        waitSpeedZero();
        switchSystemState(State_Drive); // 递归调用，状态置为前进
    }
    // 准备切换到倒车状态
    else if(state == State_SwitchToReverse)
    {
    	setSendControlCmdEnable(true);
        // 已经为R档，直接返回
        if(isReverseGear())
        {
        	cmd2_mutex_.lock(); // 确保指令正确，防止指令已更改状态未更新
			controlCmd2_.set_gear = controlCmd2_.GEAR_REVERSE;
			cmd2_mutex_.unlock();
        	system_state_ = State_Reverse;
        	return;
        }
        
        // 非R档，速度置0，然后切R档
        cmd1_mutex_.lock();
		controlCmd1_.set_driverlessMode = true;
		controlCmd1_.set_handBrake = false;
		cmd1_mutex_.unlock();

		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		cmd2_mutex_.unlock();

        waitSpeedZero();
        switchSystemState(State_Reverse); // 递归调用，状态置为倒车
    }
	else if(state == State_ForceExternControl)
	{
		// No operation
	}
}

void AutoDrive::setSendControlCmdEnable(bool flag)
{
	static bool last_flag = false;
	if(flag == last_flag)
		return;
	last_flag = flag;

	if(flag)
	{
		cmd1_timer_.start();
		cmd2_timer_.start();
	}
	else
	{
		cmd1_timer_.stop();
		cmd2_timer_.stop();
	}
}

void AutoDrive::sendCmd1_callback(const ros::TimerEvent&)
{
	std::lock_guard<std::mutex> lock(cmd1_mutex_);
	pub_cmd1_.publish(controlCmd1_);
}

void AutoDrive::sendCmd2_callback(const ros::TimerEvent&)
{
	std::lock_guard<std::mutex> lock(cmd2_mutex_);
	pub_cmd2_.publish(controlCmd2_);
}

void AutoDrive::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	Pose pose;
	pose.x = msg->pose.pose.position.x;
	pose.y = msg->pose.pose.position.y;
	pose.yaw = msg->pose.covariance[0];
	vehicle_state_.setPose(pose);
}

void AutoDrive::vehicleState1_callback(const ant_msgs::State1::ConstPtr& msg)
{
	vehicle_state_.setGear(msg->act_gear);
}

void AutoDrive::vehicleState2_callback(const ant_msgs::State2::ConstPtr& msg)
{
	if(msg->vehicle_speed > 20.0)
	{
		vehicle_state_.speed_validity = false;
		return;
	}
	vehicle_state_.setSpeed(msg->vehicle_speed); // m/s
	vehicle_state_.speed_validity = true;
}

void AutoDrive::vehicleState4_callback(const ant_msgs::State4::ConstPtr& msg)
{
	vehicle_state_.setSteerAngle(msg->roadwheelAngle);
	vehicle_state_.steer_validity = true;
}

bool AutoDrive::isReverseGear()
{
	return vehicle_state_.getGear() == ant_msgs::State1::GEAR_REVERSE;
}

bool AutoDrive::isDriveGear()
{
	return (vehicle_state_.getGear() == ant_msgs::State1::GEAR_DRIVE);
}

bool AutoDrive::isNeutralGear()
{
	return vehicle_state_.getGear() == ant_msgs::State1::GEAR_NEUTRAL;
}

bool AutoDrive::isGpsPointValid(const GpsPoint& point)
{
	if(fabs(point.x) > 100 && fabs(point.y) > 100)
		return true;
	else
		return false;
}

bool AutoDrive::loadVehicleParams()
{
	bool ok = true;
	vehicle_params_.max_roadwheel_angle = nh_private_.param<float>("vehicle/max_roadwheel_angle", 0.0);
	vehicle_params_.min_roadwheel_angle = nh_private_.param<float>("vehicle/min_roadwheel_angle", 0.0);
	vehicle_params_.min_radius = nh_private_.param<float>("vehicle/min_radius", 0.0);
	vehicle_params_.max_speed = nh_private_.param<float>("vehicle/max_speed", 0.0);
	vehicle_params_.wheel_base = nh_private_.param<float>("vehicle/wheel_base", 0.0);
	vehicle_params_.wheel_track = nh_private_.param<float>("vehicle/wheel_track", 0.0);
	vehicle_params_.width = nh_private_.param<float>("vehicle/width", 0.0);
	vehicle_params_.length = nh_private_.param<float>("vehicle/length", 0.0);
	std::string node = ros::this_node::getName();
	if(vehicle_params_.max_roadwheel_angle == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/max_roadwheel_angle.", __NAME__, node.c_str());
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "No parameter " + node + "/vehicle/max_roadwheel_angle.");
		ok = false;
	}
	if(vehicle_params_.min_roadwheel_angle == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/min_roadwheel_angle.", __NAME__, node.c_str());
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "No parameter " + node + "/vehicle/min_roadwheel_angle.");
		ok = false;
	}
	if(vehicle_params_.min_radius == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/min_radius.", __NAME__, node.c_str());
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "No parameter " + node + "/vehicle/min_radius.");
		ok = false;
	}
	if(vehicle_params_.max_speed == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/max_speed.", __NAME__, node.c_str());
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "No parameter " + node + "/vehicle/max_speed.");
		ok = false;
	}
	if(vehicle_params_.wheel_base == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/wheel_base.", __NAME__, node.c_str());
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "No parameter " + node + "/vehicle/wheel_base.");
		ok = false;
	}
	if(vehicle_params_.wheel_track == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/wheel_track.", __NAME__, node.c_str());
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "No parameter " + node + "/vehicle/wheel_track.");
		ok = false;
	}
	if(vehicle_params_.width == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/width.", __NAME__, node.c_str());
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "No parameter " + node + "/vehicle/width.");
		ok = false;
	}
	if(vehicle_params_.length == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/length.", __NAME__, node.c_str());
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "No parameter " + node + "/vehicle/length.");
		ok = false;
	}
	if(ok) vehicle_params_.validity = true;
	return ok;
}

/* @brief 载入前进任务文件（路径点位信息/停车点信息/拓展路径）
 */
bool AutoDrive::loadDriveTaskFile(const std::string& file, const bool& reverse)
{
	// 载入路网文件
	if(!loadPathPoints(__NAME__, file, reverse, global_path_))
	{
	    ROS_ERROR("[%s] Failed to load path file!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to load path file!");
		return false;
	}

	// 载入路径附加信息
	if(!reverse)
	{
		std::string path_infos_file = file.substr(0, file.find_last_of(".")) + "_info.xml";
		if(!loadPathAppendInfos(__NAME__, path_infos_file, global_path_))
		{
			ROS_ERROR("[%s] Failed to load path infomation!", __NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to load path infomation!");
		}
	}
	
	return true;
}

/* @brief 等待车速减为0
 */
void AutoDrive::waitSpeedZero()
{
    while(ros::ok() && vehicle_state_.getSpeed(LOCK) != 0.0)
		ros::Duration(0.2).sleep();
}

/* @brief 等待档位切换成功
 */
void AutoDrive::waitGearOk(int gear)
{
	int try_cnt = 0;
    while(ros::ok() && vehicle_state_.getGear() != gear && system_state_ != State_Idle)
    {
		ros::Duration(0.2).sleep();
		ROS_INFO("[%s] Waiting for gear: %d...", __NAME__, gear);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Waiting for gear: " + std::to_string(gear) + "...");
	}
}
