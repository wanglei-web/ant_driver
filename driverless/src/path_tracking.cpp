#include "driverless/path_tracking.h"
#define __NAME__ "path_tracking"

PathTracking::PathTracking():
	AutoDriveBase(__NAME__)
{
}

bool PathTracking::init(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;
	
	nh_private_.param<float>("min_foresight_distance_for_drive", min_foresight_distance_for_drive_, 5.0); // m
	nh_private_.param<float>("min_foresight_distance_for_reverse", min_foresight_distance_for_reverse_, 2.0); // m
	nh_private_.param<float>("limit_speed_for_drive", limit_speed_for_drive_, 40); // km/h
	nh_private_.param<float>("limit_speed_for_reverse", limit_speed_for_reverse_, 5); // km/h
	
	nh_private_.param<float>("fd_speed_coefficient", fd_speed_coefficient_, 1.8);
	nh_private_.param<float>("fd_lateral_error_coefficient", fd_lateral_error_coefficient_, 2.0);
	nh_private_.param<float>("max_side_acceleration", max_side_acceleration_, 1.5); // m/s2
	nh_private_.param<float>("max_deceleration", max_deceleration_, 1.0); // m/s2
	
	initDiagnosticPublisher(nh_, __NAME__);

	task_initialized_ = false;
	speed_initialized_ = false;

	is_ready_ = true;
	
	return true;
}

// 启动跟踪线程
bool PathTracking::start()
{
	if(!is_ready_)
	{
		ROS_ERROR("[%s] System is not ready!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "System is not ready!");
		return false;
	}
	if(!vehicle_params_.validity)
	{
		ROS_ERROR("[%s] Vehicle parameters is invalid!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Vehicle parameters is invalid!");
		return false;
	}
	if(!task_initialized_)
	{
		ROS_ERROR("[%s] Task is not initialized!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Task is not initialized!");
		return false;
	}
	if(!speed_initialized_)
	{
		ROS_ERROR("[%s] Speed is not initialized!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Speed is not initialized!");
		return false;
	}


	is_running_ = true;

	if(task_mode_ == DRIVE_MODE)
	{
		cmd1_timer_ = nh_.createTimer(ros::Duration(0.03), &PathTracking::cmd1_timer_callback, this);
		cmd1_check_timer_ = nh_.createTimer(ros::Duration(0.10), &PathTracking::cmd1_check_timer_callback, this);
		cmd2_timer_ = nh_.createTimer(ros::Duration(0.100), &PathTracking::cmd2_timer_callback, this);
		cmd2_check_timer_ = nh_.createTimer(ros::Duration(1.00), &PathTracking::cmd2_check_timer_callback, this);
	}
	else
	{
		cmd2_timer_ = nh_.createTimer(ros::Duration(0.100), &PathTracking::cmd2_timer_callback, this);
		cmd2_check_timer_ = nh_.createTimer(ros::Duration(1.00), &PathTracking::cmd2_check_timer_callback, this);
	}

	return true;
}

void PathTracking::stop()
{
	cmd1_timer_.stop();
	cmd1_check_timer_.stop();
	cmd2_timer_.stop();
	cmd2_check_timer_.stop();
	is_running_ = false;
	
	cmd_mutex_.lock();
	cmd_.speed = 0.0;
	cmd_.roadWheelAngle = 0.0;
	cmd_.validity = false;
	cmd_.speed_validity = false;
	cmd_mutex_.unlock();
}

bool PathTracking::isRunning()
{
	return is_running_;
}

void PathTracking::setTaskMode(const int& task)
{
	if(task != DRIVE_MODE && task != REVERSE_MODE)
	{
		// 只支持 DRIVE_MODE 和 REVERSE_MODE 任务请求
		ROS_ERROR("[%s] Request task error!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Request task error!");
		return;
	}
	task_mode_ = task;
	task_initialized_ = true;
}

void PathTracking::setExpectSpeed(const float& speed)
{
	if(!task_initialized_)
	{
		ROS_ERROR("[%s] Task must be set firstly!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Task must be set firstly!");
		return;
	}

	// speed单位km/h
	float t_speed = speed;
	if(task_mode_ == DRIVE_MODE)
	{
		t_speed = t_speed < limit_speed_for_drive_ ? t_speed : limit_speed_for_drive_;
	}
	else
	{
		t_speed = t_speed < limit_speed_for_reverse_ ? t_speed : limit_speed_for_reverse_;
	}
	expect_speed_ = t_speed / 3.6;
	speed_initialized_ = true;
}

void PathTracking::setEmergencyState(const bool& emergency)
{
	is_emergency_ = emergency;
}

// 定时回调函数（cmd1）
// 控制指令长时间未更新，恢复默认状态
void PathTracking::cmd1_check_timer_callback(const ros::TimerEvent&)
{
	if(ros::Time::now().toSec() - cmd1_time_ > 2.0)
	{
		cmd_mutex_.lock();
		cmd_.turnLight = 0;
		cmd_mutex_.unlock();
	}
}

// 定时回调函数（cmd2）
// 控制指令长时间未更新，有效位置false
void PathTracking::cmd2_check_timer_callback(const ros::TimerEvent&)
{
	if(ros::Time::now().toSec() - cmd2_time_ > 0.2)
	{
		cmd_mutex_.lock();
		cmd_.validity = false;
		cmd_.speed_validity = false;
		cmd_mutex_.unlock();
	}
}

// 定时回调函数（cmd1）
void PathTracking::cmd1_timer_callback(const ros::TimerEvent&)
{
    // 读取局部路径
	local_path_.mutex.lock();
	const Path t_path = local_path_;
	local_path_.mutex.unlock();

	// 记录上一时刻的转向指令
	static uint8_t t_light_last = 0;

	// 等待模式
	static bool wait_mode = false;
	static double wait_time = 0;

	if(t_path.size() == 0)
	{
		cmd1_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.turnLight = 0;
		t_light_last = cmd_.turnLight;
		cmd_mutex_.unlock();
		return;
	}

	if(wait_mode)
	{
		cmd1_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.turnLight = 0;
		t_light_last = cmd_.turnLight;
		cmd_mutex_.unlock();

		if(ros::Time::now().toSec() - wait_time > 1.0)
		{
			wait_mode = false;
			wait_time = 0;
		}
	    return;
	}
	
	if(t_path.turn_ranges.size() == 0)
	{
	    cmd1_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.turnLight = 0;
		t_light_last = cmd_.turnLight;
		cmd_mutex_.unlock();
	    return;
	}
	else
	{
	    // 只考虑最近一个转向区间
	    if(t_path.turn_ranges.ranges[0].start_index == 0) // 车辆处于转向区间内
	    {
	        static int cnt = 0;
	        cnt++;

			uint8_t t_light = t_path.turn_ranges.ranges[0].getCurrentLight();
			if(t_light != t_light_last && t_light != 0 && t_light_last != 0)
			{
				wait_mode = true;
				wait_time = ros::Time::now().toSec();

				cmd1_time_ = ros::Time::now().toSec();
				cmd_mutex_.lock();
				cmd_.turnLight = 0;
				t_light_last = cmd_.turnLight;
				cmd_mutex_.unlock();
				return;
			}
			else
			{
				cmd1_time_ = ros::Time::now().toSec();
				cmd_mutex_.lock();
				cmd_.turnLight = t_light;
				t_light_last = cmd_.turnLight;
				cmd_mutex_.unlock();

				if(cnt >= 10)
				{
					cnt = 0;
					if(t_light != 0)
						ROS_INFO("[%s] Set turn light: %d.", __NAME__, t_light);
						publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Set turn light: " + std::to_string(t_light) + ".");
				}
				return;
			}
	    }
	    else
	    {
	        cmd1_time_ = ros::Time::now().toSec();
		    cmd_mutex_.lock();
		    cmd_.turnLight = 0;
			t_light_last = cmd_.turnLight;
		    cmd_mutex_.unlock();
	        return;
	    }
	}
}

// 定时回调函数（cmd2）
void PathTracking::cmd2_timer_callback(const ros::TimerEvent&)
{
	if(is_emergency_)
	{
		cmd2_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.validity = true;
		cmd_.speed_validity = true;
		cmd_.speed = 0.0;
		cmd_.brake = 100.0;
		cmd_mutex_.unlock();

		return;
	}
	else
	{
		cmd2_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.validity = true;
		cmd_.speed_validity = true;
		cmd_.brake = 0.0;
		cmd_mutex_.unlock();
	}
	
	static size_t cnt = 0;
	
	// 读取车辆状态，创建副本避免多次读取
	const Pose vehicle_pose = vehicle_state_.getPose(LOCK);
	const float vehicle_speed = vehicle_state_.getSpeed(LOCK);

	// 读取局部路径
	local_path_.mutex.lock();
	const Path t_path = local_path_;
	local_path_.mutex.unlock();
	
	if(t_path.points.size() < 5)
	{
		cmd2_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.validity = true;
		cmd_.speed_validity = true;
		cmd_.speed = 0.0;
		cmd_mutex_.unlock();

		return;
	}
	
	// 当路径不包含停车点信息时，结束任务
	if(!t_path.park_points.available()) 
	{
		ROS_ERROR("[%s] No Next Parking Point!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "No Next Parking Point!");
		
		cmd2_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.validity = false;
		cmd_.speed_validity = false;
		cmd_.speed = 0.0;
		cmd_mutex_.unlock();

		return;
	}

	float t_angle_deg = 0.0; // 转向角指令，单位度，右转为负，左转为正
	float t_speed_mps = expect_speed_; // 速度指令，单位m/s

	// 选择路径中自车所在点和终点
	size_t nearest_idx = t_path.pose_index;
	size_t farthest_idx = t_path.final_index;

	// 横向偏差，单位m
	float lat_err = findMinDistance2Path(t_path, vehicle_pose.x, vehicle_pose.y, nearest_idx, nearest_idx, farthest_idx);
	
	// 计算前视距离（预瞄距离）
	// vehicle_speed单位m/s
	float foresight_distance;
	if(task_mode_ == DRIVE_MODE)
	{
		foresight_distance = fd_speed_coefficient_ * vehicle_speed + fd_lateral_error_coefficient_ * fabs(lat_err);
		if(foresight_distance < min_foresight_distance_for_drive_)
			foresight_distance  = min_foresight_distance_for_drive_;
	}
	else
	{
		foresight_distance = min_foresight_distance_for_reverse_;
	}

	// 寻找满足foresight_distance的目标点作为预瞄点
	// 计算自车当前点与预瞄点的dis和yaw
	// 当接近路径终点时，借助路径的延伸阶段正常行驶
	size_t target_idx = nearest_idx + 1;
	GpsPoint target_point = t_path[target_idx];
	std::pair<float, float> dis_yaw = getDisAndYaw(target_point, vehicle_pose);
	while(dis_yaw.first < foresight_distance)
	{
		target_idx++;
		
		// 防止索引溢出
		if(target_idx > t_path.points.size() - 1)
		{
			ROS_INFO("[%s] Path tracking completed.", __NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Path tracking completed.");
			
			cmd2_time_ = ros::Time::now().toSec();
			cmd_mutex_.lock();
			cmd_.validity = false;
			cmd_.speed_validity = false;
			cmd_.speed = 0.0;
			cmd_mutex_.unlock();
			
			return;
		}
        
		target_point = t_path[target_idx];
		dis_yaw = getDisAndYaw(target_point, vehicle_pose);
	}
    
	// 航向偏差，单位rad，限制在[-M_PI, M_PI]
	// 当车辆前进时，自车左偏为负，自车右偏为正
	float yaw_err;
	if(task_mode_ == DRIVE_MODE)
	{
		yaw_err = dis_yaw.second - vehicle_pose.yaw;
	}
	else
	{
		yaw_err = M_PI - (dis_yaw.second - vehicle_pose.yaw);
	}
	yaw_err = normalizeRadAngle(yaw_err);
	
	// 当航向偏差为0时，转向角指令置0，不修改速度指令
	if(sin(yaw_err) == 0)
	{
		cmd2_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.validity = true;
		cmd_.speed_validity = true;
		cmd_.roadWheelAngle = 0.0;
		cmd_mutex_.unlock();

		return;
	}
    
	// 计算转弯半径
	float turning_radius = (0.5 * dis_yaw.first) / sin(yaw_err);

	t_angle_deg = generateRoadwheelAngleByRadius(turning_radius);
	t_angle_deg = limitRoadwheelAngleBySpeed(t_angle_deg, vehicle_speed);
	
	float curvature_search_distance = vehicle_speed * vehicle_speed / (2 * max_deceleration_);
	float max_speed_by_curve = generateMaxSpeedByCurvature(t_path, nearest_idx, curvature_search_distance);
	t_speed_mps = t_speed_mps < max_speed_by_curve ? t_speed_mps : max_speed_by_curve;

	float max_speed_by_park = generateMaxSpeedByParkingPoint(t_path);
	t_speed_mps = t_speed_mps < max_speed_by_park ? t_speed_mps : max_speed_by_park;

	float max_speed_by_traffic_light = generateMaxSpeedByTrafficLightPoint(t_path);
	t_speed_mps = t_speed_mps < max_speed_by_traffic_light ? t_speed_mps : max_speed_by_traffic_light;
	
	float limit_speed = generateMaxSpeedBySpeedRange(t_path);
	t_speed_mps = t_speed_mps < limit_speed ? t_speed_mps : limit_speed;
	
	// 进入限速区间时，设置制动指令，以尽快减速并使制动灯可见
	if(t_path.speed_ranges.size() != 0)
	{
	    if(t_path.speed_ranges.ranges[0].start_index == 0 && vehicle_speed > t_path.speed_ranges.ranges[0].speed + 1.0)
	    {
	        cmd2_time_ = ros::Time::now().toSec();
		    cmd_mutex_.lock();
		    cmd_.validity = true;
		    cmd_.speed_validity = true;
		    cmd_.speed = 0.0;
		    cmd_.brake = 100.0;
		    cmd_mutex_.unlock();

		    return;
	    }
	    else
	    {
	        cmd2_time_ = ros::Time::now().toSec();
		    cmd_mutex_.lock();
		    cmd_.validity = true;
		    cmd_.speed_validity = true;
		    cmd_.brake = 0.0;
		    cmd_mutex_.unlock();
	    }
	}

	cmd2_time_ = ros::Time::now().toSec();
	cmd_mutex_.lock();
	cmd_.validity = true;
	cmd_.speed_validity = true;
	cmd_.speed = t_speed_mps * 3.6;
	cmd_.roadWheelAngle = t_angle_deg;
	cmd_mutex_.unlock();
	
	if((++cnt) % 30 == 0)
	{
		cnt = 0;

		ROS_INFO("[%s] Path tracking state:", __NAME__);
		ROS_INFO("[%s]  --true_v: %.2fm/s\t --curve_v: %.2fm/s\t --park_v: %.2fm/s", __NAME__, vehicle_speed, max_speed_by_curve, max_speed_by_park);
		ROS_INFO("[%s]  --t_speed: %.2fm/s\t --t_angle: %.2fdeg\t --fore_dis: %.2fm", __NAME__, t_speed_mps, t_angle_deg, foresight_distance);
		ROS_INFO("[%s]  --pose_x: %.2f\t --pose_y: %.2f\t --pose_yaw: %.2f", __NAME__, vehicle_pose.x, vehicle_pose.y, vehicle_pose.yaw);
		ROS_INFO("[%s]  --target_idx: %lu\t --final_index: %lu", __NAME__, target_idx, t_path.final_index);

		std::string diagnostic_msg_str;
		diagnostic_msg_str = "Path tracking state:";
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
		ros::Duration(0.00001).sleep(); // 睡眠0.01ms

		diagnostic_msg_str = "";
		diagnostic_msg_str += " --true_v (m/s): " + std::to_string(vehicle_speed);
		diagnostic_msg_str += " --curve_v (m/s): " + std::to_string(max_speed_by_curve);
		diagnostic_msg_str += " --park_v (m/s): " + std::to_string(max_speed_by_park);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
		ros::Duration(0.00001).sleep(); // 睡眠0.01ms

		diagnostic_msg_str = "";
		diagnostic_msg_str += " --t_speed (m/s): " + std::to_string(t_speed_mps);
		diagnostic_msg_str += " --t_angle (deg): " + std::to_string(t_angle_deg);
		diagnostic_msg_str += " --fore_dis (m): " + std::to_string(foresight_distance);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
		ros::Duration(0.00001).sleep(); // 睡眠0.01ms

		diagnostic_msg_str = "";
		diagnostic_msg_str += " --pose_x: " + std::to_string(vehicle_pose.x);
		diagnostic_msg_str += " --pose_y: " + std::to_string(vehicle_pose.y);
		diagnostic_msg_str += " --pose_yaw: " + std::to_string(vehicle_pose.yaw);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
		ros::Duration(0.00001).sleep(); // 睡眠0.01ms

		diagnostic_msg_str = " --target_idx: " + std::to_string(target_idx) + " --final_index: " + std::to_string(t_path.final_index);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
		ros::Duration(0.00001).sleep(); // 睡眠0.01ms
	}
}

float PathTracking::generateRoadwheelAngleByRadius(const float& radius)
{
	assert(radius != 0);
	return atan(vehicle_params_.wheel_base / radius) * 180 / M_PI;
}

float PathTracking::limitRoadwheelAngleBySpeed(const float& angle,
                                               const float& speed)
{
	float result = angle;
	float min_steering_radius = speed * speed / max_side_acceleration_;
	
	if(min_steering_radius < 1.0)
		return result;
	
	float max_angle = fabs(generateRoadwheelAngleByRadius(min_steering_radius));
	if(max_angle > vehicle_params_.max_roadwheel_angle)
		max_angle = vehicle_params_.max_roadwheel_angle;
	
	if(result > max_angle) result = max_angle;
	else if(result < -max_angle) result = -max_angle;

	return result;
}

float PathTracking::generateMaxSpeedByCurvature(const Path& path,
                                                const size_t& begin_idx,
											    const float& curvature_search_distance)
{
	float sum_dis = 0.0;
	float max_cur = 0.0;
	float now_cur;
	for(size_t i = begin_idx; i < path.points.size() - 1; i++)
	{
		now_cur = fabs(path.points[i].curvature);
		if(max_cur < now_cur) max_cur = now_cur;

		sum_dis	+= computeDistance(path.points[i].x, path.points[i].y, path.points[i + 1].x, path.points[i + 1].y);
		if(sum_dis >= curvature_search_distance) break;
	}

	if(max_cur == 0.0) return expect_speed_;
	return sqrt(1.0 / fabs(max_cur) * max_side_acceleration_);
}

float PathTracking::generateMaxSpeedToPrepareToStop(const float& dis2park)
{
	float basic_speed = 1.0; // 单位m/s
	float coefficient = 4.0; // 单位m/(m/s)
	if(dis2park < coefficient * expect_speed_) return basic_speed;
	else return expect_speed_;
}

float PathTracking::generateMaxSpeedByParkingPoint(const Path& path)
{
	// 只考虑最近一个停车点
	// 如果当前正在停车中，速度置0
	if(path.park_points.points[0].isParking)
	{
		ROS_INFO("[%s] Keep parking.", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Keep parking.");
		return 0.0;
	}
	
	// 计算与停车点距离
	float dis = computeDistance(path.points[path.pose_index], path.points[path.park_points.points[0].index]);
	return generateMaxSpeedToPrepareToStop(dis);
}

float PathTracking::generateMaxSpeedByTrafficLightPoint(const Path& path)
{
    // 只考虑最近一个交通灯点
	// 如果当前正在停车中，速度置0
	if(path.traffic_light_points.size() > 0)
	{
		if(path.traffic_light_points.points[0].isParking)
		{
			ROS_INFO("[%s] Keep parking.", __NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Keep parking.");
			return 0.0;
		}
	
		// 计算与停车点距离
		float dis = computeDistance(path.points[path.pose_index], path.points[path.traffic_light_points.points[0].index]);
		return generateMaxSpeedToPrepareToStop(dis);
	}
	else
	{
		return expect_speed_;
	}
		
}


float PathTracking::generateMaxSpeedBySpeedRange(const Path& path)
{
    if(path.speed_ranges.size() == 0)
	{
	    return expect_speed_;
	}
	else
	{
	    // 只考虑最近一个限速区间
	    if(path.speed_ranges.ranges[0].start_index == 0) // 车辆处于限速区间内
	    {
		    float limit_speed_mps = path.speed_ranges.ranges[0].speed / 3.6;
		    return limit_speed_mps < expect_speed_ ? limit_speed_mps : expect_speed_;
	    }
	    else
	    {
	        return expect_speed_;
	    }
	}
    
}
