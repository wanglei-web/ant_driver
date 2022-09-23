#include "driverless/local_path_planning.h"
#define __NAME__ "local_path_planning"

LocalPathPlanning::LocalPathPlanning():
	AutoDriveBase(__NAME__)
{
}

bool LocalPathPlanning::init(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;

	nh_private_.param<std::string>("sub_topic_obstacle_array", sub_topic_obstacle_array_, "/obstacle_array");
	nh_private_.param<std::string>("sub_topic_obu_fusion", sub_topic_obu_fusion_, "/obu_fusion");
	nh_private_.param<std::string>("sub_topic_light", sub_topic_light_, "/light");

	nh_private_.param<std::string>("pub_topic_marker_array", pub_topic_marker_array_, "/obstacles_in_base");
	nh_private_.param<std::string>("pub_topic_global_path", pub_topic_global_path_, "/global_path");
	nh_private_.param<std::string>("pub_topic_local_path", pub_topic_local_path_, "/local_path");

	nh_private_.param<std::string>("marker_array_frame_id", marker_array_frame_id_, "base_link");
	nh_private_.param<std::string>("global_path_frame_id", global_path_frame_id_, "base_link");
	nh_private_.param<std::string>("local_path_frame_id", local_path_frame_id_, "base_link");

	nh_private_.param<float>("max_match_distance", max_match_distance_, 10.0); // m
	nh_private_.param<float>("max_deceleration", max_deceleration_, 1.0); // m/s2
	nh_private_.param<float>("default_local_path_length", default_local_path_length_, 50.0); // m
	nh_private_.param<float>("min_following_distance", min_following_distance_, 7.5); // m
	nh_private_.param<float>("max_search_range", max_search_range_, 30.0); // m
	nh_private_.param<float>("safe_margin", safe_margin_, 0.5); // m
	nh_private_.param<float>("lane_left_width", lane_left_width_, 1.75); // m
	nh_private_.param<float>("lane_right_width", lane_right_width_, 1.75); // m

	nh_private_.param<bool>("use_avoiding", use_avoiding_, false);
	nh_private_.param<float>("max_avoiding_speed", max_avoiding_speed_, 4.0); // m/s
	nh_private_.param<float>("min_offset_increment", min_offset_increment_, 0.5); // m
	nh_private_.param<float>("max_avoiding_distance", max_avoiding_distance_, 20.0); // m
	nh_private_.param<float>("min_avoiding_distance", min_avoiding_distance_, 10.0); // m
	nh_private_.param<int>("repeat_detection_threshold", repeat_detection_threshold_, 3);
	nh_private_.param<int>("delay_threshold", delay_threshold_, 10);

	nh_private_.param<float>("dx_sensor2base", dx_sensor2base_, 0.0);
	nh_private_.param<float>("dy_sensor2base", dy_sensor2base_, 0.0);
	nh_private_.param<float>("phi_sensor2base", phi_sensor2base_, 0.00); // rad, Pandar40: 0.03, RS_Ruby: 0.00 

	nh_private_.param<float>("dx_base2gps", dx_base2gps_, 0.0);
	nh_private_.param<float>("dy_base2gps", dy_base2gps_, 0.0);
	nh_private_.param<float>("phi_base2gps", phi_base2gps_, 0.02); // rad

	pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_marker_array_, 1);
	pub_global_path_ = nh_private_.advertise<nav_msgs::Path>(pub_topic_global_path_, 1);
	pub_local_path_ = nh_private_.advertise<nav_msgs::Path>(pub_topic_local_path_, 1);

	initDiagnosticPublisher(nh_, __NAME__);

	task_initialized_ = false;
	
	// 设置默认值
	emergency_state_ = false;
	is_following_ = false;
	is_avoiding_ = false;
	offset_ = 0.0;
	obstacle_array_time_ = 0.0;
	obstacle_in_global_path_ = false;
	obstacle_in_local_path_ = false;
	
	emergency_state_for_obu_ = false;
	emergency_state_time_for_obu_ = 0.0;

	get_terminal_from_obu_ = false;

	light_passable_ = true;
    light_remain_time_ = 0;
    light_time_ = 0.0;

	is_ready_ = true;

	return true;
}

bool LocalPathPlanning::start()
{
	if(!is_ready_)
	{
		ROS_ERROR("[%s] System is not ready!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "System is not ready!");
		return false;
	}
	if(global_path_.size() == 0)
	{
		ROS_ERROR("[%s] No global path!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "No global path!");
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
	if(global_path_.park_points.size() == 0)
	{
		global_path_.park_points.points.emplace_back(global_path_.final_index, -1); // 将全局路径的终点设置为永久停车点（-1s）
	}
	if(!global_path_.park_points.isSorted())
	{
		global_path_.park_points.sort(); // 确保停车点有序
	}

	is_running_ = true;
	
	if(task_mode_ == DRIVE_MODE)
	{
		cmd_timer_ = nh_.createTimer(ros::Duration(0.03), &LocalPathPlanning::cmd_timer_callback, this);
		sub_obstacle_array_ = nh_.subscribe(sub_topic_obstacle_array_, 1, &LocalPathPlanning::obstacles_callback, this);
		sub_obu_fusion_ = nh_.subscribe(sub_topic_obu_fusion_, 1, &LocalPathPlanning::obu_callback, this);
		sub_light_ = nh_.subscribe(sub_topic_light_, 1, &LocalPathPlanning::light_callback, this);
	}
	else
	{
		cmd_timer_ = nh_.createTimer(ros::Duration(0.03), &LocalPathPlanning::cmd_timer_callback, this);
	}

	return true;
}

void LocalPathPlanning::stop()
{
	cmd_timer_.stop();
	sub_obstacle_array_.shutdown();
	is_running_ = false;
}

bool LocalPathPlanning::isRunning()
{
	return is_running_;
}

void LocalPathPlanning::setTaskMode(const int& task)
{
	if(task != DRIVE_MODE && task != REVERSE_MODE)
	{
		// 只支持 DRIVE_MODE 和 REVERSE_MODE 任务请求
		ROS_ERROR("[%s] Request task error!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Request task error!");
	}
	task_mode_ = task;
	task_initialized_ = true;
}

float LocalPathPlanning::getOffset()
{
    return offset_;
}

bool LocalPathPlanning::getAvoidingState()
{
    if(!use_avoiding_) return false;
	else return obstacle_in_local_path_;
}

bool LocalPathPlanning::getEmergencyState()
{
	if(emergency_state_ || emergency_state_for_obu_) return true;
	else return false;
}

// 定时回调函数
// 根据全局路径和障碍物信息设置局部路径
// 添加转向区间、限速区间、停车点
void LocalPathPlanning::cmd_timer_callback(const ros::TimerEvent&)
{
	if(ros::Time::now().toSec() - obstacle_array_time_ > 0.5) // 0.5秒未更新则应急状态失效
	{
		emergency_state_ = false;
		is_following_ = false;
		is_avoiding_ = false;
		offset_ = 0.0;
		obstacle_array_time_ = 0.0;
	}

	if(ros::Time::now().toSec() - emergency_state_time_for_obu_ > 5.0) // 应急状态（obu）持续5秒后失效
	{
		emergency_state_for_obu_ = false;
	}

	if(ros::Time::now().toSec() - light_time_ > 0.5) // 0.5秒未更新则红绿灯失效
	{
	    light_passable_ = true;
	    light_remain_time_ = 0;
	    light_time_ = 0.0;
	}

	// 障碍物过近触发紧急制动状态
	if(emergency_state_ || emergency_state_for_obu_)
	{
		local_path_.mutex.lock();
		local_path_.clear();
		local_path_.mutex.unlock();
		ROS_INFO("[%s] Emergency State is triggered.", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Emergency State is triggered.");
		return;
	}
	
	static size_t cnt = 0;

	// 读取车辆状态，创建副本避免多次读取
	const Pose vehicle_pose = vehicle_state_.getPose(LOCK);

	dx_gps2global_ = vehicle_pose.x;
	dy_gps2global_ = vehicle_pose.y;
	phi_gps2global_ = vehicle_pose.yaw;

	// 计算局部路径长度
	float local_path_length;
	if(is_following_)
	{
	    // 如果障碍物静止，则希望在距离障碍物min_following_distance_处停车
	    // 如果障碍物与自车均以比较高的速度运动，局部路径长度如何给定？
		local_path_length = nearest_obstacle_distance_in_local_path_ - min_following_distance_;
		if(local_path_length > default_local_path_length_) local_path_length = default_local_path_length_;
		if(local_path_length < 0.0) local_path_length = 0.0;
	}
	else
	{
		local_path_length = default_local_path_length_;
	}
	
	// 寻找当前自车位置对应的路径点，亦为局部路径起点，更新pose_index到全局路径
	size_t nearest_idx = findNearestPointInPath(global_path_, vehicle_pose, max_match_distance_);
	global_path_.pose_index = nearest_idx;

	// 如果当前点与全局路径终点过近，结束任务
	// 留出10个定位点余量，使得自车速度完全减为0时，近似到达期望终点位置
	if(global_path_.final_index - nearest_idx < 5)
	{
		ROS_INFO("[%s] Path tracking completed.", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Path tracking completed.");

		is_running_ = false; // 该变量的状态被其他节点监听
		return;
	}

	// 根据设定长度和起始点索引，寻找局部路径终点在全局路径中的索引
	// 如果设定长度为0，返回起始点索引
	// 如果全局路径剩余距离不足，返回全局路径终点索引
	size_t farthest_idx = findPointInPath(global_path_, local_path_length, nearest_idx);
	
	// 更新路径宽度信息
	lane_left_width_ = global_path_.points[global_path_.pose_index].left_width;
	lane_right_width_ = global_path_.points[global_path_.pose_index].right_width;
	if(-offset_ > lane_left_width_ || offset_ > lane_right_width_)
		offset_ = 0.0;
	
	// 设置局部路径
	local_path_.mutex.lock();
	local_path_.clear();

	// 当局部路径长度为0时，len = 1
	size_t len = farthest_idx - nearest_idx + 1;
	local_path_.points.resize(len);
	float offset_temp = offset_;
	for(size_t i = 0; i < len; i++)
	{
		local_path_.points[i].x = global_path_.points[nearest_idx + i].x;
		local_path_.points[i].y = global_path_.points[nearest_idx + i].y;
		local_path_.points[i].yaw = global_path_.points[nearest_idx + i].yaw;
		local_path_.points[i].curvature = global_path_.points[nearest_idx + i].curvature;
		local_path_.points[i].left_width = global_path_.points[nearest_idx + i].left_width;
		local_path_.points[i].right_width = global_path_.points[nearest_idx + i].right_width;

		offsetPoint(local_path_.points[i], offset_temp);
	}
	local_path_.resolution = global_path_.resolution;
	local_path_.pose_index = 0;
	local_path_.final_index = local_path_.points.size() - 1;
	
	if(task_mode_ == DRIVE_MODE)
	{
		addTurnRanges(nearest_idx, farthest_idx);
		addSpeedRanges(nearest_idx, farthest_idx);
		addParkingPoints(nearest_idx, farthest_idx);
		addTrafficLightPoints(nearest_idx, farthest_idx);
	}
	else
	{
		addParkingPoints(nearest_idx, farthest_idx);
	}
	
	// 路径拓展延伸，保证终点部分的路径跟踪过程正常，同时防止局部路径过短导致出错
	// 经过拓展延伸后，local_path_.final_index小于local_path.size() - 1
	if(local_path_.points.size() >= 5) getExtending(local_path_, 20.0);
	local_path_.mutex.unlock();

	if((++cnt) % 10 == 0)
	{
		cnt = 0;
		
		ROS_INFO("[%s] Path planning state:", __NAME__);
		ROS_INFO("[%s]  --nearest_idx: %d\t --farthest_idx: %d", __NAME__, nearest_idx, farthest_idx);
		ROS_INFO("[%s]  --path_lengh: %.2f\t --offset: %.2f", __NAME__, local_path_length, offset_temp);
		ROS_INFO("[%s]  --following: %d\t --avoiding: %d", __NAME__, is_following_, is_avoiding_);

		std::string diagnostic_msg_str;
		diagnostic_msg_str = "Path planning state:";
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
		ros::Duration(0.00001).sleep(); // 睡眠0.01ms
		diagnostic_msg_str = " --nearest_idx: " + std::to_string(nearest_idx) + " --farthest_idx: " + std::to_string(farthest_idx);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
		ros::Duration(0.00001).sleep(); // 睡眠0.01ms
		diagnostic_msg_str = " --path_lengh: " + std::to_string(local_path_length) + " --offset: " + std::to_string(offset_temp);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
		ros::Duration(0.00001).sleep(); // 睡眠0.01ms
		diagnostic_msg_str = " --following: " + std::to_string(is_following_) + " --avoiding: " + std::to_string(is_avoiding_);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
		ros::Duration(0.00001).sleep(); // 睡眠0.01ms
		
		if(obstacle_in_global_path_)
		{
		    float obs_dis_g = nearest_obstacle_distance_in_global_path_;
			size_t obs_idx_g = nearest_obstacle_index_in_global_path_;
			ROS_INFO("[%s]  --obs_dis_g: %.2f\t --obs_idx_g: %d", __NAME__, obs_dis_g, obs_idx_g);

			diagnostic_msg_str = " --obs_dis_g: " + std::to_string(obs_dis_g) + " --obs_idx_g: " + std::to_string(obs_idx_g);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
			ros::Duration(0.00001).sleep(); // 睡眠0.01ms
		}
		if(obstacle_in_local_path_)
		{
		    float obs_dis_l = nearest_obstacle_distance_in_local_path_;
			size_t obs_idx_l = nearest_obstacle_index_in_local_path_;
			ROS_INFO("[%s]  --obs_dis_l: %.2f\t --obs_idx_l: %d", __NAME__, obs_dis_l, obs_idx_l);

			diagnostic_msg_str = " --obs_dis_l: " + std::to_string(obs_dis_l) + " --obs_idx_l: " + std::to_string(obs_idx_l);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
			ros::Duration(0.00001).sleep(); // 睡眠0.01ms
		}

		// 在base系显示全局路径
		publishPath(pub_global_path_, global_path_, global_path_.pose_index, global_path_.final_index, global_path_frame_id_);
		
		// 在base系显示局部路径
		publishPath(pub_local_path_, local_path_, local_path_.pose_index, local_path_.final_index, local_path_frame_id_);
	}
}

void LocalPathPlanning::addTurnRanges(const size_t& nearest_idx,
							          const size_t& farthest_idx)
{
	for(size_t j = 0; j < global_path_.turn_ranges.size(); j++)
    {
        int turn_type = global_path_.turn_ranges.ranges[j].type;
        size_t turn_start_idx = global_path_.turn_ranges.ranges[j].start_index;
        size_t turn_end_idx = global_path_.turn_ranges.ranges[j].end_index;
        
        if(turn_start_idx < farthest_idx && turn_end_idx > nearest_idx) // 转向区间位于局部路径内
        {
            size_t turn_start_idx_l, turn_end_idx_l;
            
            if(turn_start_idx >= nearest_idx) turn_start_idx_l = turn_start_idx - nearest_idx;
            else turn_start_idx_l = 0;
            
            if(turn_end_idx <= farthest_idx) turn_end_idx_l = turn_end_idx - nearest_idx;
            else turn_end_idx_l = farthest_idx - nearest_idx;
            
            local_path_.turn_ranges.ranges.emplace_back(turn_type, turn_start_idx_l, turn_end_idx_l);
			break;
        }
    }
}

void LocalPathPlanning::addSpeedRanges(const size_t& nearest_idx,
							           const size_t& farthest_idx)
{
	for(size_t j = 0; j < global_path_.speed_ranges.size(); j++)
    {
        float limit_speed = global_path_.speed_ranges.ranges[j].speed;
        size_t limit_start_idx = global_path_.speed_ranges.ranges[j].start_index;
        size_t limit_end_idx = global_path_.speed_ranges.ranges[j].end_index;
        
        if(limit_start_idx < farthest_idx && limit_end_idx > nearest_idx) // 限速区间位于局部路径内
        {
            size_t limit_start_idx_l, limit_end_idx_l;
            
            if(limit_start_idx >= nearest_idx) limit_start_idx_l = limit_start_idx - nearest_idx;
            else limit_start_idx_l = 0;
            
            if(limit_end_idx <= farthest_idx) limit_end_idx_l = limit_end_idx - nearest_idx;
            else limit_end_idx_l = farthest_idx - nearest_idx;
            
            local_path_.speed_ranges.ranges.emplace_back(limit_speed, limit_start_idx_l, limit_end_idx_l);
        }
    }
}

void LocalPathPlanning::addParkingPoints(const size_t& nearest_idx,
							             const size_t& farthest_idx)
{
	// 从全局路径获得下一个停车点
	// 根据自车位置，更新全局路径的停车点
	ParkingPoint& cur_point = global_path_.park_points.next();
	while(cur_point.index + 50 < nearest_idx)
	{
	    global_path_.park_points.next_index++;
	    cur_point = global_path_.park_points.next();
	}
	
	// 如果停车点位于局部路径内，且停车时长不为0，将其存入局部路径中
	if(cur_point.index < farthest_idx && cur_point.index + 50 >= nearest_idx && cur_point.parkingDuration != 0)
	{
	    int idx = cur_point.index - nearest_idx;
	    if(idx < 0) idx = 0;
	    local_path_.park_points.points.emplace_back(idx, cur_point.parkingDuration, cur_point.parkingTime, cur_point.isParking);
	}

    // 计算自车当前点到停车点距离，判定是否到达
	float dis2park = computeDistance(global_path_.points[global_path_.pose_index], global_path_.points[cur_point.index]);
    if(dis2park < 0.5 && !cur_point.isParking)
    {
        if(cur_point.parkingDuration == 0)
        {
            global_path_.park_points.next_index++;
        }
        else
        {
            cur_point.isParking = true;
            cur_point.parkingTime = ros::Time::now().toSec();
		    ROS_INFO("[%s] Start parking, parking point (global): %lu.", __NAME__, cur_point.index);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK,
				"Start parking, parking point (global): " + std::to_string(cur_point.index) + ".");
        }
	}
	
	// 如果到达停车点，且停车时长满足要求，更新全局路径的停车点
	if(cur_point.isParking && ros::Time::now().toSec() - cur_point.parkingTime > cur_point.parkingDuration)
	{
		ROS_INFO("[%s] End parking, parking point (global): %lu.", __NAME__, cur_point.index);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK,
			"End parking, parking point (global): " + std::to_string(cur_point.index) + ".");
		global_path_.park_points.next_index++;
    }
    
	// 不论停车点是否位于局部路径内，将局部路径的终点设置为一个停车点（10s），以便在路径跟踪过程中控制速度
	local_path_.park_points.points.emplace_back(local_path_.final_index, 10.0);
}

void LocalPathPlanning::addTrafficLightPoints(const size_t& nearest_idx,
							    	          const size_t& farthest_idx)
{
	if(!global_path_.traffic_light_points.available()) return;

	// 从全局路径获得下一个交通灯点
	// 根据自车位置，更新全局路径的交通灯点
	TrafficLightPoint& cur_point = global_path_.traffic_light_points.next();
	while(cur_point.index + 50 < nearest_idx)
	{
		global_path_.traffic_light_points.next_index++;
		if(global_path_.traffic_light_points.available())
		{
			cur_point = global_path_.traffic_light_points.next();
		}
		else
		{
			break;
		}
	}

	// 当自车与交通灯点距离小于阈值时，根据通行状态更新停车时长
	if(nearest_idx < cur_point.index + 50 && cur_point.index + 50 - nearest_idx <= 300)
	{
		if(!light_passable_)
		{
			cur_point.parkingDuration = light_remain_time_;
			cur_point.parkingTime = ros::Time::now().toSec();
		}
		else
		{
			cur_point.parkingDuration = 0;
			cur_point.isParking = false;
		}
	}

	// 如果交通灯点位于局部路径内，且停车时长不为0，将其存入局部路径中
	if(cur_point.index < farthest_idx && cur_point.index + 50 >= nearest_idx && cur_point.parkingDuration != 0)
	{
		int idx = cur_point.index - nearest_idx;
		if(idx < 0) idx = 0;
		local_path_.traffic_light_points.points.emplace_back(idx, cur_point.parkingDuration, cur_point.parkingTime, cur_point.isParking);
	}

	// 计算自车当前点到交通灯点距离，判定是否到达
	float dis2light = computeDistance(global_path_.points[global_path_.pose_index], global_path_.points[cur_point.index]);
	if(dis2light < 0.5 && !cur_point.isParking)
	{
		if(cur_point.parkingDuration == 0)
		{
			global_path_.traffic_light_points.next_index++;
		}
		else
		{
			cur_point.isParking = true;
			cur_point.parkingTime = ros::Time::now().toSec();
			ROS_INFO("[%s] Start parking, parking point (global): %lu.", __NAME__, cur_point.index);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK,
				"Start parking, parking point (global): " + std::to_string(cur_point.index) + ".");
		}
	}

	// 如果到达交通灯点，且停车时长满足要求，更新全局路径的交通灯点
	if(cur_point.isParking && ros::Time::now().toSec() - cur_point.parkingTime > cur_point.parkingDuration)
	{
		ROS_INFO("[%s] End parking, parking point (global): %lu.", __NAME__, cur_point.index);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK,
			"End parking, parking point (global): " + std::to_string(cur_point.index) + ".");
		global_path_.traffic_light_points.next_index++;
	}
	else
	{
		cur_point.parkingTime = ros::Time::now().toSec();
	}
}

// 障碍物检测回调函数
void LocalPathPlanning::obstacles_callback(const perception_msgs::ObstacleArray::ConstPtr& obstacles)
{
    obstacle_array_time_ = ros::Time::now().toSec();

	// 读取车辆状态，创建副本避免多次读取
	const Pose vehicle_pose = vehicle_state_.getPose(LOCK);
	const float vehicle_speed = vehicle_state_.getSpeed(LOCK);

	// 选择全局路径中自车所在点和搜索区域终点
	size_t nearest_idx_g = global_path_.pose_index;
	size_t farthest_idx_g = findPointInPath(global_path_, max_search_range_, nearest_idx_g);

	// 如果没有障碍物，以默认状态行驶
	if(obstacles->obstacles.size() == 0)
	{
		emergency_state_ = false;
		is_following_ = false;
		is_avoiding_ = false;
		offset_ = 0.0;
		
		obstacle_in_global_path_ = false;
		obstacle_in_local_path_ = false;
		
		publishMarkerArray(obstacles, false, 0);
		return;
	}

	// 特殊情况下（车前一定距离内有障碍物），触发防撞机制
	size_t emergency_idx;
	if(judgeEmergency(obstacles, 5.0, emergency_idx))
	{
	    emergency_state_ = true;
	    
	    publishMarkerArray(obstacles, true, emergency_idx);
	    return;
	}
	else emergency_state_ = false;
	
	// 获取停车点索引
	// 用于限制障碍物搜索距离，超出停车点一段距离的障碍物不予考虑
	// 保证车辆驶入停车点，不被前方障碍干扰
	size_t dest_idx = global_path_.park_points.next().index;
	size_t dest_plus_idx;
	if(dest_idx == global_path_.final_index)
	{
	    dest_plus_idx = dest_idx;
	}
	else
	{
	    dest_plus_idx = findPointInPath(global_path_, 8.0, dest_idx); // 向前增加一个车长的搜索范围
	}
	
	if(farthest_idx_g >= dest_plus_idx) farthest_idx_g = dest_plus_idx;
	
	// 保证全局路径足够长，避免两个size_t数据相减
	int idx_diff_g = farthest_idx_g - nearest_idx_g;
	if(idx_diff_g < 2)
	{
	    publishMarkerArray(obstacles, false, 0);
	    return;
	}

	// 将全局路径偏移offset_作为局部路径，不直接使用local_path_是因为其长度会发生变化
	Path t_path;
	float offset_temp = offset_;
	size_t len = farthest_idx_g - nearest_idx_g + 1;
	
	t_path.points.resize(len);
	for(size_t i = 0; i < len; i++)
	{
		t_path.points[i].x = global_path_.points[nearest_idx_g + i].x;
		t_path.points[i].y = global_path_.points[nearest_idx_g + i].y;
		t_path.points[i].yaw = global_path_.points[nearest_idx_g + i].yaw;
		
		offsetPoint(t_path.points[i], offset_temp);
	}
	t_path.pose_index = 0;
	t_path.final_index = t_path.points.size() - 1;
	
	// 选择局部路径中自车所在点和搜索区域终点
	size_t nearest_idx_l = t_path.pose_index;
	size_t farthest_idx_l = t_path.final_index;
	
	// 保证局部路径足够长，避免两个size_t数据相减
	int idx_diff_l = farthest_idx_l - nearest_idx_l;
	if(idx_diff_l < 2)
	{
	    publishMarkerArray(obstacles, false, 0);
	    return;
	}

	// 不使用避障功能
	if(!use_avoiding_)
	{
		// 判定全局路径中是否存在障碍物，只考虑车前障碍物
		bool in_path;
		float obs_dis;
		size_t obs_idx;
		findNearestObstacleInPath(obstacles, global_path_, nearest_idx_g, farthest_idx_g, 1.0, in_path, obs_dis, obs_idx);
		
		obstacle_in_global_path_ = in_path;
		nearest_obstacle_distance_in_global_path_ = obs_dis;
		nearest_obstacle_index_in_global_path_ = obs_idx;
		
		if(!obstacle_in_global_path_)
		{
			is_following_ = false;
			offset_ = 0.0;
			obstacle_in_local_path_ = false;
		}
		else
		{
			is_following_ = true;
			offset_ = 0.0;
			obstacle_in_local_path_ = true;
		}
		nearest_obstacle_distance_in_local_path_ = obs_dis;
		nearest_obstacle_index_in_local_path_ = obs_idx;
		
		// 在base系显示所有障碍物
		publishMarkerArray(obstacles, in_path, obs_idx);
		return;
	}

    if(fabs(offset_temp) > 0) is_avoiding_ = true;
    else is_avoiding_ = false;

	// 未处于避障状态
	if(!is_avoiding_)
	{
		static size_t obs_cnt = 0; // 防止误检计数器
		
		// 判定全局路径中是否存在障碍物，只考虑车前障碍物
		bool in_path;
		float obs_dis;
		size_t obs_idx;
		findNearestObstacleInPath(obstacles, global_path_, nearest_idx_g, farthest_idx_g, 1.0, in_path, obs_dis, obs_idx);
		
		bool in_path_counted = count(repeat_detection_threshold_, in_path, obs_cnt);
		
		if(!in_path_counted)
		{
		    obstacle_in_global_path_ = false;
		}
		else
		{
		    obstacle_in_global_path_ = true;
	        nearest_obstacle_distance_in_global_path_ = obs_dis;
	        nearest_obstacle_index_in_global_path_ = obs_idx;
		}
		
		// 在base系显示所有障碍物
        publishMarkerArray(obstacles, in_path_counted, obs_idx);
		
		if(!in_path_counted)
		{
		    is_following_ = false;
			offset_ = 0.0;
			
			obstacle_in_local_path_ = false;
		}
		else
		{
            // 计算路径偏移量
	        float passable_offset;
	        if(!computeOffset(obstacles, global_path_, nearest_idx_g, farthest_idx_g,
	            lane_left_width_, lane_right_width_, -10.0, passable_offset))
	        {
		        // 不可以避障
		        is_following_ = true;
		        offset_ = 0.0;
		        
		        obstacle_in_local_path_ = true;
		        nearest_obstacle_distance_in_local_path_ = obs_dis;
                nearest_obstacle_index_in_local_path_ = obs_idx;
	        }
	        else
	        {
		        // 可以避障，但如果速度过快、距离过远或过近时，不执行避障
                if(vehicle_speed > max_avoiding_speed_ ||
                    nearest_obstacle_distance_in_global_path_ > max_avoiding_distance_ ||
                    nearest_obstacle_distance_in_global_path_ < min_avoiding_distance_)
                {
                    is_following_ = true;
                    offset_ = 0.0;
                    
                    obstacle_in_local_path_ = true;
                    nearest_obstacle_distance_in_local_path_ = obs_dis;
                    nearest_obstacle_index_in_local_path_ = obs_idx;
                }
                else
                {
                    is_following_ = false;
                    offset_ = passable_offset;
                    
                    obstacle_in_local_path_ = false;
                }
	        }
		}
	}

	// 处于避障状态
	else
	{
		static size_t obs_cnt_g = 0; // 防止误检计数器
		static size_t obs_cnt_l = 0; // 防止误检计数器
		static size_t go_back_cnt = 0; // 延时计数器
		bool go_back_flag;
		
		// 判定全局路径中是否存在障碍物，考虑车后一定距离内的障碍物
		bool in_path_g;
		float obs_dis_g;
		size_t obs_idx_g;
		findNearestObstacleInPath(obstacles, global_path_, nearest_idx_g, farthest_idx_g, -10.0, in_path_g, obs_dis_g, obs_idx_g);
		
		bool in_path_g_counted = count(repeat_detection_threshold_, in_path_g, obs_cnt_g);
		
		if(!in_path_g_counted)
		{
		    obstacle_in_global_path_ = false;
		}
		else
		{
		    obstacle_in_global_path_ = true;
	        nearest_obstacle_distance_in_global_path_ = obs_dis_g;
	        nearest_obstacle_index_in_global_path_ = obs_idx_g;
		}
		
		// 在base系显示所有障碍物
        publishMarkerArray(obstacles, in_path_g_counted, obs_idx_g);
		
		// 判定局部路径中是否存在障碍物
		bool in_path_l;
		float obs_dis_l;
		size_t obs_idx_l;
		findNearestObstacleInPath(obstacles, t_path, nearest_idx_l, farthest_idx_l, 1.0, in_path_l, obs_dis_l, obs_idx_l);
		
		bool in_path_l_counted = count(repeat_detection_threshold_, in_path_l, obs_cnt_l);
		
		if(!in_path_l_counted && !in_path_g_counted)
		{
		    go_back_flag = count(delay_threshold_, true, go_back_cnt);
		}
		else
		{
		    go_back_flag = count(delay_threshold_, false, go_back_cnt);
		}

		if(!in_path_l_counted)
		{
		    is_following_ = false;
		    if(!in_path_g_counted && go_back_flag) {offset_ = 0.0;} // 返回原车道
		    
		    obstacle_in_local_path_ = false;
		}
		else
		{
		    // 计算路径偏移量，二次避障
			float passable_offset;
			if(!computeOffset(obstacles, global_path_, nearest_idx_g, farthest_idx_g, lane_left_width_, lane_right_width_, -10.0, passable_offset))
			{
				// 不可以避障
				is_following_ = true;
				
				obstacle_in_local_path_ = true;
				nearest_obstacle_distance_in_local_path_ = obs_dis_l;
	            nearest_obstacle_index_in_local_path_ = obs_idx_l;
			}
			else
			{
	            // 可以避障，但如果速度过快、距离过远或过近时，不执行避障
	            if(vehicle_speed > max_avoiding_speed_ ||
	                nearest_obstacle_distance_in_local_path_ > max_avoiding_distance_ ||
	                nearest_obstacle_distance_in_local_path_ < min_avoiding_distance_)
	            {
		            is_following_ = true;
		            
		            obstacle_in_local_path_ = true;
		            nearest_obstacle_distance_in_local_path_ = obs_dis_l;
	                nearest_obstacle_index_in_local_path_ = obs_idx_l;
	            }
	            else
	            {
	                is_following_ = false;
	                offset_ = passable_offset;
	                
	                obstacle_in_local_path_ = false;
	            }
			}
		}
	}
}

void LocalPathPlanning::obu_callback(const obu_msgs::OBU_fusion::ConstPtr& container)
{
    // 遍历所有停车点，进行匹配
    size_t num = global_path_.park_points.size();

	// 从下一停车点开始搜索
	size_t next_index = global_path_.park_points.next_index;
    
    // 处理红绿灯停车点
    int light_state = container->light_state;
    int light_remain_time = container->light_remain_time;
    
    double light_x = container->light_x;
    double light_y = container->light_y;

    double dis_threshold_light = 100.0; // m
    double dis_min_light = DBL_MAX;
    size_t best_idx_light = 0;
    bool association_flag_light = false;
    
    if(light_x != -1 && light_y != -1)
    {
        for(size_t i = next_index; i < num; i++)
        {
            size_t idx = global_path_.park_points.points[i].index;

            double x = global_path_.points[idx].x;
            double y = global_path_.points[idx].y;
            
            double dis = computeDistance(light_x, light_y, x, y);
            if(dis < dis_min_light && dis < dis_threshold_light)
            {
                dis_min_light = dis;
                best_idx_light = i;
                association_flag_light = true;
            }
        }
        
        if(association_flag_light)
        {
            if(light_state == 1 || light_state == 3) // 绿灯或黄灯
			{
				global_path_.park_points.points[best_idx_light].parkingDuration = 0;
			}
                
            else if(light_state == 2) // 红灯
			{
				global_path_.park_points.points[best_idx_light].parkingDuration = light_remain_time;
				global_path_.park_points.points[best_idx_light].parkingTime = ros::Time::now().toSec();
			}
        }
    }
    
    // 处理接客停车点
    double park_x = container->park_x;
    double park_y = container->park_y;
    
    double dis_threshold_park = 30.0; // m
    double dis_min_park = DBL_MAX;
    size_t best_idx_park = 0;
    bool association_flag_park = false;
    
    if(park_x != -1 && park_y != -1)
    {
        for(size_t i = next_index; i < num; i++)
        {
            size_t idx = global_path_.park_points.points[i].index;
            double x = global_path_.points[idx].x;
            double y = global_path_.points[idx].y;
            
            double dis = computeDistance(park_x, park_y, x, y);
            if(dis < dis_min_park && dis < dis_threshold_park)
            {
                dis_min_park = dis;
                best_idx_park = i;
                association_flag_park = true;
            }
        }
        
        if(association_flag_park)
        {
            global_path_.park_points.points[best_idx_park].parkingDuration = 10.0;
        }
    }
    
    // 处理紧急车辆
    int emergency_car_is_near = container->emergency_car_is_near;
    if(emergency_car_is_near == 1 && emergency_state_time_for_obu_ == 0.0)
    {
        emergency_state_for_obu_ = true;
        emergency_state_time_for_obu_ = ros::Time::now().toSec();
    }

	// 处理终点
	double terminal_x = container->terminal_x;
    double terminal_y = container->terminal_y;

	double dis_threshold_terminal = 5.0; // m
	size_t best_idx_terminal = global_path_.final_index;
	
	if(terminal_x != -1 && terminal_y != -1 && !get_terminal_from_obu_)
	{
		size_t start_idx;
		if(global_path_.final_index >= 1000)
			start_idx = global_path_.final_index - 1000; // 搜索路径最后1000个点
		else
			start_idx = 0;
		
		best_idx_terminal = findNearestPointInPath(global_path_, terminal_x, terminal_y, start_idx, global_path_.final_index);
		
		double x = global_path_.points[best_idx_terminal].x;
		double y = global_path_.points[best_idx_terminal].y;
		
		double dis = computeDistance(x, y, terminal_x, terminal_y);
		if(dis < dis_threshold_terminal)
		{
			global_path_.park_points.points.emplace_back(best_idx_terminal, 1000.0); // 停车时长1000s
			global_path_.park_points.sort(); // 确保停车点有序
		}

		get_terminal_from_obu_ = true;
	}
	
}

void LocalPathPlanning::light_callback(const light_msgs::Light::ConstPtr& container)
{
	// 处理交通灯信息
    int state = container->light_state; // 0绿灯 1红灯 2黄灯 -1无
    int remain_time = container->light_remain_time;
    
    if(state == 1 || state == 2)
    {
        light_passable_ = false;
        light_remain_time_ = 10; // 缺省值
		light_time_ = ros::Time::now().toSec();
    }
    else if(state == 0)
    {
        light_passable_ = true;
        light_remain_time_ = 0; // 缺省值
		light_time_ = ros::Time::now().toSec();
    }
}

void LocalPathPlanning::findNearestObstacleInPath(const perception_msgs::ObstacleArray::ConstPtr& obstacles,
										          const Path& path,
							 			          const size_t& nearest_idx,
							 			          const size_t& farthest_idx,
										          const float& min_x_in_sensor,
										          bool& obs_in_path,
										          float& nearest_obs_dis,
										          size_t& nearest_obs_idx)
{
	assert(farthest_idx >= nearest_idx);
	
	obs_in_path = false;
	nearest_obs_dis = FLT_MAX;
	nearest_obs_idx = 0;
	float half_width = safe_margin_ + vehicle_params_.width / 2;
	
	for(size_t i = 0; i < obstacles->obstacles.size(); i++)
	{
		const perception_msgs::Obstacle& obs = obstacles->obstacles[i];

		// 忽略后方的障碍物
		if(obs.pose.position.x < min_x_in_sensor) continue;
		
		// 忽略过远的障碍物
		float dis2ego = computeObstacleDistance2Ego(obs);
		if(dis2ego > max_search_range_) continue;

		// 计算障碍物中心点
		double obs_x;
		double obs_y;
		getGlobalObstacle(obs, obs_x, obs_y);

		// 忽略终点以后的障碍物
		size_t idx = findNearestPointInPath(path, obs_x, obs_y, nearest_idx, nearest_idx, farthest_idx);
		if(idx >= farthest_idx) continue;

		int which_side = judgeWhichSide(obs, path, nearest_idx, farthest_idx);
		float gap2path = computeGapBetweenObstacleAndPath(obs, path, nearest_idx, farthest_idx);
		
		// 如果路径穿过障碍物
		if(which_side == 0)
		{
			obs_in_path = true;
			if(dis2ego < nearest_obs_dis)
			{
				nearest_obs_dis = dis2ego;
				nearest_obs_idx = i;
			}
		}
		// 如果障碍物距离路径足够近
		else if(gap2path <= half_width)
		{
			obs_in_path = true;
			if(dis2ego < nearest_obs_dis)
			{
				nearest_obs_dis = dis2ego;
				nearest_obs_idx = i;
			}
		}
	}
}

bool LocalPathPlanning::computeOffset(const perception_msgs::ObstacleArray::ConstPtr& obstacles,
							          const Path& path,
							          const size_t& nearest_idx,
							          const size_t& farthest_idx,
							          const float& left_width,
							          const float& right_width,
							          const float& min_x_in_sensor,
							          float& passable_offset)
{
	assert(farthest_idx >= nearest_idx);
	
	// 将路径向左右两侧平移，使车辆避开道路内所有障碍物，计算偏移量
	float half_width = safe_margin_ + vehicle_params_.width / 2;

	float left_min = 0;
	float left_max = left_width - half_width;
	float right_min = 0;
	float right_max = right_width - half_width;
	
	for(size_t i = 0; i < obstacles->obstacles.size(); i++)
	{
		const perception_msgs::Obstacle& obs = obstacles->obstacles[i];

		// 忽略后方的障碍物
		if(obs.pose.position.x < min_x_in_sensor) continue;
		// 忽略过远的障碍物
		float dis2ego = computeObstacleDistance2Ego(obs);
		if(dis2ego > max_search_range_) continue;

		// 计算障碍物中心点
		double obs_x;
		double obs_y;
		getGlobalObstacle(obs, obs_x, obs_y);

		// 忽略终点以后的障碍物
		size_t idx = findNearestPointInPath(path, obs_x, obs_y, nearest_idx, nearest_idx, farthest_idx);
		if(idx >= farthest_idx) continue;

		int which_side = judgeWhichSide(obs, path, nearest_idx, farthest_idx);
		float gap2path = computeGapBetweenObstacleAndPath(obs, path, nearest_idx, farthest_idx);
		
		// 如果路径穿过障碍物
		if(which_side == 0)
		{
			// 计算障碍物各顶点
			double obs_xs[4];
			double obs_ys[4];
			getGlobalObstacle(obs, obs_xs, obs_ys);
			for(int i = 0; i < 4; i++)
			{
				int which_side_point = judgeWhichSide(obs_xs[i], obs_ys[i], path, nearest_idx, farthest_idx);
				double dis2path = findMinDistance2Path(path, obs_xs[i], obs_ys[i], nearest_idx, nearest_idx, farthest_idx);
				
				// 点在路径上
				if(which_side_point == 0) continue;
				// 点在路径左侧
				else if(which_side_point == 1)
				{
				    float dis = dis2path + half_width;
				    left_min = dis > left_min ? dis : left_min;
				}
				// 点在路径右侧
				else if(which_side_point == -1)
				{
				    float dis = dis2path + half_width;
				    right_min = dis > right_min ? dis : right_min;
				}
			}
		}
		// 障碍物在路径左侧
		else if(which_side == 1)
		{
		    float max_dis2path = computeMaxDistanceBetweenObstacleAndPath(obs, path, nearest_idx, farthest_idx);
		    if(max_dis2path < half_width + 0.5)
		    {
		        // 障碍物足够靠近路径，修改left_min（从障碍物左侧避让）
		        float max_dis = max_dis2path + half_width;
		        left_min = max_dis > left_min ? max_dis : left_min;
		        
		        // 如果影响到right_min，则修改
		        float dis = gap2path - half_width;
		        if(dis < 0.0) right_min = -dis > right_min ? -dis : right_min;
		    }
		    else
		    {
		        // 否则，修改left_max（从障碍物右侧避让）
		        float dis = gap2path - half_width;
		        left_max = dis < left_max ? dis : left_max;
		        
		        // 如果影响到right_min，则修改
		        if(dis < 0.0) right_min = -dis > right_min ? -dis : right_min;
		    }
		    
		}
		// 障碍物在路径右侧
		else if(which_side == -1)
		{
		    float max_dis2path = computeMaxDistanceBetweenObstacleAndPath(obs, path, nearest_idx, farthest_idx);
		    if(max_dis2path < half_width + 0.5)
		    {
		        // 障碍物足够靠近路径，修改right_min（从障碍物右侧避让）
		        float max_dis = max_dis2path + half_width;
		        right_min = max_dis > right_min ? max_dis : right_min;
		        
		        // 如果影响到left_min，则修改
		        float dis = gap2path - half_width;
		        if(dis < 0.0) left_min = -dis > left_min ? -dis : left_min;
		    }
		    else
		    {
		        // 否则，修改right_max（从障碍物左侧避让）
		        float dis = gap2path - half_width;
		        right_max = dis < right_max ? dis : right_max;
		        
		        // 如果影响到left_min，则修改
		        if(dis < 0.0) left_min = -dis > left_min ? -dis : left_min;
		    }
		}
	}

	bool left_passable = false;
	bool right_passable = false;
	float left_offset;
	float right_offset;
	
	// 尝试左转，以固定间隔置offset，防止抖动
	float left_d = ((floor)(left_min / min_offset_increment_) + 1) * min_offset_increment_;
	if(left_d <= left_max)
	{
	    left_offset = left_d;
	    assert(left_offset >= 0.0);
	    left_passable = true;
	}
	
	// 尝试右转，以固定间隔置offset，防止抖动
	float right_d = ((floor)(right_min / min_offset_increment_) + 1) * min_offset_increment_;
	if(right_d <= right_max)
	{
	    right_offset = right_d;
	    assert(right_offset >= 0.0);
	    right_passable = true;
	}

	ROS_INFO("[%s] Compute offset:", __NAME__);
	ROS_INFO("[%s]  --left_min: %.2f\t --right_min: %.2f", __NAME__, left_min, right_min);
	ROS_INFO("[%s]  --left_max: %.2f\t --right_max: %.2f", __NAME__, left_max, right_max);
    ROS_INFO("[%s]  --left_width: %.2f\t --right_width: %.2f", __NAME__, left_width, right_width);

	std::string diagnostic_msg_str;
	diagnostic_msg_str = "Compute offset:";
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
	ros::Duration(0.00001).sleep(); // 睡眠0.01ms
	diagnostic_msg_str = " --left_min: " + std::to_string(left_min) + " --right_min: " + std::to_string(right_min);
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
	ros::Duration(0.00001).sleep(); // 睡眠0.01ms
	diagnostic_msg_str = " --left_max: " + std::to_string(left_max) + " --right_max: " + std::to_string(right_max);
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
	ros::Duration(0.00001).sleep(); // 睡眠0.01ms
	diagnostic_msg_str = " --left_width: " + std::to_string(left_width) + " --right_width: " + std::to_string(right_width);
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
	ros::Duration(0.00001).sleep(); // 睡眠0.01ms

	// 车辆前进时，往左为负（offset应小于0）往右为正（offset应大于0）
	if(!left_passable && !right_passable)
	{
		passable_offset = 0.0;
		return false;
	}
	else if(left_passable && !right_passable)
	{
		passable_offset = -left_offset;
		return true;
	}
	else if(!left_passable && right_passable)
	{
		passable_offset = right_offset;
		return true;
	}
	else
	{
		passable_offset = left_min < right_min ? -left_offset : right_offset;
		return true;
	}
}

bool LocalPathPlanning::judgeEmergency(const perception_msgs::ObstacleArray::ConstPtr& obstacles,
							           const float& safe_distance,
							           size_t& nearest_obs_idx)
{
	bool is_danger = false;
	float half_width = safe_margin_ + vehicle_params_.width / 2;

	for(size_t i = 0; i < obstacles->obstacles.size(); i++)
	{
		const perception_msgs::Obstacle& obs = obstacles->obstacles[i];

		// 忽略后方的障碍物
		if(obs.pose.position.x < 0) continue;

		// 忽略过远的障碍物
		if(obs.pose.position.x > safe_distance) continue;

		// 忽略两侧的障碍物
		if(fabs(obs.pose.position.y) > half_width) continue;

		is_danger = true;
		nearest_obs_idx = i;
	}

	return is_danger;
}

int LocalPathPlanning::judgeWhichSide(const double& x,
							          const double& y,
							          const Path& path,
							          const size_t& nearest_idx,
							          const size_t& farthest_idx)
{
	assert(farthest_idx - nearest_idx >= 2);

	size_t idx = findNearestPointInPath(path, x, y, nearest_idx, nearest_idx, farthest_idx);

	// 用两点(p1x, p1y)和(p2x, p2y)表示路径切线
	double p1x, p2x, p1y, p2y;
	if(idx == nearest_idx)
	{
		p1x = path.points[idx].x;
		p1y = path.points[idx].y;
		p2x = path.points[idx + 1].x;
		p2y = path.points[idx + 1].y;
	}
	else if(idx == farthest_idx)
	{
		p1x = path.points[idx - 1].x;
		p1y = path.points[idx - 1].y;
		p2x = path.points[idx].x;
		p2y = path.points[idx].y;
	}
	else
	{
		size_t idx_n, idx_f;
		
		if(idx > nearest_idx + 10) idx_n = idx - 10;
		else idx_n = nearest_idx;
		if(idx + 10 < farthest_idx) idx_f = idx + 10;
		else idx_f = farthest_idx;
		
		p1x = path.points[idx_n].x;
		p1y = path.points[idx_n].y;
		p2x = path.points[idx_f].x;
		p2y = path.points[idx_f].y;
	}

	// 若已知向量AB和一点P
	// 当向量AB × 向量AP = 0时，P与AB共线
	// 当向量AB × 向量AP > 0时，P在AB左侧，ABP为逆时针排列
	// 当向量AB × 向量AP < 0时，P在AB右侧，ABP为顺时针排列
	double cross_product = (p2x - p1x) * (y - p1y) - (x - p1x) * (p2y - p1y);
    
	if(cross_product == 0.0) return 0;
	else if(cross_product > 0.0) return 1; // 左侧
	else if(cross_product < 0.0) return -1; // 右侧
}

int LocalPathPlanning::judgeWhichSide(const perception_msgs::Obstacle& obs,
							          const Path& path,
							          const size_t& nearest_idx,
							          const size_t& farthest_idx)
{
	// 计算障碍物各顶点
	double obs_xs[4];
	double obs_ys[4];
	getGlobalObstacle(obs, obs_xs, obs_ys);
	
	int s1, s2, s3, s4;
	s1 = judgeWhichSide(obs_xs[0], obs_ys[0], path, nearest_idx, farthest_idx);
	s2 = judgeWhichSide(obs_xs[1], obs_ys[1], path, nearest_idx, farthest_idx);
	s3 = judgeWhichSide(obs_xs[2], obs_ys[2], path, nearest_idx, farthest_idx);
	s4 = judgeWhichSide(obs_xs[3], obs_ys[3], path, nearest_idx, farthest_idx);

	if(s1 == s2 && s2 == s3 && s3 == s4)
	    return s1;
    else
        return 0;
}

void LocalPathPlanning::getGlobalObstacle(const perception_msgs::Obstacle& obs,
								          double& x,
								          double& y)
{
	computeObstacleCenter(obs, x, y);
	transformSensor2Base(x, y);
	transformBase2Gps(x, y);
	transformGps2Global(x, y);
}

void LocalPathPlanning::getGlobalObstacle(const perception_msgs::Obstacle& obs,
								          double xs[4],
								          double ys[4]) // 数组作形参将自动转换为指针
{
	computeObstacleVertex(obs, xs, ys);
	transformSensor2Base(xs, ys);
	transformBase2Gps(xs, ys);
	transformGps2Global(xs, ys);
}

void LocalPathPlanning::computeObstacleCenter(const perception_msgs::Obstacle& obs,
									          double& obs_x,
									          double& obs_y)
{
	obs_x = obs.pose.position.x;
	obs_y = obs.pose.position.y;
}

void LocalPathPlanning::computeObstacleVertex(const perception_msgs::Obstacle& obs,
                                              double obs_xs[4],
									          double obs_ys[4]) // 数组作形参将自动转换为指针
{
	double obs_phi;
	computeObstacleOrientation(obs, obs_phi);
	
	obs_xs[0] = obs.scale.x / 2;
	obs_ys[0] = obs.scale.y / 2;

	obs_xs[1] = -obs.scale.x / 2;
	obs_ys[1] = obs.scale.y / 2;

	obs_xs[2] = -obs.scale.x / 2;
	obs_ys[2] = -obs.scale.y / 2;

	obs_xs[3] = obs.scale.x / 2;
	obs_ys[3] = -obs.scale.y / 2;

	double x0 = obs.pose.position.x;
	double y0 = obs.pose.position.y;

	transform2DPoints(obs_xs, obs_ys, obs_phi, x0, y0);
}

void LocalPathPlanning::computeObstacleOrientation(const perception_msgs::Obstacle& obs,
                                                   double& phi)
{
	// atan(x)求的是x的反正切，其返回值为[-pi/2, pi/2]之间的一个数
    // atan2(y, x)求的是y/x的反正切，其返回值为[-pi, pi]之间的一个数
	
	// phi属于[0, pi)
	phi = 2 * atan(obs.pose.orientation.z / obs.pose.orientation.w);
}

void LocalPathPlanning::transformSensor2Base(double& phi)
{
	phi += phi_sensor2base_;
	if(phi < 0) phi += M_PI;
	if(phi >= M_PI) phi -= M_PI;
}

void LocalPathPlanning::transformSensor2Base(double& x,
                                             double& y)
{
	transform2DPoint(x, y, phi_sensor2base_, dx_sensor2base_, dy_sensor2base_);
}

void LocalPathPlanning::transformSensor2Base(double xs[4],
                                             double ys[4])
{
	transform2DPoints(xs, ys, phi_sensor2base_, dx_sensor2base_, dy_sensor2base_);
}

void LocalPathPlanning::transformBase2Gps(double& x,
                                          double& y)
{
	transform2DPoint(x, y, phi_base2gps_, dx_base2gps_, dy_base2gps_);
}

void LocalPathPlanning::transformBase2Gps(double xs[4],
                                          double ys[4])
{
	transform2DPoints(xs, ys, phi_base2gps_, dx_base2gps_, dy_base2gps_);
}

void LocalPathPlanning::transformGps2Global(double& x,
                                            double& y)
{
	transform2DPoint(x, y, phi_gps2global_, dx_gps2global_, dy_gps2global_);
}

void LocalPathPlanning::transformGps2Global(double xs[4],
                                            double ys[4])
{
	transform2DPoints(xs, ys, phi_gps2global_, dx_gps2global_, dy_gps2global_);
}

double LocalPathPlanning::computeObstacleDistance2Ego(const perception_msgs::Obstacle& obs)
{
	// 计算障碍物各顶点
	double obs_xs[4];
	double obs_ys[4];
	computeObstacleVertex(obs, obs_xs, obs_ys);
	
	double dis_min = DBL_MAX;
	for(int i = 0; i < 4; i++)
	{
	    double dis = sqrt(pow(obs_xs[i], 2) + pow(obs_ys[i], 2));
	    if(dis < dis_min) dis_min = dis;
	}
	
	return dis_min;
}

double LocalPathPlanning::computeGapBetweenObstacleAndPath(const perception_msgs::Obstacle& obs,
												           const Path& path,
												           const size_t& nearest_idx,
												           const size_t& farthest_idx)
{
	// 计算障碍物各顶点
	double obs_xs[4];
	double obs_ys[4];
	getGlobalObstacle(obs, obs_xs, obs_ys);
	
	double gap = DBL_MAX;
	for(int i = 0; i < 4; i++)
	{
		double dis2path = findMinDistance2Path(path, obs_xs[i], obs_ys[i], nearest_idx, nearest_idx, farthest_idx);
		if(dis2path < gap) gap = dis2path;
	}

	return gap;
}

double LocalPathPlanning::computeMaxDistanceBetweenObstacleAndPath(const perception_msgs::Obstacle& obs,
												                   const Path& path,
												                   const size_t& nearest_idx,
												                   const size_t& farthest_idx)
{
    // 计算障碍物各顶点
	double obs_xs[4];
	double obs_ys[4];
	getGlobalObstacle(obs, obs_xs, obs_ys);
	
	double dis = 0;
	for(int i = 0; i < 4; i++)
	{
		double dis2path = findMinDistance2Path(path, obs_xs[i], obs_ys[i], nearest_idx, nearest_idx, farthest_idx);
		if(dis2path > dis) dis = dis2path;
	}

	return dis;
}

void LocalPathPlanning::publishMarkerArray(const perception_msgs::ObstacleArray::ConstPtr obstacles,
                                           const bool& obs_in_path,
								           const size_t& nearest_obs_idx)
{
	visualization_msgs::MarkerArray ma;
	for(size_t i = 0; i < obstacles->obstacles.size(); i++)
	{
		visualization_msgs::Marker m;
		m.header.frame_id = marker_array_frame_id_;
		m.header.stamp = ros::Time::now();

		m.ns = "obstacles";
		m.id = i;
		m.type = visualization_msgs::Marker::CUBE;
		m.action = visualization_msgs::Marker::ADD;

		double obs_x;
		double obs_y;
		computeObstacleCenter(obstacles->obstacles[i], obs_x, obs_y);
		transformSensor2Base(obs_x, obs_y);
		m.pose.position.x = obs_x;
		m.pose.position.y = obs_y;
		m.pose.position.z = obstacles->obstacles[i].pose.position.z;

		double obs_phi;
		computeObstacleOrientation(obstacles->obstacles[i], obs_phi);
		transformSensor2Base(obs_phi);
		m.pose.orientation.x = 0;
		m.pose.orientation.y = 0;
		m.pose.orientation.z = sin(0.5 * obs_phi);
		m.pose.orientation.w = cos(0.5 * obs_phi);

		m.scale.x = obstacles->obstacles[i].scale.x;
		m.scale.y = obstacles->obstacles[i].scale.y;
		m.scale.z = obstacles->obstacles[i].scale.z;

		if(obs_in_path && i == nearest_obs_idx)
		{
			// 淡红色
			m.color.r = 216 / 255.0;
			m.color.g = 0 / 255.0;
			m.color.b = 115 / 255.0;
			m.color.a = 0.85;
		}
		else
		{
			// 淡蓝色
			m.color.r = 91 / 255.0;
			m.color.g = 155 / 255.0;
			m.color.b = 213 / 255.0;
			m.color.a = 0.85;
		}

		m.lifetime = ros::Duration(0.1);
		ma.markers.push_back(m);
	}
    pub_marker_array_.publish(ma);
}

void LocalPathPlanning::publishPath(ros::Publisher& pub,
                                    const Path& path_to_pub,
						            const size_t& begin_idx,
						            const size_t& end_idx,
						            const std::string& frame_id)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id;

	if(begin_idx >= end_idx) return;
	
	// reserve为容器一次性分配内存大小，容量设置为end_idx - begin_idx + 1
	path.poses.reserve(end_idx - begin_idx + 1);
	
	for(size_t i = begin_idx; i < end_idx + 1; i++)
	{
		double x = path_to_pub.points[i].x;
		double y = path_to_pub.points[i].y;
		transformGlobal2Gps(x, y);
		transformGps2Base(x, y);
		
		geometry_msgs::PoseStamped p;
        p.pose.position.x = x;
        p.pose.position.y = y;

        p.header.frame_id = frame_id;
        path.poses.push_back(p);
	}
	pub.publish(path);
}

void LocalPathPlanning::transformGlobal2Gps(double& x,
                                            double& y)
{
	transform2DPoint(x, y, 0, -dx_gps2global_, -dy_gps2global_);
	transform2DPoint(x, y, -phi_gps2global_, 0, 0);
}

void LocalPathPlanning::transformGps2Base(double& x,
                                          double& y)
{
	transform2DPoint(x, y, 0, -dx_base2gps_, -dy_base2gps_);
	transform2DPoint(x, y, -phi_base2gps_, 0, 0);
}

bool LocalPathPlanning::count(const size_t& threshold,
                              const bool& flag,
                              size_t& cnt)
{
    if(!flag)
    {
        cnt = 0;
        return false;
    }
    else
    {
        if(cnt < threshold)
        {
            cnt++;
            return false;
        }
        else
        {
            cnt = threshold;
            return true;
        }
    }
}
