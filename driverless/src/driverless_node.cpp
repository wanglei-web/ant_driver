#include "driverless/driverless_node.h"
#define __NAME__ "driverless"

/* @function 自动驾驶请求服务器
             step1. actionlib服务器回调函数(as_callback)收到新目标请求
             step2. as_callback唤醒工作线程(workingThread)开始工作，然后as_callback挂起
             step3. workingThread任务完成后继续挂起，唤醒as_callback判断是否有新目标
             step4. 如果有新任务，返回step2, 否则as_callback退出并再次等待step1
	
             as_callback使用work_cv_条件变量唤醒workingThread
             workingThread使用listen_cv_条件变量唤醒as_callback
 */

void AutoDrive::executeDriverlessCallback(const driverless_actions::DoDriverlessTaskGoalConstPtr& goal)
{
	if(!handleNewGoal(goal)) return;
	
	std::unique_lock<std::mutex> lock_listen_cv(listen_cv_mutex_);
	listen_cv_.wait(lock_listen_cv, [&](){return request_listen_;});
	request_listen_ = false;
	listen_cv_mutex_.unlock();
	
	while(ros::ok())
	{
		if(as_->isNewGoalAvailable())
		{
			// if we're active and a new goal is available, we'll accept it, but we won't shut anything down
			ROS_INFO("[%s] The current work was interrupted by new request.", __NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "The current work was interrupted by new request.");

			driverless_actions::DoDriverlessTaskGoalConstPtr new_goal = as_->acceptNewGoal();
			if(!handleNewGoal(new_goal)) return;
			
			std::unique_lock<std::mutex> lock_listen_cv(listen_cv_mutex_);
			listen_cv_.wait(lock_listen_cv, [&](){return request_listen_;});
			request_listen_ = false;
		}
	}
}

/* @brief 处理新目标
          ①有效目标 对新目标进行预处理/载入相关文件/切换系统状态/唤醒工作线程，返回true
          ②无效目标 返回false
 * @param goal 目标信息
 */
bool AutoDrive::handleNewGoal(const driverless_actions::DoDriverlessTaskGoalConstPtr& goal)
{
	int goal_type = goal->type;
	if(goal_type != goal->FILE_TYPE)
	{
		// 只支持 FILE_TYPE 类型请求
		ROS_ERROR("[%s] Request type error!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Request type error!");

		as_->setAborted(driverless_actions::DoDriverlessTaskResult(), "Aborting on unknown goal type!");
		return false;
	}

	int goal_task = goal->task;
	if(goal_task != goal->DRIVE_TASK && goal_task != goal->REVERSE_TASK)
	{
		// 只支持 DRIVE_TASK 和 REVERSE_TASK 任务请求
		ROS_ERROR("[%s] Request task error!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Request task error!");

		as_->setAborted(driverless_actions::DoDriverlessTaskResult(), "Aborting on unknown goal task!");
		return false;
	}

	std::string goal_roadnet_file = goal->roadnet_file;
	float goal_expect_speed = goal->expect_speed;
	float goal_path_resolution = goal->path_resolution;
	bool goal_path_filp = goal->path_filp;

	ROS_INFO("[%s] New task:", __NAME__);
	ROS_INFO("[%s]  --type: %d\t --task: %d --filp: %d", __NAME__, goal_type, goal_task, goal_path_filp);
	ROS_INFO("[%s]  --roadnet_file: %s", __NAME__, goal_roadnet_file.c_str());
	ROS_INFO("[%s]  --expect_speed: %f\t --path_resolution: %f", __NAME__, goal_expect_speed, goal_path_resolution);
	
	std::string diagnostic_msg_str;
	diagnostic_msg_str = "New task:";
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
	ros::Duration(0.00001).sleep(); // 睡眠0.01ms
	diagnostic_msg_str = " --type: " + std::to_string(goal_type) + " --task: " + std::to_string(goal_task);
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
	ros::Duration(0.00001).sleep(); // 睡眠0.01ms
	diagnostic_msg_str = " --roadnet_file: " + goal_roadnet_file;
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
	ros::Duration(0.00001).sleep(); // 睡眠0.01ms
	diagnostic_msg_str = " --expect_speed: " + std::to_string(goal_expect_speed) + " --path_resolution: " + std::to_string(goal_path_resolution);
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, diagnostic_msg_str);
	ros::Duration(0.00001).sleep(); // 睡眠0.01ms

	switchSystemState(State_Stop); // 新请求，无论如何先停止, 暂未解决新任务文件覆盖旧文件导致的自动驾驶异常问题，
                                   // 因此只能停车后开始新任务
                                   // 实则，若新任务与当前任务驾驶方向一致，只需合理的切换路径文件即可！
                                   // 已经预留了切换接口，尚未解决运行中清空历史文件带来的隐患

	ROS_INFO("[%s] Ready for new task, vehicle speed is zero now.", __NAME__);
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Ready for new task, vehicle speed is zero now.");

	this->expect_speed_ = goal_expect_speed;

	if(!loadDriveTaskFile(goal_roadnet_file, goal_path_filp))
	{
		ROS_ERROR("[%s] Failed to load path file!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to load path file!");

		driverless_actions::DoDriverlessTaskResult res;
		res.success = false;
		as_->setSucceeded(res, "Aborting on task, failed to load path file!");
		return false;
	}

	if(goal_task == goal->DRIVE_TASK)
	{
		switchSystemState(State_SwitchToDrive); // 切换系统状态为: 切换到前进
	}
	else
	{
        switchSystemState(State_SwitchToReverse); // 切换系统状态为: 切换到后退
	}
	
	std::unique_lock<std::mutex> lock_work_cv(work_cv_mutex_);
	has_new_task_ = true;
	work_cv_.notify_one(); // 唤醒工作线程
	return true;
}

void AutoDrive::workingThread()
{
	is_running_ = true;
	while(ros::ok() && is_running_)
	{
		// 使用条件变量挂起工作线程，等待其他线程请求唤醒
		// 为防止虚假唤醒(系统等原因)，带有谓词参数的wait函数，唤醒的同时判断谓词是否满足，否则继续挂起
		// 条件变量与独占指针搭配使用，首先使用独占指针加锁，wait函数内部进行解锁并等待唤醒信号，线程唤醒后再次加锁
		// 当前线程被唤醒并开始工作且任务结束前，其他线程无法获得锁，当新任务到达
		std::unique_lock<std::mutex> lock_work_cv(work_cv_mutex_);
		work_cv_.wait(lock_work_cv, [&](){return has_new_task_;});

		has_new_task_ = false;
		doDriveWork();
		
		std::unique_lock<std::mutex> lock_listen_cv(listen_cv_mutex_);
		request_listen_ = true;
		listen_cv_.notify_one(); // 唤醒监听线程

		// Can not call switchSystemState(State_Stop) here !!!
		// Because the expect state has set by doDriveWork/doReverseWork
		// 此处不可切换系统状态，而需要在上述子任务函数(doDriveWork/doReverseWork)中进行切换,
		// 子任务中系统状态切换包含两种情况，
		// 1.任务完成，切换系统为停止状态
		// 2.被新任务打断，切换系统为指定状态
		// 因此！此处不能再切换系统状态，否则状态机制将被打破，导致任务无法正常运行！
	}
}

void AutoDrive::doDriveWork()
{
	int current_state = system_state_;
	int current_task;

	if(current_state == State_Drive)
	{
		current_task = DRIVE_MODE;
		ROS_INFO("[%s] Current system state: %s.", __NAME__, StateName[current_state].c_str());
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Current system state: " + StateName[current_state] + ".");
	}
	else if(current_state == State_Reverse)
	{
		current_task = REVERSE_MODE;
		ROS_INFO("[%s] Current system state: %s.", __NAME__, StateName[current_state].c_str());
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Current system state: " + StateName[current_state] + ".");
	}
	else
	{
		ROS_ERROR("[%s] Current state error!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Current state error!");
		return;
	}
	
	planner_.setTaskMode(current_task);
	if(!planner_.start())
	{
		ROS_ERROR("[%s] Failed to start path planner!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to start path planner!");
		return;
	}
	
	tracker_.setTaskMode(current_task);
	tracker_.setExpectSpeed(expect_speed_);
	if(!tracker_.start())
	{
		ROS_ERROR("[%s] Failed to start path tracker!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to start path tracker!");
		return;
	}
	
	ROS_INFO("[%s] Drive task is running.", __NAME__);
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Drive task is running.");
	
	ros::Rate loop_rate(20);
	task_running_ = true;
	
	while(ros::ok() && system_state_ != State_Stop && planner_.isRunning())
	{
		if(planner_.getEmergencyState()) tracker_.setEmergencyState(true);
		else tracker_.setEmergencyState(false);
		
		if(planner_.getAvoidingState()) tracker_.setExpectSpeed(10); // km/h
		else tracker_.setExpectSpeed(expect_speed_);
		
		tracker_cmd_ = tracker_.getControlCmd();
		this->driveDecisionMaking();

		if(as_->isActive()) // 判断action server是否为活动，防止函数的非服务调用导致的错误
		{
			driverless_actions::DoDriverlessTaskFeedback feedback;
			
			cmd2_mutex_.lock();
			feedback.speed = controlCmd2_.set_speed;
			feedback.steer_angle = controlCmd2_.set_roadWheelAngle;
			cmd2_mutex_.unlock();

			as_->publishFeedback(feedback);

			if(as_->isPreemptRequested()) 
			{
				ROS_INFO("[%s] Drive task is PreemptRequested.", __NAME__);
				publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Drive task is PreemptRequested.");

				as_->setPreempted(); // 自主触发中断请求
				break;
			}
		}
		loop_rate.sleep();
	}
	ROS_INFO("[%s] Drive task completed.", __NAME__);
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK, "Drive task completed.");

	planner_.stop();
	tracker_.stop();
	if(as_->isActive())
	{
		as_->setSucceeded(driverless_actions::DoDriverlessTaskResult(), "Drive task completed.");
	}
	task_running_ = false;
	switchSystemState(State_Stop);
}

void AutoDrive::driveDecisionMaking()
{
	// ControlCmd1指令
	std::lock_guard<std::mutex> lock1(cmd1_mutex_);
	
	// 设置转向灯指令
	if(tracker_cmd_.turnLight == 0)
	{
	    // 避障过程中，发送固定时长的转向灯指令
	    static double turn_time = 0;
	    static float last_offset = 0; // 默认初始状态直行
	    
	    if(ros::Time::now().toSec() - turn_time > 5.0)
	    {
	        controlCmd1_.set_turnLight_L = false;
	        controlCmd1_.set_turnLight_R = false;
	    }
	    
	    float offset_temp = planner_.getOffset();
	    if(offset_temp < last_offset)
	    {
	        controlCmd1_.set_turnLight_L = true;
	        controlCmd1_.set_turnLight_R = false;
	        turn_time = ros::Time::now().toSec();
	        last_offset = offset_temp;
	    }
	    else if(offset_temp > last_offset)
	    {
	        controlCmd1_.set_turnLight_L = false;
	        controlCmd1_.set_turnLight_R = true;
	        turn_time = ros::Time::now().toSec();
	        last_offset = offset_temp;
	    }
	}
	else if(tracker_cmd_.turnLight == 1)
	{
	    controlCmd1_.set_turnLight_L = true;
	}
	else if(tracker_cmd_.turnLight == 2)
	{
	    controlCmd1_.set_turnLight_R = true;
	}
	else if(tracker_cmd_.turnLight == 3)
	{
		controlCmd1_.set_turnLight_L = true;
		controlCmd1_.set_turnLight_R = true;
	}

	// ControlCmd2指令
	std::lock_guard<std::mutex> lock2(cmd2_mutex_);

	controlCmd2_.set_speed = tracker_cmd_.speed;
	controlCmd2_.set_brake = tracker_cmd_.brake;
	controlCmd2_.set_roadWheelAngle = tracker_cmd_.roadWheelAngle;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_drive_node");
	ros::AsyncSpinner spinner(5);
	spinner.start(); // 非阻塞

	ros::NodeHandle nh, nh_private("~");
    AutoDrive auto_drive;
    if(auto_drive.init(nh, nh_private))
    	ros::waitForShutdown();
    return 0;
}  


	
