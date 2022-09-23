#include <path_planning/global_path_planning.h>
#define __NAME__ "global_path_planning"

GlobalPathPlanning::GlobalPathPlanning()
{
}

bool GlobalPathPlanning::init(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;
	
	nh_private_.param<int>("task", task_, 0); // 0 - 前进任务，1 - 后退任务
	nh_private_.param<int>("type", type_, 3); // 1 - POSE_TYPE，2 - PATH_TYPE， 3 - FILE_TYPE
	
	nh_private_.param<float>("path_resolution", path_resolution_, 0.1);
	nh_private_.param<float>("expect_speed", expect_speed_, 15); // km/h
	nh_private_.param<std::string>("roadnet_file", roadnet_file_, " "); // 绝对路径，以.txt结尾
	nh_private_.param<bool>("path_filp", path_filp_, false);
	
	ac_ = new DoDriverlessTaskClient("/do_driverless_task", true); // true -> don't need ros::spin()
	
	ROS_INFO("[%s] Wait for server.", __NAME__);
	ac_->waitForServer();
	ROS_INFO("[%s] Wait for server ok.", __NAME__);

	is_ready_ = true;
	
	return true;
}

bool GlobalPathPlanning::start()
{
    if(!is_ready_)
	{
		ROS_ERROR("[%s] System is not ready!", __NAME__);
		return false;
	}
	
	driverless_actions::DoDriverlessTaskGoal goal;
	
	goal.task = task_;
	goal.type = type_;
	
	goal.path_resolution = path_resolution_;
	goal.expect_speed = expect_speed_;
	goal.roadnet_file = roadnet_file_;
	goal.path_filp = path_filp_;
	
    ac_->sendGoal(goal, boost::bind(&GlobalPathPlanning::taskDoneCallback,this,_1,_2),
                        boost::bind(&GlobalPathPlanning::taskActivedCallback,this),
                        boost::bind(&GlobalPathPlanning::taskFeedbackCallback,this,_1));
    
}

void GlobalPathPlanning::taskDoneCallback(const actionlib::SimpleClientGoalState& state,
                                          const driverless_actions::DoDriverlessTaskResultConstPtr& result)
{
    ROS_INFO("[%s] Task is done.", __NAME__);
}

void GlobalPathPlanning::taskFeedbackCallback(const driverless_actions::DoDriverlessTaskFeedbackConstPtr& feedback)
{
    ROS_INFO("[%s] Feedback.", __NAME__);
}

void GlobalPathPlanning::taskActivedCallback()
{
    ROS_INFO("[%s] Task is actived.", __NAME__);
}
    
int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_planning");
    ros::AsyncSpinner spinner(2);
	spinner.start(); //非阻塞
	
	ros::NodeHandle nh, nh_private("~");
	
	GlobalPathPlanning planner;
	if(planner.init(nh, nh_private))
	{
	    planner.start();
	    ros::waitForShutdown();
	}
	
	return 0;
}
