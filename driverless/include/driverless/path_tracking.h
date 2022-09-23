#include <ros/ros.h>
#include "auto_drive_base.h"
#include "utils.hpp"

class PathTracking : public AutoDriveBase
{
public:
	PathTracking();
	virtual ~PathTracking(){};

	virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
	virtual bool start() override;
	virtual void stop() override;
	virtual bool isRunning() override;

	void setTaskMode(const int& task);
	void setExpectSpeed(const float& speed);
	void setEmergencyState(const bool& emergency);

private:
	void cmd1_timer_callback(const ros::TimerEvent&);
	void cmd1_check_timer_callback(const ros::TimerEvent&);
	
	void cmd2_timer_callback(const ros::TimerEvent&);
	void cmd2_check_timer_callback(const ros::TimerEvent&);

	float generateRoadwheelAngleByRadius(const float& radius);
	float limitRoadwheelAngleBySpeed(const float& angle,
                                               const float& speed);
	float generateMaxSpeedByCurvature(const Path& path,
                                                const size_t& begin_idx,
											    const float& curvature_search_distance);
	float generateMaxSpeedToPrepareToStop(const float& dis2park);
	float generateMaxSpeedByParkingPoint(const Path& path);
	float generateMaxSpeedByTrafficLightPoint(const Path& path);
	float generateMaxSpeedBySpeedRange(const Path& path);
	
private:
	int task_mode_;
	bool task_initialized_;

	ros::Timer cmd1_timer_;
	ros::Timer cmd1_check_timer_;
	
	ros::Timer cmd2_timer_;
	ros::Timer cmd2_check_timer_;

	double cmd1_time_; // 指令1更新时间
	double cmd2_time_; // 指令2更新时间

	float expect_speed_; // m/s（前进或后退）
	bool speed_initialized_;

	bool is_emergency_;

	float limit_speed_for_drive_; // km/h
	float limit_speed_for_reverse_; // km/h
	float min_foresight_distance_for_drive_; // 最小前视距离（前进）
	float min_foresight_distance_for_reverse_; // 最小前视距离（后退）
	
	float fd_speed_coefficient_;
	float fd_lateral_error_coefficient_;
	float max_side_acceleration_; // 自车最大侧向加速度
	float max_deceleration_; // 自车最大减速度
};
