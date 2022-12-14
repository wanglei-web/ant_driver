#include <ros/ros.h>
#include "auto_drive_base.h"
#include "utils.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <perception_msgs/Obstacle.h>
#include <perception_msgs/ObstacleArray.h>
#include <obu_msgs/OBU_fusion.h>
#include <light_msgs/Light.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

class LocalPathPlanning : public AutoDriveBase
{
public:
	LocalPathPlanning();
	virtual ~LocalPathPlanning(){};
	
	virtual bool init(ros::NodeHandle nh, ros::NodeHandle nh_private) override;
	virtual bool start() override;
	virtual void stop() override;
	virtual bool isRunning() override;
	
	void setTaskMode(const int& task);
	float getOffset();
	bool getAvoidingState();
	bool getEmergencyState();

private:
    void cmd_timer_callback(const ros::TimerEvent&);
	void addTurnRanges(const size_t& nearest_idx, const size_t& farthest_idx);
	void addSpeedRanges(const size_t& nearest_idx, const size_t& farthest_idx);
	void addParkingPoints(const size_t& nearest_idx, const size_t& farthest_idx);
	void addTrafficLightPoints(const size_t& nearest_idx, const size_t& farthest_idx);

	void obstacles_callback(const perception_msgs::ObstacleArray::ConstPtr& obstacles);
	void obu_callback(const obu_msgs::OBU_fusion::ConstPtr& container);
	void light_callback(const light_msgs::Light::ConstPtr& container);

	void findNearestObstacleInPath(const perception_msgs::ObstacleArray::ConstPtr& obstacles,
										 const Path& path,
							 			 const size_t& nearest_idx,
							 			 const size_t& farthest_idx,
										 const float& min_x_in_sensor,
										 bool& obs_in_path,
										 float& nearest_obs_dis,
										 size_t& nearest_obs_idx);
	bool computeOffset(const perception_msgs::ObstacleArray::ConstPtr& obstacles,
							 const Path& path,
							 const size_t& nearest_idx,
							 const size_t& farthest_idx,
							 const float& left_width,
							 const float& right_width,
							 const float& min_x_in_sensor,
							 float& passable_offset);
	bool judgeEmergency(const perception_msgs::ObstacleArray::ConstPtr& obstacles,
							  const float& safe_distance,
							  size_t& nearest_obs_idx);

	int judgeWhichSide(const double& x,
							 const double& y,
							 const Path& path,
							 const size_t& nearest_idx,
							 const size_t& farthest_idx);
	int judgeWhichSide(const perception_msgs::Obstacle& obs,
							 const Path& path,
							 const size_t& nearest_idx,
							 const size_t& farthest_idx);
	void getGlobalObstacle(const perception_msgs::Obstacle& obs,
								 double& x,
								 double& y);
	void getGlobalObstacle(const perception_msgs::Obstacle& obs,
								 double xs[4],
								 double ys[4]);
	void computeObstacleCenter(const perception_msgs::Obstacle& obs,
									 double& obs_x,
									 double& obs_y);
	void computeObstacleVertex(const perception_msgs::Obstacle& obs,
                                     double obs_xs[4],
									 double obs_ys[4]);
	void computeObstacleOrientation(const perception_msgs::Obstacle& obs,
                                          double& phi);
	void transformSensor2Base(double& phi);
	void transformSensor2Base(double& x,
                                    double& y);
	void transformSensor2Base(double xs[4],
                                    double ys[4]);
	void transformBase2Gps(double& x,
                                 double& y);
	void transformBase2Gps(double xs[4],
                                 double ys[4]);
	void transformGps2Global(double& x,
                                   double& y);
	void transformGps2Global(double xs[4],
                                   double ys[4]);
	double computeObstacleDistance2Ego(const perception_msgs::Obstacle& obs);
	double computeGapBetweenObstacleAndPath(const perception_msgs::Obstacle& obs,
												  const Path& path,
												  const size_t& nearest_idx,
												  const size_t& farthest_idx);
	double computeMaxDistanceBetweenObstacleAndPath(const perception_msgs::Obstacle& obs,
												          const Path& path,
												          const size_t& nearest_idx,
												          const size_t& farthest_idx);
	void publishMarkerArray(const perception_msgs::ObstacleArray::ConstPtr obstacles,
                                  const bool& obs_in_path,
								  const size_t& nearest_obs_idx);
	void publishPath(ros::Publisher& pub,
                           const Path& path_to_pub,
						   const size_t& begin_idx,
						   const size_t& end_idx,
						   const std::string& frame_id);
	void transformGlobal2Gps(double& x,
                                   double& y);
	void transformGps2Base(double& x,
                                 double& y);
    bool count(const size_t& threshold,
                     const bool& flag,
                     size_t& cnt);

private:
	int task_mode_;
	bool task_initialized_;

	std::string sub_topic_obstacle_array_;
	std::string sub_topic_obu_fusion_;
	std::string sub_topic_light_;

	std::string pub_topic_marker_array_;
	std::string pub_topic_global_path_;
	std::string pub_topic_local_path_;

	std::string marker_array_frame_id_;
	std::string global_path_frame_id_;
	std::string local_path_frame_id_;

	ros::Subscriber sub_obstacle_array_;
	ros::Subscriber sub_obu_fusion_;
	ros::Subscriber sub_light_;

	ros::Publisher pub_marker_array_;
	ros::Publisher pub_global_path_;
	ros::Publisher pub_local_path_;

	ros::Timer cmd_timer_;

	float max_match_distance_; // ????????????????????????????????????????????????
	float max_deceleration_; // ?????????????????????
	float default_local_path_length_;
	float min_following_distance_; // ??????????????????
	float max_search_range_; // ??????????????????
	float safe_margin_; // ??????????????????
	float lane_left_width_;
	float lane_right_width_;

    bool use_avoiding_;
    float max_avoiding_speed_;
    float min_offset_increment_; // ????????????offset????????????
    float max_avoiding_distance_; // ???????????????????????????
    float min_avoiding_distance_; // ???????????????????????????
    int repeat_detection_threshold_; // ??????????????????
    int delay_threshold_; // ????????????
	
	bool emergency_state_; // ????????????
	bool is_following_; // ??????????????????
	bool is_avoiding_; // ??????????????????
	float offset_;
	double obstacle_array_time_; // ???????????????????????????
	
	bool emergency_state_for_obu_; // ???????????????obu???
	double emergency_state_time_for_obu_;
	
	bool get_terminal_from_obu_;

	bool light_passable_; // ???????????????
	int light_remain_time_; // ?????????????????????
	double light_time_; // ???????????????????????????
	
	float nearest_obstacle_distance_in_global_path_;
	float nearest_obstacle_distance_in_local_path_;
	size_t nearest_obstacle_index_in_global_path_;
	size_t nearest_obstacle_index_in_local_path_;
	bool obstacle_in_global_path_;
	bool obstacle_in_local_path_;

	// sensor????????????base???????????????
	// sensor???X?????????base???X??????????????????????????????
	float dx_sensor2base_;
	float dy_sensor2base_;
	float phi_sensor2base_;
	
	// base????????????gps???????????????
	// base???X?????????gps???X??????????????????????????????
	float dx_base2gps_;
	float dy_base2gps_;
	float phi_base2gps_;
	
	// gps????????????global???????????????
	// gps???X?????????global???X??????????????????????????????
	float dx_gps2global_;
	float dy_gps2global_;
	float phi_gps2global_;

};
