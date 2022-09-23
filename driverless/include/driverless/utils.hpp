#ifndef UTILS_NEW_H_
#define UTILS_NEW_H_

#include <ros/ros.h>

#include <cstring>
#include <cmath>
#include <assert.h>
#include <string>
#include <vector>
#include <cstdio>
#include <limits.h>
#include <exception>
#include <fstream>
#include <tinyxml2.h>

#include <driverless_common/structs.h>

/* @brief 角度归一化 (-pi, pi]
 */
static double normalizeRadAngle(double angle)
{
	double res = angle;
	while(res <= -M_PI)
		res += 2 * M_PI;

	while(res > M_PI)
		res -= 2 * M_PI;
	return res;
}

static float getYaw(const Point& point1,
                    const Point& point2)
{
	float yaw = atan2(point1.y - point2.y, point1.x - point2.x);
	
	if(yaw < 0) yaw += 2 * M_PI;
	return yaw;
}

static std::pair<float, float> getDisAndYaw(const Pose& point1,
                                            const Pose& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(y, x); // 当x = 0时，atan2(y, x)返回0
	
	if(dis_yaw.second < 0)
		dis_yaw.second += 2 * M_PI;
	return dis_yaw;
}

static double computeDistance(const Point& point1,
                              const Point& point2)
{
	double dx = point1.x - point2.x;
	double dy = point1.y - point2.y;
	return sqrt(dx * dx + dy * dy);
}

static double computeDistance(const double& x1,
                              const double& y1,
                              const double& x2,
                              const double& y2)
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}

static double computeDistance(const Path& path,
                              const size_t& begin_idx,
                              const size_t& end_idx)
{
	assert(begin_idx >= 0 && begin_idx <= path.final_index);
	assert(end_idx >= 0 && end_idx <= path.final_index);

    double dx = path.points[begin_idx].x - path.points[end_idx].x;
    double dy = path.points[begin_idx].y - path.points[end_idx].y;
    return sqrt(dx * dx + dy * dy);
}

static void transform2DPoint(double& x,
                             double& y,
							 const double& phi,
							 const double& x0,
							 const double& y0)
{
	double temp_x = x;
	double temp_y = y;
	x = temp_x * cos(phi) - temp_y * sin(phi);
	y = temp_x * sin(phi) + temp_y * cos(phi);
	x += x0;
	y += y0;
}

static void transform2DPoints(double xs[4],
                              double ys[4],
							  const double& phi,
							  const double& x0,
							  const double& y0) // 数组作形参将自动转换为指针
{
    for(int i = 0; i < 4; i++)
    {
        double temp_x = xs[i];
        double temp_y = ys[i];
        xs[i] = temp_x * cos(phi) - temp_y * sin(phi);
        ys[i] = temp_x * sin(phi) + temp_y * cos(phi);
		xs[i] += x0;
		ys[i] += y0;
    }
}

/* @brief 在目标路径path中查找距离pose最近的点，返回该点的索引值，遍历搜索，
          滤除与当前航向偏差大于45度的点（如果路径发生交叉，yaw值之差不应小于45度），
          当未查找到最近点或最近点过远时返回路径最大索引值
 * @param path 目标路径
 * @param pose 目标点
 * @param max_match_dis 容许距离误差
 */
static size_t findNearestPointInPath(const Path& path,
                                     const Pose& pose,
                                     const double& max_match_dis)
{
	size_t idx = 0;
	double min_dis = DBL_MAX;
	
	for(size_t i = 0; i < path.final_index; i++)
	{
		double yaw_err = path.points[i].yaw - pose.yaw;
		if(yaw_err > M_PI) yaw_err -= 2 * M_PI;
		else if(yaw_err < -M_PI) yaw_err += 2 * M_PI;
		
		if(fabs(yaw_err) > M_PI / 6) continue;
		double dis = computeDistance(path.points[i].x, path.points[i].y, pose.x, pose.y);
		if(dis < min_dis)
		{
			min_dis = dis;
			idx = i;
		}
	}
	if(min_dis > max_match_dis)
	{
		ROS_ERROR("[findNearestPoint] Cannot find correct nearest point! the nearest point distance is: %.2f.", min_dis);
		ROS_ERROR("[findNearestPoint]  --pose.x: %.2f\t --pose.x:%.2f", pose.x, pose.y);
		return path.final_index;
	}
	return idx;
}

/* @brief 在目标路径path中查找距离(x, y)最近的点，返回该点的索引值，遍历搜索
 * @param path 目标路径
 * @param x 目标点X坐标
 * @param y 目标点Y坐标
 * @param begin_idx 搜索起点索引
 * @param end_idx 搜索终点索引
 */
static size_t findNearestPointInPath(const Path& path,
                                     const double& x,
                                     const double& y,
                                     const size_t& begin_idx,
                                     const size_t& end_idx)
{
	assert(begin_idx >= 0 && end_idx <= path.final_index);
    assert(begin_idx < end_idx);

    size_t idx;
	double min_dis = DBL_MAX;
	
	for(size_t i = begin_idx; i < end_idx; i++)
	{
		double dis = computeDistance(path.points[i].x, path.points[i].y, x, y);
		if(dis < min_dis)
		{
			min_dis = dis;
			idx = i;
		}
	}
	return idx;
}

/* @brief 查找点(x, y)到路径path的最短距离，遍历搜索
 * @param path 目标路径
 * @param x 目标点X坐标
 * @param y 目标点Y坐标
 * @param begin_idx 搜索起点索引
 * @param end_idx 搜索终点索引
 */
static double findMinDistance2Path(const Path& path,
                                   const double& x,
                                   const double& y,
                                   const size_t& begin_idx,
                                   const size_t& end_idx)
{
    assert(begin_idx >= 0 && end_idx <= path.final_index);
    assert(begin_idx < end_idx);

    size_t idx = findNearestPointInPath(path, x, y, begin_idx, end_idx);
   
    double dx = path.points[idx].x - x;
    double dy = path.points[idx].y - y;
    return sqrt(dx * dx + dy * dy);
}

/* @brief 计算点到直线距离
 * @param x1 直线上的第一个点x坐标
 * @param y1 直线上的第一个点y坐标
 * @param x2 直线上的第二个点x坐标
 * @param y2 直线上的第二个点y坐标
 * @param x 点的x坐标
 * @param y 点的y坐标
 */
static double computeDistanceBetweenPointAndLine(const double &x1,
                                                 const double &y1,
                                                 const double &x2,
                                                 const double &y2,
                                                 const double &x,
                                                 const double &y)
{
    double vx = x2 - x1;
    double vy = y2 - y1;
    double dis;
    if(fabs(vx) > 0.0001)
    {
        double k = vy / vx;
        dis = fabs(k * x - y - k * x1 + y1) / sqrt(k * k + 1);
    }
    else
    {
        dis = fabs(x - x1);
    }
    
    return dis;
}

/* @brief 在目标路径path中查找距离(x, y)最近的点，返回该点的索引值，根据参考点搜索
 * @param path 路径
 * @param x 目标点X坐标
 * @param y 目标点Y坐标
 * @param ref_idx 参考点索引，以此参考点展开搜索，加速计算
 * @param min_idx 最小搜索索引
 * @param max_idx 最大搜索索引
 */
static size_t findNearestPointInPath(const Path& path, 
                                     const double& x,
                                     const double& y,
						             const size_t& ref_idx,
                                     const size_t& min_idx,
						             const size_t& max_idx)
{
    assert(ref_idx >= min_idx && ref_idx <= max_idx);
    assert(min_idx >= 0 && max_idx <= path.final_index);
    assert(max_idx - min_idx >= 2);

    // 搜索方向：-1 向后搜索，1 向前搜索，0 搜索完毕
    int search_direction;
    size_t idx = ref_idx;

	if(idx == min_idx)
	{
		idx = min_idx + 1;
		search_direction = 1;
	}
	else if(idx == max_idx)
	{
		idx = max_idx - 1;
		search_direction = -1;
	}
	else
	{
		float dis2ref = pow(path.points[idx].x - x, 2) + pow(path.points[idx].y - y, 2);
		float dis2last = pow(path.points[idx - 1].x - x, 2) + pow(path.points[idx - 1].y - y, 2);
		float dis2next = pow(path.points[idx + 1].x - x, 2) + pow(path.points[idx + 1].y - y, 2);
		
        if(dis2next >= dis2ref && dis2last >= dis2ref)
        {
            search_direction = 0;
        }
        else
        {
            if(dis2next > dis2last) search_direction = -1;
            else search_direction = 1;
        }
	}
	
	while(idx > min_idx && idx < max_idx)
	{
		float dis2ref = pow(path.points[idx].x - x, 2) + pow(path.points[idx].y - y, 2);
		float dis2last = pow(path.points[idx - 1].x - x, 2) + pow(path.points[idx - 1].y - y, 2);
		float dis2next = pow(path.points[idx + 1].x - x, 2) + pow(path.points[idx + 1].y - y, 2);
		
        if((search_direction == 1 && dis2next > dis2ref) ||
		   (search_direction == -1 && dis2last > dis2ref) ||
		   (search_direction == 0))
			break;
        idx += search_direction;
	}

	return idx;
}

/* @brief 查找点(x, y)到路径path的最短距离，根据参考点搜索
 * @param path 路径
 * @param x 目标点X坐标
 * @param y 目标点Y坐标
 * @param ref_idx 参考点索引，以此参考点展开搜索，加速计算
 * @param min_idx 最小搜索索引
 * @param max_idx 最大搜索索引
 */
static double findMinDistance2Path(const Path& path, 
                                   const double& x,
                                   const double& y,
						           const size_t& ref_idx,
                                   const size_t& min_idx,
						           const size_t& max_idx)
{
    assert(ref_idx >= min_idx && ref_idx <= max_idx);
    assert(min_idx >= 0 && max_idx <= path.final_index);
    assert(max_idx - min_idx >= 2);
    
    double min_dis;
	
    double ref_dx = path.points[ref_idx].x - x;
    double ref_dy = path.points[ref_idx].y - y;
    double ref_dis = sqrt(ref_dx * ref_dx + ref_dy * ref_dy);
    
    min_dis = ref_dis;

    size_t idx = findNearestPointInPath(path, x, y, ref_idx, min_idx, max_idx);
    double dx = path.points[idx].x - x;
    double dy = path.points[idx].y - y;
    double dis = sqrt(dx * dx + dy * dy);
    
    min_dis = dis < min_dis ? dis : min_dis;
    
    if(idx == min_idx)
    {
        // 用两点(p1x, p1y)和(p2x, p2y)表示路径切线
	    double p1x, p2x, p1y, p2y;
		p1x = path.points[idx].x;
		p1y = path.points[idx].y;
		p2x = path.points[idx + 2].x;
		p2y = path.points[idx + 2].y;
	    
	    double dis2line = computeDistanceBetweenPointAndLine(p1x, p1y, p2x, p2y, x, y);
	    
	    min_dis = dis2line < min_dis ? dis2line : min_dis;
    }
    else if(idx == max_idx)
    {
        // 用两点(p1x, p1y)和(p2x, p2y)表示路径切线
	    double p1x, p2x, p1y, p2y;
		p1x = path.points[idx - 2].x;
		p1y = path.points[idx - 2].y;
		p2x = path.points[idx].x;
		p2y = path.points[idx].y;
	    
	    double dis2line = computeDistanceBetweenPointAndLine(p1x, p1y, p2x, p2y, x, y);
	    
	    min_dis = dis2line < min_dis ? dis2line : min_dis;
    }

	return min_dis;
}

/* @brief 计算到当前点指定距离的路径点的索引，当路径中的点无法满足条件时，返回终点索引
 * @param path 路径
 * @param expect_dis 期望距离
 * @param begin_idx 搜索起点（当前点）索引
 */
static size_t findPointInPath(const Path& path, 
                              const double& expect_dis,
						      const size_t& begin_idx)
{
    assert(expect_dis >= 0);
    assert(begin_idx >= 0 && begin_idx <= path.final_index - 1);
    
    size_t idx = begin_idx;
    double dis_sum = 0;

    while(dis_sum < expect_dis && idx < path.final_index)
    {
        double dx = path.points[idx].x - path.points[idx + 1].x;
        double dy = path.points[idx].y - path.points[idx + 1].y;
        double dis = sqrt(dx * dx + dy * dy);
		dis_sum += dis;
		idx += 1;
    }

	return idx;
}

static void offsetPoint(GpsPoint& point,
                        const float& offset)
{
	// 车辆前进时，左负（offset应小于0）右正（offset应大于0）
	point.x = offset * sin(point.yaw) + point.x;
	point.y = -offset * cos(point.yaw) + point.y;

}

static double max(double val_1,
                  double val_2)
{
	return val_1 > val_2 ? val_1 : val_2;
}

static void getExtending(Path& path,
				         const float& extending_dis)
{
	int n = 5;
	assert(path.points.size() >= n);
	size_t end_idx = path.points.size() - 1;
	
	// 取最后一个点与倒数第n个点的连线向后插值
	float dx = (path.points[end_idx].x - path.points[end_idx - n + 1].x) / n;
	float dy = (path.points[end_idx].y - path.points[end_idx - n + 1].y) / n;
	float ds = sqrt(dx * dx + dy * dy);

	GpsPoint p;
	float remaind_dis = 0.0;
	size_t j = 1;
	while(remaind_dis < extending_dis)
	{
		p.x = path.points[end_idx].x + dx * j;
		p.y = path.points[end_idx].y + dy * j;
		p.curvature = 0.0;
		path.points.push_back(p);
		
		remaind_dis += ds;
		j++;
	}
}

static bool getCurvature(Path& path)
{
	// 如果路径已经包含曲率信息，无需重复计算
	if(path.has_curvature)
		return true;
	
	auto& points = path.points;
	size_t size = points.size();
	for(int i = 0; i < size - 1; ++i)
	{
		// 归一化旋转角
		float delta_theta = normalizeRadAngle(points[i + 1].yaw - points[i].yaw);
		// 利用两点间距近似弧长
		float length = computeDistance(points[i + 1], points[i]);
		if(length == 0)
			points[i].curvature = 0.0;
		else
			points[i].curvature = delta_theta / length;
	}
	
	// 均值滤波
	int n = 10;
	float sum = 0.0;
	for(int i = 0; i < size; ++i)
	{
		if(i < n)
		{
			sum += points[i].curvature;
		}
		else
		{
			points[i - n / 2].curvature = sum / n;
			sum += (points[i].curvature - points[i - n].curvature);
		}
	}
	
	path.has_curvature = true;
	return true;
}

/* @brief 从文件载入路径点，包括位置、航向、路径曲率，
 *        若文件中不包含曲率信息（均为0），则调用曲率计算函数进行计算
 */
static bool loadPathPoints(const std::string& user,
						   const std::string& file_path,
						   const bool& reverse,
                           Path& path)
{
	std::ifstream in_file(file_path.c_str());
	if(!in_file.is_open())
	{
		ROS_ERROR("[%s] Open %s failed!", user.c_str(), file_path.c_str());
		return false;
	}
	path.clear();
	bool has_curvature = false;

	GpsPoint point;
	std::string line;
	while(in_file.good())
	{
		getline(in_file, line);
		if(line.length() == 0) break; // 处理当文件为空或者末尾空行的情况

		std::stringstream ss(line);
		ss >> point.x >> point.y >> point.yaw >> point.curvature >> point.left_width >> point.right_width;
		
		if(!has_curvature && point.curvature != 0)
			has_curvature = true;
			
		path.points.push_back(point);
	}
	in_file.close();

	if(path.points.size() < 20)
    {
        ROS_ERROR("[%s] Path size is too small (%d)!", user.c_str(), path.points.size());
		return false;
    }

	if(reverse)
	{
		Path temp_path;
        temp_path.points.reserve(path.points.size());
        for(int i = path.points.size() - 1; i >= 0; i--)
        {
            GpsPoint& point = path.points[i];
            temp_path.points.push_back(point);
        }
        path.points.swap(temp_path.points);
	}

	// 设置路径分辨率
	float resolution = 0.1;
	path.resolution = resolution;
	
	// 设置终点索引为最后一个点
	path.final_index = path.points.size() - 1 ;
	path.has_curvature = has_curvature;
	if(!has_curvature) getCurvature(path);
	
	return true;
}

/* @brief 从xml文件载入路径信息
 */
static bool loadPathAppendInfos(const std::string& user,
                                const std::string& file,
								Path& path)
{
	if(path.size() == 0)
	{
		ROS_ERROR("[%s] Please load path points first!", user.c_str());
		return false;
	}

	ParkingPoints& park_points = path.park_points;
	TrafficLightPoints& traffic_light_points = path.traffic_light_points;
	TurnRanges& turn_ranges = path.turn_ranges;
	SpeedRanges& speed_ranges = path.speed_ranges;
	
	using namespace tinyxml2;
	XMLDocument Doc;
	XMLError res = Doc.LoadFile(file.c_str());
	
	if(XML_ERROR_FILE_NOT_FOUND == res)
	{
		ROS_ERROR("[%s] Path infomation file (%s) not exist!", user.c_str(), file.c_str());
		return false;
	}
	else if(XML_SUCCESS != res)
	{
		ROS_ERROR("[%s] Path infomation file (%s) parse error!", user.c_str(), file.c_str());
		return false;
	}
	
	// 指向根节点
	tinyxml2::XMLElement *pRoot = Doc.RootElement();
	if(pRoot == nullptr)
	{
		ROS_ERROR("[%s] Path infomation file (%s) has no root node!", user.c_str(), file.c_str());
		return false;
	}
	
	// 指向一级子节点
	tinyxml2::XMLElement *pParkingPoints = pRoot->FirstChildElement("ParkingPoints");
	if(pParkingPoints)
	{
		// 指向二级子节点
		tinyxml2::XMLElement *pParkingPoint = pParkingPoints->FirstChildElement("ParkingPoint");
		while(pParkingPoint)
		{
			uint32_t id = pParkingPoint->Unsigned64Attribute("id");
			uint32_t index = pParkingPoint->Unsigned64Attribute("index");
			float duration = pParkingPoint->FloatAttribute("duration");
			park_points.points.emplace_back(index, duration);
			
			// 转到下一子节点
			pParkingPoint = pParkingPoint->NextSiblingElement("ParkingPoint");
		}
		// 排序
		park_points.sort();
		ROS_INFO("[%s] Load Parking Points ok.", user.c_str());
	}
	else
		ROS_INFO("[%s] No Parking Points in path info file!", user.c_str());

    // 指向一级子节点
    tinyxml2::XMLElement *pTrafficLightPoints = pRoot->FirstChildElement("TrafficLightPoints");
	if(pTrafficLightPoints)
	{
		// 指向二级子节点
		tinyxml2::XMLElement *pTrafficLightPoint = pTrafficLightPoints->FirstChildElement("TrafficLightPoint");
		while(pTrafficLightPoint)
		{
			uint32_t id = pTrafficLightPoint->Unsigned64Attribute("id");
			uint32_t index = pTrafficLightPoint->Unsigned64Attribute("index");
			traffic_light_points.points.emplace_back(index, 0.0);
			
			// 转到下一子节点
			pTrafficLightPoint = pTrafficLightPoint->NextSiblingElement("TrafficLightPoint");  
		}
		// 排序
		traffic_light_points.sort();
		ROS_INFO("[%s] Load Traffic Light Points ok.", user.c_str());
	}
	else
		ROS_INFO("[%s] No Traffic Light Points in path info file!", user.c_str());

	// 指向一级子节点
	tinyxml2::XMLElement *pTurnRanges = pRoot->FirstChildElement("TurnRanges");
	if(pTurnRanges)
	{
		// 指向二级子节点
		tinyxml2::XMLElement *pTurnRange = pTurnRanges->FirstChildElement("TurnRange");
		while(pTurnRange)
		{
			int type = pTurnRange->IntAttribute("type");
			size_t start = pTurnRange->Unsigned64Attribute("start");
			size_t end = pTurnRange->Unsigned64Attribute("end");
			turn_ranges.ranges.emplace_back(type, start, end);
			
			// 转到下一子节点
			pTurnRange = pTurnRange->NextSiblingElement("TurnRange"); 
		}
		ROS_INFO("[%s] Load turn ranges ok.", user.c_str());
	}
	else
		ROS_INFO("[%s] No turn ranges in path info file!", user.c_str());
	
	// 指向一级子节点
	tinyxml2::XMLElement *pSpeedRanges = pRoot->FirstChildElement("SpeedRanges");
	if(pSpeedRanges)
	{
		// 指向二级子节点
		tinyxml2::XMLElement *pSpeedRange = pSpeedRanges->FirstChildElement("SpeedRange");
		while(pSpeedRange)
		{
			float speed = pSpeedRange->FloatAttribute("speed");
			size_t start = pSpeedRange->Unsigned64Attribute("start");
			size_t end = pSpeedRange->Unsigned64Attribute("end");
			speed_ranges.ranges.emplace_back(speed, start, end);
			
			// 转到下一子节点
			pSpeedRange = pSpeedRange->NextSiblingElement("SpeedRange"); 
		}
		ROS_INFO("[%s] Load speed ranges ok.", user.c_str());
	}
	else
		ROS_INFO("[%s] No speed ranges in path info file!", user.c_str());
	
	return true;
}

#endif
