#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <vector>
#include <unordered_set>
#include <utility>
#include <fstream>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <erp_driver/erpCmdMsg.h>
#include <erp_driver/erpStatusMsg.h>
#include "PIDController.h"

using clockclock = std::chrono::high_resolution_clock;

// Publisher
ros::Publisher cmd_pub;
ros::Publisher bigObsSteer_pub;
ros::Publisher smallObsSteer_pub; 
ros::Publisher car_pub;
ros::Publisher vizselpath_pub;
ros::Publisher vizallpath_pub;
ros::Publisher vizCheckArea_pub;
ros::Publisher vizTM_pub;
ros::Publisher vizCM_pub;
ros::Publisher vizOBS_pub;

std::string smallObsState = "Small_Obstacle_avoiding_path"; // 미리 가지고 있는 문자열
std::string bigObsState = "Big_Obstacle_avoiding_path"; // 미리 가지고 있는 문자열

// Global var
double car_x, car_y; // Global position of Car
double car_yaw = 0;  // Yaw of Car
double last_selidx = 40;

bool front_obs = false;

// Params
int Steering_Offset = 2;
double tracking_distance = 2;
double small_min_r = 0.40;
double small_max_r = 0.40;
double big_min_r = 0.55;
double big_max_r = 0.6;
double crash_dist = 2;

double obs_detect_dist = 2.4;
int obs_detect_left = 35;
int obs_detect_right =75;

bool gridReady = false;
bool gpathready = false;
bool opathready = false;
bool gpath2locReady = false;
bool opath2locReady = false;

bool bigobs = false;

nav_msgs::OccupancyGrid grid_;
std::vector<std::pair<double, double>> gpath2loc; // 전역경로를 local로 projection
std::vector<std::pair<double, double>> opath2loc;
std::vector<std::pair<double, double>> globalPath;
std::vector<std::pair<double, double>> obsPath;
std::vector<nav_msgs::Path> vizPaths; // 모든 경로 시각화용
std::vector<int> checkarea; // 주황 포인트 시각화용

struct pathPoint
{
    double x;
    double y;
    double yaw;
    double cl; // point 까지 누적 길이
};

double rad2deg(double rad) { return rad * 180.0 / M_PI; }
double deg2rad(double deg) { return deg * M_PI / 180.0; }

double dist(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

void odom_gps_Callback(const nav_msgs::Odometry::ConstPtr& msg){
    car_x = msg->pose.pose.position.x;
    car_y = msg->pose.pose.position.y;
}

void global_path_Callback(const nav_msgs::Path::ConstPtr& msg){
    globalPath.clear();
    for (const auto& pose_stamped : msg->poses){
        globalPath.emplace_back(pose_stamped.pose.position.x, pose_stamped.pose.position.y);
    }
    gpathready = true;
}


void vehicle_yaw_Callback(const std_msgs::Float32::ConstPtr& msg){
    car_yaw = msg->data;
    if(gpathready){
        gpath2loc.clear();
        for (const auto& gpoint : globalPath){
            double dx = gpoint.first- car_x;
            double dy = gpoint.second - car_y;
            
            double local_x = cos(-car_yaw) * dx - sin(-car_yaw) * dy;
            double local_y = sin(-car_yaw) * dx + cos(-car_yaw) * dy;

            gpath2loc.emplace_back(local_x, local_y);
        }
        gpath2locReady = true;
    }
}


void local_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    grid_ = *msg;
    gridReady = true;
}

// occupancy grid map과 path를 대조하는 함수.
bool isObstacleNear(double x, double y, double radius) {
    if (gridReady){
        double res = grid_.info.resolution;
        int width = grid_.info.width;
        int height = grid_.info.height;

        int gx = (x - grid_.info.origin.position.x) / res;
        int gy = (y - grid_.info.origin.position.y) / res;
        int r = radius / res;

        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                int nx = gx + dx;
                int ny = gy + dy;

                if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                    continue;

                if (std::hypot(dx * res, dy * res) > radius)
                    continue;

                int index = ny * width + nx;
                if (grid_.data[index] >= 50)  // threshold // 사실 occ grid에는 찐 장애물은 100 확장 영역은 99로 publish됨.
                    return true;
            }
        }
        return false;
    }
    else{
        return false;
    }
}

int findPathIdxFromSteering(double input_steer, const std::vector<std::vector<pathPoint>>& Path81) {
    int best_idx = 40; 
    double min_diff = 1e9; 

    for (int k = 0; k < 81; k++) {
        if (Path81[k].empty()) continue;

        int pathlen = static_cast<int>(Path81[k].size());
        int halflen = pathlen / 2;
        
        // Steering_Offset은 전역 변수이므로 접근 가능
        int target_idx = halflen + Steering_Offset;
        if (target_idx >= pathlen) target_idx = pathlen - 1;
        if (target_idx < 0) target_idx = 0;

        double dx = Path81[k][target_idx].x;
        double dy = Path81[k][target_idx].y;
        
        double calc_angle = std::atan2(dy, dx);
        double calc_steer = - (2000.0 * calc_angle / 0.4922);

        double diff = std::abs(calc_steer - input_steer);

        if (diff < min_diff) {
            min_diff = diff;
            best_idx = k;
        }
    }
    return best_idx;
}

// 주황색 확인용 마커를 발행하는 함수. 
bool vizisObstacleNear(double x, double y, double radius) {
    if (gridReady){
        double res = grid_.info.resolution;
        int width = grid_.info.width;
        int height = grid_.info.height;

        int gx = (x - grid_.info.origin.position.x) / res;
        int gy = (y - grid_.info.origin.position.y) / res;
        int r = radius / res;

        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                int nx = gx + dx;
                int ny = gy + dy;

                if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                    continue;

                if (std::hypot(dx * res, dy * res) > radius)
                    continue;

                int index = ny * width + nx;
                // if (grid_.data[index] == 100)  // threshold
                checkarea.push_back(index);
            }
        }
        return false;
    }
    else{
        return false;
    }
}

void vizCheckArea() {
    if (!gridReady) return;

    double res = grid_.info.resolution;
    int width = grid_.info.width;

    std::unordered_set<int> unique_indices(checkarea.begin(), checkarea.end());

    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "check_area";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = res;
    marker.scale.y = res;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    // 3. 인덱스를 좌표로 변환 후 마커에 추가
    for (int index : unique_indices) {
        int nx = index % width;
        int ny = index / width;

        geometry_msgs::Point p;
        p.x = (nx + 0.5) * res + grid_.info.origin.position.x;
        p.y = (ny + 0.5) * res + grid_.info.origin.position.y;
        p.z = -0.5;
        marker.points.push_back(p);
    }

    vizCheckArea_pub.publish(marker);
}



void vizSelPath(int selIDX){
    nav_msgs::Path selected = vizPaths[selIDX];
    selected.header.stamp = ros::Time::now();
    selected.header.frame_id = "base_link"; // 또는 odom 등 frame에 맞게
    vizselpath_pub.publish(selected);
}

void vizAllPaths() {
    visualization_msgs::MarkerArray marker_array;
    ros::Time now = ros::Time::now();

    for (int i = 0; i < vizPaths.size(); ++i) {
        const auto& path = vizPaths[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";  // 또는 "odom"
        marker.header.stamp = now;
        marker.ns = "paths";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.01;

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        for (const auto& pose_stamped : path.poses) {
            geometry_msgs::Point p;
            p.x = pose_stamped.pose.position.x;
            p.y = pose_stamped.pose.position.y;
            p.z = 0.1;
            marker.points.push_back(p);
        }

        marker_array.markers.push_back(marker);
    }

    vizallpath_pub.publish(marker_array);
}

void vizTrackingMarker(const std::vector<std::pair<double, double>>& points, const std::string& color) {
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "base_link";
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = "tentacle_points";
    points_marker.id = 1;
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = 0.1;
    points_marker.scale.y = 0.1;
    points_marker.lifetime = ros::Duration();

    if (color == "yellow") {
        points_marker.color.r = 1.0;
        points_marker.color.g = 1.0;
        points_marker.color.b = 0.0;
    } else { 
        points_marker.color.r = 0.0;
        points_marker.color.g = 0.0;
        points_marker.color.b = 1.0;
    }
    points_marker.color.a = 0.7; 

    for (const auto& pair : points) {
        geometry_msgs::Point p;
        p.x = pair.first;
        p.y = pair.second;
        p.z = 0;
        points_marker.points.push_back(p);
    }

    vizTM_pub.publish(points_marker);
}

void vizCrashMarker(const std::vector<std::pair<double, double>>& points, const std::string& color) {
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "base_link";
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = "tentacle_points";
    points_marker.id = 1;
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = 0.15;
    points_marker.scale.y = 0.15;
    points_marker.lifetime = ros::Duration();

    if (color == "yellow") {
        // 노란색 (R=1, G=1, B=0)
        points_marker.color.r = 1.0;
        points_marker.color.g = 1.0;
        points_marker.color.b = 0.0;
    } else { // 기본값 또는 "blue"일 경우
        // 파란색 (R=0, G=0, B=1)
        points_marker.color.r = 0.0;
        points_marker.color.g = 0.0;
        points_marker.color.b = 1.0;
    }
    points_marker.color.a = 0.3;

    // 벡터의 점들을 마커에 추가
    for (const auto& pair : points) {
        geometry_msgs::Point p;
        p.x = pair.first;
        p.y = pair.second;
        p.z = 0;
        points_marker.points.push_back(p);
    }

    vizCM_pub.publish(points_marker);
}

void vizObsMarker(const std::vector<std::pair<double, double>>& points, const std::string& color) {
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "base_link";
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = "tentacle_points";
    points_marker.id = 1;
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = 0.15;
    points_marker.scale.y = 0.15;
    points_marker.lifetime = ros::Duration();

    if (color == "white") {
        // 노란색 (R=1, G=1, B=0)
        points_marker.color.r = 1.0;
        points_marker.color.g = 1.0;
        points_marker.color.b = 1.0;
    } else { // 기본값 또는 "blue"일 경우
        // 파란색 (R=0, G=0, B=1)
        points_marker.color.r = 0.0;
        points_marker.color.g = 0.0;
        points_marker.color.b = 1.0;
    }
    points_marker.color.a = 1.0;

    // 벡터의 점들을 마커에 추가
    for (const auto& pair : points) {
        geometry_msgs::Point p;
        p.x = pair.first;
        p.y = pair.second;
        p.z = 0.2;
        points_marker.points.push_back(p);
    }

    vizOBS_pub.publish(points_marker);
}

std::pair<double,double> bezierPoint(double t,
                                    const std::pair<double,double>& P0,
                                    const std::pair<double,double>& P1,
                                    const std::pair<double,double>& P2,
                                    const std::pair<double,double>& P3) {
    double x = pow(1-t,3)*P0.first
            + 3*pow(1-t,2)*t*P1.first
            + 3*(1-t)*t*t*P2.first
            + pow(t,3)*P3.first;

    double y = pow(1-t,3)*P0.second
            + 3*pow(1-t,2)*t*P1.second
            + 3*(1-t)*t*t*P2.second
            + pow(t,3)*P3.second;

    return {x,y};
}

void Compute(){

    ROS_INFO("------------------------------");
    auto pathGen_start_time = clockclock::now();

    // 거리는 모두 m단위.
    // ego(local)의 중심 좌표계는 base_link. (not velodyne)
    // base_link : 차량의 중심 (wheelbase 중심).

    // Tentacle Parameters
    double m_dist_adj_pts = 0.1; // 점 사이 거리
    double m_center2rear = 0; // (0,0)에서 x축으로 shift하기 위한 변수
    double m_velo_e = 9;
    double m_velo_s = 0.9;
    double m_rho = 1.12; //1.12
    double v_j = 2.5;
    double sizeratio = 0.15; // 전체적인 크기 파라미터
    double q_tentacle = pow((v_j - m_velo_s)/(m_velo_e - m_velo_s), (1.0 / 1.2));
    double l_outmost = (8 + 33.5 * pow(q_tentacle, 1.2)) * sizeratio;
    double R_outmost = l_outmost / ((0.6 * M_PI) * (1 - pow(q_tentacle, 0.9)));
    double dphi_tentacle[81];
    double l_tentacle[81];
    double r_tentacle[81];

    std::vector<double> pathlen2obs(81);
    std::vector<bool> isPathSafe(81);
    std::vector<bool> isObstacle(81);
    std::fill(isPathSafe.begin(), isPathSafe.end(), true); 
    std::fill(isObstacle.begin(), isObstacle.end(), true); 

    std::vector<std::vector<pathPoint>> Path81(81); // 판단을 위한 생성한 경로 저장 벡터
    std::vector<std::pair<double,double>> TrackingPoints(81); // Path Tracking Scoreing을 위한 Point위치. bule marker로 표시됨

    // Make 81 Paths
    if (bigobs == false){ // Small Obs
        r_tentacle[40] = 999999.9;
        for (int k = 0; k < 41; k++){

            nav_msgs::Path path;
            path.header.frame_id = "base_link";

            l_tentacle[k] = l_outmost + 20 * sqrt((k + 1)/ 40.0) * sizeratio;
            r_tentacle[k] = pow(m_rho, k) * R_outmost;
            dphi_tentacle[k] = m_dist_adj_pts / abs(r_tentacle[k]); 

            double past_px = 0;
            double past_py = 0;
            bool firstflag = true;

            for (int m = 0; m < floor(l_tentacle[k]/m_dist_adj_pts); m++){ 

                double p_x = r_tentacle[k] * sin(dphi_tentacle[k] * m) - m_center2rear;
                double p_y = r_tentacle[k] * (1 - cos(dphi_tentacle[k] * m));
                double p_r = 0;
                if(firstflag){
                    p_r = 0;
                    firstflag = false;
                    past_px = p_x;
                    past_py = p_y;
                }
                else{
                    double dx = p_x - past_px;
                    double dy = p_y - past_py;
                    p_r = atan2(dy,dx);
                }
                double p_cl = m * 0.1;

                if(m == static_cast<int>(tracking_distance * 10)){
                    TrackingPoints[k] = {p_x, p_y};
                }
                
                // pose is for visualization
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "base_link";
                pose.pose.position.x = p_x;
                pose.pose.position.y = p_y;
                path.poses.push_back(pose);

                Path81[k].emplace_back(pathPoint{p_x, p_y, p_r,p_cl});
            }
            vizPaths[k] = path;
        }
        
        for (int k = 41; k < 81; k++){ // FROM RIGHT TO CENTER

            nav_msgs::Path path;
            path.header.frame_id = "base_link";

            l_tentacle[k] = l_outmost + 20 * sqrt((k - 40) / 40.0) * sizeratio;
            r_tentacle[k] = -pow(m_rho, (k - 41)) * R_outmost;
            dphi_tentacle[k] = m_dist_adj_pts / abs(r_tentacle[k]);    // angle which makes the distance between two adjacent point 10 cm

            double past_px = 0;
            double past_py = 0;
            bool firstflag = true;

            for (int m = 0; m < floor(l_tentacle[k]/m_dist_adj_pts); m++){

                double p_x = - r_tentacle[k] * sin(dphi_tentacle[k] * m) - m_center2rear;
                double p_y = r_tentacle[k] * (1 - cos(dphi_tentacle[k] * m));
                double p_r = 0;
                if(firstflag){
                    p_r = 0;
                    firstflag = false;
                    past_px = p_x;
                    past_py = p_y;
                }
                else{
                    double dx = p_x - past_px;
                    double dy = p_y - past_py;
                    p_r = atan2(dy,dx);
                }
                double p_cl = m * 0.1;

                if(m == static_cast<int>(tracking_distance * 10)){
                    TrackingPoints[k] = {p_x, p_y};
                }

                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "base_link";
                pose.pose.position.x = p_x;
                pose.pose.position.y = p_y;
                path.poses.push_back(pose);

                Path81[k].emplace_back(pathPoint{p_x, p_y, p_r, p_cl});
            }
            vizPaths[k] = path;
        }
    }
    else{ // Big Obs
        double target_x = 3.5;     // 목표 지점들의 x값
        double max_y_offset = 2; // 목표 지점들의 y 범위 (- max_y_offset ~ + max_y_offset)
        int num_tentacles = 81;  // 경로의 개수 -> small obs 개수와 맞추기.
        int resolution = 25;
        for (int k = 0; k < num_tentacles; k++) {
            double target_y = -max_y_offset + 2*max_y_offset * (k/(double)(num_tentacles-1));

            // 베지어 곡선을 통한 생성
            std::pair<double,double> P0 = {0.0, 0.0};
            std::pair<double,double> P3 = {target_x, target_y};
            std::pair<double,double> P1 = {target_x*0.50, 0.0};
            std::pair<double,double> P2 = {target_x*0.50, target_y};

            nav_msgs::Path path;
            path.header.frame_id = "base_link";

            double past_px = 0;
            double past_py = 0;
            bool firstflag = true;

            for (int i = 0; i <= resolution; i++) {
                double t = i / (double)resolution;
                auto [px, py] = bezierPoint(t, P0, P1, P2, P3);

                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "base_link";
                pose.pose.position.x = px;
                pose.pose.position.y = py;
                path.poses.push_back(pose);

                double r;
                double cl;
                if(firstflag){
                    r = 0;
                    cl = 0;
                    firstflag = false;
                    past_px = px;
                    past_py = py;
                }
                else{
                    double dx = px - past_px;
                    double dy = py - past_py;
                    r = atan2(dy,dx);
                    cl = sqrt(dx * dx + dy * dy);
                }

                Path81[k].push_back({px, py, r, cl});

                if(i == static_cast<int>(resolution * 0.5)){
                    TrackingPoints[k] = {px, py};
                }
            }
            vizPaths[k] = path;
        }
    }
    
    vizAllPaths(); // 생성한 경로를 visualization
    vizTrackingMarker(TrackingPoints, "blue"); // Tracking scoring에 사용할 point visualization
    auto pathGen_end_time = clockclock::now();
    auto pathGen_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(pathGen_end_time - pathGen_start_time);


    // Get Path Info
    // Small Obs와 Big Obs의 Safe Path 판단 기준이 다름
    // Crash distance는 Crash Distance 안에 장애물이 있을 경우에만 해당 경로를 not safe하다고 판단.
    // Small Obs는 자세한 제어가 필요함으로 Crash distance를 사용.
    // Big Obs는 상대적으로 러프한 판단이 요구됨으로 Crash distance를 사용하지 않음.
    auto pathInfo_start_time = clockclock::now();
    if(bigobs == false){ // Small obs
        std::vector<std::pair<double,double>> CrashPoints(81);
        for (int k = 0; k < 81; k++){

            for (const auto& path_point : Path81[k]){
                double c_l = path_point.cl;
                if(std::abs(c_l - crash_dist) < 0.001){
                    double p_x = path_point.x;
                    double p_y = path_point.y;
                    CrashPoints[k] = {p_x, p_y};
                }
            }
        }
        vizCrashMarker(CrashPoints, "yellow"); // Crash Distance 지점을 viz

        checkarea.clear();
        for (int k = 0; k < 81; k++) {      
            bool safeFlag = true;

            int path_idx = 0;

            for (const auto& path_point : Path81[k]){
                path_idx++;
                if(path_idx >= 5){
                    double p_x_rear = path_point.x;
                    double p_y_rear = path_point.y;
                    double pyaw = path_point.yaw;
                    double c_l = path_point.cl;

                    double car_len = 0.5;
                    double p_x_front = p_x_rear + car_len * cos(pyaw); //2 * center2rear == wheelbase
                    double p_y_front = p_y_rear + car_len * sin(pyaw);
                    
                    bool front_check = false;
                    bool rear_check = false;
                    double check_r = 0.5;
                    
                    if (gridReady == true){

                        // base link와 velodyne 기준 좌표계는 x축 기준 0.82 차이남. 좌표계 매칭.
                        // front_check = isObstacleNear(p_x_front - 0.82, p_y_front, check_r); 
                        rear_check  = isObstacleNear(p_x_rear - 0.82, p_y_rear, check_r);

                        // frontarea & reararea is Visualization of orange points 
                        // it may use quite lots of computing resources(drop loop Hz)
                        // so if you don't need to use these points, comment out the following two lines.
                        // bool frontarea = vizisObstacleNear(p_x_front - 0.82, p_y_front, check_r);
                        bool reararea = vizisObstacleNear(p_x_rear - 0.82, p_y_rear, check_r);

                        if(front_check == true || rear_check == true){
                            isPathSafe[k] = false;
                            pathlen2obs[k] = c_l;
                            safeFlag = false;
                            break;
                        }

                        // Set Crash Distance
                        // if (c_l > crash_dist){ 
                        //     if(front_check == true || rear_check == true){
                        //         // 장애물이 있어도 1.5m이내는 safePath라고 설정.
                        //         // Pathlen2obs는 장애물 탐지와 마찬가지로 설정.
                        //         isPathSafe[k] = true;
                        //         pathlen2obs[k] = c_l;
                        //         safeFlag = false;
                        //         break;
                        //     }
                        // }
                        // else{
                        //     if(front_check == true || rear_check == true){
                        //         isPathSafe[k] = false;
                        //         pathlen2obs[k] = c_l;
                        //         safeFlag = false;
                        //         break;
                        //     }
                        // }
                    }
                    else{
                        ROS_ERROR("OccGrid does not received YET!");
                        return;
                    }
                }
            }
            if (safeFlag == true){
                pathlen2obs[k] = Path81[k].back().cl; // safepath이면 마지막 누적거리가 장애물까지의 거리
            }

        }
    }
    else{ // Big Obs
        checkarea.clear();
        for (int k = 0; k < 81; k++) {
            bool safeFlag = true;
            int path_idx = 0;
            for (const auto& path_point : Path81[k]){
                path_idx++;
                double p_x_rear = path_point.x;
                double p_y_rear = path_point.y;
                double pyaw = path_point.yaw;
                double c_l = path_point.cl;

                double car_len = 0.5;
                double p_x_front = p_x_rear + car_len * cos(pyaw); //2 * center2rear == wheelbase
                double p_y_front = p_y_rear + car_len * m_center2rear * sin(pyaw);
                
                bool front_check = false;
                bool rear_check = false;

                double check_r = 0.4;
                
                if (gridReady == true){

                    // base link와 velodyne 기준 좌표계는 x축 기준 0.82 차이남. 좌표계 매칭.
                    front_check = isObstacleNear(p_x_front - 0.82, p_y_front, check_r); 
                    rear_check  = isObstacleNear(p_x_rear - 0.82, p_y_rear, check_r);

                    bool frontarea = vizisObstacleNear(p_x_front - 0.82, p_y_front, check_r);
                    bool reararea = vizisObstacleNear(p_x_rear - 0.82, p_y_rear, check_r);

                    
                    if (front_check == true || rear_check == true){
                        isPathSafe[k] = false;
                        pathlen2obs[k] = c_l;
                        safeFlag = false;
                        break;
                    }
                }
                else{
                    ROS_ERROR("OccGrid does not received YET!");
                    return;
                }
            }
            if (safeFlag == true){
                pathlen2obs[k] = Path81[k].back().cl; // safepath이면 마지막 누적거리가 장애물까지의 거리
            }
        }
    }
    vizCheckArea();
    auto pathInfo_end_time = clockclock::now();
    auto pathInfo_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(pathInfo_end_time - pathInfo_start_time);


    /////////////////////////////////////////////////////////
    // Path choosing Algorithm //////////////////////////////
    /////////////////////////////////////////////////////////

    auto pathSelect_start_time = clockclock::now();

    bool all_safe = std::all_of(isPathSafe.begin(), isPathSafe.end(), [](bool v) { return v; });   // vector가 모두 true인지
    bool all_danger = std::none_of(isPathSafe.begin(), isPathSafe.end(), [](bool v) { return v; }); // vector가 모두 false인지

    int selIDX = 1;
    double steering = 0;
    double brake = 0;
    double speed = 0;

    if(all_danger){
        // Not Drivable Case Speed & Brake

        ROS_WARN("ALL PATH is DANGER");

        brake = 0;
        speed = 35;
        
        selIDX = last_selidx;
        vizSelPath(selIDX);
        // Compute Steering
        int pathlen = static_cast<int>(Path81[selIDX].size());
        int halflen = pathlen / 2;

        steering = std::atan2(Path81[selIDX][halflen + Steering_Offset].y, Path81[selIDX][halflen].x);
        steering = - int(2000 * steering / 0.4922); // left is -

        if (steering >= 2000){
            steering = 2000;
        }
        if(steering <= -2000){
            steering = -2000;
        }
    
    }
    else if(all_safe){
        // Path tracking!
        ROS_INFO("[ == All Safe == ]");
        int nearest_IDX = -1;
        double min_d = 10000;
        
        if(gpath2locReady){
            for(int idx = 0; idx < globalPath.size(); idx++){
                double d = dist(car_x, car_y, globalPath[idx].first, globalPath[idx].second);
                if(d < min_d){
                    min_d = d;
                    nearest_IDX = idx;
                }
            }
        }
        else{
            ROS_WARN("Global path is not received");
            return;
        }
        
    
        // 차량이 경로의 왼쪽에 있는지, 오른쪽에 있는지 알아야 함
        // V1 : nearest point -> car, V2 : nearest point -> near + n idx point

        int pathYawIdx = 5;
        double v1_x;
        double v1_y;
        double v2_x;
        double v2_y;

        // if((pathYawIdx + nearest_IDX) > obsPath.size() - 1);

        v1_x = car_x - globalPath[nearest_IDX].first;
        v1_y = car_y - globalPath[nearest_IDX].second;

        v2_x = globalPath[nearest_IDX + pathYawIdx].first - globalPath[nearest_IDX].first;
        v2_y = globalPath[nearest_IDX + pathYawIdx].second - globalPath[nearest_IDX].second;
        



        double vecResult = v1_x * v2_y - v1_y * v2_x;
        
        // vecResult가 양수면 path의 오른쪽에, 음수면 path의 왼쪽에.

        // Stanley는 2개의 score를 통해 작동(path와 차량의 거리, path와 차량의 각도 차이)

        double pathDist = std::sqrt(v1_x * v1_x + v1_y * v1_y);

        double pathDeg = rad2deg(std::atan2(v2_y, v2_x));
        double carDeg = rad2deg(car_yaw);
        double degDiff = pathDeg - carDeg;
        ROS_INFO("pathDeg : %.2f",pathDeg);
        ROS_INFO("carDeg : %.2f",carDeg);
        double normalized_degDiff = std::fabs(std::fmod(degDiff + 540.0, 360.0)) - 180.0;
        ROS_INFO("norm degdiff : %.2f", normalized_degDiff);

        PIDController angleMatch_controller(15, 1, 0.1);
        PIDController distMatch_controller(10, 1, 0.1);

        double angleMatch_output = angleMatch_controller.compute(normalized_degDiff, 0);
        double distMatch_output;

        ROS_INFO("pathDist : %.2f", pathDist);
        if(vecResult >= 0){
            distMatch_output = distMatch_controller.compute(pathDist, 0);
        }
        else{
            distMatch_output = - distMatch_controller.compute(pathDist, 0);
        }

        double aGain = 3;
        double dGain = 70;
        steering = aGain * angleMatch_output + dGain * distMatch_output;

        if(steering > 2000){
            steering = 2000;
        }
        if(steering < -2000){
            steering = -2000;
        }

        int estimated_path_idx = findPathIdxFromSteering(steering, Path81);
        vizSelPath(estimated_path_idx);

        // ROS_INFO("-----------------------------");
        ROS_INFO("angle Match output : %.2f", angleMatch_output * aGain);
        ROS_INFO(" dist Match output : %.2f", distMatch_output * dGain);
        ROS_WARN("   Total output    : %.2f", steering);
    }
    else{
        ROS_INFO("[ == ELSE == ]");
        // Drivable Case Speed & Brake
        brake = 0;
        speed = 80;

        // Tracking Score
        std::vector<double> path_tracking_score(81); // 따로 초기화 해주지 않아도 0으로 초기화가 됨.

        if(gpath2locReady){
            for(int k = 0; k < 81; k++){
                double tx = TrackingPoints[k].first;
                double ty = TrackingPoints[k].second;
                double min_d = 100;
                for(const auto& gp : gpath2loc){
                    double d = dist(gp.first, gp.second, tx, ty);
                    if (d < min_d){
                        min_d = d;
                    }
                }
                path_tracking_score[k] = min_d;
            }
        }
        

        // Path Len Score
        std::vector<double> path_len_score(81);
        for(int k = 0; k < 81; k++){
            path_len_score[k] = pathlen2obs[k];
        }

        // Tracking Score와 Path len Score 모두 Safe Path인 경우에 대해서만 Min - Max 정규화 수행

            // Tracking Score 정규화
        std::vector<double> safe_tracking;
        for (int k = 0; k < 81; k++) {
            if (isPathSafe[k]) safe_tracking.push_back(path_tracking_score[k]);
        }

        double min_val_tracking = *std::min_element(safe_tracking.begin(), safe_tracking.end());
        double max_val_tracking = *std::max_element(safe_tracking.begin(), safe_tracking.end());

        for (int k = 0; k < 81; k++) {
            if (isPathSafe[k]) {
                if (min_val_tracking == max_val_tracking) {
                    path_tracking_score[k] = 0.0;
                } else {
                    path_tracking_score[k] = 1 - (path_tracking_score[k] - min_val_tracking) / (max_val_tracking - min_val_tracking);
                }
            }
        }

            // Path Len Score 정규화
        std::vector<double> safe_plen;
        for (int k = 0; k < 81; k++) {
            if (isPathSafe[k]) safe_plen.push_back(path_len_score[k]);
        }

        double min_val_len = *std::min_element(safe_plen.begin(), safe_plen.end());
        double max_val_len = *std::max_element(safe_plen.begin(), safe_plen.end());

        for (int k = 0; k < 81; k++) {
            if (isPathSafe[k]) {
                if (min_val_len == max_val_len) {
                    path_len_score[k] = 0.0;
                } else {
                    if (bigobs == false){ // small obs
                        path_len_score[k] = (path_len_score[k] - min_val_len) / (max_val_len - min_val_len);
                    }
                    else{ // big obs
                        path_len_score[k] = 1 - (path_len_score[k] - min_val_len) / (max_val_len - min_val_len);
                    }
                }
            }
        }

        // Total Score, 최종 Path 선택 및 Steering Publish//
        std::vector<double> total_score(81);
        double tracking_gain = 0.55;
        double plen_gain = 1 - tracking_gain;
        for(int k = 0; k < 81; k++){
            if (bigobs == false){
                total_score[k] = tracking_gain * path_tracking_score[k] + plen_gain * path_len_score[k];
            }
            else{
                total_score[k] = tracking_gain * path_tracking_score[k] + plen_gain * path_len_score[k];
            }
        }


        double max_score = -100;
        for(int k = 0; k < 81; k++){
            if(isPathSafe[k]){
                if(total_score[k] > max_score){
                    max_score = total_score[k];
                    selIDX = k;
                }
            }
        }
        // --------------------------
        if(selIDX == -1){
            ROS_WARN("selIDX is not Computed");
        }
        else{
            ROS_ERROR("selIDX : %d",selIDX);
        }

        // Visulalization Selected Path
        vizSelPath(selIDX);
        last_selidx = selIDX;
        // Compute Steering
        int pathlen = static_cast<int>(Path81[selIDX].size());
        int halflen = pathlen / 2;
        steering = std::atan2(Path81[selIDX][halflen + Steering_Offset].y, Path81[selIDX][halflen].x);

        steering = - int(2000 * steering / 0.4922); // left is -

        if (steering >= 2000){
            steering = 2000;
        }
        if(steering <= -2000){
            steering = -2000;
        }
    }
    
    ROS_INFO("steering : %.2f",steering);

    // // ERP42 Command Message
    // erp_driver::erpCmdMsg command;
    // command.speed = speed; // static speed..
    // command.brake = 0;
    // command.e_stop = false;
    // command.steer = steering;
    // command.gear = 0; // 0 전진 1 중립 2 후진

    // cmd_pub.publish(command);

    std_msgs::Int32 steerCMD;
    steerCMD.data = steering;

    if (bigobs){
        bigObsSteer_pub.publish(steerCMD);
    }
    else{
        smallObsSteer_pub.publish(steerCMD);
    }

    auto pathSelect_end_time = clockclock::now();
    auto pathSelect_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(pathSelect_end_time - pathSelect_start_time);
    
    ROS_INFO("%ld ms for pathGen loop", pathGen_duration_ms.count());
    ROS_INFO("%ld ms for pathInfo loop", pathInfo_duration_ms.count());
    ROS_INFO("%ld ms for pathSelect loop", pathSelect_duration_ms.count());
}


int main(int argc, char** argv){

    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh_;

    // SUBSCRIBER
    ros::Subscriber odom_gps_Sub = nh_.subscribe("/odom_gps", 3, odom_gps_Callback);
    ros::Subscriber vehicle_yaw_Sub = nh_.subscribe("/vehicle_yaw", 1, vehicle_yaw_Callback);
    ros::Subscriber global_path_Sub = nh_.subscribe("/global_path", 1, global_path_Callback);
    ros::Subscriber local_map_Sub = nh_.subscribe("/local_map", 1, local_map_Callback);

    // PUBLISHER
    vizselpath_pub = nh_.advertise<nav_msgs::Path>("viz_SelPath", 3);
    vizallpath_pub = nh_.advertise<visualization_msgs::MarkerArray>("viz_Path81", 3);
    vizCheckArea_pub = nh_.advertise<visualization_msgs::Marker>("viz_CheckArea", 3);
    vizTM_pub = nh_.advertise<visualization_msgs::Marker>("viz_TrackingMarker", 3);
    vizCM_pub = nh_.advertise<visualization_msgs::Marker>("viz_CrashDistMarker", 3);
    vizOBS_pub = nh_.advertise<visualization_msgs::Marker>("viz_ObsDetectMarker", 3);
    cmd_pub = nh_.advertise<erp_driver::erpCmdMsg>("/erp42_ctrl_cmd", 1);
    // cmd_pub = nh_.advertise<erp_driver::erpCmdMsg>("/obs_erp42cmd", 1);
    bigObsSteer_pub = nh_.advertise<std_msgs::Int32>("/obs_big_steer",3);
    smallObsSteer_pub = nh_.advertise<std_msgs::Int32>("/obs_small_steer",3);

    vizPaths.resize(81);
    ros::Rate rate(50);

    while (ros::ok()){
        auto total_start_time = clockclock::now();

        ros::spinOnce();
        Compute();
        rate.sleep();

        auto total_end_time = clockclock::now();
        auto total_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(total_end_time - total_start_time);
        ROS_INFO("%ld ms for TOTAL loop", total_duration_ms.count());
    }
}