#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <vector>
#include <unordered_set>
#include <utility>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <erp_driver/erpCmdMsg.h>
#include <erp_driver/erpStatusMsg.h>



// GLOBAL VAR

ros::Publisher path_viz;
ros::Publisher allpath_viz;
ros::Publisher cmd_pub;
ros::Publisher car_pub;

ros::Publisher check_area_marker_pub; 

static const float wheelbase = 1.2;
static const float deg_max = 30;

bool gridReady = false;
bool ConePointReady = false;

// Recived from labapath
double speed = 0;
double brake = 0;

nav_msgs::OccupancyGrid grid_;
std::pair<double, double> cone_point;
std::vector<nav_msgs::Path> vizPaths;
std::vector<int> checkarea;

struct pathPoint
{
    double x;
    double y;
    double yaw;
    double cl; // point 까지 누적 길이
};

// Params
int Steering_Offset = 2;
double Crash_distance = 2;
double check_r = 0.6;

// FUNCTIONS

double dist(double x1, double y1, double x2, double y2){
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

nav_msgs::Path makePathMsg(const std::vector<std::pair<double,double>>& cone_path) { // Path visualize
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    for (const auto& pt : cone_path) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path_msg.header;
        pose_stamped.pose.position.x = pt.first;
        pose_stamped.pose.position.y = pt.second;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0; // 회전은 기본값
        path_msg.poses.push_back(pose_stamped);
    }
    return path_msg;
}

void cone_point_Callback(const geometry_msgs::Point::ConstPtr& msg){
    // 메세지 타입은 Path이지만 실제로 path 안에는 하나의 point만 들어 있음.(목표 지점)
    ConePointReady = true;
    cone_point.first = msg->x;
    cone_point.second = msg->y;
}

void fromlaba_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // 수신된 데이터의 크기를 확인하여 2개가 맞는지 검사 (안정성을 위해)
    if (msg->data.size() == 2)
    {
        speed= msg->data[0];
        brake = msg->data[1];
    }
}

void local_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    grid_ = *msg;
    gridReady = true;
}

bool isObstacleNear(double x, double y, double radius){
    if (gridReady){
        double res = grid_.info.resolution;
        int width = grid_.info.width;
        int height = grid_.info.height;

        int gx = (x - grid_.info.origin.position.x) / res;
        int gy = (y - grid_.info.origin.position.y) / res;
        int r = radius / res;

        for (int dx = -r; dx <= r; ++dx){
            for (int dy = -r; dy <= r; ++dy){
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

bool isObstacleNearCheck(double x, double y, double radius){
    if (gridReady){
        double res = grid_.info.resolution;
        int width = grid_.info.width;
        int height = grid_.info.height;

        int gx = (x - grid_.info.origin.position.x) / res;
        int gy = (y - grid_.info.origin.position.y) / res;
        int r = radius / res;

        for (int dx = -r; dx <= r; ++dx){
            for (int dy = -r; dy <= r; ++dy){
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

void CheckingAreaPUB(){
    if (!gridReady) return;

    double res = grid_.info.resolution;
    int width = grid_.info.width;

    // 1. 중복 제거
    std::unordered_set<int> unique_indices(checkarea.begin(), checkarea.end());

    // 2. 마커 초기화
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";  // OccupancyGrid 좌표계에 맞게 설정
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

    // 4. 퍼블리시
    check_area_marker_pub.publish(marker);
}



void vizSelPath(int selIDX){
    nav_msgs::Path selected = vizPaths[selIDX];
    selected.header.stamp = ros::Time::now();
    selected.header.frame_id = "base_link"; // 또는 odom 등 frame에 맞게

    path_viz.publish(selected);
}


void vizAllPaths()
{   // 모든 Tentacle 경로를 Publish 하는 함수
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
        marker.color.a = 1.0;

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

    allpath_viz.publish(marker_array);
}

void carViz()
{   // 차가 차지하는 영역을 Visualize 하는 함수
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "rectangle";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1; // 선 두께

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    geometry_msgs::Point p;

    double car_width = 0.6;
    double car_front = 0.2; //Start at Velodyne..
    double car_rear = -1.8; //Start at Velodyne..
    double h_ = -0.3;       //Visualize height..

    p.x = car_front; p.y = car_width; p.z = h_; marker.points.push_back(p);
    p.x = car_rear; p.y = car_width; p.z = h_; marker.points.push_back(p);
    p.x = car_rear; p.y = -car_width; p.z = h_; marker.points.push_back(p);
    p.x = car_front; p.y = -car_width; p.z = h_; marker.points.push_back(p);
    p.x = car_front; p.y = car_width; p.z = h_; marker.points.push_back(p);

    car_pub.publish(marker);
}

void Compute(){

    double m_dist_adj_pts = 0.1; // be a distance between two adjacent points
    double m_center2rear = 0.5;
    double m_velo_e = 9; // 36
    double m_velo_s = 0.9;
    double m_rho = 1.15;
    double v_j = 3.0; // 속도 관련
    double sizeratio = 0.12;
    double q_tentacle = pow((v_j - m_velo_s)/(m_velo_e - m_velo_s), (1.0 / 1.2));// * sizeratio; // 0.095667?? // q_tentacle : paper eq(5)
    double l_outmost = (8 + 33.5 * pow(q_tentacle, 1.2)) * sizeratio;

    double R_outmost = l_outmost / ((0.6 * M_PI) * (1 - pow(q_tentacle, 0.9)));    // Outmost radius of tentacle : paper eq(2)  4.407623
    double dphi_tentacle[81];              // d(phi) of each tentacle
    double l_tentacle[81];                 // tentacle length : paper eq(3),(4)
    double r_tentacle[81];                 // radius of each tentacle: paper eq(3)

    std::vector<double> pathlen2obs(81);
    std::vector<bool> isPathSafe(81);
    std::fill(isPathSafe.begin(), isPathSafe.end(), true); 
    std::vector<std::vector<pathPoint>> m_TentaclePath(81);

 
    ///////////////////////////
    // local path generation //
    ///////////////////////////
    
    // 경로 생성은 1ms 이내로 완료됨
    r_tentacle[40] = 999999.9;  //infinite, straight_line for tentacle --> wtf...?
    for (int k = 0; k < 41; k++){ // FROM LEFT TO CENTER // 근데 40은 무한대 아닌가? 41->40으로 수정함 3/12

        nav_msgs::Path path;
        path.header.frame_id = "base_link";

        l_tentacle[k] = l_outmost + 20 * sqrt((k + 1)/ 40.0) * sizeratio; // k가 커질수록 l_tentacle[]이 길어짐. floor는 주어진 실수보다 작거나 같은 가장 큰 실수값 반환환
        r_tentacle[k] = pow(m_rho, k) * R_outmost;
        dphi_tentacle[k] = m_dist_adj_pts / abs(r_tentacle[k]); // angle which makes the distance between two adjacent points, 10 cm

        for (int m = 0; m < floor(l_tentacle[k]/m_dist_adj_pts); m++){ // 길이에 따라 for loop 몇번 돌지 결정 floor는 주어진 실수보다 작거나 같은 가장 큰 실수값 반환.

            double p_x = r_tentacle[k] * sin(dphi_tentacle[k] * m) - m_center2rear;
            double p_y = r_tentacle[k] * (1 - cos(dphi_tentacle[k] * m));
            double p_r = dphi_tentacle[k] * m;
            double p_cl = m * 0.1;

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "base_link";
            pose.pose.position.x = p_x;
            pose.pose.position.y = p_y;
            path.poses.push_back(pose);

            m_TentaclePath[k].emplace_back(pathPoint{p_x, p_y, p_r,p_cl});
        }
        vizPaths[k] = path;
    }
    
    for (int k = 41; k < 81; k++){ // FROM RIGHT TO CENTER

        nav_msgs::Path path;
        path.header.frame_id = "base_link";

        l_tentacle[k] = l_outmost + 20 * sqrt((k - 40) / 40.0) * sizeratio;
        r_tentacle[k] = -pow(m_rho, (k - 41)) * R_outmost;
        dphi_tentacle[k] = m_dist_adj_pts / abs(r_tentacle[k]);    // angle which makes the distance between two adjacent point 10 cm

        for (int m = 0; m < floor(l_tentacle[k]/m_dist_adj_pts); m++){

            double p_x = - r_tentacle[k] * sin(dphi_tentacle[k] * m) - m_center2rear;
            double p_y = r_tentacle[k] * (1 - cos(dphi_tentacle[k] * m));
            double p_r = - dphi_tentacle[k] * m;
            double p_cl = m * 0.1;

            //for vizpath
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "base_link";
            pose.pose.position.x = p_x;
            pose.pose.position.y = p_y;
            path.poses.push_back(pose);

            m_TentaclePath[k].emplace_back(pathPoint{p_x, p_y, p_r, p_cl});
        }
        vizPaths[k] = path;
    }
    

    ///////////////////
    // Get Path Info //
    ///////////////////

    ROS_INFO_THROTTLE(0.2, "------------------------------");


    checkarea.clear();
    for (int k = 0; k < 81; k++) {
        bool safeFlag = true;

        for (const auto& path_point : m_TentaclePath[k]){
            double p_x_rear = path_point.x;
            double p_y_rear = path_point.y;
            double pyaw = path_point.yaw;
            double c_l = path_point.cl;

            double p_x_front = p_x_rear + 2 * m_center2rear * cos(pyaw); //2 * center2rear == wheelbase
            double p_y_front = p_y_rear + 2 * m_center2rear * sin(pyaw);
            
            bool front_check = false;
            bool rear_check = false;
            
            if (gridReady == true){

                // base link와 velodyne 기준 좌표계는 x축 기준 0.82 차이남. 좌표계 매칭.
                front_check = isObstacleNear(p_x_front - 0.82, p_y_front, check_r); 
                rear_check  = isObstacleNear(p_x_rear - 0.82, p_y_rear, check_r);

                // frontarea & reararea is Visualization of orange points 
                // it may use quite lots of computing resources(drop loop Hz)
                // so if you don't need to use these points, comment out the following two lines.
                bool frontarea = isObstacleNearCheck(p_x_front - 0.82, p_y_front, check_r);
                bool reararea = isObstacleNearCheck(p_x_rear - 0.82, p_y_rear, check_r);

                // Set Crash Distance
                if (c_l > Crash_distance){ 
                    if(front_check == true || rear_check == true){
                        // 장애물이 있어도 1.5m이내는 safePath라고 설정.
                        // Pathlen2obs는 장애물 탐지와 마찬가지로 설정.
                        isPathSafe[k] = true;
                        pathlen2obs[k] = c_l;
                        safeFlag = false;
                        break;
                    }
                }
                else{
                    if(front_check == true || rear_check == true){
                        isPathSafe[k] = false;
                        pathlen2obs[k] = c_l;
                        safeFlag = false;
                        break;
                    }
                }

            }
            else{
                ROS_INFO_THROTTLE(0.2, "OccGrid does not received YET!");
            }
        }
        if (safeFlag == true){
            pathlen2obs[k] = m_TentaclePath[k].back().cl; // safepath이면 마지막 누적거리가 장애물까지의 거리
        }
    }
    CheckingAreaPUB(); // Orange Marker





    /////////////////////////////
    // Path choosing Algorithm //
    /////////////////////////////

    bool all_safe = std::all_of(isPathSafe.begin(), isPathSafe.end(), [](bool v) { return v; });   // vector가 모두 true인지
    bool all_danger = std::none_of(isPathSafe.begin(), isPathSafe.end(), [](bool v) { return v; }); // vector가 모두 false인지

    int selIDX = -1;
    double steering = 0;

    // Speed received for labapath
    if(all_danger){
        // Not Drivable Case Speed & Brake
        brake = 20;
        speed = 0;
        ROS_INFO_THROTTLE(0.2, "ALL PATH is DANGER");
    }
    else{
        // Drivable Case Speed & Brake
        // Received from labapath

        
        // Tracking Score
        std::vector<double> path_tracking_score(81); // 따로 초기화 해주지 않아도 0으로 초기화가 됨.
        double tracking_th = std::atan2(cone_point.second, cone_point.first + 0.82);

        if(ConePointReady){ // Cone point가 sub되면
            for(int k = 0; k < 81; k++){
                int pathlen = static_cast<int>(m_TentaclePath[k].size());
                int halflen = pathlen / 2;
                double path_steer = std::atan2(m_TentaclePath[k][pathlen-1].y, m_TentaclePath[k][pathlen-1].x);
                double diff = std::abs(tracking_th - path_steer);
                path_tracking_score[k] = diff;
            }
        }
        else{
            ROS_INFO_THROTTLE(0.2, "Cone point is not received");
        }

        // Path Len Score
        std::vector<double> path_len_score(81);
        for(int k = 0; k < 81; k++){
            path_len_score[k] = m_TentaclePath[k].back().cl;
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
                    ROS_INFO_THROTTLE(0.2, "something wrong");
                    break;
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
                    ROS_INFO_THROTTLE(0.2, "something wrong2");
                } else {
                    path_len_score[k] = (path_len_score[k] - min_val_len) / (max_val_len - min_val_len);
                }
            }
        }

        // Total Score, 최종 Path 선택 및 Steering Publish//
        std::vector<double> total_score(81);
        double tracking_gain = 0.8;
        double plen_gain = 1 - tracking_gain; // 0.35
        for(int k = 0; k < 81; k++){
            total_score[k] = tracking_gain * path_tracking_score[k] + plen_gain * path_len_score[k];
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
        // TEST // ----------------------------
        // int pl = -1;
        // double max_score2 = -100;
        // for(int k = 0; k < 81; k++){
        //     if(isPathSafe[k]){
        //         if(path_len_score[k] > max_score2){
        //             max_score2 = path_len_score[k];
        //             pl = k;
        //         }
        //     }
        // }
        // if(pl == -1){
        //     ROS_INFO_THROTTLE(0.2, "wrong len");
        // }
        // else{
        //     ROS_INFO_THROTTLE(0.2,"path len sel : %d",pl);
        // }

        // int pt = -1;
        // double max_score3 = -100;
        // for(int k = 0; k < 81; k++){
        //     if(isPathSafe[k]){
        //         if(path_tracking_score[k] > max_score3){
        //             max_score3 = path_tracking_score[k];
        //             pt = k;
        //         }
        //     }
        // }

        // if(pt == -1){
        //     ROS_INFO_THROTTLE(0.2, "wrong track");
        // }
        // else{
        //     ROS_INFO_THROTTLE(0.2,"path track sel : %d",pt);
        // }



        // --------------------------
        if(selIDX == -1){
            ROS_INFO_THROTTLE(0.2, "selIDX is not Computed");
        }
        else{
            ROS_INFO_THROTTLE(0.2, "selIDX : %d",selIDX);
        }

        // Visulalization Selected Path
        vizSelPath(selIDX);

        // Compute Steering
        int pathlen = static_cast<int>(m_TentaclePath[selIDX].size());
        int halflen = pathlen / 2;
        steering = std::atan2(m_TentaclePath[selIDX][halflen + Steering_Offset].y, m_TentaclePath[selIDX][halflen].x);

        steering = - int(2000 * steering / 0.4922); // left is -

        if (steering >= 2000){
            steering = 2000;
        }
        if(steering <= -2000){
            steering = -2000;
        }
    }

    // ERP42 Command Message
    ROS_INFO_THROTTLE(0.2,"steering : %.2f", steering);
    erp_driver::erpCmdMsg command;
    command.speed = static_cast<int>(speed);
    command.brake = static_cast<int>(brake);
    command.e_stop = false;
    command.steer = steering;
    command.gear = 0; // 0 전진 1 중립 2 후진
    cmd_pub.publish(command);

    //CAR AREA Visualize
    carViz();
}


int main(int argc, char** argv){
    
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh_;

    // SUBSCRIBER
    ros::Subscriber cone_point_Sub = nh_.subscribe("/final_point", 10, cone_point_Callback);
    ros::Subscriber cone_cmd_Sub = nh_.subscribe("/target_cmd", 10, fromlaba_Callback);
    ros::Subscriber local_map_Sub = nh_.subscribe("/local_map", 10, local_map_Callback);


    // READ PARAMS
    // nh_.getParam("Steering_Offset",Steering_Offset);
    // nh_.getParam("Crash_distance",Crash_distance);
    // nh_.getParam("check_r",check_r);
    ROS_INFO("-----------------------");
    ROS_INFO("Parameters Set complete");
    ROS_INFO("Steering offset : %d",Steering_Offset);
    ROS_INFO("Crash_distance : %f",Crash_distance);
    ROS_INFO("check_r : %f", check_r);
    

    // PUBLISHER
    // - visualize
    path_viz = nh_.advertise<nav_msgs::Path>("path_viz", 1); // selected path
    check_area_marker_pub = nh_.advertise<visualization_msgs::Marker>("check_area_marker", 1); //check area(orange marker)
    car_pub = nh_.advertise<visualization_msgs::Marker>("/car_area", 1); // car area
    allpath_viz = nh_.advertise<visualization_msgs::MarkerArray>("allpath_viz", 1); // not using now..
    // - command (erp42)
    cmd_pub = nh_.advertise<erp_driver::erpCmdMsg>("/erp42_ctrl_cmd", 1);

    vizPaths.resize(81);

    ros::Rate rate(30);
    while (ros::ok()){
        ros::spinOnce();
        Compute();
        rate.sleep();
    }

}