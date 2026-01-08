#include <stdio.h>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


double yaw = 0.0;
double x = 0.0, y = 0.0;
double viewer_x = 0.0, viewer_y = 0.0;


ros::Publisher vizcar_pub;

void vizCar()
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

    vizcar_pub.publish(marker);
}


void yawCallback(const std_msgs::Float32::ConstPtr& msg)
{
    yaw = msg->data;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped t;

    t.header.stamp = ros::Time::now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link_gps";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 1.7; // GPS 높이

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    br.sendTransform(t);

    static tf2_ros::TransformBroadcaster br2;
    geometry_msgs::TransformStamped t2;

    t2.header.stamp = ros::Time::now();
    t2.header.frame_id = "map";
    t2.child_frame_id = "viewer";

    t2.transform.translation.x = viewer_x;
    t2.transform.translation.y = viewer_y;
    t2.transform.translation.z = 0.0;

    // printf("%.2f , %.2f\n",(msg->poses[nearest_idx].pose.position.x / 20 ) * 20, (msg->poses[nearest_idx].pose.position.y / 20 ) * 20);

    tf2::Quaternion q2;
    q2.setRPY(0, 0, 0);
    t2.transform.rotation.x = q2.x();
    t2.transform.rotation.y = q2.y();
    t2.transform.rotation.z = q2.z();
    t2.transform.rotation.w = q2.w();

    br2.sendTransform(t2);
    vizCar();
}

void utmCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(msg->poses.size() <= 0){
        return;
    }

    viewer_x = msg->poses[0].pose.position.x;
    viewer_y = msg->poses[0].pose.position.y;


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tfmaker");
    ros::NodeHandle nh;

    ros::Subscriber yaw_sub = nh.subscribe("/vehicle_yaw", 1, yawCallback);
    ros::Subscriber utm_sub = nh.subscribe("/odom_gps", 1, utmCallback);
    ros::Subscriber path_sub = nh.subscribe("/global_path", 1, pathCallback);

    vizcar_pub = nh.advertise<visualization_msgs::Marker>("/viz_CarArea", 1); // car area

    ROS_INFO("Map --> Base_link TF Publish Started");

    ros::spin();
    return 0;
}