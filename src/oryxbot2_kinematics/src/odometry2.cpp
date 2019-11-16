#include "string"
#include "vector"
#include <functional>
#include <memory>

#include <iostream>
#include <chrono>

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"//odom
#include "geometry_msgs/msg/pose2_d.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

rclcpp::Clock::SharedPtr clock_;
rclcpp::Time current_time,last_time;
rclcpp::Node::SharedPtr node_handle = nullptr;


void vel_callback(const geometry_msgs::msg::Twist::SharedPtr vel)
{
    //get current velocity
    vx = vel->linear.x;
    vy = vel->linear.y;
    vth = vel->angular.z;

    current_time = node_handle->now();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).seconds();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
    last_time = current_time;
	//std::cout << "dt: " << dt << "x : "<< vx << "y : " << vy << std::endl;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
    node_handle = rclcpp::Node::make_shared("odometry");

//    tf2_ros::TransformBroadcaster odom_broadcaster;
    auto odom_pub = node_handle->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    auto pose2d_pub = node_handle->create_publisher<geometry_msgs::msg::Pose2D>("odom_pose2d", 10);
    auto subscription = node_handle->create_subscription<geometry_msgs::msg::Twist>("real_vel", 10, vel_callback); 

    current_time = node_handle->now();
    last_time = node_handle->now();

    rclcpp::WallRate loop_rate(50);

    while(rclcpp::ok()) {

        rclcpp::spin_some(node_handle);

        geometry_msgs::msg::Pose2D pose2d;
        pose2d.x = x;
        pose2d.y = y;
        pose2d.theta = th;
        pose2d_pub->publish(pose2d);

        tf2::Quaternion quaternion;
        // yaw, pitch and roll are rotations in z, y, x respectively
        quaternion.setRPY(0,0,th);

        static tf2_ros::TransformBroadcaster odom_broadcaster_(node_handle);

        //first, we'll publish the transform over tf
        geometry_msgs::msg::TransformStamped odom_trans;
        
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf2::toMsg(quaternion);

        //send the transform
        odom_broadcaster_.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf2::toMsg(quaternion);

        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        //publish the message
        odom_pub->publish(odom);
        loop_rate.sleep();
    }
}
