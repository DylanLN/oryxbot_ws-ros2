#ifndef _ORYXBOT2_KINEMATICS_HPP_
#define _ORYXBOT2_KINEMATICS_HPP_

#include "string"
#include "vector"
#include <functional>
#include <memory>
#include <thread>

#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "oryxbot2_msgs/msg/carcmd.hpp"
#include "oryxbot2_msgs/msg/cardata.hpp"

#include "oryxbot2_kinematics/visibility.h"

#include "boost/bind.hpp"
#include "boost/thread/thread.hpp"
using namespace std::chrono_literals;

using namespace std;

const double PI = std::acos(-1);

#define default_kinematics_mode (4)

#define default_wheel_radius (0.038)
#define default_wheel_separation (0.5)
#define default_vx (0.5)
#define default_vy (0.5)
#define default_vth (1.0)

#define default_width (0.265)
#define default_length (0.245)

using namespace std::chrono_literals;

class Datatorealvel : public rclcpp::Node
{
public:
	MINIMAL_COMPOSITION_PUBLIC Datatorealvel(rclcpp::NodeOptions options);
	
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_vel_pub;//rel_vel
	rclcpp::Subscription<oryxbot2_msgs::msg::Cardata>::SharedPtr status_sub_;//car_data
	rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber2_;
    std::vector<double> m_real_vel_;	//kinematics

	double m_wheel_radius;
	double m_max_vx;
	double m_max_vy;
	double m_max_vth;
	int m_kinematics_mode;
	//just for 2/3 wheels
	double m_wheel_separation;    

	//just for four wheels omni-directional
	double m_width;
	double m_length;
    
	std::vector<double> kinematics(std::vector<double> motorspeed);//正解
    void status_callback(const oryxbot2_msgs::msg::Cardata::SharedPtr status);
};

class OryxBotKinematics : public rclcpp::Node
{
public:
	MINIMAL_COMPOSITION_PUBLIC OryxBotKinematics(rclcpp::NodeOptions options);

	rclcpp::Publisher<oryxbot2_msgs::msg::Carcmd>::SharedPtr m_motor_speed_pub_;//car_cmd
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_vel_sub_;//cmd_vel
	rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber1_;
	std::vector<double> m_motor_speed;	//inverse_kinematics

	double m_wheel_radius;
	double m_max_vx;
	double m_max_vy;
	double m_max_vth;
	int m_kinematics_mode;
	//just for 2/3 wheels
	double m_wheel_separation;    

	//just for four wheels omni-directional
	double m_width;
	double m_length;

	std::vector<double> inverse_kinematics(double vx, double vy, double vth);//逆解
	void vel_callback(const geometry_msgs::msg::Twist::SharedPtr vel);
};

#endif
