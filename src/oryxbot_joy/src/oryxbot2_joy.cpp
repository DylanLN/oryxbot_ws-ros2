#include "string"
#include "vector"
#include <functional>
#include <memory>
#include <thread>

#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

class TeleopJoy : public rclcpp::Node
{
public:
    TeleopJoy(rclcpp::NodeOptions options) : Node("oryxbot_joy",options)
    {
        i_velLinear_x=1;
        i_velLinear_y=0;
        i_velAngular=3;
        f_velLinearMax=0.5;
        f_velAngularMAx=2;

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        callback_group_subscriber1_ = this->create_callback_group(
            rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        subscription1_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            rclcpp::QoS(10),
            std::bind(
                &TeleopJoy::callBack,
                this,
                std::placeholders::_1),
        sub1_opt);
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription1_;

    void callBack(const sensor_msgs::msg::Joy::SharedPtr msg);//joy回调函数

    int i_velLinear_x, i_velLinear_y, i_velAngular; //axes number
    double f_velLinearMax, f_velAngularMAx;
};

//回调函数实现在这里
void TeleopJoy::callBack(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    geometry_msgs::msg::Twist vel;
    vel.linear.x = msg->axes[i_velLinear_x]*f_velLinearMax;
    vel.linear.y = msg->axes[i_velLinear_y]*f_velLinearMax;
    vel.angular.z = msg->axes[i_velAngular]*f_velAngularMAx;
    publisher_->publish(vel);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::spin(std::make_shared<TeleopJoy>(rclcpp::NodeOptions()));

    rclcpp::shutdown();
    return 0;
}



