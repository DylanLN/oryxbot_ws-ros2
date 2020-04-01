#include "turtlesim/srv/spawn.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"

#define _USE_MATH_DEFINES
//tf监听器类（tf2_echo）
class echoListener
{
public:
    tf2_ros::Buffer buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    //constructor with name
    echoListener(rclcpp::Clock::SharedPtr clock) :
    buffer_(clock)
    {
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    };

    ~echoListener(){};
private:
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node_handle = rclcpp::Node::make_shared("turtle_tf_listener");
    //创建“add_two_ints”客户端
    auto client = node_handle->create_client<turtlesim::srv::Spawn>("spawn");
    //等待连接服务器
    while (!client->wait_for_service(std::chrono::seconds(1))) {
    //关闭节点
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_handle->get_logger(), "Interrupted while waiting for the service. Exiting");
        return 0;
    }
        RCLCPP_INFO(node_handle->get_logger(), "service not available, waiting again...");
    }

    //实例化消息
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    //生成小乌龟
    auto result_future = client->async_send_request(request);
    //乌龟速度发布者
    auto vel_pub = node_handle->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
    //延时 10HZ
    rclcpp::WallRate loop_rate(10.0);
    //时间
    rclcpp::Clock::SharedPtr clock = node_handle->get_clock();
    //实例化监听器echoListener
    echoListener echoListener(clock);
    //监听"turtle2"->"turtle1"
    echoListener.buffer_.canTransform("turtle2", "turtle1", tf2::TimePoint(), tf2::durationFromSec(1.0));
    //开始循环
    while(rclcpp::ok()) {
        //监听两只小海龟的坐标变换
        geometry_msgs::msg::TransformStamped echo_transform;
        echo_transform = echoListener.buffer_.lookupTransform("turtle2", "turtle1", tf2::TimePoint());
        auto translation = echo_transform.transform.translation;
        //根据两只熊海龟之间的坐标关系计算出龟2的线速度与角速度，幷将其发布出去。
        geometry_msgs::msg::Twist vel;
        vel.angular.z = 4.0 * atan2(translation.y,
                                        translation.x);
        vel.linear.x = 0.5 * sqrt(pow(translation.x, 2) +
                                      pow(translation.y, 2));
        vel_pub->publish(vel);
        //延时
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
