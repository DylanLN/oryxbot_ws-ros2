#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
rclcpp::Node::SharedPtr g_node = nullptr;
void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(g_node->get_logger(), "I heard: '%d'", msg->data);
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("simple_subscriber");
    auto subscription = g_node->create_subscription<std_msgs::msg::Int32>("counter",10, topic_callback);
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    subscription = nullptr;
    g_node = nullptr;
    return 0;
}