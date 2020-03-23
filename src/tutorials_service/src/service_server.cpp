#include <iostream>

#include <memory>
#include"oryxbot2_msgs/srv/srv_tutorial.hpp"

#include "rclcpp/rclcpp.hpp"

rclcpp::Node::SharedPtr node_handle = nullptr;


void servicecallback(const std::shared_ptr<oryxbot2_msgs::srv::SrvTutorial::Request> request,
  const std::shared_ptr<oryxbot2_msgs::srv::SrvTutorial::Response> response
)
{
  RCLCPP_INFO(node_handle->get_logger(),"request : x =%d , y =%d  response sum= %d",request->a,request->b,request->a+request->b);
  response->result = request->a + request->b;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node_handle = rclcpp::Node::make_shared("service_server");
  auto server = node_handle->create_service<oryxbot2_msgs::srv::SrvTutorial>("add_two_ints", servicecallback);
  rclcpp::spin(node_handle);
  rclcpp::shutdown();
  node_handle = nullptr;
  return 0;
}