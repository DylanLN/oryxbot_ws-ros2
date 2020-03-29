#include "rclcpp/rclcpp.hpp"
#include"oryxbot2_msgs/srv/srv_tutorial.hpp"


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  //初始化节点
  auto node = rclcpp::Node::make_shared("service_client");
  //创建“add_two_ints”客户端
  auto client = node->create_client<oryxbot2_msgs::srv::SrvTutorial>("add_two_ints");
  //等待连接服务器
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    //关闭节点
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }
  //实例化消息
  auto request = std::make_shared<oryxbot2_msgs::srv::SrvTutorial::Request>();
  //赋值
  request->a = 1;
  request->b = 1;
  //延时函数  10Hz
  rclcpp::WallRate loop_rate(10);
  //熟悉的while循环
  while (rclcpp::ok()) {
    //发送消息
    auto result_future = client->async_send_request(request);
    //如果接收到返回
    if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      //接收返回值
      auto result = result_future.get();
      RCLCPP_INFO(node->get_logger(), "Client request sum:%d  =  a: %d  +  b: %d", result->result,request->a, request->b);
      //request->a=request->b;
      request->b +=1;
    }else{//否则
      RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    }
    //延时等待
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
