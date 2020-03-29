#include <memory>

#include "oryxbot2_msgs/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using Fibonacci = oryxbot2_msgs::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

//定义节点句柄，全局变量。
rclcpp::Node::SharedPtr node_handle = nullptr;

//接收action激活时返回
void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_handle->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(node_handle->get_logger(), "Goal accepted by server, waiting for result");
  }
}

//接收feedback后调用的回调函数
void feedback_callback(GoalHandleFibonacci::SharedPtr,
                       const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
  std::stringstream ss;
  ss << "Next number in sequence received: ";
  for (auto number : feedback->partial_sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(node_handle->get_logger(), ss.str().c_str());
}

//当action完成后会调用该回调函数
void result_callback(const GoalHandleFibonacci::WrappedResult & result)
{
  //判断action返回是否成功
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_handle->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_handle->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(node_handle->get_logger(), "Unknown result code");
      return;
  }
  std::stringstream ss;
  ss << "Result received: ";
  for (auto number : result.result->sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(node_handle->get_logger(), ss.str().c_str());
  //关闭节点
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //初始化节点
  node_handle = rclcpp::Node::make_shared("fibonacci_action_client");
  //声明动作客户端
  auto client = rclcpp_action::create_client<Fibonacci>(
    node_handle->get_node_base_interface(),
    node_handle->get_node_graph_interface(),
    node_handle->get_node_logging_interface(),
    node_handle->get_node_waitables_interface(),
    "fibonacci");

  //等待连接action服务器
  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_handle->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  //声明动作目标幷指定进行10次斐波那契运算
  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 10;

  RCLCPP_INFO(node_handle->get_logger(), "Sending goal");
  //要求服务器打成某个目标
  auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
  //声明回调函数
  send_goal_options.goal_response_callback =goal_response_callback;
  send_goal_options.feedback_callback =feedback_callback;
  send_goal_options.result_callback =result_callback;
  //发送目标
  client->async_send_goal(goal_msg, send_goal_options);
  //等待返回
  rclcpp::spin(node_handle);

  rclcpp::shutdown();
  node_handle = nullptr;
  return 0;
}