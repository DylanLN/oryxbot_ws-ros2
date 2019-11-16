#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "oryxbot2_kinematics/oryxbot2_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor executor;
	rclcpp::NodeOptions options;

	//rclcpp::executors::MultiThreadedExecutor executor;
	auto datatoreal = std::make_shared<OryxBotKinematics>(options);
	auto veltocmd = std::make_shared<Datatorealvel>(options);  

	executor.add_node(datatoreal);
	executor.add_node(veltocmd);
	executor.spin();
	rclcpp::shutdown();
    return 0;
}
