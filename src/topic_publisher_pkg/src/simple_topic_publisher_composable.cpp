#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>

using namespace std::chrono_literals;
/* This example creates a subclass of Node and uses std::bind() to register a* member function as a callback from the timer.*/
class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher()
    : Node("simple_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter",10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SimplePublisher::timer_callback, this));
    }
private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Int32();
        message.data = count_;
        count_++;
        publisher_->publish(message);
    }
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
	size_t count_;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}