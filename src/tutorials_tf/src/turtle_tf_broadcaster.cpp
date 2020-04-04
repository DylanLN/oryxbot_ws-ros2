#include "turtlesim/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

std::string turtle_name;
//节点句柄
rclcpp::Node::SharedPtr node_handle = nullptr;

void pose_callback(const turtlesim::msg::Pose::SharedPtr pose)
{
    //tf广播器
    static tf2_ros::TransformBroadcaster pose_broadcaster_(node_handle);
    //广播器消息类型实例化
    geometry_msgs::msg::TransformStamped pose_tf;
    //根据海龟当前位姿，发布对世界坐标系的变换
    pose_tf.header.stamp = node_handle->now();
    pose_tf.header.frame_id = "world";
    pose_tf.child_frame_id = turtle_name;
    pose_tf.transform.translation.x = pose->x;
    pose_tf.transform.translation.y = pose->y;
    pose_tf.transform.translation.z = 0.0;
    //yaw转四元数
    tf2::Quaternion quaternion;
    // yaw, pitch and roll are rotations in z, y, x respectively
    quaternion.setRPY(0,0,pose->theta);
    pose_tf.transform.rotation = tf2::toMsg(quaternion);
    //发布坐标变换
    pose_broadcaster_.sendTransform(pose_tf);
}


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
    //初始化节点
    node_handle = rclcpp::Node::make_shared("turtle_tf_broadcaster");
    //获得小海龟的名字
    if (argc != 2)
    {
        RCLCPP_ERROR(node_handle->get_logger(), "error Exiting %d",argc);
        return -1;
    };
    turtle_name = argv[1];
    //接收小海龟的位姿信息
    auto subscription = node_handle->create_subscription<turtlesim::msg::Pose>(turtle_name+"/pose", 10, pose_callback); 
    //开始接收topic
    rclcpp::spin(node_handle);
    rclcpp::shutdown();
    return 0;
}