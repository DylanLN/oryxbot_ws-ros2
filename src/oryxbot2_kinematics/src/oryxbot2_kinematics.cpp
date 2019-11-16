#include "oryxbot2_kinematics/oryxbot2_kinematics.hpp"

#define KINEMATICS_DEBUG false
#define INVERS_KINEMATICS_DEBUG false
#include "cmath"

using namespace std::chrono_literals;

template<typename T>
std::ostream& operator<< (std::ostream& os, std::vector<T> vec)
{
    os << '[';
    for(int i=0; i<vec.size(); i++) {
        os << vec[i] <<" ";
   }
    return os << ']';
}

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

std::vector<double> OryxBotKinematics::inverse_kinematics(double vx, double vy, double vth)
{
    std::vector<double> motor_speed(m_kinematics_mode, 0);
    switch(m_kinematics_mode) {
    case 2:
        motor_speed[0] = -(vx - vth*m_wheel_separation/2)/m_wheel_radius*30/PI;
        break;
    case 3:
        motor_speed[0] = (vx*sin(PI/3) + vy*cos(PI/3) + m_wheel_separation*vth)/m_wheel_radius*30/PI;
        motor_speed[1] = -(vx*sin(PI/3) - vy*cos(PI/3) - m_wheel_separation*vth)/m_wheel_radius*30/PI;
        motor_speed[2] = (-vy + m_wheel_separation*vth)/m_wheel_radius*30/PI;
        break;
     case 4:
        //ROS_INFO("4 wheel omni-directional, inverse_kinematics");
        motor_speed[0] = (vx + vy + 0.5*vth*(m_width + m_length))/m_wheel_radius*30/PI;
        motor_speed[1] = -(vx - vy - 0.5*vth*(m_width + m_length))/m_wheel_radius*30/PI;
        motor_speed[2] = -(vx + vy - 0.5*vth*(m_width + m_length))/m_wheel_radius*30/PI;
        motor_speed[3] = (vx - vy + 0.5*vth*(m_width + m_length))/m_wheel_radius*30/PI;        
        break;
    default:
    	std::cout << "cannot support the kinematics " << std::endl;
        //ROS_INFO("cannot support the kinematics");
    }
#if INVERS_KINEMATICS_DEBUG
    std::cout << "kinematics motor speed: " << motor_speed << std::endl;
#endif
    return motor_speed;
}

//motorspeed (r/min)
std::vector<double> Datatorealvel::kinematics(std::vector<double> motorspeed)
{
    std::vector<double> velocity(3, 0);	// vx, vy, vth
    switch(m_kinematics_mode) {
    case 2:
        velocity[0] = (-motorspeed[0] + motorspeed[1])*PI*m_wheel_radius/60.0; //m/s
        velocity[1] = 0;
        velocity[2] = (motorspeed[0] + motorspeed[1])*2*PI*m_wheel_radius/m_wheel_separation/60.0; //rad/s
        break;
    case 3:
        velocity[0] = (motorspeed[0] - motorspeed[1])*PI/30.f*m_wheel_radius/sqrt(3);
        velocity[1] = (motorspeed[0] + motorspeed[1] - 2*motorspeed[2])*PI*m_wheel_radius/30.f
                        /3.f;
        velocity[2] = (motorspeed[0] + motorspeed[1] + motorspeed[2])*PI*m_wheel_radius/30.f
                        /3.f/m_wheel_separation;
        break;
     case 4:
        //ROS_INFO("4 wheels omni-directional, kinematics");
        velocity[0] = (motorspeed[0] - motorspeed[1])*m_wheel_radius/2*PI/30.f;
        velocity[1] = (motorspeed[0] - motorspeed[3])*m_wheel_radius/2*PI/30.f;
        velocity[2] = (motorspeed[1] + motorspeed[3])*m_wheel_radius/(m_width+m_length)*PI/30.f;

        break;
     default:
        ;
    }
#if KINEMATICS_DEBUG
    std::cout << "kinematics: "<< velocity << std::endl;
#endif
    return  velocity;
}

void OryxBotKinematics::vel_callback(const geometry_msgs::msg::Twist::SharedPtr vel)
{
    double vx = 0, vy = 0, vth = 0;
    if (fabs(vel->linear.x) > m_max_vx) {
        vx = fabs(vel->linear.x)/vel->linear.x*m_max_vx;
    } else {
        vx = vel->linear.x;
    }

    if (fabs(vel->linear.y) > m_max_vy) {
        vy = fabs(vel->linear.y)/vel->linear.y*m_max_vy;
    } else {
        vy = vel->linear.y;
    }

    if (fabs(vel->angular.z) > m_max_vth) {
        vth = fabs(vel->angular.z)/vel->angular.z*m_max_vth;
    } else {
        vth = vel->angular.z;
    }
#if INVERS_KINEMATICS_DEBUG
    std::cout <<"received velocity [vx vy vth] "
              << '[' << vx << " " << vy <<" " << vth << ']'
              << std::endl;
#endif
    m_motor_speed = inverse_kinematics(vx, vy, vth);
	oryxbot2_msgs::msg::Carcmd  msg;
        msg.speed = m_motor_speed;
        m_motor_speed_pub_->publish(msg);
}

void Datatorealvel::status_callback(const oryxbot2_msgs::msg::Cardata::SharedPtr status)
{
    std::vector<double> motorspeed= status->speed;
    m_real_vel_ = kinematics(motorspeed);
    //std::cout << "111" << m_real_vel_[0] <<std::endl;
    geometry_msgs::msg::Twist msg;
    msg.linear.x = m_real_vel_[0];
    msg.linear.y = m_real_vel_[1];
    msg.angular.z = m_real_vel_[2];
    m_vel_pub->publish(msg);

}


OryxBotKinematics::OryxBotKinematics(rclcpp::NodeOptions options)
: Node("kinematics_node",options)
{
    //参数初始化
    m_kinematics_mode = default_kinematics_mode;
    m_wheel_radius = default_wheel_radius;
    m_wheel_separation = default_wheel_separation;
    m_width = default_width;
    m_length = default_length;
    m_max_vx = default_vx;
    m_max_vy = default_vy;
    m_max_vth = default_vth;

    //以下是cmd_vel 和car_cmd
	callback_group_subscriber1_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);	
	auto sub1_opt = rclcpp::SubscriptionOptions();
	sub1_opt.callback_group = callback_group_subscriber1_;

	m_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",rclcpp::QoS(10),
		std::bind(
        	&OryxBotKinematics::vel_callback,  // First parameter is a reference to the function
        	this,                               // What the function should be bound to
        	std::placeholders::_1),
		sub1_opt); 

    m_motor_speed_pub_ = this->create_publisher<oryxbot2_msgs::msg::Carcmd>("car_cmd",10);

	m_motor_speed = std::vector<double>(m_kinematics_mode, 0);
    std::cout << " cmd_vel to car_cmd is ok  " << std::endl;
}

Datatorealvel::Datatorealvel(rclcpp::NodeOptions options)
: Node("oryxbot_kinematics_node",options)
{
    //参数初始化    
    m_kinematics_mode = default_kinematics_mode;
    m_wheel_radius = default_wheel_radius;
    m_wheel_separation = default_wheel_separation;
    m_width = default_width;
    m_length = default_length;
    m_max_vx = default_vx;
    m_max_vy = default_vy;
    m_max_vth = default_vth;

    //以下是car_data和odom
	callback_group_subscriber2_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);	
	auto sub2_opt = rclcpp::SubscriptionOptions();
	sub2_opt.callback_group = callback_group_subscriber2_;

	status_sub_ = this->create_subscription<oryxbot2_msgs::msg::Cardata>("car_data",rclcpp::QoS(10),
		std::bind(
        	&Datatorealvel::status_callback,  // First parameter is a reference to the function
        	this,                               // What the function should be bound to
        	std::placeholders::_1),
		sub2_opt); 

    m_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("real_vel",10);

    m_real_vel_ = std::vector<double>(m_kinematics_mode, 0);
    std::cout << " car_data to real_vel is ok  " << std::endl;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(OryxBotKinematics)


