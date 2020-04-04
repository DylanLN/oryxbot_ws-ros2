//2020.02.29
//rplidar-A1

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.h"
#include "rplidar.h"
#include <chrono>
#include <memory>
#include <sstream>
#include <vector>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif
#define M_PI 3.1415926
#define DEG2RAD(x) ((x)*M_PI/180.)

int output_angle_min = 0;
int output_angle_max = 0;
rclcpp::Node::SharedPtr node_handle = nullptr;

using namespace rp::standalone::rplidar;

RPlidarDriver * drv = NULL;

// class rplidarPublisher : public rclcpp::Node
// {
// public:
//     rplidarPublisher()
//     : Node("rplidar_node"), count_(0)
//     {
//         publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan",1000);
//         RCLCPP_INFO(node_handle->get_logger(),this->get_logger(), "RPLIDAR running on ROS package rplidar_ros. SDK Version:"RPLIDAR_SDK_VERSION"");
//         u_result     op_result;
//     }
// private:
//     void timer_callback()
//     {
//         auto message = std_msgs::msg::Int32();
//         message.data = count_;
//         count_++;
//         publisher_->publish(message);
//     }
// 	rclcpp::TimerBase::SharedPtr timer_;
// 	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
// 	size_t count_;

// public:
//     std::string serial_port;
//     int serial_baudrate = 115200;
//     std::string frame_id;
//     bool inverted = false;
//     bool angle_compensate = true;
//     float max_distance = 8.0;
//     int angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
//     std::string scan_mode;

// };

rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;//rel_vel
void publish_scan(rplidar_response_measurement_node_hq_t *nodes,
                  size_t node_count, rclcpp::Time start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance,
                  std::string frame_id)
{
    static int scan_count = 0;
    sensor_msgs::msg::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_count++;
    scan_msg.angle_min = DEG2RAD(output_angle_min);
    scan_msg.angle_max = DEG2RAD(output_angle_max);
    bool reversed = (angle_max > angle_min);
    scan_msg.angle_increment = DEG2RAD(1);
    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count-1);
    scan_msg.range_min = 0.15;
    scan_msg.range_max = max_distance;//8.0;

    std::vector<float> ranges, intensities;
    ranges.resize(node_count);
    intensities.resize(node_count);
    if(output_angle_max < 0 || output_angle_max > 180)RCLCPP_ERROR(node_handle->get_logger(),"output_angle_max invalid");
    if(output_angle_min < -180 || output_angle_min > 0)RCLCPP_ERROR(node_handle->get_logger(),"out_angle_min invalid");

    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);

    for (size_t i = 0; i < node_count; i++) {
        float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
        if (read_value == 0.0)
            ranges[i] = std::numeric_limits<float>::infinity();
        else
            ranges[i] = read_value;
        intensities[i] = (float) (nodes[i].quality >> 2);
    }

    for(int i=270-output_angle_max; i<270; i++) {
        scan_msg.ranges.push_back(ranges[i]);
        scan_msg.intensities.push_back(intensities[i]);            
    }	
    RCLCPP_DEBUG(node_handle->get_logger(),"2 scan_msg.ranges.size(): %d", scan_msg.ranges.size());

    if(output_angle_min > -90) {
	//270~359
        for(int i=270; i<270-output_angle_min; i++) {
            scan_msg.ranges.push_back(ranges[i]);
            scan_msg.intensities.push_back(intensities[i]);
        }
     	
    } else {
        for(int i=270; i<359; i++) {
            scan_msg.ranges.push_back(ranges[i]);
            scan_msg.intensities.push_back(intensities[i]);
        }
        
        for(int i=0; i<=270-output_angle_min-360; i++) {
            scan_msg.ranges.push_back(ranges[i]);
            scan_msg.intensities.push_back(intensities[i]);
        }
    }
    RCLCPP_DEBUG(node_handle->get_logger(),"1 scan_msg.ranges.size(): %d", scan_msg.ranges.size());

    if(reverse_data) {
        std::reverse(scan_msg.ranges.begin(), scan_msg.ranges.end());
        std::reverse(scan_msg.intensities.begin(), scan_msg.intensities.end());
    }
    scan_pub->publish(scan_msg);
}


bool getRPLIDARDeviceInfo(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            RCLCPP_ERROR(node_handle->get_logger(),"Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
        } else {
            RCLCPP_ERROR(node_handle->get_logger(),"Error, unexpected error, code: %x",op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n");
    RCLCPP_INFO(node_handle->get_logger(),"Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    RCLCPP_INFO(node_handle->get_logger(),"Hardware Rev: %d",(int)devinfo.hardware_version);
    return true;
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) {
        RCLCPP_INFO(node_handle->get_logger(),"RPLidar health status : %d", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            RCLCPP_ERROR(node_handle->get_logger(),"Error, rplidar internal error detected. Please reboot the device to retry.");
            return false;
        } else {
            return true;
        }

    } else {
        RCLCPP_ERROR(node_handle->get_logger(),"Error, cannot retrieve rplidar health code: %x", op_result);
        return false;
    }
}

static float getAngle(const rplidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
	node_handle = rclcpp::Node::make_shared("rplidar_node");

    std::string serial_port;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;
    float max_distance = 8.0;
    int angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
    std::string scan_mode;

    node_handle->declare_parameter("serial_port");
    node_handle->declare_parameter("serial_baudrate");
    node_handle->declare_parameter("frame_id");
    node_handle->declare_parameter("inverted");
    node_handle->declare_parameter("angle_compensate");
    node_handle->declare_parameter("output_angle_min");
    node_handle->declare_parameter("output_angle_max");

    node_handle->get_parameter_or<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    node_handle->get_parameter_or<int>("serial_baudrate", serial_baudrate, 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
    node_handle->get_parameter_or<std::string>("frame_id", frame_id, "laser_frame");
    node_handle->get_parameter_or<bool>("inverted", inverted, false);
    node_handle->get_parameter_or<bool>("angle_compensate", angle_compensate, false);
    node_handle->get_parameter_or<std::string>("scan_mode", scan_mode, std::string());
    node_handle->get_parameter_or<int>("output_angle_min",output_angle_min,-180);
    node_handle->get_parameter_or<int>("output_angle_max",output_angle_max,180);

    scan_pub = node_handle->create_publisher<sensor_msgs::msg::LaserScan>("scan",10);
    //RCLCPP_INFO(node_handle->get_logger(),"RPLIDAR running on ROS package rplidar_ros. SDK Version:"RPLIDAR_SDK_VERSION"");
    u_result     op_result;

    // create the driver instance
    drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        //std::cout<<"Create Driver fail, exit" << std::endl;
        RCLCPP_ERROR(node_handle->get_logger(),"Create Driver fail, exit");
        return -2;
    }

    // make connection...
    if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
        RCLCPP_ERROR(node_handle->get_logger(),"Error, cannot bind to the specified serial port %s.",serial_port.c_str());
        //std::cout<<"Error, cannot bind to the specified serial port %s." << std::endl;
        RPlidarDriver::DisposeDriver(drv);
        return -1;
    }

    // get rplidar device info
    if (!getRPLIDARDeviceInfo(drv)) {
        return -1;
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        RPlidarDriver::DisposeDriver(drv);
        return -1;
    }
    drv->startMotor();

    RplidarScanMode current_scan_mode;
    if (scan_mode.empty()) {
        op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    } else {
        std::vector<RplidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

        if (IS_OK(op_result)) {
            _u16 selectedScanMode = _u16(-1);
            for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == scan_mode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == _u16(-1)) {
                RCLCPP_ERROR(node_handle->get_logger(),"scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    RCLCPP_ERROR(node_handle->get_logger(),"\t%s: max_distance: %.1f m, Point number: %.1fK",  iter->scan_mode,
                                iter->max_distance, (1000/iter->us_per_sample));
                    ;
                }
                op_result = RESULT_OPERATION_FAIL;
            } else {
                op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
            }
        }
    }
    if(IS_OK(op_result)) {
        //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
        angle_compensate_multiple = (int)(1000*1000/current_scan_mode.us_per_sample/10.0/360.0);
        if(angle_compensate_multiple < 1)
            angle_compensate_multiple = 1;
        max_distance = current_scan_mode.max_distance;
        RCLCPP_INFO(node_handle->get_logger(),"current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d",  current_scan_mode.scan_mode,
                    current_scan_mode.max_distance, (1000/current_scan_mode.us_per_sample), angle_compensate_multiple);
    } else {
        RCLCPP_ERROR(node_handle->get_logger(),"Can not start scan: %08x!", op_result);
    }


    rclcpp::Time start_scan_time;
    rclcpp::Time end_scan_time;
    double scan_duration;

    while(rclcpp::ok()) {
        rplidar_response_measurement_node_hq_t nodes[360*8];
        size_t   count = _countof(nodes);

        start_scan_time = node_handle->now();
        op_result = drv->grabScanDataHq(nodes, count);
        end_scan_time = node_handle->now();
        scan_duration = (end_scan_time - start_scan_time).seconds();
        if (op_result == RESULT_OK) {
            op_result = drv->ascendScanData(nodes, count);

            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);


            if (op_result == RESULT_OK) {
                if (angle_compensate) {
                    //const int angle_compensate_multiple = 1;
                    const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                    int angle_compensate_offset = 0;
                    rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_hq_t));

                    int i = 0, j = 0;
                    for( ; i < count; i++ ) {
                        if (nodes[i].dist_mm_q2 != 0) {
                            float angle = getAngle(nodes[i]);
                            int angle_value = (int)(angle * angle_compensate_multiple);
                            if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                            for (j = 0; j < angle_compensate_multiple; j++) {
                                angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
                            }
                        }
                    }

                    publish_scan(angle_compensate_nodes, angle_compensate_nodes_count,
                                 start_scan_time, scan_duration, inverted,
                                 angle_min, angle_max, max_distance,
                                 frame_id);
                } else {
                    int start_node = 0, end_node = 0;
                    int i = 0;
                    // find the first valid node and last valid node
                    while (nodes[i++].dist_mm_q2 == 0);
                    start_node = i-1;
                    i = count -1;
                    while (nodes[i--].dist_mm_q2 == 0);
                    end_node = i+1;

                    angle_min = DEG2RAD(getAngle(nodes[start_node]));
                    angle_max = DEG2RAD(getAngle(nodes[end_node]));

                    publish_scan(&nodes[start_node], end_node-start_node +1,
                                 start_scan_time, scan_duration, inverted,
                                 angle_min, angle_max, max_distance,
                                 frame_id);
                }
            } else if (op_result == RESULT_OPERATION_FAIL) {
                // All the data is invalid, just publish them
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);

                publish_scan(nodes, count,
                             start_scan_time, scan_duration, inverted,
                             angle_min, angle_max, max_distance,
                             frame_id);
            }
        }

		rclcpp::spin_some(node_handle);
    }
    // done!
    drv->stop();
    drv->stopMotor();
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}
