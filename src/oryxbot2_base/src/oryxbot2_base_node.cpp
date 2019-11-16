#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "modbus/modbus-rtu.h"
#include "vector"
#include "string"
#include "oryxbot2_msgs/msg/carcmd.hpp"
#include "oryxbot2_msgs/msg/cardata.hpp"


#define READ_SPEED 0
#define READ_POWER_VOLTAGE  16
#define READ_MPU  17
using namespace std::chrono_literals;
using namespace std;

uint16_t mbbuf[35];
uint16_t probeen=0;
oryxbot2_msgs::msg::Cardata  my_data;
modbus_t *mb=NULL;

void translate_mb_data(void)
{
    union {
        double dd;
        uint16_t id[4];
    } d_to_uint;
    uint16_t *mbp=&mbbuf[READ_SPEED];
    for(int i=0; i<4; i++) {
        d_to_uint.id[3]=*mbp++;
        d_to_uint.id[2]=*mbp++;
        d_to_uint.id[1]=*mbp++;
        d_to_uint.id[0]=*mbp++;
        my_data.speed[i]=d_to_uint.dd;
    }

    my_data.power_voltage=mbbuf[READ_POWER_VOLTAGE];
    mbp=&mbbuf[READ_MPU];
    union {
        float fd;
        uint16_t id[2];
    } f_to_uint;
    for(int i=0; i<9; i++) {
        f_to_uint.id[1]=*mbp++;
        f_to_uint.id[0]=*mbp++;
        my_data.mpu[i]=f_to_uint.fd;
    }
}


void twistCallback(const oryxbot2_msgs::msg::Carcmd::SharedPtr mycmd)
{
    if(mycmd->speed.size()!=4) return;
    union {
        double dd;
        uint16_t id[4];
    } d_to_uint;
    uint16_t send_data[17];
    uint16_t *p=send_data;
    for(int i=0; i<4; i++) {
        d_to_uint.dd=mycmd->speed[i];
        *p++=d_to_uint.id[3];
        *p++=d_to_uint.id[2];
        *p++=d_to_uint.id[1];
        *p++=d_to_uint.id[0];
    }
    if(mycmd->been!=probeen)  send_data[16]=mycmd->been;
    else send_data[16]=0;
    probeen=mycmd->been;

    int res = modbus_write_registers(mb,0,17,send_data);
    std::cout << "res: " << res << std::endl;
}
std::string serial_port="/dev/ttyS";
std::string usbserial_port="/dev/ttyUSB";
std::string connection_port = "";
int main(int argc, char **argv)
{
    std::vector<std::string> port;
    port.push_back(serial_port);
    port.push_back(usbserial_port);
    //RCLCPP_INFO("scaning available port");
    bool is_connected = false;

    for(int j=0; j<9; j++) {
        for(int i=0; i < 127; i++) {
            std::string curr_port = port[j] + std::to_string(i);
            if(!(mb=modbus_new_rtu(curr_port.c_str(),115200,'N',8,1))) continue;
            if(modbus_set_slave(mb,1)==-1) {
                modbus_free(mb);
                continue;
            }
            if(modbus_connect(mb)==-1) {
                modbus_free(mb);
                continue;
            }
          //  modbus_set_response_timeout(mb,0,200000);
            if(modbus_read_input_registers(mb,0,35,mbbuf)==35) {
                connection_port = curr_port;
                is_connected = true;
                break;
            }
        }//end for

        if(is_connected) break;
        continue;
    }//end for
    if(!is_connected) {
        //ROS_ERROR_STREAM("control borad connection failed");
        return -1;
    }
    //ROS_INFO_STREAM("available prot: " << connection_port);
    //modbus_set_response_timeout(mb,0,200000);

    my_data.speed.resize(4);
    my_data.mpu.resize(9);
/*************************************************************************/
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("car_controller");

  auto publisher = node->create_publisher<oryxbot2_msgs::msg::Cardata>("car_data", 10);
  auto subscription = node->create_subscription<oryxbot2_msgs::msg::Carcmd>("car_cmd", 10, twistCallback); 

  rclcpp::WallRate loop_rate(50ms);

    while (rclcpp::ok()) {
        memset(mbbuf,0,35*2);
	//usleep(2*1000);
        if(modbus_read_input_registers(mb,0,35,mbbuf)==35) {
            translate_mb_data();
	    
    		publisher->publish(my_data);
        }

	rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    modbus_close(mb);
    modbus_free(mb);
	rclcpp::shutdown();
    return 0;
}

