#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <time.h>
#include <sstream>

using namespace std;

stringstream ss;
int tim1[3]={0};
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1;
rclcpp::Node::SharedPtr node_handle = nullptr;

//timer1回调处理函数
//功能：在初试时间基础上进行计算，将计算所得的时间pub到话题“time1”上
void timer1_callback()
{
	static int i = 0;
	auto message = std_msgs::msg::String();
	i++;
	//250ms中断四次加一秒
	if(i>=4){
		i=0;
		tim1[2]++;
		if(tim1[2]>=60){
			tim1[2]=0;
			tim1[1]++;
			if(tim1[1]>=60){
				tim1[1]=0;
				tim1[0]++;
				if(tim1[0]>=24)tim1[0]=0;
			}
		}
	}else{

	}
	ss.str("");
	ss.clear();
	ss<<"timer1   - "<<tim1[0]<<":"<<tim1[1]<<":"<<tim1[2];
	message.data = ss.str();
	//pub
	pub1->publish(message);
}
//timer2回调处理函数
//功能：获取系统时间并将其打印到终端。
void timer2_callback()
{
	//取得系统时间
	time_t tt;
	time( &tt );
	tt = tt + 8*3600;  // transform the time zone
	tm* t= gmtime( &tt );
	//打印到终端上
	cout<<"current time   - "<<t->tm_hour<<":"<<t->tm_min<<":"<<t->tm_sec<<endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
	//初始化函数
	node_handle = rclcpp::Node::make_shared("time_piece");
	//发布者发布“time1”话题
	pub1= node_handle->create_publisher<std_msgs::msg::String>("time1",10);

	//时间及处理
	time_t tt;
    time( &tt );
    tt = tt + 8*3600;  // transform the time zone
    tm* t= gmtime( &tt );
	ss<<"current time   - "<<t->tm_hour<<":"<<t->tm_min<<":"<<t->tm_sec;
	cout<< ss.str() <<endl;
	ss.str("");
	ss.clear();
	//存储开始时间
	tim1[0]=t->tm_hour;
	tim1[1]=t->tm_min;
	tim1[2]=t->tm_sec;

	//创建两个定时器
	rclcpp::TimerBase::SharedPtr timer1,timer2;

	//定时器timer1初始化,250ms中断，回调函数timer1_callback
    timer1 = node_handle->create_wall_timer(250ms,timer1_callback);
	//定时器timer2初始化,100ms中断，回调函数timer2_callback
    timer2 = node_handle->create_wall_timer(100ms,timer2_callback);

	//回调处理函数
    rclcpp::spin(node_handle);
	//关闭节点
	rclcpp::shutdown();
	return 0;
}