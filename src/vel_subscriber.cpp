#include "ros/ros.h"                                            
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>
//#include <serial/serial.h> // 추가
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define PI 3.141592
#define FUNCTION_WRITE 6
#define FUNCTION_ADDRESS 120
#define RPM_ADDRESS 121
#define FUNCTION_DATA_START 1
#define FUNCTION_DATA_STOP 2
#define RPM_MAX 2000
#define WHEEL_DIAMETER 0.125
#define WHEELBASE 0.28
#define MOTOR_REDUCER 20.0
#define LEFT_ID 2
#define RIGHT_ID 1

//serial::Serial ser;

int what;
float speed(0.5); // Linear velocity (m/s)
float turn(0.5); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
char key(' ');

char BufferPacket[] = {0,};

void msgCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
//데이터가 /numbers 토픽에 도달하였는지 확인하기 위해 사용
  ROS_INFO("Linear.x %f", msg->linear.x);   //-> : 역참조,Dereferencing operation
  ROS_INFO("Angular %f", msg->angular.z);   //데이터가 저장된 주소로 가서, 
                                            //그 주소에 해당하는 데이터 값에 접근하는 것
  
  if(key == 115)
  {
    speed = 0;
    turn = 0;
  }

  //ser.write(BufferPacket);//<-이부분 넣어야하지않나?..
}

int main(int argc, char **argv)                         
{
  ros::init(argc, argv, "chang_teleop_sub"); 
  ros::NodeHandle nh;                                   
  
  
  ROS_INFO_STREAM("Waiting for /chang_teleop_pub");
  
  // try
  // {
  //     ser.setPort("/dev/ttyFT232");
  //     ser.setBaudrate(115200);
  //     serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  //     ser.setTimeout(to);
  //     ser.open();
  // }

  // catch (serial::IOException& e)
  // {
  //     ROS_ERROR_STREAM("Unable to open port ");
  //     return -1;
  // }

  // if(ser.isOpen()){
  //     ROS_INFO_STREAM("Serial Port initialized");
  // }else{
  //    return -1;
  // }

  //ros::Subscriber dong_ros_sub = nh.subscribe("ros_Dong_msg", 100, msgCallback);
  ros::Subscriber sub = nh.subscribe("cmd_vel", 1, msgCallback);
  //ros::Subscriber dong_ros_sub = nh.subscribe("cmd_vel", 100, msgCallback);
  ros::Rate loop_rate(10);

  while(ros::ok()){
    
    ros::spinOnce();
    
    loop_rate.sleep();
  }
}
