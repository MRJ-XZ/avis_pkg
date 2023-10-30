#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
using namespace ros;

void distance_callback(const std_msgs::Int16 & );
void sensors_callback(const std_msgs::Int16MultiArray & );
void carSpeed_callback(const std_msgs::Int16 &);
void angle_callback(const std_msgs::Int16 & );

int main(int argc, char **argv)
{
  init(argc, argv, "controller");
  NodeHandle n;
  Publisher speed_pub = n.advertise<std_msgs::Int16>("speed", 10);
  Publisher steering_pub = n.advertise<std_msgs::Int16>("steering", 10);
  Subscriber sensor_sub = n.subscribe("sensors", 1000, sensorsCallback);
  Subscriber carspeed_sub = n.subscribe("carSpeed", 10, carSpeedCallback);
  Subscriber center_sub = n.subscribe("center", 10, diCallback);
  Subscriber angle_sub = n.subscribe("turn", 10, angleCallback);
  std_msgs::Int16 speed;
  std_msgs::Int16 steering;
  Rate loop_rate(30);
  Sensor sensor(1000 , 50);
	
  while(ok()){
  
  
    steering.data = 0; 
    speed.data = 100;
    speed_pub.publish(speed);
    steering_pub.publish(steering);
    spinOnce();
    loop_rate.sleep();
  } 
}
void distance_callback(const std_msgs::Int16& msg){
    msg.data;
}
void sensors_callback(const std_msgs::Int16MultiArray& msg){
    msg.data[i];
}
void carSpeed_callback(const std_msgs::Int16& msg){
    msg.data;
}
void angle_callback(const std_msgs::Int16& msg){
    msg.data;
}
