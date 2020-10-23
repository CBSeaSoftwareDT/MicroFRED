
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

void callback_left(const std_msgs::Float64& msg) {
  digitalWrite(13, HIGH-digitalRead(13));
}
/**
void callback_right(const std_msgs::Float64& msg) {
  Serial.print("right motor");
}
**/

ros::Subscriber<std_msgs::Float64> sub_left("left_vel", &callback_left);
//ros::Subscriber<srd_msgs::Empty> sub_right("right_vel", &callback_right);

void setup() 
{
  pinMode(13, OUTPUT);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_left);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
