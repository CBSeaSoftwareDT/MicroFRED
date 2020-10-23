#include <ros/ros.h>
#include <std_msgs/Int32.h>
#define USE_USBCON
void decode(int);
void setMotorSpeed(int, int);

int left_motor_speed = 0;
int right_motor_speed = 0;

void motor_callback(const std_msgs::Int32& msg) {
  decode(msg.data);
  setMotorSpeed(left_motor_speed, right_motor_speed); 
}



void decode(int z) {
  int w = floor( (sqrt(z*8.0+1)-1)/2 );
  int t = (w*w + w)/2;
  right_motor_speed = z - t;
  left_motor_speed  = w-right_motor_speed;
}

// speed should be a number from -3200 to 3200
void setMotorSpeed(int left_speed, int right_speed)
{
 
  ROS_INFO_STREAM("left: " << left_speed << std::endl);
  ROS_INFO_STREAM("right: " << right_speed << std::endl << std::endl);
  
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "arduino_test");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("motor_data",1, motor_callback);
	ros::spin();

	return 0;

}
