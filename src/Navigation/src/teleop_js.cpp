#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <Navigation/Joystick.h>
#include <iostream>
#include <string>

class js_Handler{

	private:
		ros::Subscriber sub;
		ros::Publisher pub;
		Navigation::Joystick joystick_data;
		
	public:
		js_Handler(ros::NodeHandle nh){
			sub = nh.subscribe("joy", 1, &js_Handler::js_callback, this);
			pub = nh.advertise<Navigation::Joystick>("user_input", 1);
		}
		
		void js_callback(const sensor_msgs::Joy::ConstPtr& js_msgs){
			// Axis 0 is horizontal, 1 is left, -1 is right
			// Axis 1 is vertical, 1 is up, -1 is down
			joystick_data.joystick_vertical = int(js_msgs->axes[1]*90);
			joystick_data.joystick_horizontal = int(js_msgs->axes[0]*90*-1);
			pub.publish(joystick_data);
		}
		
		
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "teleop_js");
	ros::NodeHandle nh;

	js_Handler js_handler = js_Handler(nh);
	
	ros::spin();
	
	return 0;
}
