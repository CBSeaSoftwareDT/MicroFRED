#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
	ros::init(argc, argv, "user_input_node");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<std_msgs::Float64>("user_input", 1);
	std_msgs::Float64 input;
	
	input.data = 0;

	ros::Rate naptime(1.0);

	while(ros::ok())
	{
		std::cout << "Enter float between -90 and 90: ";
		float float_input;
		std::cin >> float_input;
		if(float_input > 90 || float_input < -90 )
			std::cout << "Invalid input";
		else {
			input.data = float_input;
			publisher.publish(input);
		}

		naptime.sleep();
	}
}
