#include <ros/ros.h>4.h>
#include <Navigation/Joystick.h>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
	ros::init(argc, argv, "teleop");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<Navigation::Joystick>("user_input", 1);
	Navigation::Joystick user_input;
	
	int input;
	user_input.joystick_horizontal = 0;
	user_input.joystick_vertical = 0;

	ros::Rate naptime(1.0);

	while(ros::ok())
	{
		std::cout << "Enter any integer between -90 and 90 for the vertical joystick reading: ";
		
		std::cin >> input;
		if(input > 90 || input < -90 )
			std::cout << "Invalid input";
		else {
			user_input.joystick_vertical = input;
		
			std::cout << "Enter any integer between -90 and 90 for the horizontal joystick reading: ";
			std::cin >> input;
			if(input < -90 || input >90)
				std::cout << "Invalid input";
			else 
				user_input.joystick_horizontal = input;
			
			
			publisher.publish(user_input);

			naptime.sleep();
		}
	}
}
