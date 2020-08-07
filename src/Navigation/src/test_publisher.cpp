#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "test_publisher");
	ros::NodeHandle n;
	ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("user_input",1);

	while(ros::ok())
	{
		std_msgs::Float64 deg1; 
		deg1.data = 0;
		my_publisher_object.publish(deg1);
	
		std_msgs::Float64 deg2;
		my_publisher_object.publish(deg2);
		deg2.data = -85;
	
		std_msgs::Float64 deg3;
		deg3.data = 90;
		my_publisher_object.publish(deg3);
	
		std_msgs::Float64 deg4;
		deg4.data = 2;
		my_publisher_object.publish(deg4);
	}
}

