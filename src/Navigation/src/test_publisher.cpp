#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv){
	ros::init(arg, argv, "test_publisher");
	ros::NodeHandle n;
	ros::Publisher my_publisher_object = n.advertise<std_msgs::Float32>("user_input",1);

	std_msgs::Float32 deg1=0;
	std_msgs::Float32 deg2=-85;
	std_msgs::Float32 deg3=90;
	std_msgs::Float32 deg4=2;
	
	my_publisher_object.publsih(deg1);
	my_publisher_object.publsih(deg2);
	my_publisher_object.publsih(deg3);
	my_publisher_object.publsih(deg4);
}

