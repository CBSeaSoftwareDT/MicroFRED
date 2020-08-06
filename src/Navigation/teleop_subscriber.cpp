#include<ros/ros.h>
#include<std_msgs/Float32.h>

class teleop_subscriber
{
public:
	teleop_subsriber()
	{
		ros::NodeHandle nh_;
		sub_ = nh.subscribe("user_input", 1,myCallback);
		pub_left_mtr = nh_.advertise<std_msdgs::Float32>("left_vel");
		pub_right_mtr = nh_.advertise<std_msdgs::Float32>("right_vel");

	}
	
	void myCallback(const std_msgs::Float32& message_holder)
	{
	
		//add code to determine vals
		std_msgs::Float32 left;
		std_msgs::Float32 right;

		//if right command
		if(message_holder>0)
		{
			//move to top when able to determine vals
			left=0;
			right=1;
			pub_right_mtr.publish(right);
			pub_left_mtr.publish(left);
		}
		else if(message_holder<0)
		{
			left=1;
			right=0;
			pub_right_mtr.publish(right);
			pub_left_mtr.publish(left);
		}
		else
		{

			left=1;
			right=1;
			pub_right_mtr.publish(right);
			pub_left_mtr.publish(left);
		}
	};

private:
	ros::Subscriber sub_;
	ros::Publisher pub_left_mtr;
	ros::Publisher pub_right_mtr;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop_subscriber");

	teleop_subsriber inst;

	ros::spin();

	return 0;

}
