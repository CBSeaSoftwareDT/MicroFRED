#include<ros/ros.h>
#include<std_msgs/Float64.h>

class teleop_subscriber
{
public:
	
	teleop_subscriber(ros::NodeHandle nh_)
	{
		sub_ = nh_.subscribe("user_input", 1, &teleop_subscriber::myCallback,this);
		pub_left_mtr = nh_.advertise<std_msgs::Float64>("left_vel",1);
		pub_right_mtr = nh_.advertise<std_msgs::Float64>("right_vel",1);
	}

	void myCallback(const std_msgs::Float64& message_holder)
	{
	
		//add code to determine vals
		std_msgs::Float64 left;
		std_msgs::Float64 right;

		//if right command
		if(message_holder.data > 0)
		{
			//move to top when able to determine vals
			left.data = 0;
			right.data = 1;
			pub_right_mtr.publish(right);
			pub_left_mtr.publish(left);
		}
		else if(message_holder.data < 0)
		{
			left.data = 1;
			right.data = 0;
			pub_right_mtr.publish(right);
			pub_left_mtr.publish(left);
		}
		else
		{

			left.data = 1;
			right.data = 1;
			pub_right_mtr.publish(right);
			pub_left_mtr.publish(left);
		}
	}
private:
	ros::Subscriber sub_;
	ros::Publisher pub_left_mtr;
	ros::Publisher pub_right_mtr;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop_subscriber");
	ros::NodeHandle nh_;
	
	teleop_subscriber inst = teleop_subscriber(nh_);

	ros::spin();

	return 0;

}
