#include<ros/ros.h>
#include<std_msgs/Int32.h>
#include<Navigation/Joystick.h>

#define INPUT_LOWER_BOUND 0
#define INPUT_UPPER_BOUND 90
#define MOTOR_LOWER_BOUND 100
#define MOTOR_UPPER_BOUND 3200

class teleop_subscriber
{

public:
	
	teleop_subscriber(ros::NodeHandle nh)
	{
		sub = nh.subscribe("user_input", 1, &teleop_subscriber::joystick_callback,this);
		pub_motor_data = nh.advertise<std_msgs::Int32>("motor_data",1);
	}

	int map(long x, long in_min, long in_max, long out_min, long out_max) {
		return (x -in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	void joystick_callback(const Navigation::Joystick& joystick_data){
		int right_motor_speed;
		int left_motor_speed;
		std_msgs::Int32 motor_speeds;

		if(joystick_data.joystick_vertical > 5) {
			left_motor_speed = map(joystick_data.joystick_vertical, INPUT_LOWER_BOUND, 
				INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);
			right_motor_speed = left_motor_speed;
		}
		else {
			left_motor_speed = 0;
			right_motor_speed = 0;
		}

		if(joystick_data.joystick_vertical < 0) {
			//TURN LEFT 

			int temp = joystick_data.joystick_vertical*-1;
			
			left_motor_speed -= map(temp, INPUT_LOWER_BOUND, 
				INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);
			right_motor_speed += map(temp, INPUT_LOWER_BOUND, 
				INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);
			
			if (left_motor_speed < MOTOR_LOWER_BOUND) { 
				left_motor_speed = MOTOR_LOWER_BOUND;
			}

			if (right_motor_speed > MOTOR_UPPER_BOUND) {
				right_motor_speed = MOTOR_UPPER_BOUND;
			}

		}
		else if(joystick_data.joystick_vertical > 0) {
			//TURN RIGHT 

			left_motor_speed += map(joystick_data.joystick_vertical, INPUT_LOWER_BOUND, 
				INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);
			right_motor_speed -= map(joystick_data.joystick_vertical, INPUT_LOWER_BOUND, 
				INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);

			if(left_motor_speed > MOTOR_UPPER_BOUND) {
				left_motor_speed = MOTOR_UPPER_BOUND;
			}
			if (right_motor_speed < MOTOR_LOWER_BOUND) {
				right_motor_speed = MOTOR_LOWER_BOUND;
			}
		}

		motor_speeds.data = cantor_pairing_algo(left_motor_speed, right_motor_speed);
		pub_motor_data.publish(motor_speeds);

	}


	int cantor_pairing_algo(int num1, int num2) {
		return ((num1 + num2) * (num1 + num2 + 1))/2 + num2;
	}

private:
	ros::Subscriber sub;
	ros::Publisher pub_motor_data;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "user_input_subscriber");
	ros::NodeHandle nh;
	
	teleop_subscriber inst = teleop_subscriber(nh);

	ros::spin();

	return 0;

}
