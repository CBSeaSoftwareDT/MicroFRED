#include<ros/ros.h>
#include<Navigation/Joystick.h>
#include<Navigation/Motor_Speed.h>

#define INPUT_LOWER_BOUND -90		//the lowest value a user input can give you 
#define INPUT_MIDDLE 0 			// the midpoint of your user input values (your 0 point)
#define INPUT_UPPER_BOUND 90	//the maximum value a user input can give you 
#define MOTOR_LOWER_BOUND 100   //the slowest you would like the motor to spin 
#define MOTOR_UPPER_BOUND 3200  //the fastest you would like the motor to spin (this number is usually given by the manufacturer)

class teleop_subscriber
{

public:
	
	/**
	 * Constructor for the teleop_subscriber class
	 * @param nh     NodeHandle object 
	**/
	teleop_subscriber(ros::NodeHandle nh)
	{
		sub = nh.subscribe("user_input", 1, &teleop_subscriber::joystick_callback,this);
		pub_motor_data = nh.advertise<Navigation::Motor_Speed>("motor_data",1);
	}

	/**
	 * This maps an integer bleonging to a certain range into an integer 
	 * belonging to another range. 
	 * @param x       integer you would like to be mapped
	 * @param in_min  lower bound for input range 
	 * @param in_max  upper bound for input range
	 * @param out_min lower bound for output range 
	 * @param out_max upper bound for output range 
	 * @return int    new integer that is mapped to the specified range
	**/
	int map(long x, long in_min, long in_max, long out_min, long out_max) {
		return (x -in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	/**
	 * This is the callback function that gets called when the node recieves a user_inuput topic
	**/
	void joystick_callback(const Navigation::Joystick& joystick_data){
		int right_motor_speed; 	//motor speed for right motor
		int left_motor_speed; 	//motor speed for left motor
		bool forwards_motor;
		Navigation::Motor_Speed motor_speeds; //where the motor speeds will go after being processed

		//if there is a vertical component 
		if(joystick_data.joystick_vertical > INPUT_MIDDLE) {
			
			//GO FORWARD 
			// here, the left and right motor will be the same speed because it is a forward command

			//map the input, whose range is from 0-90 to the output, whose range will be 0-3200
			left_motor_speed = map(joystick_data.joystick_vertical, INPUT_MIDDLE, 
				INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);
			right_motor_speed = left_motor_speed;
			forwards_motor = true;
			}
		else if (joystick_data.joystick_vertical < INPUT_MIDDLE) {
			int temp  = joystick_data.joystick_vertical*-1;
			forwards_motor = false;
			left_motor_speed = map(temp,INPUT_MIDDLE, INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);
			right_motor_speed  = left_motor_speed;
		}
		
		else {
			left_motor_speed = 0;
			right_motor_speed = 0;
		}

		if(joystick_data.joystick_horizontal < 0) {
			//TURN LEFT
			//this means the left wheel needs to move slower than the wheel on the right 

			//change the data to a positive number
			int temp = joystick_data.joystick_horizontal*-1;
			
			//offset the left and right motor speed by the correct, mapped amount
			left_motor_speed -= map(temp, INPUT_LOWER_BOUND, 
				INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);
			right_motor_speed += map(temp, INPUT_LOWER_BOUND, 
				INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);


			//with the addition and subtraction, there is a chance the speed could go too low/high. This corrects that 
			if (left_motor_speed < MOTOR_LOWER_BOUND) { 
				left_motor_speed = MOTOR_LOWER_BOUND;
			}

			if (right_motor_speed > MOTOR_UPPER_BOUND) {
				right_motor_speed = MOTOR_UPPER_BOUND;
			}

		}
		else if(joystick_data.joystick_horizontal > 0) {
			//TURN RIGHT 
			//this means the left wheel needs to move faster than the wheel on the right 

			//offset the left and right motor speed by the correct, mapped amount
			left_motor_speed += map(joystick_data.joystick_horizontal, INPUT_LOWER_BOUND, 
				INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);
			right_motor_speed -= map(joystick_data.joystick_horizontal, INPUT_LOWER_BOUND, 
				INPUT_UPPER_BOUND, MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND);

			//ensure motor speeds are still in the correct range.
			if(left_motor_speed > MOTOR_UPPER_BOUND) {
				left_motor_speed = MOTOR_UPPER_BOUND;
			}
			if (right_motor_speed < MOTOR_LOWER_BOUND) {
				right_motor_speed = MOTOR_LOWER_BOUND;
			}
		}

		//encode the two motor speeds into a integer
		motor_speeds.forwards_direction = forwards_motor;
		motor_speeds.left_motor_speed = left_motor_speed;
		motor_speeds.right_motor_speed = right_motor_speed;
		
		//send data to the Arduino 
		pub_motor_data.publish(motor_speeds);

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
