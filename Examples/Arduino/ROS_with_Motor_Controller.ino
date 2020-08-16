#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#define USE_USBCON
#define rxPin 3  // pin 3 connects to smcSerial TX  (not used in this example)
#define txPin 4  // pin 4 connects to smcSerial RX
#define rxPin2 2  //pin 2 connects to smcSerial TX (not used in this example)
#define txPin2 5  //pin 5 connects to smcSerial RX


ros::NodeHandle nh;
int left_motor_speed = 0;
int right_motor_speed = 0;
SoftwareSerial smcSerial_left = SoftwareSerial(rxPin, txPin);
SoftwareSerial smcSerial_right = SoftwareSerial(rxPin2, txPin2);

void motor_callback(const std_msgs::Int32& msg) {
  decode(msg.data);
  setMotorSpeed(left_motor_speed, right_motor_speed);
}



ros::Subscriber<std_msgs::Int32> sub("motor_data", &motor_callback);



// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart()
{
  smcSerial_left.write(0x83);
  smcSerial_right.write(0x83);
}

void test() {
  int speed = 3200;
  smcSerial_left.write(speed & 0x1F);
  smcSerial_right.write(speed & 0x1F);
  smcSerial_left.write(speed >> 5 & 0x7f);
  smcSerial_right.write(speed >> 5 & 0x7f);
  
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
 
  smcSerial_left.write(0x85);
  smcSerial_right.write(0x85);
 
  smcSerial_left.write(left_speed & 0x1F);
  smcSerial_right.write(right_speed & 0x1F);
  
  smcSerial_left.write(left_speed >> 5 & 0x7F);
  smcSerial_right.write(right_speed >> 5 & 0x7F);
  
}



void setup()
{
  // Initialize software serial object with baud rate of 19.2 kbps.
  smcSerial_left.begin(19200);
  smcSerial_right.begin(19200);

  // The Simple Motor Controller must be running for at least 1 ms
  // before we try to send serial data, so we delay here for 5 ms.
  delay(5);

  // If the Simple Motor Controller has automatic baud detection
  // enabled, we first need to send it the byte 0xAA (170 in decimal)
  // so that it can learn the baud rate.
  smcSerial_left.write(0xAA);
  smcSerial_right.write(0xAA);

  // Next we need to send the Exit Safe Start command, which
  // clears the safe-start violation and lets the motor run.
  exitSafeStart();
  test();

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);

}
