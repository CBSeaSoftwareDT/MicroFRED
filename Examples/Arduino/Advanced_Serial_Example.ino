#include <SoftwareSerial.h>
#define rxPin 3    // pin 3 connects to SMC TX
#define txPin 4    // pin 4 connects to SMC RX
#define resetPin 5 // pin 5 connects to SMC nRST
#define errPin 6   // pin 6 connects to SMC ERR
SoftwareSerial smcSerial = SoftwareSerial(rxPin, txPin);
 
// some variable IDs
#define ERROR_STATUS 0
#define LIMIT_STATUS 3
#define TARGET_SPEED 20
#define INPUT_VOLTAGE 23
#define TEMPERATURE 24
 
// some motor limit IDs
#define FORWARD_ACCELERATION 5
#define REVERSE_ACCELERATION 9
#define DECELERATION 2
 
// read a serial byte (returns -1 if nothing received after the timeout expires)
int readByte()
{
  char c;
  if(smcSerial.readBytes(&c, 1) == 0){ return -1; }
  return (byte)c;
}
 
// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart()
{
  smcSerial.write(0x83);
}
 
// speed should be a number from -3200 to 3200
void setMotorSpeed(int speed)
{
  if (speed < 0)
  {
    smcSerial.write(0x86);  // motor reverse command
    speed = -speed;  // make speed positive
  }
  else
  {
    smcSerial.write(0x85);  // motor forward command
  }
  smcSerial.write(speed & 0x1F);
  smcSerial.write(speed >> 5 & 0x7F);
}
 
unsigned char setMotorLimit(unsigned char  limitID, unsigned int limitValue)
{
  smcSerial.write(0xA2);
  smcSerial.write(limitID);
  smcSerial.write(limitValue & 0x7F);
  smcSerial.write(limitValue >> 7);
  return readByte();
}
 
// returns the specified variable as an unsigned integer.
// if the requested variable is signed, the value returned by this function
// should be typecast as an int.
unsigned int getVariable(unsigned char variableID)
{
  smcSerial.write(0xA1);
  smcSerial.write(variableID);
  return readByte() + 256 * readByte();
}
 
void setup()
{
  Serial.begin(115200);    // for debugging (optional)
  smcSerial.begin(19200);
 
  // briefly reset SMC when Arduino starts up (optional)
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);  // reset SMC
  delay(1);  // wait 1 ms
  pinMode(resetPin, INPUT);  // let SMC run again
 
  // must wait at least 1 ms after reset before transmitting
  delay(5);
 
  // this lets us read the state of the SMC ERR pin (optional)
  pinMode(errPin, INPUT);
 
  smcSerial.write(0xAA);  // send baud-indicator byte
  setMotorLimit(FORWARD_ACCELERATION, 4);
  setMotorLimit(REVERSE_ACCELERATION, 10);
  setMotorLimit(DECELERATION, 20);
  // clear the safe-start violation and let the motor run
  exitSafeStart();
}
 
void loop()
{
  setMotorSpeed(3200);  // full-speed forward
  // signed variables must be cast to ints:
  Serial.println((int)getVariable(TARGET_SPEED));
  delay(1000);
  setMotorSpeed(-3200);  // full-speed reverse
  Serial.println((int)getVariable(TARGET_SPEED));
  delay(1000);
 
  // write input voltage (in millivolts) to the serial monitor
  Serial.print("VIN = ");
  Serial.print(getVariable(INPUT_VOLTAGE));
  Serial.println(" mV");
 
  // if an error is stopping the motor, write the error status variable
  // and try to re-enable the motor
  if (digitalRead(errPin) == HIGH)
  {
    Serial.print("Error Status: 0x");
    Serial.println(getVariable(ERROR_STATUS), HEX);
    // once all other errors have been fixed,
    // this lets the motors run again
    exitSafeStart();
  }
}
