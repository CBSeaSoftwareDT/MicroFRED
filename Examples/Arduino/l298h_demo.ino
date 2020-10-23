// Motor A
int enA = 9; //enable line
int in1 = 8; //input 1, 2
int in2 = 7;

// Motor B

int enB = 3; //enable line
int in3 = 5; //input 3,4
int in4 = 4;

//the numbers defined are the PIN numbers. The enable pins must be ones that are capable of PWM (pins 9 and 3)



void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void demoOne() {
  // this function will run the motors in both directions at a fixed speed

  //turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  //set Speed to 200 out of a range of 0-255. This sends out the PWM signals
  analogWrite(enA, 200);

  // turn on motor B  

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enB, 200);

  delay(2000);

  //change the direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  delay(2000);

  //turn motors off
  digitalWrite(in1, LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void demoTwo() {
  //turn motors on
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  for(int i = 0; i<256; i++) {
    analogWrite(enA, i);
    analogWrite(enB,i);
  }

  //decelerate 

  for(int i = 255; i>=0; i--) {
    analogWrite(enA, i);
    analogWrite(enB,i);
  }

  //turn motors off
  digitalWrite(in1, LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
  
void loop() {
  demoOne();

  delay(1000);

}
