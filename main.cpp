
/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  RobotIntro.ino
  Carlotta Berry 11.21.16

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
  The primary functions created are
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME

  Hardware Connections:
  Arduino pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
  A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182 

  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin
  digital pin 13 - enable LED on microcontroller

  digital pin 6 - red LED in series with 220 ohm resistor
  digital pin 7 - green LED in series with 220 ohm resistor
  digital pin 8 - yellow LED in series with 220 ohm resistor

  digital pin 18 - left encoder pin
  digital pin 19 - right encoder pin

  digital pin 2 - IMU INT
  digital pin 20 - IMU SDA
  digital pin 21 - IMU SCL


  INSTALL THE LIBRARY
  AccelStepper Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
  
  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
  See PlatformIO documentation for proper way to install libraries in Visual Studio
*/

//includew all necessary libraries
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <Adafruit_MPU6050.h>//Include library for MPU6050 IMU
#include <SoftwareSerial.h> //include Bluetooth module

//state LEDs connections
#define redLED 6            //red LED for displaying states
#define grnLED 7            //green LED for displaying states
#define ylwLED 8            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED

//define motor pin numbers
#define stepperEnable 48    //stepper enable pin on stepStick 
#define rtStepPin 50 //right stepper motor step pin 
#define rtDirPin 51  // right stepper motor direction pin 
#define ltStepPin 52 //left stepper motor step pin 
#define ltDirPin 53  //left stepper motor direction pin 

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

int pauseTime = 2500;   //time before robot moves
int stepTime = 500;     //delay time between high and low on step pin
int wait_time = 1000;   //delay for printing data

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

//IMU object
Adafruit_MPU6050 mpu;

//Bluetooth module connections
#define BTTX 10 // TX on chip to pin 10 on Arduino Mega
#define BTRX 11 //, RX on chip to pin 11 on Arduino Mega
SoftwareSerial BTSerial(BTTX, BTRX);

// Helper Functions

//interrupt function to count left encoder tickes
void LwheelSpeed()
{
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed()
{
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
}

//function to initialize Bluetooth
void init_BT(){
  Serial.println("Goodnight moon!");
  BTSerial.println("Hello, world?");
}
//function to initialize IMU
void init_IMU(){
  Serial.println("Adafruit MPU6050 init!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

//function to set all stepper motor variables, outputs and LEDs
void init_stepper(){
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED

  stepperRight.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10000);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
}

//function prints encoder data to serial monitor
void print_encoder_data() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT] = 0;                         //clear the right encoder data buffer
    timer = millis();                           //record current time since program started
  }
}

//function to print IMU data to the serial monitor
void print_IMU_data(){
    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
}

//function to send and receive data with the Bluetooth
void Bluetooth_comm(){
  String data="";
  if (Serial.available()) {
     while (Serial.available()){
      char nextChar = Serial.read();
      data = data + String(nextChar); 
      if (nextChar == ';') {
        break;
      }
     }
    Serial.println(data);
    BTSerial.println(data);
  }
  
  if (BTSerial.available()) {
    while (BTSerial.available()){
      char nextChar = BTSerial.read();
      data = data + String(nextChar); 
      if (nextChar == ';') {
        break;
      }
    }
    Serial.println(data);
    BTSerial.println(data);
  }
}
  
  
/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}


/*
   The move1() function will move the robot forward one full rotation and backwared on
   full rotation.  Recall that that there 200 steps in one full rotation or 1.8 degrees per
   step. This function uses setting the step pins high and low with delays to move. The speed is set by
   the length of the delay.
*/
void move1() {
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
  digitalWrite(ltDirPin, LOW); // Enables the motor to move in opposite direction
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
}

/*
   The move2() function will use AccelStepper library functions to move the robot
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move2() {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  stepperRight.moveTo(800);//move one full rotation forward relative to current position
  stepperLeft.moveTo(800);//move one full rotation forward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
  stepperRight.moveTo(0);//move one full rotation backward relative to current position
  stepperLeft.moveTo(0);//move one full rotation backward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
}

/*
   The move3() function will use the MultiStepper() class to move both motors at once
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move3() {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  long positions[2]; // Array of desired stepper positions
  positions[0] = 800;//right motor absolute position
  positions[1] = 800;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
  // Move to a different coordinate
  positions[0] = 0;//right motor absolute position
  positions[1] = 0;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
}

/*this function will move to target at 2 different speeds*/
void move4() {

  int leftPos = 5000;//right motor absolute position
  int rightPos = 1000;//left motor absolute position
  int leftSpd = 5000;//right motor speed
  int rightSpd = 1000; //left motor speed

  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  //Uncomment the next 4 lines for absolute movement
  stepperLeft.setCurrentPosition(0);//set left wheel position to zero
  stepperRight.setCurrentPosition(0);//set right wheel position to zero
  stepperLeft.moveTo(leftPos);//move left wheel to absolute position
  stepperRight.moveTo(rightPos);//move right wheel to absolute position

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);//move right wheel to relative position

  //Uncomment the next two lines to set the speed
  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed
  runAtSpeedToPosition();//run at speed to target position
}

/*This function will move continuously at 2 different speeds*/
void move5() {
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  int leftSpd = 5000;//right motor speed
  int rightSpd = 1000; //left motor speed
  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed
  runAtSpeed();
}




/*
  It turns the robot in the given direction a given number of revolutions, with the axis of rotation in one of the wheels.
  The number of rotations is converted to pulses using the diameter of the wheels, the width of the robot and 
  the number of pulses per feet, which yields the factor 4100.


  @param direction clockwise (0) or counterclockwise (1) direction
  @param spins number of full revolutions to execute

*/
void pivot(int direction, float spins) {
  spins=spins*4100; // The factor to go from revolutions to pulses was found to be 4100 pulses/rev
  int speed = 500;  // Speed of motor turning
  if(direction==1){  // Clockwise rotation
    stepperLeft.move(spins);
    stepperLeft.setMaxSpeed(speed);
  }
  else if(direction==0){  // Counter-clockwise rotation
    stepperRight.move(spins);
    stepperLeft.setMaxSpeed(speed);
  }
  steppers.runSpeedToPosition();
  runToStop();
}

/*
  It turns the robot in the given direction a given number of revolutions, with the axis of rotation in the center of the robot.
  The number of rotations is converted to pulses using the diameter of the wheels, the width of the robot and 
  the number of pulses per feet, which yields the factor 1997.
  It does not use the AccelStepper library

  @param direction clockwise (0) or counterclockwise (1) direction
  @param spins number of full revolutions to execute
*/
void spin(int direction, double spins) {
  if (direction == 1) {  // Clockwise rotation
    digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
    digitalWrite(rtDirPin, LOW); // Enables the motor to move in a particular direction
  }
  else if (direction == 0) {  // Counter-clockwise rotation
    digitalWrite(ltDirPin, LOW); // Enables the motor to move in a particular direction
    digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
  }
  else { // Wrong input
    printf("Insert 1(clockwise) or 0(counter cockwise)");
  }
  
  spins = spins*1997;  // This factor is the number of pulses of each motor for a full revolution
  for (int x = 0; x < spins; x++) { // Makes the motors move the pulses needed to complete the spins
    digitalWrite(ltStepPin, HIGH); 
    delayMicroseconds(1000);
    digitalWrite(rtStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(1000);
    delayMicroseconds(stepTime);
  }


}

/*
  It turns the robot in the given direction a given number of revolutions following a circle of a given diameter.
  The number of rotations is converted to pulses using the width of the robot, the diameter of the circle and the 
  number of 912 pulses per feet.
  The outer wheel always has a speed of 800 and the inner wheel speed is proportional to the ratio of the distances
  the inner and outer wheel travel.


  @param direction clockwise (0) or counterclockwise (1) direction
  @param spins number of full revolutions to execute
  @param diameter diameter of the turn to execute
*/
void turn(int direction, float spins, float diameter) {
float width = 8.464;                // Width of the robot
float ang = spins * 2 * 3.142;      // Rotation angle to travel
float s2 = (diameter/2) * ang;      // Inner circumference travelled by the inner wheel in inches
float s1 = s2 + (width * ang);      // Outer circumference travelled by the outer wheel in inches
float p2 = round(s2 * (912./12.));  // Inner circumference travelled by the inner wheel in steps
float p1 = round(s1 * (912./12.));  // Outer circumference travelled by the outer wheel in steps

float speed = 800;                  // Speed of outer wheel

float new_speed1 = speed;           // Speed of outer wheel
float new_speed2 = speed * (p2/p1); // Speed of inner wheel, proportional to the circuference


if (direction == 1) { // Clockwise rotation
  stepperLeft.move(p1);
  stepperRight.move(p2);
  stepperLeft.setMaxSpeed(new_speed1);
  stepperRight.setMaxSpeed(new_speed2);
}

else if (direction == 0){  // Counter-clockwise rotation
  stepperLeft.move(p2);
  stepperRight.move(p1);
  stepperLeft.setMaxSpeed(new_speed2);
  stepperRight.setMaxSpeed(new_speed1);
}
steppers.runSpeedToPosition();
runToStop();

}


/*
  Moves the robot a given distance in feet.
  It uses a factor of 912 pulses per feet. This was calculated with the diameter of the wheels and
  adjusted experimentally.
  @param distance distance to travel in feet.
*/
void forward(float distance) {
  distance = distance*912;            // We experimentally found that there are 912 pulses per feet
  stepperLeft.move(distance);         // Sets the distance to move for the left motor
  stepperRight.move(distance);        // Sets the distance to move for the right motor
  stepperLeft.setMaxSpeed(500);       // Sets the velocity to move for the left motor
  stepperRight.setMaxSpeed(500);      // Sets the velocity to move for the right motor
  steppers.runSpeedToPosition();      // Moves the motors at given speed
  runToStop();
}
/*
  Moves the robot a given distance in feet backwards.
  It uses a factor of 912 pulses per feet. This was calculated with the diameter of the wheels and
  adjusted experimentally.
  @param distance distance to travel in feet.
*/
void reverse(int distance) {
  distance = -distance*912;           // 912 is the number of pulses per feet
  stepperLeft.moveTo(distance);
  stepperRight.moveTo(distance);
  steppers.runSpeedToPosition();
  runToStop();
}
/*
  It stops both motors.
*/
void stop() {
  stepperLeft.stop();
  stepperRight.stop();
}


/*
  This function makes the robot move in a circle of a given diameter in a given direction. It turns the red LED on.
  The diameter is the inner diameter.
  It uses the turn() funtion with 1 spin.

  @param diam diameter of the circle in inches
  @param dir  direction of turning (0 is CW, 1 is CCW)
*/
void moveCircle(int diam, int dir) {
  digitalWrite(redLED, HIGH); // Turns red LED on
  turn(dir,1,diam);           // Calls the turn function for one revolution with given diameter and direction
  digitalWrite(redLED, LOW);  // Turns red LED off

}

/*
  The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
  It turns teh red and yellow LEDs on.

  @param diam diameter of each of the circles of the figure 8 in inches.
*/
void moveFigure8(int diam) {
  digitalWrite(redLED, HIGH);  // Turns red LED on
  digitalWrite(ylwLED, HIGH);  // Turns yellow LED on
  moveCircle(diam, 1);         // Calls the circle function in the clockwise direction with the given diameter
  moveCircle(diam, 0);         // Calls the circle function in the counter-clockwise direction with the given diameter
  digitalWrite(redLED, LOW);   // Turns red LED off
  digitalWrite(ylwLED, LOW);   // Turns yellow LED off

}

/*
  It makes the robot spin to point about  in a given direction.
  It uses the encoders to reduce the odometry errors.
  It implements a proportional controller that recalculates the error discretely.

  @param angle The goal angle in degrees
  @param dir the direction to rotate 0 is CW, 1 is CCW
*/
void goToAngle(float angle, int dir) {
   double goalAngle = angle*100./360.;  // The 100/360 is the number of pulses per degree (100 pulses for a full revolution)

  /*
  //This chunk of code attempts to move the motors only if the encoders have not reached the goal yet.\
  
  if (dir == 1) {     //go clockwise
  // Serial.print(goalAngle);
    while(encoder[0] < goalAngle) { // Run until you reach the goal angle
      float errorL = goalAngle - encoder[0]+2;
      float errorR = goalAngle - encoder[1]+2;
      stepperLeft.setSpeed(500);  // Make speed proportional to distance to goal
      stepperRight.setSpeed(-500);
      stepperLeft.move(errorL);  // Move to half the distance remaining and reevaluate error
      stepperRight.move(errorR);
      steppers.run();
      Serial.print("moving robot");

    }//end while
  } //end if
  
  */ 

  
  //This code moves to the desired position and then calculates the error form the goal using encoders.

  if (dir == 0) {                           // Move clockwise
    pivot(dir, angle/360.);                 // Pivots in the given direction, the number of spins given
    float errorL = goalAngle - encoder[0];  // Calculates teh error in the left encoder
    if (errorL > 0.1*goalAngle) {           // If the error is more than 10% off from the goal, correct the motor
      stepperLeft.move(errorL);             // move the motor by the offset from the goal
    }
  }
  
  else if (dir == 1) {                      // Move counter-clockwise
    pivot(dir, angle/360);                  // Pivots in the given direction, the number of spins given
    float errorR = goalAngle - encoder[1];  // Calculates teh error in the left encoder
    Serial.print(0.1*goalAngle);            // Prints how far from the goal it is
    if (errorR > 0.1*goalAngle) {           // If the error is more than 10% off from the goal, correct the motor
      digitalWrite(redLED, HIGH);           // Turn LED on to check if the encoder is correcting the distance
      stepperRight.move(errorR);
      digitalWrite(redLED, LOW);            // Turn LED off to check if the encoder is correcting it 
    }
  }

}
/*
This function takes the x and y coordinate and moves the robot to that location.
It uses the atan() command to calculate the angle and the Pythagoras theorem to calculate the distance to travel.
Uses the subfuntions goToAngle() and forward() defined above

@param    x   The x coordinate of the goal
@param    y   The y coordinate of the goal.
*/
void goToGoal(float x, float y) {
  float angle = atan2(y,x)*180/PI; // Calculate the angle it needs to point to
  
  int dir = 1;                    // Go counter-clockwise if the goal position is in the first or second quadrant
  if (y < 0) {                    // The goal position is in the third or forth quadrant
    angle = abs(angle);           // Make the angle positive
    dir = 1;                      // Go clockwise
  }

  
  float disToTravel = sqrt(x*x+y*y); // Calculates the distance to travel using the pythagoras theorem
  goToAngle(angle, dir);          // Uses the goToAngle to pivot to the appropiate direction
  forward(disToTravel);           // Moves the distance to the goal

}


/*
The robot moves in a square with a side of a given length.
It corrects for odometry error using the encoders
It uses the forward(), goToAngle() and correct 

**STILL IN PROGRESS**


@param    length length of each of the sides of the square in feet
*/
void makeSquare(int length) {
  int length_ticksL = length*45;              // Calculate the number of encoder ticks on left motor to move the distance (45ticks/feet)
  int length_ticksR = length*46;              // Calculate the number of encoder ticks on right motor to move the distance (46ticks/feet)
  forward(length);                            // Move forward the given ammount using forward()
  float errorL = length_ticksL - encoder[0];  // Error in the left encoder from goal
  float errorR = length_ticksR - encoder[1];  // Error in the right encoder from goal
  correctForwardEncoders(length_ticksL, length_ticksR, errorL, errorR);
  goToAngle(90,1);

  forward(length);
  errorL = length_ticksL - encoder[0];
  correctForwardEncoders();
  goToAngle(90,1);

  forward(length);
  correctForwardEncoders();
  goToAngle(90,1);

  forward(length);
  correctForwardEncoders();
  goToAngle(90,1);

}

void correctForwardEncoders(int length_ticksL, int length_ticksR, float errorL, float errorR) {
  if (errorL > 0.1*length_ticksL) {
    stepperLeft.move(errorL);
    stepperLeft.setMaxSpeed(300);
  }
  if (errorR > 0.1*length_ticksR) {
    stepperRight.move(errorR);
    stepperRight.setMaxSpeed(300);
  }
  steppers.runSpeedToPosition();
  encoder[0] = 0;
  encoder[1] = 0;
}



//// MAIN
void setup()
{
  int baudrate = 9600; //serial monitor baud rate'
  int BTbaud = 9600;  // HC-05 default speed in AT command more
  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder

  BTSerial.begin(BTbaud);     //start Bluetooth communication
  Serial.begin(baudrate);     //start serial monitor communication

  while (!Serial)
    delay(10); // will pause until serial console opens
  
 // init_BT(); //initialize Bluetooth

  init_IMU(); //initialize IMU
  
  Serial.println("Robot starting...");
  Serial.println("");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
}


void loop()
{
  //uncomment each function one at a time to see what the code does
  // moveCircle(36, 1);
  // delay(2000);
  // moveFigure8(36);
  // delay(2000);
  // goToAngle(60,1);
  // delay(5000);
  // goToAngle(135,0);
  // delay(5000);
  // goToGoal(0,3);

  // move1();//call move back and forth function
  // move2();//call move back and forth function with AccelStepper library functions
  //move3();//call move back and forth function with MultiStepper library functions
  //move4(); //move to target position with 2 different speeds
  move5(); //move continuously with 2 different speeds

  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  //print_encoder_data();   //prints encoder data   [45 ticks = 1 Left revolution, 46 ticks = 1 Right revolution]
  

  //Uncomment to read IMU Data (uncomment to read on serial monitor)
  print_IMU_data();         //print IMU data

  //Uncomment to Send and Receive with Bluetooth
  //Bluetooth_comm();

  delay(3*wait_time);               //wait to move robot or read data
}