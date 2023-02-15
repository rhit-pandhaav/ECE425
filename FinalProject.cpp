/*Lab3
  Author: Advait Pandharkar & Alejandro Marcenido
  Date: January 21st 2023
  This program uses bahavior based architechture for a mobile robot to implement wall following,
  random wander and hallway followin. It uses bang-bang, proportional and PD control.
  It uses a state machine to switch between the differet states.

  The flag byte (8 bits) variable will hold the IR and sonar data [X X snrRight snrLeft irLeft irRight irRear irFront]
  The state byte (8 bits) variable will hold the state information as well as motor motion [X X X wander runAway collide rev fwd]

  Hardware Connections:
  Stepper Enable          Pin 48
  Right Stepper Step      Pin 50
  Right Stepper Direction Pin 51
  Left Stepper Step       Pin 52
  Left Stepper Direction  Pin 53

  Front IR    A2
  Back IR     A0
  Right IR    A3
  Left IR     A1
  Left Sonar  A4
  Right Sonar A5
  Button      A15
*/

#include <AccelStepper.h>      //include the stepper motor library
#include <Adafruit_MPU6050.h>  //Include library for MPU6050 IMU
#include <Arduino.h>           //include for PlatformIO Ide
#include <MultiStepper.h>      //include multiple stepper motor library
#include <NewPing.h>           //Includes the NewPing library
#include <SoftwareSerial.h>    //include Bluetooth module
#include <TimerOne.h>          //Includes the TimerOne library

// state LEDs connections
#define redLED 6      // red LED for displaying states
#define grnLED 7      // green LED for displaying states
#define ylwLED 8      // yellow LED for displaying states
#define enableLED 13  // stepper enabled LED

#define stepperEnable 48  // stepper enable pin on stepStick
#define rtStepPin 50      // right stepper motor step pin
#define rtDirPin 51       // right stepper motor direction pin
#define ltStepPin 52      // left stepper motor step pin
#define ltDirPin 53       // left stepper motor direction pin

// define sensor pin numbers
#define irFront A2   // front IR analog pin
#define irRear A0    // back IR analog pin
#define irRight A3   // right IR analog pin
#define irLeft A1    // left IR analog pin
#define snrLeft A4   // front left sonar
#define snrRight A5  // front right sonar
#define button A15   // pushbutton

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);  // create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);   // create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                 // create instance to control multiple steppers at the same time
NewPing sonarLt(snrLeft, snrLeft);                                     // create an instance of the left sonar
NewPing sonarRt(snrRight, snrRight);                                   // create an instance of the right sonar

// define stepper motor constants
#define stepperEnTrue false   // variable for enabling stepper motor
#define stepperEnFalse true   // variable for disabling stepper motor
#define test_led 13           // test led to test interrupt heartbeat
#define enableLED 13          // stepper enabled LED
#define robot_spd 200         // set robot speed
#define max_accel 10000       // maximum robot acceleration
#define max_spd 1000          // maximum robot speed
#define quarter_rotation 100  // stepper quarter rotation
#define half_rotation 200     // stepper half rotation
#define one_rotation 400      // stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation 800      // stepper motor 2 rotations
#define three_rotation 1200   // stepper rotation 3 rotations
#define four_rotation 1600    // stepper rotation 4 rotations
#define five_rotation 2000    // stepper rotation 5 rotations

// define sensor constants and variables
#define irMin 4   // IR minimum threshold for wall (use a deadband of 4 to 6 inches)
#define irMax 6   // IR maximum threshold for wall (use a deadband of 4 to 6 inches)
#define snrMin 4  // sonar minimum threshold for wall (use a deadband of 4 to 6 inches)
#define snrMax 6  // sonar maximum threshold for wall (use a deadband of 4 to 6 inches)

int irFrontArray[5] = {0, 0, 0, 0, 0};  // array to hold 5 front IR readings
int irRearArray[5] = {0, 0, 0, 0, 0};   // array to hold 5 back IR readings
int irRightArray[5] = {0, 0, 0, 0, 0};  // array to hold 5 right IR readings
int irLeftArray[5] = {0, 0, 0, 0, 0};   // array to hold 5 left IR readings
int irFrontAvg;                         // variable to hold average of current front IR reading
int irLeftAvg;                          // variable to hold average of current left IR reading
int irRearAvg;                          // variable to hold average of current rear IR reading
int irRightAvg;                         // variable to hold average of current right IR reading
int irIdx = 0;                          // index for 5 IR readings to take the average
int srLeftArray[5] = {0, 0, 0, 0, 0};   // array to hold 5 left sonar readings
int srRightArray[5] = {0, 0, 0, 0, 0};  // array to hold 5 right sonar readings
int srIdx = 0;                          // index for 5 sonar readings to take the average
int srLeft;                             // variable to hold average of left sonar current reading
int srRight;                            // variable to hold average or right sonar current reading
int srLeftAvg;                          // variable to holde left sonar data
int srRightAvg;                         // variable to hold right sonar data

// STATE MACHINE TIMER INTERRUPT VARIABLES
volatile boolean test_state;  // variable to hold test led state for timer interrupt
#define timer_int 500000      // 1/2 second (500000 us) period for timer interrupt

// bit definitions for sensor data flag byte [rt_snr left_snr left_ir right_ir rear_ir front_ir]
volatile byte flag = 0;
#define obFront 0   // Front IR trip [used to detect front wall for corner]
#define obRear 1    // Rear IR trip
#define obRight 2   // Right IR trip
#define obLeft 3    // Left IR trip
#define obFLeft 4   // Left Sonar trip
#define obFRight 5  // Right Sonar trip

#define leftPR 6
#define rightPR 7

#define PR_Left A6   // Left photoresistor
#define PR_Right A7  // Right Photoresistor

// bit definitions for robot motion and state byte [follow_hallway follow_right folloleft wander avoid]
volatile byte state = 0;
#define avoid 0    // avoid behavior
#define wander 1   // wander behavior
#define fleft 2    // follow left wall behavior
#define fright 3   // follow right wall behavior
#define center 4   // follow hallway behavior
#define movingL 6  // robot left wheel moving
#define movingR 7  // robot right wheel moving

volatile byte state2 = 0;
#define love 0
#define fear 1
#define explore 2
#define aggressive 3

// define layers of subsumption architecture that are active [hallway Wall Wander Avoid]
byte layers = 4;
#define aLayer 0   // avoid obstacle layer
#define wLayer 1   // wander layer
#define fwLayer 2  // follow wall layer
#define fhLayer 3  // follow hallway layer

// define PD control global variables, curr_error = current reading - setpoint, prev_error = curr_error on previous iteration
// store previous error to calculate derror = curr_error-prev_error, side_derror = side front sensor - side back sensor
// store derror = difference between left and right error (used for hall follow to center the robot)

float ls_curr;  // left sonar current reading
float li_curr;  // left ir current reading
float rs_curr;  // right sonar current reading
float ri_curr;  // right ir current reading

float ls_cerror;  // left sonar current error
float li_cerror;  // left ir current error
float rs_cerror;  // right sonar current error
float ri_cerror;  // right ir current error

float ls_perror;  // left sonar previous error
float li_perror;  // left ir previous error
float rs_perror;  // right sonar previous error
float ri_perror;  // right ir previous error

float ls_derror;     // left sonar delta error
float li_derror;     // left ir delta error
float rs_derror;     // right sonar delta error
float ri_derror;     // right ir current error
float left_derror;   // difference between left front and back sensor, this may be useful for adjusting the turn angle
float right_derror;  // difference between right front and back sensor, this may be useful for adjusting the turn angle

float prev_left = 0;
float prev_right = 0;

float left_error;
float right_error;

float hallIR_left;
float hallIR_right;
float hallS_left;
float hallS_right;

float left_perror = 0;
float right_perror = 0;

float error;            // difference between left and right error to center robot in the hallway
float hall_perror = 0;  // previous error for hallway
float derror;           // diffrence betwen error and previous error in hallway

float Left_light;   // Left PR values of light
float right_light;  // Right PR values for light

float left_thres;
float right_thres;

#define baud_rate 9600  // set serial communication baud rate

// GUI Set-up
SoftwareSerial BTSerial(11, 10);  // RX | TX

// Message from MATLAB
String message = "";
bool stringComplete = false;
volatile byte msgState;
#define btForwards 0
#define btBackwards 1
#define btLeft 2
#define btRight 3
#define rest 4

/*This function, runToStop(), will run the robot until the target is achieved and
  then stop it
*/
void runToStop(void) {
  int runNow = 1;
  while (runNow) {
    if (!stepperRight.run()) {
      bitClear(state, movingR);  // clear bit for right motor moving
      stepperRight.stop();       // stop right motor
    }
    if (!stepperLeft.run()) {
      bitClear(state, movingL);  // clear bit for left motor moving
      stepperLeft.stop();        // stop left motor
    }                            //
    if (!bitRead(state, movingR) & !bitRead(state, movingL))
      runNow = 0;
  }
}

/*robot move forward function */
void forward(int rot) {
  stepperLeft.setMaxSpeed(robot_spd);
  stepperRight.setMaxSpeed(robot_spd);
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   // reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    // reset left motor to position 0
  positions[0] = stepperRight.currentPosition() + rot;  // right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot;   // left motor absolute position

  stepperRight.move(positions[0]);  // move right motor to position
  stepperLeft.move(positions[1]);   // move left motor to position
  bitSet(state, movingL);           // move left wheel
  bitSet(state, movingR);           // move right wheel
  runToStop();                      // run until the robot reaches the target
}

/*robot move reverse function */
void reverse(int rot) {
  stepperLeft.setMaxSpeed(robot_spd);
  stepperRight.setMaxSpeed(robot_spd);
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   // reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    // reset left motor to position 0
  positions[0] = stepperRight.currentPosition() - rot;  // right motor absolute position
  positions[1] = stepperLeft.currentPosition() - rot;   // left motor absolute position

  stepperRight.move(positions[0]);  // move right motor to position
  stepperLeft.move(positions[1]);   // move left motor to position
  bitSet(state, movingL);           // move left wheel
  bitSet(state, movingR);           // move right wheel
  runToStop();                      // run until the robot reaches the target
}

/*robot pivot function */
void pivot(int rot, int dir) {
  stepperLeft.setMaxSpeed(robot_spd);
  stepperRight.setMaxSpeed(robot_spd);
  long positions[2];                                     // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                    // reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                     // reset left motor to position 0
  if (dir > 0) {                                         // pivot right
    positions[0] = stepperRight.currentPosition();       // right motor absolute position
    positions[1] = stepperLeft.currentPosition() + rot;  // left motor absolute position
  } else                                                 // pivot left
  {
    positions[0] = stepperRight.currentPosition() + rot + 10;  // right motor absolute position
    positions[1] = stepperLeft.currentPosition();              // left motor absolute position
  }
  stepperRight.move(positions[0]);  // move right motor to position
  stepperLeft.move(positions[1]);   // move left motor to position
  bitSet(state, movingL);           // move left wheel
  bitSet(state, movingR);           // move right wheel
  runToStop();                      // run until the robot reaches the target
}

/*robot spin function */
void spin(int rot, int dir) {
  stepperLeft.setMaxSpeed(robot_spd * 3);
  stepperRight.setMaxSpeed(robot_spd * 3);
  long positions[2];                                      // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                     // reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                      // reset left motor to position 0
  if (dir > 0) {                                          // spin right
    positions[0] = stepperRight.currentPosition() - rot;  // right motor absolute position
    positions[1] = stepperLeft.currentPosition() + rot;   // left motor absolute position
  } else                                                  // spin left
  {
    positions[0] = stepperRight.currentPosition() + rot;  // right motor absolute position
    positions[1] = stepperLeft.currentPosition() - rot;   // left motor absolute position
  }
  stepperRight.move(positions[0]);  // move right motor to position
  stepperLeft.move(positions[1]);   // move left motor to position
  bitSet(state, movingL);           // move left wheel
  bitSet(state, movingR);           // move right wheel
  runToStop();                      // run until the robot reaches the target
}

/*robot stop function */
void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

/*
  wallBang() function. This function will follow the wall and stay in a range of 4-6inches (dead zone).
  It will turn by a fixed ammount when it get out of the dead zone either left or right to correct its position.
  It does not take into account the distance from the wall
*/

void PRRead() {
  float left;
  float right;
  left = analogRead(PR_Left);
  right = analogRead(PR_Right);

  // Serial.print("Left: "); Serial.print(left); Serial.print(" Right: "); Serial.println(right);
}

void wallBang() {
  if (bitRead(state, fright)) {
    if (bitRead(flag, obFront)) {  // check for a front wall before moving
      // make left turn if wall found
      reverse(one_rotation);    // back up
      spin(three_rotation, 0);  // turn left
    }
    if (ri_cerror == 0) {       // no error, robot in deadband
      forward(three_rotation);  // move robot forward
    } else {
      if (ri_cerror < 0) {           // negative error means too close
        pivot(quarter_rotation, 1);  // pivot left
        pivot(quarter_rotation, 0);  // pivot right to straighten up
        digitalWrite(ylwLED, HIGH);
      } else if (ri_cerror > 0) {    // positive error means too far
        pivot(quarter_rotation, 1);  // pivot right
        pivot(quarter_rotation, 0);  // pivot left to straighten up
        digitalWrite(redLED, HIGH);
      }
    }
  }

  else if (bitRead(state, fleft)) {
    if (bitRead(flag, obFront)) {  // check for a front wall before moving forward
      // make right turn if wall found
      reverse(one_rotation);    // back up
      spin(three_rotation, 1);  // turn right
    }
    if (li_cerror == 0) {     // no error robot in dead band drives forward
      forward(two_rotation);  // move robot forward
    } else {
      if (li_cerror < 0) {           // negative error means too close
        pivot(quarter_rotation, 1);  // pivot right
        pivot(quarter_rotation, 0);  // pivot left
        digitalWrite(ylwLED, HIGH);

      } else if (li_cerror > 0) {    // positive error means too far
        pivot(quarter_rotation, 1);  // pivot left
        pivot(quarter_rotation, 0);  // pivot right
        digitalWrite(redLED, HIGH);
      }
    }
  } else if (bitRead(state, center)) {  // follow hallway

    if (((ri_cerror == 0) && (li_cerror == 0)) || (derror == 0)) {
      forward(two_rotation);  // drive robot forward
    } else {
      // average the error between the left and right to center the robot
      if (derror > 0) {
        spin(quarter_rotation, 1);   // spin right, the left error is larger
        pivot(quarter_rotation, 0);  // pivot left to adjust forward
      } else {
        spin(quarter_rotation, 0);   // spin left the right error is larger
        pivot(quarter_rotation, 1);  // pivot right to adjust forward
      }
    }
  } else if (bitRead(state, wander)) {
    stop();
    delay(500);
    pivot(half_rotation, 0);
    forward(one_rotation);
    pivot(quarter_rotation, 1);
  }
}

/*
  updateIR() function, it takes an average of 5 readings  from the IR sensors to make the measurements more accurate.
  It uses a function that we calculated in excel to normalize the data to inches.
  It also ignores any data that is negative.
  It stores the data for the four directions in two different variables.
    One called xi_cerror is capped. If it is between 4-6 inches it will make the error 0. This is for wall following.
    Another hall_left/hall_right that is not capped. This is for hallway following.
  It also calculates the change in error used for PD controller
*/
void updateIR() {
  // declare IR variables
  float front = 0;
  float back = 0;
  float left = 0;
  float right = 0;

  // Take the average of 5 readings of the sensors for an accurate result
  for (int k = 0; k < 4; k++) {
    front += analogRead(irFront);  // read front IR sensor
    back += analogRead(irRear);    // read back IR sensor
    left += analogRead(irLeft);    // read left IR sensor
    right += analogRead(irRight);  // read right IR sensor
  }
  front = front / 5;
  back = back / 5;
  left = left / 5;
  right = right / 5;

  // Calculate the distance in inches for the front, back, left, and right sensors
  front = (1111 / (front + 3) - 1);
  back = (1116 / (back + 6) - 1);
  left = (1447 / (left - 29) - 1);
  right = (1379 / (right - 59) - 1);

  float prev_left, prev_front, prev_back, prev_right;
  
  if(left > 40 || left < 0) {
    left = prev_left;
  }
  prev_left = left;

  if(right > 40 || right < 0) {
    right = prev_right;
  }
  prev_right = right;

  if(front > 40 || front < 0) {
    front = prev_front;
  }
  prev_front = front;

  if(back > 40 || back < 0) {
    back = prev_back;
  }
  prev_back = back;

  
  BTSerial.print(front); BTSerial.print(" ");
  BTSerial.print(left); BTSerial.print(" ");
  BTSerial.print(back); BTSerial.print(" ");
  BTSerial.print(right); BTSerial.println(" ");
  

  // Stores data for the hallway following that is NOT capped between 4-6inches
  hallIR_left = left;
  hallIR_right = right;

  // Ignore values that are negative, store the previous reading
  if (left > 0) {
    prev_left = left;
  } else {
    left = prev_left;
  }
  if (right > 0) {
    prev_right = right;
  } else {
    right = prev_right;
  }

  if (front < 10 && front > 0) {  // If an object is between 0 to 10 inches, set a flag on obFront bit
    digitalWrite(redLED, HIGH);   // turn on red LED
    digitalWrite(grnLED, LOW);
    bitSet(flag, obFront);
  } else {  // otherwise, clear the flag
    bitClear(flag, obFront);
    digitalWrite(redLED, LOW);
  }

  if (back < 10 && back > 0) {   // If an object is between 0 to 10 inches set a flag on obBack bit
    digitalWrite(redLED, HIGH);  //  turn on red LED,
    digitalWrite(grnLED, LOW);
    bitSet(flag, obRear);
  } else {  // otherwise, clear the flag
    bitClear(flag, obRear);
    digitalWrite(redLED, LOW);
  }

  if (left < 10 && left > 0) {   // If an object is between 0 to 10 inches, set a flag on obLeft bit
    digitalWrite(redLED, HIGH);  // turn on red LED
    digitalWrite(grnLED, LOW);
    bitSet(flag, obLeft);
  } else {  // otherwise, clear the flag
    bitClear(flag, obLeft);
    digitalWrite(redLED, LOW);
  }

  if (right < 10 && right > 0) {  // If an object is between 0 to 10 inches, set a flag on obRight bit
    digitalWrite(redLED, HIGH);   // turn on red LED
    digitalWrite(grnLED, LOW);
    bitSet(flag, obRight);
  } else {  // otherwise, clear the flag
    bitClear(flag, obRight);
    digitalWrite(redLED, LOW);
  }

  ri_curr = right;  // log current sensor reading [right IR]
  if ((ri_curr > 6)) {
    ri_cerror = ri_curr - irMax;  // calculate current error (too far positive, too close negative)
  } else if ((ri_curr < 4)) {
    ri_cerror = ri_curr - irMin;
  } else {
    ri_cerror = 0;  // set error to zero if robot is in dead band
  }

  ri_derror = ri_cerror - ri_perror;  // calculate change in error
  ri_perror = ri_cerror;              // log current error as previous error [left sonar]

  li_curr = left;  // log current sensor reading [left sonar]
  if ((li_curr > 6)) {
    li_cerror = li_curr - irMax;  // calculate current error
  } else if ((li_curr < 4)) {
    li_cerror = li_curr - irMin;
  } else {
    li_cerror = 0;  // error is zero if in deadband
  }

  li_derror = li_cerror - li_perror;  // calculate change in error
  li_perror = li_cerror;              // log reading as previous error
}

/*
  updateSonar() function.
  It uses a function that we calculated in excel to normalize the data to inches.
  It also ignores any data that is negative.
  It stores the data for the four directions in two different variables.
    One called xi_cerror is capped. If it is between 4-6 inches it will make the error 0. This is for wall following.
    Another hall_left/hall_right that is not capped. This is for hallway following.
  It also calculates the change in error used for PD controller
*/
void updateSonar() {
  // sonar variables
  long left = 0;
  long right = 0;
  pinMode(snrRight, OUTPUT);        // set the PING pin as an output, read right sensor
  digitalWrite(snrRight, LOW);      // set the PING pin low first
  delayMicroseconds(2);             // wait 2 us
  digitalWrite(snrRight, HIGH);     // trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);             // wait 5 us
  digitalWrite(snrRight, LOW);      // set pin low first again
  pinMode(snrRight, INPUT);         // set pin as input with duration as reception
  right = pulseIn(snrRight, HIGH);  // measures how long the pin is high

  pinMode(snrLeft, OUTPUT);       // set the PING pin as an output, read left sensor
  digitalWrite(snrLeft, LOW);     // set the PING pin low first
  delayMicroseconds(2);           // wait 2 us
  digitalWrite(snrLeft, HIGH);    // trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);           // wait 5 us
  digitalWrite(snrLeft, LOW);     // set pin low first again
  pinMode(snrLeft, INPUT);        // set pin as input with duration as reception
  left = pulseIn(snrLeft, HIGH);  // measures how long the pin is high

  left = (left - 78) / 157;     // Converts the value into inches
  right = (right - 240) / 160;  // Converts the value into inches

  // Right and left sensor values for hallway follow
  hallS_right = right;
  hallS_left = left;

  if (left < 10 && left > 0) {  // If an object is between 0 to 10 inches of the left sensor, set a flag on obLeft bit and turn on red LED, otherwise, clear the flag
    bitSet(flag, obFLeft);
  } else {
    bitClear(flag, obFLeft);
  }

  if (right < 10 && right > 0) {  // If an object is between 0 to 10 inches of the right sensor, set a flag on obRight bit and turn on red LED, otherwise, clear the flag
    bitSet(flag, obFRight);
  } else {
    bitClear(flag, obFRight);
  }

  ///////////////////////update variables//////////////////
  rs_curr = right;  // log current sensor reading [right sonar]

  if ((rs_curr > snrMax) || (rs_curr < snrMin))
    rs_cerror = rs_curr - snrMax;  // calculate current error (too far positive, too close negative)
  // hallS_right = rs_curr - snrMax;
  else
    rs_cerror = 0;                    // set error to zero if robot is in dead band
  rs_derror = rs_cerror - rs_perror;  // calculate change in error
  rs_perror = rs_cerror;              // log current error as previous error [left sonar]

  ls_curr = left;  // log current sensor reading [left sonar]
  if ((ls_curr > snrMax) || (ls_curr < snrMin))
    ls_cerror = ls_curr - snrMax;  // calculate current error
  else
    ls_cerror = 0;                    // error is zero if in deadband
  ls_derror = ls_cerror - ls_perror;  // calculate change in error
  ls_perror = ls_cerror;              // log reading as previous error
}

/*
   This function will update all of the error constants to be used for P, PD control and hallwayFollowing
   store previous error to calculate derror = curr_sensor-prev_sensor, side_derror = side front sensor - side back sensor
*/
void updateError() {
  float hall_left;   // Left distance when inside hallway
  float hall_right;  // Right distance when inside hallway

  // Take the smallest reading between the IR and the sonar for left for wall following
  if (ri_cerror < rs_cerror) {
    right_error = ri_cerror;
  } else {
    right_error = rs_cerror;
  }

  // Take the smallest reading between the IR and the sonar for left for wall following
  if (li_cerror < ls_cerror) {
    left_error = li_cerror;
  } else {
    left_error = ls_cerror;
  }
  // Take the smallest reading between the IR and the sonar for left for hallway following
  if (hallIR_left < hallS_left) {
    hall_left = hallIR_left;
  } else {
    hall_left = hallS_left;
  }
  // Take the smallest reading between the IR and the sonar for left for hallway following
  if (hallIR_right < hallS_right) {
    hall_right = hallIR_right;
  } else {
    hall_right = hallS_right;
  }

  left_derror = left_error - left_perror;
  right_derror = right_error - right_perror;
  left_perror = left_error;
  right_perror = right_error;

  // Hallway error
  error = hall_left - hall_right;  // use distance from left and right to calculate the error
  derror = error - hall_perror;    // Calculates the change in error from previous reading
  hall_perror = error;             // Store previous error
}

/*
   updateState() function, updates which state it is on depending on the flags set by the sensors.
   It uses the sonars and sensors in a redundant way to get a more robust reading of where the obstacles are.
*/
void updateState() {
  if ((!bitRead(flag, obRight) && !bitRead(flag, obLeft))) {  // no sensors triggered
    bitSet(state, wander);                                    // set the wander state
    // clear all other bits
    bitClear(state, fright);                                                                                                // clear follow wall state
    bitClear(state, fleft);                                                                                                 // clear follow wall state
    bitClear(state, center);                                                                                                // clear follow wall state
  } else if ((bitRead(flag, obRight) || bitRead(flag, obFRight)) && (!bitRead(flag, obLeft) && !bitRead(flag, obFLeft))) {  // Follow Right wall
    bitSet(state, fright);                                                                                                  // set RIGHT WALL state
    // clear all other bits
    bitClear(state, wander);  // clear wander state
    bitClear(state, fleft);   // clear follow wall state
    bitClear(state, center);  // clear follow wall state
  }
  // The if statement below checks if there is something on the left sensors (IR and sonar) and there is nothing in the right sensors
  else if ((bitRead(flag, obLeft) || bitRead(flag, obFLeft)) && (!bitRead(flag, obRight) && !bitRead(flag, obFRight))) {  // Follow Left wall
    bitSet(state, fleft);                                                                                                 // set left wall state
    // clear all other bits
    bitClear(state, fright);  // clear follow wall state
    bitClear(state, wander);  // clear wander state
    bitClear(state, center);  // clear follow wall state

  }
  // The if statement below checks if there is something on the right sensors (IR and sonar) and there is nothing in the left sensors
  else if ((bitRead(flag, obLeft) && bitRead(flag, obRight)) || (bitRead(flag, obFLeft) && bitRead(flag, obFRight))) {  // Follow right
    bitSet(state, center);                                                                                              // set the hallway state
    // clear all other bits
    bitClear(state, fright);  // clear follow wall state
    bitClear(state, wander);  // clear wander state
    bitClear(state, fleft);   // clear follow wall state
  }
}

/*
  updateSensors() function, it calls all the update functions.
  It resets all the falgs and states
*/
void updateSensors() {
  test_state = !test_state;            // LED to test the heartbeat of the timer interrupt routine
  digitalWrite(test_led, test_state);  // flash the timer interrupt LED
  flag = 0;                            // clear all sensor flags
  state = 0;                           // clear all state flags
  updateIR();                          // update IR readings and update flag variable and state machine
  updateSonar();                       // update Sonar readings and update flag variable and state machine
  updateError();                       // update sensor current, previous, change in error
  updateState();                       // update State Machine based upon sensor readings
}

/*
  moveRobot() function, it moves the robot based on the state that it is in.
  When it is inside of a state it still uses the flags to take actions inside that behavior
  It also turns the LEDs according to the state it is in.
  It uses PD control for wallFollow and hallwayFollow. The turns are proportional to the error and cahnge in error
*/
void moveRobot() {
  // Here is where we add the PD controller for wall following
  // Kp = one_rotation/3
  // Kd = one_rotation/10
  float left_rotation = one_rotation * (abs(left_error) / 3 - abs(left_derror) / 10);
  float right_rotation = one_rotation * (abs(right_error) / 3 - abs(right_derror) / 10);
  digitalWrite(redLED, LOW);
  digitalWrite(grnLED, LOW);
  digitalWrite(ylwLED, LOW);

  // if (bitRead(state, fright)) {
  //   digitalWrite(redLED, HIGH);
  //   digitalWrite(ylwLED, HIGH);
  //   if (bitRead(flag, obRight) && !bitRead(flag, obFRight)) {
  //     forward(two_rotation);
  //     spin(three_rotation * 4, 1);
  //     forward(two_rotation);
  //   }
  //   if (bitRead(flag, obFront)) {  // check for a front wall before moving
  //     // make left turn if wall found
  //     digitalWrite(redLED, HIGH);
  //     digitalWrite(grnLED, HIGH);
  //     digitalWrite(ylwLED, LOW);
  //     reverse(two_rotation);        // back up
  //     spin(three_rotation * 4, 0);  // turn left
  //     spin(three_rotation * 4, 0);  // turn left
  //   }
  //   if (right_error == 0) {     // no error, robot in deadband
  //     forward(three_rotation);  // move robot forward
  //     digitalWrite(redLED, LOW);
  //     digitalWrite(ylwLED, LOW);
  //     digitalWrite(grnLED, LOW);
  //   } else {
  //     if (right_error < 0) {  // negative error means too close
  //       digitalWrite(ylwLED, HIGH);
  //       digitalWrite(redLED, LOW);
  //       pivot(right_rotation, 0);  // pivot left
  //       forward(one_rotation / 10);
  //       pivot(right_rotation, 1);    // pivot right to straighten up
  //     } else if (right_error > 0) {  // positive error means too far
  //       digitalWrite(redLED, HIGH);
  //       digitalWrite(ylwLED, LOW);
  //       pivot(right_rotation, 1);  // pivot right
  //       forward(one_rotation / 10);
  //       pivot(right_rotation, 0);  // pivot left to straighten up
  //     }
  //   }
  // } else if (bitRead(state, fleft)) {  // Follow left wall
  //   digitalWrite(ylwLED, HIGH);
  //   digitalWrite(grnLED, HIGH);
  //   if (bitRead(flag, obLeft) && !bitRead(flag, obFLeft)) {  // Create new state for outside wall and implement this code there
  //     forward(two_rotation);
  //     spin(three_rotation * 4, 0);  // Incease the spins
  //     forward(two_rotation);
  //   }
  //   if (bitRead(flag, obFront)) {  // check for a front wall before moving forward
  //     // Back up and turn 90 degrees right
  //     digitalWrite(redLED, HIGH);
  //     digitalWrite(grnLED, HIGH);
  //     digitalWrite(ylwLED, LOW);
  //     reverse(two_rotation);        // back up
  //     spin(three_rotation * 4, 1);  // turn right
  //     spin(three_rotation * 4, 1);  // turn right
  //   }
  //   if (left_error == 0) {    // no error robot in dead band drives forward
  //     forward(two_rotation);  // move robot forward
  //   } else {
  //     if (left_error < 0) {  // negative error means too close
  //       digitalWrite(ylwLED, HIGH);
  //       digitalWrite(redLED, LOW);
  //       pivot(left_rotation, 1);     // pivot right
  //       forward(one_rotation / 10);  // Move forwards
  //       pivot(left_rotation, 0);     // pivot left
  //     } else if (left_error > 0) {   // positive error means too far
  //       digitalWrite(redLED, HIGH);
  //       digitalWrite(ylwLED, LOW);
  //       pivot(left_rotation, 0);  // pivot left
  //       forward(one_rotation / 10);
  //       pivot(left_rotation, 1);  // pivot right
  //     }
  //   }
  if (bitRead(state, center)) {  // follow hallway
    digitalWrite(grnLED,HIGH);
    if (bitRead(flag, obFront)) {       // check for a front wall before moving forward
      // Turn 180 degrees left
      spin(three_rotation * 4, 0);
      spin(three_rotation * 4, 0);
      spin(three_rotation * 4, 0);
      spin(three_rotation * 4, 0);
      forward(half_rotation);  // Move forward
    }

    // Here is where we add the PD controller.
    // Kp = one_rotation/3
    // Kd = one_rotation/5
    float hallway_rotation = one_rotation * (abs(error) / 3 - abs(derror) / 5);

    if (((error < 2 && error > -2))) {
      forward(one_rotation);  // drive robot forward
    } else {
      // Use the error between the left and right to center the robot
      if (error > 0) {
        pivot(hallway_rotation, 0);  // spin right, the left error is larger
        forward(half_rotation / 5);
        pivot(hallway_rotation, 1);  // pivot left to adjust forward
      } else {
        pivot(hallway_rotation, 1);  // spin left the right error is larger
        forward(half_rotation / 5);
        pivot(hallway_rotation, 0);  // pivot right to adjust forward
      }
    }
  } else if (bitRead(state, wander)) {  // Random wander
    if (bitRead(flag, obFront)) {       // check for a front wall before wondering
      spin(three_rotation * 4, 0);      // Turn 90 degrees right
      spin(three_rotation * 4, 0);
    }
    stop();
    delay(500);
    digitalWrite(grnLED, HIGH);  // Turn green LED on
    digitalWrite(redLED, LOW);   // Turn red LED off
    digitalWrite(ylwLED, LOW);   // Turn yellow LED off
    pivot(half_rotation, 0);     // Pivot right
    forward(one_rotation);       // Go forward
    pivot(quarter_rotation, 1);  // Pivot left
  }
}

void light_follow() {  // Do Love (motor near the light moves slower) // explorer(motor not close to the light moves slower)
  bitSet(state2, love);

  float left_speed = robot_spd;
  float right_speed = robot_spd;
  float left_thres = 750;
  float right_thres = 850;
  digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, LOW);
  digitalWrite(grnLED, LOW);
  float left = analogRead(PR_Left);
  float right = analogRead(PR_Right);

  Serial.print("Left: ");
  Serial.print(left + 100);
  Serial.print(" Right: ");
  Serial.println(right);

  if (bitRead(state2, fear)) {
    Serial.print("Left: ");
    Serial.print(left + 100);
    Serial.print(" Right: ");
    Serial.println(right);
    Serial.println(state2, BIN);
    if ((left > left_thres || right > right_thres)) {  // Light detected
      if (left + 100 > right) {
        left_speed = left_speed * ((left + 100) / 500);

        stepperLeft.setMaxSpeed(left_speed);
        stepperRight.setMaxSpeed(right_speed);
        long positions[2];                                                                   // Array of desired stepper positions
        stepperRight.setCurrentPosition(0);                                                  // reset right motor to position 0
        stepperLeft.setCurrentPosition(0);                                                   // reset left motor to position 0
        positions[0] = stepperRight.currentPosition() + one_rotation;                        // right motor absolute position
        positions[1] = stepperLeft.currentPosition() + one_rotation * ((left + 100) / 500);  // left motor absolute position

        stepperRight.move(positions[0]);  // move right motor to position
        stepperLeft.move(positions[1]);   // move left motor to position
        bitSet(state, movingL);           // move left wheel
        bitSet(state, movingR);           // move right wheel
        runToStop();                      // run until the robot reaches the target
      } else {
        right_speed = right_speed * (right / 500);

        stepperLeft.setMaxSpeed(left_speed);
        stepperRight.setMaxSpeed(right_speed);
        long positions[2];                                                             // Array of desired stepper positions
        stepperRight.setCurrentPosition(0);                                            // reset right motor to position 0
        stepperLeft.setCurrentPosition(0);                                             // reset left motor to position 0
        positions[0] = stepperRight.currentPosition() + one_rotation * (right / 500);  // right motor absolute position
        positions[1] = stepperLeft.currentPosition() + one_rotation;                   // left motor absolute position

        stepperRight.move(positions[0]);  // move right motor to position
        stepperLeft.move(positions[1]);   // move left motor to position
        bitSet(state, movingL);           // move left wheel
        bitSet(state, movingR);           // move right wheel
        runToStop();                      // run until the robot reaches the target
      }
    }
  }

  if (bitRead(state2, aggressive)) {
    Serial.print("Left: ");
    Serial.print(left + 100);
    Serial.print(" Right: ");
    Serial.println(right);
    Serial.println(state2, BIN);
    if ((left > left_thres || right > right_thres)) {  // Light detected
      if (left + 100 > right) {
        right_speed = right_speed * ((left + 100) / 500);

        stepperLeft.setMaxSpeed(left_speed);
        stepperRight.setMaxSpeed(right_speed);
        long positions[2];                                                                    // Array of desired stepper positions
        stepperRight.setCurrentPosition(0);                                                   // reset right motor to position 0
        stepperLeft.setCurrentPosition(0);                                                    // reset left motor to position 0
        positions[0] = stepperRight.currentPosition() + one_rotation * ((left + 100) / 500);  // right motor absolute position
        positions[1] = stepperLeft.currentPosition() + one_rotation;                          // left motor absolute position

        stepperRight.move(positions[0]);  // move right motor to position
        stepperLeft.move(positions[1]);   // move left motor to position
        bitSet(state, movingL);           // move left wheel
        bitSet(state, movingR);           // move right wheel
        runToStop();                      // run until the robot reaches the target
      } else {
        left_speed = left_speed * (right / 500);

        stepperLeft.setMaxSpeed(left_speed);
        stepperRight.setMaxSpeed(right_speed);
        long positions[2];                                                            // Array of desired stepper positions
        stepperRight.setCurrentPosition(0);                                           // reset right motor to position 0
        stepperLeft.setCurrentPosition(0);                                            // reset left motor to position 0
        positions[0] = stepperRight.currentPosition() + one_rotation;                 // right motor absolute position
        positions[1] = stepperLeft.currentPosition() + one_rotation * (right / 500);  // left motor absolute position

        stepperRight.move(positions[0]);  // move right motor to position
        stepperLeft.move(positions[1]);   // move left motor to position
        bitSet(state, movingL);           // move left wheel
        bitSet(state, movingR);           // move right wheel
        runToStop();                      // run until the robot reaches the target
      }
    }
  }

  if (bitRead(state2, love)) {
    Serial.print("Left: ");
    Serial.print(left + 100);
    Serial.print(" Right: ");
    Serial.println(right);
    Serial.println(state2, BIN);
    if ((left > left_thres || right > right_thres)) {  // Light detected
      if (left + 100 > right) {
        left_speed = left_speed / ((left + 100) / 500);

        stepperLeft.setMaxSpeed(left_speed);
        stepperRight.setMaxSpeed(right_speed);
        long positions[2];                                                                   // Array of desired stepper positions
        stepperRight.setCurrentPosition(0);                                                  // reset right motor to position 0
        stepperLeft.setCurrentPosition(0);                                                   // reset left motor to position 0
        positions[0] = stepperRight.currentPosition() + one_rotation;                        // right motor absolute position
        positions[1] = stepperLeft.currentPosition() + one_rotation / ((left + 100) / 500);  // left motor absolute position

        stepperRight.move(positions[0]);  // move right motor to position
        stepperLeft.move(positions[1]);   // move left motor to position
        bitSet(state, movingL);           // move left wheel
        bitSet(state, movingR);           // move right wheel
        runToStop();                      // run until the robot reaches the target
      } else {
        right_speed = right_speed / (right / 500);

        stepperLeft.setMaxSpeed(left_speed);
        stepperRight.setMaxSpeed(right_speed);
        long positions[2];                                                             // Array of desired stepper positions
        stepperRight.setCurrentPosition(0);                                            // reset right motor to position 0
        stepperLeft.setCurrentPosition(0);                                             // reset left motor to position 0
        positions[0] = stepperRight.currentPosition() + one_rotation / (right / 500);  // right motor absolute position
        positions[1] = stepperLeft.currentPosition() + one_rotation;                   // left motor absolute position

        stepperRight.move(positions[0]);  // move right motor to position
        stepperLeft.move(positions[1]);   // move left motor to position
        bitSet(state, movingL);           // move left wheel
        bitSet(state, movingR);           // move right wheel
        runToStop();                      // run until the robot reaches the target
      }
    }
  }

  if (bitRead(state2, explore)) {
    Serial.print("Left: ");
    Serial.print(left + 100);
    Serial.print(" Right: ");
    Serial.println(right);
    Serial.println(state2, BIN);
    if ((left > left_thres || right > right_thres)) {  // Light detected
      if (left + 100 > right) {
        right_speed = right_speed / ((left + 100) / 500);
        stepperLeft.setMaxSpeed(left_speed);
        stepperRight.setMaxSpeed(right_speed);
        long positions[2];                                                                    // Array of desired stepper positions
        stepperRight.setCurrentPosition(0);                                                   // reset right motor to position 0
        stepperLeft.setCurrentPosition(0);                                                    // reset left motor to position 0
        positions[0] = stepperRight.currentPosition() + one_rotation / ((left + 100) / 500);  // right motor absolute position
        positions[1] = stepperLeft.currentPosition() + one_rotation;                          // left motor absolute position

        stepperRight.move(positions[0]);  // move right motor to position
        stepperLeft.move(positions[1]);   // move left motor to position
        bitSet(state, movingL);           // move left wheel
        bitSet(state, movingR);           // move right wheel
        runToStop();                      // run until the robot reaches the target
      } else {
        left_speed = left_speed / (right / 500);

        stepperLeft.setMaxSpeed(left_speed);
        stepperRight.setMaxSpeed(right_speed);
        long positions[2];                                                            // Array of desired stepper positions
        stepperRight.setCurrentPosition(0);                                           // reset right motor to position 0
        stepperLeft.setCurrentPosition(0);                                            // reset left motor to position 0
        positions[0] = stepperRight.currentPosition() + one_rotation;                 // right motor absolute position
        positions[1] = stepperLeft.currentPosition() + one_rotation / (right / 500);  // left motor absolute position

        stepperRight.move(positions[0]);  // move right motor to position
        stepperLeft.move(positions[1]);   // move left motor to position
        bitSet(state, movingL);           // move left wheel
        bitSet(state, movingR);           // move right wheel
        runToStop();                      // run until the robot reaches the target
      }
    }
  }
}

void Light_track() {
  float left_speed = robot_spd;
  float right_speed = robot_spd;
  digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, LOW);
  digitalWrite(grnLED, LOW);
  float left = analogRead(PR_Left);
  float right = analogRead(PR_Right);

  Serial.println(state2, BIN);
  if ((left > left_thres || right > right_thres)) {  // Light detected
    if (bitRead(flag, obFront)) {                    // check for a front wall before wondering
      reverse(one_rotation);
      spin(three_rotation * 4, 0);  // Turn 90 degrees right
      spin(three_rotation * 4, 0);
    }

    if (left + 100 > right) {
      left_speed = left_speed / ((left + 100) / 500);

      stepperLeft.setMaxSpeed(left_speed);
      stepperRight.setMaxSpeed(right_speed);
      long positions[2];                                                                   // Array of desired stepper positions
      stepperRight.setCurrentPosition(0);                                                  // reset right motor to position 0
      stepperLeft.setCurrentPosition(0);                                                   // reset left motor to position 0
      positions[0] = stepperRight.currentPosition() + one_rotation;                        // right motor absolute position
      positions[1] = stepperLeft.currentPosition() + one_rotation / ((left + 100) / 500);  // left motor absolute position

      stepperRight.move(positions[0]);  // move right motor to position
      stepperLeft.move(positions[1]);   // move left motor to position
      bitSet(state, movingL);           // move left wheel
      bitSet(state, movingR);           // move right wheel
      runToStop();                      // run until the robot reaches the target
    } else {
      right_speed = right_speed / (right / 500);

      stepperLeft.setMaxSpeed(left_speed);
      stepperRight.setMaxSpeed(right_speed);
      long positions[2];                                                             // Array of desired stepper positions
      stepperRight.setCurrentPosition(0);                                            // reset right motor to position 0
      stepperLeft.setCurrentPosition(0);                                             // reset left motor to position 0
      positions[0] = stepperRight.currentPosition() + one_rotation / (right / 500);  // right motor absolute position
      positions[1] = stepperLeft.currentPosition() + one_rotation;                   // left motor absolute position

      stepperRight.move(positions[0]);  // move right motor to position
      stepperLeft.move(positions[1]);   // move left motor to position
      bitSet(state, movingL);           // move left wheel
      bitSet(state, movingR);           // move right wheel
      runToStop();                      // run until the robot reaches the target
    }
  } else {
    if (bitRead(flag, obFront)) {  // check for a front wall before wondering
      reverse(one_rotation);
      spin(three_rotation * 4, 0);  // Turn 90 degrees right
      spin(three_rotation * 4, 0);
    }
    stop();
    delay(500);
    digitalWrite(grnLED, HIGH);  // Turn green LED on
    digitalWrite(redLED, LOW);   // Turn red LED off
    digitalWrite(ylwLED, LOW);   // Turn yellow LED off
    pivot(half_rotation, 0);     // Pivot right
    forward(one_rotation);       // Go forward
    pivot(quarter_rotation, 1);  // Pivot left
  }
}

void GUItrack() {
  if (bitRead(msgState, btRight)) {
    spin(three_rotation * 4, 1);  // Turn 90 degrees right
    spin(three_rotation * 4, 1);
    forward(1);
  } else if (bitRead(msgState, btLeft)) {
    spin(three_rotation * 4, 0);  // Turn 90 degrees right
    spin(three_rotation * 4, 0);
    forward(1);
  } else if (bitRead(msgState, btForwards)) {
    forward(1.5);
  } else if (bitRead(msgState, btBackwards)) {
    reverse(1.5);
  }
  bitSet(msgState, rest);
}




int position = 0;
String array[] = {};

void array_maker() {
  // Serial.println("Message recieved 1");

  
  // while(1){
  //   if (message.equalsIgnoreCase("END")){
  //     break;
  //   }

  //   array[position] = message;
  //   position++;
  //   Serial.println("Message recieved 2");
    
  //   while (BTSerial.available()) {
  //     char inChar = (char)BTSerial.read();
  //     if (inChar == '\n') {
  //       stringComplete = true;
  //     } else {
  //       message += inChar;
  //     }
  //     // Serial.println("MSG Recv 3");
  //   }
  // }
  // for (int i = 0; i < sizeof(array); i++){
  //   Serial.println(array[i]);
  // }
}




void listenBluetooth() {
  // message = "";
  while (BTSerial.available()) {
    char inChar = (char)BTSerial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      message += inChar;
    }
  }  

  if (stringComplete) {
    // Serial.print("msg: "); Serial.println(message);
    bitClear(msgState, btForwards);
    bitClear(msgState, btBackwards);
    bitClear(msgState, btLeft);
    bitClear(msgState, btRight);
    bitClear(msgState, rest);
    digitalWrite(6, HIGH);  // Turn LED on to check
    message.toUpperCase();
    // array[position] = message;
    // position++;
    if (message.equalsIgnoreCase("LEFT")) {
      bitSet(msgState, btLeft);
      spin(three_rotation * 4, 0);  // Turn 90 degrees right
      spin(three_rotation * 4, 0);
      forward(one_rotation);
      message = "";
    }
    if (message.equalsIgnoreCase("RIGHT")) {
      bitSet(msgState, btRight);
      spin(three_rotation * 4, 1);  // Turn 90 degrees right
      spin(three_rotation * 4, 1);
      forward(one_rotation);
      message = "";
    }
    if (message.startsWith("FORWARDS")) {
      // int dist = message.substring(9).toInt(); // Change the distance it moves
      bitSet(msgState, btForwards);
      forward(one_rotation);
      message = "";
    }
    if (message.startsWith("BACKWARDS")) {
      // int dist = message.substring(9).toInt(); // Change the distance it moves
      bitSet(msgState, btBackwards);
      reverse(one_rotation);
      message = "";
    }
    if (message.startsWith("STOP")) {
      bitSet(msgState, rest);
      forward(1);
      message = "";
    }

    message = "";
    stringComplete = false;
    // Serial.print("Array "); Serial.println(array[0]);

  }
}






void topo_follow() {
  while(message = "LEFT") {
    bitSet(state, center);
    if (!(bitRead(flag, obLeft))){
      forward(0.4*one_rotation);
      bitClear(state, center);
      spin(one_rotation * 4, 1);
      spin(one_rotation*4, 1);
      forward(one_rotation);
      message = "";
    } //end of if left statement
  }

  while(message = "RIGHT") {
    bitSet(state,center);
    if (!(bitRead(flag, obRight))){
      bitClear(state, center);
      spin(one_rotation * 4, 0);
      spin(one_rotation*4, 0);
      forward(one_rotation);
      message = "";
    } //end of if left statement
  }
  bitSet(state, center);
}



void setup() {
  // stepper Motor set up
  pinMode(rtStepPin, OUTPUT);                   // sets pin as output
  pinMode(rtDirPin, OUTPUT);                    // sets pin as output
  pinMode(ltStepPin, OUTPUT);                   // sets pin as output
  pinMode(ltDirPin, OUTPUT);                    // sets pin as output
  pinMode(stepperEnable, OUTPUT);               // sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);  // turns off the stepper motor driver
  stepperRight.setMaxSpeed(max_spd);            // set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);      // set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_spd);             // set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);       // set desired acceleration in steps/s^2
  stepperRight.setSpeed(robot_spd);             // set right motor speed
  stepperLeft.setSpeed(robot_spd);              // set left motor speed
  steppers.addStepper(stepperRight);            // add right motor to MultiStepper
  steppers.addStepper(stepperLeft);             // add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);   // turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);                // turn on enable LED
  Timer1.initialize(timer_int);                 // initialize timer1, and set a period in microseconds
  Timer1.attachInterrupt(updateSensors);        // attaches updateSensors() as a timer overflow interrupt
  Serial.begin(baud_rate);                      // start serial communication in order to debug the software while coding
  delay(1500);                                  // wait 3 seconds before robot moves

  // Calibrate the light sensor
  for (int k = 0; k < 5; k++) {
    left_thres += analogRead(PR_Left);
    right_thres += analogRead(PR_Right);
    // Serial.println(k);
  }
  left_thres = left_thres / 5 + 100;
  right_thres = right_thres / 5 + 100;
  // Serial.println(right_thres);
  // Serial.println(left_thres);

  // Set-up bluetooth module
  pinMode(9, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
  digitalWrite(9, HIGH);
  Serial.begin(9600);
  Serial.println("Set up:");
  BTSerial.begin(9600);  // HC-05 default speed in AT command more
}

void loop() {
  moveRobot();  // wall following proportional control
  // PRRead();
  // light_follow();
  listenBluetooth();
  // updateSensors();
  // array_maker();
  topo_follow();
  // GUItrack();
  // message = "FORWARDS";
  // Light_track();
  Serial.print("state: "); Serial.println(state);
  // delay(500);
}
