/*RobotPDControl.ino
  Author: Carlotta. A. Berry
  Date: December 31, 2016
  This program will provide a template for an example of implementing a behavior-based control architecture
  for a mobile robot to implement wall following and random wander.  It will also show how to implement the basic structure for
  bang-bang, proportional and PD control. This is just a template to give you a start, freel free to brainstorm and create
  your own state machine and subsumption architecture.

  The flag byte (8 bits) variable will hold the IR and sonar data [X X snrRight snrLeft irLeft irRight irRear irFront]
  The state byte (8 bits) variable will hold the state information as well as motor motion [X X X wander runAway collide rev fwd]

  Use the following functions to read, clear and set bits in the byte
  bitRead(state, wander)) { // check if the wander state is active
  bitClear(state, wander);//clear the the wander state
  bitSet(state, wander);//set the wander state

  Hardware Connections:
  Stepper Enable          Pin 48
  Right Stepper Step      Pin 52
  Right Stepper Direction Pin 53
  Left Stepper Step       Pin 50
  Left Stepper Direction  Pin 51

  Front IR    A8
  Back IR     A9
  Right IR    A10
  Left IR     A11
  Left Sonar  A12
  Right Sonar A13
  Button      A15
*/

// #include <AccelStepper.h>//include the stepper motor library
// #include <MultiStepper.h>//include multiple stepper motor library
// #include <NewPing.h> //include sonar library
// #include <TimerOne.h>//include timer interrupt library

#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <Adafruit_MPU6050.h>//Include library for MPU6050 IMU
#include <SoftwareSerial.h> //include Bluetooth module
#include <TimerOne.h> //Includes the TimerOne library
#include <NewPing.h>    //Includes the NewPing library


//state LEDs connections
#define redLED 6            //red LED for displaying states
#define grnLED 7            //green LED for displaying states
#define ylwLED 8            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED


#define stepperEnable 48//stepper enable pin on stepStick
#define rtStepPin 50    //right stepper motor step pin
#define rtDirPin 51     // right stepper motor direction pin
#define ltStepPin 52    //left stepper motor step pin
#define ltDirPin 53     //left stepper motor direction pin

//define sensor pin numbers
#define irFront   A2    //front IR analog pin
#define irRear    A0    //back IR analog pin
#define irRight   A3   //right IR analog pin
#define irLeft    A1   //left IR analog pin
#define snrLeft   A4   //front left sonar 
#define snrRight  A5   //front right sonar 
#define button    A15   //pushbutton 

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin); //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);  //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                //create instance to control multiple steppers at the same time
NewPing sonarLt(snrLeft, snrLeft);    //create an instance of the left sonar
NewPing sonarRt(snrRight, snrRight);  //create an instance of the right sonar

//define stepper motor constants
#define stepperEnTrue false     //variable for enabling stepper motor
#define stepperEnFalse true     //variable for disabling stepper motor
#define test_led 13             //test led to test interrupt heartbeat
#define enableLED 13            //stepper enabled LED
#define robot_spd 500           //set robot speed
#define max_accel 10000         //maximum robot acceleration
#define max_spd 1000            //maximum robot speed
#define quarter_rotation 100    //stepper quarter rotation
#define half_rotation 200       //stepper half rotation
#define one_rotation  400       //stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  800      //stepper motor 2 rotations
#define three_rotation 1200     //stepper rotation 3 rotations
#define four_rotation 1600      //stepper rotation 4 rotations
#define five_rotation 2000      //stepper rotation 5 rotations

//define sensor constants and variables
#define irMin    150               // IR minimum threshold for wall (use a deadband of 4 to 6 inches)
#define irMax    300               // IR maximum threshold for wall (use a deadband of 4 to 6 inches)
#define snrMin   400               // sonar minimum threshold for wall (use a deadband of 4 to 6 inches)
#define snrMax   600               // sonar maximum threshold for wall (use a deadband of 4 to 6 inches)

int irFrontArray[5] = {0, 0, 0, 0, 0};//array to hold 5 front IR readings
int irRearArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 back IR readings
int irRightArray[5] = {0, 0, 0, 0, 0};//array to hold 5 right IR readings
int irLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left IR readings
int irFrontAvg;                       //variable to hold average of current front IR reading
int irLeftAvg;                        //variable to hold average of current left IR reading
int irRearAvg;                        //variable to hold average of current rear IR reading
int irRightAvg;                       //variable to hold average of current right IR reading
int irIdx = 0;                        //index for 5 IR readings to take the average
int srLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left sonar readings
int srRightArray[5] = {0, 0, 0, 0, 0};//array to hold 5 right sonar readings
int srIdx = 0;                        //index for 5 sonar readings to take the average
int srLeft;                           //variable to hold average of left sonar current reading
int srRight;                          //variable to hold average or right sonar current reading
int srLeftAvg;                        //variable to holde left sonar data
int srRightAvg;                       //variable to hold right sonar data

//STATE MACHINE TIMER INTERRUPT VARIABLES
volatile boolean test_state;          //variable to hold test led state for timer interrupt
#define timer_int 500000              //1/2 second (500000 us) period for timer interrupt

//bit definitions for sensor data flag byte [rt_snr left_snr left_ir right_ir rear_ir front_ir]
volatile byte flag = 0;
#define obFront   0   // Front IR trip [used to detect front wall for corner]
#define obRear    1   // Rear IR trip
#define obRight   2   // Right IR trip
#define obLeft    3   // Left IR trip
#define obFLeft   4   // Left Sonar trip
#define obFRight  5   // Right Sonar trip

//bit definitions for robot motion and state byte [follow_hallway follow_right follow_left wander avoid]
volatile byte state = 0;
#define avoid     0   //avoid behavior
#define wander    1   //wander behavior
#define fleft     2   //follow left wall behavior
#define fright    3   //follow right wall behavior
#define center    4   //follow hallway behavior
#define movingL   6   //robot left wheel moving
#define movingR   7   //robot right wheel moving

//define layers of subsumption architecture that are active [hallway Wall Wander Avoid]
byte layers = 4;
#define aLayer 0      //avoid obstacle layer
#define wLayer 1      //wander layer
#define fwLayer 2     //follow wall layer
#define fhLayer 3     //follow hallway layer

//define PD control global variables, curr_error = current reading - setpoint, prev_error = curr_error on previous iteration
//store previous error to calculate derror = curr_error-prev_error, side_derror = side front sensor - side back sensor
//store derror = difference between left and right error (used for hall follow to center the robot)

int ls_curr;    //left sonar current reading
int li_curr;    //left ir current reading
int rs_curr;    //right sonar current reading
int ri_curr;    //right ir current reading

int ls_cerror;    //left sonar current error
int li_cerror;    //left ir current error
int rs_cerror;    //right sonar current error
int ri_cerror;    //right ir current error

int ls_perror;    //left sonar previous error
int li_perror;    //left ir previous error
int rs_perror;    //right sonar previous error
int ri_perror;    //right ir previous error

int ls_derror;  //left sonar delta error
int li_derror;  //left ir delta error
int rs_derror;  //right sonar delta error
int ri_derror;  //right ir current error
int left_derror;   //difference between left front and back sensor, this may be useful for adjusting the turn angle
int right_derror;  //difference between right front and back sensor, this may be useful for adjusting the turn angle

int derror;       //difference between left and right error to center robot in the hallway
float prev_left = 0;

#define baud_rate 9600  //set serial communication baud rate

void runToStop ( void ) {
  int runNow = 1;
  //  stepperRight.setMaxSpeed(max_spd);
  //  stepperLeft.setMaxSpeed(max_spd);
  //  stepperRight.setSpeed(robot_spd);
  //  stepperLeft.setSpeed(robot_spd);
  while (runNow) {
    // Serial.println("waiting");
    if (!stepperRight.run()) {
      bitClear(state, movingR);  // clear bit for right motor moving
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      bitClear(state, movingL);   // clear bit for left motor moving
      stepperLeft.stop();//stop left motor
    }//
    if (!bitRead(state, movingR) & !bitRead(state, movingL))
      runNow = 0;
  }
}

/*robot move forward function */
void forward(int rot) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  positions[0] = stepperRight.currentPosition() + rot;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot;   //left motor absolute position

  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*robot move reverse function */
void reverse(int rot) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  positions[0] = stepperRight.currentPosition() - rot;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() - rot;   //left motor absolute position

  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*robot pivot function */
void pivot(int rot, int dir) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  if (dir > 0) {//pivot right
    positions[0] = stepperRight.currentPosition();    //right motor absolute position
    positions[1] = stepperLeft.currentPosition() + rot ; //left motor absolute position
  }
  else//pivot left
  {
    positions[0] = stepperRight.currentPosition() + rot + 10; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() ;     //left motor absolute position
  }
  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

// void pivot(int direction, float spins) {
//   spins=spins*4100; // The factor to go from revolutions to pulses was found to be 4100 pulses/rev
//   int speed = 900;  // Speed of motor turning
//   if(direction==1){  // Clockwise rotation
//     stepperLeft.setMaxSpeed(speed);
//     stepperLeft.move(spins);
//   }
//   else if(direction==0){  // Counter-clockwise rotation
//     stepperLeft.setMaxSpeed(speed);
//     stepperRight.move(spins);
//   }
//   steppers.runSpeedToPosition();
//   // runToStop();
// }


















/*robot spin function */
void spin(int rot, int dir) {
  long positions[2];                                    // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);                   //reset right motor to position 0
  stepperLeft.setCurrentPosition(0);                    //reset left motor to position 0
  if (dir > 0) {//spin right
    positions[0] = stepperRight.currentPosition() - rot; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() + rot; //left motor absolute position
  }
  else//spin left
  {
    positions[0] = stepperRight.currentPosition() + rot; //right motor absolute position
    positions[1] = stepperLeft.currentPosition() - rot;  //left motor absolute position
  }
  stepperRight.move(positions[0]);    //move right motor to position
  stepperLeft.move(positions[1]);     //move left motor to position
  bitSet(state, movingL);             //move left wheel
  bitSet(state, movingR);             //move right wheel
  runToStop();                        //run until the robot reaches the target
}

/*robot stop function */
void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}


/*
   This is a sample wallBang() function, the description and code should be updated to reflect the actual robot motion function that you will implement
   based upon the the lab requirements.  Some things to consider, you cannot use a blocking motor function because you need to use sensor data to update
   movement.  You also need to continue to poll    the sensors during the motion and update flags and state because this will serve as your interrupt to
   stop or change movement. This function will have the robot follow the wall if it is within 4 to 6 inches from the wall by moving forward and turn on the
   controller if it is outside that band to make an adjustment to get back within the band.
*/
void wallBang() {
  // Serial.print("\nWallBang: li_cerror ri_cerror\t");
  // Serial.print(li_cerror); Serial.print("\t");
  // Serial.println(ri_cerror);
  Serial.println("1");
  if (bitRead(state, fright)) {
    Serial.println("6");
    // Serial.println("right wall found");
    if (bitRead(flag, obFront)) { //check for a front wall before moving
      // Serial.print("right wall: front corner ");
      //make left turn if wall found
      reverse(one_rotation);              //back up
      spin(three_rotation, 0);              //turn left
    }
    if (ri_cerror == 0) {                 //no error, robot in deadband
      // Serial.println("right wall detected, drive forward");
      forward(three_rotation);            //move robot forward
      // digitalWrite(redLED, LOW);
      // digitalWrite(ylwLED, LOW);
    }
    else {
      //Serial.println("rt wall: adjust turn angle based upon error");
      if (ri_cerror < 0 && rs_curr < 1000) {          //negative error means too close
        // Serial.println("\trt wall: too close turn left");
        pivot(quarter_rotation, 1);      //pivot left
        pivot(quarter_rotation, 0);   //pivot right to straighten up
        // pivot(1, 0.25);
        // pivot(0,0.25);
        digitalWrite(ylwLED, HIGH);
        // digitalWrite(redLED, LOW);
      }
      else if (ri_cerror > 0 && rs_curr > 500) {     //positive error means too far
        // Serial.println("\trt wall: too far turn right");
        pivot(quarter_rotation, 1);      //pivot right
        pivot(quarter_rotation, 0);   //pivot left to straighten up
        // pivot(1,0.25);
        // pivot(0,0.25);
        // digitalWrite(ylwLED, LOW);
        digitalWrite(redLED, HIGH);
      }
    }
  }
  
  else if (bitRead(state, fleft)  ) {
      Serial.println("2");

    if (bitRead(flag, obFront)) { //check for a front wall before moving forward
      //make right turn if wall found
      // Serial.print("left wall: front corner ");
      //make left turn if wall found
      reverse(one_rotation);              //back up
      spin(three_rotation, 1);              //turn right
    }
    if (li_cerror == 0) {           //no error robot in dead band drives forward
      // Serial.println("lt wall detected, drive forward");
      forward(two_rotation);      //move robot forward
      // digitalWrite(redLED, LOW);
      // digitalWrite(ylwLED, LOW);
    }
    else {
      //Serial.println("lt wall detected: adjust turn angle based upon error");
      if (li_cerror < 0 && ls_curr < 1000) { //negative error means too close
        // Serial.println("\tlt wall: too close turn right");
        pivot(quarter_rotation, 1);      //pivot right
        pivot(quarter_rotation, 0);   //pivot left
        // pivot(1,0.25);
        // pivot(0,0.25);
        digitalWrite(ylwLED, HIGH);
        // digitalWrite(redLED, LOW);

      }
      else if (li_cerror > 0 && ls_curr > 500)  { //positive error means too far
        // Serial.println("\tlt wall: too far turn left");
        pivot(quarter_rotation, 1);      //pivot left
        pivot(quarter_rotation, 0);   //pivot right
        // pivot(1,0.25);
        // pivot(0,0.25);
        // digitalWrite(ylwLED, LOW);
        digitalWrite(redLED, HIGH);
      }
    }
  }
  else if (bitRead(state, center) ) {//follow hallway
    Serial.println("3");

    if (((ri_cerror == 0) && (li_cerror == 0)) || (derror == 0)) {
      // Serial.println("hallway detected, drive forward");
      forward(two_rotation);          //drive robot forward
    }
    else {
      //Serial.println("hallway detected: adjust turn angle based upon error");
      //try to average the error between the left and right to center the robot
      if (derror > 0) {
        spin(quarter_rotation, 1);        //spin right, the left error is larger
        pivot(quarter_rotation, 0);       //pivot left to adjust forward
        // pivot(0,0.25);
      }
      else
      {
        spin(quarter_rotation, 0);        //spin left the right error is larger
        pivot(quarter_rotation, 1);       //pivot right to adjust forward
        // pivot(1,0.25);
      }
    }
  }
  else  if (bitRead(state, wander)) {
      Serial.println("4");

    // Serial.println("nothing to see here, I need to look for a wall");
    stop();
    delay(500);
    //reverse(half_rotation);
    spin(half_rotation, 0);
    forward(one_rotation);
    pivot(quarter_rotation,1);
    // pivot(1,0.25);
  }
    Serial.print("5");Serial.println(state);

}


/*
  This is a sample updateSensors() function and it should be updated along with the description to reflect what you actually implemented
  to meet the lab requirements.
*/

/*
   This is a sample updateIR() function, the description and code should be updated to take an average, consider all sensor and reflect
   the necesary changes for the lab requirements.
*/
void updateIR() {
  float front, back, left, right;         //declare IR variables
  // front = analogRead(irFront);          //read front IR sensor
  // back = analogRead(irRear);            //read back IR sensor
  // left = analogRead(irLeft);            //read left IR sensor
  // right = analogRead(irRight);          //read right IR sensor

  //  print IR data
  //  Serial.println("frontIR\tbackIR\tleftIR\trightIR");
  //  Serial.print(front); Serial.print("\t");
  //  Serial.print(back); Serial.print("\t");
  //  Serial.print(left); Serial.print("\t");
  //  Serial.println(right);


for (int k =0; k<4; k++) {                      //Take the average of 5 readings of the sensors for an accurate result
  front += analogRead(irFront);          //read front IR sensor
  back += analogRead(irRear);            //read back IR sensor
  left += analogRead(irLeft);            //read left IR sensor
  right += analogRead(irRight);          //read right IR sensor
  }
  front = front/5;
  back = back/5;
  left = left/5;
  right = right/5;

  front = (1111/(front+3) - 1);                   //Calculate the distance of the object in inches for the front, back, left, and right sensors
  back = (1116/(back+6) - 1);
  left = (1447/(left-29) - 1);
  right = (1379/(right-59) - 1);

  // if (left > 0) {
  //   prev_left = left;
  // }
  // else{
  //   left = prev_left;
  // }
  // Serial.print("Left IR: "); Serial.println(left); 

  if(front < 8 && front > 1) {                    //If an object is between 0 to 5 inches of the front sensor, set a flag on obFront bit and turn on red LED, otherwise, clear the flag
    digitalWrite(redLED, HIGH);
    digitalWrite(grnLED,LOW);
    bitSet(flag, obFront);
  }
  else {
    bitClear(flag, obFront);
    digitalWrite(redLED, LOW);
  }

  if(back < 8 && back > 1) {                    //If an object is between 0 to 5 inches of the back sensor, set a flag on obBack bit and turn on red LED, otherwise, clear the flag
    digitalWrite(redLED, HIGH);
    digitalWrite(grnLED,LOW);
    bitSet(flag, obRear);
  }
  else {
    bitClear(flag, obRear);
    digitalWrite(redLED, LOW);
  }

  if(left < 8 && left > 1) {                    //If an object is between 0 to 5 inches of the left sensor, set a flag on obLeft bit and turn on red LED, otherwise, clear the flag
    digitalWrite(redLED, HIGH);
    digitalWrite(grnLED,LOW);
    bitSet(flag, obLeft);
  }
  else {
    bitClear(flag, obLeft);
    digitalWrite(redLED, LOW);
  }

  if(right < 8 && right > 1) {                    //If an object is between 0 to 5 inches of the right sensor, set a flag on obRight bit and turn on red LED, otherwise, clear the flag
    digitalWrite(redLED, HIGH);
    digitalWrite(grnLED,LOW);
    bitSet(flag, obRight);
  }
  else {
    bitClear(flag, obRight);
    digitalWrite(redLED, LOW);
  }

  
  // Serial.print(" Right IR: "); Serial.println(right);
  




  // if (right > irMin - 50) {
  //   //Serial.println("\t\tset right obstacle");
  //   bitSet(flag, obRight);            //set the right obstacle
  // }
  // else
  //   bitClear(flag, obRight);          //clear the right obstacle

  // if (left > irMin - 50) {
  //   //Serial.println("\t\tset left obstacle");
  //   bitSet(flag, obLeft);             //set the left obstacle
  // }
  // else
  //   bitClear(flag, obLeft);           //clear the left obstacle

  // if (front > irMax - 50) {
  //   //Serial.println("set front obstacle bit");
  //   bitSet(flag, obFront);            //set the front obstacle
  // }
  // else
  //   bitClear(flag, obFront);          //clear the front obstacle

  //  if (back > irMin - 25) {
  //    //Serial.println("set back obstacle bit");
  //    bitSet(flag, obRear);             //set the back obstacle
  //  }
  //  else
  //    bitClear(flag, obRear);           //clear the back obstacle

  ///////////////////////update variables
  //Serial.print(left);Serial.print("\t");
  //Serial.println(right);


  ri_curr = right;             //log current sensor reading [right IR]
  if ((ri_curr > 6) | (ri_curr < 4))
    ri_cerror = irMax - ri_curr;  //calculate current error (too far positive, too close negative)
  else
    ri_cerror = 0;                  //set error to zero if robot is in dead band
  ri_derror = ri_cerror - ri_perror; //calculate change in error
  ri_perror = ri_cerror;            //log current error as previous error [left sonar]

  li_curr = left;                   //log current sensor reading [left sonar]
  if ((li_curr > 6) | (li_curr < 4))
    li_cerror = irMax - li_curr;   //calculate current error
  else
    li_cerror = 0;                  //error is zero if in deadband
  li_derror = li_cerror - li_perror; //calculate change in error
  li_perror = li_cerror;                //log reading as previous error

  ///// print IR data
  if (right > 0 && right < 1000) { //filter out garbage readings
    //    Serial.print("right IR current = \t"); Serial.print(ri_curr);
    //Serial.print("\tright IR cerror = \t"); Serial.println(ri_cerror);
    //    Serial.print("\tright IR derror = \t"); Serial.print(ri_derror);
    //    Serial.print("\tright IR perror = \t"); Serial.println(ri_perror);
  }

  if (left > 0 && left < 1000) { //filter out garbage readings
    //    Serial.print("left IR current = \t"); Serial.print(li_curr);
    //Serial.print("\tleft IR cerror = \t"); Serial.println(li_cerror);
    //    Serial.print("\tleft IR derror = \t"); Serial.print(li_derror);
    //    Serial.print("\tleft IR perror = \t"); Serial.println(li_perror);
  }
}

/*
   This is a sample updateSonar() function, the description and code should be updated to take an average, consider all sensors and reflect
   the necesary changes for the lab requirements.
*/
void updateSonar() {
  long left, right;             //sonar variables
  pinMode(snrRight, OUTPUT);    //set the PING pin as an output, read right sensor
  digitalWrite(snrRight, LOW);  //set the PING pin low first
  delayMicroseconds(2);         //wait 2 us
  digitalWrite(snrRight, HIGH); //trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);         //wait 5 us
  digitalWrite(snrRight, LOW);  //set pin low first again
  pinMode(snrRight, INPUT);     //set pin as input with duration as reception
  right = pulseIn(snrRight, HIGH);//measures how long the pin is high

  pinMode(snrLeft, OUTPUT);     //set the PING pin as an output, read left sensor
  digitalWrite(snrLeft, LOW);   //set the PING pin low first
  delayMicroseconds(2);         //wait 2 us
  digitalWrite(snrLeft, HIGH);  //trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);         //wait 5 us
  digitalWrite(snrLeft, LOW);   //set pin low first again
  pinMode(snrLeft, INPUT);      //set pin as input with duration as reception
  left = pulseIn(snrLeft, HIGH);//measures how long the pin is high

  ///////////////////////update variables
  //  Serial.print(left); Serial.print("\t");
  //  Serial.println(right);
  rs_curr = right;             //log current sensor reading [right sonar]
  if ((rs_curr > snrMax) || (rs_curr < snrMin))
    rs_cerror = rs_curr - snrMax;    //calculate current error (too far positive, too close negative)
  else
    rs_cerror = 0;                  //set error to zero if robot is in dead band
  rs_derror = rs_cerror - rs_perror; //calculate change in error
  rs_perror = rs_cerror;            //log current error as previous error [left sonar]

  ls_curr = left;                   //log current sensor reading [left sonar]
  if ((ls_curr > snrMax) || (ls_curr < snrMin))
    ls_cerror = ls_curr - snrMax;     //calculate current error
  else
    ls_cerror = 0;                  //error is zero if in deadband
  ls_derror = ls_cerror - ls_perror; //calculate change in error
  ls_perror = ls_cerror;                //log reading as previous error

  ///// print sonar data
  if (right > 0 && right < 1000) { //filter out garbage readings
    //    Serial.print("right sonar current = \t"); Serial.print(rs_curr);
    //    Serial.print("\tright sonar cerror = \t"); Serial.print(rs_cerror);
    //    Serial.print("\tright sonar derror = \t"); Serial.print(rs_derror);
    //    Serial.print("\tright sonar perror = \t"); Serial.println(rs_perror);
  }

  if (left > 0 && left < 1000) { //filter out garbage readings
    //    Serial.print("left sonar current = \t"); Serial.print(ls_curr);
    //    Serial.print("\tleft sonar cerror = \t"); Serial.print(ls_cerror);
    //    Serial.print("\tleft sonar derror = \t"); Serial.print(ls_derror);
    //    Serial.print("\tleft sonar perror = \t"); Serial.println(ls_perror);
  }
}


/*
  This is a sample updateSonar2() function, the description and code should be updated to take an average, consider all sensors and reflect
  the necesary changes for the lab requirements.
*/
void updateSonar2() {
  srRightAvg =  sonarRt.ping_in();//read right sonar in inches
  delay(50);                      //delay 50 ms
  srLeftAvg = sonarLt.ping_in();  //reaqd left sonar in inches
  //print sonar data
  //    Serial.print("lt snr:\t");
  //    Serial.print(srLeftAvg);
  //    Serial.print("rt snr:\t");
  //    Serial.println(srRightAvg);
  /////////////////////print sonar data
  //    Serial.println("leftSNR\trightSNR");
  //    Serial.print(left); Serial.print("\t");
  //    Serial.println(right);

  ///////////////////////update variables
  rs_curr = srRightAvg;             //log current sensor reading [right sonar]
  if ((rs_curr > snrMax) || (rs_curr < snrMin))
    rs_cerror = rs_curr - snrMax;    //calculate current error (too far positive, too close negative)
  else
    rs_cerror = 0;                  //set error to zero if robot is in dead band
  rs_derror = rs_cerror - rs_perror; //calculate change in error
  rs_perror = rs_cerror;            //log current error as previous error [left sonar]

  ls_curr = srLeftAvg;                   //log current sensor reading [left sonar]
  if ((ls_curr > snrMax) || (ls_curr < snrMin))
    ls_cerror = ls_curr - snrMax;     //calculate current error
  else
    ls_cerror = 0;                  //error is zero if in deadband
  ls_derror = ls_cerror - ls_perror; //calculate change in error
  ls_perror = ls_cerror;                //log reading as previous error

  ///// print sonar data
  if ((srRightAvg > 0 && srRightAvg < 20)) { //filter out garbage readings
    //    Serial.print("right sonar current = \t"); Serial.print(rs_curr);
    //    Serial.print("\tright sonar cerror = \t"); Serial.print(rs_cerror);
    //    Serial.print("\tright sonar derror = \t"); Serial.print(rs_derror);
    //    Serial.print("\tright sonar perror = \t"); Serial.println(rs_perror);
  }

  if ((srRightAvg > 0 && srRightAvg < 20)) { //filter out garbage readings
    //    Serial.print("left sonar current = \t"); Serial.print(ls_curr);
    //    Serial.print("\tleft sonar cerror = \t"); Serial.print(ls_cerror);
    //    Serial.print("\tleft sonar derror = \t"); Serial.print(ls_derror);
    //    Serial.print("\tleft sonar perror = \t"); Serial.println(ls_perror);
  }
}

/*
   This function will update all of the error constants to be used for P and PD control
   store previous error to calculate derror = curr_sensor-prev_sensor, side_derror = side front sensor - side back sensor
*/
void updateError() {
  left_derror = ls_cerror - li_cerror; //difference between left front and back sensor, use threshold for robot mostly parallel to wall
  right_derror = rs_cerror - ri_cerror; //difference between right front and back sensor, use threshold for robot mostly parallel to wall
  //derror = ls_cerror - rs_cerror;//use sonar data for difference error
  derror = li_cerror - ri_cerror; //use IR data for difference error
  //  Serial.print("left derror\t"); Serial.print(left_derror);
  //  Serial.print("\tright derror\t"); Serial.println(right_derror);
}

/*
   This is a sample updateState() function, the description and code should be updated to reflect the actual state machine that you will implement
   based upon the the lab requirements.
*/
void updateState() {
  if ((!bitRead(flag, obRight) && !bitRead(flag, obLeft))) { //no sensors triggered
    //set random wander bit
    // Serial.println("\tset random wander state");
    bitSet(state, wander);//set the wander state
    //clear all other bits
    bitClear(state, fright);//clear follow wall state
    bitClear(state, fleft);//clear follow wall state
    bitClear(state, center);//clear follow wall state
  }
  else if (bitRead(flag, obRight) && !bitRead(flag, obLeft) ) {
    // Serial.println("\tset follow right state");
    bitSet(state, fright);    //set RIGHT WALL state
    //clear all other bits
    bitClear(state, wander);  //clear wander state
    bitClear(state, fleft);   //clear follow wall state
    bitClear(state, center);  //clear follow wall state
  }
  else if (bitRead(flag, obLeft) && !bitRead(flag, obRight) ) {
    // Serial.println("\tset follow left state");
    bitSet(state, fleft);     //set left wall state
    //clear all other bits
    bitClear(state, fright);  //clear follow wall state
    bitClear(state, wander);  //clear wander state
    bitClear(state, center);  //clear follow wall state
  }
  else if (bitRead(flag, obLeft) && bitRead(flag, obRight) ) {
    // Serial.println("\tset follow hallway state");
    bitSet(state, center);      //set the hallway state
    //clear all other bits
    bitClear(state, fright);    //clear follow wall state
    bitClear(state, wander);    //clear wander state
    bitClear(state, fleft);     //clear follow wall state
  }

  //print flag byte
  //  Serial.println("\trtSNR\tltSNR\tltIR\trtIR\trearIR\tftIR");
  //  Serial.print("flag byte: ");
  //  Serial.println(flag, BIN);
  //print state byte
  //  Serial.println("\tfollowHall\tfollowLeft\tfollowRight\twander\tavoid");
  //  Serial.print("state byte: ");
  //  Serial.println(state, BIN);
}

void updateSensors() {
  //Serial.println("updateSensors\t");
  test_state = !test_state;             //LED to test the heartbeat of the timer interrupt routine
  digitalWrite(test_led, test_state);   //flash the timer interrupt LED
  flag = 0;                             //clear all sensor flags
  state = 0;                            //clear all state flags
  updateIR();                           //update IR readings and update flag variable and state machine
  updateSonar();                        //update Sonar readings and update flag variable and state machine
  //updateSonar2();                     //there are 2 ways to read sonar data, this is the 2nd option, use whichever one works best for your hardware
  updateError();                        //update sensor current, previous, change in error
  updateState();                        //update State Machine based upon sensor readings
}




/*This function, runToStop(), will run the robot until the target is achieved and
  then stop it
*/

void setup()
{
  //stepper Motor set up
  pinMode(rtStepPin, OUTPUT);                 //sets pin as output
  pinMode(rtDirPin, OUTPUT);                  //sets pin as output
  pinMode(ltStepPin, OUTPUT);                 //sets pin as output
  pinMode(ltDirPin, OUTPUT);                  //sets pin as output
  pinMode(stepperEnable, OUTPUT);             //sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  stepperRight.setMaxSpeed(max_spd);          //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);    //set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_spd);           //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);     //set desired acceleration in steps/s^2
  stepperRight.setSpeed(robot_spd);           //set right motor speed
  stepperLeft.setSpeed(robot_spd);            //set left motor speed
  steppers.addStepper(stepperRight);          //add right motor to MultiStepper
  steppers.addStepper(stepperLeft);           //add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue); //turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);              //turn on enable LED
  Timer1.initialize(timer_int);               //initialize timer1, and set a period in microseconds
  Timer1.attachInterrupt(updateSensors);      //attaches updateSensors() as a timer overflow interrupt
  Serial.begin(baud_rate);                    //start serial communication in order to debug the software while coding
  delay(1500);                                //wait 3 seconds before robot moves
}


void loop()
{
  wallBang();           //wall following bang-bang control
  //wallP();            //wall following proportional control
  //wallPD();           //wall following PD control
  //follow_hallway();   //robot moves to follow center of hallway when two walls are detected
  //wander();           //random wander behavior
  //avoid();            //avoid obstacle behavior
  //delay(500);     //added so that you can read the data on the serial monitor
}





















// /*
//   ECE 425 Lab 2
//   Done by: Advait Pandharkar and Alejandro Marcinedo Laregolla
//   Robot name: Kamen
  
//   Libraries included: PinChangeint, TimerOne, NewPing
//   Interrupts
//   https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
//   https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
//   https://playground.arduino.cc/code/timer1
//   https://playground.arduino.cc/Main/TimerPWMCheatsheet
//   http://arduinoinfo.mywikis.net/wiki/HOME

//   Hardware Connections:
//   Arduino pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
//   A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182 

//   digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
//   digital pin 50 - right stepper motor step pin
//   digital pin 51 - right stepper motor direction pin
//   digital pin 52 - left stepper motor step pin
//   digital pin 53 - left stepper motor direction pin
//   digital pin 13 - enable LED on microcontroller

//   digital pin 6 - red LED in series with 220 ohm resistor
//   digital pin 7 - green LED in series with 220 ohm resistor
//   digital pin 8 - yellow LED in series with 220 ohm resistor

//   digital pin 18 - left encoder pin
//   digital pin 19 - right encoder pin

//   digital pin 2 - IMU INT
//   digital pin 20 - IMU SDA
//   digital pin 21 - IMU SCL


//   INSTALL THE LIBRARY
//   AccelStepper Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
  
//   Sketch->Include Library->Manage Libraries...->AccelStepper->Include
//   OR
//   Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
//   See PlatformIO documentation for proper way to install libraries in Visual Studio
// */

// //includew all necessary libraries
// #include <Arduino.h>//include for PlatformIO Ide
// #include <AccelStepper.h>//include the stepper motor library
// #include <MultiStepper.h>//include multiple stepper motor library
// #include <Adafruit_MPU6050.h>//Include library for MPU6050 IMU
// #include <SoftwareSerial.h> //include Bluetooth module
// #include <TimerOne.h> //Includes the TimerOne library
// #include <NewPing.h>    //Includes the NewPing library

// //state LEDs connections
// #define redLED 6            //red LED for displaying states
// #define grnLED 7            //green LED for displaying states
// #define ylwLED 8            //yellow LED for displaying states
// #define enableLED 13        //stepper enabled LED

// //define motor pin numbers
// #define stepperEnable 48    //stepper enable pin on stepStick 
// #define rtStepPin 50 //right stepper motor step pin 
// #define rtDirPin 51  // right stepper motor direction pin 
// #define ltStepPin 52 //left stepper motor step pin 
// #define ltDirPin 53  //left stepper motor direction pin 

// // define the pins for the Sonar and IR sensors
// #define sonarL A4
// #define sonarR A5
// #define IR_front A2
// #define IR_back A0
// #define IR_Left A1
// #define IR_right A3

// // define the bits for the Front, Back, and Left, Right sensors
// #define obFront 0
// #define obRight 1
// #define obBack 2
// #define obLeft 3
// #define obSLeft 4
// #define obSRight 5


// // define the numbers for the states that te robot can be in
// #define forwards 0
// #define backwards 1
// #define leftwards 2
// #define rightwards 3
// #define collide 4
// #define run_away 5
// #define random 6
// #define rest 7
// #define sGoToGoal 8
// #define RightWall 9
// #define LeftWall 10
// #define BothWalls 11

// //Define the flag and state as a volatile byte
// volatile byte flag = 0;
// volatile byte state = 0;

// //define global variables for wander loops and wander_n (both used in the random wander function)
// int wanderloops = 0;
// int wander_n;

// //Define the final and current values for x and y (used in the smart homing funciton)
// float current_x;
// float current_y;
// float final_x;
// float final_y;

// //define the orientation of the robot in degrees with respect to the positive x axis 
// float orient=0;


// AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
// AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
// MultiStepper steppers;//create instance to control multiple steppers at the same time

// #define stepperEnTrue false //variable for enabling stepper motor
// #define stepperEnFalse true //variable for disabling stepper motor

// int pauseTime = 2500;   //time before robot moves
// int stepTime = 500;     //delay time between high and low on step pin
// int wait_time = 1000;   //delay for printing data
// float Left_dist;
// float Right_dist;

// //define encoder pins
// #define LEFT 0        //left encoder
// #define RIGHT 1       //right encoder
// const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
// const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
// volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
// int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
// int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

// //IMU object
// Adafruit_MPU6050 mpu;

// //Bluetooth module connections
// #define BTTX 10 // TX on chip to pin 10 on Arduino Mega
// #define BTRX 11 //, RX on chip to pin 11 on Arduino Mega
// SoftwareSerial BTSerial(BTTX, BTRX);

// // Helper Functions

// //interrupt function to count left encoder tickes
// void LwheelSpeed()
// {
//   encoder[LEFT] ++;  //count the left wheel encoder interrupts
// }

// //interrupt function to count right encoder ticks
// void RwheelSpeed()
// {
//   encoder[RIGHT] ++; //count the right wheel encoder interrupts
// }

// //function to initialize Bluetooth
// void init_BT(){
//   Serial.println("Goodnight moon!");
//   BTSerial.println("Hello, world?");
// }
// //function to initialize IMU
// void init_IMU(){
//   Serial.println("Adafruit MPU6050 init!");

//   // Try to initialize!
//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("MPU6050 Found!");

//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   Serial.print("Accelerometer range set to: ");
//   switch (mpu.getAccelerometerRange()) {
//   case MPU6050_RANGE_2_G:
//     Serial.println("+-2G");
//     break;
//   case MPU6050_RANGE_4_G:
//     Serial.println("+-4G");
//     break;
//   case MPU6050_RANGE_8_G:
//     Serial.println("+-8G");
//     break;
//   case MPU6050_RANGE_16_G:
//     Serial.println("+-16G");
//     break;
//   }
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   Serial.print("Gyro range set to: ");
//   switch (mpu.getGyroRange()) {
//   case MPU6050_RANGE_250_DEG:
//     Serial.println("+- 250 deg/s");
//     break;
//   case MPU6050_RANGE_500_DEG:
//     Serial.println("+- 500 deg/s");
//     break;
//   case MPU6050_RANGE_1000_DEG:
//     Serial.println("+- 1000 deg/s");
//     break;
//   case MPU6050_RANGE_2000_DEG:
//     Serial.println("+- 2000 deg/s");
//     break;
//   }

//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//   Serial.print("Filter bandwidth set to: ");
//   switch (mpu.getFilterBandwidth()) {
//   case MPU6050_BAND_260_HZ:
//     Serial.println("260 Hz");
//     break;
//   case MPU6050_BAND_184_HZ:
//     Serial.println("184 Hz");
//     break;
//   case MPU6050_BAND_94_HZ:
//     Serial.println("94 Hz");
//     break;
//   case MPU6050_BAND_44_HZ:
//     Serial.println("44 Hz");
//     break;
//   case MPU6050_BAND_21_HZ:
//     Serial.println("21 Hz");
//     break;
//   case MPU6050_BAND_10_HZ:
//     Serial.println("10 Hz");
//     break;
//   case MPU6050_BAND_5_HZ:
//     Serial.println("5 Hz");
//     break;
//   }
// }

// //function to set all stepper motor variables, outputs and LEDs
// void init_stepper(){
//   pinMode(rtStepPin, OUTPUT);//sets pin as output
//   pinMode(rtDirPin, OUTPUT);//sets pin as output
//   pinMode(ltStepPin, OUTPUT);//sets pin as output
//   pinMode(ltDirPin, OUTPUT);//sets pin as output
//   pinMode(stepperEnable, OUTPUT);//sets pin as output
//   digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
//   pinMode(enableLED, OUTPUT);//set enable LED as output
//   digitalWrite(enableLED, LOW);//turn off enable LED
//   pinMode(redLED, OUTPUT);//set red LED as output
//   pinMode(grnLED, OUTPUT);//set green LED as output
//   pinMode(ylwLED, OUTPUT);//set yellow LED as output
//   digitalWrite(redLED, HIGH);//turn on red LED
//   digitalWrite(ylwLED, HIGH);//turn on yellow LED
//   digitalWrite(grnLED, HIGH);//turn on green LED
//   delay(pauseTime / 5); //wait 0.5 seconds
//   digitalWrite(redLED, LOW);//turn off red LED
//   digitalWrite(ylwLED, LOW);//turn off yellow LED
//   digitalWrite(grnLED, LOW);//turn off green LED

//   stepperRight.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
//   stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
//   stepperLeft.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
//   stepperLeft.setAcceleration(10000);//set desired acceleration in steps/s^2
//   steppers.addStepper(stepperRight);//add right motor to MultiStepper
//   steppers.addStepper(stepperLeft);//add left motor to MultiStepper
//   digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
//   digitalWrite(enableLED, HIGH);//turn on enable LED
// }

// //function prints encoder data to serial monitor
// void print_encoder_data() {
//   static unsigned long timer = 0;                           //print manager timer
//   if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
//     lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
//     lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
//     accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
//     accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
//     Serial.println("Encoder value:");
//     Serial.print("\tLeft:\t");
//     Serial.print(encoder[LEFT]);
//     Serial.print("\tRight:\t");
//     Serial.println(encoder[RIGHT]);
//     Serial.println("Accumulated Ticks: ");
//     Serial.print("\tLeft:\t");
//     Serial.print(accumTicks[LEFT]);
//     Serial.print("\tRight:\t");
//     Serial.println(accumTicks[RIGHT]);
//     encoder[LEFT] = 0;                          //clear the left encoder data buffer
//     encoder[RIGHT] = 0;                         //clear the right encoder data buffer
//     timer = millis();                           //record current time since program started
//   }
// }

// //function to print IMU data to the serial monitor
// void print_IMU_data(){
//     /* Get new sensor events with the readings */
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   /* Print out the values */
//   Serial.print("Acceleration X: ");
//   Serial.print(a.acceleration.x);
//   Serial.print(", Y: ");
//   Serial.print(a.acceleration.y);
//   Serial.print(", Z: ");
//   Serial.print(a.acceleration.z);
//   Serial.println(" m/s^2");

//   Serial.print("Rotation X: ");
//   Serial.print(g.gyro.x);
//   Serial.print(", Y: ");
//   Serial.print(g.gyro.y);
//   Serial.print(", Z: ");
//   Serial.print(g.gyro.z);
//   Serial.println(" rad/s");

//   Serial.print("Temperature: ");
//   Serial.print(temp.temperature);
//   Serial.println(" degC");

//   Serial.println("");
// }

// //function to send and receive data with the Bluetooth
// void Bluetooth_comm(){
//   String data="";
//   if (Serial.available()) {
//      while (Serial.available()){
//       char nextChar = Serial.read();
//       data = data + String(nextChar); 
//       if (nextChar == ';') {
//         break;
//       }
//      }
//     Serial.println(data);
//     BTSerial.println(data);
//   }
  
//   if (BTSerial.available()) {
//     while (BTSerial.available()){
//       char nextChar = BTSerial.read();
//       data = data + String(nextChar); 
//       if (nextChar == ';') {
//         break;
//       }
//     }
//     Serial.println(data);
//     BTSerial.println(data);
//   }
// }
  
  
// /*function to run both wheels to a position at speed*/
// void runAtSpeedToPosition() {
//   stepperRight.runSpeedToPosition();
//   stepperLeft.runSpeedToPosition();
// }

// /*function to run both wheels continuously at a speed*/
// void runAtSpeed ( void ) {
//   while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
//   }
// }

// /*This function, runToStop(), will run the robot until the target is achieved and
//    then stop it
// */
// void runToStop ( void ) {
//   int runNow = 1;
//   int rightStopped = 0;
//   int leftStopped = 0;

//   while (runNow) {
//     if (!stepperRight.run()) {
//       rightStopped = 1;
//       stepperRight.stop();//stop right motor
//     }
//     if (!stepperLeft.run()) {
//       leftStopped = 1;
//       stepperLeft.stop();//stop ledt motor
//     }
//     if (rightStopped && leftStopped) {
//       runNow = 0;
//     }
//   }
// }


// /*
//    The move1() function will move the robot forward one full rotation and backwared on
//    full rotation.  Recall that that there 200 steps in one full rotation or 1.8 degrees per
//    step. This function uses setting the step pins high and low with delays to move. The speed is set by
//    the length of the delay.
// */
// void move1() {
//   digitalWrite(redLED, HIGH);//turn on red LED
//   digitalWrite(grnLED, LOW);//turn off green LED
//   digitalWrite(ylwLED, LOW);//turn off yellow LED
//   digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
//   digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
//   // Makes 800 pulses for making one full cycle rotation
//   for (int x = 0; x < 800; x++) {
//     digitalWrite(rtStepPin, HIGH);
//     digitalWrite(ltStepPin, HIGH);
//     delayMicroseconds(stepTime);
//     digitalWrite(rtStepPin, LOW);
//     digitalWrite(ltStepPin, LOW);
//     delayMicroseconds(stepTime);
//   }
//   delay(1000); // One second delay
//   digitalWrite(ltDirPin, LOW); // Enables the motor to move in opposite direction
//   digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
//   // Makes 800 pulses for making one full cycle rotation
//   for (int x = 0; x < 800; x++) {
//     digitalWrite(rtStepPin, HIGH);
//     digitalWrite(ltStepPin, HIGH);
//     delayMicroseconds(stepTime);
//     digitalWrite(rtStepPin, LOW);
//     digitalWrite(ltStepPin, LOW);
//     delayMicroseconds(stepTime);
//   }
//   delay(1000); // One second delay
// }

// /*
//    The move2() function will use AccelStepper library functions to move the robot
//    move() is a library function for relative movement to set a target position
//    moveTo() is a library function for absolute movement to set a target position
//    stop() is a library function that causes the stepper to stop as quickly as possible
//    run() is a library function that uses accel and decel to achieve target position, no blocking
//    runSpeed() is a library function that uses constant speed to achieve target position, no blocking
//    runToPosition() is a library function that uses blocking with accel/decel to achieve target position
//    runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
//    runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
// */
// void move2() {
//   digitalWrite(redLED, LOW);//turn off red LED
//   digitalWrite(grnLED, HIGH);//turn on green LED
//   digitalWrite(ylwLED, LOW);//turn off yellow LED
//   stepperRight.moveTo(800);//move one full rotation forward relative to current position
//   stepperLeft.moveTo(800);//move one full rotation forward relative to current position
//   stepperRight.setSpeed(1000);//set right motor speed
//   stepperLeft.setSpeed(1000);//set left motor speed
//   stepperRight.runSpeedToPosition();//move right motor
//   stepperLeft.runSpeedToPosition();//move left motor
//   runToStop();//run until the robot reaches the target
//   delay(1000); // One second delay
//   stepperRight.moveTo(0);//move one full rotation backward relative to current position
//   stepperLeft.moveTo(0);//move one full rotation backward relative to current position
//   stepperRight.setSpeed(1000);//set right motor speed
//   stepperLeft.setSpeed(1000);//set left motor speed
//   stepperRight.runSpeedToPosition();//move right motor
//   stepperLeft.runSpeedToPosition();//move left motor
//   runToStop();//run until the robot reaches the target
//   delay(1000); // One second delay
// }

// /*
//    The move3() function will use the MultiStepper() class to move both motors at once
//    move() is a library function for relative movement to set a target position
//    moveTo() is a library function for absolute movement to set a target position
//    stop() is a library function that causes the stepper to stop as quickly as possible
//    run() is a library function that uses accel and decel to achieve target position, no blocking
//    runSpeed() is a library function that uses constant speed to achieve target position, no blocking
//    runToPosition() is a library function that uses blocking with accel/decel to achieve target position
//    runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
//    runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
// */
// void move3() {
//   digitalWrite(redLED, LOW);//turn off red LED
//   digitalWrite(grnLED, LOW);//turn off green LED
//   digitalWrite(ylwLED, HIGH);//turn on yellow LED
//   long positions[2]; // Array of desired stepper positions
//   positions[0] = 800;//right motor absolute position
//   positions[1] = 800;//left motor absolute position
//   steppers.moveTo(positions);
//   steppers.runSpeedToPosition(); // Blocks until all are in position
//   delay(1000);//wait one second
//   // Move to a different coordinate
//   positions[0] = 0;//right motor absolute position
//   positions[1] = 0;//left motor absolute position
//   steppers.moveTo(positions);
//   steppers.runSpeedToPosition(); // Blocks until all are in position
//   delay(1000);//wait one second
// }

// /*this function will move to target at 2 different speeds*/
// void move4() {
//   int leftPos = 5000;//right motor absolute position
//   int rightPos = 1000;//left motor absolute position
//   int leftSpd = 5000;//right motor speed
//   int rightSpd = 1000; //left motor speed
//   digitalWrite(redLED, HIGH);//turn on red LED
//   digitalWrite(grnLED, HIGH);//turn on green LED
//   digitalWrite(ylwLED, LOW);//turn off yellow LED
//   //Uncomment the next 4 lines for absolute movement
//   stepperLeft.setCurrentPosition(0);//set left wheel position to zero
//   stepperRight.setCurrentPosition(0);//set right wheel position to zero
//   stepperLeft.moveTo(leftPos);//move left wheel to absolute position
//   stepperRight.moveTo(rightPos);//move right wheel to absolute position
//   //Unomment the next 2 lines for relative movement
//   stepperLeft.move(leftPos);//move left wheel to relative position
//   stepperRight.move(rightPos);//move right wheel to relative position
//   //Uncomment the next two lines to set the speed
//   stepperLeft.setSpeed(leftSpd);//set left motor speed
//   stepperRight.setSpeed(rightSpd);//set right motor speed
//   runAtSpeedToPosition();//run at speed to target position
// }

// /*This function will move continuously at 2 different speeds*/
// void move5() {
//   digitalWrite(redLED, LOW);//turn off red LED
//   digitalWrite(grnLED, HIGH);//turn on green LED
//   digitalWrite(ylwLED, HIGH);//turn on yellow LED
//   int leftSpd = 5000;//right motor speed
//   int rightSpd = 1000; //left motor speed
//   stepperLeft.setSpeed(leftSpd);//set left motor speed
//   stepperRight.setSpeed(rightSpd);//set right motor speed
//   runAtSpeed();
// }

// void step_and_run(int speed){           //This function sets the steppers to a given speed and makes them run
//   stepperLeft.setSpeed(speed);
//   stepperRight.setSpeed(speed);
//   stepperLeft.run();
//   stepperRight.run();
// }

// void sonarRead(){
//   for (int i =0; i<=1; i++) {
//     pinMode(sonarL, OUTPUT);
//     digitalWrite(sonarL, LOW);
//     delayMicroseconds(2);
//     digitalWrite(sonarL, HIGH);
//     delayMicroseconds(5);
//     digitalWrite(sonarL, LOW);
//     pinMode(sonarL, INPUT);
//     Left_dist = Left_dist + pulseIn(sonarL, HIGH); // Gets the value for left sonar

//     pinMode(sonarR, OUTPUT);
//     digitalWrite(sonarR, LOW);
//     delayMicroseconds(2);
//     digitalWrite(sonarR, HIGH);
//     delayMicroseconds(5);
//     digitalWrite(sonarR, LOW);
//     pinMode(sonarR, INPUT);
//     Right_dist = Right_dist + pulseIn(sonarR, HIGH); // Gets the value for left sonar
//   }
//   Left_dist = Left_dist/2;          //Get the average of 2 readings of the left sonar for accuracy
//   Left_dist = (Left_dist-78)/157;   //Change the value into inches
//   Right_dist = Right_dist/2;          //Get the average of 2 readings of the right sonar for accuracy
//   Right_dist = (Right_dist-240)/160;  //Change the value into inches

//   // Serial.print("Left Sonar: "); Serial.print(Left_dist); Serial.print(" Right Sonar: "); Serial.println(Right_dist);

//   // if(Right_dist < 4){          //If the right sonar reads any object within 5 inches, set the flag on the obSRight bit, otherwise, clear it
//   //   bitSet(flag, obSRight);
//   // }
//   // else {
//   //   bitClear(flag, obSRight);
//   // }
//   // if(Left_dist < 5){          //If the left sonar reads any object within 5 inches, set the flag on the obSLeft bit, otherwise, clear it
//   //   bitSet(flag, obSLeft);
//   // }
//   // else {
//   //   bitClear(flag, obSLeft);
//   // }

// }


// void updateIR() {
//   float front = 0;                                //Set the initial values of all 4 directions to 0
//   float left=0;
//   float right =0;
//   float back =0;
//   for (int k =0; k<4; k++) {                      //Take the average of 5 readings of the sensors for an accurate result
//     front = front + analogRead(IR_front);
//     left = left + analogRead(IR_Left);
//     right = right + analogRead(IR_right);
//     back = back + analogRead(IR_back);
//   }
//   front = front/5;
//   back = back/5;
//   left = left/5;
//   right = right/5;

//   front = (1111/(front+3) - 1);                   //Calculate the distance of the object in inches for the front, back, left, and right sensors
//   back = (1116/(back+6) - 1);
//   left = (1447/(left-29) - 1);
//   right = (1379/(right-59) - 1);

//   if(front < 5 && front > 0) {                    //If an object is between 0 to 5 inches of the front sensor, set a flag on obFront bit and turn on red LED, otherwise, clear the flag
//     digitalWrite(redLED, HIGH);
//     digitalWrite(grnLED,LOW);
//     bitSet(flag, obFront);
//   }
//   else {
//     bitClear(flag, obFront);
//     digitalWrite(redLED, LOW);
//   }

//   if(back < 5 && back > 0) {                    //If an object is between 0 to 5 inches of the back sensor, set a flag on obBack bit and turn on red LED, otherwise, clear the flag
//     digitalWrite(redLED, HIGH);
//     digitalWrite(grnLED,LOW);
//     bitSet(flag, obBack);
//   }
//   else {
//     bitClear(flag, obBack);
//     digitalWrite(redLED, LOW);
//   }

//   if(left < 5 && left > 0) {                    //If an object is between 0 to 5 inches of the left sensor, set a flag on obLeft bit and turn on red LED, otherwise, clear the flag
//     digitalWrite(redLED, HIGH);
//     digitalWrite(grnLED,LOW);
//     bitSet(flag, obLeft);
//   }
//   else {
//     bitClear(flag, obLeft);
//     digitalWrite(redLED, LOW);
//   }

//   if(right < 5 && right > 0) {                    //If an object is between 0 to 5 inches of the right sensor, set a flag on obRight bit and turn on red LED, otherwise, clear the flag
//     digitalWrite(redLED, HIGH);
//     digitalWrite(grnLED,LOW);
//     bitSet(flag, obRight);
//   }
//   else {
//     bitClear(flag, obRight);
//     digitalWrite(redLED, LOW);
//   }
// }


// void agressive_state() {      //This function is the behavior for aggressive kid
//   if (bitRead(flag, obBack)) {
//     bitClear(state, backwards);
//     bitSet(state, collide);
//   }
//   else if(bitRead(flag, obFront)){
//     Serial.print("state with flag: "); Serial.println(state, BIN);
//     bitClear(state, backwards);
//     bitSet(state, collide);
//   }
//   else if (bitRead(flag,obLeft)) {
//     bitClear(state, backwards);
//     bitSet(state, collide);
//   }
// }
 
// void shy_state(){
//   state = sGoToGoal;
//   if (bitRead(flag, obBack) && bitRead(flag, obFront) && bitRead(flag, obLeft) && bitRead(flag, obRight)) {
//     state = collide;                 //If all 4 sensors are triggered, do not move
//   }
//   else if(bitRead(flag, obBack) && bitRead(flag, obLeft) && bitRead(flag, obRight)){
//     state = forwards;                //If the right, left, and back sensor is triggered, move forward
//   }
//   else if (bitRead(flag, obBack) && bitRead(flag, obFront) && bitRead(flag, obLeft)) {
//     state = rightwards;              //If the back,front, and left sensor is triggered, move to the right
//   }
//   else if (bitRead(flag, obBack) && bitRead(flag, obFront) && bitRead(flag, obRight)) {
//     state = leftwards;               //If the right, back, and front sensor is triggered, move to the left
//   }
//   else if (bitRead(flag, obFront) && bitRead(flag, obLeft) && bitRead(flag, obRight)) {
//     state=backwards;                 //If the right, left, and front sensor is triggered, move back
//   }
//   else if (bitRead(flag, obFront) && bitRead(flag, obRight)) {
//     state=leftwards;                 //If the right and front sensor is triggered, move to the left
//   }
//   else if (bitRead(flag, obBack) && bitRead(flag, obFront)) {
//     state=rightwards;                //If the back and front sensor is triggered, move to the right
//   }
//   else if (bitRead(flag, obLeft) && bitRead(flag, obFront)) {
//     state=rightwards;                //If the left and front sensor is triggered, move to the right
//   }
//   else if (bitRead(flag, obBack) && bitRead(flag, obRight)) {
//     state=forwards;                  //If the right and back sensor is triggered, move forward
//   }
//   else if (bitRead(flag, obLeft) && bitRead(flag, obRight)) {
//     state=forwards;                  //If the right and left sensor is triggered, move forward
//   }
//   else if (bitRead(flag, obBack) && bitRead(flag, obLeft)) {
//     state=forwards;                  //If the back and left sensor is triggered, move forward
//   }
//   else if (bitRead(flag, obBack)) {
//     state=forwards;                  //If the back sensor is triggered, move forward
//   }
//   else if(bitRead(flag, obFront)){
//     state=rightwards;                //If the front sensor is triggered, move to the right
//   }
//   else if (bitRead(flag, obLeft)){
//     state=rightwards;                //If the left sensor is triggered, move to the right             
//   }
//   else if (bitRead(flag, obRight)) {
//     state=leftwards;                 //If the right sensor is triggered, move to the left
//   }
// }


// void updateSensors(){
//   flag = 0;               //Do not set any flag on any of the bits by default
//   state = sGoToGoal;      //The default state is set to carry out the smart homing function
//   updateIR();             //Update IR reads the values from the IR sensors and sets flags accordingly
//   shy_state();            //The shy state avoids and obstacles it sees
//   sonarRead();            //Uses the sonar sensors and updates the distance of the objects seen by the sonar
// }


// /*
//   It turns the robot in the given direction a given number of revolutions, with the axis of rotation in one of the wheels.
//   The number of rotations is converted to pulses using the diameter of the wheels, the width of the robot and 
//   the number of pulses per feet, which yields the factor 4100.

//   @param direction clockwise (1) or counterclockwise (0) direction
//   @param spins number of full revolutions to execute

// */
// void pivot(int direction, float spins) {
//   spins=spins*4100; // The factor to go from revolutions to pulses was found to be 4100 pulses/rev
//   int speed = 900;  // Speed of motor turning
//   if(direction==1){  // Clockwise rotation
//     stepperLeft.setMaxSpeed(speed);
//     stepperLeft.move(spins);
//   }
//   else if(direction==0){  // Counter-clockwise rotation
//     stepperLeft.setMaxSpeed(speed);
//     stepperRight.move(spins);
//   }
//   steppers.runSpeedToPosition();
//   runToStop();
// }


// /*
//   It turns the robot in the given direction a given number of revolutions, with the axis of rotation in the center of the robot.
//   The number of rotations is converted to pulses using the diameter of the wheels, the width of the robot and 
//   the number of pulses per feet, which yields the factor 1997.
//   It does not use the AccelStepper library

//   @param direction clockwise (0) or counterclockwise (1) direction
//   @param spins number of full revolutions to execute
// */
// void spin(int direction, double spins) {
//   if (direction == 1) {  // Clockwise rotation
//     digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
//     digitalWrite(rtDirPin, LOW); // Enables the motor to move in a particular direction
//   }
//   else if (direction == 0) {  // Counter-clockwise rotation
//     digitalWrite(ltDirPin, LOW); // Enables the motor to move in a particular direction
//     digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
//   }
//   else { // If there is a wrong input, let the user know 
//     printf("Insert 1(clockwise) or 0(counter cockwise)");
//   }
//   if (direction == 0) {             //This if-else statement tracks the angle at which the robot is facing relative to the positive X-axis
//     orient += spins * 360;
//   }
//   else {
//     orient -= spins * 360;
//   }

//   spins = spins*1997;  // This factor is the number of pulses of each motor for a full revolution
//   for (int x = 0; x < spins; x++) { // Makes the motors move the pulses needed to complete the spins
//     digitalWrite(ltStepPin, HIGH); 
//     delayMicroseconds(1000);
//     digitalWrite(rtStepPin, HIGH);
//     delayMicroseconds(stepTime);
//     digitalWrite(rtStepPin, LOW);
//     digitalWrite(ltStepPin, LOW);
//     delayMicroseconds(1000);
//     delayMicroseconds(stepTime);
//   }

// }

// /*
//   It turns the robot in the given direction a given number of revolutions following a circle of a given diameter.
//   The number of rotations is converted to pulses using the width of the robot, the diameter of the circle and the 
//   number of 912 pulses per feet.
//   The outer wheel always has a speed of 800 and the inner wheel speed is proportional to the ratio of the distances
//   the inner and outer wheel travel.

//   @param direction clockwise (0) or counterclockwise (1) direction
//   @param spins number of full revolutions to execute
//   @param diameter diameter of the turn to execute
// */
// void turn(int direction, float spins, float diameter) {
// float width = 8.464;                // Width of the robot
// float ang = spins * 2 * 3.142;      // Rotation angle to travel
// float s2 = (diameter/2) * ang;      // Inner circumference travelled by the inner wheel in inches
// float s1 = s2 + (width * ang);      // Outer circumference travelled by the outer wheel in inches
// float p2 = s2 * (912/12);    // Inner circumference travelled by the inner wheel in steps
// float p1 = s1 * (912/12);    // Outer circumference travelled by the outer wheel in steps
// float speed = 800;                  // Speed of outer wheel
// float new_speed1 = speed;           // Speed of outer wheel
// float new_speed2 = speed * (p2/p1); // Speed of inner wheel, proportional to the circuference
// if (direction == 1) { // Clockwise rotation
//   stepperLeft.move(p1);
//   stepperRight.move(p2);
//   stepperLeft.setSpeed(new_speed1);
//   stepperRight.setSpeed(new_speed2);
// }

// else if (direction == 0){  // Counter-clockwise rotation
//   stepperLeft.move(p2);
//   stepperRight.move(p1);
//   stepperLeft.setSpeed(new_speed2);
//   stepperRight.setSpeed(new_speed1); 
// }
// steppers.runSpeedToPosition();
// }


// /*
//   Moves the robot a given distance in feet.
//   It uses a factor of 912 pulses per feet. This was calculated with the diameter of the wheels and
//   adjusted experimentally.
//   @param distance distance to travel in feet.
// */
// void Goforward(float distance) {
//   float currentL = encoder[0];    //Set initial variables for left and right motor encoders
//   float currentR = encoder[1];
//   while (encoder[0] < currentL + distance*45 || encoder[1] < currentR + distance * 46) {    //While loop goes reads the encoders and keeps running till it reaches the length
//     stepperLeft.setSpeed(600);        //Set speed for the motors 
//     stepperRight.setSpeed(600);
//     stepperLeft.run();                //make the motors run
//     stepperRight.run();
//   }
//   current_x += distance*cos(orient*PI/180);       //This line tracks the current X position of the robot for smart homing
//   current_y += distance*sin(orient*PI/180);       //This line tracks the current Y position of the robot for smart homing

// }



// /*
//   Moves the robot a given distance in feet backwards.
//   It uses a factor of 912 pulses per feet. This was calculated with the diameter of the wheels and
//   adjusted experimentally.
//   @param distance distance to travel in feet.
// */
// void reverse(int distance) {
//   float currentL = encoder[0];    //Set initial variables for left and right motor encoders
//   float currentR = encoder[1];
//   while (encoder[0] < currentL + distance*45 || encoder[1] < currentR + distance * 46) {    //While loop goes reads the encoders and keeps running till it reaches the length
//     stepperLeft.setSpeed(-600);        //Set speed for the motors 
//     stepperRight.setSpeed(-600);
//     stepperLeft.run();                //make the motors run
//     stepperRight.run();
//   }
// }
// /*
//   It stops both motors.
// */
// void stop() {
//   stepperLeft.stop();
//   stepperRight.stop();
// }


// /*
//   This function makes the robot move in a circle of a given diameter in a given direction. It turns the red LED on.
//   The diameter is the inner diameter.
//   It uses the turn() funtion with 1 spin.

//   @param diam diameter of the circle in inches
//   @param dir  direction of turning (1 is CW, 0 is CCW)
// */
// void moveCircle(int diam, int dir) {
//   digitalWrite(redLED, HIGH); // Turns red LED on
//   turn(dir,1,diam);           // Calls the turn function for one revolution with given diameter and direction
//   digitalWrite(redLED, LOW);  // Turns red LED off
// }

// /*
//   The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
//   twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
//   It turns teh red and yellow LEDs on.

//   @param diam diameter of each of the circles of the figure 8 in feet.
// */
// void moveFigure8(int diam) {
//   digitalWrite(redLED, HIGH);  // Turns red LED on
//   digitalWrite(ylwLED, HIGH);  // Turns yellow LED on
//   moveCircle(diam, 1);         // Calls the circle function in the clockwise direction with the given diameter
//   moveCircle(diam, 0);         // Calls the circle function in the counter-clockwise direction with the given diameter
//   digitalWrite(redLED, LOW);   // Turns red LED off
//   digitalWrite(ylwLED, LOW);   // Turns yellow LED off

// }

// /*
//   It makes the robot spin to point about  in a clockwise direction.
//   It uses the encoders to reduce the odometry errors.

//   @param angle The goal angle in degrees
// */
// void goToAngle(float angle) {
//   int speed = 600;
//   float currentL = encoder[0];                                                //Set initial variables for left and right motor encoders
//   float currentR = encoder[1];
//   float ticks_per_degree = 102./360.;                                         //converts degrees into ticks per one degree
//   if(angle > 180) {
//     speed = -speed;
//     angle = 360-angle;
//   }
//   angle = angle * ticks_per_degree;                                           //convert angle from degrees to ticks
//   while (encoder[0] < currentL + angle || encoder[1] < currentR + angle) {    //While loop will keep the robot running till it reaches the desired number of ticks
//     stepperLeft.setSpeed(-speed);                                                //Set speed for the motors 
//     stepperRight.setSpeed(speed);
//     stepperLeft.run();                                                        //make the motors run
//     stepperRight.run();
//   }

// }


// /*
// This function takes the x and y coordinate and moves the robot to that location.
// It uses the atan() command to calculate the angle and the Pythagoras theorem to calculate the distance to travel.
// Uses the subfuntions goToAngle() and forward() defined above
// The robot should start facing the positive y axis

// @param    x   The x coordinate of the goal
// @param    y   The y coordinate of the goal.
// */
// void goToGoal(float x, float y) {
//   float angle = atan2(y,x)*180/PI;              // Calculate the angle it needs to point to
//   Serial.print("Angle: "); Serial.println(angle);
//   if (angle < 0) {                                  // The goal position is in the third or forth quadrant
//     angle = angle + 360;                         // Make the angle positive
//     Serial.print(" New Angle: "); Serial.println(angle);

//   }
//   float disToTravel = sqrt(x*x+y*y);            // Calculates the distance to travel using the pythagoras theorem
//   goToAngle(angle);                             // Uses the goToAngle to pivot to the appropiate direction
//   Goforward(disToTravel);                         // Moves the distance to the goal
// }


// /*
// The robot moves in a square with a side of a given length.
// It corrects for odometry error using the encoders
// It uses the forward(), goToAngle()

// @param    length length of each of the sides of the square in feet
// */

// void square(int len) {
//   Goforward(len);       //This function calls the forward and go to angle function 4 times to complete a square
//   goToAngle(90);
//   Goforward(len);
//   goToAngle(90);
//   Goforward(len);
//   goToAngle(90);
//   Goforward(len);
//   goToAngle(90);
// }

// void setGoal(float x, float y) {            // This function sets the global variables final_x and final_y to the goal we want
//   final_x = x;
//   final_y = y;
// }

// void wander(){
//   if (wanderloops < 1){ 
//     wander_n = rand() % 6;              // Select a random number between 0 and 5
//     wanderloops=10;                     //Create a variable wander loop. This variable is here so that the wander function will get a new random number once every 10 runs
//   }
//   if (wander_n == 0){                   //If n is 0, call the forward function
//     Goforward(0.01);
//   }//end if 0
//   else if (wander_n == 1) {                   //If n is 1, call the reverse function
//     reverse(0.01);
//   }//end if 1
//   else if (wander_n == 2) {                   //If n is 2, call the turn function in the clockwise direction
//     stop();
//     turn(1, 0.01, 20);
//   }//end if 2
//   else if (wander_n == 3) {                   //If n is 3, call the turn function in the counter clockwise direction
//     stop();
//     turn(0, 0.01, 20);
//   }//end if 3
//   else if (wander_n == 4) {                   //If n is 4, call the pivot function in the counter clockwise direction
//     stop();
//     pivot(0, 0.01);
//   }//end if 4
//   else if (wander_n == 5) {                   //If n is 5, call the pivot function in the clockwise direction
//     stop();
//     pivot(1, 0.01);
//   }//end if 5
//   wanderloops--;                   //subtract the value of wander loop by 1
// }

// void smartGoToGoal() {  //The robot should start facing the postive x axis
//   if(abs(current_x - final_x) < 0.1 && abs(current_y - final_y) < 0.1) {
//     stop();                                                                 //If the robot is within 2 inches of the goal, stop it and break out of the function
//     return;
//   }
//   float angle = atan2(double(final_y-current_y),double(final_x-current_x))*180/PI;     // Calculate the angle it needs to point to
//   float new_angle = angle-orient;                                                      //Calculate the new angle and taking the orientation (absolute angle) of the robot in mind
//   if (new_angle < 0) {                                                                // If the new angle is less than 0
//     new_angle = new_angle + 360;                                                      // Add 360 to the angle
//   }
//   goToAngle(new_angle);                           //Go to the angle that is calculated above
//   orient = new_angle + orient;                    // update the orient valriable to get the angle at which the robot is currently facing
//   if (orient >360){
//     orient = orient - 360;                        //adjust the value of orient if it goes above 360
//   }
//   Goforward(0.05);                                // Call the forward function for 0.05 feet, and check the sensors for any obstacles
// }

// void followRight() {
//   if (Right_dist > 6) {
//     //LEDs
//     //get closer
//   }
//   else if (Right_dist < 4) {
//     //LEDs
//     //get farther
//   }
//   else {
//     state = forwards;
//   }

// }


// void moveRobot() {
//   digitalWrite(redLED, LOW);            //Start with all LEDs turned off
//   digitalWrite(ylwLED, LOW);
//   digitalWrite(grnLED, LOW);
//   if(state == collide) {              //If the state is set to collide, call the stop function
//     stop();
//     digitalWrite(redLED, HIGH);
//     state = sGoToGoal;                // Set the state to the smart homing
//   }
//   else if (state == forwards)         //If the state is set to forwards, call the forward function
//   {
//     digitalWrite(redLED, HIGH);       //Set red LED to high
//     Goforward(1);
//     state = sGoToGoal;                // Set the state to the smart homing
//   }
//   else if (state == backwards){       //If the state is set to backwards, call the reverse function
//     digitalWrite(redLED, HIGH);       //set red LED to high
//     reverse(1);
//     state = sGoToGoal;                // Set the state to the smart homing
//   }
//   else if (state == rightwards) {     //If the state is set to rightwards, call the spin function, then the forward function
//     digitalWrite(redLED, HIGH);       //set red LED to high
//     spin(1, 0.25);
//     Goforward(1);
//     state = sGoToGoal;                // Set the state to the smart homing
//   }
//   else if (state == leftwards) {      //If the state is set to leftwards, call the spin function, then the forward function
//     digitalWrite(redLED, HIGH);       //set red LED to high
//     spin(0, 0.25);
//     Goforward(1);
//     state = sGoToGoal;                // Set the state to the smart homing
//   }
//   else if (state == rest) {           //If the state is set to rest, call the stop function
//     stop();
//     digitalWrite(redLED, LOW);        //turn red LED off
//   }
//   else if (state == random) {         //If state is on random, call the random wander function
//     digitalWrite(grnLED, HIGH);       //turn the green LED on
//     wander();                         //Cal the wander function
//   }
//   else if (state == sGoToGoal) {      //If the state is on sGoToGoal, call the smart homing function
//     digitalWrite(grnLED, HIGH);       //Turn on all 3 LEDs
//     digitalWrite(ylwLED, HIGH);
//     digitalWrite(redLED, HIGH);
//     smartGoToGoal();                  //Call the Smart homing function
//   }

// }




// // MAIN
// void setup()
// {
//   int baudrate = 9600;        //serial monitor baud rate'
//   int BTbaud = 9600;          // HC-05 default speed in AT command more
//   init_stepper();             //set up stepper motor
//   attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
//   attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder
//   BTSerial.begin(BTbaud);     //start Bluetooth communication
//   Serial.begin(baudrate);     //start serial monitor communication
//   // Timer1.initialize(1000000);
//   // Timer1.attachInterrupt(updateSensors);
//   while (!Serial)
//     delay(10); // will pause until serial console opens
//   // init_BT(); //initialize Bluetooth
//   // init_IMU(); //initialize IMU
//   Serial.println("Robot starting...");
//   Serial.println("");
//   delay(pauseTime); //always wait 2.5 seconds before the robot moves
// }


// void loop()
// {
//   // setGoal(0, -4);   //Set Goal coordinates for the Smart homing function
//   // moveRobot();      //Call the moveRobot function
//   sonarRead();

//   // print_encoder_data();     //prints encoder data   [45 ticks = 1 Left revolution, 46 ticks = 1 Right revolution]
//   // print_IMU_data();         //print IMU data
//   //Bluetooth_comm();          //Bluetooth connection
//   delay(wait_time);         //wait to move robot or read data
// }