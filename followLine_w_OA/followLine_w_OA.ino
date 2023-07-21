/* 
 * Still have to add IR sensors into this program.
 * Right now, only line follower values are taken into account, must look at turn speeds.
 * IR sensors WIP at time of comment.
 */

#include "MeMegaPi.h"
#include "SoftwareSerial.h"


//define the pin associated with the various sensors
#define leftLine A9
#define rightLine A10
#define leftIR A6
#define centerIR A7
#define rightIR A8



MeMegaPiDCMotor leftFront(PORT1A); //front left  //want to rename to leftFront, rightRear, etc.
MeMegaPiDCMotor rightFront(PORT1B); //front right
MeMegaPiDCMotor rightRear(PORT2A); //rear right
MeMegaPiDCMotor leftRear(PORT2B); //rear left
MeInfraredReceiver infraredReceiverDecode(PORT_6);



void setup() {

  Serial.begin(9600);

  infraredReceiverDecode.begin();

  pinMode(leftLine, INPUT); //left line follower
  pinMode(rightLine, INPUT); //right line follower
  pinMode(leftIR, INPUT); //left IR
  pinMode(centerIR, INPUT); //center IR
  pinMode(rightIR, INPUT); //right IR
}



void loop() {
  //read in values from the sensor
  int leftLineValue = analogRead(leftLine);
  int rightLineValue = analogRead (rightLine);
  int leftIRValue = digitalRead(leftIR);
  int centerIRValue = digitalRead(centerIR);
  int rightIRValue = digitalRead(rightIR);

  Serial.print("LIR =");
  Serial.println(leftIRValue);
  Serial.print("CIR =");
  Serial.println(centerIRValue);
  Serial.print("RIR =");
  Serial.println(rightIRValue);


  bool leftOnline = offLine(leftLineValue);
  bool rightOnline = offLine(rightLineValue);
  
  //This will short circuit, so since the center IR is "most important",
  //put that first in the list

  bool detectedObject = detectObject(centerIRValue) || detectObject(leftIRValue) ||detectObject(rightIRValue);  
  //bool detectedObject = false;

  //determine if the robot should stop or keep going
  if(detectedObject) {
    avoidObject();
  } else {
    driveRobotLine(leftOnline, rightOnline);
  }
  
  delay(100);

}

  //I really dislike having so many global variables,
  //this is probably a better use case for static, but
  //I'm trying not to change too much
  bool wasLeft = true;
  int rightspeed = 85;
  int leftspeed = 85;
  int counter = 0;
  int straightcounter = 0;
  int temp = 0;

/*
 * Determines how to steer the robot given readings from the line sensors
 */
void driveRobotLine(bool leftOnline, bool rightOnline) {
  if (!leftOnline && rightOnline) {
    //this is turning left
    straightcounter = 0;
    rightspeed += 8;
    if (counter < 4) {
      leftspeed -= 8;
    } else {
      leftspeed = -30;
    }
    counter++;
    //leftspeed -= 2;//make left motors slower- play around with speed
    wasLeft = true;
  } 
  else if (leftOnline && !rightOnline) {
    //this is turning right
    straightcounter = 0;
    if (counter < 4) {
      rightspeed -= 8;
    } else {
      rightspeed = -30;
    }
    counter++;
    leftspeed += 8; //make right motors slower- play around with speed
    wasLeft = false;
  } 
  else if (leftOnline && rightOnline) { //both off- turn in last remembered direction
    //if no last remembered direction? 
    //should not reach this case often (if at all), at least in first two courses, because 
    if (wasLeft) {
      leftspeed = -30;
      rightspeed = 75;
    }
    else {
      leftspeed = 75;
      rightspeed = -30;
    }
  }
  else { //both sensors on line- continue straight
    if (straightcounter < counter && counter < 4) {
      if (straightcounter == 0){
        temp = leftspeed;
        leftspeed = rightspeed;
        rightspeed = temp;
      }
      straightcounter++;
    } else {
    straightcounter = 0;
    counter = 0;
    leftspeed = 85;
    rightspeed= 85; //can use turn function with same speed- should be straight
    }
  }
  turn(leftspeed, rightspeed);
}

void turn(int leftSpeed, int rightSpeed) {
  //turn left motors by leftSpeed
  leftFront.run(leftSpeed);
  rightFront.run(rightSpeed);
  leftRear.run(leftSpeed);
  rightRear.run(rightSpeed);
}

void avoidObject(){
  int loops = 0;
  bool detectedObject = detectObject(digitalRead(centerIR)) || detectObject(digitalRead(leftIR)) ||detectObject(digitalRead(rightIR));  
  while (detectedObject){
    stepRight();
    detectedObject = detectObject(digitalRead(centerIR)) || detectObject(digitalRead(leftIR)) ||detectObject(digitalRead(rightIR));  
    loops += 3;
  }
  stepRight();
  delay(1000);
  moveStraight();
  delay(2000);
  while (loops > 0){
    stepLeft();
    loops -= 1;
  }
  stepLeft();
  delay(1000);
  moveStraight();
  delay(2000);
  stopMove();
  delay(10000);
}

void stepLeft(){
  leftFront.run(-85);
  rightFront.run(85);
  rightRear.run(-85);
  leftRear.run(85);
}

void stepRight(){
  leftFront.run(85);
  rightFront.run(-85);
  rightRear.run(85);
  leftRear.run(-85);
}

void moveStraight(){
  leftFront.run(85);
  rightFront.run(85);
  rightRear.run(85);
  leftRear.run(85);
}

void stopMove(){
  leftFront.run(0);
  rightFront.run(0);
  rightRear.run(0);
  leftRear.run(0);
}

/*
 * Given a reading, determines if a line sensor is on or off the line
 */
boolean offLine(int sensor) {
  if (sensor) {
    return true;
  }
  return false;
}

/*
 * Given a reading, determines if IR sensors detect an object
 */
boolean detectObject(int sensor) {
  if (!sensor) {
    return true;
  }
  return false;
}
