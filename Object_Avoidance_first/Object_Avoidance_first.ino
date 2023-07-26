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

  bool detectedObject = detectObject(centerIRValue) || detectObject(leftIRValue) || detectObject(rightIRValue);  
  

  //determine if the robot should stop or keep going
  if(detectedObject) {
    avoidObject();
  } else {
    driveRobotLine(leftOnline, rightOnline);
  }
  
}

  //I really dislike having so many global variables,
  //this is probably a better use case for static, but
  //I'm trying not to change too much
  bool initiatedLeft = false;
  bool initiatedRight = false;
  bool initiatedHardLeft = false;
  bool initiatedHardRight = true;
  int counter = 0;
  int straightcounter = 0;
  int temp = 0;
  int maxSpeed = 145;
  int rightspeed = maxSpeed;
  int leftspeed = maxSpeed;

/*
 * Determines how to steer the robot given readings from the line sensors
 */
void driveRobotLine(bool leftOnline, bool rightOnline) {
  if (!leftOnline && rightOnline) {
    
    if (initiatedHardLeft) { //turn right
      leftspeed = maxSpeed;
      rightspeed = maxSpeed - 40;
      initiatedLeft = false;
      initiatedRight = true;      
    }
    
    //this is turning left
    if (!initiatedLeft) { 
      leftspeed = maxSpeed - 30;
      rightspeed = maxSpeed;
      initiatedLeft = true;
      initiatedRight = false;
    }
    straightcounter = 0;
    if (counter < 150) { //limit
      leftspeed -= 1; //rate
    } else {
      leftspeed = -30;
    }
    counter++;
  } 
  else if (leftOnline && !rightOnline) {

    if (initiatedHardRight){ //turn left
      leftspeed = maxSpeed - 40;
      rightspeed = maxSpeed;
      initiatedLeft = true;
      initiatedRight = false;
    }

    //this is turning right
    if (!initiatedRight) {
      leftspeed = maxSpeed;
      rightspeed = maxSpeed - 30;
      initiatedLeft = false;
      initiatedRight = true;
    }
    straightcounter = 0;
    if (counter < 150) { //limit
      rightspeed -= 1; //rate
    } else {
      rightspeed = -30;
    }
    counter++;
  } 
  else if (leftOnline && rightOnline) { //both off- turn in last remembered direction
    //if no last remembered direction? 
    //should not reach this case often (if at all), at least in first two courses, because 
    
      if (initiatedLeft && !initiatedHardLeft) {
        leftspeed = maxSpeed - 100;
        rightspeed = maxSpeed;
        initiatedHardLeft = true;
      }
      if (initiatedHardLeft) {
        if (leftspeed < 30 && leftspeed > -40) {
          leftspeed = -45;
        }
        if (counter < 60/2) { //limit
          leftspeed -= 2; //rate ****NEEDS to be increased with maxSpeed increase***
          rightspeed = maxSpeed - 10;
        }
        else {
          leftspeed = -90;
          rightspeed = maxSpeed - 10; 
        }
      }

      if (initiatedRight && !initiatedHardRight) {
        rightspeed = maxSpeed - 100;
        leftspeed = maxSpeed;
        initiatedHardRight = true;
      }
      if (initiatedHardRight) {
        if (rightspeed < 30 && rightspeed > -40) {
          rightspeed = -45;
        }
        if (counter < 60/2) { //limit
          rightspeed -= 2; //rate ****NEEDS to be increased with maxSpeed increase***
          leftspeed = maxSpeed - 10;
        }
        else {
          rightspeed = -90;
          leftspeed = maxSpeed - 10; 
        }
       }
       
    counter++;
  }
  else { //both sensors on line- continue straight

    leftspeed = maxSpeed;
    rightspeed = maxSpeed;
    initiatedLeft = false;
    initiatedRight = false;
    initiatedHardLeft = false;
    initiatedHardRight = false;
        
    if (straightcounter < counter && counter < 4) {
      if (straightcounter == 0){
        temp = leftspeed;
        leftspeed = rightspeed;
        rightspeed = temp;
      }
      
    } else {
    straightcounter = 0;
    counter = 0;
    leftspeed = maxSpeed;
    rightspeed= maxSpeed; //can use turn function with same speed- should be straight
    }
    straightcounter++;
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
  //lump in if we just detect the center so we pick a direction of travel
  bool detectedLeft = detectObject(detectObject(digitalRead(leftIR)) || detectObject(digitalRead(centerIR))); 
  if(detectedLeft) {
    avoidLeft();
  } else {
    avoidRight();
  }
}

//This function is meant to prevent the bug that the robot thinks it has already moved far enough left/right away from the object, but it has not
//Prevents the robot from hitting into the object by blindly going straight
void checkWhileMoving(const char* moveDir) {
  if (!strcmp(moveDir, "Right") && !strcmp(moveDir, "Left")){
    Serial.println("String compare is wrong. Need to figure out");
  }
  unsigned long endTime = millis() + 2000;
    moveStraight();
  while (millis() < endTime) {
      bool detectedObject = detectObject(digitalRead(leftIR)) || detectObject(digitalRead(centerIR)) || detectObject(digitalRead(rightIR));
      if (detectedObject && strcmp(moveDir, "Right")){
        while(detectedObject){
          stepRight();
          detectedObject = detectObject(digitalRead(leftIR)) || detectObject(digitalRead(centerIR)) || detectObject(digitalRead(rightIR));
        }
      }
      else if (detectedObject){
        while(detectedObject){
          stepLeft();
          detectedObject = detectObject(digitalRead(leftIR)) || detectObject(digitalRead(centerIR)) || detectObject(digitalRead(rightIR));
        }
      }
      moveStraight();
  }
}

//Steers the robot left
void avoidLeft() {
  moveStraight();
  delay(50);
  bool detectedObject = detectObject(digitalRead(centerIR)) || detectObject(digitalRead(leftIR)) ||detectObject(digitalRead(rightIR)); 
  while (detectedObject){
    stepLeft();
    detectedObject = detectObject(digitalRead(centerIR)) || detectObject(digitalRead(leftIR)) ||detectObject(digitalRead(rightIR));
  }
  stepLeft();
  delay(300);
  int timer = 0;
  int rightLineValue = analogRead(rightLine);
  while (timer < 100)
  {
    moveStraight();
    timer += 1;
    rightLineValue = analogRead(rightLine);
    if (!offLine(rightLineValue))
    {
      timer = 9999;
    }
    delay(10);
  }

  
  while (offLine(rightLineValue)){
    stepRight();
    rightLineValue = analogRead(rightLine);
  }
}

//Steers the robot right
void avoidRight() {
  moveStraight();
  delay(50);

  bool detectedObject = detectObject(digitalRead(centerIR)) || detectObject(digitalRead(leftIR)) ||detectObject(digitalRead(rightIR)); 
  while (detectedObject){
    stepRight();
    detectedObject = detectObject(digitalRead(centerIR)) || detectObject(digitalRead(leftIR)) ||detectObject(digitalRead(rightIR));
  }
  stepRight();
  delay(300);
  int timer = 0;
  int leftLineValue = analogRead(leftLine);
  while (timer < 100)
  {
    moveStraight();
    timer += 1;
    leftLineValue = analogRead(leftLine);
    if (!offLine(leftLineValue))
    {
      timer = 9999;
    }
    delay(10);
  }

  
  while (offLine(leftLineValue)){
    stepLeft();
    leftLineValue = analogRead(leftLine);
  }
}

void stepLeft(){
  leftFront.run(-100);
  rightFront.run(100);
  rightRear.run(-100);
  leftRear.run(100);
}

void stepRight(){
  leftFront.run(100);
  rightFront.run(-100);
  rightRear.run(100);
  leftRear.run(-100);
}

void moveStraight(){
  leftFront.run(maxSpeed);
  rightFront.run(maxSpeed);
  rightRear.run(maxSpeed);
  leftRear.run(maxSpeed);
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
