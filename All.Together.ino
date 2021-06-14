// Final Code Modifications

#define LEncoderPin 3     
#define REncoderPin 2

#include <AFMotor.h>
#include <PID_v2.h> 

AF_DCMotor left_motor(1, MOTOR12_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(3, MOTOR34_1KHZ); // right motor to M3 on motor control board

int LeftEncoderCount = 0;
int RightEncoderCount = 0;
int countDiff = 0;
int delayTime = 200;

int leftSpeed;
int rightSpeed;
int leftSpeedSet = 150;
int rightSpeedSet = 150;
int turnR = 13;
int turnL = 12;
int turnA = 25;

float averagePulse;
float destDis = 300;
float distance;
int prevIR;
int vStop = 1;

int avgTrans;
int transItera;
int cumTrans;
int refirFlag;

int maxWidth = 0;

float width1; // x
float depth1; // y
float width2;
float depth2;
float obst1;    //distance from start to obatacle 1
float obst2;    //distance between obstacle 1/2
float destination;  //distance from obstacle 1/2 to destination
int obst1Flag;    //1 for engaged obst 1
int obst2Flag;    //1 for engaged obst 2

int TicksPerRot = 20;

//global variables for final search applications
int right = 90;
int left = -90;
int halfLength;
int sideLength;
int angle = 0;


void setup() {
  Serial.begin (115200);
  pinMode(A0, INPUT); // left IR
  pinMode(A1, INPUT); // right IR
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, OUTPUT); // triger
  pinMode(A5, INPUT); // echo
 
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);

  attachInterrupt(1, countLEncoder, RISING); //calls on any CHANGE on pin 3 (interrupt #1, soldered to Pin3) 
  attachInterrupt(0, countREncoder, RISING); //calls on any CHANGE on pin 2 (interrupt #0, connected to header on Pin2) 
  interrupts();

  cumTrans = 0;
  transItera = 0;
  avgTrans = 0;
  vStop = 1;
  refirFlag = 0;
  
  //startCheck();
  //Serial.println("test1");
  
  wait(3000); // delay for preperation after setup check
}

void loop() {
  //Serial.println("test2");
  //destDis = 10 * 6.5 * 3.14; //cm
  destDis = 300;
  while(vStop == 1)
  {
    //Serial.println("test3");
    cumTrans = 0;
    avgTrans = 0;
    transItera = 0;
    destination = 0;
    destination = driveStraight(destDis); // forward to marker
    //if (vStop == 0)   break; // bottom marker detected
    while(1){};
    if (obst2Flag = 1) // obst 2 engaged
    {
      destination = (destination - obst1 - obst2);
    }
    else // only obst 1 engaged
    {
      destination = destination - obst2;
    }
    
    backUp(3);
    turnRight(33);
    wait(300);
    while(1){};
    driveStraight(destDis); // out of end marker area
    vStop = 0;
  }
  while (1){};
}

void avoidObstacles()
{
  wait(delayTime);
  backUp(3);
  wait(500);
  turnLeft(12); // facing left
  wait(500);
  if (obst2Flag == 0) // -x travel
  {
    width1 = overcomeCheck() + 25;
  }
  else // now on obst 2
  {
    width2 = overcomeCheck() + 25;
  }
  simpleStraight(25);
  
  wait(500);
  turnRight(13); // facing forward
  wait(500);
  if (obst2Flag == 0) // +y travel
  {
    simpleStraight(15);
    depth1 = overcomeCheck() + 40;
  }
  else // obst 2
  {
    simpleStraight(15);
    depth2 = overcomeCheck() + 40;
  }
  simpleStraight(25);
  
  wait(500);
  // overcome the obstacle
  turnRight(13); // +x travel
  wait(500);
  if (obst2Flag == 0)
  {
    simpleStraight(width1);
  }
  else // obst 2
  {
    simpleStraight(width2);
  }
  wait(500);
  turnLeft(13); // back to m line
  wait(700);
  //  while (1){};
}

float overcomeCheck ()
{
  int OCC = 10;
  wait(500);
  simpleStraight(10);
  if (getVoltage(A0) < 1 || getVoltage(A1) < 1) // obstacle on the side
    {
      OCC += overcomeCheck();
    }
  //simpleStraight(10);
  //OCC += 10;
  return OCC; // return overall distance between a turn and overcome
}


float driveStraight(long i) 
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
  left_motor.setSpeed(145);
  leftSpeed = 145;
  right_motor.setSpeed(155);
  rightSpeed = 155;
  prevIR = refIR();
  refirFlag = 0;
  
  int totalDistance = 0;
  
  float dist;
  unsigned long speedAdjCount = millis();
  unsigned long speedAdjInterval = 0;
  unsigned long refspeedAdjCount = millis();
  unsigned long refspeedAdjInterval = 0;
  
  while (distance < i)
  {
      countDiff = LeftEncoderCount - RightEncoderCount;
      unsigned int input = (int) round (0.8 * countDiff);

      speedAdjInterval = millis() - speedAdjCount;
      if (speedAdjInterval > 50)
      {
        if (LeftEncoderCount > RightEncoderCount)
      {
        if (rightSpeed > 150)
        {
          leftSpeed = leftSpeed - 2;
        }
        else if (leftSpeed < 150)
        {
          rightSpeed = rightSpeed + 2;
        }
        else
        {
          leftSpeed = leftSpeed - 4;
          rightSpeed = rightSpeed + 4;
        }
        
      }
      if (LeftEncoderCount < RightEncoderCount)
      {
        if (leftSpeed > 150)
        {
          rightSpeed = rightSpeed - 2;
        }
        else if (rightSpeed < 150)
        {
          leftSpeed = leftSpeed + 2;
        }
        else
        {
          rightSpeed = rightSpeed - 4;
          leftSpeed = leftSpeed + 4;
        }
      }
      //dist = ultraSonic();
      /*
      if (dist < 25)
      {
        int leftLow = leftSpeed / 3 * 2;
        int rightLow = rightSpeed / 3 * 2;
        left_motor.setSpeed(leftLow);
        right_motor.setSpeed(rightLow);
      }
      else
      {
        left_motor.setSpeed(leftSpeed);
        right_motor.setSpeed(rightSpeed);
      }
      */

      left_motor.setSpeed(leftSpeed);
      right_motor.setSpeed(rightSpeed);
      
      Serial.print("\nleft speed: ");
      Serial.print(leftSpeed);
      Serial.print("  right speed: ");
      Serial.print(rightSpeed);
       
      averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
      distance = (averagePulse / 40) * 3.14 * 6.5;
      Serial.print("\nLEncoder: ");
      Serial.print(LeftEncoderCount);
      Serial.print(", REncoder: ");
      Serial.println(RightEncoderCount);
      
      Serial.print(" diff: ");
      Serial.print(countDiff);
      Serial.print("\ndistance traveled: ");
      Serial.print(distance);
      Serial.print("\n");
        
        refirFlag = 1;
        speedAdjCount = millis();
        speedAdjInterval = 0;
      }

      dist = ultraSonic();
      if (dist < 7)   // getVoltage(A0) < 1 || getVoltage(A1) < 1 
      {
        LeftEncoderCount = 0;
        RightEncoderCount = 0;
        left_motor.run(RELEASE);
        right_motor.run(RELEASE);
        float distanceTemp = distance;
        //
        if (obst1Flag == 1) // obst 1 has been engaged
        {
          obst2Flag = 1; // now engaging obst 2
        }
        
        if (obst2Flag == 1) // encountered obst 2
        {
            obst2 = distanceTemp - obst1; // distance from satrt to obstacle 2
        }
        else
        {
          obst1 = distanceTemp; // engaging obst 1 if all values are unused
        }
        obst1Flag = 1; // engaing obst 1
        //
        int distY = reactObstacles(); // modified from avoidObstacles, distY distance along Y
        left_motor.run(RELEASE);
        right_motor.run(RELEASE);
        //vStop = 0;
        //break;
        left_motor.run(FORWARD);
        right_motor.run(FORWARD);
        left_motor.setSpeed(141);
        leftSpeed = 141;
        right_motor.setSpeed(rightSpeedSet);
        rightSpeed = rightSpeedSet;
        totalDistance += distanceTemp + distY; // summing up 
        distanceTemp = 0;
        LeftEncoderCount = 0;
        RightEncoderCount = 0;
      }
    
      refspeedAdjInterval = millis() - refspeedAdjCount;
      if (refspeedAdjInterval > 50)
      {
        int Q = refIR();
        int cond = Q - prevIR;
        prevIR = Q;
        refspeedAdjCount = millis();
        refspeedAdjInterval = 0;
        if (cond > 7)
        {
          cumTrans = 0;
          transItera = 0;
          avgTrans = 0;
          vStop = 0;
          refirFlag = 0;
          //turnRight(33);
          break;
        }
      }
  }
  totalDistance += distance; // summing up the last distance traveled
  return totalDistance;
  //vStop = 0;
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  wait(delayTime);
} // end driveStraight


void backUp(long b)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  left_motor.run(BACKWARD);
  right_motor.run(BACKWARD);
  left_motor.setSpeed(145);
  leftSpeed = 145;
  right_motor.setSpeed(155);
  rightSpeed = 155;
  prevIR = refIR();
  float backDistance;
  while (backDistance < b)
  {
     
      countDiff = LeftEncoderCount - RightEncoderCount;
      int input = (int) round (0.2 * countDiff);
      
      if (LeftEncoderCount > RightEncoderCount)
      {
        if (rightSpeed > 150)
        {
          leftSpeed = leftSpeed - 1;
        }
        else if (leftSpeed < 150)
        {
          rightSpeed = rightSpeed + 1;
        }
        else
        {
          leftSpeed = leftSpeed - 2;
          rightSpeed = rightSpeed + 2;
        }
        
      }
      if (LeftEncoderCount < RightEncoderCount)
      {
        if (leftSpeed > 150)
        {
          rightSpeed = rightSpeed - 1;
        }
        else if (rightSpeed < 150)
        {
          leftSpeed = leftSpeed + 1;
        }
        else
        {
          rightSpeed = rightSpeed - 2;
          leftSpeed = leftSpeed + 2;
        }
      }
      
        left_motor.setSpeed(leftSpeed);
        right_motor.setSpeed(rightSpeed);
      
      
      Serial.print("\nleft speed: ");
      Serial.print(leftSpeed);
      Serial.print("  right speed: ");
      Serial.print(rightSpeed);
       
      averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
      backDistance = (averagePulse / 40) * 3.14 * 6.5;
      Serial.print("\nLEncoder: ");
      Serial.print(LeftEncoderCount);
      Serial.print(", REncoder: ");
      Serial.println(RightEncoderCount);
      
      Serial.print(" diff: ");
      Serial.print(countDiff);
      Serial.print("\ndistance traveled back: ");
      Serial.print(backDistance);
      Serial.print("\n");
  }
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  wait(delayTime);
} // end backUp()


// TURNS

void turnRight(float turnR)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  left_motor.run(FORWARD);
  right_motor.run(BACKWARD);
  Serial.print("turn right");
  left_motor.setSpeed(150);
  right_motor.setSpeed(150);
  while (distance < turnR)
  {
    wait(20);
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;  
  }
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  Serial.print("\nEnd");
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  wait(delayTime); // wait for vehicle to become stable
} // end turnRight


void turnLeft(float turnL)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  left_motor.run(BACKWARD);
  right_motor.run(FORWARD);
  Serial.print("turn left");
  left_motor.setSpeed(150);
  right_motor.setSpeed(150);
  while (distance < turnL)
  {
    wait(20);
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
  }
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  Serial.print("\nEnd");
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  wait(delayTime); // wait for vehicle to become stable
} // end turnLeft



void startCheck()
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  transItera = 0;
  //wait(50);
  Serial.println("initialising sensors...");
  Serial.print("checking IR obstacle sensor...");
  while(1){if (getVoltage(A0) < 1 && getVoltage(A1) < 1)   break;} // IR obstacle check
  Serial.println("\nDone");
  wait(1000); // preperation before next sensor test
  Serial.print("checking ultrasonic sensor...");
  unsigned long ultraCT = millis();
  unsigned long ultraInterval = 0;
  while(1)
  {
    if (ultraInterval > 500)
    {
      if (ultraSonic() < 7)    break;
      ultraCT = millis();
      ultraInterval = 0;
    }
    ultraInterval = millis() - ultraCT;
    //wait(1000);
  } // ultrasonic check
  Serial.println("Done");
  Serial.print("checking IR reflective sensor...\n");
  unsigned long refIR = millis();
  unsigned long refInterval = 0;
  while(1)
  {
    //wait(1000);
    if (refInterval > 500)
    {
      int currRead = analogRead(A3);
      cumTrans += currRead;
      transItera ++;
      avgTrans = cumTrans / transItera;
      int cond = currRead - avgTrans;
      Serial.print(" TCRT5000L: ");
      Serial.println(currRead);
      if (cond > 10)
      {
        cumTrans = 0;
        transItera = 0;
        avgTrans = 0;
        //vStop = 0;
        break;
      }
      refInterval = 0;
      refIR = millis();
    }
    refInterval = millis() - refIR;
  }
  Serial.println("\nDone");
  Serial.print("checking left encoder...\n");
  unsigned long leftCT = millis();
  unsigned long leftInterval = 0;
  while(1)
  {
    if (leftInterval > 500)
    {
      Serial.println(LeftEncoderCount);
      if (LeftEncoderCount > 50)
      {
        LeftEncoderCount = 0;
        break;  
      }
      leftInterval = 0;
      leftCT = millis();
    }
    leftInterval = millis() - leftCT;
  }
  Serial.println("\nDone");
  Serial.print("checking right encoder...\n");
  //Serial.println(RightEncoderCount);
  RightEncoderCount = 0;
  unsigned long rightCT = millis();
  unsigned long rightInterval = 0;
  while(1)
  {
    //wait(1000);
    if (rightInterval > 500)
    {
      Serial.println(RightEncoderCount);
      if (RightEncoderCount > 50)
      {
        RightEncoderCount = 0;
        break;  
      }
      rightCT = millis();
      rightInterval = 0;
    }
    rightInterval = millis() - rightCT;
  }
  Serial.println(" \nDone\nsystemcheck completed");
  return;  
} // end startCheck



// SENSORS



int refIR()
{
  
  int refI = 0;
  int refSum = 0;
  int currRead1 = 0;
  int cumTrans1 = 0;
  int avgTrans1 = 0;
  for(refI = 0; refI < 3; refI ++)
  {
    currRead1 = analogRead(A3);
    cumTrans1 += currRead1;
  }
  avgTrans1 = cumTrans1 / 3;
      currRead1 = analogRead(A3);
      Serial.print("\nTCRT5000L: ");
      //Serial.print(currRead);
      //int cond = currRead - avgTrans;
      //Serial.print("  avg: ");
      Serial.println(avgTrans1);
      //Serial.print("  A3 diff: ");
      //Serial.print(cond)
      return avgTrans1;
} // end refIR

float ultraSonic()
{
  int ultraI = 0;
  float distSum = 0;
  // tests the ultrasonic sensor 5 times, and averages the result to avoid noise
  for(ultraI = 0; ultraI < 5; ultraI ++)
  {
    digitalWrite(A4, LOW);
    unsigned long startTime = micros();
    while (micros()-startTime < 10){
    }    digitalWrite(A4, HIGH);
    startTime = micros();
    while (micros()-startTime < 10){
    }
    digitalWrite(A4, LOW);
    unsigned long duration = pulseIn(A5, HIGH);
    float ultraDist1 = (duration * 0.034) / 2;
    distSum += ultraDist1;
  }
  float ultraDist = distSum / 5;
  Serial.print("\ndistance(cm): ");
  Serial.println(ultraDist);
  
  return ultraDist;
} // end ultraSonic

void countLEncoder(){ // interrupt function for left encoder
    LeftEncoderCount++;
} // end countLEncoder

void countREncoder(){ // interrupt function for right encoder
      RightEncoderCount++;
} //end countREncoder

float getVoltage(int pin) {
  float voltage = 5.0 * analogRead(pin) / 1024;
  return voltage;
} //end getVoltage

// a function to pause the program while still being non-blocking
void wait(float w){
  unsigned long prevTime = millis();
  while (millis()- prevTime < w){}
  return;
} //end wait




int reactObstacles(){
  int distX = 0;
  int distY = 0;
   
  if (getVoltage(A0) > 1){
    turnLeft(turnL);
    int angle = 90;
  }
  else{
    turnRight(turnR);
    int angle = 270;
  }
  
  do{

    // if facing Left, travel along the side of the obstacle
    if (angle == 90){
      distX += getWidth(0);
      //if possible, turn Right, if not, turn Left.
      if (ultraSonic() > 10){
        turnRight(turnR);
        angle -= 90;
      }
      else {
        turnLeft(turnL);
        angle += 90;
      }
    }
    else if (angle == 270){
      distX -= getWidth(distX);
      //if possible, turn Right, if not, turn Left.
      if (ultraSonic() > 10){
        turnLeft(turnL);
        angle += 90;
      }
      else {
        turnRight(turnR);
        angle -= 90;
      }
    }
    else if (angle == 180){
      distY -= getWidth(0);
      //if possible, turn Right, if not, turn Left.
      if (ultraSonic() > 10){
        turnRight(turnR);
        angle -= 90;
      }
      else {
        turnLeft(turnL);
        angle += 90;
      }
    }
    else{
      distY += getWidth(0);
      //if possible, turn Right, if not, turn Left.
      if (ultraSonic() > 10){
        turnRight(turnR);
        angle -= 90;
      }
      else {
        turnLeft(turnL);
        angle += 90;
      }
    }

    // keeps the angle between 0 and 360 degrees
    if (angle == 360) angle = 0;

    if (abs(distX) > maxWidth) maxWidth = distX;
        
  }while (distX != 0);  // continue loop until car has returned to original path

  // face car back to original direction
  if (angle == 90) turnRight(turnR);
  else if (angle == 180) turnLeft(turnA);
  else if (angle == 270) turnLeft(turnL);

  return distY;
}

int getWidth(int maxDist){
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  int width = 0;
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
  left_motor.setSpeed(leftSpeedSet);
  leftSpeed = leftSpeedSet;
  right_motor.setSpeed(rightSpeedSet);
  rightSpeed = rightSpeedSet;

  //car continues forward as long as there is still an object beside it and there's nothing in front of the object
  //car will also stop if the vehicle is returning to the straight path to the destination
  while(((getVoltage(A0) < 1)||(getVoltage(A1) < 1)) && (ultraSonic() > 10) && ((width < maxDist) && (maxDist != 0)))  {
    wait(50);
    
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    width = (averagePulse / 40) * 3.14 * 6.5;
  }

  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);

  //car continues to move forward to make sure its back is past the obstacle
  if (ultraSonic() > 10 || maxDist != 0){
    simpleStraight(25);
    width += 25;
  }

  return width;
} // end getWidth

//End of main code 
//Will now be adding Final Search and go Back code

//this is the main function that will call the Square function
//and increase the square size each time it is not successful.
void finalSearch(){

  //turn vehicle to the left to position for square searching
  //Left is vehicle orientation of angle 0.
  turnLeft(turnL);
  delay(500);
  check();
  delay(500);

  for(sideLength = 10 ; sideLength < 31 ; sideLength += 10){
    //first check for the paper to be sure
    check();
    //then call on the Square function for a length of 10
    Square(sideLength);
    //check again for efficiency
    check();
    //increase sideLength variable and loop again 
  }
} // end final search function.

//function does a square based on the sideLength variable given
//to the function and checks for the paper at all times.
void Square(int sideLength){

  halfLength = sideLength/2; //this is needed for the half square length 
                             //to start and finish the square.

  Serial.print("Angle: ");
  Serial.println(angle);
  simpleStraight(halfLength); //check bottom IR while going straight in this function
  delay(500);
  
  turnRight(turnR);
  delay(500);
  angle += right;
  Serial.print("Angle: ");
  Serial.println(angle);
  check();
  delay(500);
  simpleStraight(sideLength);
  delay(500);
  
  turnRight(turnR);
  delay(500);
  angle += right;
  Serial.print("Angle: ");
  Serial.println(angle);
  check();
  delay(500);
  simpleStraight(sideLength);
  delay(500);

  turnRight(turnR);
  delay(500);
  angle += right;
  Serial.print("Angle: ");
  Serial.println(angle);
  check();
  delay(500);
  simpleStraight(sideLength);
  delay(500);

  turnRight(turnR);
  delay(500);
  angle+= right;
  Serial.print("Angle: ");
  Serial.println(angle);
  check();
  delay(500);
  simpleStraight(halfLength);
  delay(500);

  //RESET THE ANGLE FOR NEXT RUN THROUGH
  angle = 0;
  Serial.print("Angle: ");
  Serial.println(angle);
} // end Square function

//drives straight given distance and checks for IR
void simpleStraight(long sS)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
  left_motor.setSpeed(139);
  leftSpeed = 139;
  right_motor.setSpeed(157);
  rightSpeed = 157;
  while(distance < sS)
  {
    delay(70);
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
    Serial.print("\nsimple forward: ");
    Serial.print(distance);
    check(); //bottom IR check
  }
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
} // end simpleStraight

//Checks for goal location
void check(){
      int readrir = analogRead(A3);
      Serial.print(" readRIR: ");
      Serial.println(readrir);
      if (readrir > 20){
       left_motor.setSpeed(0);
       right_motor.setSpeed(0);
       delay(500);
       reposition();
     }
} // end check

//Simple turn right function that does a 180.
void turnAround(){
  Serial.print("Turn Around");
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  left_motor.run(FORWARD);
  right_motor.run(BACKWARD);
  left_motor.setSpeed(155);
  right_motor.setSpeed(145);
  while (1)
  {
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
    if (distance > turnA)    break;
    delay(10);
  }
  Serial.print("\n TURNED AROUND");
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  delay(200);
}


void reposition(){
  Serial.println("REPOSITIONING");
  
  //reposition based on the angle and then call
  //the goBack() function.
  if(angle == 0 || angle == 360){
    //need to turn 90 degrees left to point at starting point
    turnLeft(turnL);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    Serial.println("POINTING TOWARDS STARTING POINT CALL RETURN FUNCTION");
    while(1){
     
    }
    goBack(destDis);
  }

    if(angle == 90){
  //looking opposite direction of starting point must turn around
    turnAround();
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    Serial.println("POINTING TOWARDS STARTING POINT CALL RETURN FUNCTION");
    while(1){
      
    }
    goBack(destDis);
  }

  if(angle == 180){
    //need to turn 90 degrees right to point at starting point
    turnRight(turnR);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    Serial.println("POINTING TOWARDS STARTING POINT CALL RETURN FUNCTION");
  }
    while(1){
      
    }
    goBack(destDis);
}  //end of reposition function

//function to return to starting location
void goBack(int destDist){
  turnRight(turnA);
  turnLeft(turnL);
  if (width1 > width2) simpleStraight(width1);
  else simpleStraight(width2);
  turnRight(turnR);
  driveStraight(destDis);
  turnRight(turnR);
  if (width1 > width2) simpleStraight(width1);
  else simpleStraight(width2);
} // end of goBack() function 
