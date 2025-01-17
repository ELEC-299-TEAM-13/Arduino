#define LEncoderPin 3
#define REncoderPin 2
#include <AFMotor.h>

AF_DCMotor left_motor(1, MOTOR12_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(3, MOTOR34_1KHZ); // right motor to M3 on motor control board

volatile int LeftEncoderCount = 0;
volatile int RightEncoderCount = 0;

int initialleftSpeed = 133; //unchanging PWM
int initialrightSpeed = 152; //unadjustable PWM
int leftSpeed = 133; //adjustable PWM
int rightSpeed = 152; //adjustable PWM
float averagePulse;
float destDis;
float distance;
int prevIR;
int vStop = 1;

int turnL = 10; //encoder count for 90 degree turn to the left
int turnR = 11; //encoder count for 90 degree turn to the right

int avgTrans;
int transItera;
int cumTrans;
int refirFlag;

float width1; // x
float depth1; // y
float width2;
float depth2;
float obst1;    //distance from start to obatacle 1
float obst2;    //distance between obstacle 1/2
float destination;  //distance from obstacle 1/2 to destination
int obst1Flag;    //1 for engaged obst 1
int obst2Flag;    //1 for engaged obst 2
int rightFlag;    //1 for special right turn
int sideLength;
int halfLength;
int angle;
int right = 90;
int left = -90;
float stepAside;  // the distance vehicle step aside before straveling back
float backward;   // distance need to travel back

int TicksPerRot = 20;


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

  startCheck();

  wait(3000); // wait for preperation after setup check
}

void loop() {
  destDis = 300;
  while (vStop == 1)
  {
    cumTrans = 0;
    avgTrans = 0;
    transItera = 0;
    destination = 0;
    destination = driveStraight(destDis); // forward to marker

    if (obst2Flag = 1) // obst 2 engaged
    {
      destination = (destination - obst1 - obst2);
    }
    else // only obst 1 engaged
    {
      destination = destination - obst1;
    }

    backUp(3);
    wait(300);

    //while(1){};
    float stepAside;  // the distance vehicle step aside before straveling back
    float backward;   // distance need to travel back

    if (obst1Flag != 0) // obs1 presented
    {
      if (obst2Flag != 0) // obst2 presented
      {
        if (width1 > width2)  // obst1 wider
        {
          stepAside = width1;
          backward = destination + obst2 + obst1 + depth1 + depth2;
        }
        else          // obst2 wider
        {
          stepAside = width2;
          backward = destination + obst2 + obst1 + depth1 + depth2;
        }
      }
      else  // only obst1 presented
      {
        stepAside = width1;
        backward = destination + obst1 + depth1;
      }
    }
    else  // not obstacle presented
    {
      stepAside = 0;
      backward = destination;
    }

    if (vStop == 1) // search for the marker if not
      // detected with in the indicated range
    {
      finalSearch();
    }
    else
    {
      turnRight(turnR);
      wait(200);
      turnRight(turnR);
      wait(300);
      goBack();
    }
  }
}

void goBack() {

  wait(500);
  if (obst1Flag == 0) // did not encountered any obstacles
  {
    simpleStraight(backward);
    vStop = 0;
    return;
  }

  if (rightFlag == 1) // special right case
  {
    simpleStraight2(3);
    wait(300);
    turnLeft(turnL);
    wait(300);
    simpleStraight2(stepAside);
    wait(300);
    turnRight(turnR);
    wait(300);
    simpleStraight2(backward);
    wait(300);
    turnRight(turnR);
    wait(300);
    simpleStraight2(stepAside);
    wait(300);
    turnLeft(turnL);
    wait(300);
    simpleStraight2(3);
  }
  else  // normal case
  {
    simpleStraight2(3);
    wait(300);
    turnRight(turnR);
    wait(300);
    simpleStraight2(stepAside);
    wait(300);
    turnLeft(turnL);
    wait(300);
    simpleStraight(backward);
    wait(300);
    turnLeft(turnL);
    wait(300);
    simpleStraight2(stepAside);
    wait(300);
    turnRight(turnR);
    wait(300);
    simpleStraight2(3);
  }
  vStop = 0;

  while (1) {};
}

void avoidObstacles()
{
  wait(200);
  backUp(3);
  wait(500);
  turnLeft(turnL); // facing left
  wait(500);
  if (ultraSonic() < 20)// obstacle on the left side blocking the left turn
  {
    turnRight(turnR); // facing to right
    wait(200);
    turnRight(turnR);
    if (obst2Flag == 0) // +x travel
    {
      width1 = overcomeCheck() + 25;  // forward 25 for fully passing (vehicle body length)
    }
    else // now on obst 2
    {
      width2 = overcomeCheck() + 25;
    }
    simpleStraight(25);

    rightFlag = 1; // marking a right turn

    wait(500);
    turnLeft(turnL); // facing forward
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
    turnLeft(turnL); // +x travel
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
    turnRight(turnR); // back to m line
    wait(700);
  }
  else // default left turn
  {
    //
    if (obst2Flag == 0) // -x travel
    {
      width1 = overcomeCheck() + 25;  // forward 25 for fully passing (vehicle body length)
    }
    else // now on obst 2
    {
      width2 = overcomeCheck() + 25;
    }
    simpleStraight(25);

    wait(500);
    turnRight(turnR); // facing forward
    wait(500);

    if (obst2Flag == 0) // +y travel
    {
      simpleStraight(15); // forward 15 for fully engaging
      depth1 += overcomeCheck() + 40; // 15 + 25 = 40
    }
    else // obst 2
    {
      simpleStraight(15);
      depth2 += overcomeCheck() + 40;
    }
    simpleStraight(25);

    wait(500);
    // overcome the obstacle
    turnRight(turnR); // facing right +x travel

    while (1) // front left back right
    {
      wait(300);
      if (ultraSonic() < 20)
      {
        turnLeft(turnL); // facing forward
        wait(300);
        simpleStraight(15);
        if (obst2Flag == 0) // +y travel
        {
          depth1 += 15;
        }
        else // obst 2
        {
          depth2 += 15;
        }
        wait(300);
        turnRight(turnR);  // facing right
        continue; //  check again
      }
      else
      {
        break;  // nothing blocking
      }
    }

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
    turnLeft(turnL); // back to m line
    wait(700);
  }
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
  return OCC; // return overall distance between a turn and overcome
}

float simpleStraight(long sS)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
  left_motor.setSpeed(initialleftSpeed);
  right_motor.setSpeed(initialrightSpeed);
  while (distance < sS)
  {
    wait(70);
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
    Serial.print("\nsimple forward: ");
    Serial.print(distance);
    check();
  }

  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  return distance;
} // end simpleStraight

//simpleStraight2 is only called when the goal location is already found.
//Do not need to keep checking for it anymore.
float simpleStraight2(long sS)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
  left_motor.setSpeed(initialleftSpeed);
  right_motor.setSpeed(initialrightSpeed);
  while (distance < sS)
  {
    wait(70);
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
    Serial.print("\nsimple forward: ");
    Serial.print(distance);
  }

  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  return distance;
} // end simpleStraight2


float driveStraight(long i)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
  left_motor.setSpeed(initialleftSpeed);
  right_motor.setSpeed(initialrightSpeed);
  prevIR = refIR();
  refirFlag = 0;

  int totalDistance = 0;

  float dist;
  unsigned long speedAdjCount = millis();
  unsigned long speedAdjInterval = 0;
  unsigned long refspeedAdjCount = millis();
  unsigned long refspeedAdjInterval = 0;

  while ((distance += totalDistance) < i)
  {

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

      left_motor.setSpeed(leftSpeed);
      right_motor.setSpeed(rightSpeed);

      averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
      distance = (averagePulse / 40) * 3.14 * 6.5;

      refirFlag = 1;
      speedAdjCount = millis();
      speedAdjInterval = 0;
    }

    dist = ultraSonic();
    if (dist < 10)   // if an obstacle within 10cm
    {
      LeftEncoderCount = 0;
      RightEncoderCount = 0;
      left_motor.run(RELEASE);
      right_motor.run(RELEASE);
      float distanceTemp = distance;

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

      avoidObstacles();
      left_motor.run(RELEASE);
      right_motor.run(RELEASE);
      //vStop = 0;
      //break;
      left_motor.run(FORWARD);
      right_motor.run(FORWARD);
      left_motor.setSpeed(initialleftSpeed);
      right_motor.setSpeed(initialrightSpeed);
      totalDistance += distanceTemp;
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
        break;
      }
    }
  }
  totalDistance += distance;
  return totalDistance;
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
} // end driveStraight


void backUp(long b)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  left_motor.run(BACKWARD);
  right_motor.run(BACKWARD);
  left_motor.setSpeed(initialleftSpeed);
  right_motor.setSpeed(initialrightSpeed);
  prevIR = refIR();
  float backDistance;
  while (backDistance < b)
  {

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

    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    backDistance = (averagePulse / 40) * 3.14 * 6.5;

  }
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
} // end backUp()

// TURNS

void turnRight(float turnR)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  left_motor.run(FORWARD);
  right_motor.run(BACKWARD);
  left_motor.setSpeed(initialleftSpeed);
  right_motor.setSpeed(initialrightSpeed);
  while (distance < turnR)
  {
    wait(20);
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
  }
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  wait(200); // wait for vehicle to become stable
} // end turnRight


void turnLeft(float turnL)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  left_motor.run(BACKWARD);
  right_motor.run(FORWARD);
  left_motor.setSpeed(initialleftSpeed);
  right_motor.setSpeed(initialrightSpeed);
  while (distance < turnL)
  {
    wait(20);
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
  }
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  wait(200); // wait for vehicle to become stable
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
  while (1) {
    if (getVoltage(A0) < 1 && getVoltage(A1) < 1)   break; // IR obstacle check
  }
  Serial.println("\nDone");
  wait(1000); // preperation before next sensor test
  Serial.print("checking ultrasonic sensor...");
  unsigned long ultraCT = millis();
  unsigned long ultraInterval = 0;
  while (1)
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
  while (1)
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
  while (1)
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
  while (1)
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

//this is the main function that will call the Square function
//and increase the square size each time it is not successful.
void finalSearch() {

  //turn vehicle to the left to position for square searching
  //Left is vehicle orientation of angle 0.
  turnLeft(turnL);
  wait(500);
  check();
  wait(500);

  for (sideLength = 10 ; sideLength < 51 ; sideLength += 10) {
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
void Square(int sideLength) {

  halfLength = sideLength / 2; //this is needed for the half square length
  //to start and finish the square.

  simpleStraight(halfLength); //check bottom IR while going straight in this function
  wait(500);

  turnRight(turnR);
  wait(500);
  angle += right;
  check();
  wait(500);
  simpleStraight(sideLength);
  wait(500);

  turnRight(turnR);
  wait(500);
  angle += right;
  check();
  wait(500);
  simpleStraight(sideLength);
  wait(500);

  turnRight(turnR);
  wait(500);
  angle += right;
  check();
  wait(500);
  simpleStraight(sideLength);
  wait(500);

  turnRight(turnR);
  wait(500);
  angle += right;
  check();
  wait(500);
  simpleStraight(halfLength);
  wait(500);

  //RESET THE ANGLE FOR NEXT RUN THROUGH
  angle = 0;
} // end Square function

void check() {
  int readrir = analogRead(A3);
  if (readrir > 25) {
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    wait(500);
    reposition();
  }
} // end check

void reposition() {
  //reposition based on the angle and then call
  //the goBack() function.
  if (angle == 0 || angle == 360) {
    //need to turn 90 degrees left to point at starting point
    turnLeft(turnL);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    goBack();
  }

  if (angle == 90) {
    //looking opposite direction of starting point must turn around
    turnRight(turnR);
    wait(500);
    turnRight(turnR);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    goBack();
  }

  if (angle == 180) {
    //need to turn 90 degrees right to point at starting point
    turnRight(turnR);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    goBack();
  }

  //already pointed towards starting point
  if (angle == 270) {
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    goBack();
  }

} //end reposition

// SENSORS

int refIR()
{

  int refI = 0;
  int refSum = 0;
  int currRead1 = 0;
  int cumTrans1 = 0;
  int avgTrans1 = 0;
  for (refI = 0; refI < 3; refI ++)
  {
    currRead1 = analogRead(A3);
    cumTrans1 += currRead1;
  }
  avgTrans1 = cumTrans1 / 3;
  currRead1 = analogRead(A3);
  Serial.print("\nTCRT5000L: ");
  Serial.println(avgTrans1);
  return avgTrans1;
}

float ultraSonic()
{
  int ultraI = 0;
  float distSum = 0;
  for (ultraI = 0; ultraI < 5; ultraI ++)
  {
    digitalWrite(A4, LOW);
    delayMicroseconds(10);
    digitalWrite(A4, HIGH);
    delayMicroseconds(10);
    digitalWrite(A4, LOW);
    unsigned long duration = pulseIn(A5, HIGH);
    float ultraDist1 = (duration * 0.034) / 2;
    distSum += ultraDist1;
  }
  float ultraDist = distSum / 5;
  Serial.print("\ndistance(cm): ");
  Serial.println(ultraDist);

  return ultraDist;
}

void countLEncoder() { // interrupt function for left encoder
  LeftEncoderCount++;
}

void countREncoder() { // interrupt function for right encoder
  RightEncoderCount++;
}
float getVoltage(int pin) {
  float voltage = 5.0 * analogRead(pin) / 1024;
  return voltage;
}

void wait(float w) {
  unsigned long prevTime = millis();
  while (millis() - prevTime < w) {}
  return;
}
