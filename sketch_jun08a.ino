#define LEncoderPin 3     
#define REncoderPin 2

#include <AFMotor.h>
#include <PID_v2.h> 

AF_DCMotor left_motor(1, MOTOR12_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(3, MOTOR34_1KHZ); // right motor to M3 on motor control board

double Kp = 0.5, Ki = 0.5, Kd = 0.1; // starting PID controller settings
// double Kp = 1, Ki = 0, Kd = 0; // PID controller settings using only immediate difference, similar to simple closed loop control


double EncDelta=0;
double OutDelta=-20.0;
double EncDeltaSet=0;

int LeftEncoderCount = 0;
int RightEncoderCount = 0;
int prevLeftEncoderCount = 0;
int prevRightEncoderCount = 0;
int leftInter;
int rightInter;
int countDiff = 0;
int DelayTime = 200;
int LPWM=255;
int RPWM=255;

int leftSpeed;
int rightSpeed;
float averagePulse;
float destDis;
float distance;
int prevIR;
int vStop = 1;

int avgTrans;
int transItera;
int cumTrans;
int refirFlag;


int TicksPerRot = 20;



void setup() {
  Serial.begin (115200);
  pinMode(A0, INPUT); // left IR
  pinMode(A1, INPUT); // right IR
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, OUTPUT); // triger
  pinMode(A5, INPUT); // echo
  DiffSpeed.SetMode(AUTOMATIC);
  DiffSpeed.SetOutputLimits(-150,150);
 
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
  //Serial.println("test1");
  
  wait(3000); // delay for preperation after setup check
}

void loop() {
  //Serial.println("test2");
  //destDis = 10 * 6.5 * 3.14; //cm
  destDis = 9999;
  while(vStop == 1)
  {
    //Serial.println("test3");
    cumTrans = 0;
    avgTrans = 0;
    transItera = 0;
    driveStraight(destDis); // out of start marker area
    if (vStop == 0)   break;
    backUp(3);
    turnRight(33);
    wait(300);
    driveStraight(destDis); // out of end marker area
    vStop = 0;
  }
  while (1){};
}

void ReactObstacles()
{
  //wait(100);
  if (getVoltage(A0) < 1 && getVoltage(A1) < 1)
  {
    backUp(3);
    turnRight(30);
    return;
  }
  else if (getVoltage(A0) < 1) // left sensor
  {
    backUp(3);
    turnRight(7);
  }
  else if (getVoltage(A1) < 1) // right sensor
  {
    backUp(3);
    turnLeft(7);
  }
  return;
}

void avoidObstacles()
{
  wait(200);
  backUp(3);
  wait(500);
  turnLeft(12);
  wait(500);
  simpleStraight(45);
  wait(500);
  turnRight(13);
  wait(500);
  simpleStraight(94);
  wait(500);
  turnRight(13);
  wait(500);
  simpleStraight(45);
  wait(500);
  turnLeft(13);
  wait(700);
  //  while (1){};
}


void simpleStraight(long sS)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  prevLeftEncoderCount = 0;
  prevRightEncoderCount = 0;
  distance = 0;
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
  left_motor.setSpeed(139);
  leftSpeed = 139;
  right_motor.setSpeed(157);
  rightSpeed = 157;
  while(distance < sS)
  {
    wait(70);
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
    Serial.print("\nsimple forward: ");
    Serial.print(distance);
  }
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
} // end simpleStraight


void driveStraight(long i) 
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  prevLeftEncoderCount = 0;
  prevRightEncoderCount = 0;
  distance = 0;
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
  left_motor.setSpeed(145);
  leftSpeed = 145;
  right_motor.setSpeed(155);
  rightSpeed = 155;
  prevIR = refIR();
  refirFlag = 0;

  unsigned long speedAdjCount = millis();
  unsigned long speedAdjInterval = 0;
  unsigned long refspeedAdjCount = millis();
  unsigned long refspeedAdjInterval = 0;
  
  while (distance < i)
  {
      leftInter = prevLeftEncoderCount - LeftEncoderCount;
      rightInter = prevRightEncoderCount - RightEncoderCount;
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
      float dist = ultraSonic();
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
      
      prevLeftEncoderCount = LeftEncoderCount;
      prevRightEncoderCount = RightEncoderCount;

      if (getVoltage(A0) < 1 || getVoltage(A1) < 1)   
      {
        //ReactObstacles();
        left_motor.run(RELEASE);
        right_motor.run(RELEASE);
        avoidObstacles();
        left_motor.run(RELEASE);
        right_motor.run(RELEASE);
        //vStop = 0;
        //break;
        left_motor.run(FORWARD);
        right_motor.run(FORWARD);
        left_motor.setSpeed(141);
        leftSpeed = 141;
        right_motor.setSpeed(157);
        rightSpeed = 157;
        distance = 0;
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
  //vStop = 0;
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  //wait(200);
} // end driveStraight


void backUp(long b)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  prevLeftEncoderCount = 0;
  prevRightEncoderCount = 0;
  left_motor.run(BACKWARD);
  right_motor.run(BACKWARD);
  left_motor.setSpeed(145);
  leftSpeed = 145;
  right_motor.setSpeed(155);
  rightSpeed = 155;
  prevIR = refIR();
  while (backDistance < b)
  {
     
      leftInter = prevLeftEncoderCount - LeftEncoderCount;
      rightInter = prevRightEncoderCount - RightEncoderCount;
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
      float backDistance = (averagePulse / 40) * 3.14 * 6.5;
      Serial.print("\nLEncoder: ");
      Serial.print(LeftEncoderCount);
      Serial.print(", REncoder: ");
      Serial.println(RightEncoderCount);
      
      Serial.print(" diff: ");
      Serial.print(countDiff);
      Serial.print("\ndistance traveled back: ");
      Serial.print(backDistance);
      Serial.print("\n");
      prevLeftEncoderCount = LeftEncoderCount;
      prevRightEncoderCount = RightEncoderCount;
  }
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  //wait(200);
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
  left_motor.setSpeed(145);
  right_motor.setSpeed(155);
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
  wait(200); // wait for vehicle to become stable
} // end turnRight


void turnLeft(float turnL)
{
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  distance = 0;
  left_motor.run(BACKWARD);
  right_motor.run(FORWARD);
  Serial.print("turn left");
  left_motor.setSpeed(145);
  right_motor.setSpeed(155);
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
}

float ultraSonic()
{
  int ultraI = 0;
  float distSum = 0;
  for(ultraI = 0; ultraI < 5; ultraI ++)
  {
    digitalWrite(A4, LOW);
    unsigned long startTime = micros();
    while (micros()-startTime < 10){
    }
    digitalWrite(A4, HIGH);
    startTime = micros();
    while (micros()-startTime < 10){
    }    digitalWrite(A4, LOW);
    unsigned long duration = pulseIn(A5, HIGH, 3000);
    float ultraDist1 = (duration * 0.034) / 2;
    if (duration == 0) ultraDist = 0.51;
    distSum += ultraDist1;
  }
  float ultraDist = distSum / 5;
  Serial.print("\ndistance(cm): ");
  Serial.println(ultraDist);
  
  //wait(20);
  return ultraDist;
}

void countLEncoder(){ // interrupt function for left encoder
    LeftEncoderCount++;
}

void countREncoder(){ // interrupt function for right encoder
      RightEncoderCount++;
}
float getVoltage(int pin) {
  float voltage = 5.0 * analogRead(pin) / 1024;
  return voltage;
}

void wait(float w){
  unsigned long prevTime = millis();
  while (millis()- prevTime < w){}
  return;
}
