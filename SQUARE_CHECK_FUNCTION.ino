//Variable in all code
#define LEncoderPin 3     
#define REncoderPin 2

#include <AFMotor.h>
#include <PID_v2.h> 

AF_DCMotor left_motor(1, MOTOR12_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(3, MOTOR34_1KHZ); // right motor to M3 on motor control board

int LeftEncoderCount = 0;
int RightEncoderCount = 0;
int prevLeftEncoderCount = 0;
int prevRightEncoderCount = 0;
int countDiff = 0;
int DelayTime = 200;
int LPWM=255;
int RPWM=255;

int leftSpeed;
int rightSpeed;
float averagePulse;
float destDis;
float distance;

//Square_Check_Function specific variables
int turnL = 13; //encoder count for left 90 degrees
int turnR = 12; //encoder count for right 90 degrees
int turnA = 25; //encoder count to turn right 180 degrees
int angle = 0;
int right = 90;
int left = -90;
int halfLength;
int sideLength;

void setup() {
  // put your setup code here, to run once:
   Serial.begin (115200);
  pinMode(A0, INPUT); // left IR
  pinMode(A1, INPUT); // right IR
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, OUTPUT); // triger
  pinMode(A5, INPUT); // echo
  //DiffSpeed.SetMode(AUTOMATIC);
  //DiffSpeed.SetOutputLimits(-150,150);
 
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);

  attachInterrupt(1, countLEncoder, RISING); //calls on any CHANGE on pin 3 (interrupt #1, soldered to Pin3) 
  attachInterrupt(0, countREncoder, RISING); //calls on any CHANGE on pin 2 (interrupt #0, connected to header on Pin2) 
  interrupts();

   delay(20000);
} //end of setup

void loop() {
  // put your main code here, to run repeatedly:
  finalSearch();
  while(1){
    
  }
}  //end of loop()

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
    delay(70);
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
    Serial.print("\nsimple forward: ");
    Serial.print(distance);
    check();
  }
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
} // end simpleStraight

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
    //goBack();
  }

    if(angle == 90){
  //looking opposite direction of starting point must turn around
    turnAround();
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    Serial.println("POINTING TOWARDS STARTING POINT CALL RETURN FUNCTION");
    while(1){
      
    }
    //goBack();
  }

  if(angle == 180){
    //need to turn 90 degrees right to point at starting point
    turnRight(turnR);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    Serial.println("POINTING TOWARDS STARTING POINT CALL RETURN FUNCTION");
    while(1){
      
    }
    //goBack();
  }

  //already pointed towards starting point
  if(angle == 270){
    //goBack();
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    Serial.println("POINTING TOWARDS STARTING POINT CALL RETURN FUNCTION");
    while(1){
  
    }
  }

} //end reposition

void turnRight(float turnR){
Serial.println("TURN RIGHT");

  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  left_motor.run(FORWARD);
  right_motor.run(BACKWARD);
  Serial.print("turn right");
  left_motor.setSpeed(145);
  right_motor.setSpeed(155);
  while (1)
  {
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
    if (distance > turnR)
    {
      left_motor.run(RELEASE);
      right_motor.run(RELEASE);
      break;
    }
    delay(20);
  }
  Serial.println("turned Right");
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  delay(200); // wait for vehicle to become stable
} // end turn right function

void turnLeft(float turnL){
Serial.println("Turn Left");

  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  left_motor.run(BACKWARD);
  right_motor.run(FORWARD);
  Serial.print("turn left");
  left_motor.setSpeed(145);
  right_motor.setSpeed(155);
  while (1)
  {
    averagePulse = (LeftEncoderCount + RightEncoderCount) / 2;
    distance = (averagePulse / 40) * 3.14 * 6.5;
    Serial.print("distance ");
    Serial.println(distance);
    if (distance > turnL)
    {
       left_motor.run(RELEASE);
       right_motor.run(RELEASE);
      break;  
    }
    delay(20);
  }
  Serial.println("Turned Left");
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  delay(200); // wait for vehicle to become stable
  
} //end turn left function




void countLEncoder(){ // interrupt function for left encoder
    LeftEncoderCount++;
}

void countREncoder(){ // interrupt function for right encoder
      RightEncoderCount++;
}
