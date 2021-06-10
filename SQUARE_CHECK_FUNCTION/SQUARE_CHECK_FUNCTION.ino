
int angle;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  if (bottomIR <25){
    
  }
}

void goalSearch(){
  
  int angle = 0;
  int right = 90;
  int left = -90;
  

  turnRight();
  angle += right;
  check();
  turnAround();
  angle -= right;
  angle -= right;
  check();
  goStraight(0.5); //check bottom IR while going straight in this function
  
  turnRight();
  angle += right;
  check();
  goStraight(1);
  
  turnRight();
  angle += right;
  check();
  goStraight(1);

  turnRight();
  angle += right;
  check();
  goStraight(1);

  turnRight();
  angle+= right;
  check();
  goStraight(0.5);
  
  
  
}

void check(){
  if (bottomIR == mark){
    reposition();
  }
}

void reposition(){

  if(angle == 270){
    turnLeft();
    goBack();
  }

  if(angle == 90){
    turnRight();
    goBack();
  }

  if(angle == 0){
    turnAround();
    goBack();
  }

  if(angle == 180){
    goBack();
  }
}
