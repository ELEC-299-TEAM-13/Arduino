float width1; // x
float depth1; // y
float width2;
float depth2;
float obst1;		//distance from start to obatacle 1
float obst2;		//distance between obstacle 1/2
float destination;	//distance from obstacle 1/2 to destination
int obst1Flag; 		//1 for engaged obst 1
int obst2Flag; 		//1 for engaged obst 2


void avoidObstacles()
{
  delay(200);
  backUp(3);
  delay(500);
  turnLeft(12); // facing left
  delay(500);
  if (obst2Flag == 0) // -x travel
  {
    width1 = overcomeCheck() + 25;
  }
  else // now on obst 2
  {
    width2 = overcomeCheck() + 25;
  }
  simpleStraight(25);
  
  delay(500);
  turnRight(13); // facing forward
  delay(500);
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
  
  delay(500);
  // overcome the obstacle
  turnRight(13); // +x travel
  delay(500);
  if (obst2Flag == 0)
  {
    simpleStraight(width1);
  }
  else // obst 2
  {
    simpleStraight(width2);
  }
  delay(500);
  turnLeft(13); // back to m line
  delay(700);
  //  while (1){};
}

// driveStraight & simpleStraight need return distance

float overcomeCheck ()
{
  int OCC = 10;
  delay(500);
  simpleStraight(10);
  if (getVoltage(A0) < 1 || getVoltage(A1) < 1) // obstacle on the side
    {
      OCC += overcomeCheck();
    }
  //simpleStraight(10);
  //OCC += 10;
  return OCC; // return overall distance between a turn and overcome
}
