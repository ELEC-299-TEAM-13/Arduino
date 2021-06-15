float width1; // x
float depth1; // y
float width2;
float depth2;
float obst1;		//distance from start to obatacle 1
float obst2;		//distance between obstacle 1/2
float destination;	//distance from obstacle 1/2 to destination or full distance
int obst1Flag; 		//1 for engaged obst 1
int obst2Flag; 		//1 for engaged obst 2
int rightFlag;    //1 for special right turn

/*
void avoidObstacles()
{
  delay(200);
  backUp(3);
  delay(500);
  turnLeft(12); // facing left
  delay(500);
  if (obst2Flag == 0) // -x travel
  {
    width1 = overcomeCheck() + 25; // forward 25 for fully passing (vehicle body length)
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
    simpleStraight(15); 			// forward 15 for fully engaging
    depth1 = overcomeCheck() + 40; 	// 15 + 25 = 40
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
*/
// driveStraight & simpleStraight need return distance

void avoidObstacles()
{
  wait(200);
  backUp(3);
  wait(500);
  turnLeft(13); // facing left
  wait(500);
  if (ultraSonic() < 20)// obstacle on the left side blocking the left turn
  {
    turnRight(33);  // facing to right
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
      turnLeft(13); // facing forward
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
      turnLeft(13); // +x travel
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
      turnRight(13); // back to m line
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
      turnRight(13); // facing forward
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
      turnRight(13); // facing right +x travel

      while (1) // front left back right
      {
        wait(300);
        if (ultraSonic() < 20)
        {
          turnLeft(13); // facing forward
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
          turnRight(13);  // facing right
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
      turnLeft(13); // back to m line
      wait(700);
      //  while (1){};
  }
}



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
