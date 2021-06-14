float width1; // x
float depth1; // y
float width2;
float depth2;
float obst1;		//distance from start to obatacle 1
float obst2;		//distance between obstacle 1/2
float destination;	//distance from obstacle 1/2 to destination or full distance
int obst1Flag; 		//1 for engaged obst 1
int obst2Flag; 		//1 for engaged obst 2


	float stepAside;	// the distance vehicle step aside before straveling back
	float backward;		// distance need to travel back
	
	if (obst1Flag != 0)	// obs1 presented
	{
		if (obst2Flag != 0) // obst2 presented
		{
			if (width1 > width2)	// obst1 wilder
			{
				stepAside = width1;
				backward = destination + obst2 + obst1 + depth1 + depth2;
			}
			else					// obst2 wilder
			{
				stepAside = width2;
				backward = destination + obst2 + obst1 + depth1 + depth2;
			}
		}
		else	// only obst1 presented
		{
			stepAside = width1;
			backward = destination + obst1 + depth1;
		}
	}
	else	// not obstacle presented
	{
		stepAside = 0;
		backward = destination;
	}
	