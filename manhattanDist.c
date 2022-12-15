
#include <ev3.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stdlib.h>
#define PI 3.1415926
#define wheel_radius 0.0275 //in meters
#define MAX_OBSTACLES 25 /* maximum number of obstacles */
int num_obstacles = 18; /* number of obstacles */

double obstacle[MAX_OBSTACLES][2] = /* obstacle locations */
{{0.915, 0.305},{0.915, 0.61},{0.915, 0.915},{0.915, 1.22},
{1.829, 0.915},{ 1.829, 1.22}, {1.829, 1.525},{1.829, 1.829},
{1.829,2.134},{1.829,2.439},{1.829, 2.743},{3.048, 1.22},{3.048, 1.525},{3.048, 1.829},
{3.353, 0.915},
{3.353, 1.22},{3.658, 0.915},{3.658, 1.22},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
{-1,-1}};

double start[2] = {0.305,0.61}; /* start location */
double goal[2] = {3.658, 1.829};
void turnLeft()
{

	Wait(MS_800);

   int start_angle = readSensor(IN_1);
    OnRevSync(OUT_AB, 8);
    Wait(MS_900);
    while(true)
    {
        OnRevReg(OUT_B, 5);
        OnFwdReg(OUT_A, 5);
        Wait(MS_1);
        int current_angle = readSensor(IN_1);
        if(start_angle - current_angle >= 86)
        {
            Off(OUT_B);
            Off(OUT_A);
            break;
        }
   }
    OnFwdSync(OUT_AB, 8);
            Wait(MS_900);
            Off(OUT_B);
            Off(OUT_A);
}
void turnRight()
{
   int start_angle = readSensor(IN_1);
    OnRevSync(OUT_AB, 8);
    Wait(MS_900);
    Off(OUT_B);
    Off(OUT_A);
    while(true)
    {
        OnRevReg(OUT_A, 5);
        OnFwdReg(OUT_B, 5);
        Wait(MS_1);
       int current_angle = readSensor(IN_1);
            if(current_angle - start_angle >= 88)
            {
                Off(OUT_B);
                Off(OUT_A);
                break;
            }

       }
        OnFwdSync(OUT_AB, 8);
                Wait(MS_900);
                Off(OUT_B);
                Off(OUT_A);
}
void forward()
{
    OnFwdSync(OUT_AB, 15);
    Wait(SEC_4);
    Wait(MS_200);
    Off(OUT_AB);

}
int main(void)
{
	double xVal[17][11];
	double yVal[17][11];
	int mDist[17][11];
	int i,j,k, goalX, goalY = 0;
	double x,y = 0.305;
	//setting up x values for each cell
	for(j=0;j<=10;j++)
	{
		x = 0;
		for(i=0;i<=16;i++)
		{
			x = i * 0.305;
			xVal[i][j] = x;
		}
	}
	//setting up y values for each cell
	for(j=0;j<=16;j++)
	{
		y = 0;
		for(i=0;i<=10;i++)
		{
			y = i * 0.305;
			yVal[j][i] = y;
		}
	}
	//setting each manhattan distance to -1
	for(j=0;j<=10;j++)
	{
		for(i=0;i<=16;i++)
		{
			mDist[i][j] = -1;
		}
	}
	//checking each cell for to see if it is an obstacle if it is set manhattan distance to 100
	double upper_bound = 0;
	double lower_bound = 0;
	for(k=0;k<num_obstacles;k++)
	{
		for(j=0;j<=10;j++)
		{
			for(i=0;i<=16;i++)
			{
				upper_bound = obstacle[k][0] + 0.003;
				lower_bound = obstacle[k][0] - 0.003;
				x = xVal[i][j];
				//printf("for i = %d and j = %d: upper_bound = %f and lower_bound = %f and x = %f\n", i,j, upper_bound, lower_bound,x);
				if((x <= upper_bound) && (x >= lower_bound))
				{
					upper_bound = obstacle[k][1] + 0.003;
					lower_bound = obstacle[k][1] - 0.003;
					y = yVal[i][j];
					if((y <= upper_bound) && (y >= lower_bound))
					{
						mDist[i][j] = 100;
					}
				}
			}
		}
	}
	/*for(j=0;j<=10;j++)
	{
		for(i=0;i<=16;i++)
		{
			if(mDist[i][j] == 100)
			{
				printf("Cell [%d,%d] is an obstacle.\n", i, j);
			}
		}
	}*/
	//Start by finding where goal is
	for(j=0;j<=10;j++)
	{
	    for(i=0;i<=16;i++)
	    {
	        upper_bound = goal[0] + 0.003;
	        lower_bound = goal[0] - 0.003;
	        x = xVal[i][j];
	        //printf("for i = %d and j = %d: upper_bound = %f and lower_bound = %f and x = %f\n", i,j, upper_bound, lower_bound,x);
	        if((x <= upper_bound) && (x >= lower_bound))
	        {
	            upper_bound = goal[1] + 0.003;
	            lower_bound = goal[1] - 0.003;
	            y = yVal[i][j];
	            if((y <= upper_bound) && (y >= lower_bound))
	            {
	                goalX = i;
	                goalY = j;
	            }
	        }
	    }
	}
	//printf("The goal is at: [%d,%d]\n", goalX, goalY);
	// finding distance between every point and the goal
	for(j=0;j<=10;j++)
	{
		for(i=0;i<=16;i++)
		{
			if(mDist[i][j] != 100)
	        {
	            mDist[i][j] = abs(goalX - i) + abs(goalY - j);
	        }
		}
	}
	   for(j=0;j<=10;j++)
	    {
	        for(i=0;i<=16;i++)
	        {
	            if(mDist[i][j] < 100)
	            {
	                mDist[i][j] = abs(goalX - i) + abs(goalY - j);
	            }
	            /*if(mDist[i][j] >= 100)
	            {
	                mDist[i-1][j-1]++;
	                mDist[i][j-1]++;
	                mDist[i-1][j]++;
	                mDist[i+1][j+1]++;
	                mDist[i][j+1]++;
	                mDist[i+1][j]++;
	                mDist[i-1][j+1]++;
	                mDist[i+1][j-1]++;
	            }*/
	        }
	    }
	    for(j=0;j<=10;j++)
	    {
	        for(i=0;i<=16;i++)
	        {
	            if(mDist[i][j+1] == mDist[i][j-1] && mDist[i][j] != 0)
	            {
	                mDist[i][j] =mDist[i][j]+7;
	            }

	        }
	    }

	//this for loop prints the cells and the manhattan distance to the goal
	/*for(j=10;j>=0;j--)
	{
		printf("\n");
		for(i=0;i<=16;i++)
		{
			printf("%3d   ", mDist[i][j]);
		}
	}*/
	int startIndexes[2] = {0,0};
	double MtoIN = 3.281;
	for(i = 0; i < 2; i++)
	{
		double dX = start[i] * MtoIN;
		int iX = start[i] * MtoIN;
		double compX = dX - iX;
		if(compX > .5)
		{
			iX++;
		}
		startIndexes[i] = iX;
	}

	int goalIndex[2] = {0,0};
	for(i = 0; i < 2; i++)
	{
		double dX = start[i] * MtoIN;
		int iX = start[i] * MtoIN;
		double compX = dX - iX;
		if(compX > .5)
		{
			iX++;
		}
		goalIndex[i] = iX;
	}

	int path[50][2];
	for(i = 0; i < 50; i++)
	{
		path[i][0] = 0;
		path[i][1] = 0;
	}

	int visited[17][11];
	for(i = 0; i < 11; i++)
	{
		for(j = 0; i < 17; i++)
		{
			visited[i][j] = 0;
		}
	}
	int curr[2] = {startIndexes[0], startIndexes[1]};
	path[0][0] = curr[0];
	path[0][1] = curr[1];
	int pathIndex = 1;
	while(curr[0] != goalX || curr[1] != goalY)
	    {
	        visited[curr[0]][curr[1]] = 1;
	        int minIndex[2] = {};
	        int minVal = 100;

	        if(goalX < startIndexes[0]){
	            if(mDist[curr[0]+1][curr[1]] == mDist[curr[0]][curr[1]-1] && (visited[curr[0]+1][curr[1]] != 1 && visited[curr[0]][curr[1]-1] != 1))
	            {
	                if(mDist[curr[0]+1][curr[1]-1] > mDist[curr[0]-1][curr[1]-1])
	                {
	                    minVal = mDist[curr[0]][curr[1]-1];
	                    minIndex[0] = curr[0];
	                    minIndex[1] = curr[1]-1;
	                }
	                else
	                {
	                    minVal = mDist[curr[0] + 1][curr[1]];
	                    minIndex[0] = curr[0]+1;
	                    minIndex[1] = curr[1];
	                }
	            }
	            if(mDist[curr[0]-1][curr[1]] == mDist[curr[0]][curr[1]-1] && (visited[curr[0]-1][curr[1]] != 1 && visited[curr[0]][curr[1]-1] != 1))
	            {
	                if(mDist[curr[0]-1][curr[1]-1] < mDist[curr[0]-1][curr[1]+1])
	                {
	                    minVal = mDist[curr[0]][curr[1]-1];
	                    minIndex[0] = curr[0];
	                    minIndex[1] = curr[1]-1;
	                }
	                else
	                {
	                    minVal = mDist[curr[0] - 1][curr[1]];
	                    minIndex[0] = curr[0]-1;
	                    minIndex[1] = curr[1];
	                }
	            }
	            if( mDist[curr[0]-1][curr[1]] == mDist[curr[0]][curr[1]+1] && (visited[curr[0]-1][curr[1]] != 1 && visited[curr[0]][curr[1]+1] != 1))
	            {
	                if(mDist[curr[0]-1][curr[1]+1] < mDist[curr[0]+1][curr[1]+1])
	                {
	                    minVal = mDist[curr[0]][curr[1]+1];
	                    minIndex[0] = curr[0];
	                    minIndex[1] = curr[1]+1;
	                }
	                else
	                {
	                    minVal = mDist[curr[0] - 1][curr[1]];
	                    minIndex[0] = curr[0]-1;
	                    minIndex[1] = curr[1];
	                }
	            }
	            if( mDist[curr[0]+1][curr[1]] == mDist[curr[0]][curr[1]+1] && (visited[curr[0]+1][curr[1]] != 1 && visited[curr[0]][curr[1]+1] != 1))
	            {
	                if(mDist[curr[0]+1][curr[1]+1] > mDist[curr[0]+1][curr[1]-1])
	                {
	                    minVal = mDist[curr[0]][curr[1]+1];
	                    minIndex[0] = curr[0];
	                    minIndex[1] = curr[1]+1;
	                }
	                else
	                {
	                    minVal = mDist[curr[0] + 1][curr[1]];
	                    minIndex[0] = curr[0]+1;
	                    minIndex[1] = curr[1];
	                }
	            }
	        }
	        int compVal = mDist[curr[0]+1][curr[1]];
	        		if(minVal > compVal && visited[curr[0] + 1][curr[1]] != 1) // check right
	        		{
	        			minVal = mDist[curr[0] + 1][curr[1]];
	        			minIndex[0] = curr[0]+1;
	        			minIndex[1] = curr[1];
	        		}
	        		compVal = mDist[curr[0]][curr[1]-1];
	        		if(minVal > compVal && visited[curr[0]][curr[1]-1] != 1) // check below
	        		{
	        			minVal = mDist[curr[0]][curr[1]-1];
	        			minIndex[0] = curr[0];
	        			minIndex[1] = curr[1]-1;
	        		}
	        compVal = mDist[curr[0]][curr[1]+1];
	        		if(minVal > compVal && visited[curr[0]][curr[1]+1] != 1) // check above
	        		{
	        			minVal = mDist[curr[0]][curr[1]+1];
	        			minIndex[0] = curr[0];
	        			minIndex[1] = curr[1]+1;
	        		}
	        		compVal = mDist[curr[0]-1][curr[1]];
	        		if(minVal > compVal && visited[curr[0]-1][curr[1]] != 1) // check left
	        		{
	        			minVal = mDist[curr[0]-1][curr[1]];
	        			minIndex[0] = curr[0]-1;
	        			minIndex[1] = curr[1];
	        		}

	       // printf("%d %d\n", curr[0], curr[1]);
	        curr[0] = minIndex[0];
	        curr[1] = minIndex[1];

	        path[pathIndex][0] = curr[0];
	        path[pathIndex][1] = curr[1];

	        pathIndex++;
	    }
	int orientation = 1;

	InitEV3();
	setAllSensorMode(GYRO_ANG, NO_SEN, NO_SEN, NO_SEN);
	/*struct timeval begin_time;
	struct timeval end_time;
	int left_encoder, right_encoder;

	gettimeofday( &begin_time, NULL );

	//DO ACTION HERE
	ResetRotationCount(OUT_A);
	ResetRotationCount(OUT_B);*/

	TermPrintf("Running Path\n");
	//Moves the robot based on its position, orientation and next position
	for(i = 0; i < 50; i++)
	{
		if(path[i][0] == goalX && path[i][1] == goalY)
		{
			break;
		}
		else if(orientation == 1 && path[i][0]  > path[i+1][0])
		{
			orientation = 3;
			turnLeft();
			turnLeft();
			forward();
		}
		else if(orientation == 2 && path[i][1]  < path[i+1][1])
		{
			orientation = 4;
			turnLeft();
			turnLeft();
			forward();
		}
		else if(orientation == 3 && path[i][0]  < path[i+1][0])
		{
			orientation = 1;
			turnLeft();
			turnLeft();
			forward();
		}
		else if(orientation == 4 && path[i][1]  > path[i+1][1])
		{
			orientation = 2;
			turnLeft();
			turnLeft();
			forward();
		}
		else if((orientation == 1 || orientation == 3) && path[i][1]  == path[i+1][1])
		{
			forward();
		}
		else if((orientation == 2 || orientation == 4) && path[i][0]  == path[i+1][0])
		{
			forward();
		}
		else if(path[i][0] == path[i + 1][0] && path[i][1] < path[i+1][1] && orientation == 1)
		{
			orientation = 4;
			turnLeft();
			forward();
		}
		else if(path[i][0] == path[i + 1][0] && path[i][1] > path[i+1][1] && orientation == 1)
		{
			orientation = 2;
			turnRight();
			forward();
		}
		else if(path[i][0] == path[i + 1][0] && path[i][1] < path[i+1][1] && orientation == 3)
		{
			orientation = 4;
			turnRight();
			forward();
		}
		else if(path[i][0] == path[i + 1][0] && path[i][1] > path[i+1][1] && orientation == 3)
		{
			orientation = 2;
			turnLeft();
			forward();
		}
		//
		else if(path[i][0] < path[i + 1][0] && path[i][1] == path[i+1][1] && orientation == 2)
		{
			orientation = 1;
			turnLeft();
			forward();
		}
		else if(path[i][0] > path[i + 1][0] && path[i][1] == path[i+1][1] && orientation == 2)
		{
			orientation = 3;
			turnRight();
			forward();
		}
		else if(path[i][0] > path[i + 1][0] && path[i][1] == path[i+1][1] && orientation == 4)
		{
			orientation = 3;
			turnLeft();
			forward();
		}
		else if(path[i][0] < path[i + 1][0] && path[i][1] == path[i+1][1] && orientation == 4)
		{
			orientation = 1;
			turnRight();
			forward();
		}
	}

	//PRINT TO SCREEN
	TermPrintf("Press ENTER to stop\n");

	ButtonWaitForPress(BUTTON_ID_ENTER);

	FreeEV3();
	return 0;
}
