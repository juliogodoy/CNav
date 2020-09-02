/*Code used for the C-Nav vs ORCA experiments
 * Based on the RVO2 library.
 * Author: Julio Godoy Del Campo
 * Please send all comments and/or questions to juliogodoy@gmail.com
 *  
 * */
#include <iostream>
#include <vector>
#include <fstream>
//#include <gl/glut.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <string.h>
#include <algorithm>
#include "simulationDefs.h"



#if HAVE_OPENMP || _OPENMP
#include <omp.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>   
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#ifdef __APPLE__
#include <RVO/RVO.h>
#else
#include "RVO.h"
#endif

#ifndef M_PI
static const float M_PI = 3.14159265358979323846f;
#endif


void InitGL(void)     // OpenGL function
{
	
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearColor(0.5f, 0.5f, 0.5f, 0.5f);				// Black Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glEnable ( GL_COLOR_MATERIAL );
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}


void reshape(int width, int height) //OpenGL function
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	

	float q = width/(float)height;
 
    switch(scenario)
	{
		
	case 1: glOrtho(-10,sidesize+10,-10,sidesize+10, -2, 20);
            break;
            
        case 2: gluOrtho2D(-q*45, q*45, -45, 45 );
            break;
            
        case 3: gluOrtho2D(-q*20, q*20, 10, 50 );
            break;
            
        case 4: gluOrtho2D(-q*25, q*25, -25, 25 );
            break;
            
        case 5: glOrtho(-20,10,-10,10, -2, 20);
            break;
            
        case 6: gluOrtho2D(-q*15, q*15, -15, 15 );
            break;
            
        case 7: glOrtho(-30,30,-20,20, -2, 20);
            break;
        case 8: glOrtho(-10,sidesize+10,-10,sidesize+10, -2, 20);
            break; 
           
        case 9: glOrtho(-20,10,-10,10, -2, 20);
            break;
            
        case 10: glOrtho(-5,35,-10,20, -5, 35);//gluOrtho2D(-q*45, q*45, -45, 45 );
            break;
           
		
	}
	
	
	
	glMatrixMode(GL_MODELVIEW);
}

void renderBitmapString(float x, float y, void *font, char *string) 
{  
	char *c;
	glRasterPos2f(x,y);
	for (c=string; *c != '\0'; c++) 
	{
		glutBitmapCharacter(font, *c);
	}
}

void idle(void) //OpenGL function
{
	glutPostRedisplay();
}

void countCollisions()
{
	int i,j;
	
	for(i=0;i<Agents;i++)
	{
		
		
			if(isinGoal[i])
			{ 
				continue;
				
			}
			
			for(j=0;j<Agents;j++)
			{
				
				if(isinGoal[j])
				{
					continue;
					
				}
				
				if(i!=j)
				{
					
					float distancia;
					distancia=RVO::abs(sim->getAgentPosition(i)-sim->getAgentPosition(j));
					
					
			
					if(distancia<.95*(sim->getAgentRadius(i)+ sim->getAgentRadius(j) ) )
					{
						
						colisiones++;
						
						
					}
					
					
				}
				
			}
		
	}
	
}

int getClosestToGoal()
{	
	float min_distance_to_goal=10000, dist_to_goal;
	int closest=-1;		
	
			for (size_t i = 0; i < sim->getNumAgents(); ++i) 
			{
				if(!isinGoal[i])
				{
					dist_to_goal= RVO::abs(goals[i]-sim->getAgentPosition(i));
					if(dist_to_goal<min_distance_to_goal)
					{
						min_distance_to_goal=dist_to_goal;
						closest=i;
						}
					
				}
				
				
		    }
		    
		    return closest;
}



bool checkOverlap(int i)  //Checks whether the position of agent i overlaps the position of an agent previously located (in which case it returns 0)
{
	for (int j=0;j<i;j++)
	{
		if(RVO::abs(sim->getAgentPosition(j)-sim->getAgentPosition(i))<2*sim->getAgentRadius(i))
		{
			return 0;	
		}
		
	}
	
	return 1;
}


void genScenarioCong() //Generates the positions of the agents in the Crowd scenario
{
	float xpos,xgpos, ypos,ygpos;
	srand (time(NULL));
	RVO::Vector2 tempgoal,initpos;
	std::ofstream initpositions; 
	//std::cout << "Generating agents..\n";
	initpositions.open("positions.txt", std::fstream::trunc);
	for (int i=0; i<Agents; i++) 
		{
			
		sim->addAgent(RVO::Vector2());
		
		do{
			xpos= (rand()%9)-4;
			ypos= (rand()%20)-9.5;
			
			initpos=RVO::Vector2(xpos,ypos);
			sim->setAgentPosition(i,initpos);
		
		}while(!checkOverlap(i));  
		
		
		initpositions << sim->getAgentPosition(i).x() << "\n"<< sim->getAgentPosition(i).y() << "\n" << xgpos <<"\n" << ygpos<<"\n";
		
		}
		
 	initpositions.close();
 	
}


void genScenario() //Generates the positions of the agents in the Crowd scenario
{
	float xpos,xgpos, ypos,ygpos;
	srand (time(NULL));
	RVO::Vector2 tempgoal,initpos;
	std::ofstream initpositions; 
	//std::cout << "Generating agents..\n";
	initpositions.open("positions.txt", std::fstream::trunc);
	for (int i=0; i<Agents; i++) 
		{
			
		sim->addAgent(RVO::Vector2());
		
		do{
			xpos= rand()%sidesize;
			ypos= rand()%sidesize;
			
			initpos=RVO::Vector2(xpos,ypos);
			sim->setAgentPosition(i,initpos);
			do{
				xgpos= rand()%(sidesize);
				ygpos= rand()%(sidesize);
				tempgoal=RVO::Vector2(xgpos,ygpos);
			}while((RVO::abs(tempgoal-initpos)<((float)sidesize/(float)2))||((xgpos>=(((float)sidesize/(float)2)-3))&&(xgpos<=(((float)sidesize/(float)2)+3))&&(ygpos>=(((float)sidesize/(float)2)-3))&&(ygpos<=(((float)sidesize/(float)2)+3))));
			
		}while((!checkOverlap(i))||((xpos>=(((float)sidesize/(float)2)-3))&&(xpos<=(((float)sidesize/(float)2)+3))&&(ypos>=(((float)sidesize/(float)2)-3))&&(ypos<=(((float)sidesize/(float)2)+3))));  //||((xpos>=6)&&(xpos<=9)&&(ypos<=(sidesize-6))&&(ypos>=(sidesize-9))) ||((xpos>=16)&&(xpos<=19)&&(ypos<=(sidesize-16))&&(ypos>=(sidesize-19))));// ||((xpos>=4)&&(xpos<=5)&&(ypos<=(sidesize-15))&&(ypos>=(sidesize-17))) ||((xpos>=13)&&(xpos<=17)&&(ypos<=(sidesize-4))&&(ypos>=(sidesize-6)))   );
		
		
		initpositions << sim->getAgentPosition(i).x() << "\n"<< sim->getAgentPosition(i).y() << "\n" << xgpos <<"\n" << ygpos<<"\n";
		
		
		goals.push_back(RVO::Vector2(xgpos,ygpos));
		
		 sim->setAgentGoal(i, RVO::Vector2(xgpos,ygpos));
		
		}
		
 	initpositions.close();
 	
}

void getScenario()  //Reads the position of the agents in the Crowd scenario, for a previously generated environment.
{
	std::string line;
	int i=0,pos1,pos2,goal1,goal2;
	std::ifstream initpositions;
	
	initpositions.open("positions.txt", std::ios_base::app);

	if (initpositions.is_open())
	{
		while ( getline (initpositions,line) )
		{
			pos1=atoi(line.c_str());
			getline(initpositions,line);
			pos2=atoi(line.c_str());
			getline(initpositions,line);
			goal1=atoi(line.c_str());
			getline(initpositions,line);
			goal2=atoi(line.c_str());
			sim->setAgentPosition(i, RVO::Vector2(pos1,pos2));
			goals.push_back(RVO::Vector2(goal1,goal2));
			sim->setAgentGoal(i, RVO::Vector2(goal1,goal2));
			
			i=i+1;
		}
		initpositions.close();
	}
	
	else std::cout << "Unable to open file";
	
}


float getOriSimilarNeighbor(int i, int k)
{
	int agenteSimilar =k;
	
	
	RVO::Vector2 VelNorm = RVO::normalize(sim->getAgentVelocity(agenteSimilar));
	
	RVO::Vector2 correctedNeighborPos= (sim->getAgentPosition(agenteSimilar)-VelNorm*sim->getAgentRadius(i));//- sim->getAgentPosition(i);
	
	
	
	RVO::Vector2 RelPosNeigh=correctedNeighborPos-sim->getAgentPosition(i) ;
	
	float Cx= correctedNeighborPos.x();
	
	float Ax = sim->getAgentPosition(i).x();
	
	RVO::Vector2 PrefVelPoint= RVO::Vector2(sim->getAgentPosition(i).x()+ RVO::normalize(goals[i] - sim->getAgentPosition(i)).x(), sim->getAgentPosition(i).y()+ RVO::normalize(goals[i] - sim->getAgentPosition(i)).y());
	
	
	float Cy =correctedNeighborPos.y();
	float By =PrefVelPoint.y();
	float Bx =PrefVelPoint.x();
	float Ay = sim->getAgentPosition(i).y();
	
	float lado = (Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax);
	
	float tempp=((RelPosNeigh*RVO::normalize(goals[i] - sim->getAgentPosition(i)))/(RVO::abs(RelPosNeigh)*RVO::abs(RVO::normalize(goals[i] - sim->getAgentPosition(i))) )); 
	
	float angulorad;
	
	
	if(((tempp-1)<0.000001)&&((tempp-1)>-0.000001))
	{	
		angulorad=0;		
	}
	else
	{
		
		angulorad= acos((RelPosNeigh*RVO::normalize(goals[i] - sim->getAgentPosition(i)))/(RVO::abs(RelPosNeigh)*RVO::abs(RVO::normalize(goals[i] - sim->getAgentPosition(i))) ));
	}
	float angulodeg= angulorad*180/M_PI;
	
	
	if((lado>0)&&(angulodeg<180)) // it is on the other side:
	{
		
		
		angulodeg=360-angulodeg;
		angulorad=-angulorad;
		
	}
	
	
		
	return angulorad;
	
	
}



void computeSimilarAgentsDir(int i)
{
	
	std::vector< std::pair<float, int> > vect;
	totalConsideredNeighs[i]=0;
	
	for(int j=0; j<sim->getAgentNumAgentNeighbors(i); j++)
	{
		float GoalDist = RVO::abs(goals[i]-sim->getAgentPosition(i))-RVO::abs(goals[i]-sim->getAgentPosition(sim->getAgentAgentNeighbor(i,j)));
		float PrefVelSim=sim->getAgentGoalPrefVelocity(i)*sim->getAgentPrefVelocity(sim->getAgentAgentNeighbor(i,j));
		
		 if(i==agentViewed)
        {
			std::cout << "Agent " <<sim->getAgentAgentNeighbor(i,j) << " has GoalDist " << GoalDist<< " and PrefVelSim " << PrefVelSim <<  " cause my goalVpref is " << sim->getAgentGoalPrefVelocity(i) <<" \n";
		}
		
			
		if((isinGoal[sim->getAgentAgentNeighbor(i,j)]==0)&&(GoalDist>0.0)&&(PrefVelSim>0))
		{ //Here it should only consider those agents closer to the goal for similarity
			
		if( (goalvectors[i]*sim->getAgentVelocity(sim->getAgentAgentNeighbor(i,j)))>0) // If neighbor is actually going in the same direction as my goal
			{
				
		
			vect.push_back(std::make_pair( (goalvectors[i]*sim->getAgentVelocity(sim->getAgentAgentNeighbor(i,j)))/(float)(RVO::abs(goalvectors[i]*1.5))    , sim->getAgentAgentNeighbor(i,j)));
		    totalConsideredNeighs[i]++;		   
		
			}
		}
		
		
	}
	
		
	sort(vect.begin(),vect.end());		
	int contadour2=0;
	
	float politeness4=0;
	
	
	indexMostSimilar[i]=-1;
	for (int e=0; e<totalConsideredNeighs[i]; e++) 
    { 
		mostSimilar[i][e]=-1;
		 
           
       
		
		indexMostSimilar[i]= vect[e].second;
        mostSimilar[i][e]= vect[e].second;
        oriMostSimilar[i][e]= getOriSimilarNeighbor(i,mostSimilar[i][e]);
         
             politeness4=politeness4+ vect[e].first;
             contadour2++;
            
    } 
	
	if(indexMostSimilar[i]>-1)
	{
	float aver= getOriSimilarNeighbor(i,indexMostSimilar[i]);
	
	 if(i==agentViewed)
        {
			std::cout << " The most similar is... " << indexMostSimilar[i] << " with an angle of " << aver<< "\n";
			
		}
	
	
	}
	
	
}


float computeSimilarAgentSpeed(int i,int similar, RVO::Vector2 goalVect)
{
	
	return (goalVect*sim->getAgentVelocity(similar));
}



void simulateVelocities()  //Simulates the execution of each action/velocity for a number of timesteps (defined in timeHorizon) in the future.
{
	
    sim->buildTree();
    sim->setVvalues();
    int maxEvalActions=Actions;
	
    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) 
    {
		
	
		if(needtoUpdate[i])
		{
			if(!eval_method[i])
			{
			
			float finalrw, finalaction=0;
						
			if(followNeigh)
			{
			computeSimilarAgentsDir(i);
			maxEvalActions=Actions-1;
		
			}
			else
			{
			totalConsideredNeighs[i]=0;	
	
	
		
			}
	

	if(followNeigh)
	{
		
		bestSimilarNeigh[i]=-1;
		float bestFollowMove=-100, rewardFollow=-1;
	
		if(totalConsideredNeighs[i]>0)
		{	
			for (int e=0; e<totalConsideredNeighs[i]; e++) 
			{
				ActionDir[i][Actions-1]= getOriSimilarNeighbor(i,mostSimilar[i][e]);
		
				ActionSpeed[i][Actions-1] = computeSimilarAgentSpeed(i,mostSimilar[i][e],goalvectors[i]);
		
				rewardFollow= sim->SimulateVel(i, goals[i], timeHorizon, Actions-1,numNeighbors, Agents,  coord_factor, coord_factor_alan,ActionSpeed[i][Actions-1],ActionDir[i][Actions-1], allNeigh, contadourX);
	 
	
				if(i==agentViewed)
				{
					std::cout << "************ " << e << " similar neigh is : " << mostSimilar[i][e] << " and value is " <<rewardFollow << "\n";
				}
	
				if(rewardFollow>bestFollowMove)
				{
					bestSimilarNeigh[i]= mostSimilar[i][e];
					bestFollowMove = rewardFollow;
				}
	
	
	
			}
	
			ActionDir[i][Actions-1]= getOriSimilarNeighbor(i,bestSimilarNeigh[i]);
			ActionSpeed[i][Actions-1]=computeSimilarAgentSpeed( i, bestSimilarNeigh[i],goalvectors[i]);
			RewardAvg[i][Actions-1]= bestFollowMove;

			if(i==agentViewed)
			{
				std::cout << "************ Best Similar neigh is : " << bestSimilarNeigh[i] << " and value is " <<bestFollowMove << " Dir " <<ActionDir[i][Actions-1] << " Mag: " <<ActionSpeed[i][Actions-1] <<  "\n";
			}
		}

    }
	

	if((totalConsideredNeighs[i]<1)&&(followNeigh))
	{
		ActionDir[i][Actions-1]= 0.0000;
		ActionSpeed[i][Actions-1] = 1.5;
		RewardAvg[i][Actions-1]=-1000;	
	}
							
		for (int a=0;a<maxEvalActions; a++)
		{   
			RewardAvg[i][a]=0;
			RewardAvg[i][a] = sim->SimulateVel(i, goals[i], timeHorizon, a,numNeighbors, Agents,  coord_factor, coord_factor_alan,ActionSpeed[i][a],ActionDir[i][a], allNeigh, contadourX);
		}
				
		finalrw=-1000;
		float totalCnavPoliteness=0;
		for (int ac=0;ac<Actions; ac++)
		{
			if(i==agentViewed)
			{
		   std::cout << " Action " << ac << " value: " << 	RewardAvg[i][ac] << " goalProg: " << sim->getAgentGoalProg(i,ac) << " CNAVPol: " << sim->getAgentCnavPoliteness(i,ac) /*<< " ALANPol: " <<sim->getAgentALANPoliteness(i,ac)*/ << "\n";
			}
						
			totalCnavPoliteness+=sim->getAgentCnavPoliteness(i,ac);
					
			if(RewardAvg[i][ac]>=(finalrw))
			{
				finalrw=RewardAvg[i][ac];
			    finalaction=ac;
							
			}
						
		}
					
		float avgCnavPoliteness=totalCnavPoliteness/(float)Actions;
				
				
		float tempstd=0;
		
            
        for(int ac=0;ac<Actions;ac++)
        {
            tempstd=tempstd+ ((sim->getAgentCnavPoliteness(i,ac)-avgCnavPoliteness)*(sim->getAgentCnavPoliteness(i,ac)-avgCnavPoliteness));
                
         
        }
            
            
        tempstd=tempstd/(float)(Actions-1);
			
        tempstd=sqrt(tempstd);
				
		if(avgCnavPoliteness>0.99) //means that neighbors are immune to actions of this agent 
		{
					
				  //use ALAN politness	
				  finalrw=-1000;
				  for (int ac=0;ac<Actions; ac++)
					{
					
						
						if(RewardAvg[i][ac]>=(finalrw))
						{
							finalrw=RewardAvg[i][ac];
						    finalaction=ac;
							
						}
						
						
						
						
					}
					
		}
					
				chosenAction[i]=finalaction;
				
				if(i==agentViewed)
						{
							std::cout << " CHOSEN ACTION: " << chosenAction[i]<< "\n";
						}
				
				if((scenario==3)&&(i==agentViewed)&&(timestep>1))
						{
							getchar();
							int n;
							scanf("%d",&n);
					    }
				
		}
			
			
			
		} 
		 
    }
  
}

void setActions(int i)  //Sets the actions for each agent
{
    int  o;
    float novector, fullvector;
    for(o=0;o<Actions;o++)
    {
        ActionSpeed[i][o]=actionVectorMag[o];
        ActionDir[i][o]= actionVector[o];
         goalvectors[i]= RVO::normalize(goals[i] - sim->getAgentPosition(i));
          RVO::Vector2 pVel = RVO::Vector2((RVO::normalize(goalvectors[i])*ActionSpeed[i][o]).x()*std::cos(ActionDir[i][o])+(goalvectors[i]*ActionSpeed[i][o]).y()*std::sin(ActionDir[i][o]), (goalvectors[i]*ActionSpeed[i][o]).y()*std::cos(ActionDir[i][o])+(goalvectors[i]*ActionSpeed[i][o]).x()*-std::sin(ActionDir[i][o])); 
		  novector = (1-coord_factor)*(RVO::Vector2(0,0)*goalvectors[i])/(float)1.5 +coord_factor*(RVO::Vector2(0,0)*pVel)/(float)2.25 ;
		  fullvector= (1-coord_factor)*(pVel*goalvectors[i])/(float)1.5 +coord_factor*(pVel*pVel)/(float)2.25 ;
		  
		  		
		  		 if(fullvector>novector)
		  		 {
					defaultValue[i][o]= fullvector;
					}
					else
					{
						defaultValue[i][o]= novector;
					}
		  		 
		  		 if (i==agentViewed)
		  		 {
					 std::cout << "Default value action " << o << " is " << defaultValue[i][o] << "\n";
					 
					 
					 
				}
		  		 
    }
    float suma=0;
    for (o=0; o<Actions; o++) 
	{ 	
		suma=suma+defaultValue[i][o];
	}
	
	if (i==agentViewed)
		  		 {
					 std::cout << "SUM " << suma << " AVG " << suma/(float)Actions << "\n";

				}
	
	for (o=0; o<Actions; o++) 
	{
		Boltz[i][o]=100*(exp(defaultValue[i][o]/(float).05))/suma;
	}
    
}

void setPreferredVelocities() //Computes the preferred velocity of the agents based on the action chosen (chosenAction[])
{
	
	
	for (int i = 0; i <Agents ; ++i) 
	{
		/* 
		 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
		 * direction of the goal.
		 */
	
		if(isinGoal[i]==0) //If agent i has not reached its goal
		{
							
				goaldistance[i]=RVO::abs(goals[i] - sim->getAgentPosition(i));
				goalvectors[i]= RVO::normalize(goals[i] - sim->getAgentPosition(i));
				goalx[i]=goalvectors[i].x();
				goaly[i]=goalvectors[i].y();
			
			    sim->setAgentPrefVelocity(i, goalvectors[i]*ActionSpeed[i][chosenAction[i]]);
			    sim->setAgentGoalPrefVelocity(i, goalvectors[i]*1.5);
			
			if(algo==1)
			{
				
				
				angle = std::rand() * 2.0f * M_PI / RAND_MAX;
				dist = std::rand() * 0.01f / RAND_MAX;
				
				sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) + dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
				
			}
			
			
			
			
			if(algo==2)
			{
				angle =  ActionDir[i][chosenAction[i]];
				dist = std::rand() * 0.01f / RAND_MAX;
				sim->setAgentPrefVelocity(i, RVO::Vector2(sim->getAgentPrefVelocity(i).x()*std::cos(angle)+sim->getAgentPrefVelocity(i).y()*std::sin(angle),  sim->getAgentPrefVelocity(i).y()*std::cos(angle)+sim->getAgentPrefVelocity(i).x()*-std::sin(angle)) );
	
	
	/*
			 * Perturb a little to avoid deadlocks due to perfect symmetry.
			 */
					if(randomPert)
					{
					angle = std::rand() * 2.0f * M_PI / RAND_MAX;
					
					sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +  dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
					}
					
					
			}
		
			
		}
		else  //if agent i has reached its goal, move it to position (-1000, -1000) and stop it
		{
			sim->setAgentPosition(i,RVO::Vector2(-1000.0f,-1000.0f));
			sim->setAgentVelocity(i, RVO::Vector2());
			
		}
		
		
	}
	
	
	
}

void setupScenario(RVO::RVOSimulator* sim) //Initialize the Simulation, positions of the obstacles and the agents
{
	// Specify the global time step of the simulation. 
	sim->setTimeStep(simTimeStep);
	
	int j;
	totalingoal=0;
	timestep=0;
	totalrewardsimulation=0;
  
	
	for (int i = 0; i < Agents; ++i) 
	{
		
		needtoUpdate[i]=0;
		sim->setAgentDefaults(neighborDistance, numNeighbors, timeHorizonORCA , timeHorizonObstORCA , radiusORCA , maxSpeedORCA);
		chosenAction[i]=0;
		currentVel[i]=0;
		eval_method[i]=0; // 0 uses simulated reward, 1 uses reward based on execution
		
		TimetoGoal[i]=0;

		isinGoal[i]=0;
		
	
				
		for(j=0;j<Actions;j++)
		{
			RewardAvg[i][j]=0;
			LastActionEstimate[i][j]=0;
		}						
		
		
		if(algo==1)
		{
			ActionSpeed[i][0]=1.5;
			ActionDir[i][0]=0;
			chosenAction[i]=0;
		}
		
		
		
	
		
	}
	
	// Adding Obstacles for each scenario
	
	
	
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4,obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10, obstacle11, obstacle12, obstacle13;
	
	    if(scenario==3)
    {
        obstacle1.push_back(RVO::Vector2(200.0f,   31.0f));
        obstacle1.push_back(RVO::Vector2(-200.0f,   31.0f));
        obstacle1.push_back(RVO::Vector2(-200.0f, 30.6f));
        obstacle1.push_back(RVO::Vector2(200.0f, 30.6f));
        
        obstacle2.push_back(RVO::Vector2(200.0f,   27.4f));
        obstacle2.push_back(RVO::Vector2(-200.0f, 27.4f));
        obstacle2.push_back(RVO::Vector2(-200.0f, 27.0f));
        obstacle2.push_back(RVO::Vector2(200.0f, 27.0f));
        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        
        sim->processObstacles();
        
    }
    
  if(scenario==4)
  {
	  
	    obstacle1.push_back(RVO::Vector2(200.0f,   3.0f));
        obstacle1.push_back(RVO::Vector2(2.0f,   3.0f));
        obstacle1.push_back(RVO::Vector2(2.0f, 2.0f));
        obstacle1.push_back(RVO::Vector2(200.0f, 2.0f));
        
        obstacle2.push_back(RVO::Vector2(200.0f,   -2.0f));
        obstacle2.push_back(RVO::Vector2(2.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(2.0f, -3.0f));
        obstacle2.push_back(RVO::Vector2(200.0f, -3.0f));
        
        
        
        obstacle3.push_back(RVO::Vector2(-2.0f,   3.0f));
        obstacle3.push_back(RVO::Vector2(-200.0f,   3.0f));
        obstacle3.push_back(RVO::Vector2(-200.0f, 2.0f));
        obstacle3.push_back(RVO::Vector2(-2.0f, 2.0f));
        
        obstacle4.push_back(RVO::Vector2(-2.0f, -2.0f));
        obstacle4.push_back(RVO::Vector2(-200.0f,   -2.0f));
        obstacle4.push_back(RVO::Vector2(-200.0f, -3.0f));
        obstacle4.push_back(RVO::Vector2(-2.0f, -3.0f));
        
        
        
        
	    obstacle5.push_back(RVO::Vector2(3.0f,200.0f));
        obstacle5.push_back(RVO::Vector2(2.0f, 200.0f));
        obstacle5.push_back(RVO::Vector2(2.0f, 3.0f));
        obstacle5.push_back(RVO::Vector2(3.0f, 3.0f));
        
        obstacle6.push_back(RVO::Vector2(-2.0f,   200.0f));
        obstacle6.push_back(RVO::Vector2(-3.0f, 200.0f));
        obstacle6.push_back(RVO::Vector2(-3.0f, 3.0f));
        obstacle6.push_back(RVO::Vector2(-2.0f, 3.0f));
        
        
        
        obstacle7.push_back(RVO::Vector2(3.0f,   -3.0f));
        obstacle7.push_back(RVO::Vector2(2.0f,   -3.0f));
        obstacle7.push_back(RVO::Vector2(2.0f, -200.0f));
        obstacle7.push_back(RVO::Vector2(3.0f, -200.0f));
        
        obstacle8.push_back(RVO::Vector2(-2.0f, -3.0f));
        obstacle8.push_back(RVO::Vector2(-3.0f,   -3.0f));
        obstacle8.push_back(RVO::Vector2(-3.0f, -200.0f));
        obstacle8.push_back(RVO::Vector2(-2.0f, -200.0f));
        
        
	  

        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        
         sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        sim->addObstacle(obstacle7);
        sim->addObstacle(obstacle8);
        
        sim->processObstacles(); 
	  
	 }  
	
	
	if(scenario==5)
	{ 
		
		
		obstacle1.push_back(RVO::Vector2(-10.0f,   350.0f));
		obstacle1.push_back(RVO::Vector2(-11.0f,   350.0f));
		obstacle1.push_back(RVO::Vector2(-11.0f, 0.7f));
		obstacle1.push_back(RVO::Vector2(-10.0f, 0.7f));
					
		obstacle2.push_back(RVO::Vector2(-10.0f,   -0.7f));
		obstacle2.push_back(RVO::Vector2(-11.0f, -0.7f));
		obstacle2.push_back(RVO::Vector2(-11.0f, -150.0f));
		obstacle2.push_back(RVO::Vector2(-10.0f, -150.0f));
							
		sim->addObstacle(obstacle1);
		sim->addObstacle(obstacle2);
				
		sim->processObstacles();
	}
		
		
	if(scenario==10) 
		{
			obstacle1.push_back(RVO::Vector2(0.0f,   12.0f));
			obstacle1.push_back(RVO::Vector2(-2.0f,   12.0f));
			
			obstacle1.push_back(RVO::Vector2(-2.0f,   -2.0f));
			obstacle1.push_back(RVO::Vector2(0.0f,   -2.0f));
			

			obstacle2.push_back(RVO::Vector2(30.0f,   0.0f));
			obstacle2.push_back(RVO::Vector2(0.0f,   0.0f));
			
			obstacle2.push_back(RVO::Vector2(0.0f,   -2.0f));
			obstacle2.push_back(RVO::Vector2(30.0f,   -2.0f));
			
			
			obstacle3.push_back(RVO::Vector2(30.0f,   12.0f));
			obstacle3.push_back(RVO::Vector2(0.0f,   12.0f));
			
			obstacle3.push_back(RVO::Vector2(0.0f,   10.0f));
			obstacle3.push_back(RVO::Vector2(30.0f,   10.0f));
			
			
			
			obstacle4.push_back(RVO::Vector2(32.0f,   12.0f));
			obstacle4.push_back(RVO::Vector2(30.0f,   12.0f));
			
			obstacle4.push_back(RVO::Vector2(30.0f,   -2.0f));
			obstacle4.push_back(RVO::Vector2(32.0f,   -2.0f));
			
			
			
			
			
			obstacle5.push_back(RVO::Vector2(11.0f,   2.95f));
			obstacle5.push_back(RVO::Vector2(5.0f,   2.95f));
			
			obstacle5.push_back(RVO::Vector2(5.0f,   1.05f));
			obstacle5.push_back(RVO::Vector2(11.0f,   1.05f));
			
			
			obstacle6.push_back(RVO::Vector2(11.0f,   5.95f));
			obstacle6.push_back(RVO::Vector2(5.0f,   5.95f));
			
			obstacle6.push_back(RVO::Vector2(5.0f,   4.05f));
			obstacle6.push_back(RVO::Vector2(11.0f,   4.05f));
			
			
			obstacle7.push_back(RVO::Vector2(11.0f,   8.95f));
			obstacle7.push_back(RVO::Vector2(5.0f,   8.95f));
			
			obstacle7.push_back(RVO::Vector2(5.0f,   7.05f));
			obstacle7.push_back(RVO::Vector2(11.0f,   7.05f));
			
			
			
			obstacle8.push_back(RVO::Vector2(18.0f,   8.95f));
			obstacle8.push_back(RVO::Vector2(12.0f,   8.95f));
			
			obstacle8.push_back(RVO::Vector2(12.0f,   7.05f));
			obstacle8.push_back(RVO::Vector2(18.0f,   7.05f));
			
			
			obstacle9.push_back(RVO::Vector2(18.0f,   2.95f));
			obstacle9.push_back(RVO::Vector2(12.0f,   2.95f));
			
			obstacle9.push_back(RVO::Vector2(12.0f,   1.05f));
			obstacle9.push_back(RVO::Vector2(18.0f,   1.05f));
			
			
			
			obstacle10.push_back(RVO::Vector2(18.0f,   5.95f));
			obstacle10.push_back(RVO::Vector2(12.0f,   5.95f));
			
			obstacle10.push_back(RVO::Vector2(12.0f,   4.05f));
			obstacle10.push_back(RVO::Vector2(18.0f,   4.05f));
			
			
			
			obstacle11.push_back(RVO::Vector2(25.0f,   8.95f));
			obstacle11.push_back(RVO::Vector2(19.0f,   8.95f));
			
			obstacle11.push_back(RVO::Vector2(19.0f,   7.05f));
			obstacle11.push_back(RVO::Vector2(25.0f,   7.05f));
			
			
			obstacle12.push_back(RVO::Vector2(25.0f,   2.95f));
			obstacle12.push_back(RVO::Vector2(19.0f,   2.95f));
			
			obstacle12.push_back(RVO::Vector2(19.0f,   1.05f));
			obstacle12.push_back(RVO::Vector2(25.0f,   1.05f));
			
			
			
			obstacle13.push_back(RVO::Vector2(25.0f,   5.95f));
			obstacle13.push_back(RVO::Vector2(19.0f,   5.95f));
			
			obstacle13.push_back(RVO::Vector2(19.0f,   4.05f));
			obstacle13.push_back(RVO::Vector2(25.0f,   4.05f));
			
			
			
		sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        
        sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        sim->addObstacle(obstacle7);
        
        sim->addObstacle(obstacle8);
        sim->addObstacle(obstacle9);
        sim->addObstacle(obstacle10);
        
        sim->addObstacle(obstacle11);
        sim->addObstacle(obstacle12);
        sim->addObstacle(obstacle13);
        
        sim->processObstacles();
			
		}
	
		if(scenario==1) 
		{
		
				
			obstacle1.push_back(RVO::Vector2(-1.5f,   -0.5f));
			obstacle1.push_back(RVO::Vector2(-1.5f,   -1.5f));
			obstacle1.push_back(RVO::Vector2(sidesize+1.5f, -1.5f));
			obstacle1.push_back(RVO::Vector2(sidesize+1.5f, -0.5f));
				
			obstacle2.push_back(RVO::Vector2(sidesize+0.5f,   sidesize+1.5f));
			obstacle2.push_back(RVO::Vector2(sidesize+0.5f,   -1.5f));
			obstacle2.push_back(RVO::Vector2(sidesize+1.5f, -1.5f));
			obstacle2.push_back(RVO::Vector2(sidesize+1.5f, sidesize+1.5f));
				
			obstacle3.push_back(RVO::Vector2(-1.5f,   sidesize+1.5f));
			obstacle3.push_back(RVO::Vector2(-1.5f,   sidesize+0.5f));
			obstacle3.push_back(RVO::Vector2(sidesize+1.5f,  sidesize+0.5f));
			obstacle3.push_back(RVO::Vector2(sidesize+1.5f,  sidesize+1.5f));
				
			obstacle4.push_back(RVO::Vector2(-1.5f,   sidesize+1.5f));
			obstacle4.push_back(RVO::Vector2(-1.5f,   -1.5f));
			obstacle4.push_back(RVO::Vector2(-0.5f, -1.5f));
			obstacle4.push_back(RVO::Vector2(-0.5f, sidesize+1.5f));
					
			sim->addObstacle(obstacle1);
			sim->addObstacle(obstacle2);
			sim->addObstacle(obstacle3);
			sim->addObstacle(obstacle4);
		
				
			sim->processObstacles();
				
			if(iteration>1)
			{
				getScenario();
				
			}
				 
			if(iteration==1)
			{
				genScenario();
			
			}
			
		}
		
		   
    if(scenario==7)
    {
        obstacle1.push_back(RVO::Vector2(-10.0f,   2.0f));
        obstacle1.push_back(RVO::Vector2(-10.0f,   0.6f));
        obstacle1.push_back(RVO::Vector2(10.0f, 0.6f));
        obstacle1.push_back(RVO::Vector2(10.0f, 2.0f));
        
        obstacle2.push_back(RVO::Vector2(-10.0f,   -0.6f));
        obstacle2.push_back(RVO::Vector2(-10.0f,   -2.0f));
        obstacle2.push_back(RVO::Vector2(10.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(10.0f, -0.6f));
        
        obstacle3.push_back(RVO::Vector2(-100.0f,   100.0f));
        obstacle3.push_back(RVO::Vector2(-100.0f,   10.0f));
        obstacle3.push_back(RVO::Vector2(-10.0f, 0.6f));
        obstacle3.push_back(RVO::Vector2(-10.0f, 100.0f));
        
         obstacle4.push_back(RVO::Vector2(-100.0f,   -10.0f));
        obstacle4.push_back(RVO::Vector2(-100.0f,   -100.0f));
        obstacle4.push_back(RVO::Vector2(-10.0f, -100.0f));
        obstacle4.push_back(RVO::Vector2(-10.0f, -0.6f));
        
        
        obstacle5.push_back(RVO::Vector2(10.0f,   100.0f));
        obstacle5.push_back(RVO::Vector2(10.0f,   0.6f));
        obstacle5.push_back(RVO::Vector2(100.0f, 10.0f));
        obstacle5.push_back(RVO::Vector2(100.0f, 100.0f));
        
         obstacle6.push_back(RVO::Vector2(10.0f,   -0.6f));
        obstacle6.push_back(RVO::Vector2(10.0f,   -100.0f));
        obstacle6.push_back(RVO::Vector2(100.0f, -100.0f));
        obstacle6.push_back(RVO::Vector2(100.0f, -10.0f));
        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        
         sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        
          sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        
        sim->processObstacles();
    }
    
      
   
    
    if(scenario==8)
    {
		
		
        
        obstacle1.push_back(RVO::Vector2(-1.0f,   0.5f));
        obstacle1.push_back(RVO::Vector2(-1.0f,   -1.5f));
        obstacle1.push_back(RVO::Vector2(1.0f, -1.5f));
        obstacle1.push_back(RVO::Vector2(1.0f, 0.5f));
        
        obstacle2.push_back(RVO::Vector2(3.0f,   3.5f));
        obstacle2.push_back(RVO::Vector2(3.0f,   1.5f));
        obstacle2.push_back(RVO::Vector2(5.0f, 1.5f));
        obstacle2.push_back(RVO::Vector2(5.0f, 3.5f));
        
        obstacle3.push_back(RVO::Vector2(-4.5f,   -2.0f));
        obstacle3.push_back(RVO::Vector2(-4.5f,   -4.0f));
        obstacle3.push_back(RVO::Vector2(-2.5f, -4.0f));
        obstacle3.push_back(RVO::Vector2(-2.5f, -2.0f));
        
         obstacle4.push_back(RVO::Vector2(-1.5f,   6.5f));
        obstacle4.push_back(RVO::Vector2(-1.5f,   4.5f));
        obstacle4.push_back(RVO::Vector2(0.5f, 4.5f));
        obstacle4.push_back(RVO::Vector2(0.5f, 6.5f));
        
  
                          
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        sim->processObstacles();
        
	}
	
	
	
		if(scenario==9)
	{ 
		
		
		obstacle1.push_back(RVO::Vector2(-10.0f,   350.0f));
		obstacle1.push_back(RVO::Vector2(-11.0f,   350.0f));
		obstacle1.push_back(RVO::Vector2(-11.0f, 0.7f));
		obstacle1.push_back(RVO::Vector2(-10.0f, 0.7f));
					
		obstacle2.push_back(RVO::Vector2(-10.0f,   -0.7f));
		obstacle2.push_back(RVO::Vector2(-11.0f, -0.7f));
		obstacle2.push_back(RVO::Vector2(-11.0f, -150.0f));
		obstacle2.push_back(RVO::Vector2(-10.0f, -150.0f));
							
		sim->addObstacle(obstacle1);
		sim->addObstacle(obstacle2);
				
		sim->processObstacles();
	}
	//Adding agents for each scenario
	
	 if(scenario==2) //Circle Scenario
	{
			
		for (int i = 0; i < Agents; ++i) 
			{
			sim->addAgent(float(Agents) *
						  RVO::Vector2(std::cos(i * 2.0f * M_PI / float(Agents))+(std::rand() * 0.01f /(float)RAND_MAX) , std::sin(i * 2.0f * M_PI / float(Agents))+(std::rand() * 0.01f /(float)RAND_MAX) ));
			
			goals.push_back(-sim->getAgentPosition(i));
			    sim->setAgentGoal(i, -sim->getAgentPosition(i));
			}		
	}
	
	if(scenario==3)//Bidirectional Flow
	{
		
		
			sim->addAgent(RVO::Vector2(-20.0f,  30.0f));
			sim->addAgent(RVO::Vector2(-17.0f,  30.0f));
			sim->addAgent(RVO::Vector2(-14.0f,  30.0f));
			sim->addAgent(RVO::Vector2(-20.0f,  29.0f));
			sim->addAgent(RVO::Vector2(-17.0f,  29.0f));
			
			sim->addAgent(RVO::Vector2(-14.0f,  29.0f));
			sim->addAgent(RVO::Vector2(-20.0f,  28.0f));
			sim->addAgent(RVO::Vector2(-17.0f,  28.0f));
			sim->addAgent(RVO::Vector2(-14.0f,  28.0f));
			sim->addAgent(RVO::Vector2(20.0f,  30.0f));
			
			sim->addAgent(RVO::Vector2(17.0f,  30.0f));
			sim->addAgent(RVO::Vector2(14.0f,  30.0f));
			sim->addAgent(RVO::Vector2(20.0f,  29.0f));
			sim->addAgent(RVO::Vector2(17.0f,  29.0f));
			sim->addAgent(RVO::Vector2(14.0f,  29.0f));
			
			sim->addAgent(RVO::Vector2(20.0f,  28.0f));
			sim->addAgent(RVO::Vector2(17.0f,  28.0f));
			sim->addAgent(RVO::Vector2(14.0f,  28.0f));
			
		
			
		
		
		
		for (int i = 0; i < Agents; ++i) 
		{			
			goals.push_back(RVO::Vector2(-sim->getAgentPosition(i).x(),sim->getAgentPosition(i).y() ) );
			sim->setAgentGoal(i, RVO::Vector2(-sim->getAgentPosition(i).x(),sim->getAgentPosition(i).y() ));
					
		}
	}
	
	 if(scenario==4)
    {
        
        
        sim->addAgent(RVO::Vector2(-30.0f,  0.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  0.5f));
        sim->addAgent(RVO::Vector2(-22.0f,   0.5f));
        sim->addAgent(RVO::Vector2(-18.0f,   0.5f));
        sim->addAgent(RVO::Vector2(-14.0f,   0.5f));
        
        
        sim->addAgent(RVO::Vector2(-30.0f, -0.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(-22.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(-18.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(-14.0f,  -0.5f));
        
          sim->addAgent(RVO::Vector2(-30.0f, -1.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-22.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-18.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-14.0f,  -1.5f));
        
         sim->addAgent(RVO::Vector2(-30.0f,  1.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  1.5f));
        sim->addAgent(RVO::Vector2(-22.0f,   1.5f));
        sim->addAgent(RVO::Vector2(-18.0f,   1.5f));
        sim->addAgent(RVO::Vector2(-14.0f,   1.5f));
      
        sim->addAgent(RVO::Vector2(30.0f,  0.5f));
        sim->addAgent(RVO::Vector2(26.0f,  0.5f));
        sim->addAgent(RVO::Vector2(22.0f,  0.5f));
        sim->addAgent(RVO::Vector2(18.0f,  0.5f));
        sim->addAgent(RVO::Vector2(14.0f,  0.5f));
        
        
        sim->addAgent(RVO::Vector2(30.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(26.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(22.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(18.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(14.0f,  -0.5f));
        
         sim->addAgent(RVO::Vector2(30.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(26.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(22.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(18.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(14.0f,  -1.5f));
        
        sim->addAgent(RVO::Vector2(30.0f,  1.5f));
        sim->addAgent(RVO::Vector2(26.0f,  1.5f));
        sim->addAgent(RVO::Vector2(22.0f,  1.5f));
        sim->addAgent(RVO::Vector2(18.0f,  1.5f));
        sim->addAgent(RVO::Vector2(14.0f,  1.5f));
       
        sim->addAgent(RVO::Vector2(0.5f,  30.0f));
        sim->addAgent(RVO::Vector2(0.5f,  26.0f));
        sim->addAgent(RVO::Vector2(0.5f,   22.0f));
        sim->addAgent(RVO::Vector2(0.5f,   18.0f));
        sim->addAgent(RVO::Vector2(0.5f,   14.0f));
        
        
       sim->addAgent(RVO::Vector2(-0.5f, 30.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  26.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  22.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  18.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  14.0f));
        
              sim->addAgent(RVO::Vector2(-1.5f, 30.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  26.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  22.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  18.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  14.0f));
       
        sim->addAgent(RVO::Vector2(1.5f,  30.0f));
        sim->addAgent(RVO::Vector2(1.5f,  26.0f));
        sim->addAgent(RVO::Vector2(1.5f,   22.0f));
        sim->addAgent(RVO::Vector2(1.5f,   18.0f));
        sim->addAgent(RVO::Vector2(1.5f,   14.0f));
      
        
        sim->addAgent(RVO::Vector2(0.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -14.0f));
        
        
        sim->addAgent(RVO::Vector2(-0.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -14.0f));
        
        
          sim->addAgent(RVO::Vector2(-1.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -14.0f));
        
        sim->addAgent(RVO::Vector2(1.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -14.0f));
 
        
        for (int i = 0; i < Agents; ++i)
        {
            initPos[i]=sim->getAgentPosition(i);
            
            if(i<(Agents/4)) //10
            {
                goals.push_back(RVO::Vector2(10, sim->getAgentPosition(i).y() ));
                sim->setAgentGoal(i, RVO::Vector2(10, sim->getAgentPosition(i).y() ));
                
            }
            
            if((i>=(Agents/4))&&(i<(Agents/2))) 
            {                
                  goals.push_back(RVO::Vector2(-10, sim->getAgentPosition(i).y() ));
                   sim->setAgentGoal(i, RVO::Vector2(-10, sim->getAgentPosition(i).y() ));
                
            }
            
            if((i>=(Agents/2))&&(i<(Agents/((float)4/(float)3) ))) 
            {
                goals.push_back(RVO::Vector2(sim->getAgentPosition(i).x() ,-10));
                 sim->setAgentGoal(i, RVO::Vector2(sim->getAgentPosition(i).x() ,-10));
            }
            
            
            if((i>=(Agents/((float)4/(float)3) ))&&(i<Agents)) 
            {
                goals.push_back(RVO::Vector2(sim->getAgentPosition(i).x() ,10));
                 sim->setAgentGoal(i, RVO::Vector2(sim->getAgentPosition(i).x() ,10));
            }
            
            
            
            
        }
        
    }
    
    
    if(scenario==10)
    {
		goals.clear();
		sim->addAgent( RVO::Vector2(3.0f,  0.5f));
        sim->addAgent( RVO::Vector2(27.0f,  0.5f));
        
        initPos[0]= RVO::Vector2(3.0f,  0.5f);
        initPos[1]= RVO::Vector2(27.0f,  0.5f);
        
        
        goals.push_back(RVO::Vector2(27.0f, 0.5f ) );
        sim->setAgentGoal(0,RVO::Vector2(27.0f, 0.5f ) );
        loop[0]=0;
        goals.push_back(RVO::Vector2(3.0f, 0.5f ) );
        sim->setAgentGoal(1,RVO::Vector2(3.0f, 0.5f ) );
        loop[1]=0;
        
        
        
        sim->addAgent( RVO::Vector2(3.0f,  3.5f));
        sim->addAgent(  RVO::Vector2(27.0f,  3.5f));
        
        initPos[2]= RVO::Vector2(3.0f,  3.5f);
        initPos[3]= RVO::Vector2(27.0f,  3.5f);
        
        goals.push_back(RVO::Vector2(27.0f, 3.5f ) );
        sim->setAgentGoal(2,RVO::Vector2(27.0f, 3.5f ) );
        loop[2]=0;
        goals.push_back(RVO::Vector2(3.0f, 3.5f ) );
        sim->setAgentGoal(3,RVO::Vector2(3.0f, 3.5f ) );
        loop[3]=0;
        
        
        
        sim->addAgent( RVO::Vector2(3.0f,  6.5f));
        sim->addAgent(  RVO::Vector2(27.0f,  6.5f));
        
        initPos[4]= RVO::Vector2(3.0f,  6.5f);
        initPos[5]= RVO::Vector2(27.0f,  6.5f);
        
        goals.push_back(RVO::Vector2(27.0f, 6.5f ) );
        sim->setAgentGoal(4,RVO::Vector2(27.0f, 6.5f ) );
        loop[4]=0;        
        goals.push_back(RVO::Vector2(3.0f, 6.5f ) );
        sim->setAgentGoal(5,RVO::Vector2(3.0f, 6.5f ) );
        loop[5]=0;
        
        
        
        
        sim->addAgent( RVO::Vector2(3.0f,  9.5f));
        sim->addAgent(  RVO::Vector2(27.0f,  9.5f));
        
        initPos[6]= RVO::Vector2(3.0f,  9.5f);
        initPos[7]= RVO::Vector2(27.0f,  9.5f);
        
         goals.push_back(RVO::Vector2(27.0f, 9.5f ) );
        sim->setAgentGoal(6,RVO::Vector2(27.0f, 9.5f ) );
        loop[6]=0; 
        goals.push_back(RVO::Vector2(3.0f, 9.5f ) );
        sim->setAgentGoal(7,RVO::Vector2(3.0f, 9.5f ) );
        loop[7]=0;      
                
        
        
		
		
	}
		
 	
	if(scenario==5)
    {
		
				 
			
		genScenarioCong();
			
        for (int i = 0; i < Agents; ++i)
        {
            goals.push_back(RVO::Vector2(-10.0,0.0));
             sim->setAgentGoal(i,RVO::Vector2(-10.0,0.0));
            
        }
        
    }
    
      if(scenario==6)
    {
        
        sim->addAgent( RVO::Vector2(-500.0f,  -10.0f));
        sim->addAgent(  RVO::Vector2(-400.0f,  -10.0f));
        sim->addAgent( RVO::Vector2(-300.0f,  -10.0f));
        sim->addAgent( RVO::Vector2(-200.0f,  -10.0f));
        sim->addAgent( RVO::Vector2(-100.0f,  -10.0f));
        
        sim->addAgent( RVO::Vector2(-500.0f,  -9.0f));
        sim->addAgent(  RVO::Vector2(-4.0f,  -9.0f));
        sim->addAgent( RVO::Vector2(-3.0f,  -9.0f));
        sim->addAgent(  RVO::Vector2(-2.0f,  -9.0f));
        sim->addAgent(  RVO::Vector2(-1.0f,  -9.0f));
        
        sim->addAgent( RVO::Vector2(-5.0f,  -8.0f));
        sim->addAgent(  RVO::Vector2(-4.0f,  -8.0f));
        sim->addAgent(  RVO::Vector2(-3.0f,  -8.0f));
        sim->addAgent( RVO::Vector2(-2.0f,  -8.0f));
        sim->addAgent(  RVO::Vector2(-1.0f,  -8.0f));
      
        sim->addAgent(  RVO::Vector2(-3.0f,  3.0f));
        
        
        for (int i = 0; i < Agents-1; ++i)
        {
            goals.push_back(RVO::Vector2(sim->getAgentPosition(i).x(),24+sim->getAgentPosition(i).y() ) );
            sim->setAgentGoal(i, RVO::Vector2(sim->getAgentPosition(i).x(),24+sim->getAgentPosition(i).y() ) );
        }
        
        goals.push_back(RVO::Vector2(sim->getAgentPosition(15).x(),-20 ) );
        sim->setAgentGoal(15, RVO::Vector2(sim->getAgentPosition(15).x(),-20 )  );
       
        
    }
    
    
    if(scenario==7)
    {
        
        sim->addAgent(  RVO::Vector2(-21.0f,  -0.0f));
        sim->addAgent( RVO::Vector2(21.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-23.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(23.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-25.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(25.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-27.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(27.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-29.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(29.0f,  0.0f));
        
        goals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        goals[0]=RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) ;
        sim->setAgentGoal(0,RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        goals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        goals[1]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(1,RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        goals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        goals[2]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(2,RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        goals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        goals[3]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(3,RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        goals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        goals[4]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(4,RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        goals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        goals[5]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(5,RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        goals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        goals[6]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(6,RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        goals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        goals[7]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(7,RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        
        goals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        goals[8]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(8,RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        goals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        goals[9]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(9,RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        
    }
    
    
    if(scenario==8)
    {
		
	sim->addAgent(  RVO::Vector2(-10.0f,  0.0f));	
	sim->addAgent(  RVO::Vector2(-15.0f,  5.0f));	
	sim->addAgent(  RVO::Vector2(-12.0f,  -1.0f));	
	sim->addAgent(  RVO::Vector2(-15.0f,  2.5f));	
	sim->addAgent(  RVO::Vector2(-13.0f,  -2.5f));	
	
		sim->addAgent(  RVO::Vector2(-7.0f,  0.0f));	
	sim->addAgent(  RVO::Vector2(-11.0f,  5.0f));	
	sim->addAgent(  RVO::Vector2(-10.0f,  -1.0f));	
	sim->addAgent(  RVO::Vector2(-9.0f,  2.5f));	
	sim->addAgent(  RVO::Vector2(-11.0f,  -2.5f));
	
		
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(0).y() ) );
	sim->setAgentGoal(0, RVO::Vector2(15.0f,sim->getAgentPosition(0).y() ));
	
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(1).y() ) );
	sim->setAgentGoal(1, RVO::Vector2(15.0f,sim->getAgentPosition(1).y() ));
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(2).y() ) );
	sim->setAgentGoal(2, RVO::Vector2(15.0f,sim->getAgentPosition(2).y() ));
	
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(3).y() ) );
	sim->setAgentGoal(3, RVO::Vector2(15.0f,sim->getAgentPosition(3).y() ));
	
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(4).y() ) );
	sim->setAgentGoal(4, RVO::Vector2(15.0f,sim->getAgentPosition(4).y() ));
	
	
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(5).y() ) );
	sim->setAgentGoal(5, RVO::Vector2(15.0f,sim->getAgentPosition(5).y() ));
	
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(6).y() ) );
	sim->setAgentGoal(6, RVO::Vector2(15.0f,sim->getAgentPosition(6).y() ));
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(7).y() ) );
	sim->setAgentGoal(7, RVO::Vector2(15.0f,sim->getAgentPosition(7).y() ));
	
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(8).y() ) );
	sim->setAgentGoal(8, RVO::Vector2(15.0f,sim->getAgentPosition(8).y() ));
	
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(9).y() ) );
	sim->setAgentGoal(9, RVO::Vector2(15.0f,sim->getAgentPosition(9).y() ));
	
	
	
	
	}
	
	
	if(scenario==9)
    {
		
		sim->addAgent( RVO::Vector2(-9.5f,  1.5f));
        sim->addAgent( RVO::Vector2(-9.5f,  2.5f));
        sim->addAgent( RVO::Vector2(-9.5f,  3.5f));
        sim->addAgent( RVO::Vector2(-9.5f,  4.5f));
       
        for (int i = 0; i < Agents; ++i)
        {
            goals.push_back(RVO::Vector2(-10.0,0.0));
             sim->setAgentGoal(i,RVO::Vector2(-10.0,0.0));
            
        }
        
    }
	
	 

	for (int i = 0; i < Agents; ++i) 
	{		
		setActions(i);
		sim->setAgentinGoal(i,0);	
	}
	
	
	setPreferredVelocities();



   float orcaavgtime=0;
    for (int i = 0; i < Agents; ++i)
    {   loop[i]=0;
		
		orcaavgtime=orcaavgtime-0.3 + RVO::abs(goals[i]-sim->getAgentPosition(i))/(float)1.5;
		
		sim->setAgentVelocity(i,sim->getAgentPrefVelocity(i));
		ori[i] = atan2(sim->getAgentVelocity(i).y(),sim->getAgentVelocity(i).x());
	}
	
	orcaavgtime=orcaavgtime/(float)Agents;
	
	float tempstandard=0;
	for (int i = 0; i < Agents; ++i)
    {
		energy[i]=0;
		prefVelProg[i]=0;
		energyPerProg[i]=0;
		tempstandard=tempstandard+ ((  (RVO::abs(goals[i]-sim->getAgentPosition(i))/(float)1.5)  -orcaavgtime-0.3)*(  (RVO::abs(goals[i]-sim->getAgentPosition(i))/(float)1.5)   -orcaavgtime-0.3));
	 

    }
    
    if(Agents>1)
    {
      tempstandard=tempstandard/(float)(Agents-1);
			}
			else
			{
			 tempstandard=0;
			
			}
            tempstandard=sqrt(tempstandard);
				
            orcaavgtime=orcaavgtime+3*tempstandard;
            baseScore=orcaavgtime;
}

//******** Check if agents have reached their destination
bool reachedGoal()
{
	/* Check if all agents have reached their goals. */
	int allingoal=1;
	totalnotingoal=0;

        
        
        
		
		for (size_t i = 0; i < sim->getNumAgents(); ++i) 
		{
			
			float distToGoal = RVO::abs(sim->getAgentPosition(i)-goals[i]);
			
			if(scenario==10)
			{
				
				if((distToGoal > 0.5)&&(isinGoal[i]==0))
				{
					isinGoal[i]=0;
					allingoal=0;
					totalnotingoal++;
					
				}
				
				if(distToGoal <= 0.5)
				{
					if(loop[i]==0)
					{	
							goals[i]=initPos[i];		
							loop[i]=1;		
					
					}
					else
					{
							if(!isinGoal[i])
							{
					
								totalingoal=totalingoal+1;
							}
				
							isinGoal[i]=1;
							sim->setAgentinGoal(i,1);
				
							if(TimetoGoal[i]==0)
							{
								TimetoGoal[i]=sim->getGlobalTime();
							}
						
						
					}
				}
			
			
			
				
			}
			else
			{
			
			
				if (((distToGoal > 0.5)&&(isinGoal[i]==0)&&((scenario!=5)&&(scenario!=9)))||(((scenario==5)||(scenario==9))&&(sim->getAgentPosition(i).x() >-10.0f) ))    
				{
					//Agent is consireded to reach its goal if it is 0.5 or less meters from it
					isinGoal[i]=0;
					allingoal=0;
					totalnotingoal++;
			
				
				}
				else 
				{
					if(!isinGoal[i])
					{
					
						totalingoal=totalingoal+1;
					}
				
					isinGoal[i]=1;
					sim->setAgentinGoal(i,1);
				
					if(TimetoGoal[i]==0)
					{
						TimetoGoal[i]=sim->getGlobalTime();
					}
				
				}
			}
			
		}
		
		if (allingoal==0)
		{
			
			return false;
		}
		
		return true;
		
}



//**** Update the agents in the simulation ****** //
void updateVisualization() //Displays the agents (if display is enabled), and calls the methods for the algorithm chosen (ORCA or C-NAV). It is called each timestep
{
	
	std::ofstream posangleagg;
	
	
	posangleagg.open("Traject.txt", std::ios_base::app);
	
	timestep++;
	
	if(visualizer)
	{
	glLineWidth(2);
	
	}
	
	if (lastFrameTime == 0)
	{
		if(visualizer)
		{
		lastFrameTime = glutGet(GLUT_ELAPSED_TIME);
		}
			
	}
	
	
	if(visualizer)
	{
		lastFrameTime = timestep;
	}
	else
	{
			lastFrameTime = timestep;
	}
	
	
	if((!finalize)) //To prevent iterations from occuring when the iteration is setting up
	{
		if ((!reachedGoal())&&(timestep<threshold)) // Only enters if there are agents that have not reach the goal
		{
			
			
			if(visualizer)
			{
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			}
			float nowtime=sim->getGlobalTime();
			for (size_t i = 0; i < sim->getNumAgents(); ++i) 
			{
			//	srand(time(NULL));
				needtoUpdate[i]=0;
				
				
				if(timestep>1)
				{
					
					if(((std::rand()/(float)RAND_MAX)<=updateProbability)||(timestep==2)) //If true, a new motion decision will be made, otherwise the agent keeps its previous action
					{ 
						needtoUpdate[i]=1;
				
					}
					
				}
				
				
				 
				if((timestep-1)%2==0) //This indicates how often the output to Pos_ori is produced
				{
					
					float alphaO = .9f;
					
					if (RVO::abs(sim->getAgentVelocity(i)) < .02)
					{
						alphaO = 1;
					}
					
					RVO::Vector2 normv ;
					
					normv = sim->getAgentVelocity(i);
					
					
					normalize(normv);
					
					RVO::Vector2 normpref = sim->getAgentPrefVelocity(i);
					
					normalize(normpref);
					
					float score = normv*normpref;
					
					if (score < 0) 
					{
						alphaO = 1;
					}
					else 
					{
						alphaO = .9;
					}
					float ordiff;
					
					
					ordiff = atan2(sim->getAgentVelocity(i).y(),sim->getAgentVelocity(i).x()) - ori[i];
					
					if (ordiff > M_PI)
					{
						ordiff-=2*M_PI;
					}
					
					if (ordiff < -M_PI)
					{
						ordiff+=2*M_PI;
					}
					
					ori[i]+= (1.f-alphaO)*ordiff;
					
					if (ori[i] > M_PI)
					{
						ori[i]-=2*M_PI;
					}
					
					if (ori[i] < -M_PI)
					{
						ori[i]+=2*M_PI;
					}
					
					//Writing the pos and orientation of the agent
					
				    //positionvel[i][0] = x;
					
					//positionvel[i][1] = y;

					//Output the agent information to a file PosOri
					// posangleagg << i<< ", " << 0 << ", "<< sim->getAgentPosition(i).x()  << ", " << sim->getAgentPosition(i).y() << ", " << cos(ori[i]) << ", " << sin(ori[i]) << ", 0.5, "<< nowtime << /*" vel " << sim->getAgentVelocity(i).y() <<*/"\n";
				
				}
				
				
			
				/***************Drawing the agent:*/
				
				if(visualizer)
				{
				
					glPushMatrix();
					glTranslatef(sim->getAgentPosition(i).x(), sim->getAgentPosition(i).y(), 0.0f);
					
					int num = i;
					char buffer[10]={'\0'};
					sprintf(buffer, "%d", num); //%d is for integers 
					
					
					renderBitmapString(0.3, 0.3, GLUT_BITMAP_TIMES_ROMAN_24, buffer);
					
					glColor3f(0.4f,0.9f,0.0f); //greenish color for the agents
					
					if(i==agentViewed)
                    {                        
                        glColor3f(0.9f,0.9f,0.9f); //the agent to view is almost white
                    }
                    
                    
					glutSolidSphere(0.5f,8,8);
					glDisable( GL_LIGHTING );
					glColor3f(0,0,0);		
		

					glBegin(GL_LINES);
					glVertex3f(0.f,0.f,1.0f);
					RVO::Vector2 pfrev=  sim->getAgentPrefVelocity(i);
					glVertex3f(pfrev.x(),pfrev.y(),1.0f);
					glEnd();
					
		
					
					glColor3f(0,0,0);		
		
						if((indexMostSimilar[i]>=0)&&(i==agentViewed))
						{
							glBegin(GL_LINES);
							glVertex3f(0.f,0.f,1.0f);
							RVO::Vector2 simi=  RVO::Vector2(sim->getAgentPosition(bestSimilarNeigh[i])-sim->getAgentPosition(i));
							glVertex3f(simi.x(),simi.y(),1.0f);
							glEnd();
						}
						glPopMatrix();		
		
		
				}
	
	
			} //End of the iteration of all agents
			
			
			
			if(algo==1) //ORCA
			{
		
				setPreferredVelocities();

			}
			
			
			if(algo==2) //C-Nav
			{
								
				if(timestep>1)
				{

					simulateVelocities(); 
					 
				
					
				}
                setPreferredVelocities();
		
			}
			
	
		
			sim->doStep(); //calls a method in RVOSimulator.cpp to update the simulation 
			
			countCollisions();
			
			if(needtoUpdate[agentViewed])
			{
		    std::cout << " Real Vel: " << sim->getAgentVelocity(agentViewed) << " magnitude " << RVO::abs(sim->getAgentVelocity(agentViewed))<< " with prefVel " << sim->getAgentPrefVelocity(agentViewed) << "\n";	
			
			
			
			
			std::cout << "Compared to Expected Vel: " << sim->getAgentInitialVelocity(agentViewed, chosenAction[agentViewed]) << " magnitude " 
			<< RVO::abs(sim->getAgentInitialVelocity(agentViewed, chosenAction[agentViewed]))<< "\n";
			std::cout << "Compared to Expected PrefVel: " << sim->getAgentInitialPrefVelocity(agentViewed, chosenAction[agentViewed]) << " magnitude " << RVO::abs(sim->getAgentInitialPrefVelocity(agentViewed, chosenAction[agentViewed]))<< "\n";
			std::cout << " DIFFERENCE: "<< RVO::abs(sim->getAgentInitialVelocity(agentViewed, chosenAction[agentViewed])- sim->getAgentVelocity(agentViewed)) <<"\n";
			getchar();	
				
			}
			int closest=getClosestToGoal();
			
			totalMotion+= RVO::abs(sim->getAgentPrefVelocity(closest)-sim->getAgentVelocity(closest));
			totaldeviation++;
			
			for (size_t i = 0; i < sim->getNumAgents(); ++i) 
			{
				
				if(!isinGoal[i])
				{
				    if(i==1011)
					{
					std::cout << "Agent " <<i << " Real vel is " << sim->getAgentVelocity(i)<< "\n";
					}
					
					
					prefVelProg[i]=  sim->getAgentVelocity(i) *RVO::normalize(goals[i]-sim->getAgentPosition(i));
					energy[i] = 2.25 + 1* (RVO::abs(sim->getAgentVelocity(i)))*(RVO::abs(sim->getAgentVelocity(i)));  
					
					energyPerProg[i] = energyPerProg[i] + energy[i];
										
				
				}
				
				if((!isinGoal[i])&&(needtoUpdate[i]))
				{
					if(RVO::abs(sim->getAgentInitialVelocity(i, chosenAction[i])- sim->getAgentVelocity(i))>0.5)
					{
						if(i==agentViewed)
						{
						std::cout << "(" << i << ") Cant trust prediction, switching to execution-based reward at t " << timestep <<"\n";	
						
						}
						
						
					}
				
				}
				
				
			}
		

			if(visualizer)
			{
                    glPushMatrix();
					glTranslatef(0, 0, 0.0f);
					glColor3f(0.0f, 0.0f, 0.0f);
					
					
					 
                if(scenario==1)
                {
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-1.5f,   -0.5f);
                    glVertex2f(-1.5f,   -1.5f);
                    glVertex2f(sidesize+1.5f, -1.5f);
                    glVertex2f(sidesize+1.5f, -0.5f);
                    glEnd();
                    
                    
                    glBegin(GL_QUADS);
                    glVertex2f(sidesize+0.5f,   sidesize+1.5f);
                    glVertex2f(sidesize+0.5f,   -1.5f);
                    glVertex2f(sidesize+1.5f, -1.5f);
                    glVertex2f(sidesize+1.5f, sidesize+1.5f);
                    glEnd();
                    
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-1.5f,   sidesize+1.5f);
                    glVertex2f(-1.5f,   sidesize+0.5f);
                    glVertex2f(sidesize+1.5f,  sidesize+0.5f);
                    glVertex2f(sidesize+1.5f,  sidesize+1.5f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-1.5f,   sidesize+1.5f);
                    glVertex2f(-1.5f,   -1.5f);
                    glVertex2f(-0.5f, -1.5f);
                    glVertex2f(-0.5f, sidesize+1.5f);
                    glEnd();
                    
                   
                    
                }
                
                
                if(scenario==3)
                {
                    glBegin(GL_QUADS);
                    glVertex2f(200.0f,   31.0f);
                    glVertex2f(-200.0f,   31.0f);
                    glVertex2f(-200.0f, 30.5f);
                    glVertex2f(200.0f, 30.5f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(200.0f,   27.5f);
                    glVertex2f(-200.0f,   27.5f);
                    glVertex2f(-200.0f, 27.0f);
                    glVertex2f(200.0f, 27.0f);
                    glEnd();
                    
                    
                }
                
                
                
                if(scenario==10)
                {
					 
					glBegin(GL_QUADS);
                    glVertex2f(0.0f,   12.0f);
                    glVertex2f(-2.0f,   12.0f);
                    glVertex2f(-2.0f, -2.0f);
                    glVertex2f(0.0f, -2.0f);
                    glEnd();
					
					glBegin(GL_QUADS);
                    glVertex2f(30.0f,   0.0f);
                    glVertex2f(0.0f,   0.0f);
                    glVertex2f(0.0f, -2.0f);
                    glVertex2f(30.0f, -2.0f);
                    glEnd();
					
					
					glBegin(GL_QUADS);
                    glVertex2f(30.0f,   12.0f);
                    glVertex2f(0.0f,   12.0f);
                    glVertex2f(0.0f, 10.0f);
                    glVertex2f(30.0f, 10.0f);
                    glEnd();
						
					glBegin(GL_QUADS);
                    glVertex2f(32.0f,   12.0f);
                    glVertex2f(30.0f,   12.0f);
                    glVertex2f(30.0f, -2.0f);
                    glVertex2f(32.0f, -2.0f);
                    glEnd();
					

				    glBegin(GL_QUADS);
                    glVertex2f(11.0f,   3.0f);
                    glVertex2f(5.0f,   3.0f);
                    glVertex2f(5.0f, 1.0f);
                    glVertex2f(11.0f, 1.0f);
                    glEnd();
			
			
					glBegin(GL_QUADS);
                    glVertex2f(11.0f,   6.0f);
                    glVertex2f(5.0f,   6.0f);
                    glVertex2f(5.0f, 4.0f);
                    glVertex2f(11.0f, 4.0f);
                    glEnd();
			
					glBegin(GL_QUADS);
                    glVertex2f(11.0f,   9.0f);
                    glVertex2f(5.0f,   9.0f);
                    glVertex2f(5.0f, 7.0f);
                    glVertex2f(11.0f, 7.0f);
                    glEnd();
						
					glBegin(GL_QUADS);
                    glVertex2f(18.0f,   9.0f);
                    glVertex2f(12.0f,   9.0f);
                    glVertex2f(12.0f, 7.0f);
                    glVertex2f(18.0f, 7.0f);
                    glEnd();
			
					glBegin(GL_QUADS);
                    glVertex2f(18.0f,   3.0f);
                    glVertex2f(12.0f,   3.0f);
                    glVertex2f(12.0f, 1.0f);
                    glVertex2f(18.0f, 1.0f);
                    glEnd();
			
					glBegin(GL_QUADS);
                    glVertex2f(18.0f,   6.0f);
                    glVertex2f(12.0f,   6.0f);
                    glVertex2f(12.0f, 4.0f);
                    glVertex2f(18.0f, 4.0f);
                    glEnd();
                    
                    
                    glBegin(GL_QUADS);
                    glVertex2f(25.0f,   9.0f);
                    glVertex2f(19.0f,   9.0f);
                    glVertex2f(19.0f, 7.0f);
                    glVertex2f(25.0f, 7.0f);
                    glEnd();
			
					glBegin(GL_QUADS);
                    glVertex2f(25.0f,   3.0f);
                    glVertex2f(19.0f,   3.0f);
                    glVertex2f(19.0f, 1.0f);
                    glVertex2f(25.0f, 1.0f);
                    glEnd();
			
					glBegin(GL_QUADS);
                    glVertex2f(25.0f,   6.0f);
                    glVertex2f(19.0f,   6.0f);
                    glVertex2f(19.0f, 4.0f);
                    glVertex2f(25.0f, 4.0f);
                    glEnd();
		
					
					
					
					
					}
                
                
                if(scenario==4)
                {
               
                    
                    glBegin(GL_QUADS);
                    glVertex2f(3.0f,200.0f);
                    glVertex2f(2.0f, 200.0f);
                    glVertex2f(2.0f, 3.0f);
                    glVertex2f(3.0f, 3.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-2.0f,   200.0f);
                    glVertex2f(-3.0f, 200.0f);
                    glVertex2f(-3.0f, 3.0f);
                    glVertex2f(-2.0f, 3.0f);
                    glEnd();
                      


                    glBegin(GL_QUADS);
                    glVertex2f(3.0f,   -3.0f);
                    glVertex2f(2.0f,   -3.0f);
                    glVertex2f(2.0f, -200.0f);
                    glVertex2f(3.0f, -200.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-2.0f, -3.0f);
                    glVertex2f(-3.0f,   -3.0f);
                    glVertex2f(-3.0f, -200.0f);
                    glVertex2f(-2.0f, -200.0f);
                    glEnd();

              
                    glBegin(GL_QUADS);
                    glVertex2f(200.0f,   3.0f);
                    glVertex2f(2.0f,   3.0f);
                    glVertex2f(2.0f, 2.0f);
                    glVertex2f(200.0f, 2.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(200.0f,   -2.0f);
                    glVertex2f(2.0f, -2.0f);
                    glVertex2f(2.0f, -3.0f);
                    glVertex2f(200.0f, -3.0f);
                    glEnd();
                      


                    glBegin(GL_QUADS);
                    glVertex2f(-2.0f,   3.0f);
                    glVertex2f(-200.0f,   3.0f);
                    glVertex2f(-200.0f, 2.0f);
                    glVertex2f(-2.0f, 2.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-2.0f, -2.0f);
                    glVertex2f(-200.0f,   -2.0f);
                    glVertex2f(-200.0f, -3.0f);
                    glVertex2f(-2.0f, -3.0f);
                    glEnd();

                }
                
                 if(scenario==9)
                {
                    glBegin(GL_QUADS);
                    glVertex2f(-10.0f,   350.0f);
                    glVertex2f(-11.0f,   350.0f);
                    glVertex2f(-11.0f, 0.6f);
                    glVertex2f(-10.0f, 0.6f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-10.0f,   -0.6f);
                    glVertex2f(-11.0f,   -0.6f);
                    glVertex2f(-11.0f, -150.0f);
                    glVertex2f(-10.0f, -150.0f);
                    glEnd();
                }
                
                
                
                if(scenario==5)
                {
                    glBegin(GL_QUADS);
                    glVertex2f(-10.0f,   350.0f);
                    glVertex2f(-11.0f,   350.0f);
                    glVertex2f(-11.0f, 0.6f);
                    glVertex2f(-10.0f, 0.6f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-10.0f,   -0.6f);
                    glVertex2f(-11.0f,   -0.6f);
                    glVertex2f(-11.0f, -150.0f);
                    glVertex2f(-10.0f, -150.0f);
                    glEnd();
                }
                
                
                if(scenario==7)
                {
                    
                    glPushMatrix();
                    glTranslatef(0, 0, 0.0f);
                    glColor3f(0.0f, 0.0f, 0.0f);
                                                          
                    glBegin(GL_QUADS);
                    glVertex2f(  -10.0f,   2.0f);
                    glVertex2f(-10.0f,   0.6f);
                    glVertex2f(10.0f, 0.6f);
                    glVertex2f(  10.0f, 2.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(  -10.0f,   -0.6f);
                    glVertex2f(-10.0f,   -2.0f);
                    glVertex2f(10.0f, -2.0f);
                    glVertex2f(  10.0f, -0.6f);
                    glEnd();
                    
                    
                }
                
                
                            
                
                if(scenario==8)
                {	
					
		
        
					glPushMatrix();
                    glTranslatef(0, 0, 0.0f);
                    glColor3f(0.0f, 0.0f, 0.0f);
        
					glBegin(GL_QUADS);
                    glVertex2f(-1.0f,   0.5f);
                    glVertex2f(-1.0f,   -1.5f);
                    glVertex2f(1.0f, -1.5f);
                    glVertex2f(1.0f, 0.5f);
                    glEnd();
					
					
					glBegin(GL_QUADS);
                    glVertex2f(3.0f,   3.5f);
                    glVertex2f(3.0f,   1.5f);
                    glVertex2f(5.0f, 1.5f);
                    glVertex2f(5.0f, 3.5f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-4.5f,   -2.0f);
                    glVertex2f(-4.5f,   -4.0f);
                    glVertex2f(-2.5f, -4.0f);
                    glVertex2f(-2.5f, -2.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-1.5f,   6.5f);
                    glVertex2f(-1.5f,   4.5f);
                    glVertex2f(0.5f, 4.5f);
                    glVertex2f(0.5f, 6.5f);
                    glEnd();
					
					
        
        
					
				}
				
					glPopMatrix();
		
        	}
	
	
	
	
		}
		else //If all agents have reached their goals
		{
			
		//    posangleagg << "***********************************************************************************\n"; //Separete between agents pos and vel in different iterations
		      finalize=1;
		      
		      if(timestep==threshold)
		      {   
				  notFinished++;
				  finalTime=0;
				 
			  }
			  else
			  {
          
            
				float totalTimeToGoal=0;
            
				for(int i=0;i<Agents;i++)
				{ 
            
				avgEnergy[i]=0;
				}
		
				totalAvgEnergy=0;
				for(int i=0;i<Agents;i++)
				{ 
					if(isinGoal[i])
					{
					totalTimeToGoal+=TimetoGoal[i];  //TimetoGoal[i] is the time agent i reached its goal
					avgEnergy[i] = (float)energyPerProg[i];
					totalAvgEnergy= totalAvgEnergy+ avgEnergy[i];
					}
					else
					{
					TimetoGoal[i]= sim->getGlobalTime();
					totalTimeToGoal+=	TimetoGoal[i];
					}
				}
				totalrewardsimulation=totalTimeToGoal/(float)Agents;
            
				float AvgStd=0, tempstd=0;
				AvgStd=AvgStd+ totalrewardsimulation;
            
				for(int i=0;i<Agents;i++)
				{
					tempstd=tempstd+ ((TimetoGoal[i]-totalrewardsimulation)*(TimetoGoal[i]-totalrewardsimulation));
						
				}
            
				if(Agents>1)
				{
				tempstd=tempstd/(float)(Agents-1);
				}
				else
				{
				tempstd=0;
			
				}
				tempstd=sqrt(tempstd);
				AvgStd=AvgStd+3*tempstd;
				finalTime=AvgStd;
				SimScore=SimScore+finalTime;
            
				totalAvgEnergy=(float)totalAvgEnergy/(float)Agents;
       
			  }
		}
	
	} 
	
	
	posangleagg.close();
	if(visualizer)
	{
		glutSwapBuffers();
		if(finalize)
		{
			glutLeaveMainLoop();
		
		}
	}
	
}



//This function sets up the actions and the RVO parameters for each iteration of the experiment
float crowd_simulation_eval(const float* actionsDir, const float* actionsMag, int num_actions)
{
	Actions = num_actions;
	for (int i = 0; i < Actions; i++)
	{
		actionVector[i] = actionsDir[i];
		actionVectorMag[i] = actionsMag[i];
	}
	
	sim = new RVO::RVOSimulator();
	iteration=1;
	setupScenario(sim);
	
	if(visualizer)
	{
		glutMainLoop();
	}
	else
	{
		do {
			updateVisualization();
			}
		while (!finalize); 
	}
	delete sim;
	
	return finalTime;
		
}



int main(int argc, char* argv[])
{
    
    if( argc == 9 )
    {
          visualizer= atoi(argv[3]);
        algo = atoi(argv[1]);
        finalIteration= atoi(argv[4]);
        coord_factor= atof(argv[5]);
        allNeigh=atoi(argv[6]); // all neighbors(1) or just closer to goal (0)
        contadourX=atoi(argv[7]);
         followNeigh=atoi(argv[8]);
        
        agentViewed=999; //agent to be closely checked, 999 means no agent checked
        
        
    }
    else if( argc > 8 )
    {
        std::cout <<"Too many arguments supplied.\n";
        exit(1);
    }
    else
    {
        std::cout << "Indicate the algorithm, scenario, visualization option, number of iterations, coordination factor, type of neighbors considered in the politeness computation, number of neighbors considered in the politeness computation, and whether or not to use a following behavior.\n Algorithm:\n 1.- ORCA\n 2.- C-Nav\nScenarios:\n 1.- Crowd\n 2.- Circle\n 3.- Bidirectional\n 4.- PerpCrossing\n 5.- Congested\n 6.- Crossing\n 7.- Deadlock\n 8.- Blocks\n 9.- Line\nVisualization:\n 0.-Off\n 1.-On\n \n Number of iterations\n\n Coordination Factor\n\n Type of neighbors considered for politeness (1: all; 0: in front)\n\n Number of neighbors considered in the politeness factor\n\n Following behavior (0: disabled; 1: enabled)\n ";
        exit(1);
    }
    
     srand(time(NULL));
    
      
    ////////////////////////////////////////////////////////////////////
    //////Sample actions: 8 velocities whose direction is in radians (velDir) and the magnitudes are in meters per second (velMag)
    
    float velDir[9] = {0.00000, M_PI/4, M_PI/2, M_PI-M_PI/4,M_PI, -3*M_PI/4   , -M_PI/2  ,-M_PI/4, 0.00000};
    float velMag[9]= {1.5,  1.5,1.5, 1.5,1.5,  1.5,1.5, 1.5,1.5};
   
    int accountActions;
    if(followNeigh)
    {
		accountActions=9;
			
		
	}
    else
    {
		 accountActions=8;
		
	}
   
    
    float Result[400], ResultEnergy[400];
    notFinished=0;

    float sumTotalEnergies=0;
    for(int i=0;i<finalIteration; i++) //The code below is executed for each iteration
    {
        if(visualizer)
        {
            glutInit(&argc, argv);
            glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
            glutInitWindowSize(800, 600);
            glutCreateWindow("ORCA");
            glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
            InitGL();
            glutDisplayFunc(updateVisualization);
            glutReshapeFunc(reshape);
            glutIdleFunc(idle);
        }
        
        finalize=0;
        
        
        switch(atoi(argv[2]))
        {
            case 1:Agents=300,scenario=1;
                break;
                
            case 2:Agents=128,scenario=2;
                break;
                
            case 3:Agents=18,scenario=3;
                break;
                
            case 4:Agents=80,scenario=4;
                break;
                
            case 5:Agents=64,scenario=5;
                break;
                
            case 6:Agents=16,scenario=6;
                break;
                
            case 7:Agents=10,scenario=7;
                break;
            case 8:Agents=10,scenario=8;
                break;
            case 9:Agents=4,scenario=9;
                break;
                
            case 10:Agents=8,scenario=10;
                break;
            
        }
        Result[i]=  crowd_simulation_eval(velDir, velMag, accountActions);
        
        ResultEnergy[i]= totalAvgEnergy;
        sumTotalEnergies+=totalAvgEnergy;
                
         
    
	      
     
        
    }
    
  
    float ResultStd=0, ResultTotal=0, ResultEnergyStd=0;
  
    for(int i=0;i<finalIteration;i++)
				{
					
					if(Result[i]>0)
					{
						ResultEnergyStd= ResultEnergyStd + ((ResultEnergy[i]-(sumTotalEnergies/((float)finalIteration-notFinished)) )*(ResultEnergy[i]-(sumTotalEnergies/((float)finalIteration-notFinished) ) ));
					    ResultStd=ResultStd+ ((Result[i]-(SimScore/((float)finalIteration-notFinished)) )*(Result[i]-(SimScore/((float)finalIteration-notFinished) ) ));
                        ResultTotal=ResultTotal+Result[i];
					}
                    
                }
				
				
				ResultEnergyStd= ResultEnergyStd/(float)(finalIteration-notFinished-1);
				ResultEnergyStd=sqrt(ResultEnergyStd);
				ResultEnergyStd=(ResultEnergyStd)/(float)sqrt(finalIteration-notFinished);
				ResultStd=ResultStd/(float)(finalIteration-notFinished-1);
				ResultStd=sqrt(ResultStd);
				ResultStd=(ResultStd)/(float)sqrt(finalIteration-notFinished);
    //Output the average interaction overhead accross all iterations, its std dev, the percentage of goal reachibility, the average of the energy computation and its standard deviation 
   std::cout << "AvgIO: " << (SimScore/((float)finalIteration-notFinished)) -baseScore<< ", StdDevIO: " << ResultStd<< ", " << 100*(float)(finalIteration-notFinished)/(float)finalIteration << "% goal reachibility, AvgEnergy " << (float)sumTotalEnergies/(float)finalIteration << " StdDevEnergy: " <<ResultEnergyStd <<   "\n";
   return 0;
    
}


