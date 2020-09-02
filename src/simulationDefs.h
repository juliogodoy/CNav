#ifndef __SIMULATIONDEFS_H_INCLUDED__   // if x.h hasn't been included yet...
#define __SIMULATIONDEFS_H_INCLUDED__ 

#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"
#include <stdlib.h>
#include <cstdlib>
#include <iostream>

#ifdef _OPENMP
#include <omp.h>
#endif

//**************Default Simulation Parameters*********************//

float updateProbability=0.25; // In average, each agent makes decision every (1/updateProbability) timesteps (directly dep in timestep length) 
float simTimeStep=0.05; // Length of the timestep, in each second there are (1/simTimeStep) timesteps
int sidesize=25; //Size of each side of the environment of the Crowd scenario
bool randomPert=1; //Enables small random perturbations on the preferred velocities computed
bool finalize=0; //  marks the end of the simulation
bool followNeigh;

const int numNeighbors=10;//number of neighbors that each agent considers
const float neighborDistance=15.0f; //max distance that agents can perceive neighbors
const float timeHorizonORCA=5.0f;  // time horizon to determine collisions with other agents
const float timeHorizonObstORCA=1.3f;// time horizon to determine collisions with obstacles
const float radiusORCA=0.5f;  // distance that the agents want to keep from other agents
const float maxSpeedORCA=1.5f; //maximum speed that agents can move with

int agentViewed;
float SimScore=0;

float coord_factor=0.1; // Coordination factor, that balances between the goal progress and the politeness of the agents
float coord_factor_alan=0.4; 
static const int timeHorizon=2; //Number of (future) timesteps to simulate to evaluate the effect of each action
int allNeigh=1; //0 makes agents consider only neighbors closer than itself to its goal. 1 makes agents consider all neighbors around it, with limit numNeighbors
int contadourX=1;
int threshold=20000; //Maximum number of timesteps before the simulation is declared not finished.
float baseScore;
int colisiones=0;
//General variable declaration
RVO::RVOSimulator* sim;
int algo,Agents, scenario, notFinished, Actions,chosenAction[320], bestSimilarNeigh[320], currentVel[320], totalConsideredNeighs[320], indexMostSimilar[320],mostSimilar[320][50], iteration, timestep, lastFrameTime = 0, totalnotingoal,totalingoal,finalIteration;
bool isinGoal[320],visualizer, eval_method[320], loop[320];
float ori[750];
RVO::Vector2 goalvectors[320],goalVector, initPos[320];
std::vector<RVO::Vector2> goals;
float totalMotion=0, angle,dist,actionVector[50],actionVectorMag[50],finalTime=-100,oriMostSimilar[320][50], RewardAvg[320][50],TimetoGoal[320],goaldistance[320],ActionSpeed[320][50],ActionDir[320][50], goalx[320], goaly[320],totalrewardsimulation=0,totalrewarditeration, needtoUpdate[320], LastActionEstimate[320][20], Boltz[320][20],defaultValue[320][20];
int totaldeviation=0;

#endif
