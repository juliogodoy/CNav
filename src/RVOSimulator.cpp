/*
 * RVOSimulator.cpp
 * RVO2 Library
 *
 * Copyright (c) 2008-2010 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the authors <geom@cs.unc.edu> or the Office of
 * Technology Development at the University of North Carolina at Chapel Hill
 * <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"
#include <stdlib.h>
#include <cstdlib>
#include <utility>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <map>
#include <set>
#include <algorithm>
#include <functional>
#include <stdio.h>


#ifdef _OPENMP
#include <omp.h>
#endif

namespace RVO {
	RVOSimulator::RVOSimulator() : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(0.0f)
	{
		kdTree_ = new KdTree(this);
	}
	
	RVOSimulator::RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity) : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(timeStep)
	{
		kdTree_ = new KdTree(this);
		defaultAgent_ = new Agent(this);
		
		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeHorizonObst_ = timeHorizonObst;
		defaultAgent_->velocity_ = velocity;
		
		defaultAgent_->hypoVelocity_ = velocity;
		defaultAgent_->Vvelocity_ = velocity;
	}
	
	RVOSimulator::~RVOSimulator()
	{
		if (defaultAgent_ != NULL) {
			delete defaultAgent_;
		}
		
		for (size_t i = 0; i < agents_.size(); ++i) {
			delete agents_[i];
		}
		
		for (size_t i = 0; i < obstacles_.size(); ++i) {
			delete obstacles_[i];
		}
		
		delete kdTree_;
	}
	
	
	bool RVOSimulator::sortinrev(const std::pair<int,int> &a,  
               const std::pair<int,int> &b) 
{ 
       return (a.first > b.first); 
} 

  float RVOSimulator::SimulateVel(size_t agentNo, RVO::Vector2 goal, int timeHorizon, int actionNum, int numNeighbors, int numAgents, float coord_factor, float coord_factor_alan, float ActionSpeed, float ActionDir, int allNeigh, int contadourX)
	{// std::cout <<"ENTERING SIMULATEVEL..\n";
		float VprefVelocity_, politeness=0,goalProgress=0,reward=0;
		int NeighborList[numNeighbors], neighbors=0,n,r=0,endOfPlan=timeHorizon, foundAt=0;
		Vector2 goalVector,prefVelocityBackup_= agents_[agentNo]->prefVelocity_;
		size_t i = agentNo;
		bool goalFound=0, numNeigh=1; // When the goal is reached, the agent just stays there
		float avgPolitenessCNav=0;
		float avgPolitenessALAN=0;
		float avgGoalProg=0;
		float changeVel=0;
		
		std::vector< std::pair<float, int> > vect;
		float politeness4=0;
		
			
		for (n=0; n<numNeighbors; n++) 
		{
			NeighborList[n]=-1;
		}

		for(n=0;n<numAgents;n++)
		{
			agents_[n]->vOpt_ = agents_[n]->Vvelocity_;
			agents_[n]->Vposition_ = agents_[n]->position_;
			agents_[n]->Vvelocity_ = agents_[n]->velocity_;
			agents_[n]->VprefVelocity_ = agents_[n]->prefVelocity_;
			
		}
		agents_[i]->computeNeighbors();	
		
		
			for (size_t j = 0; j < agents_[i]->agentNeighbors_.size(); ++j) //Computing those neighbors of neighbor k, which the agent 'agentNo' can see
			{    
				  size_t k=  agents_[i]->agentNeighbors_[j].second->id_;
				  agents_[k]->VcomputeNeighborsModel(agents_[agentNo], agents_[agentNo]->timeHorizonObst_,  agents_[agentNo]->maxSpeed_, agents_[agentNo]->radius_ ,agents_[agentNo]->neighborDist_,  agents_[agentNo]->timeHorizonObst_ , agents_[agentNo]->maxSpeed_, agents_[agentNo]->radius_  , 	agents_[agentNo]->maxNeighbors_ ,agents_[agentNo]->neighborDist_);
				
				
			}
		
		if( agents_[i]->agentNeighbors_.size()==0)
		{			
		 numNeigh=0;	
		}
		
		if(i==15)
					{
					//	std::cout << "************\n";
					  //  std::cout << " With action "<< actionNum <<" :\n";
					}
		
        #pragma omp parallel for

		for(int t=0;t<timeHorizon;t++) // For each timestep in the time horizon...
		{
			neighbors=0;
			//std::cout <<"T: " << t<< "..\n";
			kdTree_->VbuildAgentTree();
		    goalVector= RVO::normalize(goal- agents_[i]->Vposition_);
		    vect.clear();
	      
	        if (t<timeHorizon)  //...the agent computes a new preferred velocity
			{
				agents_[i]->prefVelocity_= goalVector*ActionSpeed;
			    agents_[i]->prefVelocity_=Vector2(agents_[i]->prefVelocity_.x()*std::cos(ActionDir)+(agents_[i]->prefVelocity_.y())*std::sin(ActionDir), (agents_[i]->prefVelocity_.y())*std::cos(ActionDir)+(agents_[i]->prefVelocity_.x())*-std::sin(ActionDir));
			
			if(i==0)
			{
			//std::cout << " Action 	" << actionNum <<" Vpref: " <<agents_[i]->prefVelocity_ << " magnitued " << RVO::abs(agents_[i]->prefVelocity_)<< "\n";
			}
					
			
			
			}
					
					
					
			//For each neighbor of the agent, we need to compute its collision-free velocity		
			for (size_t j = 0; j < agents_[i]->agentNeighbors_.size(); ++j) 
			{ 
				size_t k=  agents_[i]->agentNeighbors_[j].second->id_;
					
				int exists=0;
				for (int u=0; u<numNeighbors; u++) 
				{
					if((int)k==NeighborList[u])
					{
						
						exists=1;
					}
				}
				
				if(exists==0)
				{ 
					NeighborList[r]=(int)k;
										
					if(r<(numNeighbors-1))
					{
						r++;
					}
					
				}
             
			
										
				if(t>0) //For all t>0, we compute how much each action simulated by agent 'i' affects its neighbor k
				{ 
					if(i==1015)
					{
					  std::cout << "Agent 	" << k <<" distance to goal: " << RVO::abs(goal- agents_[k]->position_)<< " vs mine " << RVO::abs(goal- agents_[i]->position_)<< "\n";
					}
					
					if((allNeigh)||((RVO::abs(goal- agents_[i]->position_))>(RVO::abs(goal- agents_[k]->position_)))) //If its closer than me to my goal...
					{
							    neighbors=neighbors+1;
							    changeVel= agents_[i]->VcomputeVelChange(j);
							    
								//politeness= politeness+ (changeVel)/(float)1.5; //Consider the change in velocity, as a by product of my (agent i) action choice, in the politeness
								
								vect.push_back(std::make_pair((changeVel)/(float)1.5, agents_[j]->id_));
								
								//Vchange[(changeVel/(float)1.5)]= agents_[j]->id_;
								if((i==0))
								{
									//std::cout <<" Neighbor " <<j << " velocity change " <<changeVel << " with my Vpref " << agents_[i]->prefVelocity_	 << " size " << Vchange.size()<<"\n";
									//getchar();
								}
								agents_[k]->newVelocity_= agents_[i]->neighNewVelocity_;
								
											
					}
	
				}	
				
				
				
				if(t==0) //in the first timesteps, the neighbors react to the previously observed velocity of the agent
				{					
					agents_[k]->computeHypoVelocityNeighbor();
				}
							
			}
			
		
		
		   //Sort the neighbors' velocity change by magnitude of change:
		  // if(i==15)
		//   {//std::cout <<"\n\n********\n";
			   
			/*   std::map<float,int > :: iterator it; 
			   for (it=Vchange.begin() ; it!=Vchange.end() ; it++) 
        std::cout << "(" << (*it).first << ", "
             << (*it).second << ")" << "\n"; 
			   */
			   
if(t>0) //For all t>0, we compute how much each action simulated by agent 'i' affects its neighbor k
				{			
					
sort(vect.begin(), vect.end());		
int contadour2=0;
 politeness=0;
 politeness4=0;

for (int e=0; e<neighbors; e++) 
    { 
        // "first" and "second" are used to access 
        // 1st and 2nd element of pair respectively 
        //std::cout << vect[e].first << " - " << vect[e].second << "\n"; 
             politeness4=politeness4+ vect[e].first;
             contadour2++;
             
              if((i==7))
				{
				//	std::cout <<" Neighbor agent " <</*vect[e].second*/  agents_[i]->agentNeighbors_[vect[e].second].second->id_ << " velocity change " <<vect[e].first << " with my Vpref " << agents_[i]->prefVelocity_	 << "\n";
					//getchar();
				}
             
              if(contadour2==contadourX)
             {
				 break;
				 }
				 
				 
    } 			



if(i==12)
				   {
					/*   std::cout << " Politeness: "<< politeness << " vs politeness3 " << politeness3 << " vs politeness4 " << politeness4 <<"\n";
					  std::cout << " Neighbors: "<< neighbors << " vs contadour " << contadour << " vs contadour2 " << contadour2<< "\n";
getchar();*/
}

neighbors=contadour2;
politeness=politeness4;
}
	//	avgPolitenessCNav= p->first;	   
/*    
		   
		   */
		   
		   //std::cout<< "The most constrained is " << Vchange[Vchange.end()].second << " with Vchange of " << Vchange[Vchange.size()].first <<"\n";
		   
		  // }
		   
		
		if((i==15))
			{
			  	//std::cout <<" Time t " <<t << " action " <<actionNum	 <<"\n";
				
			}
			
			agents_[i]->computeHypoVelocityAgent(); // Compute the projected collision free velocity of the agent in the future
			if((i==1015))
			{
			//getchar();
			//std::cout <<"At T= " << t<<" Expected NewVel of action " << actionNum << " : " << agents_[i]->newVelocity_ << " with Vpref: " <<agents_[i]->prefVelocity_ << " POL: " << (agents_[i]->newVelocity_*agents_[i]->prefVelocity_)/(float)2.25 <<"\n";
			
			}
			Vector2 finalVel= agents_[i]->newVelocity_;  //finalVel is the projected velocity
			
			if(t==0)
			{
			setAgentInitialVelocity(i,agents_[i]->newVelocity_,actionNum);
			setAgentInitialPrefVelocity(i,agents_[i]->prefVelocity_,actionNum);
			}
			
			
			if((i==7)/*&&(actionNum==0)*/)
			{
			 // std::cout <<"At T= " << t<<" Expected FirstVel of action " << actionNum << " : " << agents_[i]->newVelocity_ << " magnitude " << abs(agents_[i]->newVelocity_) << " Vpref: " <<agents_[i]->prefVelocity_ << "\n";
				
			}
			
							
			agents_[i]->Vupdate(); //Agent i uses the finalVel
	
	        for (size_t j = 0; j < agents_[i]->agentNeighbors_.size(); ++j) 
			{
				size_t k=  agents_[i]->agentNeighbors_[j].second->id_;
				agents_[k]->Vupdate();
			}
			float politeness2= agents_[i]->prefVelocity_*agents_[i]->newVelocity_/2.25;
			
			if(neighbors==0)
			{
				//politeness=1;
			//	avgPolitenessCNav+=politeness;
			}
			
			
			if(neighbors>0)
			{
				
				if(i==15)
				{
				//std::cout << "Adding politeness " << politeness << " with numNeigh " <<  neighbors << " total: " << politeness/(float)neighbors << " and current Avg is " << avgPolitenessCNav<< "\n";
				}
			}
			
			//avgPolitenessCNav= 
				
		        avgPolitenessALAN+=politeness2;
		        avgGoalProg+=(normalize(goalVector)* finalVel)/((float)agents_[i]->maxSpeed_);
			
		/////////////////////////////////////////////////////////////////
		///Reward function computation: Rg*coord_factor + (1-coord_factor)*Rp, where Rg: Goal progress, and Rp: Politeness
			
			        goalProgress=/*goalProgress+*/((normalize(goalVector)* finalVel)/((float)agents_[i]->maxSpeed_));
			
					if(t==0)
					{
						politeness=0;
						goalProgress=0;
					}
					
					if(t>0)
					{
						if(t==1)
						{
							//goalProgress=goalProgress/(float)2; //averages the goal progress of  the first two timesteps
						
						}
					
						
						if(neighbors>0) //(numNeigh)//
						{
							avgPolitenessCNav+=politeness/(float)neighbors;
							reward= reward + ((goalProgress)*(1-coord_factor)+  (politeness)*/*politeness2*/(coord_factor)/(float)neighbors);//(float)((1+1*agents_[i]->newVelocity_*agents_[i]->newVelocity_));
						}
						else
						{
							politeness=1;
							avgPolitenessCNav+=politeness/(float)neighbors;
							reward= reward + ((goalProgress)*(1-coord_factor)+  (politeness)*/*politeness2*/(coord_factor)/1);//(float)(1+1*agents_[i]->prefVelocity_*agents_[i]->prefVelocity_);
						}
						
						
		/////////////////////////////////////////////////////////////////////////				
							if((i==7))	
						{
						    
						//std::cout << "Action: " << actionNum<< " Goal: " << goalProgress << " Polite: " << politeness/(float)neighbors << " Reward: " <<reward <<  " neighbors: " << neighbors<< "\n \n\n";
						
							
						}			
						
					}
		
			if((abs(agents_[i]->Vposition_-goal)<0.15)&&(goalFound!=1))
			{		
				goalFound=1;
				foundAt=t;
				endOfPlan=t+1;
	
			}
			
		} // End of time Horizon
			
		
		
		//Reset previous parameters of the neighbors
		int p;
		for(n=0;n<numNeighbors;n++)
		{
			
			if(NeighborList[n]>-1)
			{  
				p=agents_[NeighborList[n]]->id_;
				agents_[p]->Vposition_=agents_[p]->position_;
				agents_[p]->prefVelocity_= agents_[p]->VprefVelocity_ ;
				agents_[p]->Vvelocity_=agents_[p]->velocity_;
				
			}
			
		}
		
		
		agents_[agentNo]->prefVelocity_= prefVelocityBackup_;
		
		
		setCnavPoliteness(i,avgPolitenessCNav,actionNum);
		setALANPoliteness(i,avgPolitenessALAN,actionNum);
		setGoalProg(i,avgGoalProg/(float)timeHorizon,actionNum);
		
	//std::cout <<"LEAVING SIMULATEVEL..\n";
		return reward/(endOfPlan-1); //Returns the average reward for all timesteps
		
		
		
	}
		
	
float RVOSimulator::SimulateVelNoComm(size_t agentNo, RVO::Vector2 goal, int timeHorizon, int actionNum, int numNeighbors, int numAgents, float coord_factor, float ActionSpeed, float ActionDir)
	{
		float VprefVelocity_, politeness=0,goalProgress=0,reward=0;
		int NeighborList[numNeighbors], neighbors=0,n,r=0,endOfPlan=timeHorizon, foundAt=0;
		Vector2 goalVector,prefVelocityBackup_= agents_[agentNo]->prefVelocity_;
		size_t i = agentNo;
		bool goalFound=0; // When the goal is reached, the agent just stays there
				float avgPolitenessCNav=0;
		float avgPolitenessALAN=0;
		float avgGoalProg=0;
			
		for (n=0; n<numNeighbors; n++) 
		{
			NeighborList[n]=-1;
		}

		for(n=0;n<numAgents;n++)
		{
			agents_[n]->vOpt_ = agents_[n]->Vvelocity_;
			agents_[n]->Vposition_ = agents_[n]->position_;
			agents_[n]->Vvelocity_ = agents_[n]->velocity_;
			agents_[n]->VprefVelocity_ = agents_[n]->prefVelocity_;
			
		}
		agents_[i]->computeNeighbors();	
		
		
		
		
        #pragma omp parallel for
        
        kdTree_->VbuildAgentTree();
		
		for(int t=0;t<timeHorizon;t++) // For each timestep in the time horizon...
		{
			
		    goalVector= RVO::normalize(goal- agents_[i]->Vposition_);
	      
	        if (t<timeHorizon)  //...the agent computes a new preferred velocity
			{
				agents_[i]->prefVelocity_= goalVector*ActionSpeed;
			    agents_[i]->prefVelocity_=Vector2(agents_[i]->prefVelocity_.x()*std::cos(ActionDir)+(agents_[i]->prefVelocity_.y())*std::sin(ActionDir), (agents_[i]->prefVelocity_.y())*std::cos(ActionDir)+(agents_[i]->prefVelocity_.x())*-std::sin(ActionDir));
			}
					
					
					
			//For each neighbor of the agent, we need to compute its collision-free velocity		
			for (size_t j = 0; j < agents_[i]->agentNeighbors_.size(); ++j) 
			{ 
				size_t k=  agents_[i]->agentNeighbors_[j].second->id_;
					
				int exists=0;
				for (int u=0; u<numNeighbors; u++) 
				{
					if((int)k==NeighborList[u])
					{
						
						exists=1;
					}
				}
				
				if(exists==0)
				{ 
					NeighborList[r]=(int)k;
										
					if(r<(numNeighbors-1))
					{
						r++;
					}
					
				}
             
			
										
				
					  neighbors=neighbors+1;
				
					agents_[k]->newVelocity_=agents_[k]->velocity_;
						
			}
			
		
			
			agents_[i]->computeHypoVelocityAgent_all(); // Compute the projected collision free velocity of the agent in the future
			
			Vector2 finalVel= agents_[i]->newVelocity_;  //finalVel is the projected velocity
			
			politeness= (agents_[i]->prefVelocity_*	finalVel)/(float)2.25;			
			 goalProgress=((normalize(goalVector)* finalVel)/((float)1.5));
			 
			agents_[i]->Vupdate(); //Agent i uses the finalVel
	
	        for (size_t j = 0; j < agents_[i]->agentNeighbors_.size(); ++j) 
			{
				size_t k=  agents_[i]->agentNeighbors_[j].second->id_;
				agents_[k]->Vupdate();
			}
			
						
			if(t==0)
			{
			setAgentInitialVelocity(i,agents_[i]->newVelocity_,actionNum);
			setAgentInitialPrefVelocity(i,agents_[i]->prefVelocity_,actionNum);
			}
			
			float politeness2= agents_[i]->prefVelocity_*agents_[i]->newVelocity_/2.25;
			
			if(neighbors==0)
			{
				avgPolitenessCNav+=politeness;
			}
			
			
			if(neighbors>0)
			{
				avgPolitenessCNav+=politeness/(float)neighbors;
				if(i==15)
				{
				//std::cout << "Adding politeness " << politeness << " with numNeigh " <<  neighbors << " total: " << politeness/(float)neighbors << " and current Avg is " << avgPolitenessCNav<< "\n";
				}
			}
				
		    avgPolitenessALAN+=politeness2;
		    avgGoalProg+=(normalize(goalVector)* finalVel)/((float)agents_[i]->maxSpeed_);
			
		/////////////////////////////////////////////////////////////////
		///Reward function computation: Rg*coord_factor + (1-coord_factor)*Rp, where Rg: Goal progress, and Rp: Politeness
			         
			       
			
					
						
							//goalProgress=goalProgress/(float)2; //averages the goal progress of  the first two timesteps
						
						
						
					
							reward= reward + (goalProgress)*coord_factor+  (politeness)*(1-coord_factor);
						
						
						
		/////////////////////////////////////////////////////////////////////////				
							
											
						
					
		
			if((abs(agents_[i]->Vposition_-goal)<0.15)&&(goalFound!=1))
			{		
				goalFound=1;
				foundAt=t;
				endOfPlan=t+1;
	
			}
			
		} // End of time Horizon
			
		
		
		//Reset previous parameters of the neighbors
		int p;
		for(n=0;n<numNeighbors;n++)
		{
			
			if(NeighborList[n]>-1)
			{  
				p=agents_[NeighborList[n]]->id_;
				agents_[p]->Vposition_=agents_[p]->position_;
				agents_[p]->prefVelocity_= agents_[p]->VprefVelocity_ ;
				agents_[p]->Vvelocity_=agents_[p]->velocity_;
				
			}
			
		}
		
		
			
		setCnavPoliteness(i,avgPolitenessCNav,actionNum);
		setALANPoliteness(i,avgPolitenessALAN,actionNum);
		setGoalProg(i,avgGoalProg,actionNum);
		
		
		agents_[agentNo]->prefVelocity_= prefVelocityBackup_;
		
	
		return reward/(endOfPlan); //Returns the average reward for all timesteps
		
		
		
	}	
	
	const Vector2 &RVOSimulator::getNeighPredVel(size_t agentNo, size_t neighbor, int t) const
	{
		return agents_[agentNo]->predVelocityNeigh_[neighbor][t];
	}
	

	void RVOSimulator::buildTree()
	{
		
		kdTree_->buildAgentTree();
		
	}
    
	void RVOSimulator::setVvalues()
	{
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) 
		{
			
			
			agents_[i]->Vposition_ = agents_[i]->position_;
			agents_[i]->Vvelocity_ = agents_[i]->velocity_;
			
		}
		
	} 
	
	
	float RVOSimulator::getSimComp(size_t agentNo, int component, int action, int timestep)
	{
		
		
		if(component==1)
		{
			
			if((action==2)&&(agentNo==0))
			{
			//std::cout << " About TO RETURN " << 	agents_[agentNo]->SimComp1_t[action][timestep]<< " from " <<agentNo <<" "<< component << " "   << action<<" " <<timestep <<  "\n";
			}
		return  agents_[agentNo]->SimComp1_t[action][timestep];
		
		
		}
		else
		{
		return  agents_[agentNo]->SimComp2_t[action][timestep];
		}
		
	}
	
	
	const Vector2 &RVOSimulator::getSimNeighVel(size_t agentNo, size_t neighbor, int action, int timestep) const
	{
		
		return  agents_[agentNo]->SimVelNeigh_t[neighbor][action][timestep];
		
	}
	
	
	const Vector2 &RVOSimulator::getSimNeighPos(size_t agentNo, size_t neighbor, int action, int timestep) const
	{
		
		return  agents_[agentNo]->SimPosNeigh_t[neighbor][action][timestep];
		
	}
	size_t RVOSimulator::addAgent(const Vector2 &position)
	{
		if (defaultAgent_ == NULL) {
			return RVO_ERROR;
		}
		
		Agent *agent = new Agent(this);
		
		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
		agent->neighborDist_ = defaultAgent_->neighborDist_;
		agent->radius_ = defaultAgent_->radius_;
		agent->timeHorizon_ = defaultAgent_->timeHorizon_;
		agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
		agent->velocity_ = defaultAgent_->velocity_;
		
		agent->id_ = agents_.size();
		
		agents_.push_back(agent);
		
		return agents_.size() - 1;
	}
	
	size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
	{
		Agent *agent = new Agent(this);		
		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->timeHorizon_ = timeHorizon;
		agent->timeHorizonObst_ = timeHorizonObst;
		agent->velocity_ = velocity;
		agent->id_ = agents_.size();
		agents_.push_back(agent);
		
		return agents_.size() - 1;
	}
	
	size_t RVOSimulator::addObstacle(const std::vector<Vector2> &vertices)
	{
		if (vertices.size() < 2) {
			return RVO_ERROR;
		}
		
		const size_t obstacleNo = obstacles_.size();
		
		for (size_t i = 0; i < vertices.size(); ++i) {
			Obstacle *obstacle = new Obstacle();
			obstacle->point_ = vertices[i];
			
			if (i != 0) {
				obstacle->prevObstacle_ = obstacles_.back();
				obstacle->prevObstacle_->nextObstacle_ = obstacle;
			}
			
			if (i == vertices.size() - 1) {
				obstacle->nextObstacle_ = obstacles_[obstacleNo];
				obstacle->nextObstacle_->prevObstacle_ = obstacle;
			}
			
			obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);
			
			if (vertices.size() == 2) {
				obstacle->isConvex_ = true;
			}
			else {
				obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
			}
			
			obstacle->id_ = obstacles_.size();
			
			obstacles_.push_back(obstacle);
		}
		
		return obstacleNo;
	}
	
	void RVOSimulator::doStep()
	{
		kdTree_->buildAgentTree();
		
#ifdef _OPENMP
#pragma omp parallel for
#endif
		
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) 
			{
				
			agents_[i]->computeNeighbors();
			//agents_[i]->HcomputeNewVelocity();// Assuming type 1 is Helbing				
			//agents_[i]->computeNeighbors();
			agents_[i]->computeNewVelocity();
			}
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) 
		{
			agents_[i]->update();
		}
		
		globalTime_ += timeStep_;
	}
	
	size_t RVOSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
	}
	
	int RVOSimulator::getAgentAgentNeighborConstraint(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->constraint_[neighborNo];
	}
	
	float RVOSimulator::getAgentConstraint(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->NonOrderConstraint_[neighborNo];
	}
	
	size_t RVOSimulator::getAgentAgentSimilarity(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->similarity_[neighborNo];
	}
	
	size_t RVOSimulator::getAgentMaxNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->maxNeighbors_;
	}
	
	float RVOSimulator::getAgentMaxSpeed(size_t agentNo) const
	{
		return agents_[agentNo]->maxSpeed_;
	}
	
	float RVOSimulator::getAgentNeighborDist(size_t agentNo) const
	{
		return agents_[agentNo]->neighborDist_;
	}
	
	float RVOSimulator::getAgentNeighborObsSim(size_t agentNo,size_t neighborNo) const
	{
		return agents_[agentNo]->observedSim[neighborNo];
	}
	
	float RVOSimulator::getAgentNeighborExtSim(size_t agentNo,size_t neighborNo) const
	{
		return agents_[agentNo]->externalSim[neighborNo];
	}
	
	size_t RVOSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->agentNeighbors_.size();
	}
	
	size_t RVOSimulator::getAgentNumObstacleNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_.size();
	}
	
	size_t RVOSimulator::getAgentNumORCALines(size_t agentNo) const
	{
		return agents_[agentNo]->orcaLines_.size();
	}
	
	size_t RVOSimulator::getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
	}
	
	const Line &RVOSimulator::getAgentORCALine(size_t agentNo, size_t lineNo) const
	{
		return agents_[agentNo]->orcaLines_[lineNo];
	}
	
	const Vector2 &RVOSimulator::getAgentPosition(size_t agentNo) const
	{
		return agents_[agentNo]->position_;
	}
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity(size_t agentNo) const
	{
        if(agentNo==0)
        {
			// std::cout << "GETting the PREF VEL of 0: " << agents_[agentNo]->prefVelocity_  << "\n";
        }
		
		return agents_[agentNo]->prefVelocity_;
		
		
	}
	
	const Vector2 &RVOSimulator::getAgentGoal(size_t agentNo) const
	{
		return agents_[agentNo]->goal_;
	}
	
	
	const Vector2 &RVOSimulator::getAgentGoalPrefVelocity(size_t agentNo) const
	{
        if(agentNo==0)
        {
			// std::cout << "GETting the PREF VEL of 0: " << agents_[agentNo]->prefVelocity_  << "\n";
        }
		
		return agents_[agentNo]->goalprefVelocity_;
	}
	
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity0(size_t agentNo) const
	{
		
		return agents_[agentNo]->prefVelocity0_;
		
	}
	
	const Vector2 &RVOSimulator::getAgentPredVelocity(size_t agentNo, int t) const
	{
		
		return agents_[agentNo]->predVelocity_[t];
		
	}
	
	
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity1(size_t agentNo) const
	{
		
		return agents_[agentNo]->prefVelocity1_;
		
	}
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity2(size_t agentNo) const
	{
		
		return agents_[agentNo]->prefVelocity2_;
		
	}
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity3(size_t agentNo) const
	{
		
		return agents_[agentNo]->prefVelocity3_;
		
	}
	
	const Vector2 &RVOSimulator::getAgentInitialPrefVelocity(size_t agentNo, int actionNum) const
	{
		
		if((actionNum==0)&&(agentNo==15))
		{
			//std::cout << " Looking for action and agent " << actionNum << " " << agentNo<< "\n";
			
			}
		
		
		return agents_[agentNo]->initprefvelocity_[actionNum];
	}
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity4(size_t agentNo) const
	{
		
		return agents_[agentNo]->prefVelocity4_;
		
	}
	
	
	float RVOSimulator::getAgentRadius(size_t agentNo) const
	{
		return agents_[agentNo]->radius_;
	}
	
	float RVOSimulator::getAgentTimeHorizon(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizon_;
	}
	
	float RVOSimulator::getAgentTimeHorizonObst(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizonObst_;
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->velocity_;
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity0(size_t agentNo) const
	{
		return agents_[agentNo]->velocity0_;
	}
	
	const Vector2 &RVOSimulator::getAgentInitialVelocity(size_t agentNo, int actionNum) const
	{
		
		if((actionNum==0)&&(agentNo==15))
		{
			//std::cout << " Looking for action and agent " << actionNum << " " << agentNo<< "\n";
			
			}
		
		
		return agents_[agentNo]->initvelocity_[actionNum];
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity1(size_t agentNo) const
	{
		return agents_[agentNo]->velocity1_;
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity2(size_t agentNo) const
	{
		return agents_[agentNo]->velocity2_;
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity3(size_t agentNo) const
	{
		return agents_[agentNo]->velocity3_;
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity4(size_t agentNo) const
	{
		return agents_[agentNo]->velocity4_;
	}
	
	
	void RVOSimulator::setAgentPrefVelocity0(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity0_ = prefVelocity;
		
        if(agentNo==0)
        {
			// std::cout << "Setting the PREF VEL to" << agents_[agentNo]->prefVelocity_  << "\n";
        }
	}
	
	void RVOSimulator::setAgentPrefVelocity1(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity1_ = prefVelocity;
		
        if(agentNo==0)
        {
			// std::cout << "Setting the PREF VEL to" << agents_[agentNo]->prefVelocity_  << "\n";
        }
	}
	
	void RVOSimulator::setAgentPrefVelocity2(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity2_ = prefVelocity;
		
        if(agentNo==0)
        {
			// std::cout << "Setting the PREF VEL to" << agents_[agentNo]->prefVelocity_  << "\n";
        }
	}
	
	void RVOSimulator::setAgentPrefVelocity3(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity3_ = prefVelocity;
		
        if(agentNo==0)
        {
			// std::cout << "Setting the PREF VEL to" << agents_[agentNo]->prefVelocity_  << "\n";
        }
	}
	
	
	void RVOSimulator::setAgentPrefVelocity4(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity4_ = prefVelocity;
		
        if(agentNo==0)
        {
			// std::cout << "Setting the PREF VEL to" << agents_[agentNo]->prefVelocity_  << "\n";
        }
	}
	
	float RVOSimulator::getGlobalTime() const
	{
		return globalTime_;
	}
	
	size_t RVOSimulator::getNumAgents() const
	{
		return agents_.size();
	}
	
	size_t RVOSimulator::getNumObstacleVertices() const
	{
		return obstacles_.size();
	}
	
	size_t RVOSimulator::getAgentMostSimilarAgent(size_t agentNo)
	{
		return agents_[agentNo]->MostSimilar_;
	}
	
	size_t RVOSimulator::getAgentMostConstrainedAgent(size_t agentNo)
	{
		return agents_[agentNo]->MostConstrained_;
	}
	
	size_t RVOSimulator::getAgentLeastConstrainedAgent(size_t agentNo)
	{
		return agents_[agentNo]->LeastConstrained_;
	}
	
	const Vector2 &RVOSimulator::getObstacleVertex(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->point_;
	}
	
	size_t RVOSimulator::getNextObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->nextObstacle_->id_;
	}
	
	size_t RVOSimulator::getPrevObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->prevObstacle_->id_;
	}
	
	float RVOSimulator::getTimeStep() const
	{
		return timeStep_;
	}
	
	void RVOSimulator::processObstacles()
	{
		kdTree_->buildObstacleTree();
	}
	
	bool RVOSimulator::queryVisibility(const Vector2 &point1, const Vector2 &point2, float radius) const
	{
		return kdTree_->queryVisibility(point1, point2, radius);
	}
	
	void RVOSimulator::setAgentinGoal(size_t agentNo,bool pos)
	{
		agents_[agentNo]->inGoal_ = pos;
		
	}
	
	void RVOSimulator::setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
	{
		if (defaultAgent_ == NULL) {
			defaultAgent_ = new Agent(this);
		}
		
		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeHorizonObst_ = timeHorizonObst;
		defaultAgent_->velocity_ = velocity;
		defaultAgent_->numInteractions = 0;
		defaultAgent_->MostSimilar_ = -1000;
		
		int i;
		
		for (i=0; i<100; i++)
		{
			defaultAgent_->avgNeighInfluence[i]=0;	
			defaultAgent_->numNeighInfluence[i]=0;
			defaultAgent_->intentionSim[i]=0;
			defaultAgent_->observedSim[i]=0;
			defaultAgent_->externalSim[i]=0;
			
		}
		
		for (i=0; i<15; i++)
		{
			defaultAgent_->constraint_[i]=-1000;	
			defaultAgent_->similarity_[i]=-1000;
		}
	}
	
	void RVOSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
	{
		agents_[agentNo]->maxNeighbors_ = maxNeighbors;
	}
	
	void RVOSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
	{
		agents_[agentNo]->maxSpeed_ = maxSpeed;
	}
	
	void RVOSimulator::setAgentGoal(size_t agentNo, const Vector2 &goal)
	{
		agents_[agentNo]->goal_ = goal;
	}
	
	void RVOSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
	{
		agents_[agentNo]->neighborDist_ = neighborDist;
	}
	
	void RVOSimulator::setAgentPosition(size_t agentNo, const Vector2 &position)
	{
		agents_[agentNo]->position_ = position;
	}
	
	void RVOSimulator::setAgentPrefVelocity(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity_ = prefVelocity;
		
        if(agentNo==0)
        {
			// std::cout << "Setting the PREF VEL to" << agents_[agentNo]->prefVelocity_  << "\n";
        }
	}
	
	void RVOSimulator::setAgentGoalPrefVelocity(size_t agentNo, const Vector2 &goalprefVelocity)
	{
		agents_[agentNo]->goalprefVelocity_ = goalprefVelocity;
		
        if(agentNo==0)
        {
			// std::cout << "Setting the PREF VEL to" << agents_[agentNo]->prefVelocity_  << "\n";
        }
	}
	
	void RVOSimulator::setAgentAgentNeighborConstraint(size_t agentNo, size_t rank, size_t rankid) 
	{ // agentNo is the base agent, neighor
		agents_[agentNo]->constraint_[rank]=rankid;
	}
	
	void RVOSimulator::setAgentAgentSimilarity(size_t agentNo, size_t rank, size_t rankid) 
	{ // agentNo is the base agent, neighor
		agents_[agentNo]->similarity_[rank]=rankid;
	}
	
	void RVOSimulator::setAgentRadius(size_t agentNo, float radius)
	{
		agents_[agentNo]->radius_ = radius;
	}
	
	void RVOSimulator::setAgentMostSimilarAgent(size_t agentNo, size_t agentNo2)
	{
		agents_[agentNo]->MostSimilar_ = agentNo2;
	}
	
	void RVOSimulator::setAgentMostConstrainedAgent(size_t agentNo, size_t agentNo2)
	{
		agents_[agentNo]->MostConstrained_ = agentNo2;
	}
	
	void RVOSimulator::setAgentLeastConstrainedAgent(size_t agentNo, size_t agentNo2)
	{
		agents_[agentNo]->LeastConstrained_ = agentNo2;
	}
	
	void RVOSimulator::setAgentTimeHorizon(size_t agentNo, float timeHorizon)
	{
		agents_[agentNo]->timeHorizon_ = timeHorizon;
	}
	
	void RVOSimulator::setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst)
	{
		agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
	}
	
	void RVOSimulator::setAgentNeighAngle(size_t agentNo, size_t Neigh, float angle)
	{
		agents_[agentNo]->NeighAngle[Neigh] = angle;
	}
	
	
	
	void RVOSimulator::setAgentVelocity(size_t agentNo, const Vector2 &velocity)
	{
		agents_[agentNo]->velocity_ = velocity;
	}
	
	
	void RVOSimulator::setCnavPoliteness(size_t agentNo, float avgPolitenessCNav,int actionNum)
	{
		agents_[agentNo]->cnavpoliteness_[actionNum] =avgPolitenessCNav;
		
	}
	
	void RVOSimulator::setALANPoliteness(size_t agentNo, float avgPolitenessALAN,int actionNum)
	{
		agents_[agentNo]->alanpoliteness_[actionNum] =avgPolitenessALAN;
		
	}
	
	float  RVOSimulator::getAgentCnavPoliteness(size_t agentNo, int actionNum)
	{
		
		if((actionNum==0)&&(agentNo==15))
		{
			//std::cout << " Looking for action and agent " << actionNum << " " << agentNo<< "\n";
			
			}
		
		
		return agents_[agentNo]->cnavpoliteness_[actionNum];
	}
	
	float  RVOSimulator::getAgentALANPoliteness(size_t agentNo, int actionNum)
	{
		
		if((actionNum==0)&&(agentNo==15))
		{
			//std::cout << " Looking for action and agent " << actionNum << " " << agentNo<< "\n";
			
			}
		
		
		return agents_[agentNo]->alanpoliteness_[actionNum];
	}
	
	float  RVOSimulator::getAgentGoalProg(size_t agentNo, int actionNum)
	{
		
		if((actionNum==0)&&(agentNo==15))
		{
			//std::cout << " Looking for action and agent " << actionNum << " " << agentNo<< "\n";
			
			}
		
		
		return agents_[agentNo]->goalprog_[actionNum];
	}
	
	void RVOSimulator::setGoalProg(size_t agentNo, float avgGoalProg,int actionNum)
	{
		agents_[agentNo]->goalprog_[actionNum] =avgGoalProg;
		
	}
	
	void RVOSimulator::setAgentInitialVelocity(size_t agentNo, const Vector2 &velocity, int actionNum)
	{
		agents_[agentNo]->initvelocity_[actionNum] = velocity;
		
		if((actionNum==0)&&(agentNo==15))
		{
			//std::cout << " Just SET it to " << velocity << "\n";
			
			}
	}
	
	
	void RVOSimulator::setAgentInitialPrefVelocity(size_t agentNo, const Vector2 &velocity, int actionNum)
	{
		agents_[agentNo]->initprefvelocity_[actionNum] = velocity;
		
		if((actionNum==0)&&(agentNo==15))
		{
			//std::cout << " Just SET it to " << velocity << "\n";
			
			}
	}
	
	
	void RVOSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}
}
