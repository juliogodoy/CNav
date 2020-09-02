/*
 * Agent.cpp
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

#include "Agent.h"

#include "KdTree.h"
#include "Obstacle.h"
#include <stdlib.h>
#include <iostream>


const float SOCIAL_SCALING = 2000.f; // 2000 N
const float AGENT_RADIUS_MAX = 0.5f; // 0.7m diamater
const float AGENT_REACTION_TIME = 0.5f; // 0.5s
const float REPULSE_SPRING_CONSTANT = 120000.f; // 2.4e4 kg/sec^2
const float COEFF_SLIDING_FRICTION = 240000.f;


namespace RVO {
	Agent::Agent(RVOSimulator *sim) : maxNeighbors_(0), maxSpeed_(0.0f), neighborDist_(0.0f), radius_(0.0f), sim_(sim), timeHorizon_(0.0f), timeHorizonObst_(0.0f), id_(0) { }
	
	
	
	void Agent::computeNeighborsNoObstacles()
	{
		obstacleNeighbors_.clear();
		//float rangeSq = sqr(timeHorizonObst_ * maxSpeed_ + radius_);
		//sim_->kdTree_->computeObstacleNeighbors(this, rangeSq);
		
		agentNeighbors_.clear();
		
		if (maxNeighbors_ > 0) {
			float rangeSq = sqr(neighborDist_);
			sim_->kdTree_->computeAgentNeighbors(this, rangeSq);
		}
	}
	
	void Agent::computeNeighbors()
	{
		obstacleNeighbors_.clear();
		float rangeSq = sqr(timeHorizonObst_ * maxSpeed_ + radius_);
		sim_->kdTree_->computeObstacleNeighbors(this, rangeSq);
		
		agentNeighbors_.clear();
		
		if (maxNeighbors_ > 0) {
			rangeSq = sqr(neighborDist_);
			sim_->kdTree_->computeAgentNeighbors(this, rangeSq);
		}
	}
    //Virtual
    void Agent::VcomputeNeighbors()
    {
        obstacleNeighbors_.clear();
        float rangeSq = sqr(timeHorizonObst_ * maxSpeed_ + radius_);
		
		
        sim_->kdTree_->VcomputeObstacleNeighbors(this, rangeSq);
		
        agentNeighbors_.clear();
		
        if (maxNeighbors_ > 0) {
            rangeSq = sqr(neighborDist_);
            sim_->kdTree_->VcomputeAgentNeighbors(this, rangeSq);
        }
    }
	
	 void Agent::VcomputeNeighborsModel(Agent *agentSource, float sourcetimeHorizonObstModel_, float sourcemaxSpeedModel_,float sourceradiusModel_ ,float sourceneighborDistModel_,  float timeHorizonObstModel_, float maxSpeedModel_,float radiusModel_ , int maxNeighborsModel_, float neighborDistModel_)
   {
        obstacleNeighbors_.clear();
        float rangeSq = sqr(timeHorizonObstModel_ * maxSpeedModel_ + radiusModel_);
		float rangeSqsource=sqr(sourcetimeHorizonObstModel_ * sourcemaxSpeedModel_ + sourceradiusModel_);
        sim_->kdTree_->VcomputeObstacleNeighborsExternal(agentSource,this, rangeSqsource, rangeSq);
		
        VagentNeighbors_.clear();
			
        if (maxNeighborsModel_ > 0) 
		{
            rangeSq = sqr(neighborDistModel_);
			rangeSqsource=sqr(sourceneighborDistModel_);
            sim_->kdTree_->VcomputeAgentNeighborsExternal(agentSource,this, rangeSqsource , rangeSq, maxNeighborsModel_);
        }
    }
	
  
	
	/* Search for the best new velocity. */
	void Agent::computeNewVelocity()
	{
		orcaLines_.clear();
		
		
		const float invTimeHorizonObst = 1.0f / timeHorizonObst_;
		
		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {
			
			const Obstacle *obstacle1 = obstacleNeighbors_[i].second;
			const Obstacle *obstacle2 = obstacle1->nextObstacle_;
			
			const Vector2 relativePosition1 = obstacle1->point_ - position_;
			const Vector2 relativePosition2 = obstacle2->point_ - position_;
			
			/*
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;
			
			for (size_t j = 0; j < orcaLines_.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}
			
			if (alreadyCovered) {
				continue;
			}
			
			/* Not yet covered. Check for collisions. */
			
			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);
			
			const float radiusSq = sqr(radius_);
			
			const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
			const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
			const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);
			
			Line line;
			
			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex_) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
					orcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
					orcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line.point = Vector2(0.0f, 0.0f);
				line.direction = -obstacle1->unitDir_;
				orcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */
			
			Vector2 leftLegDirection, rightLegDirection;
			
			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle2 = obstacle1;
				
				const float leg1 = std::sqrt(distSq1 - radiusSq);
				leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle1 = obstacle2;
				
				const float leg2 = std::sqrt(distSq2 - radiusSq);
				leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir_;
				}
				
				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir_;
				}
			}
			
			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */
			
			const Obstacle *const leftNeighbor = obstacle1->prevObstacle_;
			
			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;
			
			if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir_;
				isLeftLegForeign = true;
			}
			
			if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir_;
				isRightLegForeign = true;
			}
			
			/* Compute cut-off centers. */
			const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - position_);
			const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - position_);
			const Vector2 cutoffVec = rightCutoff - leftCutoff;
			
			/* Project current velocity on velocity obstacle. */
			
			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
			const float tLeft = ((velocity_ - leftCutoff) * leftLegDirection);
			const float tRight = ((velocity_ - rightCutoff) * rightLegDirection);
			
			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const Vector2 unitW = normalize(velocity_ - leftCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const Vector2 unitW = normalize(velocity_ - rightCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (rightCutoff + tRight * rightLegDirection)));
			
			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir_;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}
				
				line.direction = leftLegDirection;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}
				
				line.direction = -rightLegDirection;
				line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
		}
		
		const size_t numObstLines = orcaLines_.size();
	
		
		const float invTimeHorizon = 1.0f / timeHorizon_;
		
		
		/* Create agent ORCA lines. */
		
		//Get number of neighbors in this interaction
		numInteractions = numInteractions +1;
		avgNumNeigh=(avgNumNeigh*(numInteractions-1)+ agentNeighbors_.size())/numInteractions;
	
		
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) 
		{
			
			
			VprefDiff[i]=-1000;
			Vdiff[i]=-1000;	
			VelSimPref[i]=-1000;
			VelSimVel[i]=-1000;
			indiVel[i]=Vector2(0.0f, 0.0f);
			
			const Agent *const other = agentNeighbors_[i].second;
			
			
			if(id_==15)
			{
				//std::cout <<"Real Position of neighbor agent " << other->id_ << ": " <<other->Vposition_ << " and Vel " <<  other->Vvelocity_ <<"\n";
				
			}
			
			const Vector2 relativePosition = other->position_ - position_;
			const Vector2 relativeVelocity = velocity_ - other->velocity_;
			const float distSq = absSq(relativePosition);
			const float combinedRadius = radius_ + other->radius_;
			const float combinedRadiusSq = sqr(combinedRadius);
			
			Line line;
			Vector2 u;
			
			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = absSq(w);
				
				const float dotProduct1 = w * relativePosition;
				
				if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vector2 unitW = w / wLength;
					
					line.direction = Vector2(unitW.y(), -unitW.x());
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				}
				else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);
					
					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}
					else {
						/* Project on right leg. */
						line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}
					
					const float dotProduct2 = relativeVelocity * line.direction;
					
					u = dotProduct2 * line.direction - relativeVelocity;
				}
			}
			else {
				/* Collision. Project on cut-off circle of time timeStep. */
				const float invTimeStep = 1.0f / sim_->timeStep_;
				
				/* Vector from cutoff center to relative velocity. */
				const Vector2 w = relativeVelocity - invTimeStep * relativePosition;
				
				const float wLength = abs(w);
				const Vector2 unitW = w / wLength;
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				u = (combinedRadius * invTimeStep - wLength) * unitW;
			}
			
			line.point = velocity_ + 0.5f * u;
			
			if(id_==15)
			{//std::cout<< " Pushing line " << line.point <<"\n";
			}
			
			orcaLines_.push_back(line);
			
						
		}
		
		
		
		size_t lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);
		
		
		if (lineFail < orcaLines_.size()) {
			linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
		}
		
	
	}
	
	  float Agent::magnitude(Vector2 Point1, Vector2 Point2){
    return sqrtf( (Point2.x() - Point1.x()) * (Point2.x() - Point1.x()) + (Point2.y() - Point1.y()) * (Point2.y() - Point1.y()));
  }


////////////////////////////////////////////////////////////

  Vector2 Agent::closestPointLine2(Vector2 Point, Vector2 LineStart, Vector2 LineDir, double& dist){
    double LineMag;
    double U;
    Vector2 Intersection;
    
    LineMag = abs(LineDir);
    
    U = ( ( ( Point.x() - LineStart.x() ) * LineDir.x()) +
         ( (Point.y() - LineStart.y()) * LineDir.y()) ) /
    (LineMag * LineMag);
    
 
    if (U < 0.0f) U = 0.0f;
    if (U > 1.0f) U = 1.0f;
    
    Intersection = LineStart + U * LineDir;
  
    dist = magnitude(Point, Intersection);
    return Intersection;
  }
  ///////////////////////////////////////////////////////////
  

	
	void Agent::getMaxInfluenceVpref()
	{
		int maxInf=-1000;
		float tempMax=1000;
		
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) 
		{
			if(VelSimPref[i]<tempMax)
			{
				tempMax=	VelSimPref[i];
				maxInf= i;
				
			}
			
			
		}

		
	}
	
	void Agent::getMaxInfluenceVel()
	{
		
		int maxInf=-1000;
		float tempMax=1000;
		
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) 
		{
			if(VelSimVel[i]<tempMax)
			{
				tempMax=	VelSimVel[i];
				maxInf= i;
				
			}
			
			
		}
		
	
	}
	

	float Agent::VcomputeVelChange2(int neighbor) //includes other agents
	{
		indivorcaLines_.clear();
		float neighborVelChange=-1000, neighborGoalVelChange=-1000;
		
		
	    const float invTimeHorizonObst = 1.0f / timeHorizonObst_;
		const Agent *const other = agentNeighbors_[neighbor].second;
		
		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < other->obstacleNeighbors_.size(); ++i) {
			
			
			const Obstacle *obstacle1 = other->obstacleNeighbors_[i].second;
			const Obstacle *obstacle2 = obstacle1->nextObstacle_;
			
			const Vector2 relativePosition1 = obstacle1->point_ - other->Vposition_;
			const Vector2 relativePosition2 = obstacle2->point_ - other->Vposition_;
		
			/* 
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;
			
			for (size_t j = 0; j < indivorcaLines_.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - indivorcaLines_[j].point, indivorcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - indivorcaLines_[j].point, indivorcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}
			
			if (alreadyCovered) {
				continue;
			}
			
			/* 
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added. Legs extend cut-off line when
			 * nonconvex vertex.
			 */
			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);
			
			const float radiusSq = sqr(radius_);
			
			
			
            const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
			const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
			const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);
			
			Line line;
			
			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex_) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
					indivorcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
					indivorcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line.point = Vector2(0.0f, 0.0f);
				line.direction = -obstacle1->unitDir_;
				indivorcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */
			
			Vector2 leftLegDirection, rightLegDirection;
			
			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle2 = obstacle1;
				
				const float leg1 = std::sqrt(distSq1 - radiusSq);
				
				leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle1 = obstacle2;
				
				const float leg2 = std::sqrt(distSq2 - radiusSq);
				
				leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir_;
				}
				
				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir_;
				}
			}
			
			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */
			
			const Obstacle *const leftNeighbor = obstacle1->prevObstacle_;
			
			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;
			
			if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
			
				
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir_;
				isLeftLegForeign = true;
			}
			
			if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
			
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir_;
				isRightLegForeign = true;
			}
			
			/* Compute cut-off centers. */
            const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - other->Vposition_);
            const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - other->Vposition_);
			const Vector2 cutoffVec = rightCutoff - leftCutoff;
			
			/* Project current velocity on velocity obstacle. */
			
			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((other->Vvelocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
			const float tLeft = ((other->Vvelocity_ - leftCutoff) * leftLegDirection);
			const float tRight = ((other->Vvelocity_ - rightCutoff) * rightLegDirection);
			
			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const Vector2 unitW = normalize(other->Vvelocity_ - leftCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
				indivorcaLines_.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const Vector2 unitW = normalize(other->Vvelocity_ - rightCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
				indivorcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(other->Vvelocity_ - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(other->Vvelocity_ - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(other->Vvelocity_ - (rightCutoff + tRight * rightLegDirection)));
			
			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir_;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				indivorcaLines_.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}
				
				line.direction = leftLegDirection;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				indivorcaLines_.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}
				
				line.direction = -rightLegDirection;
				line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				indivorcaLines_.push_back(line);
				continue;
			}
		}
		
		
		
		
		const size_t numObstLines =indivorcaLines_.size();
				
		const float invTimeHorizon = 1.0f / timeHorizon_;
		
		//Here the agent first compute the constraints on the neighbor imposed by MY neighbors
		
		for (size_t i = 0; i < VagentNeighbors_.size(); ++i) 
		{
			
			const Agent *const other2 = VagentNeighbors_[i].second;
			if(other2->id_ != other->id_)
		{
	
			const Vector2 relativePosition = other2->Vposition_-other->Vposition_; // "other/other2" vs "me/other" 
			const Vector2 relativeVelocity = other->Vvelocity_ - other2->Vvelocity_;
			const float distSq = absSq(relativePosition);
			const float combinedRadius = other2->radius_ + other->radius_;
			const float combinedRadiusSq = sqr(combinedRadius);
			
					
			Line line;
			Vector2 u;
			
			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition; 
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = absSq(w);
				
				const float dotProduct1 = w * relativePosition;
				
				if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vector2 unitW = w / wLength;
					
					line.direction = Vector2(unitW.y(), -unitW.x());
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				} else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);
					
					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					} else {
						/* Project on right leg. */
						line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}
					
					const float dotProduct2 = relativeVelocity * line.direction;
					
					u = dotProduct2 * line.direction - relativeVelocity;
				}
			} else {
				
				/* Collision. Project on cut-off circle of time timeStep. */
				const float invTimeStep = 1.0f / sim_->timeStep_;
				
				/* Vector from cutoff center to relative velocity. */
				const Vector2 w = relativeVelocity - invTimeStep * relativePosition;
				
				const float wLength = abs(w);
				const Vector2 unitW = w / wLength;
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				u = (combinedRadius * invTimeStep - wLength) * unitW;
			}
			
			line.point = other->Vvelocity_ + 0.5f * u;
			
			
			if(this->id_==16)
		{
			
		   // std::cout <<" Another line added to neighbor " << other->id_ << " by its neighbor " <<other2->id_ << "\n";
		}	
			
			
			indivorcaLines_.push_back(line);
			
			
			if(this->id_==16)
		{
			
		//std::cout <<"FIRST  Neighbor " << other->id_ << " (neigh is " <<other2->prefVelocity_ << ", pos: " <<other2->Vposition_ << " my pos " <<other->Vposition_ << " Vvel: " <<other2->Vvelocity_ << ")\n";
		}
		}
		}
		
		
	/*		size_t lineFF = linearProgram2(indivorcaLines_, maxSpeed_, other->prefVelocity_, false, neighNewVelocity_); // Because I don't know my neighbors' Vpref
		
				
		if (lineFF < indivorcaLines_.size()) {
			linearProgram3(indivorcaLines_, numObstLines, lineFF, maxSpeed_, neighNewVelocity_);
		}
		*/
		
		//indivorcaLines_.clear();
		
			
		if(this->id_==1015)
		{
			
		std::cout <<"FIRST  Neighbor " << other->id_ << "Change is " << maxSpeed_-RVO::abs(other->prefVelocity_-neighNewVelocity_) << " Prev Vel is " << 	other->prefVelocity_ << " but new vel is " << neighNewVelocity_ << ")\n";
		}
		
		
		
		
		//Here the agent computes the constraints on the neighbor imposed by HIMSELF
		const Vector2 relativePosition = Vposition_ - other->Vposition_;
		
		const Vector2 relativeVelocity = other->Vvelocity_ - Vvelocity_;//prefVelocity_;//Vvelocity_;
		const float distSq = absSq(relativePosition);
		const float combinedRadius = radius_ + other->radius_;
		const float combinedRadiusSq = sqr(combinedRadius);
		
		Line line;
		Vector2 u;
		
		if (distSq > combinedRadiusSq) {
			/* No collision. */
			const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
			/* Vector from cutoff center to relative velocity. */
			const float wLengthSq = absSq(w);
			
			const float dotProduct1 = w * relativePosition;
			
			if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
				/* Project on cut-off circle. */
				const float wLength = std::sqrt(wLengthSq);
				const Vector2 unitW = w / wLength;
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				u = (combinedRadius * invTimeHorizon - wLength) * unitW;
			}
			else {
				/* Project on legs. */
				const float leg = std::sqrt(distSq - combinedRadiusSq);
				
				if (det(relativePosition, w) > 0.0f) {
					/* Project on left leg. */
					line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
				}
				else {
					/* Project on right leg. */
					line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
				}
				
				const float dotProduct2 = relativeVelocity * line.direction;
				
				u = dotProduct2 * line.direction - relativeVelocity;
			}
		}
		else {
			/* Collision. Project on cut-off circle of time timeStep. */
			const float invTimeStep = 1.0f / sim_->timeStep_;
			
			/* Vector from cutoff center to relative velocity. */
			const Vector2 w = relativeVelocity - invTimeStep * relativePosition;
			
			const float wLength = abs(w);
			const Vector2 unitW = w / wLength;
			
			line.direction = Vector2(unitW.y(), -unitW.x());
			u = (combinedRadius * invTimeStep - wLength) * unitW;
		}
		
		line.point = other->Vvelocity_ + 0.5f * u;
				

		
		indivorcaLines_.push_back(line);
		
		size_t lineF = linearProgram2(indivorcaLines_, maxSpeed_, other->prefVelocity_, false, neighNewVelocity_); // Because I don't know my neighbors' Vpref
		
				
		if (lineF < indivorcaLines_.size()) {
			linearProgram3(indivorcaLines_, numObstLines, lineF, maxSpeed_, neighNewVelocity_);
		}
		
		
		indivorcaLines_.clear();
		
			
		if(this->id_==1015)
		{
			if(other->id_==11)
			{
			std::cout <<" Neighbor " << other->id_ << "Change is " << maxSpeed_-RVO::abs(other->prefVelocity_-neighNewVelocity_) << " Prev Vel is " << 	other->prefVelocity_ << " but new vel is " << neighNewVelocity_ << " (mine is " <<prefVelocity_ << ", pos: " <<Vposition_ << " Vvel: " <<Vvelocity_ << ")\n";
		    }
		}
				
		//neighborVelChange = maxSpeed_ - RVO::abs(other->goalprefVelocity_ - neighNewVelocity_);
		int scaling;
		if(other->prefVelocity_ * goalprefVelocity_ <0) //(other->goalprefVelocity_ * goalprefVelocity_ <0)
		{
			
		scaling=1;	
		neighborVelChange= maxSpeed_-(scaling)*RVO::abs(other->prefVelocity_-neighNewVelocity_);
			/*if(this->id_==7)
			{
				 std::cout <<" Neighbors " << other->id_ << " is OPPOSITE!!! \n";
			}*/
			
		}
		else
		{
			/*if(this->id_==7)
			{
				std::cout <<" Neighbors " << other->id_ << " is FRIENDLY!!! \n";
			}*/
			
		scaling=1;
		neighborVelChange= maxSpeed_-(scaling)*RVO::abs(other->prefVelocity_-neighNewVelocity_);
			
		}
		
		
		
		//neighborVelChange= maxSpeed_-(scaling)*RVO::abs(other->prefVelocity_-neighNewVelocity_);  // neighborVelChange  compares the max velocity with the difference between the magnitude of the pref velocity and of the 
		//'new' velocity that the neighbor would get from interacting with the agent if this difference is zero, then neighborVelChange= maxSpeed_, if the difference is max (2*maxSpeed_) neighborVelChange=-maxSpeed_
		if(neighborVelChange < -maxSpeed_)
		{
			neighborVelChange = -maxSpeed_;
			}
		
		return neighborVelChange; // ranges from -maxSpeed_ to maxSpeed_, where maxSpeed_ means no change
		
		
	}


	float Agent::VcomputeVelChange(int neighbor)
	{
		indivorcaLines_.clear();
		float neighborVelChange=-1000, neighborGoalVelChange=-1000;
		
		
	    const float invTimeHorizonObst = 1.0f / timeHorizonObst_;
		const Agent *const other = agentNeighbors_[neighbor].second; //ID of the neighbor of which I am evaluating my impact
		
		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < other->obstacleNeighbors_.size(); ++i) {
			
			
			const Obstacle *obstacle1 = other->obstacleNeighbors_[i].second;
			const Obstacle *obstacle2 = obstacle1->nextObstacle_;
			
			const Vector2 relativePosition1 = obstacle1->point_ - other->Vposition_;
			const Vector2 relativePosition2 = obstacle2->point_ - other->Vposition_;
		
			/* 
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;
			
			for (size_t j = 0; j < indivorcaLines_.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - indivorcaLines_[j].point, indivorcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - indivorcaLines_[j].point, indivorcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}
			
			if (alreadyCovered) {
				continue;
			}
			
			/* 
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added. Legs extend cut-off line when
			 * nonconvex vertex.
			 */
			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);
			
			const float radiusSq = sqr(radius_);
			
			
			
            const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
			const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
			const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);
			
			Line line;
			
			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex_) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
					indivorcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
					indivorcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line.point = Vector2(0.0f, 0.0f);
				line.direction = -obstacle1->unitDir_;
				indivorcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */
			
			Vector2 leftLegDirection, rightLegDirection;
			
			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle2 = obstacle1;
				
				const float leg1 = std::sqrt(distSq1 - radiusSq);
				
				leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle1 = obstacle2;
				
				const float leg2 = std::sqrt(distSq2 - radiusSq);
				
				leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir_;
				}
				
				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir_;
				}
			}
			
			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */
			
			const Obstacle *const leftNeighbor = obstacle1->prevObstacle_;
			
			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;
			
			if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
			
				
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir_;
				isLeftLegForeign = true;
			}
			
			if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
			
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir_;
				isRightLegForeign = true;
			}
			
			/* Compute cut-off centers. */
            const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - other->Vposition_);
            const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - other->Vposition_);
			const Vector2 cutoffVec = rightCutoff - leftCutoff;
			
			/* Project current velocity on velocity obstacle. */
			
			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((other->Vvelocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
			const float tLeft = ((other->Vvelocity_ - leftCutoff) * leftLegDirection);
			const float tRight = ((other->Vvelocity_ - rightCutoff) * rightLegDirection);
			
			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const Vector2 unitW = normalize(other->Vvelocity_ - leftCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
				indivorcaLines_.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const Vector2 unitW = normalize(other->Vvelocity_ - rightCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
				indivorcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(other->Vvelocity_ - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(other->Vvelocity_ - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(other->Vvelocity_ - (rightCutoff + tRight * rightLegDirection)));
			
			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir_;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				indivorcaLines_.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}
				
				line.direction = leftLegDirection;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				indivorcaLines_.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}
				
				line.direction = -rightLegDirection;
				line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				indivorcaLines_.push_back(line);
				continue;
			}
		}
		
		
		
		
		const size_t numObstLines =indivorcaLines_.size();
				
		const float invTimeHorizon = 1.0f / timeHorizon_;
		
		
		const Vector2 relativePosition = Vposition_ - other->Vposition_;
		
		const Vector2 relativeVelocity = other->Vvelocity_ - Vvelocity_;//prefVelocity_;//Vvelocity_;
		const float distSq = absSq(relativePosition);
		const float combinedRadius = radius_ + other->radius_;
		const float combinedRadiusSq = sqr(combinedRadius);
		
		Line line;
		Vector2 u;
		
		if (distSq > combinedRadiusSq) {
			/* No collision. */
			const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
			/* Vector from cutoff center to relative velocity. */
			const float wLengthSq = absSq(w);
			
			const float dotProduct1 = w * relativePosition;
			
			if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
				/* Project on cut-off circle. */
				const float wLength = std::sqrt(wLengthSq);
				const Vector2 unitW = w / wLength;
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				u = (combinedRadius * invTimeHorizon - wLength) * unitW;
			}
			else {
				/* Project on legs. */
				const float leg = std::sqrt(distSq - combinedRadiusSq);
				
				if (det(relativePosition, w) > 0.0f) {
					/* Project on left leg. */
					line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
				}
				else {
					/* Project on right leg. */
					line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
				}
				
				const float dotProduct2 = relativeVelocity * line.direction;
				
				u = dotProduct2 * line.direction - relativeVelocity;
			}
		}
		else {
			/* Collision. Project on cut-off circle of time timeStep. */
			const float invTimeStep = 1.0f / sim_->timeStep_;
			
			/* Vector from cutoff center to relative velocity. */
			const Vector2 w = relativeVelocity - invTimeStep * relativePosition;
			
			const float wLength = abs(w);
			const Vector2 unitW = w / wLength;
			
			line.direction = Vector2(unitW.y(), -unitW.x());
			u = (combinedRadius * invTimeStep - wLength) * unitW;
		}
		
		line.point = other->Vvelocity_ + 0.5f * u;
		//line.point = other->Vvelocity_ + 1.0f * u;

		
		indivorcaLines_.push_back(line);
		
		size_t lineF = linearProgram2(indivorcaLines_, maxSpeed_, other->prefVelocity_, false, neighNewVelocity_); // Because I don't know my neighbors' Vpref
		
				
		if (lineF < indivorcaLines_.size()) {
			linearProgram3(indivorcaLines_, numObstLines, lineF, maxSpeed_, neighNewVelocity_);
		}
		
		
		indivorcaLines_.clear();
		
			
		if(this->id_==1016)
		{
			
		std::cout <<" Neighbor " << other->id_ << "Change is " << maxSpeed_-RVO::abs(other->prefVelocity_-neighNewVelocity_) << " Prev Vel is " << 	other->prefVelocity_ << " but new vel is " << neighNewVelocity_ << " (mine is " <<prefVelocity_ << ", pos: " <<Vposition_ << " Vvel: " <<Vvelocity_ << ")\n";
		}	
			
		if(this->id_==7)
		{
			
		// std::cout <<" Neighbors " << other->id_ << "Change is " << maxSpeed_-RVO::abs(other->prefVelocity_-neighNewVelocity_) << " Prev Vel is " << 	other->prefVelocity_ << " but new vel is " << neighNewVelocity_ << " (mine is " <<prefVelocity_ << ", pos: " <<Vposition_ << " Vvel: " <<Vvelocity_ << ")\n";
		}
				
		//neighborVelChange = maxSpeed_ - RVO::abs(other->goalprefVelocity_ - neighNewVelocity_);
		int scaling;
		if(other->prefVelocity_ * goalprefVelocity_ <0) //(other->goalprefVelocity_ * goalprefVelocity_ <0)
		{
			
		scaling=1;	
		neighborVelChange= maxSpeed_-(scaling)*RVO::abs(other->prefVelocity_-neighNewVelocity_);
			/*if(this->id_==7)
			{
				 std::cout <<" Neighbors " << other->id_ << " is OPPOSITE!!! \n";
			}*/
			
		}
		else
		{
			/*if(this->id_==7)
			{
				std::cout <<" Neighbors " << other->id_ << " is FRIENDLY!!! \n";
			}*/
			
		scaling=1;
		neighborVelChange= maxSpeed_-(scaling)*RVO::abs(other->prefVelocity_-neighNewVelocity_);
			
		}
		
		//neighborVelChange= maxSpeed_-(scaling)*RVO::abs(other->prefVelocity_-neighNewVelocity_);  // neighborVelChange  compares the max velocity with the difference between the magnitude of the pref velocity and of the 
		//'new' velocity that the neighbor would get from interacting with the agent if this difference is zero, then neighborVelChange= maxSpeed_, if the difference is max (2*maxSpeed_) neighborVelChange=-maxSpeed_
		if(neighborVelChange < -maxSpeed_)
		{
			neighborVelChange = -maxSpeed_;
			}
		
		return neighborVelChange; // ranges from -maxSpeed_ to maxSpeed_, where maxSpeed_ means no change
		
		
	}
	
	
	void Agent::computeHypoVelocityAgent() 
	{
		
		orcaLines_.clear();
		
		const float invTimeHorizonObst = 1.0f / timeHorizonObst_;
		
		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {
			
			
			const Obstacle *obstacle1 = obstacleNeighbors_[i].second;
			const Obstacle *obstacle2 = obstacle1->nextObstacle_;
			
			const Vector2 relativePosition1 = obstacle1->point_ - Vposition_;
			const Vector2 relativePosition2 = obstacle2->point_ - Vposition_;
			
			
			/* 
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;
			
			for (size_t j = 0; j < orcaLines_.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}
			
			if (alreadyCovered) {
				continue;
			}
			
			/* 
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added. Legs extend cut-off line when
			 * nonconvex vertex.
			 */
			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);
			
			const float radiusSq = sqr(radius_);
			
			
			
            const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
			const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
			const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);
			
			Line line;
			
			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex_) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
					orcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
					orcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line.point = Vector2(0.0f, 0.0f);
				line.direction = -obstacle1->unitDir_;
				orcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */
			
			Vector2 leftLegDirection, rightLegDirection;
			
			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle2 = obstacle1;
				
				const float leg1 = std::sqrt(distSq1 - radiusSq);
				
				leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle1 = obstacle2;
				
				const float leg2 = std::sqrt(distSq2 - radiusSq);
				
				leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir_;
				}
				
				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir_;
				}
			}
			
			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */
			
			const Obstacle *const leftNeighbor = obstacle1->prevObstacle_;
			
			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;
			
			if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
				
				
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir_;
				isLeftLegForeign = true;
			}
			
			if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
				
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir_;
				isRightLegForeign = true;
			}
			
			/* Compute cut-off centers. */
            const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - Vposition_);
            const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - Vposition_);
			const Vector2 cutoffVec = rightCutoff - leftCutoff;
			
			/* Project current velocity on velocity obstacle. */
			
			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((Vvelocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
			const float tLeft = ((Vvelocity_ - leftCutoff) * leftLegDirection);
			const float tRight = ((Vvelocity_ - rightCutoff) * rightLegDirection);
			
			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const Vector2 unitW = normalize(Vvelocity_ - leftCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const Vector2 unitW = normalize(Vvelocity_ - rightCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(Vvelocity_ - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(Vvelocity_ - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(Vvelocity_ - (rightCutoff + tRight * rightLegDirection)));
			
			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir_;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}
				
				line.direction = leftLegDirection;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}
				
				line.direction = -rightLegDirection;
				line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
		}
		
		const size_t numObstLines = orcaLines_.size();
		
		Vector2 newGoal=goal_;
	
		const float invTimeHorizon = 1.0f / timeHorizon_;
		if(id_==15)
			{
				//std::cout <<"My goal is " << goal_ << " \n";
				
			}
	
		/* Create agent ORCA lines */
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) 
		{
			
			const Agent *const other = agentNeighbors_[i].second;
			if(id_==15)
			{
				//std::cout <<"Virtual Position of neighbor agent " << other->id_ << ": " <<other->Vposition_ << " and Vel " <<  other->Vvelocity_ <<"\n";
				
			}

			const Vector2 relativePosition = other->Vposition_-Vposition_;
			const Vector2 relativePosition2 =Vposition_ -  other->Vposition_;
			const Vector2 relativeVelocity = Vvelocity_ - other->Vvelocity_;
			const Vector2 relativeGoalN= other->goal_ - other->Vposition_;
			const float GoalDist= abs(newGoal-position_)-abs(newGoal-other->position_);
		
			const float distSq = absSq(relativePosition);
			const float combinedRadius = radius_ + other->radius_;
			const float combinedRadiusSq = sqr(combinedRadius);
			
			
			float x = acos( (relativePosition2* relativeGoalN )/(abs(relativePosition2)*abs(relativeGoalN))        );
			
			if(id_==1)
			{
				
			//std::cout << " 1	Considering agent.." << other->id_ <<" " << x<< " \n";
			}
			
			
			if(GoalDist<0) //if(x>(3.14159265358979323846f/(float)2))//(GoalDist<0)//if(GoalDist<0) //If the neighbor is farther from the goal than the agent, do not consider it
			{  // <0 means agents farther than ME to the goal --> should ignore!!!
				//float x= acosf(3);		
			if(id_==1)
			{
				
			//	std::cout << " IGNODIRNG agent.." << other->id_ <<  " goaldist " <<GoalDist << "\n\n";
			}
				continue; // These agents will be ignored
			}
			
			if(id_==1)
			{
				
			//std::cout << " 2	Considering agent.." << other->id_ <<  " goaldist " <<GoalDist << "\n\n";
			}
			
			Line line;
			Vector2 u;
			
			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition; 
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = absSq(w);
				
				const float dotProduct1 = w * relativePosition;
				
				if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vector2 unitW = w / wLength;
					
					line.direction = Vector2(unitW.y(), -unitW.x());
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				} else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);
					
					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					} else {
						/* Project on right leg. */
						line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}
					
					const float dotProduct2 = relativeVelocity * line.direction;
					
					u = dotProduct2 * line.direction - relativeVelocity;
				}
			} else {
				
				/* Collision. Project on cut-off circle of time timeStep. */
				const float invTimeStep = 1.0f / sim_->timeStep_;
				
				/* Vector from cutoff center to relative velocity. */
				const Vector2 w = relativeVelocity - invTimeStep * relativePosition;
				
				const float wLength = abs(w);
				const Vector2 unitW = w / wLength;
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				u = (combinedRadius * invTimeStep - wLength) * unitW;
			}
			
			line.point = Vvelocity_ + 0.5f * u;
			if(id_==15)
			{//std::cout<< " Pushing VIRTUAL line " << line.point <<"\n";
			}
		
			orcaLines_.push_back(line);
			
		}
		
		size_t lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);
		
		
		if (lineFail < orcaLines_.size()) {
			linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
		}
	}
	
	
	void Agent::computeHypoVelocityAgent_all()
	{
		orcaLines_.clear();
		
		const float invTimeHorizonObst = 1.0f / timeHorizonObst_;
		
		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {
			
			
			const Obstacle *obstacle1 = obstacleNeighbors_[i].second;
			const Obstacle *obstacle2 = obstacle1->nextObstacle_;
			
			const Vector2 relativePosition1 = obstacle1->point_ - Vposition_;
			const Vector2 relativePosition2 = obstacle2->point_ - Vposition_;
			
			
			/* 
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;
			
			for (size_t j = 0; j < orcaLines_.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}
			
			if (alreadyCovered) {
				continue;
			}
			
			/* 
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added. Legs extend cut-off line when
			 * nonconvex vertex.
			 */
			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);
			
			const float radiusSq = sqr(radius_);
			
			
			
            const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
			const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
			const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);
			
			Line line;
			
			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex_) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
					orcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
					orcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line.point = Vector2(0.0f, 0.0f);
				line.direction = -obstacle1->unitDir_;
				orcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */
			
			Vector2 leftLegDirection, rightLegDirection;
			
			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle2 = obstacle1;
				
				const float leg1 = std::sqrt(distSq1 - radiusSq);
				
				leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle1 = obstacle2;
				
				const float leg2 = std::sqrt(distSq2 - radiusSq);
				
				leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir_;
				}
				
				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir_;
				}
			}
			
			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */
			
			const Obstacle *const leftNeighbor = obstacle1->prevObstacle_;
			
			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;
			
			if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
				
				
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir_;
				isLeftLegForeign = true;
			}
			
			if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
				
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir_;
				isRightLegForeign = true;
			}
			
			/* Compute cut-off centers. */
            const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - Vposition_);
            const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - Vposition_);
			const Vector2 cutoffVec = rightCutoff - leftCutoff;
			
			/* Project current velocity on velocity obstacle. */
			
			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((Vvelocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
			const float tLeft = ((Vvelocity_ - leftCutoff) * leftLegDirection);
			const float tRight = ((Vvelocity_ - rightCutoff) * rightLegDirection);
			
			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const Vector2 unitW = normalize(Vvelocity_ - leftCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const Vector2 unitW = normalize(Vvelocity_ - rightCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(Vvelocity_ - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(Vvelocity_ - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(Vvelocity_ - (rightCutoff + tRight * rightLegDirection)));
			
			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir_;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}
				
				line.direction = leftLegDirection;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}
				
				line.direction = -rightLegDirection;
				line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
		}
		
		const size_t numObstLines = orcaLines_.size();
		
		Vector2 newGoal=goal_;
	
		const float invTimeHorizon = 1.0f / timeHorizon_;
		if(id_==15)
			{
				//std::cout <<"My goal is " << goal_ << " \n";
				
			}
	
		/* Create agent ORCA lines */
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) 
		{
			
			const Agent *const other = agentNeighbors_[i].second;
			if(id_==15)
			{
				//std::cout <<"Virtual Position of neighbor agent " << other->id_ << ": " <<other->Vposition_ << " and Vel " <<  other->Vvelocity_ <<"\n";
				
			}

			const Vector2 relativePosition = other->Vposition_-Vposition_;
			const Vector2 relativeVelocity = Vvelocity_ - other->Vvelocity_;
			const float GoalDist= abs(newGoal-position_)-abs(newGoal-other->position_);
		
			const float distSq = absSq(relativePosition);
			const float combinedRadius = radius_ + other->radius_;
			const float combinedRadiusSq = sqr(combinedRadius);
			
			
			
			
			Line line;
			Vector2 u;
			
			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition; 
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = absSq(w);
				
				const float dotProduct1 = w * relativePosition;
				
				if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vector2 unitW = w / wLength;
					
					line.direction = Vector2(unitW.y(), -unitW.x());
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				} else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);
					
					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					} else {
						/* Project on right leg. */
						line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}
					
					const float dotProduct2 = relativeVelocity * line.direction;
					
					u = dotProduct2 * line.direction - relativeVelocity;
				}
			} else {
				
				/* Collision. Project on cut-off circle of time timeStep. */
				const float invTimeStep = 1.0f / sim_->timeStep_;
				
				/* Vector from cutoff center to relative velocity. */
				const Vector2 w = relativeVelocity - invTimeStep * relativePosition;
				
				const float wLength = abs(w);
				const Vector2 unitW = w / wLength;
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				u = (combinedRadius * invTimeStep - wLength) * unitW;
			}
			
			line.point = Vvelocity_ + 0.5f * u;
			if(id_==15)
			{//std::cout<< " Pushing VIRTUAL line " << line.point <<"\n";
			}
		
			orcaLines_.push_back(line);
			
		}
		
		size_t lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);
		
		
		if (lineFail < orcaLines_.size()) {
			linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
		}
		
		
	}
	
	void Agent::computeHypoVelocityNeighbor()
	{
		
		orcaLines_.clear();
		
		const float invTimeHorizonObst = 1.0f / timeHorizonObst_;

		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {
			
			
			const Obstacle *obstacle1 = obstacleNeighbors_[i].second;
			const Obstacle *obstacle2 = obstacle1->nextObstacle_;
			
			const Vector2 relativePosition1 = obstacle1->point_ - Vposition_;
			const Vector2 relativePosition2 = obstacle2->point_ - Vposition_;
			// std::cout <<"Relative Position of obstacle in: "<< obstacle1->point_<< " and agent at " << Vposition_  << " so relative Pos is " <<relativePosition1 << "\n";
			// std::cout <<"OthRelative Position of obstacle in: "<< obstacle2->point_<< " and agent at " << Vposition_  << " so relative Pos is " <<relativePosition2 << "\n";
			
			//const Vector2 relativePosition1 = obstacle1->point_ - Vposition_;
			// const Vector2 relativePosition2 = obstacle2->point_ - Vposition_;
			// const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
			
			/* 
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;
			
			for (size_t j = 0; j < orcaLines_.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}
			
			if (alreadyCovered) {
				continue;
			}
			
			/* 
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added. Legs extend cut-off line when
			 * nonconvex vertex.
			 */
			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);
			
			const float radiusSq = sqr(radius_);
			
			
			
            const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
			const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
			const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);
			
			Line line;
			
			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex_) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
					orcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
					orcaLines_.push_back(line);
				}
				
				continue;
			}
			else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line.point = Vector2(0.0f, 0.0f);
				line.direction = -obstacle1->unitDir_;
				orcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */
			
			Vector2 leftLegDirection, rightLegDirection;
			
			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle2 = obstacle1;
				
				const float leg1 = std::sqrt(distSq1 - radiusSq);
				
				leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}
				
				obstacle1 = obstacle2;
				
				const float leg2 = std::sqrt(distSq2 - radiusSq);
				
				leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir_;
				}
				
				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir_;
				}
			}
			
			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */
			
			const Obstacle *const leftNeighbor = obstacle1->prevObstacle_;
			
			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;
			
			if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
				// std::cout << "1 SHOULD NOT BE HERE!!!!\n";
				
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir_;
				isLeftLegForeign = true;
			}
			
			if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
				// std::cout << "2 SHOULD NOT BE HERE!!!!\n";
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir_;
				isRightLegForeign = true;
			}
			
			/* Compute cut-off centers. */
            const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - Vposition_);
            const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - Vposition_);
			const Vector2 cutoffVec = rightCutoff - leftCutoff;
			
			/* Project current velocity on velocity obstacle. */
			
			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((Vvelocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
			const float tLeft = ((Vvelocity_ - leftCutoff) * leftLegDirection);
			const float tRight = ((Vvelocity_ - rightCutoff) * rightLegDirection);
			
			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const Vector2 unitW = normalize(Vvelocity_ - leftCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const Vector2 unitW = normalize(Vvelocity_ - rightCutoff);
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			
			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(Vvelocity_ - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(Vvelocity_ - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(Vvelocity_ - (rightCutoff + tRight * rightLegDirection)));
			
			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir_;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}
				
				line.direction = leftLegDirection;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}
				
				line.direction = -rightLegDirection;
				line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
		}
		
		const size_t numObstLines = orcaLines_.size();
	
		
		const float invTimeHorizon = 1.0f / timeHorizon_;
		
		/* Create agent ORCA lines. FOR NEIGHBOR AGENTS */
		for (size_t i = 0; i < VagentNeighbors_.size(); ++i) 
		{
			
			const Agent *const other = VagentNeighbors_[i].second;
			
	
			const Vector2 relativePosition = other->Vposition_-Vposition_;
			const Vector2 relativeVelocity = Vvelocity_ - other->Vvelocity_;
			const float distSq = absSq(relativePosition);
			const float combinedRadius = radius_ + other->radius_;
			const float combinedRadiusSq = sqr(combinedRadius);
			
			Line line;
			Vector2 u;
			
			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition; 
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = absSq(w);
				
				const float dotProduct1 = w * relativePosition;
				
				if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vector2 unitW = w / wLength;
					
					line.direction = Vector2(unitW.y(), -unitW.x());
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				} else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);
					
					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					} else {
						/* Project on right leg. */
						line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}
					
					const float dotProduct2 = relativeVelocity * line.direction;
					
					u = dotProduct2 * line.direction - relativeVelocity;
				}
			} else {
				
				/* Collision. Project on cut-off circle of time timeStep. */
				const float invTimeStep = 1.0f / sim_->timeStep_;
				
				/* Vector from cutoff center to relative velocity. */
				const Vector2 w = relativeVelocity - invTimeStep * relativePosition;
				
				const float wLength = abs(w);
				const Vector2 unitW = w / wLength;
				
				line.direction = Vector2(unitW.y(), -unitW.x());
				u = (combinedRadius * invTimeStep - wLength) * unitW;
			}
			
			line.point = Vvelocity_ + 0.5f * u;
			
			
			orcaLines_.push_back(line);
		}
		
		size_t lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);
		
		
		if (lineFail < orcaLines_.size()) {
			linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
		}
		
		
		if(this->id_==11)
		{
			//std::cout <<" T 0 Neighbor " << this->id_ << " Pref Vel is " << 	this->prefVelocity_ << " but new vel is " << newVelocity_ << " \n";
		    
		}
	}
	
	Vector2 Agent::computeHelbing(Vector2 veli){
    int i = 0, n = 0;
    Vector2 force = Vector2(0,0);
    float mass = 80;
    force = force + (veli - velocity_) * mass / AGENT_REACTION_TIME;
	
	
  
    for (size_t i = 0; i < agentNeighbors_.size(); ++i) 
		{
      const Agent *const other = agentNeighbors_[i].second;
      
	 	
      Vector2 normal_ij = position_ - other->position_;
      float distance_ij = abs(normal_ij);
	  if(distance_ij <0.000001)
	  {
		distance_ij=0.00001;  
		  
	}
      float Radii_ij = radius_ + other->radius_;
      Vector2 f_social = normal_ij * (SOCIAL_SCALING * expf((Radii_ij - distance_ij)/.08));
      
      Vector2 f_pushing = Vector2(0.f, 0.f); 
      Vector2 f_friction = Vector2(0.f, 0.f);
      
      //if (distance_ij > 2) continue;
      
      if (distance_ij < Radii_ij) {
        // pushing
        Vector2 tangent_ij( normal_ij.y(), -normal_ij.x()); 
        
        // make sure direction is opposite agent velocity
        if ((tangent_ij*velocity_) < 0.f) {
          tangent_ij = tangent_ij * -1; 
        }
        f_pushing = normal_ij * (REPULSE_SPRING_CONSTANT * (Radii_ij - distance_ij)); 
        
        f_friction = tangent_ij * (COEFF_SLIDING_FRICTION * (Radii_ij - distance_ij)) * (((other->velocity_ - velocity_)*tangent_ij) /distance_ij);
      }
      
      force = force + f_social + f_pushing + f_friction; 
      
      if(abs(f_pushing)>0)
      {
      //std::cout << " Force: " << force << " social: " <<f_social << " pushing: " << f_pushing << " friction: " << f_friction << "\n"; 	
  }
  if((id_==15)&&(other->id_==12))
  {
	   //std::cout << " social: " <<f_social  << "= normal " << normal_ij << " scaling " << SOCIAL_SCALING<< " e " << expf((Radii_ij - distance_ij)/.08) << " subt " <<(SOCIAL_SCALING * expf((Radii_ij - distance_ij)/.08)) << "\n"; 
	   
	  // char p = getchar();
	 }
	     if(distance_ij<0.001)
    {
	//std::cout << "2  Force: " << force << " dist " <<distance_ij << " between: " << id_ << " and " << other->id_ << "\n"; 	
	
	
	
	}
    
      
      //b[i]->pos
    }
	
	
	
	
    
    for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {
      
      const Obstacle *obstacle1 = obstacleNeighbors_[i].second;
      const Obstacle *obstacle2 = obstacle1->nextObstacle_;
    
   // for (int obs = 0; obs < MAX_NEAR_OBS; obs++){
     // if (a.nearObstacle[obs] == NULL) break;
      double dist;
      const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
      Vector2 nearestPt = closestPointLine2(position_, obstacle1->point_,obstacleVector, dist);//a.nearObstacle[obs]->a , a.nearObstacle[obs]->b-a.nearObstacle[obs]->a , dist);
      
      Vector2 normal_io = position_ - nearestPt;
      float dist_io = abs(normal_io);//.magnitude(); 
      normal_io = normal_io/dist_io; 
      
      Vector2 f_obs = normal_io * (SOCIAL_SCALING * exp((radius_ - dist_io) / 0.08f)); 
      Vector2 f_pushing = Vector2(0.f, 0.f);
      Vector2 f_friction = Vector2(0.f, 0.f);
      
      // pushing, friction
      if (dist_io < radius_) { // intersection has occurred
		Vector2 tangent_io( normal_io.y(), -normal_io.x()); 
        
		// make sure direction is opposite i's velocity
		if ((tangent_io*velocity_) < 0.f) {
          tangent_io = tangent_io *-1;
		}
        
		f_pushing = normal_io * (REPULSE_SPRING_CONSTANT * (radius_  - dist_io)); 
        
		// friction
		f_friction = tangent_io * COEFF_SLIDING_FRICTION * (radius_ - dist_io) * (velocity_*tangent_io);
      }
      force = force +  f_obs + f_pushing - f_friction; 
    }
    
    Vector2 acc = force / mass;
    Vector2 vel = velocity_ + acc*sim_->timeStep_;
    return vel;
  }
  


void Agent::HcomputeNewVelocity()
{
	
	  Vector2 newvel;
 
    newvel = computeHelbing(prefVelocity_);
	
	
   // printf("Col: %d\n", colliding);
  
	
   float mag = abs(newvel);
    if (mag > 1.5){
      newvel = newvel / mag;
      newvel = newvel * 1.5;
    }
    
	newVelocity_= newvel;
}



	
	Vector2 Agent::getHypoVelocity()
	{
		hypoVelocity_=newVelocity_;
		
		return hypoVelocity_;
	}
	
	void Agent::insertAgentNeighbor(const Agent *agent, float &rangeSq)
	{
		if (this != agent) {
			const float distSq = absSq(position_ - agent->position_);
			
			if (distSq < rangeSq) {
				if (agentNeighbors_.size() < maxNeighbors_) {
					agentNeighbors_.push_back(std::make_pair(distSq, agent));
				}
				
				size_t i = agentNeighbors_.size() - 1;
				
				while (i != 0 && distSq < agentNeighbors_[i - 1].first) {
					agentNeighbors_[i] = agentNeighbors_[i - 1];
					--i;
				}
				
				agentNeighbors_[i] = std::make_pair(distSq, agent);
				
				if (agentNeighbors_.size() == maxNeighbors_) {
					rangeSq = agentNeighbors_.back().first;
				}
			}
		}
	}
	
    void Agent::VinsertAgentNeighbor(const Agent *agent, float &rangeSq)
    {
        if (this != agent) {
            const float distSq = absSq(Vposition_ - agent->Vposition_);
			
            if (distSq < rangeSq) {
                if (agentNeighbors_.size() < maxNeighbors_) {
                    agentNeighbors_.push_back(std::make_pair(distSq, agent));
                }
				
                size_t i = agentNeighbors_.size() - 1;
				
                while (i != 0 && distSq < agentNeighbors_[i - 1].first) {
                    agentNeighbors_[i] = agentNeighbors_[i - 1];
                    --i;
                }
				
                agentNeighbors_[i] = std::make_pair(distSq, agent);
				
                if (agentNeighbors_.size() == maxNeighbors_) {
                    rangeSq = agentNeighbors_.back().first;
                }
            }
        }
    }
	
	void Agent::insertObstacleNeighbor(const Obstacle *obstacle, float rangeSq)
	{
		const Obstacle *const nextObstacle = obstacle->nextObstacle_;
		
		const float distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, position_);
		
		if (distSq < rangeSq) {
			obstacleNeighbors_.push_back(std::make_pair(distSq, obstacle));
			
			size_t i = obstacleNeighbors_.size() - 1;
			
			while (i != 0 && distSq < obstacleNeighbors_[i - 1].first) {
				obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
				--i;
			}
			
			obstacleNeighbors_[i] = std::make_pair(distSq, obstacle);
		}
	}
	
    void Agent::VinsertObstacleNeighbor(const Obstacle *obstacle, float rangeSq)
    {
        const Obstacle *const nextObstacle = obstacle->nextObstacle_;
		
        const float distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, Vposition_);
		
        if (distSq < rangeSq) {
            obstacleNeighbors_.push_back(std::make_pair(distSq, obstacle));
			
            size_t i = obstacleNeighbors_.size() - 1;
			
            while (i != 0 && distSq < obstacleNeighbors_[i - 1].first) {
                obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
                --i;
            }
			
            obstacleNeighbors_[i] = std::make_pair(distSq, obstacle);
        }
    }
	
	  void Agent::VinsertAgentNeighborExternal(const Agent *agent, float &rangeSq, const Agent *source, float &rangeSqsource, int maxNeighborsModel_)
    {
        if (this != agent) {
            const float distSq = absSq(Vposition_ - agent->Vposition_);
			
			const float distSqsource = absSq(source->Vposition_ - agent->Vposition_);
				
            if ((distSq < rangeSq)&&(distSqsource < rangeSqsource)) {
                if (VagentNeighbors_.size() < maxNeighborsModel_) {
					
			        VagentNeighbors_.push_back(std::make_pair(distSq, agent));
                }
				
                size_t i = VagentNeighbors_.size() - 1;
				
                while (i != 0 && distSq < VagentNeighbors_[i - 1].first) {
                    VagentNeighbors_[i] = VagentNeighbors_[i - 1];
                    --i;
                }
				
                VagentNeighbors_[i] = std::make_pair(distSq, agent);
				
                if (VagentNeighbors_.size() == maxNeighborsModel_) {
                    rangeSq = VagentNeighbors_.back().first;
                }
            }
        }
    }
  
  	void Agent::VinsertObstacleNeighborExternal(const Obstacle *obstacle, float rangeSq, Agent *source, float rangeSqsource)
    {
		
	    const Obstacle *const nextObstacle = obstacle->nextObstacle_;
		const float distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, Vposition_);
		const float distSqsource = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, source->Vposition_);
		
		
		
        if ((distSq < rangeSq)&&(sqrt( distSqsource)<= source->neighborDist_)) 
		{  
		    obstacleNeighbors_.push_back(std::make_pair(distSq, obstacle));
			
            size_t i = obstacleNeighbors_.size() - 1;
			
            while (i != 0 && distSq < obstacleNeighbors_[i - 1].first) {
                obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
                --i;
            }
			
            obstacleNeighbors_[i] = std::make_pair(distSq, obstacle);
			
        }
    }
	
	void Agent::update()
	{
		velocity_ = newVelocity_;
		
		position_ += velocity_ * sim_->timeStep_;
		
		
		Vposition_=position_;
	}
	
	
	void Agent::updateNoObstacles()
	{
		velocity_ = newVelocity_;
		position_ += velocity_ * sim_->timeStep_;
		Vposition_=position_;
	}
	void Agent::Vupdate()
	{
		
		Vvelocity_ = newVelocity_;
		
		Vposition_ += Vvelocity_ * sim_->timeStep_;
		
	}
	
	bool linearProgram1(const std::vector<Line> &lines, size_t lineNo, float radius, const Vector2 &optVelocity, bool directionOpt, Vector2 &result)
	{
		const float dotProduct = lines[lineNo].point * lines[lineNo].direction;
		const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(lines[lineNo].point);
		
		if (discriminant < 0.0f) {
			/* Max speed circle fully invalidates line lineNo. */
			return false;
		}
		
		const float sqrtDiscriminant = std::sqrt(discriminant);
		float tLeft = -dotProduct - sqrtDiscriminant;
		float tRight = -dotProduct + sqrtDiscriminant;
		
		for (size_t i = 0; i < lineNo; ++i) {
			const float denominator = det(lines[lineNo].direction, lines[i].direction);
			const float numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point);
			
			if (std::fabs(denominator) <= RVO_EPSILON) {
				/* Lines lineNo and i are (almost) parallel. */
				if (numerator < 0.0f) {
					return false;
				}
				else {
					continue;
				}
			}
			
			const float t = numerator / denominator;
			
			if (denominator >= 0.0f) {
				/* Line i bounds line lineNo on the right. */
				tRight = std::min(tRight, t);
			}
			else {
				/* Line i bounds line lineNo on the left. */
				tLeft = std::max(tLeft, t);
			}
			
			if (tLeft > tRight) {
				return false;
			}
		}
		
		if (directionOpt) {
			/* Optimize direction. */
			if (optVelocity * lines[lineNo].direction > 0.0f) {
				/* Take right extreme. */
				result = lines[lineNo].point + tRight * lines[lineNo].direction;
			}
			else {
				/* Take left extreme. */
				result = lines[lineNo].point + tLeft * lines[lineNo].direction;
			}
		}
		else {
			/* Optimize closest point. */
			const float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);
			
			if (t < tLeft) {
				result = lines[lineNo].point + tLeft * lines[lineNo].direction;
			}
			else if (t > tRight) {
				result = lines[lineNo].point + tRight * lines[lineNo].direction;
			}
			else {
				result = lines[lineNo].point + t * lines[lineNo].direction;
			}
		}
		
		return true;
	}
	
	size_t linearProgram2(const std::vector<Line> &lines, float radius, const Vector2 &optVelocity, bool directionOpt, Vector2 &result)
	{
		if (directionOpt) {
			/*
			 * Optimize direction. Note that the optimization velocity is of unit
			 * length in this case.
			 */
			result = optVelocity * radius;
			
		}
		else if (absSq(optVelocity) > sqr(radius)) {
			/* Optimize closest point and outside circle. */
			result = normalize(optVelocity) * radius;
			
		}
		else {
			/* Optimize closest point and inside circle. */
			result = optVelocity;
			
		}
		
		for (size_t i = 0; i < lines.size(); ++i) {
			
			if (det(lines[i].direction, lines[i].point - result) > 0.0f) {
				
				/* Result does not satisfy constraint i. Compute new optimal result. */
				const Vector2 tempResult = result;
				
				if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
					result = tempResult;
                    //std::cout <<" RESULT: "<<result <<" \n";
					return i;
				}
			}
		}
		
		return lines.size();
	}
	
	void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius, Vector2 &result)
	{
		float distance = 0.0f;
		
		for (size_t i = beginLine; i < lines.size(); ++i) {
			if (det(lines[i].direction, lines[i].point - result) > distance) {
				/* Result does not satisfy constraint of line i. */
				std::vector<Line> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));
				
				for (size_t j = numObstLines; j < i; ++j) {
					Line line;
					
					float determinant = det(lines[i].direction, lines[j].direction);
					
					if (std::fabs(determinant) <= RVO_EPSILON) {
						/* Line i and line j are parallel. */
						if (lines[i].direction * lines[j].direction > 0.0f) {
							/* Line i and line j point in the same direction. */
							continue;
						}
						else {
							/* Line i and line j point in opposite direction. */
							line.point = 0.5f * (lines[i].point + lines[j].point);
						}
					}
					else {
						line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
					}
					
					line.direction = normalize(lines[j].direction - lines[i].direction);
					projLines.push_back(line);
				}
				
				const Vector2 tempResult = result;
				
				if (linearProgram2(projLines, radius, Vector2(-lines[i].direction.y(), lines[i].direction.x()), true, result) < projLines.size()) {
					/* This should in principle not happen.  The result is by definition
					 * already in the feasible region of this linear program. If it fails,
					 * it is due to small floating point error, and the current result is
					 * kept.
					 */
					result = tempResult;
				}
				
				distance = det(lines[i].direction, lines[i].point - result);
			}
		}
	}
}
