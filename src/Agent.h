/*
 * Agent.h
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

#ifndef RVO_AGENT_H_
#define RVO_AGENT_H_

/**
 * \file       Agent.h
 * \brief      Contains the Agent class.
 */

#include "Definitions.h"
#include "RVOSimulator.h"

namespace RVO {
	/**
	 * \brief      Defines an agent in the simulation.
	 */
	class Agent {
	private:
		/**
		 * \brief      Constructs an agent instance.
		 * \param      sim             The simulator instance.
		 */
		explicit Agent(RVOSimulator *sim);
		
		/**
		 * \brief      Computes the neighbors of this agent.
		 */
		void computeNeighbors();
		
        //Virtual
        void VcomputeNeighbors();
		
		
		/**
		 * \brief      Computes the new velocity of this agent.
		 */
		void computeNewVelocity();
		
		/**
		 * \brief      Inserts an agent neighbor into the set of neighbors of
		 *             this agent.
		 * \param      agent           A pointer to the agent to be inserted.
		 * \param      rangeSq         The squared range around this agent.
		 */
		void insertAgentNeighbor(const Agent *agent, float &rangeSq);
		
		void computeNeighborsNoObstacles();
		
        //Virtual
        void VinsertAgentNeighbor(const Agent *agent, float &rangeSq);
		
		/**
		 * \brief      Inserts a static obstacle neighbor into the set of neighbors
		 *             of this agent.
		 * \param      obstacle        The number of the static obstacle to be
		 *                             inserted.
		 * \param      rangeSq         The squared range around this agent.
		 */
		void insertObstacleNeighbor(const Obstacle *obstacle, float rangeSq);
		
        //Virtual
        void VinsertObstacleNeighbor(const Obstacle *obstacle, float rangeSq);
		
		
		void updateNoObstacles();
		/**
		 * \brief      Updates the two-dimensional position and two-dimensional
		 *             velocity of this agent.
		 */
		void update();
		
		void Vupdate();
		
		Vector2 getHypoVelocity();
		
		void getMaxInfluenceVpref();
		void getMaxInfluenceVel();
		void computeHypoVelocityNeighbor();
		
		void computeHypoVelocityAgent();
		void computeHypoVelocityAgent_all();
		
		float computeVelChange(int neighbor);
		float VcomputeVelChange(int neighbor);
		float VcomputeVelChange2(int neighbor);
		void computeHypoVelocityModel(float timeHorizonObstModel_,float timeHorizonModel_, float maxSpeedModel_,float radiusModel_ ,Vector2 prefVelocityModel_);
		void VcomputeNeighborsModel(Agent *agentSource, float sourcetimeHorizonObstModel_, float sourcemaxSpeedModel_,float sourceradiusModel_ ,float sourceneighborDistModel_,  float timeHorizonObstModel_, float maxSpeedModel_,float radiusModel_ , int maxNeighborsModel_, float neighborDistModel_);
  
	 void VinsertAgentNeighborExternal(const Agent *agent, float &rangeSq, const Agent *source, float &rangeSqsource, int maxNeighborsModel_);
		  
		void VinsertObstacleNeighborExternal(const Obstacle *obstacle, float rangeSq, Agent *source, float rangeSqsource);
		
        void HcomputeNewVelocity();
  	    Vector2 computeHelbing(Vector2 veli);
		float magnitude(Vector2 Point1, Vector2 Point2);
		Vector2 closestPointLine2(Vector2 Point, Vector2 LineStart, Vector2 LineDir, double& dist);
		
	
		std::vector<std::pair<float, const Agent *> > agentNeighbors_;
		size_t maxNeighbors_;
		float maxSpeed_;
		float neighborDist_;
		Vector2 VnewVelocity_;
		Vector2 newVelocity_;
		Vector2 neighNewVelocity_;
		std::vector<std::pair<float, const Obstacle *> > obstacleNeighbors_;
		std::vector<Line> orcaLines_;
		std::vector<Line> indivorcaLines_;
		Vector2 position_;
		Vector2 goal_;
		Vector2 prefVelocity_;
		Vector2 goalprefVelocity_;
		Vector2 Vposition_;
		float radius_;
		RVOSimulator *sim_;
		float timeHorizon_;
		float timeHorizonObst_;
		Vector2 velocity_;
		Vector2 initvelocity_[20];
		Vector2 initposition_[320];
		Vector2 initprefvelocity_[20];
		Vector2 hypoVelocity_;
		Vector2 vOpt_;
		Vector2 Vposition[15];
		float NeighAngle[10];
		Vector2 Vvelocity_;
		Vector2 VprefVelocity_;
		Vector2 indiVel[15];
		float intentionSim[1500];
		float observedSim[1500];
		float externalSim[1500];

		std::vector<std::pair<float, const Agent *> > VagentNeighbors_;
		
		bool inGoal_;
		float avgNumNeigh;
		int numInteractions;
		int MostSimilar_;
		float physicalradius_;
		int MostConstrained_;
		int LeastConstrained_;
		int constraint_[15];
		float NonOrderConstraint_[15];
		int similarity_[15];
		
		float VprefDiff[15];
		float Vdiff[15];
		float VelSimPref[15];
		float VelSimVel[15];
		
			float SimComp1_t[10][150]; // here 10 is the number of actions,  50 is the max time horizon, should be adjusted if using a higher value.
		float SimComp2_t[10][150]; // here 50 is the max time horizon, should be adjusted if using a higher value.
		
		Vector2 SimVelNeigh_t[300][10][150]; //[num_neighbors][number_actions][time_horizon]
		Vector2 SimPosNeigh_t[300][10][150]; //[num_neighbors][number_actions][time_horizon]
		
		float neighSim[15];
		//float maxNeighSim[15];
		float avgNeighInfluence[1500];
		int  numNeighInfluence[1500];
		
		Vector2 predVelocity_[300];
		float predReward;
		Vector2 predVelocityNeigh_[15][300];
		
		Vector2 newVelocity0_;
		Vector2 newVelocity1_;
		Vector2 newVelocity2_;
		Vector2 newVelocity3_;
		Vector2 newVelocity4_;
		float cnavpoliteness_[20];
		float alanpoliteness_[20];
		float goalprog_[20];
		
	
		
		Vector2 prefVelocity0_;
		Vector2 prefVelocity1_;
		Vector2 prefVelocity2_;
		Vector2 prefVelocity3_;
		Vector2 prefVelocity4_;
		size_t id_;
		
		
		Vector2 velocity0_;
		Vector2 velocity1_;
		Vector2 velocity2_;
		Vector2 velocity3_;
		Vector2 velocity4_;
		friend class KdTree;
		friend class RVOSimulator;
	};
	
	/**
	 * \relates    Agent
	 * \brief      Solves a one-dimensional linear program on a specified line
	 *             subject to linear constraints defined by lines and a circular
	 *             constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      lineNo        The specified line constraint.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     True if successful.
	 */
	bool linearProgram1(const std::vector<Line> &lines, size_t lineNo,
						float radius, const Vector2 &optVelocity,
						bool directionOpt, Vector2 &result);
	
	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             constraints defined by lines and a circular constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     The number of the line it fails on, and the number of lines if successful.
	 */
	size_t linearProgram2(const std::vector<Line> &lines, float radius,
						  const Vector2 &optVelocity, bool directionOpt,
						  Vector2 &result);
	
	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             constraints defined by lines and a circular constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      numObstLines  Count of obstacle lines.
	 * \param      beginLine     The line on which the 2-d linear program failed.
	 * \param      radius        The radius of the circular constraint.
	 * \param      result        A reference to the result of the linear program.
	 */
	void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine,
						float radius, Vector2 &result);
}

#endif /* RVO_AGENT_H_ */
