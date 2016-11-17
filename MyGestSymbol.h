/*
* Copyright (c) 2016, Georgia Tech Research Corporation
* All rights reserved.
*
* Author(s): John Turner <jturner65@gatech.edu>
*
* Georgia Tech Graphics Lab and Humanoid Robotics Lab
*
* Directed by Prof. C. Karen Liu and Prof. Mike Stilman
* <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
*
* This file is provided under the following "BSD-style" License:
*   Redistribution and use in source and binary forms, with or
*   without modification, are permitted provided that the following
*   conditions are met:
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
*   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
*   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
*   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
*   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef APPS_GESTURE_MYGESTSYMBOL_H_
#define APPS_GESTURE_MYGESTSYMBOL_H_

#include <Eigen/StdVector>
#include <memory>
#include <Eigen/Dense>
#include "apps/gestureIK/GestGlbls.h"
#include "apps/gestureIK/MyGestTraj.h"

#include "dart/dart.h"
namespace dart {
	namespace renderer {
		class RenderInterface;
	}  // namespace renderer
} // namespace dart

	/* 
		this class represents an instance of a letter.  it will consist of a collection of trajectories
		it is responsible for determining which component traj should be used and for linking components together 
		which may include making new non-specified trajectories to link existing components whose endpoints don't meet in space

		omniglot is collected turk data
		symbols in BPL omniglot data have x,y data and time in milliseconds, individually recorded for each component trajectory
		alphabet is #22, latin, in dataset
		
		each symbol will be recorded in xml file with # of component trajectories, names of each trajectory file.
		each trajectory file will be csv file of location of primary trajectory x,y points and velocities
	*/
namespace gestureIKApp {
	class IKSolver;
	class MyGestTraj;

	//collection of 1 or more trajectories making up a symbol
	class MyGestSymbol {
	public:
		MyGestSymbol(const std::string& name, int srcIDX);
		virtual ~MyGestSymbol();

		//draw trajectories of this symbol - if set change color for each trajectory
		void drawTrajs(dart::renderer::RenderInterface* mRI);

		//call when symbol first chosen to be IKed to
		void initSymbolIK();
		//solve IK for this symbol's current trajectory - returns true if finished
		bool solve();

		//set flags of all subordinate symbols
		void setFlags(int idx, bool val);
		//set solver for this trajectory
		void setSolver(std::shared_ptr<gestureIKApp::IKSolver> _slv);

		//buld a randomized version of the passed symbol
		bool buildRandomSymbol(std::shared_ptr<MyGestSymbol> _base, std::shared_ptr<MyGestSymbol> _thisSP, bool isFast);
		//load this symbol's raw trajectories and build MyGestTrajs for each trajectory - build at symbol level, not trajectory level
		void buildTrajsFromFile(std::vector<std::string>& trajFileNames, std::shared_ptr<MyGestSymbol> _this);
		//find average x-y location of trajectory points in component trajectories of this symbol, and closest and furthest points from average. - these will be used to map to "drawing plane" in ik sim world frame 
		void calcTransformPts();
		//find length of all trajectories
		void calcAllTrajsLen();
		//process trajectories and build linking trajectories to connect disjoint trajectories
		void buildTrajComponents();

		//set symbol trajectory pointer and elbow centers and normal vectors of planes drawn on - planeNorm must be specified first
		void setSymbolCenters(const Eigen::Ref<const Eigen::Vector3d>& ctPt);

		//generate transition traj between symbols using neville
		std::shared_ptr<MyGestTraj> genConnectTraj(const Eigen::Ref<const Eigen::Vector3d>& ctPt, const Eigen::Ref<const Eigen::Vector3d>& traj1End, const Eigen::Ref<const Eigen::Vector3d>& traj2St);
		std::shared_ptr<MyGestTraj> genInterSymbolTraj(const Eigen::Ref<const Eigen::Vector3d>& ctPt, const Eigen::Ref<const Eigen::Vector3d>& sym1End, const Eigen::Ref<const Eigen::Vector3d>& midpt, const Eigen::Ref<const Eigen::Vector3d>& sym2St);
		
		friend std::ostream& operator<<(std::ostream& out, MyGestSymbol& sym);

	public :	//variables
		std::shared_ptr<gestureIKApp::IKSolver> IKSolve;						//ref to ik solver
		std::shared_ptr<MyGestSymbol> _self;									//ref to shared ptr to self, to be handed off to trajectories

		unsigned int curTraj,													//idx of current trajectory being processed
			curFrame,															//current frame of this letter being processed
			srcSymbolIDX;														//idx in owning letter symbols vector of source symbol for this symbol, or it's own idx if from a file

		double allTrajsLen,													//length of all trajectories of this letter
			trajVel,														//avg velocity to finish trajectory
			sclAmt,															//scale amount for this symbol to scale from matlab space to IK space
			curTrajDist;														//current position along length of trajectory

		std::vector<std::shared_ptr<gestureIKApp::MyGestTraj> > trajectories;	//component trajectories making up this symbol
		std::vector<double>trajLens;											//array of total length of trajectory at a location, including piece at IDX
		std::string name;														//symbol name - letter + # of symbol

		std::vector<bool> flags;					//state flags of trajectory
		static const unsigned int 
			debugIDX = 0,			//debug mode
			diffClrIDX = 1,			//use different colors for each component trajectory of this symbol
			drawConnTrajIDX = 2,	//draw the connecting trajectories between two data trajectories	
			randCircleIDX = 3,		//if true, randomize the center, radius and plane normal of the bounding circle of this letter
			isFastDrawnIDX = 4;		//this symbol is drawn quickly - flip orientation of elbow plane normal, 1.5x base drawing speed
		static const unsigned int numFlags = 5;

		Eigen::Vector3d avgLoc;													//average location of symbol data in matlab space
		//	maxLoc;																//disp vector of point furthest from average location of all points in this symbol

		Eigen::Vector3d	ptrCtrPt,												//center point of circle the transformed version of this symbol is inscribed within 
			elbowCtrPt,															//center point of circle the transformed elbow traj of this symbol is incribed within
			ptrPlaneNorm,														//normal of plane of circle this symbol is inscribed upon
			elbowPlaneNorm;														//normal of plane of elbow circle 
		double circleRad;														//radius of circle this symbol is inscribed within

	};
} // namespace gestureIKApp

#endif // #ifndef APPS_GESTURE_MYGESTSYMBOL_H_
