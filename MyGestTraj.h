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

#ifndef APPS_GESTURE_MYGESTTRAJ_H_
#define APPS_GESTURE_MYGESTTRAJ_H_

#include <Eigen/StdVector>
#include <memory>
#include <Eigen/Dense>
#include "apps/gestureIK/GestGlbls.h"
#include "apps/gestureIK/MyGestSymbol.h"

#include "dart/dart.h"

namespace dart {
	namespace renderer {
		class RenderInterface;
	}  
} 

namespace gestureIKApp {
	class IKSolver;
	class MyGestSymbol;

	class MyGestTraj { 
	public:

		MyGestTraj(const std::string& _fname,std::shared_ptr<gestureIKApp::MyGestSymbol> _p, int _num);
		virtual ~MyGestTraj();

		//load this trajectory's csv trajectory file - expected to be x,y,millis or x,y,vx,vy if useVel
		void readTrajFile(bool useVel);

		//set tracked marker trajectory points used by solver
		bool setTrkMrkrAndSolve(double lenFromStTraj);
		//build IK trajectory from matalb data
		void buildTrajFromData(bool init, eignVecTyp& _Initpts);

		//convert image-space source trajectory for pointer to properly scaled traj for IK
		eignVecTyp convSrcTrajToGestTraj(double sclAmt);

		//convert pointer location to elbow space location
		Eigen::Vector3d convPtrToElbow(const Eigen::Ref<const Eigen::Vector3d>& _pt);
		//copy the source info from passed trajectory
		void copySrcInfo(std::shared_ptr<gestureIKApp::MyGestTraj> _src, double newSclAmtZ, double newSclAmtY);

		inline eignVecTyp getTrajFrame(int idx) {	return trajTargets[idx];		}

		inline eignVecTyp getFirstFrame() { return trajTargets[0]; }
		inline eignVecTyp getLastFrame() { return trajTargets[trajTargets.size()-1]; }

		//build full trajectories
		void buildFullTraj(); 

		//called by symbol - calculate src data traj disp vectors (vectors from avg location to point location in source data
		void calcSrcTrajDispVecs(const Eigen::Ref<const Eigen::Vector3d>& _avgLoc);

		//calculate length of trajectory at given idx (corresponding to which marker this trajectory is being applied to)
		double calcTrajLength(eignVecTyp& _pts);
		//double calcTrajLength(int idx);
		eignVecTyp procPts(int _typ, eignVecTyp& _pts, double val, bool wrap);
		eignVecTyp equiDist(eignVecTyp& _pts, double _len, std::vector<double>& ptsDistFromSt);
		eignVecTyp resample(eignVecTyp& _pts, double _len, std::vector<double>& _ptsDistFromSt, int numPts, bool wrap);
		std::vector<double> calcDistsFromStart(eignVecTyp& _pts);
		Eigen::Vector3d at(double t, double _ttllen, eignVecTyp& _pts, std::vector<double>& _distAtEachPt);
		void processPts(int numPts);
		void setPts(eignVecTyp& tmp);
		//whether or not to use this trajectory - currently only ever set false if length of traj is >0 but < some small threshold.  denotes traj noise
		inline bool useTraj() { return flags[useTrajIDX]; }
		//whether or not this trajectory is a connecting trajectory
		inline bool isConnTraj() { return flags[connTrajIDX]; }

		//set flags to show all letter trajectories (for debug display of traj distribution)
		inline void setShowAllTrajs(bool val) { flags[showAllTrajsIDX] = val; }

		//draw this trajectory
		void drawDebugTraj(dart::renderer::RenderInterface* mRI, const Eigen::Ref<const Eigen::Vector3d>& clr);

		//set solver for this trajectory
		//void setSolver(std::shared_ptr<gestureIKApp::IKSolver> _slv) {		IKSolve = _slv;		} 

		friend std::ostream& operator<<(std::ostream& out, MyGestTraj& traj);


	public :	//variables
		std::shared_ptr<gestureIKApp::IKSolver> IKSolve;			//ref to ik solver
		std::shared_ptr<gestureIKApp::MyGestSymbol> parentSymbol;	//owning symbol of this trajectory

		//used for mapping
		Eigen::Vector3d avgLoc;				//average location of all trajectories in the symbol this traj belongs to	- !!in matlab space!!	
		Eigen::Vector3d ctrPoint;				//center point of traj	- !!in IK space!!	
			
			//length of trajectory
		double trajLen,		
			//longest source displacement length (to speed up scale calculation)
			lenMaxSrcDisp,					
			//space between points in final trajectory
			perPtSpace;						

		std::string filename,				//filename source for this trajectory
					name;					//trajectory name (contains ltr name, symbol #, ltr #
		eignVecVecTyp trajTargets;			//vector(all frames) of vector per target point of trajectory values - each value is a Eigen::Vector3d 

		eignVecTyp srcTrajData,					//source trajectory points in matlab frame - need to be transformed
			srcTrajVelData,						//source trajectory velocity in matlab image frame - only used if present
			srcTrajDispVecs,				//source trajectory displacement vectors from avg loc - scale by scale amt and add to IKSolver->drawCrclCtr to find new points
			convTrajPts,				//converted trajectory points
			debugTrajPts;				//for debugging only

		std::vector<double> srtTrajTiming, trajPtDistFromSt;		//timing of source trajectory, dists to each point from start of arc

		std::vector<bool> flags;					//state flags of trajectory
		static const unsigned int
			debugIDX = 0,				//debug mode
			testLtrQualIDX = 1,			//test all symbols to make sure no abberrant trajectories
			loadedIDX = 2,				//trajectory data is loaded or not (srcTrajData will not grow)
			builtIDX = 3,				//trajectory targets data for all tracked markers has been derived
			useTrajIDX = 4,				//whether or not to use this trajectory component - if size is very small but not 0 might mean this trajectory is something separate and noisy
			closeTrajIDX = 5,			//whether trajectory is closed or not
			connTrajIDX = 6,			//this is a connecting trajectory (generated between two sourced actual trajectories)
			showAllTrajsIDX = 7;

		static const unsigned int numFlags = 8;

	};	
}

#endif 