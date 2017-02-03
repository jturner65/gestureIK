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

#ifndef APPS_GESTURE_IKSolver_H_
#define APPS_GESTURE_IKSolver_H_

#include <vector>
#include <Eigen/Dense>
#include "dart/dart.h"
#include "apps/gestureIK/GestGlbls.h"
#include "apps/gestureIK/GestIKParams.h"
#include "apps/gestureIK/MyGestTraj.h"
#include "apps/gestureIK/GestIKParser.h"


namespace dart {
	namespace renderer {
		class RenderInterface;
	}  // namespace renderer
	namespace dynamics {
		class MeshShape;
	} // namespace dynamics
} // namespace dart

namespace gestureIKApp {
	class MyGestTraj;
	//class describing structure holding fixed and tracked markers on skeleton
	class trackedMarker {
	public:
		//tracked markers are markers that are solved for in IK solver, fixed means they use the original positions as their targets
		trackedMarker(dart::dynamics::Marker* _m) :	ID(_m->getID()), m(_m), isFixed(false), origPos(_m->getWorldPosition()), tarPos(0, 0, 0) {}
		virtual ~trackedMarker() {}
		//set obj target position to current or original location and set to be fixed
		void fixToLoc(bool useOrig) { isFixed = true; tarPos << (useOrig ? origPos : m->getWorldPosition());}
		//release fixed object
		inline void release(bool _fix) { isFixed = false; }
		//set target position if not a fixed cnstrnt/marker
		inline void setTarPos(const Eigen::Ref<const Eigen::Vector3d>& _tar) {	if (!isFixed) { tarPos << _tar; } }
		//get if fixed
		inline bool IsFixed() {	return isFixed; }
		//get target position
		inline Eigen::Vector3d getTarPos() { return tarPos; }
		//get constraint value - curr position-tar position
		inline Eigen::Vector3d getCnstrntVec() { return (m->getWorldPosition() - tarPos); }
		//draw this point's target position
		void draw(dart::renderer::RenderInterface* mRI);

		friend std::ostream& operator<<(std::ostream& out, trackedMarker& tmrk);

		//marker's id
		unsigned int ID;
		//ref to marker
		dart::dynamics::Marker* m;

	protected ://variables
		//whether or not marker is fixed
		bool isFixed;
		//original position and current target position
		Eigen::Vector3d origPos, tarPos;
	};


	class IKSolver{

	public:
		IKSolver(dart::dynamics::SkeletonPtr _skel);
		virtual ~IKSolver();

		//load/reload all xml-based sim parameters
		void loadIKParams();
		
		//set tracked marker positions - call this or something like it before solve
		void setTrkMrkrs(eignVecTyp& _tarPos);

		void solve(eignVecTyp& _tarPos);

		void solve();
		//get error in current pose between markers and targets
		double getPoseError();
		//set sample trajectory pointer and elbow centers and normal vectors of planes drawn on (also used for defaults)
		void setSampleCenters();

		//get gauss random double with mean mu and std = std - put here so that accessible in all classes
		inline double getRandDbl(double mu, double std = 1.0) {
			double val = (*normDist)(mtrn_gen);
			val *= std;// (std * std); //transform val (1 stdev, 0 mean) to new variable with std stdev and mu mean using general theory of probability
			val += mu;
			return val;
		}
		//get uniform random double between min and max - put here so that accessible in all classes
		inline double getUniRandDbl(double min, double max) {
			if (min == max) { return min; }
			if (min > max) {double t = min; min = max; max = t;	}//if wrong order, swap
			double val = (*uniDist)(mtrn_gen);//0-1
			val *= (max - min);		//0-(max-min)
			val += min;				//min-max
			return val;
		}

		//get gauss random double with mean mu and std = std - put here so that accessible in all classes
		inline Eigen::Vector3d getRandVec(const Eigen::Ref<const Eigen::Vector3d>& mu, const Eigen::Ref<const Eigen::Vector3d>& stdVec) {
			Eigen::Vector3d val(0, 0, 0);
			val<< (*normDist)(mtrn_gen), (*normDist)(mtrn_gen), (*normDist)(mtrn_gen);
			val(0) *= stdVec(0);//(stdVec(0) * stdVec(0));//vector to shape distribution
			val(1) *= stdVec(1);//(stdVec(1) * stdVec(1));
			val(2) *= stdVec(2);//(stdVec(2) * stdVec(2));
			val += mu;
			return val;
		}
		//get gauss random double with mean mu and std = std - put here so that accessible in all classes
		inline Eigen::Vector3d getUniRandVec(const Eigen::Ref<const Eigen::Vector3d>& min, const Eigen::Ref<const Eigen::Vector3d>& max) {
			return Eigen::Vector3d(getUniRandDbl(min(0), max(0)), getUniRandDbl(min(1), max(1)), getUniRandDbl(min(2), max(2)));
		}


		void drawTrkMrkrs(dart::renderer::RenderInterface* mRI, bool onlyTraj);

		inline dart::dynamics::SkeletonPtr getSkel() const { return skelPtr; }

		//set trajectory for current solver
		inline void setTrajectory(std::shared_ptr<gestureIKApp::MyGestTraj> _traj) { trajectory = _traj; }

		inline Eigen::Vector3d getPtrPos() { return  skelPtr->getMarker("ptrFinger_r")->getWorldPosition(); }
		inline Eigen::Vector3d getElbowPos() { return  skelPtr->getMarker("ptrElbow_r")->getWorldPosition(); }

		inline Eigen::Vector3d getPtrCenter() { return  skelPtr->getMarker("ptrElbow_r")->getWorldPosition(); }


		std::shared_ptr<trkMrkMap> trkMarkers;										//all tracked markers in this skeleton

		double reach,		//approx distance skel can reach without changing posture too much(i.e. twisting at waist); (rest dist from scap to ptrFinger_r)
			bicepLen,	//length from writing shoulder mrkr to writing elbow marker
			frArmLen,	//length from elbow marker to wrist marker
			handLen;	//length from wrist marker to ptrFinger_r marker
		
		//center of drawn circle (some distance forward from drawing arm's shoulder); center of circle of drawing elbow; 

		Eigen::Vector3d drawCrclCtr, drawElbowCtr, tstRShldrSt, elbowShldrNormal;

		std::shared_ptr<gestureIKApp::GestIKParams> params;
	protected:
		void createMarkers();

		Eigen::VectorXd calcGradient();

		dart::dynamics::SkeletonPtr skelPtr;

		//trajectory this solver is currently solving for
		std::shared_ptr<gestureIKApp::MyGestTraj> trajectory;
		std::map<std::string, double> mrkrWts;

		unsigned int numDofs;// , numCnstrnts;
		std::shared_ptr<std::normal_distribution<double> > normDist;
		std::shared_ptr<std::uniform_real_distribution<double> > uniDist; //set to 0-1

		Eigen::VectorXd initPose, newPose, lastNewPose, grads, mC;
		Eigen::MatrixXd mJ;
	};

}//namespace gestureIKApp 
#endif  // APPS_GESTURE_CONTROLLER_H_
