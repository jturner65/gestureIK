/*
* Copyright (c) 2015, Georgia Tech Research Corporation
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

#ifndef APPS_GESTURE_GestIKParams_H_
#define APPS_GESTURE_GestIKParams_H_

#include <vector>
#include <string>

#include <iostream>

#include <Eigen/Dense>
#include <tinyxml2.h>
#include "dart/utils/Parser.h"
#include <boost/algorithm/string.hpp>

using namespace std;
namespace gestureIKApp {
	//any necessary hyperparams needed to be loaded from XML file
	class GestIKParams {
	public:
		GestIKParams();
		virtual ~GestIKParams();

		vector<double> accumulateVals();
		void distributeVals(vector<double>& vals);
		void setDefaultVals();
		void setCurrentValsAsDefault();
		void setParamValFromXMLStr(const std::string& _name, const std::string& s);

		void resetValues();

		friend std::ostream& operator<<(std::ostream& out, GestIKParams& GestIK);

	public:	//variables
		vector<double> defaultVals;

		//data collection parameters
		//base value of # of trajectories to capture for training and testing data
		int dataCapNumExamples;
		//threshold between testing and training data as % of total data captured
		double dataCapTestTrainThresh;
		
		//IK solver and skel configuration parameters
		//% of full reach to set up imaginary writing plane
		double IK_reachPct;
		//radius of drawn circle
		double IK_drawRad;
		//per frame iterations of IK solver
		unsigned int IK_solveIters;
		//learning rate for IK solver
		double IK_alpha;
		//max allowed IK sq error 
		double IK_maxSqError;
		//scale of circular projection of trajectory on elbow plane compared to pointer plane
		double IK_elbowScale;
		//multiplier to velocity for "fast" symbols (determined randomly)
		double IK_fastTrajMult;
		//min allowed traj length for multi-point trajectories
		double trajLenThresh;
		//# of iterations of tuck/untuck/subdivide
		int trajNumReps;
		//multiplier of trajectory distance per frame for trajectory point gen
		double trajDistMult;
		//desired target velocity of motion, in units of IK space at ptr finger
		double trajDesiredVel;
		//amount of overshoot to use for 4-pt neville (for building connecting trajs)
		double trajNev4OvPct;
		//int width/height of windows
		int win_Width, win_Height;
		//# of letters in alphabet to process (first n) - 26 for whole alphabet, 1 for just a, etc
		int numLetters;
		//initial zoom
		float origZoom;
		//background colors
		double bkgR, bkgG, bkgB, bkgA;

		std::vector<bool> flags;									//various boolean flags used to drive GestIK
		static const int 
			IDX_useLeftHand	= 0;						//whether or not to use left hand to draw


		static const int numFlags = 1;
	};

}//namespace gestureIKApp 
#endif  // APPS_GESTURE_GestIKParams_H_
