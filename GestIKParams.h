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
#include "apps/gestureIK/GestGlbls.h"
#include <Eigen/Dense>
#include <tinyxml2.h>
#include "dart/utils/Parser.h"
#include <boost/algorithm/string.hpp>


namespace gestureIKApp {
	//any necessary hyperparams needed to be loaded from XML file
	class GestIKParams {
	public:
		GestIKParams();
		virtual ~GestIKParams();

		std::vector<double> accumulateVals();
		void distributeVals(std::vector<double>& vals);
		void setDefaultVals();
		void setCurrentValsAsDefault();
		void setParamValFromXMLStr(const std::string& _name, const std::string& s);

		//whether or not to add the possibility of using the left hand to draw to random letter gen
		inline bool useLeftHand() { return flags[IDX_useLeftHand]; }
		//whether or not to generate randomized versions of loaded letters - note numTotSymPerLtr still needs to be specified to be greater than # of file-based samples to get any random samples
		inline bool genRandLtrs() { return flags[IDX_genRandLtrs]; }
		//whether or not to make img sequences of the non-random-generated (i.e. file based) letters.  this will speed up processing if only added more letters to test/train
		inline bool regenNotAppend() { return flags[IDX_regenNotAppend]; }
		//whether or not to change colors of trajectories for debug display
		inline bool chgTrajClrs() { return flags[IDX_chgTrajDbgClrs]; }
		//whether or not to force training data to be 16 frames long
		inline bool limitTo16Frames() { return (dataType == FIXED_16); }
		//whether or not to use fixed global velocity for all trajectories
		inline bool useFixedGlblVel() { return (dataType == CONST_VEL); }
		void resetValues();
		//set current date as date-based file name offset
		void setDateFNameOffset();
		//get the string that will be used to offset a particular generation of letters - this will add some date-based string to each generated dataset
		std::string getGenOffsetStr() { return dateFNameOffset; }

		friend std::ostream& operator<<(std::ostream& out, GestIKParams& GestIK);

	public:	//variables
		std::vector<double> defaultVals;

		//data collection parameters
		//base value of # of trajectories to capture for training and testing data ONLY SAMPLE SYMBOLS - use numTotSymPerLtr for letter trajectories
		int dataCapNumExamples;
		//threshold between testing and training data as % of total data captured
		//REMOVED : handle partitioning in python script
		//double dataCapTestTrainThresh;
		
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
		//how much to bring center down in y direction (needs to be negative)
		double IK_ctrYOffset;

		//min allowed traj length for multi-point trajectories
		double trajLenThresh;
		//std of randomized trajectory scales
		double trajRandCtrStd;
		//X,Y,Z scaling amounts for std dev
		double trajRandCtrStdScale_X, trajRandCtrStdScale_Y, trajRandCtrStdScale_Z;

		//std of randomized trajectory center
		double trajRandSclStd;
		//# of iterations of tuck/untuck/subdivide for trajectory smoothing
		int trajNumReps;
		//multiplier of trajectory distance per frame for trajectory point gen
		double trajDistMult;
		//desired target velocity of motion, in units of IK space at ptr finger
		double trajDesiredVel;
		//amount of overshoot to use for 4-pt neville (for building connecting trajs)
		double trajNev4OvPct;

		//randomization control
		//random camera orientation theta
		double rnd_camThet;
		//int width/height of windows
		int win_Width, win_Height;

		//# of letters in alphabet to process (first n) - 26 for whole alphabet, 1 for just a, etc
		int numLetters;
		//# of total symbols of a particular letter we're generating - if < # of file-based examples, is ignored, if > then we generate random versions of the file based letters to make up diff
		int numTotSymPerLtr;

		//ltr idx to begin saving (a==0, b==1, etc) (>25 forced to 25)
		int ltrIdxStSave;
		//size of window for LSTM in training - make clips fixed to multples of this length to prevent image paddings
		int fixedClipLen;
		//offset in naming so that multiple sets can be generated.
		int clipCountOffset;
		//initial zoom
		float origZoom;
		//letter type to be generated - either constant velocity, 16-frame long train/mult8 test, or mult8 train/mult8 test
		DataType dataType;

		//background colors
		double bkgR, bkgG, bkgB, bkgA;

		//not saved as default or read from file : string to hold some value that denotes current date.   used to add to generated file names to make unique
		std::string dateFNameOffset;

	private :
		std::vector<bool> flags;									//various boolean flags used to drive GestIK
		static const int
			//whether or not to add the possibility of using the left hand to draw to random letter gen
			IDX_useLeftHand = 0,
			//whether or not to generate randomized versions of loaded letters - note numTotSymPerLtr still needs to be specified to be greater than # of file-based samples to get any random samples
			IDX_genRandLtrs = 1,
			//whether or not to regenerate the original non-randomized (i.e. file based) letters.  this will speed up processing if only added more letters to test/train
			IDX_regenNotAppend = 2,
			//whether or not to change colors per trajectory for debug display
			IDX_chgTrajDbgClrs = 3;

		static const int numFlags = 4;
	};

}//namespace gestureIKApp 
#endif  // APPS_GESTURE_GestIKParams_H_
