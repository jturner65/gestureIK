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
	//simple struct holding name strings and vector of vals for offset for marker info read in from xml
	class markerXMLData {
	public:
		markerXMLData(const std::string& bName, const std::string& offsetStr, const std::string& mName) : offset(), bnodeName(bName), markerName(mName) {
			std::istringstream iss(offsetStr);
			std::vector<double> offsetVals = std::vector<double>{std::istream_iterator<double>(iss),std::istream_iterator<double>()};
			offset << offsetVals[0], offsetVals[1], offsetVals[2];
		}
		virtual ~markerXMLData() {}
		
		friend std::ostream& operator<<(std::ostream& out, markerXMLData& m) {
			out << "BNodeName : " << m.bnodeName << "\t MarkerName : " << m.markerName << " \tOffset : (" << m.offset(0) << ", "<< m.offset(1) << ", "<< m.offset(2) << ")\n";
			return out;
		}
		//vars
		Eigen::Vector3d offset;
		std::string bnodeName;
		std::string markerName;
	};

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
		//read in marker values/locations
		void setMarkerLocVals(const std::string& bName, const std::string& offsetStr, const std::string& mName);

		//whether or not to add the possibility of using the left hand to draw to random letter gen - needs to move active shoulder as camera target
		inline bool useLeftHand() { return flags[IDX_useLeftHand]; } 
		//whether or not to use motion blur to render frames
		inline bool useMotionBlur() {		return flags[IDX_useMotionBlur]; }
		//whether or not to change colors of trajectories for debug display
		inline bool chgTrajClrs() {		return flags[IDX_chgTrajDbgClrs]; }
		//random skel or camera checks
		//randomize camera orientation
		inline bool rndCamOrient() {	return flags[IDX_rndCamOrient]; }
		//random camera translation
		inline bool rndCamLoc() {		return flags[IDX_rndCamLoc]; }
		//random head width/height (ellispoid)
		inline bool rndHeadDims() {		return flags[IDX_rndHeadDims]; }
		//random head color (avoid colors close to gray)
		inline bool rndHeadClr() {		return flags[IDX_rndHeadClr]; }
		//random hand shape - ellipsoid or rectangular
		inline bool rndHandShape() {	return flags[IDX_rndHandShape]; }
		//random hand width/depth/length
		inline bool rndHandDims() {		return flags[IDX_rndHandDims]; }
		//random hand color
		inline bool rndHandClr() { return flags[IDX_rndHandClr]; }
		//save hand and elbow com-related values
		inline bool saveHandCOMVals() { return flags[IDX_saveHandCOMVals]; }
		//save screenshots - true if either not saving hand com vals or if IDX_alwaysSaveImgs is specified as true
		inline bool saveScreenShots() { return (!flags[IDX_saveHandCOMVals] || flags[IDX_alwaysSaveImgs]); }

		//whether or not to force training data to be 16 frames long
		inline bool limitTo16Frames() { return (dataType == FIXED_16); }
		//whether or not to use fixed global velocity for all trajectories
		inline bool useFixedGlblVel() { return (dataType == CONST_VEL); }
		void resetValues();
		//set current date as date-based file name offset
		void setDateFNameOffset();

		//returns base output directory, if used, otherwise default relative path set in GestGlbls - must have '/' as final char
		inline std::string getOutputBaseDir() { 	return (flags[IDX_useOutputDir] ? baseOutDir : framesFilePath);	}
		//returns directory letters are stored in
		std::string getDataOutputDir(bool isCSV);

		friend std::ostream& operator<<(std::ostream& out, GestIKParams& GestIK);

	public:	//variables
		std::vector<double> defaultVals;
		//marker location information
		std::vector<markerXMLData> markerLocs;
		//marker location xml file name
		std::string markerXMLFileName;
		
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

		//randomization control vars
		//random camera orientation theta
		double rnd_camThet;
		//random camera zoom standard dev
		double rnd_camZoom;
		//random camera translation std
		double rnd_camTrans;
		//bound around background color to avoid when generating a color for head
		double rnd_headClrBnd;
		//pct of original size that head dims can vary
		double rnd_headDimPct;
		//bound around background color to avoid when generating a color for hand
		double rnd_handClrBnd;
		//pct of original size that hand dimensions can vary
		double rnd_handDimPct;
		//end randomization control vars

		//quality of motion blur - 0-5 from xml file
		//int motionBlurQual;
		//# of motion blur frames before current frame to use 
		int mBlurPreFrames;
		//# of motion blur frames after current frame to use 
		int mBlurPostFrames;

		//int width/height of windows
		int win_Width, win_Height;
		//# of total symbols of a particular letter we're generating
		int numTotSymPerLtr;

		//size of window for LSTM in training - make clips fixed to multples of this length to prevent image paddings
		int fixedClipLen;

		//initial zoom
		float origZoom;
		//letter type to be generated - either constant velocity, 16-frame long train/mult8 test, or mult8 train/mult8 test
		DataType dataType;

		//background colors
		double bkgR, bkgG, bkgB, bkgA;

		//not saved as default or read from file : string to hold some value that denotes current date.   used to add to generated file names to make unique
		std::string dateFNameOffset;

	private :
		//base output directory to write results in, if not default
		std::string baseOutDir;

		std::vector<bool> flags;									//various boolean flags used to drive GestIK
		static const int
			//whether or not to add the possibility of using the left hand to draw to random letter gen
			IDX_useLeftHand = 0,  //TODO not implemented yet
			//whether or not to change colors per trajectory for debug display
			IDX_chgTrajDbgClrs = 1,
			//randomization settings - whether or not to turn on randomization for camera or skeleton dimensions
			//random camera orientation
			IDX_rndCamOrient = 2,
			//random camera location (translate)
			IDX_rndCamLoc = 3,
			//random head width/height
			IDX_rndHeadDims = 4,
			//random head color(near black or near white) - always ellipsoid
			IDX_rndHeadClr = 5,
			//randomize hand shape - either block or ellipsoid
			IDX_rndHandShape = 6,
			//randomize hand width/depth/length
			IDX_rndHandDims = 7,
			//randomize hand color
			IDX_rndHandClr = 8,
			//use motion blur
			IDX_useMotionBlur = 9,
			//whether or not to use xml-specified output dir
			IDX_useOutputDir = 10,
			//whether or not to save COM-related values (hand COM, COM vel, elbow COM, COM vel) in screen coords
			IDX_saveHandCOMVals = 11,
			//whether or not to always save screen shots - should only be false if saving com values (but not always false - may want both)
			IDX_alwaysSaveImgs = 12;

		static const int numFlags = 13;
	};

}//namespace gestureIKApp 
#endif  // APPS_GESTURE_GestIKParams_H_
