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

#ifndef APPS_GESTURE_MYWINDOW_H_
#define APPS_GESTURE_MYWINDOW_H_

#include <string>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include "dart/dart.h"
#include "apps/gestureIK/GestGlbls.h"
#include "apps/gestureIK/MyGestLetter.h"
#include "apps/gestureIK/IKSolver.h"
#include "apps/gestureIK/GestIKParser.h"

using namespace gestureIKApp;

class MyWindow : public dart::gui::SimWindow {
public:
	MyWindow(std::shared_ptr<IKSolver> _controller);
	virtual ~MyWindow();

	//regenerate the corners for the passed traj
	template<std::size_t SIZE>
	eignVecVecTyp regenCorners(std::array<double, SIZE>const & arr, bool randomize, int stCrnr);

	template<std::size_t SIZE>
	double calcTrajLength(std::array<double, SIZE>const & arr);
	///////////////////
	//end templates
	////

	void initCustWindow(std::string _winTtl);

	void buildLetterList();
	//build debug letters
	void buildRandDbugLetterList();
	eignVecVecTyp buildTrajSeq(void (MyWindow::*trajFunc)(eignVecTyp&), int traj, double& t);

	void calcCircleTrajPoints(eignVecTyp& _trajPts);
	void calcTrajPoints(eignVecTyp& _trajPts, eignVecVecTyp& arr, int numPts, double& globT);
	//check capturing state upon reset of indexes (always start capturing at beginning of trajectory TODO change this to manage automated multi-capture also)
	void checkCapture(bool drawLtr) {//should only be called when t transitions to 0, at the end of a trajectory
		mCapture = false;			//turn off capturing - will turn back on if necessary
		if (drawLtr) {																//drawing a letter trajectory and not a sample symbol trajectory
			if (flags[collectDataIDX]) { //finished complete letter trajectory, performing screen capture, set up next letter
				trainLtrDatManageCol();
				return; 
			}				//manage transitions for automated training data capture whenever letter is finished
		}
		else {
			if (flags[collectDataIDX]) { trainSymDatManageCol(); return; }				//manage transitions for automated training data capture whenever t transitions to 0 (a trajectory has just finished)
			if (flags[stCaptAtZeroIDX]) {
				flags[stCaptAtZeroIDX] = false;
				mCapture = true;
			}
		}
	}
	//screen cap all letters
	void trainLtrDatManageCol();
	//override
	virtual void timeStepping();
	//override
	virtual void displayTimer(int _val);
	//render function for using motionblur
	void renderBlur();
	//override render from win3d
	virtual void render() override;
	//override draw function in SimWindow
	virtual void draw();
	void drawAxes(const Eigen::Ref<const Eigen::Vector3d>& axesLoc, float len, bool altColor);
	void drawCurTraj();
	void drawJointAxis(dart::dynamics::BodyNode* node);
	void drawJointAxes();
	void drawPoint(const Eigen::Ref<const Eigen::Vector3d>& pt);
	virtual void drawSkels();

	//get file directory for current trajectory - only for SYMBOLS
	std::string getCurTrajFileDir(int dataIterVal);

	//get file name for screencapture, including file path - called by capture function - for both symbols and letters
	std::string getScreenCapDirFileName();

	Eigen::Vector3d getCirclEndPoint(const Eigen::Ref<const Eigen::Vector3d>& ctrPt, double theta, double rad);

	//open index file for current data collection, either training or testing idx
	void openIndexFile(std::ofstream& strm, bool append);

	virtual void keyboard(unsigned char _key, int _x, int _y);
	//regenerate sample object trajectories with or without randomization
	void regenerateSampleData(bool rand);

	//reload all xml-based parameters, and reset values derived from these parameters
	void reloadXml() {
		IKSolve->loadIKParams();
		//rebuild motionBlurFreq if used
		setMotionBlurQual();
	}//

	//override glutwindow screenshot function
	virtual bool screenshot();

	//setup each letter or sample trajectory
	void setDrawLtrOrSmpl(bool drawLtr, int idx, int symIdx = 0);

	//set motionBlurFreq based on motionblur quality specified in IKParams
	void setMotionBlurQual();

	bool makeDirectory(const std::string& tmp);

	//return string of full path to where the screen cap files will be written
	inline std::string getFullBasePath() {
		std::stringstream ss;
		ss << framesFilePath;
		if (flags[useLtrTrajIDX]) {	ss << "letters_" << DataType2strAbbrev[IKSolve->params->dataType] <<"_"<< IKSolve->params->dateFNameOffset << "/";}
		else {		ss <<"samples/";	}		
		return ss.str();
	}

	//initialize state to automate training data collection
	void trainDatInitCol(bool isLtr);
	//manage the process of writing all training and testing data
	void trainSymDatManageCol();
	//write line in text file to 
	void trainDatWriteIndexFile(std::ofstream& outFile, const std::string& fileDir, int cls);
	//update the time click after each IK iteration
	//void updateTVals(int traj);
	//write csv trajectory file
	void writeTrajCSVFile(const std::string& _fname, eignVecVecTyp& _trajAra);
	void writeTrajCSVFileRow(std::ofstream& outFile, eignVecTyp& _trajAra);

protected :
	inline std::string getCurrLocAndQuat();
	inline std::string getCurrQuatAsRot(int row);

	//test the results of the random # generator
	inline void testRandom() {
		double mu = 0;
		for (int i = 0; i < 1000; ++i) {
			auto rnd = IKSolve->getRandDbl(mu, .00001);
			std::cout << "i : " << i << "\tgetRandDbl with mu : " << mu << " : " << rnd << std::endl;
		}
	}

	//reset all control variables 
	inline void resetCurVars() {
		curSymIDX = 0;
		curLtrIDX = 0;
		flags[collectDataIDX] = false;
		setDrawLtrOrSmpl(false, 0);
		mCapture = false;
	}

	//set random values for orientation and/or skeleton geometry
	void setRandomValues();

	//set camera values to passed values
	inline void setCameraVals(const Eigen::Quaterniond& _tbQuat, float _zoom, const Eigen::Ref<const Eigen::Vector3d>& _trans) {
		mTrackBall.setQuaternion(_tbQuat);
		mZoom = _zoom;
		mTrans << _trans;
	}
	//reset skeleton values to original values
	void resetSkelVals();
	//reset initial values size and color for passed body node name
	void setShapeSize(const std::string& nodeName, int visIDX, const Eigen::Ref<const Eigen::Vector3d>& _origSize);
	void setShapeClr(const std::string& nodeName, int visIDX, const Eigen::Ref<const Eigen::Vector3d>& _origClr);
	//save initial skeleton values 
	void saveInitSkelVals();
	//save size and color vals for particular shape
	void saveInitShapeVals(const std::string& nodeName, int visIDX, Eigen::Ref<Eigen::Vector3d> _destSize, Eigen::Ref<Eigen::Vector3d> _destClr);
	//add a spherical visualization shape to each hand
	void MyWindow::addSphereHand(const std::string& nodeName);

	///////////////////////////
	//variables 
	////////////////////////
	//skeleton
	dart::dynamics::SkeletonPtr skelPtr;
	//original skeleton values, to reset values
	Eigen::Vector3d skel_headSize,
		skel_headClr,
		skel_handSize,
		skel_handClr;
	//0 == rect, 1 == ellipse
	int curHandShape;
	//ticks for sample traj drawing
	std::vector<double> tVals, tBnds, tIncr;
	//file stream for test and train index files - remove distinction, have python script handle partition
	std::ofstream trainDataFileStrm;
	//strings for file names and directories
	//name of directory of current trajectory
	std::string curTrajDirName;
	//IK solver
	std::shared_ptr<IKSolver> IKSolve;

	//current trajectory name as string, current class name as string - for letters curTrajStr will be e.g a_12 and curClassName will be e.g. a
	std::string curClassName;

	///sample generated non-letter trajectory symbols
	//points to track - finger and elbow, fixed points around shoulders and head
	eignVecTyp mTrajPoints;
	//current values for corners of triangle, square, star
	eignVecVecTyp triCrnrs, sqrCrnrs, starCrnrs;
	//current sample trajectory idx
	int curTraj;
	//all idxed by curTraj (symbols only): current screen capture idx for each of 4 trajs,current start vertex for each of 4 trajs (circle idx ignored TODO),# of trajectories catured for training data, testing data for each of 4 trajs
	std::vector<int> captCount, curStIdx, dataGenIters;

	//precalced based on window dims set from IKSolve->params
	double aspectRatio;

	///Motion Blur
	//motion blur + / - frames 
	int motionBlurFrames;

	///letter trajectories
	//structure holding all letters
	std::vector<std::shared_ptr<MyGestLetter>> letters;
	//currently drawn letter
	std::shared_ptr<MyGestLetter> curLetter;
	//current letter in list of shrd ptrs of ltrs, current symbol in list of symbols in letter, start idx of symbols - either 0 or the # of file-loaded symbols for a letter (if specified to regen only random and not file-built sequences)
	int curLtrIDX, curSymIDX;

	//state flags 
	std::vector<bool> flags;				
	static const unsigned int debugIDX = 0,				//debug mode
		stCaptAtZeroIDX = 1,					//start capturing trajectory when t == 0
		initTrnDatCapIDX = 2,					//start capturing all trajectories at next displayTimer loop
		useLtrTrajIDX = 3,						//use curLetter trajectory to draw, instead of sample circle/tri/square/star
		drawTrkMrkrsIDX = 4,					//yes/no to draw markers on skel
		drawLtrTrajIDX = 5,						//draw all trajectory components of symbol or sample symbol
		collectDataIDX = 6,						//turn on the training/testing data collection mechanism
		doneTrajIDX = 7,						//finished current trajectory
		pauseIKLtrIDX = 8,						//pause between IK frames - for debugging purposes
		testLtrQualIDX = 9,						//iterate through all letters without screen cap to test traj quality
		showAllTrajsIDX = 10,					//show all trajectories of letters, to show distribution results - debug
		debugLtrsBuiltIDX = 11,					//set of debug letters built for visualization
		useMotionBlurIDX = 12;					//use motion blur when capturing/rendering

	static const unsigned int numFlags = 13;
};

//calc total length of combined trajectories
template<std::size_t SIZE>
double MyWindow::calcTrajLength(std::array<double, SIZE>const & arr) {
	double res = 0;
	double rad = IKSolve->params->IK_drawRad*sqrt2;
	Eigen::Vector3d stVec, endVec;
	for (int i = 0; i < SIZE; ++i) {
		stVec = getCirclEndPoint(IKSolve->drawCrclCtr, arr[i], rad);
		endVec = getCirclEndPoint(IKSolve->drawCrclCtr, arr[(i+1)% SIZE], rad);
		res += (stVec - endVec).norm();
	}
	return res;
}

//regenerate the corners for the passed array - use this vector of vector of corners for sample trajectory building
template<std::size_t SIZE>
eignVecVecTyp MyWindow::regenCorners(std::array<double, SIZE>const & arr, bool randomize, int stCrnr) {
	eignVecVecTyp crnrs(0);

	double rad = IKSolve->params->IK_drawRad*sqrt2;
	double randThet = 0, randRad = 0;
	if (randomize) {
		int numCrnrs = SIZE + 1;
		for (int i = 0; i < numCrnrs; ++i) {
			eignVecTyp crnrVec(0);
			int arrIdx = ((i%SIZE) + stCrnr) % SIZE;
			if (flags[debugIDX]) { std::cout << "Obj size : " << SIZE << "\tRandom i : " << i << " start corner : " << stCrnr << "arr Idx : " << arrIdx << std::endl; }
			randThet = IKSolve->getRandDbl(arr[arrIdx],  .09);
			randRad = IKSolve->getRandDbl(rad, .04);
			crnrVec.push_back(getCirclEndPoint(IKSolve->drawCrclCtr, randThet, randRad));
			crnrVec.push_back(getCirclEndPoint(IKSolve->drawElbowCtr, randThet, .25 * randRad));
			crnrs.push_back(std::move(crnrVec));
		}
	}
	else {
		int numCrnrs = SIZE;
		for (int i = 0; i < numCrnrs; ++i) {
			eignVecTyp crnrVec(0);
			int arrIdx = ((i%SIZE) + stCrnr) % SIZE;
			if (flags[debugIDX]) { std::cout << "Obj size : " << SIZE << "\tNonrandom i : " << i << " start corner : " << stCrnr << "arr Idx : " << arrIdx << std::endl; }
			crnrVec.push_back(getCirclEndPoint(IKSolve->drawCrclCtr, arr[arrIdx], rad));
			crnrVec.push_back(getCirclEndPoint(IKSolve->drawElbowCtr, arr[arrIdx], .25 * rad));
			crnrs.push_back(std::move(crnrVec));
		}
	}
	return crnrs;
}



////calc location of pointer and elbow traj point
////_trajPts : vector of eigen v3d of trajectory points
////arr : point locations along edge of circle of radius rad*sqrt2
//template<std::size_t SIZE>
//void MyWindow::calcCornerTrajPoints(eignVecTyp& _trajPts, std::array<double, SIZE>const & arr, double& globT) {
//	int sIdx = (int)(SIZE*globT);
//	int nextIDX = (sIdx + 1) % SIZE;
//	double t = (SIZE*globT) - sIdx;
//	//getCirclEndPoint(const Eigen::Ref<const Eigen::Vector3d>& ctrPt, const Eigen::Ref<const Eigen::Vector3d>& n, double theta, double rad)
//	double rad = IKSolve->params->IK_drawRad*sqrt2;
//	Eigen::Vector3d stVec = getCirclEndPoint(IKSolve->drawCrclCtr, arr[sIdx], rad);
//	Eigen::Vector3d nextVec = getCirclEndPoint(IKSolve->drawCrclCtr, arr[nextIDX], rad);
//	_trajPts[0] = interpVec(stVec, nextVec, t);
//
//	stVec = getCirclEndPoint(IKSolve->drawElbowCtr, arr[sIdx], .25 * rad);
//	nextVec = getCirclEndPoint(IKSolve->drawElbowCtr,  arr[nextIDX], .25 * rad);
//	_trajPts[1] = interpVec(stVec, nextVec, t);
//
//	globT += IKSolve->params->IK_desiredVel/SIZE;
//	if (globT > 1) { 
//		globT = 0;
//		checkCapture(); 
//	}
//}

#endif  // APPS_GESTURE_MYWINDOW_H_
