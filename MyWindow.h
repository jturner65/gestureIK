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
#include "apps/gestureIK/MyGuiHandler.h"
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

	//template<std::size_t SIZE>
	//void calcCornerTrajPoints(eignVecTyp& _trajPts, std::array<double, SIZE>const & arr,  double& t);

	template<std::size_t SIZE>
	double calcTrajLength(std::array<double, SIZE>const & arr);

	///////////////////
	//end templates
	////

	void buildLetterList();
	eignVecVecTyp buildTrajSeq(void (MyWindow::*trajFunc)(eignVecTyp&), int traj, double& t);

	void calcCircleTrajPoints(eignVecTyp& _trajPts);
	void calcTrajPoints(eignVecTyp& _trajPts, eignVecVecTyp& arr, int numPts, double& globT);
	//check capturing state upon reset of indexes (always start capturing at beginning of trajectory TODO change this to manage automated multi-capture also)
	void checkCapture(bool drawLtr) {//should only be called when t transitions to 0, at the end of a trajectory
		mCapture = false;			//turn off capturing
		if (drawLtr) {

		}
		else {
			if (flags[collectDataIDX]) { trainDatManageCol(); return; }				//manage transitions for automated training data capture whenever t transitions to 0 (a trajectory has just finished)
			if (flags[stCaptAtZeroIDX]) {
				flags[stCaptAtZeroIDX] = false;
				mCapture = true;
			}
		}
	}

	//override
	virtual void displayTimer(int _val);
	//override draw function in SimWindow
	virtual void draw();
	void drawAxes(const Eigen::Vector3d& axesLoc, float len, bool altColor);
	void drawCurTraj();
	void drawJointAxis(dart::dynamics::BodyNode* node);
	void drawJointAxes();
	void drawPoint(const Eigen::Ref<const Eigen::Vector3d>& pt);
	virtual void drawSkels();

	//get file directory for current trajectory
	std::string getCurTrajFileDir(int dataIterVal);

	//get file name for screencapture, including file path - called by capture function
	std::string getScreenCapDirFileName();

	Eigen::Vector3d getCirclEndPoint(const Eigen::Ref<const Eigen::Vector3d>& ctrPt, double theta, double rad);
	//get random double with mean mu and std = std
	inline double getRandDbl(double mu, double std = 1.0) {
		double val = (*normdist)(mtrn_gen);
		val *= (std * std);
		val += mu;
		return val;
	}

	virtual void keyboard(unsigned char _key, int _x, int _y);
	//regenerate sample object trajectories with or without randomization
	void regenerateSampleData(bool rand);
	//override glutwindow screenshot function
	virtual bool screenshot();

	void setDrawLtrOrSmpl(bool drawLtr, int idx) {
		flags[useLtrTrajIDX] = drawLtr;
		flags[doneTrajIDX] = false;
		if (drawLtr) {
			curLetter = letters[idx];
			curLetter->setRandSymbolIdx();							//set random symbol among list of symbols for this letter
			curTrajStr = curLetter->getCurSymbolName();				//build name of current symbol trajectory for screen cap purposes
		} else {
			curTraj = idx;
			tVals[idx] = 0;
			curTrajStr = trajNames[curTraj];
			captCount[idx] = 0;
		}
	}

	//initialize state to automate training data collection
	void trainDatInitCol();
	//manage the process of writing all training and testing data
	void trainDatManageCol();
	//write line in text file to 
	void trainDatWriteIndexFile(std::ofstream& outFile, const std::string& fileDir, int cls);
	//update the time click after each IK iteration
	//void updateTVals(int traj);
	//write csv trajectory file
	void writeTrajCSVFile(const std::string& _fname, eignVecVecTyp& _trajAra);
	void writeTrajCSVFileRow(std::ofstream& outFile, eignVecTyp& _trajAra);

private:
	inline std::string getCurrLocAndQuat();

	//test the results of the random # generator
	inline void testRandom() {
		double mu = 0;
		for (int i = 0; i < 1000; ++i) {
			auto rnd = getRandDbl(mu, .00001);
			cout << "i : " << i << "\tgetRandDbl with mu : " << mu << " : " << rnd << "\n";

		}
	}

	//skeleton
	dart::dynamics::SkeletonPtr skelPtr;
	//ticks for sample traj drawing
	vector<double> tVals, tBnds, tIncr;
	//file stream for test and train index files
	std::ofstream testOutFile, trainOutFile;
	//strings for file names and directories
	//name of directory of current trajectory
	std::string curTrajDirName;
	//IK solver
	std::shared_ptr<IKSolver> IKSolve;

	///sample generated non-letter trajectory symbols
	//points to track - finger and elbow, fixed points around shoulders and head
	eignVecTyp mTrajPoints;
	//current values for corners of triangle, square, star
	eignVecVecTyp triCrnrs, sqrCrnrs, starCrnrs;
	//current sample trajectory idx
	int curTraj;
	//current screen capture idx for each of 4 trajs
	std::vector<int> captCount;
	//current start vertex for each of 4 trajs (circle idx ignored TODO)
	std::vector<int> curStIdx;
	//# of trajectories catured for training data, testing data
	std::vector<int> dataGenIters;
	//current trajectory as string
	std::string curTrajStr;

	///letter trajectories
	//structure holding all letters
	std::vector<std::shared_ptr<MyGestLetter>> letters;
	//currently drawn letter
	std::shared_ptr<MyGestLetter> curLetter;

	//state flags 
	std::vector<bool> flags;				
	static const unsigned int debugIDX = 0,				//debug mode
		stCaptAtZeroIDX = 1,					//start capturing trajectory when t == 0
		useLtrTrajIDX = 2,						//use curLetter trajectory to draw, instead of sample circle/tri/square/star
		drawTrkMrkrsIDX = 3,					//yes/no to draw markers on skel
		drawLtrTrajIDX = 4,						//draw all trajectory components of symbol or sample symbol
		collectDataIDX = 5,						//turn on the training/testing data collection mechanism
		doneTrajIDX = 6;						//finished current trajectory

	static const unsigned int numFlags = 7;

	std::shared_ptr<std::normal_distribution<double> > normdist;
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
			if (flags[debugIDX]) { cout << "Obj size : " << SIZE << "\tRandom i : " << i << " start corner : " << stCrnr << "arr Idx : " << arrIdx << "\n"; }
			randThet = getRandDbl(arr[arrIdx], .3);
			randRad = getRandDbl(rad, .2);
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
			if (flags[debugIDX]) {	cout << "Obj size : " << SIZE << "\tNonrandom i : " << i << " start corner : " << stCrnr << "arr Idx : " << arrIdx << "\n"; }
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