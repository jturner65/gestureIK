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

#include "dart/math/Helpers.h"
#include <sys/types.h>
#include "apps/gestureIK/MyWindow.h"
#include <iostream>
#include <math.h>       /* fmod */
#include <direct.h>

#include <fstream>


using namespace gestureIKApp;
using namespace dart::gui;

static int screenCapCnt = 0, trainSymDatCnt = 0, trainLtrtCnt = 0, displayTmrCnt = 0, drawCnt = 0;

MyWindow::MyWindow(std::shared_ptr<IKSolver> _ikslvr) : SimWindow(),  IKSolve(_ikslvr), tVals(4), tBnds(4,1), tIncr(4,1), testOutFile(), trainOutFile(),
	curTrajDirName(""),  mTrajPoints(7), letters(0), curTraj(0), curTrajStr(trajNames[0]), curClassName(trajNames[0]), curSymIDX(0),
	triCrnrs(0),  sqrCrnrs(0), starCrnrs(0),captCount(4,0), curStIdx(4,0), dataGenIters(4, 0),
	flags(numFlags,false){

	mBackground[0] = IKSolve->params->bkgR;
	mBackground[1] = IKSolve->params->bkgG;
	mBackground[2] = IKSolve->params->bkgB;
	mBackground[3] = IKSolve->params->bkgA;

	tBnds[0] = DART_2PI;//circle bounds on t value
	//set init display vals
	mDisplayTimeout = 10;				//set to be faster frame rate
	mTrackBall.setQuaternion(origTrackBallQ);
	mZoom = IKSolve->params->origZoom;
	mTrans = origMTrans;

	//start trajectory at initial position of hand
	skelPtr = IKSolve->getSkel();
	
	//set values used to map out trajectories to draw - needs to be owned by each symbol
	IKSolve->setSampleCenters();

	tIncr[0] = .2;//set to be .2 because it is a deltaTheta for the circle
	for (int i = 1; i < 4; ++i) {
		tIncr[i] = IKSolve->params->trajDesiredVel;// / (i + 2.0);//dividing by # of sides because interpolating between each pair of verts with a local t: 0->1, but treating all sets of edges as complete trajectory (i.e. glbl t 0->1)
	}
	std::cout << "Per Frame ptr distance travelled calculated : " << IKSolve->params->trajDesiredVel << std::endl;

	triCrnrs = regenCorners<3U>(triCrnrConsts, false, curStIdx[1]);
	sqrCrnrs = regenCorners<4U>(sqCrnrConsts, false, curStIdx[2]);
	starCrnrs = regenCorners<5U>(strCrnrConsts, false, curStIdx[3]);

	////writes trajectory for circle
	//curTraj = 0;//set to 1,2, 3 for tri/square/star
	//void(MyWindow::*trajFunc)(eignVecTyp&);
	//trajFunc = &MyWindow::calcCircleTrajPoints;
	//auto trajSeq = buildTrajSeq(trajFunc, curTraj, tVals[curTraj]);
	//writeTrajCSVFile(trajNames[curTraj], trajSeq);

	//read all letter files and build trajectories
	buildLetterList();
	
	curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
	//set refresh function
	glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
}

MyWindow::~MyWindow() {}

//set window size based on params value loaded from xml
void MyWindow::initCustWindow(std::string _winTtl) {
	initWindow(IKSolve->params->win_Width, IKSolve->params->win_Height, _winTtl.c_str());
}

void MyWindow::openIndexFile(bool isTrain, std::ofstream& strm, bool append) {
	std::stringstream ss;
	ss << getFullBasePath() << (isTrain ? "TrainDataIndexFile.txt" : "TestDataIndexFile.txt");
	const std::string tmp = ss.str();
	unsigned int mode = (append ? std::fstream::app : std::fstream::out);
	strm.open(tmp.c_str(), mode);
	if (!strm.is_open()) {
		std::cout << tmp << " failed to open!"<< std::endl;
	}
}//openIndexFile

 //get current trajectory's file directory
 //dataIterVal is value of current iteration of data generation
//or current symbol name if using letters
std::string MyWindow::getCurTrajFileDir(int dataIterVal) {
	if (flags[useLtrTrajIDX]) {
		return curLetter->getCurSymbolName();
	}
	else {
		std::stringstream ss("");
		//<trajType>_<trajIter>
		ss << curTrajStr << "_" << dataIterVal;
		return ss.str();
	}
}

//set up necessary functionality and state to conduct randomized data collection
//this will entail building trajectories for triangle, square and star, starting at each vertex,
//with and without random shifts in end points,  [and moving in clockwise or ccw direction (TODO)]
void MyWindow::trainDatInitCol(bool isLtr) {
	if ((flags[useLtrTrajIDX]) && ((!IKSolve->params->mkNonRandSeq()) && (!IKSolve->params->genRandLtrs()))) {
			std::cout << "XML specifies to not save sequences of file based letters(IDX_mkNonRandSeq), and to not make any random letters(IDX_genRandLtrs), and so there's nothing to save."<< std::endl;
			curSymIDX = 0;
			curLtrIDX = 0;
			flags[collectDataIDX] = false;
			setDrawLtrOrSmpl(false, 0);
			mCapture = false;
			return;
	}	
	flags[useLtrTrajIDX] = isLtr;
	flags[collectDataIDX] = true;
	//turn off all markers and trajectory displays
	flags[drawTrkMrkrsIDX] = false;
	flags[drawLtrTrajIDX] = false;
	mShowMarkers = false;
	
	if (flags[useLtrTrajIDX]) {
		if (flags[testLtrQualIDX]){
			std::cout << "Testing letter trajectories." << std::endl;
			curLtrIDX = IKSolve->params->ltrIdxStSave;		//either start at beginning or start at a specific letter
			stSymIdx = 0;									//test starting from beginning
			curSymIDX = stSymIdx;
			flags[drawTrkMrkrsIDX] = true;
			flags[drawLtrTrajIDX] = true;
		}
		else {
			std::cout << "Initializing random data collection for Letter Trajectories." << std::endl;
			bool append = !IKSolve->params->mkNonRandSeq();
			openIndexFile(false, testOutFile, append);
			openIndexFile(true, trainOutFile, append);
			//1st time initialize letter collection - init curLtrIDX, curSymIDX
			curLtrIDX = IKSolve->params->ltrIdxStSave;		//either start at beginning or start at a specific letter
			stSymIdx = append ? letters[curLtrIDX]->numFileSymbols : 0;		//either start at beginning or start saving only randomized letters
			curSymIDX = stSymIdx;
		}
		//start capture
		trainLtrDatManageCol();
	}
	else {
		openIndexFile(false, testOutFile, false);
		openIndexFile(true, trainOutFile, false);
		std::cout << "Initializing random data collection for Symbols."<< std::endl;
		//initialize starting idxs, t values and counts of training and testing data for all symbol trajectories
		for (int i = 0; i < 4; ++i) {
			curStIdx[i] = 0;
			tVals[i] = 0;
			dataGenIters[i] = 0;
			captCount[i] = 0;
		}//idx 0 corresponds to circle but not used currently
			//start on triangle - set all relevant info
		setDrawLtrOrSmpl(false, 1);
		//start capture
		trainSymDatManageCol();
	}
}//trainDatInitCol


//manage the process of writing all training and testing data - call whenever a trajectory has been captured (from checkCapture() )
//ONLY FOR SAMPLE SYMBOLS that consist of single trajectories!
void MyWindow::trainSymDatManageCol() {
	//if (flags[debugIDX]) { cout << "Start trainSymDatManageCol : " << trainSymDatCnt << std::endl; }
	//call first time right after init
	//flags[collectDataIDX] pre-empts flags[stCaptAtZeroIDX] so needs to set mCapture
	//regerenate all endpoints - we start on non-random data - for current trajectory, make #verts trajectories starting on each vert without random, before using random trajectories
	regenerateSampleData(dataGenIters[curTraj] >= (curTraj + 2));

	//global name of directory to write to, of form : <trajType>_<trajIter> --> add _<frame#>.png for file name
	curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);

	//need to write entry into either testing or training file - if dataGenIters > some threshold make train file, else make test file
	if (dataGenIters[curTraj] < IKSolve->params->dataCapTestTrainThresh*IKSolve->params->dataCapNumExamples) {
		trainDatWriteIndexFile(trainOutFile, curTrajDirName, curTraj);
	}
	else {
		trainDatWriteIndexFile(testOutFile, curTrajDirName, curTraj);
	}

	//after drawing all frames of a trajectory the following needs to be reset (i.e. every call of this function) - this is frame # of capture of current trajectory
	captCount[curTraj] = 0;
	//...and need to advance data counts
	dataGenIters[curTraj]++;

	//after transitioning to a different symbol, the following needs to be reset
	if (dataGenIters[curTraj] > IKSolve->params->dataCapNumExamples) {
		dataGenIters[curTraj] = 0;
		setDrawLtrOrSmpl(false, (curTraj + 1) % trajNames.size());		//iterate to next trajectory if we've capture sufficient samples
		dataGenIters[curTraj] = 0;
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "Test/train data gen transition to sample " << trajNames[curTraj] << " trajectory"<< std::endl;
	}
	if (curTraj != 0) { mCapture = true; }		//repeat for next trajectory type
	else {//at curTraj == 0 we have completed all data gen, and should stop - reset all important stuff to circle as if key 1 was pressed
		flags[collectDataIDX] = false;
		testOutFile.close();
		trainOutFile.close();
		setDrawLtrOrSmpl(false, 0);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "Finished generating test/train data, reset to sample " << trajNames[curTraj] << " trajectory"<< std::endl;
	}	
	//if (flags[debugIDX]) { cout << "End trainSymDatManageCol : " << trainSymDatCnt++ << std::endl; }
}

//capture all letters
void MyWindow::trainLtrDatManageCol() {
	if (curLtrIDX >= letters.size()) {//done every letter
		curLtrIDX = 0;
		stSymIdx = IKSolve->params->mkNonRandSeq() ? 0 : letters[curLtrIDX]->numFileSymbols;		//either start at beginning or start saving only randomized letters
		curSymIDX = stSymIdx;
		if (!flags[testLtrQualIDX]) {
			testOutFile.close();
			trainOutFile.close();
		}
		flags[testLtrQualIDX] = false;
		flags[collectDataIDX] = false;
		mCapture = false;
		setDrawLtrOrSmpl(false, 0);
		return;
	}
	if (!flags[testLtrQualIDX]) {
		std::cout << "Start trainLtrDatManageCol : " << trainLtrtCnt << "\tcurLtrIDX : " << curLtrIDX << "\tcurSymbIDX : " << curSymIDX << std::endl;
	}
	//need to cycle through all letters 
	setDrawLtrOrSmpl(true, curLtrIDX, false, curSymIDX);
	curTrajDirName = getCurTrajFileDir(-1);
	//need to write entry into either testing or training file - if dataGenIters > some threshold make train file, else make test file 
	if (!flags[testLtrQualIDX]) {
		//if (curSymIDX < IKSolve->params->dataCapTestTrainThresh*curLetter->symbols.size()) { //flags[isTrainDatIDX]
		if (curLetter->symbols[curSymIDX]->isTrainDat()) {
			trainDatWriteIndexFile(trainOutFile, curTrajDirName, curLtrIDX);
		}
		else {
			trainDatWriteIndexFile(testOutFile, curTrajDirName, curLtrIDX);
		}
		mCapture = true;
	}
	//for next iteration
	curSymIDX += 1;
	if (curSymIDX >= curLetter->symbols.size()){
		curLtrIDX += 1;
		stSymIdx = IKSolve->params->mkNonRandSeq() ? 0 : (curLtrIDX >= letters.size() ? 0 : letters[curLtrIDX]->numFileSymbols);		//either start at beginning or start saving only randomized letters
		curSymIDX = stSymIdx;
	}
	if (!flags[testLtrQualIDX]) {
		std::cout << "End trainLtrDatManageCol : " << trainLtrtCnt++ << std::endl;
	}
}//trainLtrDatManageCol

//write line in index/label text file for each example
void MyWindow::trainDatWriteIndexFile(std::ofstream& outFile, const std::string& fileDir, int cls) {
	std::stringstream ss;
	ss.str("");
	ss << curClassName <<"/"<<fileDir << " " << cls << std::endl;
	if (flags[debugIDX]) { std::cout << ss.str(); }	//debug
	outFile << ss.str();
}


//build map of all available symbols, idxed by name
void MyWindow::buildLetterList() {
	letters.clear();
	
	for (int i = 0; i < IKSolve->params->numLetters; ++i) {
		std::string c(1, i + 97);
		letters.push_back(std::make_shared<MyGestLetter>(c));
		letters[i]->setSolver(IKSolve);
		GestIKParser::readGestLetterXML(letters[i]);
		std::cout << "Made letter : " << (*letters[i]) << std::endl;
	}
	//buildRandomSymbolTrajs
	if (!IKSolve->params->genRandLtrs()) {
		std::cout << "Not making random letters due to 'IDX_genRandLtrs' flag in config xml setting.  Set to true to make them."<< std::endl;
	}
	for (int i = 0; i < letters.size(); ++i) {
		if (IKSolve->params->numTotSymPerLtr > letters[i]->symbols.size()) {
			letters[i]->buildRandomSymbolTrajs(IKSolve->params->numTotSymPerLtr, IKSolve->params->dataCapTestTrainThresh* IKSolve->params->numTotSymPerLtr);
			std::cout << "Made " << (IKSolve->params->numTotSymPerLtr - letters[i]->numFileSymbols) << " Random versions of letter : " << (*letters[i]) << std::endl;
		}
		else {
			std::cout << "Insufficient letters specified " << IKSolve->params->numTotSymPerLtr << " so no random versions of letter : " << (*letters[i]) << " made."<< std::endl;
		}
	}
}//buildLetterList

//build circular trajectory and write to csv file
eignVecVecTyp MyWindow::buildTrajSeq(void (MyWindow::*trajFunc)(eignVecTyp&), int traj, double& t) {
	eignVecVecTyp _trajAraAra(0);
	eignVecTyp _tmpTrajFrame(0);
	do {
		_tmpTrajFrame.clear();
		for (int i = 0; i < mTrajPoints.size(); ++i) {
			_tmpTrajFrame.push_back(mTrajPoints[i]);						//creates a copy of mTrajPoints[i]
		}
		(*this.*trajFunc)(_tmpTrajFrame);									//zoinks - call trajectory functio
		_trajAraAra.push_back(std::move(_tmpTrajFrame));

	} while (t > 0);			//do 1 cycle of trajectory
	return _trajAraAra;
}

static int pauseCount = 0;
void MyWindow::displayTimer(int _val){
	if (flags[initTrnDatCapIDX]) {//initialize trajectories for training/testing data capture
		flags[initTrnDatCapIDX] = false;
		trainDatInitCol(flags[useLtrTrajIDX]);
	}
	//if last time around we finished trajectory, reset values/recapture, etc, for next trajectory
	if (flags[doneTrajIDX]) { flags[doneTrajIDX] = false; checkCapture(flags[useLtrTrajIDX]); }			//check at end of trajectory if attempting to save screen shots

	//if (flags[debugIDX]) { cout << "displayTimer : IKSolve start"<< std::endl; }
	if (flags[useLtrTrajIDX]) {
		if (flags[pauseIKLtrIDX]) {//wait for keypress between draw frames			
			pauseCount = pauseCount + 1;
			if (pauseCount % 20 != 0) {
				//exit without IK solving
				glutPostRedisplay();
				glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
				return;
			} else {
				pauseCount = 0;
			}
		}

		//cout << "TODO : IK on letter " << curLetter->ltrName << " symbol idx : "<< curLetter->curSymbol << std::endl;
		//setup trajectories and then solve 
		flags[doneTrajIDX] = curLetter->solve();
		//if (flags[debugIDX]) { cout << "displayTimer : letter solve done : disp timr cnt" << (displayTmrCnt++) << " with draw count : " << drawCnt << std::endl; }
	}
	else {
		switch (curTraj) {
		case 0: {		calcCircleTrajPoints(mTrajPoints);		break;		}
		case 1: {		calcTrajPoints(mTrajPoints, triCrnrs, (curTraj + 2), tVals[curTraj]);		break;		}
		case 2: {		calcTrajPoints(mTrajPoints, sqrCrnrs, (curTraj + 2), tVals[curTraj]);		break;		}
		case 3: {		calcTrajPoints(mTrajPoints, starCrnrs, (curTraj + 2), tVals[curTraj]);		break;		}
		}
		IKSolve->solve(mTrajPoints);
		//if (flags[debugIDX]) { cout << "displayTimer : IKSolve done : disp timr cnt" << (displayTmrCnt++) << " with draw count : " << drawCnt << std::endl; }
	}
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::drawCurTraj() {
	eignVecTyp drawPoints(7);
	switch (curTraj) {
	case 0: {		break;		}
	case 1: {	for (double t = 0; t < 1.0; t += .01) { calcTrajPoints(drawPoints, triCrnrs, 3, t); drawPoint(drawPoints[0]);	}	break;		}
	case 2: {	for (double t = 0; t < 1.0; t += .01) { calcTrajPoints(drawPoints, sqrCrnrs, 4, t); drawPoint(drawPoints[0]); }		break;		}
	case 3: {	for (double t = 0; t < 1.0; t += .01) { calcTrajPoints(drawPoints, starCrnrs, 5, t);  drawPoint(drawPoints[0]); }	break;		}
	}
}

//override draw in simwindow
void MyWindow::draw() {
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//cout << "draw start : " << (drawCnt) << std::endl;

	drawSkels();
	//traw tracked markers
	if (flags[drawTrkMrkrsIDX]) {
		IKSolve->drawTrkMrkrs(mRI, true);
	}
	if (flags[drawLtrTrajIDX]) {
		if ((flags[useLtrTrajIDX]) && (nullptr != curLetter)) {
			curLetter->drawLetter(mRI);
		}
		else {
			drawCurTraj();
		}
	}
	drawEntities();
	if (flags[debugIDX]) {
		glColor3f(0.0, 0.0, 0.0);
		dart::gui::drawStringOnScreen(0.02f, 0.02f, getCurrLocAndQuat());
	}
	glEnable(GL_LIGHTING);
	//cout << "draw end : " << (drawCnt++) << std::endl;
}

std::string MyWindow::getCurrLocAndQuat() {
	std::stringstream ss;
	ss.str("");
	ss << "Loc : " << buildStrFrmEigV3d(this->mTrans) << "\tZoom : "<<(this->mZoom)<<"\tQuat : w:" << this->mTrackBall.getCurrQuat().w() << " vec : (" << buildStrFrmEigV3d(this->mTrackBall.getCurrQuat().vec()) << ")"<< std::endl;
	return ss.str();
}

void MyWindow::drawSkels() {
	skelPtr->draw(mRI);
	// Draw the markers on the skeleton.
	if (mShowMarkers) {
		Eigen::Vector4d color;
		color << 0.95, 0.25, 0.25, 1.0;
		skelPtr->drawMarkers(mRI, color, false);
	}
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
	unsigned int keyVal = _key;
	if ((keyVal >= 65) && (keyVal <= 90)) {		//draw letter trajectories if any capital letter is selected
		int idx = keyVal - 65;
		//setDrawLtrOrSmpl(true, idx, true, -1);
		setDrawLtrOrSmpl(true, idx, false, curSymIDX);
		curSymIDX = (curSymIDX + 1) % curLetter->symbols.size();
		return;
	} 
	switch (_key) {
	case '1': {  // st trajectory to traj 1
		setDrawLtrOrSmpl(false, 0);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "sample "<< trajNames[curTraj] << " trajectory"<< std::endl;
		break; }
	case '2': {  // st trajectory to traj 2 tTri, tSqr, tStar,
		setDrawLtrOrSmpl(false, 1);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "sample " << trajNames[curTraj] << " trajectory"<< std::endl;
		break; }
	case '3': {  // st trajectory to traj 3
		setDrawLtrOrSmpl(false, 2);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "sample " << trajNames[curTraj] << " trajectory"<< std::endl;
		break; }
	case '4': {  // st trajectory to traj 4
		setDrawLtrOrSmpl(false, 3);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "sample " << trajNames[curTraj] << " trajectory"<< std::endl;
		break; }
	case '`': {         //reset trackball loc with origTrackBallQ and zoom with 1
		mTrackBall.setQuaternion(origTrackBallQ);
		mZoom = IKSolve->params->origZoom;
		mTrans = origMTrans;
		break; }
	case 'a': {  // screen capture all letters (train and test)
		flags[initTrnDatCapIDX] = true;
		flags[useLtrTrajIDX] = true;
		//turn off any tests that may have been initiated
		flags[testLtrQualIDX] = false;
		for (int i = 0; i < letters.size(); ++i) { letters[i]->setTestLtrQual(flags[testLtrQualIDX]); }
		std::cout << "Capture all letters with Train data restricted to 16 frames : " << (flags[limitTrainTo16IDX] ? "True" : "False") << std::endl;
		break; }
	case 'b': {  // pause between IK frames for debugging
		flags[pauseIKLtrIDX] = !flags[pauseIKLtrIDX];
		std::cout << "Pause between IK frames : " << (flags[pauseIKLtrIDX] ? "True" : "False")<<std::endl;
		break; }
	case 'c': {  // screen capture
		flags[stCaptAtZeroIDX] = true;				//start capturing when trajectory gets to 0
		std::cout << "start capturing when trajectory gets to t == 0\n.";
		break; }
	case 'd': { // debug mode
		flags[debugIDX] = !flags[debugIDX];
		std::cout << "Debug Mode : " << (flags[debugIDX] ? "True" : "False") << std::endl;
		for (int i = 0; i < letters.size(); ++i) {		letters[i]->setSymbolFlags(debugIDX, flags[debugIDX]);		}	//debugIDX is 0 in every class
		break; }
	case 'f': { //test all letters - same as screen cap except no IK and no file save
		flags[testLtrQualIDX] = !flags[testLtrQualIDX];
		std::cout<<"Test letter process : " << (flags[testLtrQualIDX] ? "True" : "False") << std::endl;
		if (flags[testLtrQualIDX]) {//turning on
			flags[initTrnDatCapIDX] = true;
			flags[useLtrTrajIDX] = true;
		}
		for (int i = 0; i < letters.size(); ++i) { letters[i]->setTestLtrQual(flags[testLtrQualIDX]); }
		break;}
	case 'g': {//toggle between limiting to 16 frames for each entire training example or splitting regular-duration training up into windows of 16 frames TODO		
		flags[limitTrainTo16IDX] = !flags[limitTrainTo16IDX];
		for (int i = 0; i < letters.size(); ++i) { letters[i]->limitTrainTo16(flags[limitTrainTo16IDX]); }
		std::cout << "Capture all letters with Train data restricted to 16 frames : " << (flags[limitTrainTo16IDX] ? "True" : "False") << std::endl;
		break; }
	case 'r': {//randomize symbols
		regenerateSampleData(true);
		break; }
	case 's': { // show or hide letter trajectories
		flags[drawLtrTrajIDX] = !flags[drawLtrTrajIDX];
		break; }
	case 't': { // show or hide tracked
		flags[drawTrkMrkrsIDX] = !flags[drawTrkMrkrsIDX];
		break; }
	case 'v': { // show or hide markers
		mShowMarkers = !mShowMarkers;
		break; }
	case 'w': {//generate test and train data on triangle, square, star
		flags[initTrnDatCapIDX] = true;
		flags[useLtrTrajIDX] = false;
		break; }
	case 'y': {//un-randomize symbols
		regenerateSampleData(false);
		break; }

    default:{    Win3D::keyboard(_key, _x, _y);}
	}
  glutPostRedisplay();
}

//automate the capture of training data - regenerate all corners with random values, and reset t-values
void MyWindow::regenerateSampleData(bool rand) {
	triCrnrs = regenCorners<3U>(triCrnrConsts, rand, curStIdx[1]);
	sqrCrnrs = regenCorners<4U>(sqCrnrConsts, rand, curStIdx[2]);
	starCrnrs = regenCorners<5U>(strCrnrConsts, rand, curStIdx[3]);
	for (int i = 0; i < 4; ++i) { 
		curStIdx[i] = curStIdx[i] + 1 % (i + 2);	
		tVals[i] = 0; 
		captCount[i] = 0;
	}//idx 0 corresponds to circle but not used.
}

//get file name for screen shot based on current trajectory name and count of frames
std::string MyWindow::getScreenCapDirFileName() {
	std::stringstream ss;
	ss << getFullBasePath() << curTrajDirName;
	const std::string tmp = ss.str();

	int nError = 0;
#if defined(_WIN32)
	nError = _mkdir(tmp.c_str()); // can be used on Windows
#else 
	nError = mkdir(tmp.c_str(), 0733); // can be used on non-Windows
#endif
	if ((nError != 0) && (nError != -1)) {//-1 is exists already
		std::cout << "Error attempting to create path : " << tmp << "\terror : "<<nError<< std::endl;
	}

	ss.str("");
	char buf[5];
	sprintf(buf, "%.4d", (flags[useLtrTrajIDX] ? (curLetter->getCurSymbolFrame()-1) : captCount[curTraj]++));
	ss << tmp <<"/"<< curTrajDirName<<"."<< buf << ".png";
	return ss.str();
}

bool MyWindow::screenshot() {
	//cout << "Screen cap start : " << screenCapCnt << " for draw iter : "<< (drawCnt-1)<< std::endl;
	const std::string tmp = getScreenCapDirFileName();
	const char* fileName = tmp.c_str();

	int tw = glutGet(GLUT_WINDOW_WIDTH);
	int th = glutGet(GLUT_WINDOW_HEIGHT);

	//glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

	// reverse temp2 temp1
	int tw4 = tw * 4;
	for (int row = 0; row < th; row++) {
		memcpy(&mScreenshotTemp2[row * tw4], &mScreenshotTemp[(th - row - 1) * tw4], tw4);
	}

	unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

	// if there's an error, display it
	if (result) {
		std::cout << "lodepng error " << result << ": "
			<< lodepng_error_text(result) << std::endl;
		return false;
	}
	else {
		std::cout << "wrote screenshot " << fileName //<< "\t screenCapCnt : " << screenCapCnt++ 
			<< std::endl;

		return true;
	}
}//screenshot

void MyWindow::drawAxes(const Eigen::Ref<const Eigen::Vector3d>& axesLoc, float len, bool altColor) {
	float col = (altColor ? .5f : 0);
	glPushMatrix();
	glTranslatef(axesLoc(0), axesLoc(1), axesLoc(2));
	glPushMatrix();
	glColor4f(1.0f, col, 0, 1.0f);
	glBegin(GL_LINES);                glVertex3f(0, 0, 0);                glVertex3f(len, 0, 0);            glEnd();
	glColor4f(0, 1.0f, col, 1.0f);
	glBegin(GL_LINES);                glVertex3f(0, 0, 0);                glVertex3f(0, len, 0);            glEnd();
	glColor4f(col, 0, 1.0f, 1.0f);
	glBegin(GL_LINES);                glVertex3f(0, 0, 0);                glVertex3f(0, 0, len);            glEnd();
	glPopMatrix();
	glPopMatrix();
}
//draws axes for all nodes that are children of node
void MyWindow::drawJointAxis(dart::dynamics::BodyNode* node) {
	glPushMatrix();
	node->getParentJoint()->applyGLTransform(mRI);      //gives location but not orientation
	drawAxes(Eigen::Vector3d(0, 0, 0), .1f, false);
	for (unsigned int i = 0; i < node->getNumChildBodyNodes(); i++) {
		drawJointAxis(node->getChildBodyNode(i));
	}
	glPopMatrix();
}
//draw all joint axes
void MyWindow::drawJointAxes() {//Vector3d nodeY = node->getWorldTransform().linear().col(1); 
	drawJointAxis(skelPtr->getRootBodyNode());
}


void MyWindow::drawPoint(const Eigen::Ref<const Eigen::Vector3d>& pt) {
	mRI->pushMatrix();
	mRI->setPenColor(Eigen::Vector3d(.25, .75, .75));
	mRI->translate(pt);
	mRI->drawEllipsoid(Eigen::Vector3d(0.03, 0.03, 0.03));
	mRI->popMatrix();
}

//project points at certain t (== theta) in trajectory upon a plane described by normal n, inscribed within a circle of radius 2*rad, centered at ctrPt
//plane normal always points back to tstRShldrSt
Eigen::Vector3d MyWindow::getCirclEndPoint(const Eigen::Ref<const Eigen::Vector3d>& ctrPt, double theta, double rad) {
	Eigen::Vector3d n = (IKSolve->tstRShldrSt - ctrPt).normalized();
	Eigen::Vector3d R(0, 0, 1), S = n.cross(R).normalized();
	double hlfRC = rad* std::cos(theta), hlfRS = rad* std::sin(theta);
	return ctrPt + (hlfRC * R) + (hlfRS * S);
}//getCirclEndPoint

void MyWindow::calcTrajPoints(eignVecTyp& _trajPts, eignVecVecTyp& vec, int numPts, double& globT) {
	int sIdx = (int)(numPts*globT);
	int nextIDX = (sIdx + 1) % vec.size();//1 bigger to catch final point - so that the trajectoires dont close if they have
	double t = (numPts*globT) - sIdx;

	_trajPts[0] = interpVec(vec[sIdx][0], vec[nextIDX][0], t);
	_trajPts[1] = interpVec(vec[sIdx][1], vec[nextIDX][1], t);

	int numTraj = numPts - 2;
	globT += tIncr[numTraj];
	if (globT > tBnds[numTraj]) {
		globT = 0;
		flags[doneTrajIDX] = true;
	}
}//calcTrajPoints

void MyWindow::calcCircleTrajPoints(eignVecTyp& _trajPts) {
	_trajPts[0] = getCirclEndPoint(IKSolve->drawCrclCtr, tVals[curTraj], IKSolve->params->IK_drawRad);
	_trajPts[1] = getCirclEndPoint(IKSolve->drawElbowCtr, tVals[curTraj], .25 * IKSolve->params->IK_drawRad);

	tVals[curTraj] += tIncr[curTraj];
	if (tVals[curTraj] > tBnds[curTraj]) {
		tVals[curTraj] = 0;
		flags[doneTrajIDX] = true;
	}
}//calcCircleTrajPoints

//write trajectory file to csv named _fname
void MyWindow::writeTrajCSVFile(const std::string& _fname, eignVecVecTyp& _trajAraAra) {
	int numRows = _trajAraAra.size(), numCols = _trajAraAra[0].size();
	std::stringstream ss;
	ss.str("");
	ss << csvFilePath << _fname << "Traj.csv";
	std::ofstream outFile;
	if (flags[debugIDX]) { std::cout << "Fully Qualified File name : " << ss.str() << "\tnumRows : " << numRows << "\tnumCols : " << numCols << std::endl; }
	outFile.open(ss.str());
	//output column marker names and whether fixed or not - 3 cols per value (x,y,z)
	ss.str("");
	std::array<std::string, 3> const dims{ " X"," Y"," Z" };

	//column names
	ss << trkedMrkrNames[0] << dims[0];
	for (int i = 1; i < 3; ++i) { ss << "," << trkedMrkrNames[0] << dims[i]; }//first set without leading comma
	for (int col = 1; col < numCols; ++col) {
		for (int i = 0; i < 3; ++i) { ss << "," << trkedMrkrNames[col] << dims[i]; }
	}
	ss << std::endl;
	if (flags[debugIDX]) { std::cout << ss.str(); }	//debug
	outFile << ss.str();

	ss.str("");
	//fixed or not
	std::string isFixed = ((*IKSolve->trkMarkers)[trkedMrkrNames[0]]->IsFixed() ? "true" : "false");
	ss << isFixed << "," << isFixed << "," << isFixed;
	for (int col = 1; col < numCols; ++col) {
		isFixed = ((*IKSolve->trkMarkers)[trkedMrkrNames[col]]->IsFixed() ? "true" : "false");
		ss << "," << isFixed << "," << isFixed << "," << isFixed;
	}
	ss << std::endl;
	if (flags[debugIDX]) { std::cout << ss.str(); }	//debug
	outFile << ss.str();

	//traj data
	for (int row = 0; row < numRows; ++row) {
		writeTrajCSVFileRow(outFile, _trajAraAra[row]);
	}
}//writeTrajCSVFile

 //write a row == 1 frame of 1-mrkr-per-col traj data
void MyWindow::writeTrajCSVFileRow(std::ofstream& outFile, eignVecTyp& _trajAra) {
	int numCols = _trajAra.size();
	int numSigDigs = 9;
	std::stringstream ss;
	ss.str("");
	ss << buildStrFrmEigV3d(_trajAra[0], numSigDigs);
	for (int col = 1; col < numCols; ++col) { ss << "," << buildStrFrmEigV3d(_trajAra[col], numSigDigs); }
	ss << std::endl;
	if (flags[debugIDX]) { std::cout << ss.str(); }	//debug
	outFile << ss.str();
}

