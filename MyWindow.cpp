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
#if defined(_WIN32)
	#include <direct.h>		//code for windows
#else 
	#include <unistd.h>
	#include <sys/stat.h>   //code for non-Windows
#endif

#include <direct.h> 
#include <fstream>


using namespace gestureIKApp;
using namespace dart::gui;

static int screenCapCnt = 0, trainSymDatCnt = 0, trainLtrtCnt = 0, displayTmrCnt = 0, drawCnt = 0;

MyWindow::MyWindow(std::shared_ptr<IKSolver> _ikslvr) : SimWindow(),  IKSolve(_ikslvr), tVals(4), tBnds(4,1), tIncr(4,1), trainDataFileStrm(), comValsDataFileStrm(),
	curTrajDirName(""),  mTrajPoints(7), letters(0), curTraj(0), saveCOMVals(0), curClassName(trajNames[0]), curSymIDX(0), mrkrClr(0.95, 0.25, 0.25, 1.0),
	triCrnrs(0),  sqrCrnrs(0), starCrnrs(0),captCount(4,0), curStIdx(4,0), dataGenIters(4, 0), aspectRatio(1.0),
	skel_headSize(0,0,0), skel_headClr(0, 0, 0), skel_handSize(0, 0, 0), skel_handClr(0, 0, 0), curHandShape(0), flags(numFlags,false)
{
	//IKsolver instanced before window, so relevant components loaded into solver and then into window from solver TODO: change this?  
	skelPtr = IKSolve->getSkel();
	saveInitSkelVals();
	mDisplayTimeout = 10;
	setValsFromXml();

	tBnds[0] = DART_2PI;//circle bounds on t value
	//set init display vals
	//mTrackBall.setQuaternion(origTrackBallQ);
	//mZoom = IKSolve->params->origZoom;
	//mTrans = origMTrans;
	
	////set values used to map out trajectories to draw - needs to be owned by each symbol
	//IKSolve->setSampleCenters();

	//FOR SAMPLE TRAJS ONLY
	tIncr[0] = .2;//set to be .2 because it is a deltaTheta for the circle
	for (int i = 1; i < 4; ++i) {
		tIncr[i] = IKSolve->params->trajDesiredVel;// / (i + 2.0);//dividing by # of sides because interpolating between each pair of verts with a local t: 0->1, but treating all sets of edges as complete trajectory (i.e. glbl t 0->1)
	}
	//std::cout << "Per Frame ptr distance travelled calculated : " << IKSolve->params->trajDesiredVel << "\n";

	triCrnrs = regenCorners<3U>(triCrnrConsts, false, curStIdx[1]);
	sqrCrnrs = regenCorners<4U>(sqCrnrConsts, false, curStIdx[2]);
	starCrnrs = regenCorners<5U>(strCrnrConsts, false, curStIdx[3]);

	//following is for symbol generation, not used for letters
	////writes trajectory for circle 
	//curTraj = 0;//set to 1,2, 3 for tri/square/star
	//void(MyWindow::*trajFunc)(eignVecTyp&);
	//trajFunc = &MyWindow::calcCircleTrajPoints;
	//auto trajSeq = buildTrajSeq(trajFunc, curTraj, tVals[curTraj]);
	//writeTrajCSVFile(trajNames[curTraj], trajSeq);
	//end symbol gen

	////read all letter files and build trajectories
	//buildLetterList();
	//init traj dir name for test symbols
	curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
	//set refresh function
	glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
}

MyWindow::~MyWindow() {}

//set window size based on params value loaded from xml
void MyWindow::initCustWindow(std::string _winTtl) {
	//set motionblur flag and quality - needs to be called after mWorld is set
	//setMotionBlurQual();

	initWindow(IKSolve->params->win_Width, IKSolve->params->win_Height, _winTtl.c_str());
	//glutDisplayFunc(myRefresh);

	aspectRatio = static_cast<double>(mWinWidth) / static_cast<double>(mWinHeight);
}//initCustWindow
//testing overriding refresh setting in glutDisplayFunc
//void MyWindow::myRefresh() {
//	//static int count = 0;
//	//std::cout << "myRefresh " << count++ << std::endl;
//	//refresh();
//}

//handles only 1 file stream being written - use python script to split data into train/test sets
void MyWindow::openIndexFile(std::ofstream& strm, bool isCSV, bool append) {

	std::stringstream ss;
	ss << getFullBasePath(isCSV) << "GenTrainDataIndexFile.txt";
	const std::string tmp = ss.str();
	std::ios_base::openmode mode = (append ? std::fstream::app : std::fstream::out);
	strm.open(tmp.c_str(), mode);
	if (!strm.is_open()) {
		std::cout << tmp << " failed to open!\n";
	}
}//openIndexFile

 //get current trajectory's file directory
 //dataIterVal is value of current iteration of data generation
//or current symbol name if using letters - dataIterVal == -1 for letters, otherwise for debug symbols
std::string MyWindow::getCurTrajFileDir(int dataIterVal) {
	if (flags[useLtrTrajIDX]) {
		return curLetter->getSymbolFileName();
	}
	else {
		std::stringstream ss("");
		//<trajType>_<trajIter>
		ss << curClassName << "_" << dataIterVal;
		return ss.str();
	}
}

//set up necessary functionality and state to conduct randomized data collection
//this will entail building trajectories for triangle, square and star, starting at each vertex,
//with and without random shifts in end points,  [and moving in clockwise or ccw direction (TODO)]
void MyWindow::trainDatInitCol(bool isLtr) {
	flags[useLtrTrajIDX] = isLtr;
	flags[collectDataIDX] = true;
	//turn off all markers and trajectory displays
	flags[drawTrkMrkrsIDX] = false;
	flags[drawLtrTrajIDX] = false;
	mShowMarkers = false;
	
	if (flags[useLtrTrajIDX]) {
		curSymIDX = 0;
		curLtrIDX = 0;
		if (flags[testLtrQualIDX]){//testing letters, drawing many trajs simultaneously
			std::cout << "Testing letter trajectories.\n";
			//turn on for debugging
			flags[drawTrkMrkrsIDX] = true;
			flags[drawLtrTrajIDX] = true;
		}
		else {
			std::cout << "Initializing random data collection for Letter Trajectories - Opening index file.\n";
			if (IKSolve->params->saveScreenShots()) {	openIndexFile(trainDataFileStrm, false, false);	}
			if (IKSolve->params->saveHandCOMVals()) {	openIndexFile(comValsDataFileStrm, true, false);	}
		}
		//start capture
		trainLtrDatManageCol();
	}
	else {  //Test : original symbols - triangle, square, star
		openIndexFile(trainDataFileStrm, false, false);
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

//manage capture for all letters - handles transition from symbol to symbol, letter to letter
void MyWindow::trainLtrDatManageCol() {
	if (curLtrIDX >= letters.size()) {//we're finished with every letter
		resetCurVars();
		//in case any camera or skeleton settings have changed for randomization
		setCameraVals(origTrackBallQ, IKSolve->params->origZoom, origMTrans);
		resetSkelVals();
		curSymIDX = 0;
		if (!flags[testLtrQualIDX]){//not testing, so close training data filename listing file
			if (IKSolve->params->saveScreenShots()) { trainDataFileStrm.close(); }
			if (IKSolve->params->saveHandCOMVals()) { comValsDataFileStrm.close(); }
		}
		flags[testLtrQualIDX] = false;
		return;
	}
	if (!flags[testLtrQualIDX]) {
		std::cout << "Function : trainLtrDatManageCol : current letter : " << trainLtrtCnt++ << "\tcurLtrIDX : " << curLtrIDX << "\tcurSymbIDX : " << curSymIDX << "\n";
	}
	//need to cycle through all letters - call with idx of next symbol and letter to draw
	setDrawLtrOrSmpl(true, curLtrIDX,  curSymIDX);
	curTrajDirName = getCurTrajFileDir(-1);

	if (!flags[testLtrQualIDX]) {
		if (IKSolve->params->saveScreenShots()) { trainDatWriteIndexFile(trainDataFileStrm, curTrajDirName, false, curLtrIDX); }
		if (IKSolve->params->saveHandCOMVals()) { trainDatWriteIndexFile(comValsDataFileStrm, curTrajDirName, true, curLtrIDX); }
		mCapture = true;
	}

	//for next iteration
	//TODO get rid of curSymIDX - let each letter maintain count of how many symbols have been drawn
	++curSymIDX;
	bool doneWithLetter = (curSymIDX >= IKSolve->params->numTotSymPerLtr);
	//finished all symbols of this letter
	if (doneWithLetter) {
		++curLtrIDX;
		curSymIDX = 0;
	}
}//trainLtrDatManageCol

void MyWindow::setRandomValues() {
	//randomize orientation
	if (IKSolve->params->rndCamOrient()) { mTrackBall.setQuaternion(curLetter->curSymbol->rndVals->cameraRot); }
	if (IKSolve->params->rndCamLoc()) {
		//get values from curLetter
		mZoom = curLetter->curSymbol->rndVals->cameraZoom;
		//set mTrans with symbol's cameraTans
		mTrans = curLetter->curSymbol->rndVals->cameraTrans;
	}
	
	//random head width/height (ellispoid)
	if (IKSolve->params->rndHeadDims()) {
		Eigen::Vector3d newHeadSize(skel_headSize);
		for (unsigned int i = 0; i < 3; ++i) {
			newHeadSize(i) += newHeadSize(i)*curLetter->curSymbol->rndVals->rnd_headSize(i);
		}
		setShapeSize("h_head", 0, newHeadSize);
	}
	//random head color (avoid colors close to gray)
	if (IKSolve->params->rndHeadClr()) {
		setShapeClr("h_head", 0, curLetter->curSymbol->rndVals->rnd_headClr);
	}
	//random hand shape - ellipsoid or rectangular
	if (IKSolve->params->rndHandShape()){
		int hndShIdx = curLetter->curSymbol->rndVals->rnd_handShape;
		if (curHandShape != hndShIdx) {//if different, hide current, unhide new
			skelPtr->getBodyNode("h_hand_right")->getVisualizationShape(curHandShape)->setHidden(true);
			skelPtr->getBodyNode("h_hand_left")->getVisualizationShape(curHandShape)->setHidden(true);
			skelPtr->getBodyNode("h_hand_right")->getVisualizationShape(hndShIdx)->setHidden(false);
			skelPtr->getBodyNode("h_hand_left")->getVisualizationShape(hndShIdx)->setHidden(false);
			curHandShape = hndShIdx;
		}
	}
	//random hand width/depth/length
	if (IKSolve->params->rndHandDims()){
		Eigen::Vector3d newHandSize(skel_handSize);
		for (unsigned int i = 0; i < 3; ++i) {
			newHandSize(i) += newHandSize(i)*curLetter->curSymbol->rndVals->rnd_handSize(i);
		}
		setShapeSize("h_hand_right", curHandShape, newHandSize);
		setShapeSize("h_hand_left", curHandShape, newHandSize);
	}
	//random hand color
	if (IKSolve->params->rndHandClr()){
		setShapeClr("h_hand_right", curHandShape, curLetter->curSymbol->rndVals->rnd_handClr);
		setShapeClr("h_hand_left", curHandShape, curLetter->curSymbol->rndVals->rnd_handClr);
	}
}

//setup each new letter symbol or sample trajectory
void MyWindow::setDrawLtrOrSmpl(bool drawLtr, int idx, int symNum) {
	flags[useLtrTrajIDX] = drawLtr;
	flags[doneTrajIDX] = false;

	if (drawLtr) {													//drawing actual letter
		if ((!flags[testLtrQualIDX]) && (nullptr != curLetter)) {
			std::cout << "Wrote " << screenCapCnt << " .pngs of symbol traj : " << curLetter->getCurSymbolName() << "\n";
		}
		curLtrIDX = idx;
		curLetter = letters[idx];
		//only pre-solve IK if using motion blur; don't display if debugging only (other debug text makes it redundant)
		curLetter->buildSymbolAndSolveIK(symNum, IKSolve->params->useMotionBlur(), !flags[debugIDX]);		
		//set randomization values based on setting for current letter
		setRandomValues();
		curClassName = std::string(curLetter->ltrName);				//class name for test/train index file - use letter name not symbol name
	}
	else {														//drawing triangle/square/star example symbols
		curTraj = idx;
		tVals[idx] = 0;
		curClassName = std::string(trajNames[curTraj]);
		captCount[idx] = 0;
	}

	screenCapCnt = 0;												//since changing symbol, reset count of screen captures
}//setDrawLtrOrSmpl


//write line in index/label text file for each example
void MyWindow::trainDatWriteIndexFile(std::ofstream& outFile, const std::string& fileDir, bool isCSV, int cls) {
	std::stringstream ss;
	ss << curClassName << "/";
	if (isCSV) {	ss << "COMVals_" << fileDir<< ".csv";}		//each example is a csv file named "COMVals_<fileDir>.csv"
	else {			ss << fileDir;	}							//each example is a listing of images within filedir
	ss << " " << cls << "\n";
	if (flags[debugIDX]) { std::cout << ss.str(); }	//debug
	outFile << ss.str();
}

//build map of all available symbols, idxed by name
void MyWindow::buildLetterList() {
	letters.clear();
	for (unsigned int i = 0; i < 26; ++i) {
		std::string c(1, i + 97);
		//only place prototype letters are loaded from file 
		letters.push_back(std::make_shared<MyGestLetter>(c, i, IKSolve));
		GestIKParser::readGestLetterXML(letters[i]);
		std::cout << "Made letter : " << (*letters[i]) << " from file description.\n";
	}
}
void MyWindow::buildRandDbugLetterList() {
	for (int i = 0; i < letters.size(); ++i) {
		letters[i]->buildRandExSymbolTrajs(40);
		std::cout << "Made 20 Random versions of letter : " << (*letters[i]) << " for debug purposes.\n";
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
void MyWindow::timeStepping() {
	if (flags[initTrnDatCapIDX]) {//initialize trajectories for training/testing data capture
		flags[initTrnDatCapIDX] = false;
		trainDatInitCol(flags[useLtrTrajIDX]);
	}
	//if last time around we finished trajectory, reset values/recapture, etc, for next trajectory
	if (flags[doneTrajIDX]) { flags[doneTrajIDX] = false; endCapture(flags[useLtrTrajIDX]); }		//check at end of trajectory if attempting to save screen shot
	if (flags[useLtrTrajIDX]) {
		if (flags[pauseIKLtrIDX]) {//wait for 20 frames between draw frames			
			pauseCount = pauseCount + 1;
			if (pauseCount % 20 != 0) { return; }//exit without IK solving - pause between frames
			else { pauseCount = 0; }
		}
		//setup trajectories and then solve - executes IK to current trajectory location and returns true if done
		//flags[doneTrajIDX] = curLetter->curSymbol->setIKSkelPose(0, true);
		flags[doneTrajIDX] = curLetter->curSymbol->solve();
		if (mCapture && IKSolve->params->saveHandCOMVals()) {//capturing to CSV files - only for non-blur - capture hand and arm COM locs
			recSkelCOMVals();								//accumulate values
			if (flags[doneTrajIDX]) { saveSkelCOMVals(); }//write to disk
		}
		//if (flags[debugIDX]) { cout << "displayTimer : letter solve done : disp timr cnt" << (displayTmrCnt++) << " with draw count : " << drawCnt << "\n"; }
	}
	else {//IK on sample symbols
		switch (curTraj) {
		case 0: {		calcCircleTrajPoints(mTrajPoints);		break;		}
		case 1: {		calcTrajPoints(mTrajPoints, triCrnrs, (curTraj + 2), tVals[curTraj]);		break;		}
		case 2: {		calcTrajPoints(mTrajPoints, sqrCrnrs, (curTraj + 2), tVals[curTraj]);		break;		}
		case 3: {		calcTrajPoints(mTrajPoints, starCrnrs, (curTraj + 2), tVals[curTraj]);		break;		}
		}
		IKSolve->solve(mTrajPoints);
		//if (flags[debugIDX]) { cout << "displayTimer : IKSolve done : disp timr cnt" << (displayTmrCnt++) << " with draw count : " << drawCnt << "\n"; }
	}
}//timeStepping

void MyWindow::displayTimer(int _val) {
	//static int count = 0;
	//std::cout << "Display timer " << count++<< std::endl;
	//If motion blur, all skel poses are precalculated, and set during rendering to build blur image	
	if (!(IKSolve->params->useMotionBlur() && flags[useLtrTrajIDX])) { timeStepping(); }//NOTE no blur on sample symbols
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}//displayTimer

//render function for using motionblur - TODO can't do this here, need to figure blur externally
void MyWindow::renderBlur() {
	if (flags[initTrnDatCapIDX]) {//initialize trajectories for training/testing data capture
		flags[initTrnDatCapIDX] = false;
		trainDatInitCol(flags[useLtrTrajIDX]);
	}
	//check at end of trajectory if attempting to save screen shots	
	if (flags[doneTrajIDX]) { flags[doneTrajIDX] = false; endCapture(flags[useLtrTrajIDX]); return; }
	//TODO make mult weighted, e.g. to give skew blur?
	float mult = 1.0f / (IKSolve->params->mBlurPreFrames + IKSolve->params->mBlurPostFrames + 1);

	//motionblur frames is treated as offset to whichever current frame is being rendered to build blur
	//avoid if check for glAccum - 1st pass is GL_LOAD, subsequent is GL_ACCUM
	flags[doneTrajIDX] = curLetter->curSymbol->setIKSkelPose(-IKSolve->params->mBlurPreFrames, false);
	draw();
	glAccum(GL_LOAD, mult);
	for (int j = 1 - IKSolve->params->mBlurPreFrames; j <= IKSolve->params->mBlurPostFrames; ++j) { //either 1 (motionBlurFrames == 0) motion blur frames, 3, 5, 7, 9, etc 
		//instead of solving IK, need to just iterate through pose state array being sent to skeleton with appropriate offest for motionBlur
		flags[doneTrajIDX] = curLetter->curSymbol->setIKSkelPose(j, j == IKSolve->params->mBlurPostFrames);
		draw();
		glAccum(GL_ACCUM, mult);
	}
	endRender();
}//renderBlur

//override render originally in wind3d
void MyWindow::render() {
	//static int count = 0;
	//std::cout << "render " << count++ << std::endl;
	//motion blur has a different program flow than regular rendering - motion blur precalculates and stores all skel dofs for an entire trajectory before rendering any
	//and then sets the skeleton with the appropriate sequence of dofs for each end frame
	if (IKSolve->params->useMotionBlur() && flags[useLtrTrajIDX]) { renderBlur(); return; }
	if (mCapture && !IKSolve->params->saveScreenShots()) {//if capturing only trajectories, no need to render skel
		return;
	}
	draw();
	glAccum(GL_LOAD, 1.0f);
	endRender();
}//render

void MyWindow::endRender() {
	// Draw trackball indicator
	if (mRotate && !mCapture) { mTrackBall.draw(mWinWidth, mWinHeight); }
	// Clear and return the buffer
	glAccum(GL_RETURN, 1.0f);
	if (mCapture && IKSolve->params->saveScreenShots()) {
		//std::cout << std::endl;//to flush
		screenshot(); }//save screenshots on capture if not saving hand com or if specified to save both in XML
	//need to swap after capture TODO verify
	glutSwapBuffers();
}//endRender

//save screen space values of COM and COM vels for hand and forearm - save when skel is rendered
void MyWindow::recSkelCOMVals(){
	glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	Eigen::Vector3d resHand = skelPtr->getBodyNode("h_hand_right")->getCOM(), resArm = skelPtr->getBodyNode("h_forearm_right")->getCOM();
	Eigen::VectorXd resCoords(12);
	//map locs to screen space
	gluProject(resHand(0), resHand(1), resHand(2), model_view, projection, viewport, &resCoords(0), &resCoords(1), &resCoords(2));
	gluProject(resArm(0), resArm(1), resArm(2), model_view, projection, viewport, &resCoords(6), &resCoords(7), &resCoords(8));
	//handle vel in screen space - finite diff vel calc
	if (saveCOMVals.size() == 0) {//first pass
		resCoords.segment<3>(3) << 0, 0, 0;
		resCoords.segment<3>(9) << 0, 0, 0;
	}
	else {//subsequent passes
		Eigen::VectorXd oldRes = saveCOMVals.back();
		resCoords.segment<3>(3) << resCoords.segment<3>(0) - oldRes.segment<3>(0);
		resCoords.segment<3>(9) << resCoords.segment<3>(6) - oldRes.segment<3>(6);
	}
	saveCOMVals.push_back(resCoords);
}//recSkelCOMVals

//call at end of letter rendering if saving COM vals
void MyWindow::saveSkelCOMVals() {
	std::stringstream ss;

	ss << getFullBasePath(true) << "COMVals_" << curTrajDirName << ".csv";
	std::string fileName = ss.str();
	std::cout << "CSV File name : " << fileName << "\n";
	std::cout << std::endl;
	std::ofstream outFile;
	outFile.open(fileName);

	for (int i = 0; i < saveCOMVals.size(); ++i) {
		ss.str("");
		for (int j = 0; j < 12; ++j) {
			if ((j % 3) == 2) { continue; }					//skip z(depth) coord - irrelevant in screen space
			ss << saveCOMVals[i](j) << ", ";
		}
		ss << "\n";
		outFile << ss.str();
	}
	outFile.close();
	saveCOMVals.clear();
}//saveSkelCOMVals

//override draw in simwindow
void MyWindow::draw() {
///////////////////from render
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(mPersp, aspectRatio, 0.1, 10.0);
	gluLookAt(mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	initGL();
	mTrackBall.applyGLRotation();

	// Draw world origin indicator
	if (!mCapture){
		glEnable(GL_DEPTH_TEST);
		//glDisable(GL_TEXTURE_2D);
		glDisable(GL_LIGHTING);
		glLineWidth(2.0);
		if (mRotate || mTranslate || mZooming) {
			glColor3f(1.0f, 0.0f, 0.0f);
			glBegin(GL_LINES);
			glVertex3f(-0.1f, 0.0f, -0.0f);
			glVertex3f(0.15f, 0.0f, -0.0f);
			glEnd();

			glColor3f(0.0f, 1.0f, 0.0f);
			glBegin(GL_LINES);
			glVertex3f(0.0f, -0.1f, 0.0f);
			glVertex3f(0.0f, 0.15f, 0.0f);
			glEnd();

			glColor3f(0.0f, 0.0f, 1.0f);
			glBegin(GL_LINES);
			glVertex3f(0.0f, 0.0f, -0.1f);
			glVertex3f(0.0f, 0.0f, 0.15f);
			glEnd();
		}
	}

	glScalef(mZoom, mZoom, mZoom);
	glTranslatef(mTrans[0] * 0.001, mTrans[1] * 0.001, mTrans[2] * 0.001);
	initLights();
///////////////////end from render

	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//remove drawSkels() call
	skelPtr->draw(mRI);
	// Draw the markers on the skeleton.
	if (mShowMarkers) {
		skelPtr->drawMarkers(mRI, mrkrClr, false);
	}
	//draw tracked markers
	if (flags[drawTrkMrkrsIDX]) {		IKSolve->drawTrkMrkrs(mRI, true);	}

	if (flags[useLtrTrajIDX] && (nullptr != curLetter)) {
		//if show all trajs (show all symbols for a letter, for debugging) and showing letters and a current letter exists
		if (flags[showAllTrajsIDX]){		curLetter->drawSymbolTrajDist(mRI);} 
		else if (flags[drawLtrTrajIDX]) {	curLetter->drawLetter(mRI);	}
	} else if (flags[drawLtrTrajIDX]) {		drawCurTraj();		}//draw trajectory for sample symbol

	if (flags[debugIDX]) {
		glColor3f(1.0f, 0.0f, 0.0f);
		float ySt = .11f;
		for (int i = 0; i < 3; ++i) {
			dart::gui::drawStringOnScreen(0.02f, ySt, getCurrQuatAsRot(i));
			ySt -= .03f;
		}
		dart::gui::drawStringOnScreen(0.02f, ySt, getCurrLocAndQuat());
	}
	glEnable(GL_LIGHTING);
	//cout << "draw end : " << (drawCnt++) << "\n";
}//draw

std::string MyWindow::getCurrQuatAsRot(int row) {
	std::stringstream ss;
	ss << "Row " << row << " :  ["; //<< this->mTrackBall.getRotationMatrix().row(row) << "]";
	Eigen::Vector3d matRow = this->mTrackBall.getRotationMatrix().row(row);
	ss << std::fixed << std::setprecision(6) << matRow(0) << "   ";
	ss << std::fixed << std::setprecision(6) << matRow(1) << "   ";
	ss << std::fixed << std::setprecision(6) << matRow(2);
	ss << "]";
	return ss.str();
}

std::string MyWindow::getCurrLocAndQuat() {
	std::stringstream ss;
	ss << "Loc : " << buildStrFrmEigV3d(this->mTrans) << "     Zoom : " << (this->mZoom) << "     Quat : w:";
	ss << this->mTrackBall.getCurrQuat().w() << " | vec : (" << buildStrFrmEigV3d(this->mTrackBall.getCurrQuat().vec()) << ")\n";
	return ss.str();
}

bool MyWindow::startCapture() {
	if (letters.size() == 0) {
		std::cout << "No letters loaded, so aborting screen capture save\n";
		return false;
	}
	flags[initTrnDatCapIDX] = true;
	flags[useLtrTrajIDX] = true;
	std::stringstream ss;
	if (IKSolve->params->saveHandCOMVals()) {//save com/com vel csvs - make base directory for csvs
		ss << getFullBasePath(true);
		const std::string tmp1 = ss.str();
		bool made = makeDirectory(tmp1);
		if (!made) { std::cout << "Failed to make base COM CSV directory : " << tmp1 << "\n";	return false; }
		else { std::cout << "Made base COM CSV directory : " << tmp1 << "\n"; }
		ss.str("");
	}
	if (IKSolve->params->saveScreenShots()) {//save screen shots if not saving hand com vals or if saving screenshots always is enabled
		ss << getFullBasePath(false);
		const std::string tmp = ss.str();
		bool made = makeDirectory(tmp);
		if (!made) { std::cout << "Failed to make base letter directory : " << tmp << "\n";	return false; }
		else { std::cout << "Made base letter directory : " << tmp << "\n"; }
	}
	//turn off any tests that may have been initiated
	flags[testLtrQualIDX] = false;
	for (int i = 0; i < letters.size(); ++i) { letters[i]->setTestLtrQual(flags[testLtrQualIDX]); }
	std::cout << "Capture all letters with data type : " << DataType2str[IKSolve->params->dataType] << "\n";
	return true;
}//startCapture

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
	unsigned int keyVal = _key;
	if ((keyVal >= 65) && (keyVal <= 90)) {		//draw letter trajectories if any capital letter is selected
		int idx = keyVal - 65;
		if (letters.size() <= idx) {
			std::cout << "Insufficient letters loaded (" << letters.size() << ") to handle letter : "<< static_cast<char>(keyVal) <<" at idx : "<<idx<< std::endl;
			return;
		}
		//repeatedly pressing the same letter will cycle through all available symbols of that letter
		//TODO have this work only using To be coded debug array of example random trajs
		std::cout << "MyWindow::keyboard request draw letter : "<< static_cast<char>(keyVal) <<"\tsymbol " << curSymIDX << std::endl;
		setDrawLtrOrSmpl(true, idx, curSymIDX);
		curSymIDX = (curSymIDX + 1) % curLetter->getNumSymbols();
		return;
	} 
	switch (_key) {
		//1-4 are all sample trajectories - char '1' == 49
	case '1':
	case '2':
	case '3':
	case '4': {  // st trajectory to traj 1
		setDrawLtrOrSmpl(false, keyVal - 49);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "sample "<< trajNames[curTraj] << " trajectory"<< std::endl;
		break; }
	case '`': {         //reset trackball loc with origTrackBallQ and zoom with 1
		setCameraVals(origTrackBallQ, IKSolve->params->origZoom, origMTrans);
		resetSkelVals();
		break; }
	case 'a': {  // screen capture all letters (train and test)
		if (!startCapture()) { std::cout << std::endl; return; }
		std::cout << std::endl;//to flush

		//if (letters.size() == 0) {
		//	std::cout << "No letters loaded, so aborting screen capture save\n";
		//	return;
		//}
		//flags[initTrnDatCapIDX] = true;
		//flags[useLtrTrajIDX] = true;
		//std::stringstream ss;
		//if (IKSolve->params->saveHandCOMVals()) {//save com/com vel csvs - make base directory for csvs
		//	ss << getFullBasePath(true);
		//	const std::string tmp1 = ss.str();
		//	bool made = makeDirectory(tmp1);
		//	if (!made) { std::cout << "Failed to make base COM CSV directory : " << tmp1 << "\n";	return; }
		//	else { std::cout << "Made base COM CSV directory : " << tmp1 << "\n"; }
		//	ss.str("");
		//}
		//if (IKSolve->params->saveScreenShots()) {//save screen shots if not saving hand com vals or if saving screenshots always is enabled
		//	ss << getFullBasePath(false);
		//	const std::string tmp = ss.str();
		//	bool made = makeDirectory(tmp);
		//	if (!made) { std::cout << "Failed to make base letter directory : " << tmp << "\n";	return; }
		//	else { std::cout << "Made base letter directory : " << tmp << "\n"; }
		//}
		////turn off any tests that may have been initiated
		//flags[testLtrQualIDX] = false;
		//for (int i = 0; i < letters.size(); ++i) { letters[i]->setTestLtrQual(flags[testLtrQualIDX]); }
		//std::cout << "Capture all letters with data type : "<< DataType2str[IKSolve->params->dataType] << "\n";
		break; }
	case 'b': { // reload xml and build lists of letters and random letters
		//reload params from xml 
		reloadXml();
		//read all letter files and build trajectories
		buildLetterList();
		std::cout << std::endl;//to flush
		break; }
	case 'c': {  // screen capture
		flags[stCaptAtZeroIDX] = true;				//start capturing when trajectory gets to 0
		std::cout << "start capturing when trajectory gets to t == 0"<< std::endl;
		break; }
	case 'd': { // debug mode
		flags[debugIDX] = !flags[debugIDX];
		if (flags[debugIDX] && !flags[debugLtrsBuiltIDX]) {
			flags[debugLtrsBuiltIDX] = true;
			//buildRandDbugLetterList();
		}
		std::cout << "Debug Mode : " << (flags[debugIDX] ? "True\n" : "False\n") ;
		for (int i = 0; i < letters.size(); ++i) { letters[i]->setDebug( flags[debugIDX]); }	//debugIDX is 0 in every class
		std::cout << std::endl;
		break; }
	case 'e': { // show all letter trajectories
		flags[showAllTrajsIDX] = !flags[showAllTrajsIDX];
		std::cout << "Show all letter trajectories : " << (flags[showAllTrajsIDX] ? "True\n" : "False\n") ;
		for (int i = 0; i < letters.size(); ++i) { letters[i]->setShowAllTrajs( flags[showAllTrajsIDX]); }	//debugIDX is 0 in every class
		std::cout << std::endl;
		break; }
	case 'f': { //test all letters - same as screen cap except no IK and no file save
		if (letters.size() == 0) {
			std::cout << "No letters loaded, so aborting letter test\n";
			return;
		}
		flags[testLtrQualIDX] = !flags[testLtrQualIDX];
		std::cout<<"Test letter process : " << (flags[testLtrQualIDX] ? "True\n" : "False\n") ;
		if (flags[testLtrQualIDX]) {//turning on
			flags[initTrnDatCapIDX] = true;
			flags[useLtrTrajIDX] = true;
		}
		for (int i = 0; i < letters.size(); ++i) { letters[i]->setTestLtrQual(flags[testLtrQualIDX]); }
		std::cout << std::endl;
		break;}
	case 'g' : {//build debug letter lists for visualizing 
		if (letters.size() == 0) {
			std::cout << "No letters loaded, so can't build randomized versions.\n";
			return;
		}
		std::cout << "Building example randomized versions of letters \n";
		buildRandDbugLetterList();
		std::cout << std::endl;
		break; }
	case 'p' : {  // pause between IK frames for debugging
		flags[pauseIKLtrIDX] = !flags[pauseIKLtrIDX];
		std::cout << "Pause between IK frames : " << (flags[pauseIKLtrIDX] ? "True\n" : "False\n") ;
		std::cout << std::endl;
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

//save initial value for particular shape
void MyWindow::saveInitShapeVals(const std::string& nodeName, int visIDX, Eigen::Ref<Eigen::Vector3d> _destSize, Eigen::Ref<Eigen::Vector3d> _destClr) {
	Eigen::Vector3d tmp(0, 0, 0);
	dart::dynamics::ShapePtr _bodyShape = skelPtr->getBodyNode(nodeName)->getVisualizationShape(visIDX);
	//must cast to appropriate child class
	switch (_bodyShape->getShapeType()) {
		case dart::dynamics::Shape::BOX: {
			const dart::dynamics::BoxShape* box1 = static_cast<const dart::dynamics::BoxShape*>(_bodyShape.get());
			tmp = box1->getSize();
			break;
		}
		case dart::dynamics::Shape::ELLIPSOID: {
			const dart::dynamics::EllipsoidShape* ellipsoid1 = static_cast<const dart::dynamics::EllipsoidShape*>(_bodyShape.get());
			tmp = ellipsoid1->getSize();
			break;
		}
		default: {}//if not either shape then problem, do nothing
	}
	_destSize << tmp;
	_destClr = _bodyShape->getColor();
}//saveInitSkelShapeSize

void MyWindow::addSphereHand(const std::string& nodeName) {
	dart::dynamics::ShapePtr _handShape = skelPtr->getBodyNode(nodeName)->getVisualizationShape(0);		//get box shape of hand
	const dart::dynamics::BoxShape* box1 = static_cast<const dart::dynamics::BoxShape*>(_handShape.get());
	Eigen::Vector3d sz (box1->getSize());
	sz *= 1.1;
	std::shared_ptr<dart::dynamics::Shape> shape (new dart::dynamics::EllipsoidShape(sz));
	skelPtr->getBodyNode(nodeName)->addVisualizationShape(shape);
	shape->setColor(box1->getColor());
	shape->setLocalTransform(box1->getLocalTransform());
	//add vis shape, now hide it
	skelPtr->getBodyNode(nodeName)->getVisualizationShape(1)->setHidden(true);
}

//save initial skeleton values 
void MyWindow::saveInitSkelVals() {
	if (nullptr != skelPtr) {
		//head initial values
		saveInitShapeVals("h_head", 0, skel_headSize, skel_headClr);
		//hand initial values - use right hand for base
		saveInitShapeVals("h_hand_right", 0, skel_handSize, skel_handClr);
		//add spherical visualization shapes of the same dimensions to both hands and hide them
		addSphereHand("h_hand_right");
		addSphereHand("h_hand_left");
	}
}//saveInitSkelVals

////set motionBlurFreq based on motionblur quality specified in IKParams
//void MyWindow::setMotionBlurQual() {
//	flags[useMotionBlurIDX] = IKSolve->params->useMotionBlur();
//	//0 or OOB == no motion blur
//	motionBlurFrames = 0;
//	if ((flags[useMotionBlurIDX]) && (IKSolve->params->motionBlurQual > 0)){// && (IKSolve->params->motionBlurQual <= 5)) { 
//		//varies from 1 to 5, with 1 == low motion blur and 5 == highest quality 
//		//motionBlurFreq = std::min(motionBlurFreq, (1 << (5 - IKSolve->params->motionBlurQual)));
//		motionBlurFrames = IKSolve->params->motionBlurQual;
//	}
//}//setMotionBlurQual

void MyWindow::setShapeSize(const std::string& nodeName, int visIDX, const Eigen::Ref<const Eigen::Vector3d>& _origSize) {
	dart::dynamics::ShapePtr _bodyShape = skelPtr->getBodyNode(nodeName)->getVisualizationShape(0);
	//must cast to appropriate child class
	switch (_bodyShape->getShapeType()) {
	case dart::dynamics::Shape::BOX: {
		dart::dynamics::BoxShape* box1 = static_cast<dart::dynamics::BoxShape*>(_bodyShape.get());
		box1->setSize(_origSize);
		break;
	}
	case dart::dynamics::Shape::ELLIPSOID: {
		dart::dynamics::EllipsoidShape* ellipsoid1 = static_cast<dart::dynamics::EllipsoidShape*>(_bodyShape.get());
		ellipsoid1->setSize(_origSize);
		break;
	}
	default: {}//if not either shape then problem, do nothing
	}
}//setShapeSize

void MyWindow::setShapeClr(const std::string& nodeName, int visIDX, const Eigen::Ref<const Eigen::Vector3d>& _origClr) {
	skelPtr->getBodyNode(nodeName)->getVisualizationShape(visIDX)->setColor(Eigen::Vector3d(_origClr));
}//setShapeClr


//reset all skeleton values to initial values
void MyWindow::resetSkelVals() {
	if (nullptr != skelPtr) {
		//reset head initial values
		setShapeSize("h_head", 0, skel_headSize);
		setShapeClr("h_head", 0, skel_headClr);
		//reset hand initial values - TODO reset shape first
		for (unsigned int i = 0; i < 2; ++i) {
			setShapeSize("h_hand_right", i, skel_handSize);
			setShapeClr("h_hand_right", i, skel_handClr);
			setShapeSize("h_hand_left", i, skel_handSize);
			setShapeClr("h_hand_left", i, skel_handClr);
			skelPtr->getBodyNode("h_hand_right")->getVisualizationShape(i)->setHidden(i);
			skelPtr->getBodyNode("h_hand_left")->getVisualizationShape(i)->setHidden(i);
		}
		curHandShape = 0;
		////spherical shape hand - hide initially
		//setShapeSize("h_hand_right", 1, skel_handSize);
		//setShapeClr("h_hand_right", 1, skel_handClr);
		//setShapeSize("h_hand_left", 1, skel_handSize);
		//setShapeClr("h_hand_left", 1, skel_handClr);
		////hide ellipsoid hand
		//skelPtr->getBodyNode("h_hand_right")->getVisualizationShape(1)->setHidden(true);
		//skelPtr->getBodyNode("h_hand_left")->getVisualizationShape(1)->setHidden(true);

	}
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

bool MyWindow::makeDirectory(const std::string& tmp) {
	int nError = 0;
#if defined(_WIN32)
	nError = _mkdir(tmp.c_str()); // can be used on Windows
#else 
	nError = mkdir(tmp.c_str(), 0733); // can be used on non-Windows
#endif
	if ((nError != 0) && (nError != -1)) {//-1 is exists already
		std::cout << "Error attempting to create path : " << tmp << "\terror : " << nError << std::endl;
		return false;
	}
	return true;
}

//get file name for screen shot based on current trajectory name and count of frames
std::string MyWindow::getScreenCapDirFileName() {
	std::stringstream ss;
	ss << getFullBasePath(false) << curTrajDirName;
	const std::string tmp = ss.str();

	bool made = makeDirectory(tmp);

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
		std::cout << "lodepng error " << result << ": " << lodepng_error_text(result) << "\n";
		return false;
	}
	else {
		screenCapCnt++;
		//std::cout << "wrote screenshot " << fileName << "\n";
		return true;
	}
}//screenshot

void MyWindow::drawAxes(const Eigen::Ref<const Eigen::Vector3d>& axesLoc, float len, bool altColor) {
	float col = (altColor ? .5f : 0);//allow for alternate color axes x:orange, y:cyan, z:purple
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

void MyWindow::drawCurTraj() {
	eignVecTyp drawPoints(7);
	switch (curTraj) {
	case 0: {		break;		}
	case 1: {	for (double t = 0; t < 1.0; t += .01) { calcTrajPoints(drawPoints, triCrnrs, 3, t); drawPoint(drawPoints[0]); }	break;		}
	case 2: {	for (double t = 0; t < 1.0; t += .01) { calcTrajPoints(drawPoints, sqrCrnrs, 4, t); drawPoint(drawPoints[0]); }		break;		}
	case 3: {	for (double t = 0; t < 1.0; t += .01) { calcTrajPoints(drawPoints, starCrnrs, 5, t);  drawPoint(drawPoints[0]); }	break;		}
	}
}
 //manage the process of writing all training and testing data - call whenever a trajectory has been captured (from checkCapture() )
 //ONLY FOR SAMPLE SYMBOLS (triangle, square, star) that consist of single trajectories!
void MyWindow::trainSymDatManageCol() {
	//if (flags[debugIDX]) { cout << "Start trainSymDatManageCol : " << trainSymDatCnt << "\n"; }
	//call first time right after init
	//flags[collectDataIDX] pre-empts flags[stCaptAtZeroIDX] so needs to set mCapture
	//regerenate all endpoints - we start on non-random data - for current trajectory, make #verts trajectories starting on each vert without random, before using random trajectories
	regenerateSampleData(dataGenIters[curTraj] >= (curTraj + 2));

	//global name of directory to write to, of form : <trajType>_<trajIter> --> add _<frame#>.png for file name
	curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);

	//need to write entry into index file
	trainDatWriteIndexFile(trainDataFileStrm, curTrajDirName, false, curTraj);
	//after drawing all frames of a trajectory the following needs to be reset (i.e. every call of this function) - this is frame # of capture of current trajectory
	captCount[curTraj] = 0;
	//...and need to advance data counts
	dataGenIters[curTraj]++;

	//after transitioning to a different symbol, the following needs to be reset
	if (dataGenIters[curTraj] > IKSolve->params->numTotSymPerLtr) {
		dataGenIters[curTraj] = 0;
		setDrawLtrOrSmpl(false, (curTraj + 1) % trajNames.size());		//iterate to next trajectory if we've captured sufficient samples
		dataGenIters[curTraj] = 0;
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "Test/train data gen transition to sample " << trajNames[curTraj] << " trajectory\n";
	}
	if (curTraj != 0) { mCapture = true; }		//repeat for next trajectory type
	else {//at curTraj == 0 we have completed all data gen, and should stop - reset all important stuff to circle as if key 1 was pressed
		flags[collectDataIDX] = false;
		trainDataFileStrm.close();
		setDrawLtrOrSmpl(false, 0);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "Finished generating test/train data, reset to sample " << trajNames[curTraj] << " trajectory\n";
	}
	//if (flags[debugIDX]) { cout << "End trainSymDatManageCol : " << trainSymDatCnt++ << "\n"; }
}

//write trajectory file to csv named _fname
void MyWindow::writeTrajCSVFile(const std::string& _fname, eignVecVecTyp& _trajAraAra) {
	int numRows = _trajAraAra.size(), numCols = _trajAraAra[0].size();
	std::stringstream ss;
	ss << csvFilePath << _fname << "Traj.csv";
	std::ofstream outFile;
	if (flags[debugIDX]) { std::cout << "Fully Qualified File name : " << ss.str() << "\tnumRows : " << numRows << "\tnumCols : " << numCols << "\n"; }
	outFile.open(ss.str());
	//output column marker names and whether fixed or not - 3 cols per value (x,y,z)
	ss.str("");
	std::array<std::string, 3> const dims{ " X"," Y"," Z" };

	//column names
	ss << IKSolve->trkedMrkrNames[0] << dims[0];
	for (int i = 1; i < 3; ++i) { ss << "," << IKSolve->trkedMrkrNames[0] << dims[i]; }//first set without leading comma
	for (int col = 1; col < numCols; ++col) {
		for (int i = 0; i < 3; ++i) { ss << "," << IKSolve->trkedMrkrNames[col] << dims[i]; }
	}
	ss << "\n";
	if (flags[debugIDX]) { std::cout << ss.str(); }	//debug
	outFile << ss.str();

	ss.str("");
	//fixed or not
	std::string isFixed = ((*IKSolve->trkMarkers)[IKSolve->trkedMrkrNames[0]]->IsFixed() ? "true" : "false");
	ss << isFixed << "," << isFixed << "," << isFixed;
	for (int col = 1; col < numCols; ++col) {
		isFixed = ((*IKSolve->trkMarkers)[IKSolve->trkedMrkrNames[col]]->IsFixed() ? "true" : "false");
		ss << "," << isFixed << "," << isFixed << "," << isFixed;
	}
	ss << "\n";
	if (flags[debugIDX]) { std::cout << ss.str(); }	//debug
	outFile << ss.str();

	//traj data
	for (int row = 0; row < numRows; ++row) {
		writeTrajCSVFileRow(outFile, _trajAraAra[row]);
	}
	outFile.close();
}//writeTrajCSVFile

 //write a row == 1 frame of 1-mrkr-per-col traj data
void MyWindow::writeTrajCSVFileRow(std::ofstream& outFile, eignVecTyp& _trajAra) {
	int numCols = _trajAra.size();
	int numSigDigs = 9;
	std::stringstream ss;
	ss << buildStrFrmEigV3d(_trajAra[0], numSigDigs);
	for (int col = 1; col < numCols; ++col) { ss << "," << buildStrFrmEigV3d(_trajAra[col], numSigDigs); }
	ss << "\n";
	if (flags[debugIDX]) { std::cout << ss.str(); }	//debug
	outFile << ss.str();
}

