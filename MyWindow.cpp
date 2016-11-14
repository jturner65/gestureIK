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

using namespace std;
using namespace Eigen;
using namespace gestureIKApp;
using namespace dart::gui;


MyWindow::MyWindow(std::shared_ptr<IKSolver> _ikslvr) : SimWindow(),  IKSolve(_ikslvr), tVals(4), tBnds(4,1), tIncr(4,1), testOutFile(), trainOutFile(),
	curTrajDirName(""),  mTrajPoints(7), letters(0), curTraj(0), curTrajStr(trajNames[0]),
	triCrnrs(0),  sqrCrnrs(0), starCrnrs(0),captCount(4,0), curStIdx(4,0), dataGenIters(4, 0),
	flags(numFlags,false), normdist(nullptr){
	tBnds[0] = DART_2PI;//circle bounds on t value
	//set init display vals
	mDisplayTimeout = 10;				//set to be faster frame rate
	mTrackBall.setQuaternion(origTrackBallQ);
	mZoom = origZoom;
	mTrans = origMTrans;

	//start trajectory at initial position of hand
	skelPtr = IKSolve->getSkel();
	
	std::cout << "In MyWindow : Right arm reach : " << IKSolve->reach << " center " << buildStrFrmEigV3d(IKSolve->drawCrclCtr) << " elbow center " << buildStrFrmEigV3d(IKSolve->drawElbowCtr) <<" and rad of test circle "<< IKSolve->params->IK_drawRad<<"\n";
	//calcCircleTrajPoints(mTrajPoints);
	//fixed points
	int numMovePts = trkedMrkrNames.size() - fixedMrkrNames.size();
	mTrajPoints[0] << IKSolve->drawCrclCtr;
	mTrajPoints[1] << IKSolve->drawElbowCtr;
	//target positions for all fixed markers
	for (int i = 0; i < fixedMrkrNames.size(); ++i) {
		mTrajPoints[i + numMovePts] = skelPtr->getMarker(fixedMrkrNames[i])->getWorldPosition();
	}
	IKSolve->solve(mTrajPoints);
	//recalc elbow center after IKing ptr to circle center location
	IKSolve->updateElbowLoc();

	tIncr[0] = .2;//set to be .2 because it is a deltaTheta for the circle
	for (int i = 1; i < 4; ++i) {
		tIncr[i] = IKSolve->params->trajDesiredVel / (i + 2.0);//dividing by # of sides because interpolating between each pair of verts with a local t: 0->1, but treating all sets of edges as complete trajectory (i.e. glbl t 0->1)
	}
	cout << "Per Frame ptr distance travelled calculated : " << IKSolve->params->trajDesiredVel << "\n";

	triCrnrs = regenCorners<3U>(triCrnrConsts, false, curStIdx[1]);
	sqrCrnrs = regenCorners<4U>(sqCrnrConsts, false, curStIdx[2]);
	starCrnrs = regenCorners<5U>(strCrnrConsts, false, curStIdx[3]);

	////writes trajectory for circle
	//void(MyWindow::*trajFunc)(eignVecTyp&);
	//trajFunc = &MyWindow::calcCircleTrajPoints;
	//auto trajSeq = buildTrajSeq(trajFunc, 0, tVals[0]);
	//writeTrajCSVFile(trajNames[curTraj], trajSeq);
	
	////writes 1st trajectory for triangle, square, star
	//trajFunc = &MyWindow::calcTriangleTrajPoints;
	//trajSeq = buildTrajSeq(trajFunc, 1, tVals[1]);
	//writeTrajCSVFile(trajNames[1], trajSeq);

	//trajFunc = &MyWindow::calcSquareTrajPoints;
	//trajSeq = buildTrajSeq(trajFunc, 2, tVals[2]);
	//writeTrajCSVFile(trajNames[2], trajSeq);

	//trajFunc = &MyWindow::calcStarTrajPoints;
	//trajSeq = buildTrajSeq(trajFunc, 3, tVals[3]);
	//writeTrajCSVFile(trajNames[3], trajSeq);

	//read all letter files and build trajectories
	buildLetterList();
	//normal distribution randomiser
	normdist = make_shared<normal_distribution<double> >(0, 1.0);
	
	curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
	//set refresh function
	glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
}

MyWindow::~MyWindow() {}

//project points at certain t (== theta) in trajectory upon a plane described by normal n, inscribed within a circle of radius 2*rad, centered at ctrPt
//plane normal always points back to tstRShldrSt
Eigen::Vector3d MyWindow::getCirclEndPoint(const Eigen::Ref<const Eigen::Vector3d>& ctrPt, double theta, double rad) {
	Eigen::Vector3d n = (IKSolve->tstRShldrSt - ctrPt).normalized();
	Eigen::Vector3d R(0, 0, 1), S = n.cross(R).normalized();
	double hlfRC = rad* std::cos(theta), hlfRS = rad* std::sin(theta);
	return ctrPt + (hlfRC * R) + (hlfRS * S);
}

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
}

void MyWindow::calcCircleTrajPoints(eignVecTyp& _trajPts) {
	_trajPts[0] = getCirclEndPoint(IKSolve->drawCrclCtr, tVals[curTraj], IKSolve->params->IK_drawRad);
	_trajPts[1] = getCirclEndPoint(IKSolve->drawElbowCtr, tVals[curTraj], .25 * IKSolve->params->IK_drawRad);

	tVals[curTraj] += tIncr[curTraj];
	if (tVals[curTraj] > tBnds[curTraj]) {
		tVals[curTraj] = 0;
		flags[doneTrajIDX] = true;
	}
}

void MyWindow::openIndexFile(bool isTrain, std::ofstream& strm, const std::string& filePrefix) {
	stringstream ss;
	ss.str("");
	ss << framesFilePath << filePrefix << (isTrain ? "TrainDataIndexFile.txt" : "TestDataIndexFile.txt");
	const std::string tmp = ss.str();
	strm.open(tmp.c_str());
	if (!strm.is_open()) {
		cout << tmp << " failed to open!\n";
	}
}


//set up necessary functionality and state to conduct randomized data collection
//this will entail building trajectories for triangle, square and star, starting at each vertex,
//with and without random shifts in end points,  [and moving in clockwise or ccw direction (TODO)]
void MyWindow::trainDatInitCol() {
	cout << "Initializing random data collection for "<<((flags[useLtrTrajIDX]) ? "Letter Trajectories." : "Symbols." )<<"\n";
	flags[collectDataIDX] = true;
	//turn off all markers and trajectory displays
	flags[drawTrkMrkrsIDX] = false;
	flags[drawLtrTrajIDX] = false;
	mShowMarkers = false;

	openIndexFile(false, testOutFile, "");
	openIndexFile(false, trainOutFile, "");
	//stringstream ss;
	//ss.str("");
	//ss << framesFilePath << "TestDataIndexFile.txt";
	//const std::string tmp = ss.str();
	//testOutFile.open(tmp.c_str());

	//if (!testOutFile.is_open()) {
	//	cout << "testOutFile failed to open!\n";
	//}

	//ss.str("");
	//ss << framesFilePath << "TrainDataIndexFile.txt";
	//const std::string tmp2 = ss.str();
	//trainOutFile.open(tmp2.c_str());
	//if (!trainOutFile.is_open()) {
	//	cout << "trainOutFile failed to open!\n";
	//}
	 

	//initialize starting idxs, t values and counts of training and testing data for all trajectories
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
}//trainDatInitCol

//get current trajectory's file directory
//dataIterVal is value of current iteration of data generation
std::string MyWindow::getCurTrajFileDir(int dataIterVal) {
	//if(){
	//	return curLetter->
	//}

	stringstream ss("");
	//<trajType>_<trajIter>
	ss << curTrajStr << "_" << dataIterVal;
	return ss.str();
}

//manage the process of writing all training and testing data - call whenever a trajectory has been captured (from checkCapture() )
//ONLY FOR SAMPLE SYMBOLS that consist of single trajectories!
void MyWindow::trainSymDatManageCol() {
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
		setDrawLtrOrSmpl(false, (curTraj + 1) % 4);
		dataGenIters[curTraj] = 0;
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "Test/train data gen transition to sample " << trajNames[curTraj] << " trajectory\n";
	}
	if (curTraj != 0) { mCapture = true; }		//repeat for next trajectory type
	else {//at curTraj == 0 we have completed all data gen, and should stop - reset all important stuff to circle as if key 1 was pressed
		flags[collectDataIDX] = false;
		testOutFile.close();
		trainOutFile.close();
		setDrawLtrOrSmpl(false, 0);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "Finished generating test/train data, reset to sample " << trajNames[curTraj] << " trajectory\n";
	}	
}

//write line in index/label text file for each example
void MyWindow::trainDatWriteIndexFile(std::ofstream& outFile, const std::string& fileDir, int cls) {
	stringstream ss;
	ss.str("");
	ss << curTrajStr<<"/"<<fileDir << " " << cls << "\n";
	if (flags[debugIDX]) { std::cout << ss.str(); }	//debug
	outFile << ss.str();
}


//build map of all available symbols, idxed by name
void MyWindow::buildLetterList() {
	letters.clear();
	int ltrsToDo = 26;		//26;
	for (int i = 0; i < ltrsToDo; ++i) {
		string c(1, i + 97);
		letters.push_back(std::make_shared<MyGestLetter>(c));
		letters[i]->setSolver(IKSolve);
		GestIKParser::readGestLetterXML(letters[i]);
		cout << "Made letter : " << (*letters[i]) << "\n";
	}
}

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

//write trajectory file to csv named _fname
void MyWindow::writeTrajCSVFile(const std::string& _fname, eignVecVecTyp& _trajAraAra) {
	int numRows = _trajAraAra.size(), numCols = _trajAraAra[0].size();
	stringstream ss;
	ss.str("");
	ss << csvFilePath << _fname << "Traj.csv";
	std::ofstream outFile;
	if (flags[debugIDX]) { std::cout << "Fully Qualified File name : " << ss.str() << "\tnumRows : " << numRows << "\tnumCols : " << numCols << "\n"; }
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
	ss << "\n";
	if (flags[debugIDX]){ std::cout << ss.str(); }	//debug
	outFile << ss.str();

	ss.str("");
	//fixed or not
	std::string isFixed = ((*IKSolve->trkMarkers)[trkedMrkrNames[0]]->IsFixed() ? "true" : "false");
	ss << isFixed << "," << isFixed << "," << isFixed;
	for (int col = 1; col < numCols; ++col) {
		isFixed = ((*IKSolve->trkMarkers)[trkedMrkrNames[col]]->IsFixed() ? "true" : "false");
		ss << "," << isFixed << "," << isFixed << "," << isFixed;
	}
	ss << "\n";
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
	stringstream ss;
	ss.str("");
	ss << buildStrFrmEigV3d(_trajAra[0], numSigDigs);
	for (int col = 1; col < numCols; ++col) { ss << "," << buildStrFrmEigV3d(_trajAra[col], numSigDigs); }
	ss << "\n";
	if (flags[debugIDX]) { std::cout << ss.str(); }	//debug
	outFile << ss.str();
}

void MyWindow::displayTimer(int _val){
	if (flags[useLtrTrajIDX]) {
		//cout << "TODO : IK on letter " << curLetter->ltrName << " symbol idx : "<< curLetter->curSymbol <<"\n";
		flags[doneTrajIDX] = curLetter->solve();
	}
	else {
		switch (curTraj) {
		case 0: {		calcCircleTrajPoints(mTrajPoints);		break;		}
		case 1: {		calcTrajPoints(mTrajPoints, triCrnrs, (curTraj + 2), tVals[curTraj]);		break;		}
		case 2: {		calcTrajPoints(mTrajPoints, sqrCrnrs, (curTraj + 2), tVals[curTraj]);		break;		}
		case 3: {		calcTrajPoints(mTrajPoints, starCrnrs, (curTraj + 2), tVals[curTraj]);		break;		}
		}
		IKSolve->solve(mTrajPoints);
	}
	if (flags[doneTrajIDX]) { flags[doneTrajIDX] = false; checkCapture(flags[useLtrTrajIDX]); }
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
}

std::string MyWindow::getCurrLocAndQuat() {
	stringstream ss;
	ss.str("");
	ss << "Loc : " << buildStrFrmEigV3d(this->mTrans) << "\tQuat : w:" << this->mTrackBall.getCurrQuat().w() << " vec : (" << buildStrFrmEigV3d(this->mTrackBall.getCurrQuat().vec()) << ")\n";
	return ss.str();
}

void MyWindow::drawSkels() {
	//for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++) {
	//	mWorld->getSkeleton(i)->draw(mRI);
	//}
	skelPtr->draw(mRI);
	// Draw the markers on the skeleton.
	if (mShowMarkers) {
		Vector4d color;
		color << 0.95, 0.25, 0.25, 1.0;
		skelPtr->drawMarkers(mRI, color, false);
	}
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
	unsigned int keyVal = _key;
	if ((keyVal >= 65) && (keyVal <= 90)) {		//draw letter trajectories if any capital letter is selected
		int idx = keyVal - 65;
		setDrawLtrOrSmpl(true, idx);
		return;
	} 
	switch (_key) {
	case '1': {  // st trajectory to traj 1
		setDrawLtrOrSmpl(false, 0);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "sample "<< trajNames[curTraj] << " trajectory\n";
		break; }
	case '2': {  // st trajectory to traj 2 tTri, tSqr, tStar,
		setDrawLtrOrSmpl(false, 1);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "sample " << trajNames[curTraj] << " trajectory\n";
		break; }
	case '3': {  // st trajectory to traj 3
		setDrawLtrOrSmpl(false, 2);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "sample " << trajNames[curTraj] << " trajectory\n";
		break; }
	case '4': {  // st trajectory to traj 4
		setDrawLtrOrSmpl(false, 3);
		curTrajDirName = getCurTrajFileDir(dataGenIters[curTraj]);
		std::cout << "sample " << trajNames[curTraj] << " trajectory\n";
		break; }
	case '`':          //reset trackball loc with origTrackBallQ and zoom with 1
		mTrackBall.setQuaternion(origTrackBallQ);
		mZoom = origZoom;
		mTrans = origMTrans;
		break;
	//case 'p': { // playBack
	//	mPlay = !mPlay;
	//	if (mPlay) {
	//		mSimulating = false;
	//	}
	//	break;}
 //   case '[': {  // step backward
	//	if (!mSimulating) {
	//		mPlayFrame--;
	//		if (mPlayFrame < 0){mPlayFrame = 0;}
	//		glutPostRedisplay();
	//	}
	//	break;}
 //   case ']': { // step forwardward
	//	if (!mSimulating) {
	//		mPlayFrame++;
	//		if (mPlayFrame >= mWorld->getRecording()->getNumFrames()){mPlayFrame = 0;}
	//		glutPostRedisplay();
	//	}
	//	break;}
	case 'c': {  // screen capture
		flags[stCaptAtZeroIDX] = true;				//start capturing when trajectory gets to 0
		cout << "start capturing when trajectory gets to t == 0\n.";
		break; }
	case 'd': { // debug mode
		flags[debugIDX] = !flags[debugIDX];
		for (int i = 0; i < letters.size(); ++i) {		letters[i]->setSymbolFlags(debugIDX, flags[debugIDX]);		}	//debugIDX is 0 in every class
		break; }
	case 'r': {//randomize
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
		trainDatInitCol();
		break; }
	case 'y': {//un-randomize
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
	//TODO query current symbol being displayed
	stringstream ss;
	ss.str("");
	ss << framesFilePath << curTrajDirName;
	const std::string tmp = ss.str();

	int nError = 0;
#if defined(_WIN32)
	nError = _mkdir(tmp.c_str()); // can be used on Windows
#else 
	nError = mkdir(tmp.c_str(), 0733); // can be used on non-Windows
#endif
	if ((nError != 0) && (nError != -1)) {//-1 is exists already
		cout << "Error attempting to create path : " << tmp << "\terror : "<<nError<<"\n";
	}

	ss.str("");
	char buf[5];
	sprintf(buf, "%.4d", captCount[curTraj]++);
	ss << tmp <<"/"<< curTrajDirName<<"."<< buf << ".png";
	return ss.str();
}

bool MyWindow::screenshot() {
	
	const std::string tmp = getScreenCapDirFileName();
	const char* fileName = tmp.c_str();

	int tw = glutGet(GLUT_WINDOW_WIDTH);
	int th = glutGet(GLUT_WINDOW_HEIGHT);
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
		std::cout << "wrote screenshot " << fileName << "\n";
		return true;
	}
}//screenshot
void MyWindow::drawAxes(const Vector3d& axesLoc, float len, bool altColor) {
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
	drawAxes(Vector3d(0, 0, 0), .1f, false);
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


