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

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

#include "apps/gestureIK/MyGestSymbol.h"
#include "apps/gestureIK/IKSolver.h"

namespace gestureIKApp {
	SymbolRandVals::SymbolRandVals(std::shared_ptr<gestureIKApp::IKSolver> _IKSolve) : IKSolve(_IKSolve), cameraRot(origTrackBallQ), cameraTrans(0, 0, 0), cameraZoom(-1),//set when IKSolve is set
		rnd_headClr(.9, .9, .9), rnd_handClr(.9, .9, .9), rnd_handShape(0) {
		//zoom may vary based on randomization, if this symbol is randomized
		cameraZoom = IKSolve->params->origZoom;
		setRandCamSkelVals();
	}

	void SymbolRandVals::setRandCamSkelVals() {
		//build random camera orientation
		//first pick random rot axis in y/z plane and normalize while find x direction deviation (default camera is in positive x looking toward negative x, so this should be small) - set as rotvec
		double phi = IKSolve->getUniRandDbl(-DART_2PI, DART_2PI);
		Eigen::Vector3d rotVec(IKSolve->getRandDbl(0, .0001), sin(phi), cos(phi));
		rotVec.normalize();
		//then find rotational amount thet around this axis, build quat as w == cos(thet/2); v == sin(thet/2) * rotvec
		double thet = IKSolve->getUniRandDbl(-IKSolve->params->rnd_camThet, IKSolve->params->rnd_camThet);
		thet *= .5;
		Eigen::Vector3d tmpV(sin(thet) * rotVec);
		Eigen::Quaterniond tmpRot(cos(thet), tmpV(0), tmpV(1), tmpV(2));
		tmpRot.normalize();
		cameraRot = tmpRot * cameraRot;
		cameraRot.normalize();
		//random zoom
		//find variation amt 
		cameraZoom = IKSolve->params->origZoom + static_cast<float>(IKSolve->getUniRandDbl(-IKSolve->params->rnd_camZoom, IKSolve->params->rnd_camZoom));
		//std::cout << "Old Zoom : " << IKSolve->params->origZoom << "\t New Zoom : " << cameraZoom << "\n";
		//random camera location 
		Eigen::Vector3d min(-IKSolve->params->rnd_camTrans, -IKSolve->params->rnd_camTrans, -IKSolve->params->rnd_camTrans),
			max(IKSolve->params->rnd_camTrans, IKSolve->params->rnd_camTrans, IKSolve->params->rnd_camTrans);
		cameraTrans = origMTrans + IKSolve->getUniRandVec(min, max);
		//std::cout << "Base (original) Translation : (" << origMTrans(0)<<"," << origMTrans(1) << "," << origMTrans(2) << ")\tTranslation modification : (" << cameraTrans(0) << "," << cameraTrans(1) << "," << cameraTrans(2) <<")"<< std::endl;
		//random skel head dims - pct of base +/- in each dim - y set to zero to not get smaller (separate from skel)
		Eigen::Vector3d headMin(-IKSolve->params->rnd_headDimPct, 0, -IKSolve->params->rnd_headDimPct),
			headMax(IKSolve->params->rnd_headDimPct, IKSolve->params->rnd_headDimPct, IKSolve->params->rnd_headDimPct);
		rnd_headSize = IKSolve->getUniRandVec(headMin, headMax);

		//random skel head color - color is grayscale so just 1 double 0-1, avoid values +/- bnd amt around background clr
		float rndAmt;
		do {
			rndAmt = static_cast<float>(IKSolve->getUniRandDbl(0, 1));
		} while ((rndAmt > (IKSolve->params->bkgR - IKSolve->params->rnd_headClrBnd)) && (rndAmt < (IKSolve->params->bkgR + IKSolve->params->rnd_headClrBnd)));//in gray range
		rnd_headClr << rndAmt, rndAmt, rndAmt;
		//random skel hand shape : 0: rectangular, 1: ellipsoid
		rnd_handShape = (IKSolve->getUniRandDbl(0, 1) >= .5 ? 1 : 0);
		//random skel hand dims - pct of base +/- in each dim - y set to zero to not get smaller (separate from skel)
		Eigen::Vector3d handMin(-IKSolve->params->rnd_handDimPct, 0, -IKSolve->params->rnd_handDimPct),
			handMax(IKSolve->params->rnd_handDimPct, IKSolve->params->rnd_handDimPct, IKSolve->params->rnd_handDimPct);
		rnd_handSize = IKSolve->getUniRandVec(handMin, handMax);

		//random skel hand color - color is grayscale so just 1 double 0-1, avoid values .33-.66
		do {
			rndAmt = static_cast<float>(IKSolve->getUniRandDbl(0, 1));
		} while ((rndAmt >(IKSolve->params->bkgR - IKSolve->params->rnd_handClrBnd)) && (rndAmt < (IKSolve->params->bkgR + IKSolve->params->rnd_handClrBnd)));//in gray range
		rnd_handClr << rndAmt, rndAmt, rndAmt;
	}//setRandCamSkelVals


	MyGestSymbol::MyGestSymbol(std::shared_ptr<gestureIKApp::IKSolver> _IKSolve, const std::string& _name, int _srcIDX) : IKSolve(_IKSolve), _self(nullptr), 
		rndVals(std::allocate_shared<SymbolRandVals>(Eigen::aligned_allocator <SymbolRandVals>(), IKSolve)), IKPoses(0),
		//cameraRot(0.5*sqrt(2), 0, -0.5*sqrt(2), 0),
		curFrame(0), numTrajFrames(0), srcSymbolIDX(_srcIDX), allTrajsLen(0), //trajVel(.03), 
		sclAmt(1.0), curTrajDist(0), trajectories(0), trajLens(0), trajFrameIncrs(0), name(_name), flags(numFlags,false),
		avgImgSpcLoc(0,0,0), ptrCtrPt(0,0,0), elbowCtrPt(0,0,0), ptrPlaneNorm(-1,0,0), elbowPlaneNorm(0,0,0), circleRad(0), curTraj(0)

	{
		//show connecting trajectories in debug output
		flags[drawConnTrajIDX] = true;
		circleRad = IKSolve->params->IK_drawRad;
		ptrCtrPt << IKSolve->drawCrclCtr;
		//vector from drawing center to drawing shoulder marker
		ptrPlaneNorm << (IKSolve->tstRShldrSt - ptrCtrPt).normalized();
		elbowCtrPt << IKSolve->drawElbowCtr;
		elbowPlaneNorm << IKSolve->elbowShldrNormal;
	}//ctor
	
	MyGestSymbol::~MyGestSymbol() {}//dtor
	
	//list of trajectory file names composing this symbol
	void MyGestSymbol::buildTrajsFromFile(std::vector< std::string >& trajFileNames, std::shared_ptr<MyGestSymbol> _thisSP, bool useVel){
		_self = _thisSP;
		trajectories.clear();
		for (int i = 0; i < trajFileNames.size(); ++i) {
			trajectories.push_back(std::make_shared<MyGestTraj>(trajFileNames[i], _thisSP, i));
			//trajectories[i]->setSolver(IKSolve);
			trajectories[i]->readTrajFile(useVel);	
			//NOTE Trajectory timing info from matlab includes time span between ending and beginning trajectories.  might need to normalize for this
		}
		//all trajectories read by here, and each traj's srcTrajData is set
		//find average, min and max values in matab space for all and set all trajs with info - need this to project symbol in front of char for IK
		calcTransformPts();
		//build trajectories and linking trajectories for basic letters in IK space
		buildTrajComponents();
	}//readSymbolFile

	//build all trajectory components for this symbol
	void MyGestSymbol::buildTrajComponents() {
		//now remap all trajectories to be proscribed within a circle of specified radius from certain point around specified normal from center point to shoulder
		eignVecTyp ignoredPts(0);
		for (int i = 0; i < trajectories.size(); ++i) {
			trajectories[i]->buildTrajFromData(true, ignoredPts);
		}

		std::vector<std::shared_ptr<gestureIKApp::MyGestTraj> > tmpTraj(0);
		//get rid of tiny trajectories - don't want to connect to them
		for (int i = 0; i < trajectories.size(); ++i) {
			if (trajectories[i]->useTraj()) { tmpTraj.push_back(trajectories[i]); }
		}
		trajectories = tmpTraj;
		tmpTraj.clear();
		//get rid of single point trajectories that are close to previous/next trajectory end/start points
		for (int i = 0; i < trajectories.size(); ++i) {
			if (trajectories[i]->trajTargets.size() != 1) {		tmpTraj.push_back(trajectories[i]);		}//more than 1 point
			else {//only 1 point, make sure further than delta from previous traj end point or next traj start point
				//either first point-traj in list and further than epsilon from 2nd traj, or last point-traj in list and further than epsilon from end of prev traj, or
				//in-middle-of-list point-traj and further than epsilon from both surrounding trajs
				if (((i == trajectories.size() - 1) && (trajectories[i - 1]->getLastFrame()[0] - trajectories[i]->getFirstFrame()[0]).norm() > DART_EPSILON)		//last point traj in list, further than eps from prev end
					|| ((i == 0) && (trajectories[i]->getLastFrame()[0] - trajectories[i + 1]->getFirstFrame()[0]).norm() > DART_EPSILON)							//first point traj in list, further than eps from next traj st
					|| (((i>0) && (trajectories[i - 1]->getLastFrame()[0] - trajectories[i]->getFirstFrame()[0]).norm() > DART_EPSILON) &&
					((i < trajectories.size()-1) && (trajectories[i]->getLastFrame()[0] - trajectories[i+1]->getFirstFrame()[0]).norm() > DART_EPSILON))) {			//middle of list, further than eps from both prev and next
					tmpTraj.push_back(trajectories[i]);			//in middle of list of trajectories and further than epsilon from other trajs in list
					//std::cout << "Keeping 1-point traj :" << trajectories[i]->name << " since not redundant\n";
				}
				else {
					std::cout << "Tossing traj :" << trajectories[i]->name << " due to having only 1 redundant point\n";
				}
			}
		}
		trajectories = tmpTraj;
		tmpTraj.clear();

		if (trajectories.size() > 1) {
			bool unusedTraj = false;
			eignVecTyp stFrame, endFrame;
			tmpTraj.clear();
			//since we've gotten rid of all unused trajs by here, only potential trajectories we don't want are single-point trajs where the point overlaps a previous or next traj's end/start point
			int stIdx = 0, endIdx = 1;
			for (int i = 1; i < trajectories.size(); ++i) {
				//if (!trajectories[i - 1]->useTraj()) { continue; }
				stFrame = trajectories[i - 1]->getLastFrame();
				tmpTraj.push_back(trajectories[i - 1]);
				endFrame = trajectories[i]->getFirstFrame();
				if ((stFrame[0] - endFrame[0]).norm() < DART_EPSILON) {//don't need a connecting trajectory if they match endpoints
					if (trajectories[i]->trajTargets.size() == 1) {			//don't need this trajectory if it only has 1 point
						trajectories[i]->flags[MyGestTraj::useTrajIDX] = false;
						unusedTraj = true;
					}
					continue;
				}
				tmpTraj.push_back(genConnectTraj(ptrCtrPt, stFrame[0], endFrame[0]));
			}
			tmpTraj.push_back(trajectories[trajectories.size() - 1]);
			trajectories = tmpTraj;
			if (unusedTraj) {	//get rid of unused trajs found as connecting trajectories were built
				tmpTraj.clear();
				for (int i = 0; i < trajectories.size(); ++i) {		if (trajectories[i]->useTraj()) { tmpTraj.push_back(trajectories[i]); }		}
				trajectories = tmpTraj;
			}
		}
		calcAllTrajsLen();
	}//buildTrajComponents

	//calculate total length of trajectories, component lengths of each trajectory, and 
	void MyGestSymbol::calcAllTrajsLen() {
		allTrajsLen = 0;
		eignVecTyp stFrame, endFrame;
		for (int i = 0; i < trajectories.size(); ++i) {
			if (trajectories[i]->useTraj()) {
				allTrajsLen += trajectories[i]->trajLen;
			}
			else {
				std::cout<<"Nonused trajectory  for " << name << " idx : "<<i <<" of len "<< trajectories[i]->trajLen << "\n";
			}
			if (i < trajectories.size()-1) {//add distance between endpoints
				stFrame = trajectories[i]->getLastFrame();
				endFrame = trajectories[i+1]->getFirstFrame();
				double d = (stFrame[0] - endFrame[0]).norm();
				if (d > DART_EPSILON) {
					//std::cout<<"Non-zero inter-traj dist for " << name << " trajectories : "<<i<<" and "<<(i+1)<<" = "<<d<< std::endl;
					allTrajsLen += d;
				}
			}
			if (std::isnan(allTrajsLen)) {
				std::cout << "NAN traj length for " << name << " iter : " << i << " so setting to 0." << std::endl;
				allTrajsLen = 0;
			}
			trajLens.push_back(allTrajsLen);
		}
		//else { std::cout << "Total Traj len for : " << name << " : " << allTrajsLen << " from " << trajectories.size() << " trajectories."<< std::endl; }
		buildTrajFrameIncrs();
	}//calcAllTrajsLen

	//set up list holding per-frame distances allowed per trajectory
	void  MyGestSymbol::buildTrajFrameIncrs() {
		trajFrameIncrs.clear();
		numTrajFrames = 0 ;
		double trajIncrAmt = IKSolve->params->trajDesiredVel;

		if (IKSolve->params->useFixedGlblVel()) {//make all trajectories fixed per-frame speed
			trajIncrAmt = IKSolve->params->trajDesiredVel;
			numTrajFrames = (int)(ceil(allTrajsLen / trajIncrAmt))+1;

		}
		else if (IKSolve->params->limitTo16Frames()){
			numTrajFrames = 16;
			trajIncrAmt = allTrajsLen / (numTrajFrames - 1);
		}
		else if (IKSolve->params->useVarVelocity()) {
			//use delta arclength specified within xml letter trajectory description - different displacement for every sample.  randomization should scale time for total letter drawing (so all del arclength values should be scaled equally)

		}
		else {//calc variable length multiple of 8, greater than 16 frame clips DEPRECATED
			double trajAvgMult = (flags[isFastDrawnIDX] ? IKSolve->params->IK_fastTrajMult : 1.0);		//alternate between slow and fast trajs
			trajIncrAmt = (trajAvgMult * IKSolve->params->trajDesiredVel);
			numTrajFrames = (int)(allTrajsLen / trajIncrAmt);
			if (numTrajFrames < 16) {
				numTrajFrames = 16;
			}
			//clip to nearest mult of IKSolve->params->fixedClipLen
			int numSegMult = (int)((numTrajFrames + (IKSolve->params->fixedClipLen - 1)) / IKSolve->params->fixedClipLen);
			numTrajFrames = numSegMult * IKSolve->params->fixedClipLen;
			trajIncrAmt = allTrajsLen / (numTrajFrames - 1);
		}
		//TODO build avg increments based on desired trajectory velocity profile
		//scale amount by curavature? (slower at angles, faster at straights) scale by gravity? (faster going down, slower going up)
		//add one to have extra element in array for edge cases
		for (int i = 0; i < numTrajFrames + 1; ++i) {
			trajFrameIncrs.push_back(trajIncrAmt);
		}
		//if (flags[isFastDrawnIDX]) {
		//double trajLen = 0;
		//std::cout << "numTrajFrames : " << numTrajFrames << "\n";
		//for (int i = 0; i < numTrajFrames; ++i) {
		//	std::cout<<"\ttraj incr "<<i<<" for symbol "<<name<<" based on src idx "<< srcSymbolIDX << ": "<< trajFrameIncrs[i]<<" len by here : "<< trajLen <<"\ttotal len : "<< allTrajsLen << "\n";
		//	trajLen += trajFrameIncrs[i];
		//}
		//}
	}//buildTrajFrameIncrs

	 //set up all variables for initial IK'ing of this symbol's trajectories
	void MyGestSymbol::initSymbolIK() {
		//current length along trajectory total being processed
		curTrajDist = 0;
		//current trajectory idx of this symbol being processed
		curTraj = 0;
		//frame of IK being processed
		curFrame = 0;
		//not finished 
		flags[isDoneDrawingIDX] = false;
	}//initSymbolIK

	 //returns true if finished with trajectories
	bool MyGestSymbol::solve() {
		if (allTrajsLen == 0) { std::cout << "Error : trying to IK to length 0 trajectory for symbol : " << name << ".\n"; return true; }
		if (flags[isDoneDrawingIDX]) {//perform here so that we cover final point and don't change curFrame until after image is rendered
			initSymbolIK();
		}
		double trajStartLoc = (curTraj > 0) ? trajLens[curTraj - 1] : 0;
		//std::cout << "Cur Traj Dist for " << name << " in solve : " << curTrajDist << " and start length : " << trajStartLoc << " and end length of this traj : " << trajLens[curTraj] << "\n";
		//below calls IKSolver->solve(), solves IK
		bool finishedCurTraj = trajectories[curTraj]->setTrkMrkrAndSolve(curTrajDist - trajStartLoc);
		//save solved newPos from IKSolver
		IKPoses.push_back(IKSolve->getNewPose());
		//std::cout << "Frame : " << curFrame << " Name : "<<name<<" CurTrajDist : " << curTrajDist << " for traj of length : " << allTrajsLen << "\tdiff : "<< (curTrajDist - trajStartLoc) <<std::endl;
		++curFrame;
		if (flags[debugIDX]) {
			if (finishedCurTraj) { std::cout << "finishedCurTraj is true for " << name << " #trajs : " << trajLens.size() << " curTraj : " << curTraj << " curTrajDist : allTrajsLen : trajStartLoc " << curTrajDist << " : " << allTrajsLen << " : " << trajStartLoc << "\t frame count : " << curFrame << "\n"; }
			if (curTrajDist >= allTrajsLen) {
				std::cout << "\n";
				if (!finishedCurTraj) {
					std::cout << "!!!finishedCurTraj is false for " << name << " #trajs : " << trajLens.size() << " when ending - frame count : " << curFrame << "\n";
				}
				//std::cout << "Return from solve for " << name << " made " << curFrame << " frames.  finishedCurTraj : " << finishedCurTraj << "\n";
				if ((!IKSolve->params->useFixedGlblVel()) && (curFrame % IKSolve->params->fixedClipLen != 0)) {//don't check if not setting to be multiple of 16
					std::cout << "!!!! Non-mult of specified fixed clip length " << IKSolve->params->fixedClipLen << " : Frame count for symbol : " << name << " count : " << curFrame << "\n";
				}
				std::cout << "\n";
			}
		}
		if (curTrajDist >= allTrajsLen) {
			//if (flags[debugIDX]) {
			//	std::cout << "\n";
			//	if (!finishedCurTraj) {
			//		std::cout << "!!!finishedCurTraj is false for " << name << " #trajs : " << trajLens.size() << " when ending - frame count : " << curFrame << "\n";
			//	}
			//	//std::cout << "Return from solve for " << name << " made " << curFrame << " frames.  finishedCurTraj : " << finishedCurTraj << "\n";
			//	if ((!IKSolve->params->useFixedGlblVel()) && (curFrame % IKSolve->params->fixedClipLen != 0)) {//don't check if not setting to be multiple of 16
			//		std::cout << "!!!! Non-mult of specified fixed clip length " << IKSolve->params->fixedClipLen <<" : Frame count for symbol : " << name << " count : " << curFrame << "\n";
			//	}
			//	std::cout << "\n";
			//}
			flags[isDoneDrawingIDX] = true;
			//curFrame = 0;			//reset to 0 so can be used again for setIKSkelPose
			return true;
		}	//traversed entire length of trajectory

		curTrajDist += trajFrameIncrs[curFrame];		//desired displacement per frame
		if (curTrajDist - allTrajsLen > -DART_EPSILON) {				//check if past or very close to bounds, clip to final bound, just to make sure we get last marker but get it only 1 time
			curTrajDist = allTrajsLen;
		}
		if (curTrajDist > trajLens[curTraj]) {//last trajLens element should be length of entire trajectory (allTrajsLen) - won't ever increment curTraj for last frame
			++curTraj;
		}//		curTraj = idx;
		return false;
	}//solve

	//set skeleton pose to be pose of curRndrFrame + offFrame (using precalced dofs ara IKPoses)
	bool MyGestSymbol::setIKSkelPose(int offFrame, bool incrFrame) {
		int poseIdx = curFrame + offFrame;
		if (poseIdx < 0) { poseIdx = 0; }
		else if(poseIdx >= IKPoses.size()){ poseIdx = IKPoses.size()-1; }
		//set pose of skeleton from precalculated poses - used in motion blur
		IKSolve->setPoseAndCompSkelKin(IKPoses[poseIdx]);
		if (incrFrame) { ++curFrame; }
		return (curFrame == IKPoses.size());//if done or not
	}//setIKSkelPose

	//make this symbol a randomized version of the passed symbol - 
	bool MyGestSymbol::buildRandomSymbol(std::shared_ptr<MyGestSymbol> _src, std::shared_ptr<MyGestSymbol> _thisSP, bool showCtr, bool isFast) {
		_self = _thisSP;
		avgImgSpcLoc << _src->avgImgSpcLoc;		//avg location in matlab space
		//use this for random :  double getRandDbl(double mu, double std = 1.0) 
		//get gaussian random value for scl using base symbol scale amt as mean and std described in XML
		double scaleZ, scaleY;
		//do {
		//	sclAmt = IKSolve->getRandDbl(_src->sclAmt, IKSolve->params->trajRandSclStd);
		//} while (sclAmt <= 0);
		//this scales the y and z coords of the displacement vectors of the trajectories
		do {
			scaleZ = IKSolve->getRandDbl(_src->sclAmt, IKSolve->params->trajRandSclStd);
			scaleY = IKSolve->getRandDbl(_src->sclAmt, IKSolve->params->trajRandSclStd);
		} while ((scaleZ <= 0) || (scaleY <= 0));//repeat until both scale values are legal
		sclAmt = .5*(scaleZ + scaleY);
		//fast drawn trajectory wags arm/elbow - elbow IK trajectory flipped on elbow-normal plane
		flags[isFastDrawnIDX] = isFast;
		//std dev vector for random ctr point derivation
		//Eigen::Vector3d stdVec(IKSolve->params->trajRandCtrStdScale_X*IKSolve->params->trajRandCtrStd, IKSolve->params->trajRandCtrStdScale_Y*IKSolve->params->trajRandCtrStd, IKSolve->params->trajRandCtrStdScale_Z*IKSolve->params->trajRandCtrStd);
		Eigen::Vector3d stdVec = IKSolve->params->getCtrStdVec();
		ptrCtrPt << IKSolve->getRandVec(_src->ptrCtrPt, stdVec);				//
		if (showCtr) { std::cout << "MyGestSymbol::buildRandomSymbol : ptrCtrPt = " << buildStrFrmEigV3d(ptrCtrPt) << " |\t src ctrPt : " << buildStrFrmEigV3d(_src->ptrCtrPt) << "|\tstd vec : " << buildStrFrmEigV3d(stdVec) << "\n"; }
		setSymbolCenters(ptrCtrPt);				//need to set this before interacting with trajectories
		trajectories.clear();
		if(_src->trajectories.size() > 6){std::cout<<"Too many trajectories in symbol ("<< _src->trajectories.size()<<") so not using to generate random traj."<< std::endl; return false; }
		for (int i = 0; i < _src->trajectories.size(); ++i) {
			if (!_src->trajectories[i]->useTraj()) { std::cout << "Unused traj in trajectories for ltr : " << _src->name << " : symbol idx " << i << " so not using this symbol."<< std::endl; return false; }
			trajectories.push_back(std::make_shared<MyGestTraj>("not_from_file", _self, i));
			trajectories[i]->copySrcInfo(_src->trajectories[i], scaleZ / _src->sclAmt, scaleY / _src->sclAmt);
		}
		calcAllTrajsLen();
		return true;
	}//buildRandomSymbol

	//NOTE IK is done in z-y plane (camera along x axis)
	//find average z-y location of matlab-space trajectory points in component trajectories of this symbol, and closest and furthest points from average.
	//these will be used to map to "drawing plane" in ik sim world frame - all trajs will be treated the same, so avg/min/maxs apply to all
	//THIS ALSO CALLS FUNCTION THAT BUILDS THE DISPLACEMENT VECTORS used to rebuild trajectory in IK space
	void MyGestSymbol::calcTransformPts() {
		avgImgSpcLoc.setZero();
		int totPts = 0;
		eignVecTyp allTrajPts(0);
		//find average location
		int destIdx = 0;
		for (int i = 0; i < trajectories.size(); ++i) {
			std::shared_ptr<MyGestTraj> traj = trajectories[i];
			for (int j = 0; j < traj->srcTrajData.size(); ++j) {
				//allTrajPts.push_back(traj->srcTrajData[j]);
				avgImgSpcLoc += traj->srcTrajData[j];
				//allTrajPts[destIdx++](0) = 0;		//x is timing info, not to be used for location
			}
			totPts += traj->srcTrajData.size();
		}
		if (totPts == 0) {	std::cout << "Error : no points found for symbol " << name << " to calculate average from."<< std::endl;	return;	}
		//find avg of all pts - ignore x coord, which holds timing/arclength info
		//for (int i = 0; i < allTrajPts.size(); ++i) { avgImgSpcLoc += allTrajPts[i];		}
		avgImgSpcLoc(0) = 0;//x should be 0 since img/matlab space is z-y
		avgImgSpcLoc /= totPts;
		//std::cout << "Avg Loc for symbol : " << (this->name) << " with # of trajs : " << trajectories.size() << " and total # of pts : " << totPts << " : (" << buildStrFrmEigV3d(avgLoc) << ")"<< std::endl;
		double maxDist = -99999999, minDist = 99999999;
		//set avg loc and build disp std::vectors from avg location to each src point
		for (int i = 0; i < trajectories.size(); ++i) {
			trajectories[i]->calcSrcTrajDispVecs(avgImgSpcLoc);
		}
		//for all trajs for all points - find min/max based on avg loc - need to be done here
		for (int i = 0; i < trajectories.size(); ++i) {
			if (trajectories[i]->lenMaxSrcDisp > maxDist) { maxDist = trajectories[i]->lenMaxSrcDisp; }
			if (trajectories[i]->lenMinSrcDisp <= minDist) { minDist = trajectories[i]->lenMinSrcDisp; }
		}
		if (IKSolve->params->useMaxDispLen()) {		sclAmt = circleRad / maxDist;	}
		else {			sclAmt = circleRad / (.5*(maxDist + minDist));		}

		//std::cout << "Max Loc for symbol : " << (this->name) << " with # of trajs : " << trajectories.size() << " and total # of pts : " << totPts << " : (" << buildStrFrmEigV3d(maxLoc) << ") and Min Dist : " << maxDist << "\n";
		//find scale amount by using params->IK_drawRad and maxDist - scales all std::vectors by this amount to fit within proscribing circle
	}//calcTransformPts


	//draw all trajectory points of this letter
	void MyGestSymbol::drawTrajs(dart::renderer::RenderInterface* mRI) {
		mRI->pushMatrix();
		Eigen::Vector3d penClr = Eigen::Vector3d(.9, .9, .9);
		for (int i = 0; i < trajectories.size(); ++i) {
			if (((!flags[drawConnTrajIDX]) && (trajectories[i]->isConnTraj())) || (!trajectories[i]->useTraj())) { continue; }		//don't draw connecting trajs unless we want to display them, or trajs that we arent using
			if (IKSolve->params->chgTrajClrs()) {
				penClr = trajColors[(i % trajColors.size())];
			}
			trajectories[i]->drawDebugTraj(mRI, penClr);
		}
		mRI->popMatrix();
	}//drawTrajs

	//passes center point to use, build all other relevant vals off this value (for random centers).  
	void MyGestSymbol::setSymbolCenters(const Eigen::Ref<const Eigen::Vector3d>& ctPt) {
		ptrCtrPt << ctPt;

		ptrPlaneNorm << (IKSolve->tstRShldrSt - ptrCtrPt).normalized();
		//instead of winging it for elbow, IK to draw center and update elbow location - use this estimate as start loc
		elbowCtrPt = .5 * (IKSolve->tstRShldrSt + ptrCtrPt);
		//move directly down by scaled amt of reach approx equal to height of isoc triangle of forearm and bicep
		elbowCtrPt(1) -= (IKSolve->reach * IKSolve->params->IK_elbowScale);
		//normal point from elbow to shoulder as arm is extended to ~average position
		elbowPlaneNorm = (IKSolve->tstRShldrSt - elbowCtrPt).normalized();
		if (flags[debugIDX]) {
			std::cout << "In Symbol " << name << " : Right arm reach : " << IKSolve->reach << " center " << buildStrFrmEigV3d(ptrCtrPt) << " elbow center " << buildStrFrmEigV3d(elbowCtrPt) << " and rad of test circle " << IKSolve->params->IK_drawRad << "\n";
		}
		IKSolve->setTargetPos(ptrCtrPt, elbowCtrPt);
		//(*IKSolve->trkMarkers)[trkedMrkrNames[0]]->setTarPos(ptrCtrPt);
		//(*IKSolve->trkMarkers)[trkedMrkrNames[1]]->setTarPos(elbowCtrPt);
		IKSolve->solve();
		//recalc elbow center after IKing ptr to circle center location
		if (flags[debugIDX]) {	
			std::cout << "Sample Elbow Location being updated after IKing to Ctr point in IKSolver"<< std::endl;
		}
		//IKSolve->drawElbowMrkrName set when IKSolve->params loaded
		elbowCtrPt = IKSolve->getSkel()->getMarker(IKSolve->drawElbowMrkrName)->getWorldPosition();
		//normal point from elbow to shoulder as arm is extended to ~average position
		elbowPlaneNorm = (IKSolve->tstRShldrSt - elbowCtrPt).normalized();
	}//setSymbolCenters

	//set flags from owning letter, passed from user input
	void MyGestSymbol::setFlags(int idx, bool val) {
		flags[idx] = val;
		switch (idx) {
		case debugIDX: {
			for (int i = 0; i < trajectories.size(); ++i) { trajectories[i]->flags[debugIDX] = val; }//turn on debug for all trajectories
			break; }		//debug mode
		case testLtrQualIDX: {//test all symbols to make sure no abberrant trajectories
			for (int i = 0; i < trajectories.size(); ++i) { trajectories[i]->flags[testLtrQualIDX] = val; }//turn on testing of trajs for all trajectories
			break;		}
		case drawConnTrajIDX: {
			break; } //draw the inter-trajectory connecting generated trajectories
		case showAllTrajsIDX: {
			for (int i = 0; i < trajectories.size(); ++i) { trajectories[i]->setShowAllTrajs( val); }//turn on debug for all trajectories
			break;}
		}
	}//setFlags

	//generate a linking trajectory from the end of 1 trajectory to the beginning of another using 4 pt neville between end points 
	std::shared_ptr<MyGestTraj> MyGestSymbol::genConnectTraj(const Eigen::Ref<const Eigen::Vector3d>& ctPt, const Eigen::Ref<const Eigen::Vector3d>& traj1End, const Eigen::Ref<const Eigen::Vector3d>& traj2St) {
		std::shared_ptr<MyGestTraj> res = std::make_shared<MyGestTraj>("", _self, -1);
		res->flags[res->connTrajIDX] = true;
		//gen 4 pt neville in plane of motion
		Eigen::Vector3d stPtVec = (traj1End - ctPt), endPtVec(traj1End - traj2St);
		double scAmt1 = stPtVec.norm() * nevPct,
			scAmt2 = endPtVec.norm() * nevPct;
		stPtVec.normalize();
		endPtVec.normalize();
		eignVecTyp genPoints(0);
		//intermediate neville points
		Eigen::Vector3d p1 = traj1End + (stPtVec *scAmt1),	p2 = traj2St + (endPtVec *scAmt2);	
		//neville between traj1End, p1, p2, traj2St
		float tot = 0.0f;
		res->srtTrajTiming.clear();
		for (float t = 0; t <= 1.0f; t += .01f) {
			genPoints.push_back(QInterp(traj1End, p1, p2, traj2St,t));
			res->srtTrajTiming.push_back(.01f);
			tot += .01f;
		}
		for (int i = 0; i < res->srtTrajTiming.size(); ++i) {	res->srtTrajTiming[i] /= tot;	}
		//TODO add points to new MyGestTraj and calculate all component target trajectories
		res->buildTrajFromData(false, genPoints);

		//NOTE : playback of trajectory needs to be driven by desired motion velocity
		return res;
	}//genConnectTraj

	//generate trajectory to be used between symbols - this symbol's last traj point and next symbol's first traj point
	//4 pt neville between end points through pt
	//ctPt is center of projection of symbol, sym1End is ending point of last trajectory of ending symbol, midPt is pull-back point toward shoulder, sym2St is first point of first trajectory of new symbol
	std::shared_ptr<MyGestTraj> MyGestSymbol::genInterSymbolTraj(const Eigen::Ref<const Eigen::Vector3d>& ctPt, const Eigen::Ref<const Eigen::Vector3d>& sym1End, const Eigen::Ref<const Eigen::Vector3d>& midpt, const Eigen::Ref<const Eigen::Vector3d>& sym2St) {
		//start with 4 pt neville in plane of motion and take result and generate regular neville to get "pull back"
		std::shared_ptr<MyGestTraj> res = genConnectTraj(ctPt, sym1End, sym2St);
		//interpolate res toward midpt - midpt needs to be center of res trajectory moved back in neg direction of (normal from shoulder forward) some %
		//TODO
		return res;
	}
	
	std::ostream& operator<<(std::ostream& out, MyGestSymbol& sym) {
		out << "Symbol : " << sym.name << "\tavg loc : ("<< buildStrFrmEigV3d(sym.avgImgSpcLoc)<<")"<< std::endl;
		return out;
	}


}  // namespace gestureIKApp
