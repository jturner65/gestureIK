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

	MyGestSymbol::MyGestSymbol(std::string _name, int _srcIDX) : IKSolve(nullptr), _self(nullptr), curTraj(0), curFrame(0), srcSymbolIDX(_srcIDX), allTrajsLen(0), trajVel(.03), sclAmt(1.0), curTrajDist(0),
		trajectories(0), trajLens(0),name(_name), flags(numFlags,false),
		avgLoc(0,0,0),// maxLoc(0,0,0), 
		ptrCtrPt(0,0,0), elbowCtrPt(0,0,0), ptrPlaneNorm(-1,0,0), elbowPlaneNorm(0,0,0), circleRad(0)

	{
		flags[diffClrIDX] = true;
		//show connecting trajectories
		flags[drawConnTrajIDX] = true;
	}
	
	MyGestSymbol::~MyGestSymbol() {}
	
	//list of trajectory file names composing this symbol
	void MyGestSymbol::buildTrajsFromFile(vector< std::string >& trajFileNames, std::shared_ptr<MyGestSymbol> _thisSP){
		_self = _thisSP;
		trajectories.clear();
		for (int i = 0; i < trajFileNames.size(); ++i) {
			trajectories.push_back(std::make_shared<MyGestTraj>(trajFileNames[i], _thisSP, i));
			trajectories[i]->setSolver(IKSolve);
			trajectories[i]->readTrajFile();
			//NOTE Trajectory timing info from matlab includes time span between ending and beginning trajectories.  might need to normalize for this
		}
		//all trajectories read by here, and each traj's srcTrajData is set
		//find average, min and max values in matab space for all and set all trajs with info
		calcTransformPts();
		//build trajectories and linking trajectories
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

		if (trajectories.size() > 1) {
			//TODO for all trajectories link endpoints with virtual trajectories
			eignVecTyp stFrame, endFrame;
			tmpTraj.clear();
			//since we've gotten rid of all unused trajs by here, only potential trajectories we don't want are single-point trajs where the point overlaps a previous or next traj's end/start point
			for (int i = 1; i < trajectories.size(); ++i) {
				if (!trajectories[i - 1]->useTraj()) { continue; }
				stFrame = trajectories[i - 1]->getLastFrame();
				tmpTraj.push_back(trajectories[i - 1]);
				endFrame = trajectories[i]->getFirstFrame();
				if ((stFrame[0] - endFrame[0]).squaredNorm() < .00000001) {//don't need a connecting trajectory if they match endpoints
					if (trajectories[i]->trajTargets.size() == 1) {			//don't need this trajectory if it only has 1 point
						trajectories[i]->flags[MyGestTraj::useTrajIDX] = false;
					}
					continue;
				}
				tmpTraj.push_back(genConnectTraj(ptrCtrPt, stFrame[0], endFrame[0]));
			}
			tmpTraj.push_back(trajectories[trajectories.size() - 1]);
			trajectories = tmpTraj;
		}
		calcAllTrajsLen();
	}//buildTrajComponents
	void MyGestSymbol::calcAllTrajsLen() {
		allTrajsLen = 0;
		for (int i = 0; i < trajectories.size(); ++i) {
			if (trajectories[i]->useTraj()) {
				allTrajsLen += trajectories[i]->trajLen;
			}
			trajLens.push_back(allTrajsLen);
		}
		if (isnan(allTrajsLen)) {
			cout << "NAN traj length for " << name << " so setting to 0.\n";
			allTrajsLen = 0;
		}
		else { cout << "Total Traj len for : " << name << " : " << allTrajsLen << " from " << trajectories.size() << " trajectories.\n"; }
	}//calcAllTrajsLen

	 //set up all variables for initial IK'ing of this symbol's trajectories
	void MyGestSymbol::initSymbolIK() {
		//current length along trajectory total being processed
		curTrajDist = 0;
		//current trajectory idx of this symbol being processed
		curTraj = 0;
		//frame of IK being processed
		curFrame = 0;
	}//initSymbolIK

	 //returns true if finished with trajectories
	bool MyGestSymbol::solve() {
		if (curTrajDist > allTrajsLen) {
			initSymbolIK();
		}
		if (allTrajsLen == 0) {	cout << "Error : trying to IK to length 0 trajectory.";	}
		curTraj = 0;
		double trajStartLoc = 0;
		for (int idx = 0; idx < trajLens.size(); ++idx) {
			if (curTrajDist <= trajLens[idx]) {//last trajLens element should be length of entire trajectory
				curTraj = idx;
				if (idx > 0) {	trajStartLoc = trajLens[idx - 1]; }
				break;
			}
		}

		int numPts = trajectories[curTraj]->trajTargets.size();
		//symbol needs to manage time passage, trajectories should just be responsive
		//find idx's of 
		//cout << "Cur Traj Dist for " << name << " in solve : " << curTrajDist << " and start length : " << trajStartLoc << " and end length of this traj : " << trajLens[curTraj] << "\n";
		trajectories[curTraj]->setTrkMrkrAndSolve(curTrajDist - trajStartLoc);
		++curFrame;
		double trajMult = (flags[isFastDrawnIDX] ? IKSolve->params->IK_fastTrajMult : 1.0);
		curTrajDist += IKSolve->params->trajDesiredVel;		//desired displacement per frame
		if (curTrajDist > allTrajsLen) { return true; }	//traversed entire length of trajectory
		return false;
	}//solve

	//make this symbol a randomized version of the passed symbol - 
	void MyGestSymbol::buildRandomSymbol(std::shared_ptr<MyGestSymbol> _src, std::shared_ptr<MyGestSymbol> _thisSP, bool isFast) {
		_self = _thisSP;
		avgLoc << _src->avgLoc;		//avg location in matlab space

		//use this for random :  double getRandDbl(double mu, double std = 1.0) 
		do {
			sclAmt = IKSolve->getRandDbl(_src->sclAmt, IKSolve->params->trajRandSclStd);
		} while (sclAmt <= 0);
		bool isFastTraj = isFast;
		trajVel = IKSolve->params->trajDesiredVel;
		if (isFastTraj) {
			trajVel *= IKSolve->params->IK_fastTrajMult;
		}
		ptrCtrPt << IKSolve->getRandVec(_src->ptrCtrPt, IKSolve->params->trajRandCtrStd);				//
		setSymbolCenters(ptrCtrPt);				//need to set this before interacting with trajectories
		trajectories.clear();
		for (int i = 0; i < _src->trajectories.size(); ++i) {
			if (!_src->trajectories[i]->useTraj()) { cout << "NonUsed traj in trajectories for ltr : " << _src->name << " : idx " << i << "\n"; }
			trajectories.push_back(std::make_shared<MyGestTraj>("not_from_file", _self, i));
			trajectories[i]->setSolver(IKSolve);
			trajectories[i]->copySrcInfo(_src->trajectories[i]);
		}
		calcAllTrajsLen();
	}//buildRandomSymbol

	//find average x-y location of matlab-space trajectory points in component trajectories of this symbol, and closest and furthest points from average.
	//these will be used to map to "drawing plane" in ik sim world frame - all trajs will be treated the same, so avg/min/maxs apply to all
	void MyGestSymbol::calcTransformPts() {
		avgLoc.setZero();
		int totPts = 0;
		eignVecTyp allTrajPts(0);
		//find average location
		int destIdx = 0;
		for (int i = 0; i < trajectories.size(); ++i) {
			shared_ptr<MyGestTraj> traj = trajectories[i];
			for (int j = 0; j < traj->srcTrajData.size(); ++j) {
				allTrajPts.push_back(traj->srcTrajData[j]);
				allTrajPts[destIdx++](0) = 0;		//x is timing info, not to be used for location
			}
			totPts += traj->srcTrajData.size();
		}
		if (totPts == 0) {	cout << "Error : no points found for symbol " << (*this) << " to calculate average from.\n";	return;	}
		//find avg of all pts
		for (int i = 0; i < allTrajPts.size(); ++i) {		avgLoc += allTrajPts[i];		}
		avgLoc /= totPts;
		//cout << "Avg Loc for symbol : " << (this->name) << " with # of trajs : " << trajectories.size() << " and total # of pts : " << totPts << " : (" << buildStrFrmEigV3d(avgLoc) << ")\n";
		//set for all trajs - avgLoc,  maxLoc,  maxDist(0),
		//maxLoc.setZero();
		double maxDist = -99999999;
		//set avg loc and build disp vectors from avg location to each src point
		for (int i = 0; i < trajectories.size(); ++i) {
			trajectories[i]->calcSrcTrajDispVecs(avgLoc);
		}
		//for all trajs for all points - find min/max based on avg loc - need to be done here
		for (int i = 0; i < trajectories.size(); ++i) {		
			if (trajectories[i]->lenMaxSrcDisp > maxDist) { maxDist = trajectories[i]->lenMaxSrcDisp; }	
		}

		//cout << "Max Loc for symbol : " << (this->name) << " with # of trajs : " << trajectories.size() << " and total # of pts : " << totPts << " : (" << buildStrFrmEigV3d(maxLoc) << ") and Min Dist : " << maxDist << "\n";
		//find scale amount by using params->IK_drawRad and maxDist - scales all vectors by this amount to fit within proscribed circle
		sclAmt = circleRad / maxDist;
	}//calcTransformPts

	//draw all trajectory points of this letter
	void MyGestSymbol::drawTrajs(dart::renderer::RenderInterface* mRI) {
		mRI->pushMatrix();
		Eigen::Vector3d penClr = Eigen::Vector3d(.1, .1, .1);
		for (int i = 0; i < trajectories.size(); ++i) {
			if (((!flags[drawConnTrajIDX]) && (trajectories[i]->isConnTraj())) || (!trajectories[i]->useTraj())) { continue; }		//don't draw connecting trajs unless we want to display them, or trajs that we arent using
			if (flags[diffClrIDX]) {
				penClr = trajColors[(i % trajColors.size())];
			}
			trajectories[i]->drawTraj(mRI, penClr);
		}
		mRI->popMatrix();
	}//drawTrajs

	 //set reference to IK solver - set in all trajectories
	void MyGestSymbol::setSolver(std::shared_ptr<gestureIKApp::IKSolver> _slv) {
		IKSolve = _slv;
		circleRad = IKSolve->params->IK_drawRad;
		ptrCtrPt << IKSolve->drawCrclCtr;
		ptrPlaneNorm << (IKSolve->tstRShldrSt - ptrCtrPt).normalized();
		elbowCtrPt << IKSolve->drawElbowCtr;
		elbowPlaneNorm << IKSolve->elbowShldrNormal;
		trajVel = IKSolve->params->trajDesiredVel;

		for (int i = 0; i<trajectories.size(); ++i) { trajectories[i]->setSolver(IKSolve); }
	}

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
			cout << "In Symbol " << name << " : Right arm reach : " << IKSolve->reach << " center " << buildStrFrmEigV3d(ptrCtrPt) << " elbow center " << buildStrFrmEigV3d(elbowCtrPt) << " and rad of test circle " << IKSolve->params->IK_drawRad << "\n";
		}
		(*IKSolve->trkMarkers)[trkedMrkrNames[0]]->setTarPos(ptrCtrPt);
		(*IKSolve->trkMarkers)[trkedMrkrNames[1]]->setTarPos(elbowCtrPt);
		IKSolve->solve();
		//recalc elbow center after IKing ptr to circle center location
		if (flags[debugIDX]) {	
			cout << "Sample Elbow Location being updated after IKing to Ctr point in IKSolver\n";
		}
		elbowCtrPt = IKSolve->getSkel()->getMarker("ptrElbow_r")->getWorldPosition();
		//normal point from elbow to shoulder as arm is extended to ~average position
		elbowPlaneNorm = (IKSolve->tstRShldrSt - elbowCtrPt).normalized();
	}//setSymbolCenters

	//set flags from owning letter, passed from user input
	void MyGestSymbol::setFlags(int idx, bool val) {
		flags[idx] = val;
		switch (idx) {
		case debugIDX: {
			for (int i = 0; i < trajectories.size(); ++i) { trajectories[i]->flags[0] = val; }//turn on debug for all trajectories
			break; }		//debug mode
		case diffClrIDX: {
			break; }		//use different colors for each component trajectory of this symbol
		case drawConnTrajIDX: {
			break; } //draw the inter-trajectory connecting generated trajectories
		}
	}//setFlags

	//TODO move to MyGestLetter ?
	//generate a linking trajectory from the end of 1 trajectory to the beginning of another using 4 pt neville between end points 
	std::shared_ptr<MyGestTraj> MyGestSymbol::genConnectTraj(const Eigen::Ref<const Eigen::Vector3d>& ctPt, const Eigen::Ref<const Eigen::Vector3d>& traj1End, const Eigen::Ref<const Eigen::Vector3d>& traj2St) {
		std::shared_ptr<MyGestTraj> res = std::make_shared<MyGestTraj>("", _self, -1);
		res->setSolver(IKSolve);
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
		for (float t = 0; t <= 1.0f; t += .01f) {
			genPoints.push_back(QInterp(traj1End, p1, p2, traj2St,t));
		}
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
		out << "Symbol : " << sym.name << "\tavg loc : ("<< buildStrFrmEigV3d(sym.avgLoc)<<")\n";
		return out;
	}


}  // namespace gestureIKApp
