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

	MyGestSymbol::MyGestSymbol(std::string _name):IKSolve(nullptr),curTraj(0), curFrame(0), allTrajsLen(0), curTrajDist(0),trajectories(0), trajLens(0),name(_name), flags(numFlags,false),
		avgLoc(0,0,0), maxLoc(0,0,0)
	{
		flags[diffClrIDX] = true;
		//show connecting trajectories
		flags[drawConnTrajIDX] = true;
	}
	
	MyGestSymbol::~MyGestSymbol() {}
	
	//list of trajectory file names composing this symbol
	void MyGestSymbol::readTrajFiles(vector< std::string >& trajFileNames, std::shared_ptr<MyGestSymbol> _thisSP){
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
		//now remap all trajectories
		eignVecTyp ignoredPts(0);
		for (int i = 0; i < trajectories.size(); ++i) {
			trajectories[i]->buildTrajFromData(true, ignoredPts);
		}

		std::vector<std::shared_ptr<gestureIKApp::MyGestTraj> > tmpTraj(0);
		//get rid of tiny trajectories - don't want to connect to them
		for (int i = 0; i < trajectories.size(); ++i) {
			if (trajectories[i]->useTraj()) {			tmpTraj.push_back(trajectories[i]);		}			
		}
		trajectories = tmpTraj;

		if (trajectories.size() > 1) {
			//TODO for all trajectories link endpoints with virtual trajectories
			eignVecTyp stFrame, endFrame;
			tmpTraj.clear();
			//since we've gotten rid of all unused trajs by here, only potential trajectories we don't want are single-point trajs where the point overlaps a previous or next traj's end/start point
			for (int i = 1; i < trajectories.size(); ++i) {
				if (!trajectories[i - 1]->useTraj()) {	continue;}
				stFrame = trajectories[i - 1]->getLastFrame();
				tmpTraj.push_back(trajectories[i - 1]);

				endFrame = trajectories[i]->getFirstFrame();
				if ((stFrame[0] - endFrame[0]).squaredNorm() < .00000001) {//don't need a connecting trajectory if they match endpoints
					if (trajectories[i]->trajTargets.size() == 1) {			//don't need this trajectory if it only has 1 point
						trajectories[i]->flags[MyGestTraj::useTrajIDX] = false;
					}
					continue;
				}
				tmpTraj.push_back(genConnectTraj(IKSolve->drawCrclCtr, stFrame[0], endFrame[0]));
			}
			tmpTraj.push_back(trajectories[trajectories.size() - 1]);
			
			trajectories = tmpTraj;
		}
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
		else {
			cout << "Total Traj len for : " << name << " : " << allTrajsLen << " from " << trajectories.size() << " trajectories.\n"; 
		}
	}//readSymbolFile

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
		curTrajDist += IKSolve->params->trajDesiredVel;		//desired displacement per frame
		if (curTrajDist > allTrajsLen) { initSymbolIK(); return true; }	//traversed entire length of trajectory
		return false;
	}//solve


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
		//set for all trajs - avgLoc, minLoc, maxLoc, minDist(0), maxDist(0),
		maxLoc.setZero();
		double maxSqDist = -99999999;
		Eigen::Vector3d tmp;
		//for all trajs for all points - find min/max based on avg loc
		for (int i = 0; i < trajectories.size(); ++i) {
			shared_ptr<MyGestTraj> traj = trajectories[i];
			traj->avgLoc = avgLoc;
			for (int j = 0; j < traj->srcTrajData.size(); ++j) {
				tmp <<0, (traj->srcTrajData[j](1) - avgLoc(1)), (traj->srcTrajData[j](2) - avgLoc(2));			//only z-y coords - in drawing plane.  displacement vector
				double dSq = tmp.squaredNorm();
				if (dSq > maxSqDist) {	maxSqDist = dSq;	maxLoc << tmp;	}
				traj->srcTrajDispVecs.push_back(std::move(tmp));
			}
		}
		double maxDist = sqrt(maxSqDist);
		//cout << "Max Loc for symbol : " << (this->name) << " with # of trajs : " << trajectories.size() << " and total # of pts : " << totPts << " : (" << buildStrFrmEigV3d(maxLoc) << ") and Min Dist : " << maxDist << "\n";
		//cout << "\n";
		//set vectors from avg loc to all pts
		//find scale amount by using params->IK_drawRad and maxDist
		double scAmt = IKSolve->params->IK_drawRad / maxDist;
		//cout << "Scale amt for symbol : " << (this->name) << " with # of trajs : " << trajectories.size() << " and total # of pts : " << totPts << " : " << scAmt << "\n";
		//set min/max pts and dists for all trajs
		for (int i = 0; i < trajectories.size(); ++i) {
			trajectories[i]->sclAmt = scAmt;
		}
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
		for (int i = 0; i<trajectories.size(); ++i) { trajectories[i]->setSolver(IKSolve); }
	}

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

		return res;
	}
	
	std::ostream& operator<<(std::ostream& out, MyGestSymbol& sym) {
		out << "Symbol : " << sym.name << "\tavg loc : ("<< buildStrFrmEigV3d(sym.avgLoc)<<")\n";
		return out;
	}


}  // namespace gestureIKApp
