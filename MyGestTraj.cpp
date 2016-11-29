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

#include "apps/gestureIK/MyGestTraj.h"
#include "apps/gestureIK/IKSolver.h"

namespace gestureIKApp {
	MyGestTraj::MyGestTraj(const std::string& _fname, std::shared_ptr<gestureIKApp::MyGestSymbol> _p, int _num):
		IKSolve(nullptr), parentSymbol(_p), avgLoc(0,0,0), trajLen(0), lenMaxSrcDisp(0), ctrPoint(0,0,0),
		filename(_fname), name("tmp"),trajTargets(), srcTrajData(), srcTrajDispVecs(), convTrajPts(), debugTrajPts(), srtTrajTiming(), trajPtDistFromSt(),
		flags(numFlags, false) 
	{
		std::stringstream ss;
		ss << parentSymbol->name << "_" << _num;
		name = ss.str();
	}
	 
	MyGestTraj::~MyGestTraj() {}

	//set tracked marker trajectory target positions - perform this every frame, update the IKsolver's trackedMarkers to have the current frame's appropriate locations/trajectories
	//returns true if using last points in trajectory and interpolant is exactly 1
	bool MyGestTraj::setTrkMrkrAndSolve(double lenFromStTraj) {
		bool finCurTraj = false;
		if (1 == trajPtDistFromSt.size()) {//only 1 point in this trajectory
			finCurTraj = true;
			//if (flags[debugIDX]) { std::cout << "Single Point Traj : " << name << " finCurTraj : " << (finCurTraj ? "True " : "False ") << "\tt : " << 1 << "\tlenFromStTraj : " << lenFromStTraj << "\t# trajPtDistFromSt " << trajPtDistFromSt.size() << "\ttrajPtDistFromSt[" << 0 << "] = "<< trajPtDistFromSt[0] << std::endl; }
			//set targets for IK
			for (int i = 0; i < trkedMrkrNames.size(); ++i) {
				Eigen::Vector3d vec = trajTargets[0][i];
				(*IKSolve->trkMarkers)[trkedMrkrNames[i]]->setTarPos(vec);
			}
		} else {
			int frame1 = -1, frame2 = -1;
			double curFrmStDist = 0, t = 0;
			//clip len from st traj to be limited to max len of this traj if this is not closed
			if (lenFromStTraj > trajPtDistFromSt[trajPtDistFromSt.size() - 1]) {
				if (flags[closeTrajIDX]) {
					std::cout << "Closed traj : " << name << std::endl;
					frame1 = trajPtDistFromSt.size() - 1;
					frame2 = 0;
					t = (lenFromStTraj - trajPtDistFromSt[frame1]) / (trajPtDistFromSt[frame2] - trajPtDistFromSt[frame1]);
					finCurTraj = abs(t - 1.0) < DART_EPSILON;
				}
				else {	//clip to end length
					t = 1;
					finCurTraj = true;
					frame1 = trajPtDistFromSt.size() - 2;
					frame2 = trajPtDistFromSt.size() - 1;
					lenFromStTraj = trajPtDistFromSt[trajPtDistFromSt.size() - 1];
				}
			}
			else {
				if (lenFromStTraj == 0) {
					t = 0;
					frame1 = 0;
					frame2 = 1;
					finCurTraj = false;
				}
				else {
					for (int i = 1; i < trajPtDistFromSt.size(); ++i) {
						if ((lenFromStTraj > trajPtDistFromSt[i - 1]) && (lenFromStTraj <= trajPtDistFromSt[i])) {
							frame1 = i - 1;
							frame2 = i;
							break;
						}
					}
					t = (lenFromStTraj - trajPtDistFromSt[frame1]) / (trajPtDistFromSt[frame2] - trajPtDistFromSt[frame1]);
					finCurTraj = (!flags[closeTrajIDX]) && (frame1 == trajPtDistFromSt.size() - 2) && (frame2 == trajPtDistFromSt.size() - 1) &&( abs(t - 1.0) < DART_EPSILON);
				}
			}

			//if ((flags[debugIDX]) && (t == 1)){ std::cout << "Traj : " << name << " finCurTraj : " << (finCurTraj ? "True " : "False ") << "\tt : " << t << "\tlenFromStTraj : " << lenFromStTraj << "\t# trajPtDistFromSt " << trajPtDistFromSt.size() << "\ttrajPtDistFromSt[" << frame1 << "] = " << trajPtDistFromSt[frame1] << "\ttrajPtDistFromSt[" << frame2 << "] = " << trajPtDistFromSt[frame2] << std::endl; }
			if (t > 1) {
				std::cout << "\n!!!!!!Traj : " << name << " t greater than 1 for t : " << t << "\tlenFromStTraj : " << lenFromStTraj << "\t# trajPtDistFromSt " << trajPtDistFromSt.size() << "\ttrajPtDistFromSt[" << frame1 << "] = " << trajPtDistFromSt[frame1] << "\ttrajPtDistFromSt[" << frame2 << "] = " << trajPtDistFromSt[frame2] << "\n" << std::endl;
				t = 1;
			}
			//set targets for IK
			for (int i = 0; i < trkedMrkrNames.size(); ++i) {
				Eigen::Vector3d vec = interpVec(trajTargets[frame1][i], trajTargets[frame2][i], t);
				(*IKSolve->trkMarkers)[trkedMrkrNames[i]]->setTarPos(vec);
			}
		}
		//if (!flags[testLtrQualIDX]) {
		IKSolve->solve();
		//}
		return finCurTraj;
	}//setTrkMrkrAndSolve

	void MyGestTraj::calcSrcTrajDispVecs(const Eigen::Ref<const Eigen::Vector3d>& _avgLoc) {
		avgLoc << _avgLoc;
		srcTrajDispVecs.clear();
		double maxSqDisp = -99999;
		Eigen::Vector3d tmp(0, 0, 0);
		for (int j = 0; j < srcTrajData.size(); ++j) {
			tmp << 0, (srcTrajData[j](1) - avgLoc(1)), (srcTrajData[j](2) - avgLoc(2));			//only z-y coords since this is the format from the matlab data
			double sqLen = tmp.squaredNorm();
			if (sqLen > maxSqDisp) {	maxSqDisp = sqLen;	}
			srcTrajDispVecs.push_back(std::move(tmp));
		}
		lenMaxSrcDisp = sqrt(maxSqDisp);
	}//calcSrcTrajDispVecs

	//return vector of points from srcTrajData transformed from matlab space to IK pointer space
	eignVecTyp MyGestTraj::convSrcTrajToGestTraj() {
		eignVecTyp res(0);
		for (int i = 0; i < srcTrajData.size(); ++i) {//1 frame of trajtarget for every src traj value
			Eigen::Vector3d tmp(ctrPoint);
			tmp += (parentSymbol->sclAmt * srcTrajDispVecs[i]);
			res.push_back(std::move(tmp));
		}
		return res;
	}//convSrcTrajToGestTraj

	//draw component trajectories of letters based on either matlab generated/mech turk data or manufactured inter-traj connecting trajectories
	void MyGestTraj::buildTrajFromData(bool fileInit, eignVecTyp& _Initpts) {
		ctrPoint << parentSymbol->ptrCtrPt;
		if (fileInit) {//if file init then ignore _initpts and instead transform points read in from file to be bounded by a circle at a given point with a specific radius on a plane with a particular normal
			//scales around center point and using max dist point inscribed on a circle
			convTrajPts = convSrcTrajToGestTraj();
			trajLen = calcTrajLength(convTrajPts);
			trajPtDistFromSt.clear();
			trajPtDistFromSt = calcDistsFromStart(convTrajPts);
		}
		else {
			setPts(_Initpts);
		}
		trajTargets.clear();
		flags[useTrajIDX] = true;

		//TODO (maybe?) use matlab timings stored in srtTrajTiming to get drawing speed
		//using average desired speed from xml IK_desiredVel to decide how many points to find.
		//we will end up picking points by moving some percentage along trajectory and interpolating locations between end-points of that trajectory component
		perPtSpace = IKSolve->params->trajDistMult * IKSolve->params->trajDesiredVel;
		int numObjs = (int)(trajLen / perPtSpace);
		for (int i = 0; i < convTrajPts.size(); ++i) {
			debugTrajPts.push_back(convTrajPts[i]);
		}
		if(convTrajPts.size() > 1){
			if ((trajLen > 0) && (trajLen < IKSolve->params->trajLenThresh)) {
				//cout << "Traj len for traj : " << name << " smaller than threshold so traj will be ignored. |\t# conv pts : " << convTrajPts.size() << "|\t traj len : " << trajLen << std::endl;
				flags[useTrajIDX] = false;
			}
			else {
				//int beforeSize = convTrajPts.size();
				//cout << "Traj : " << name << " convTrajPts before size : " << beforeSize << "\tdesired size : " << numObjs;
				if (numObjs < 2) { 
					numObjs = 2; 
				//	cout << "\tDesired Size is small - setting to 2 : traj Len : "<< trajLen; 
				}
				//cout << std::endl;
				processPts(numObjs);
				//set actual conv traj points
				setPts(debugTrajPts);
				//cout << "Traj : " << name << " convTrajPts before size : " << beforeSize << " debugTrajPts after size : " << debugTrajPts.size() << "\tRatio : "<<(debugTrajPts.size() /(1.0 * beforeSize))<< std::endl;
			}
		}
		debugTrajPts.clear();
		//build actual tracking data trajectory
		buildFullTraj();
	}//buildTrajFromData

	//convert passed point in IK pointer space to elbow space, using appropriate transformations - project on plane ortho to elbow-shoulder vector with scaled size .25 * the distance in pointer space for symbol center (avgLoc)
	//and theta == angle of point
	Eigen::Vector3d MyGestTraj::convPtrToElbow(const Eigen::Ref<const Eigen::Vector3d>& pt) {
		//new length is .25 * length of point from avg loc
		Eigen::Vector3d tmp(pt - ctrPoint);
		tmp *= .25;
		//target plane basis vex
		//this is backwards projection onto elbow plane (for fast trajectories?)
		//Eigen::Vector3d yVec(0, 1, 0);
		Eigen::Vector3d yVec(0, -1, 0);
		//if (parentSymbol->flags[parentSymbol->isFastDrawnIDX]) { yVec *= -1; }		//reverses elbow directions for fast trajectories - "wagging arm"
		//use ptrPlaneNorm since wll never be colinear with 0,-1,0 (no drawing on ground)
		//Eigen::Vector3d R = parentSymbol->ptrPlaneNorm.cross(yVec).normalized(), S = parentSymbol->elbowPlaneNorm.cross(R).normalized();
		Eigen::Vector3d R = parentSymbol->elbowPlaneNorm.cross(yVec).normalized(), S = parentSymbol->elbowPlaneNorm.cross(R).normalized();
		return parentSymbol->elbowCtrPt + (tmp(2) * R) + (tmp(1) * S);
	}
	//build full trajectory data based on convTrajPts - only build after convTrajPts is built!
	void MyGestTraj::buildFullTraj() {
		//trajTargets is a vector of frames of IK target locations. 
		trajTargets.clear();
		for (int i = 0; i < convTrajPts.size(); ++i) {//1 frame of result for every src traj value
			eignVecTyp tmpVec(0);
			for (int j = 0; j < trkedMrkrNames.size(); ++j) {	//set all vectors for 1 frame of trajectory locations using default trajectory values
				tmpVec.push_back((*IKSolve->trkMarkers)[trkedMrkrNames[j]]->getTarPos());
			}
			trajTargets.push_back(std::move(tmpVec));
		}
		//set pointer and elbow targets
		for (int i = 0; i < convTrajPts.size(); ++i) {//1 frame of trajtarget for every converted traj value
			trajTargets[i][0] << convTrajPts[i];
			//convert convTrajPts[i] to be in "elbow space"
			trajTargets[i][1] << convPtrToElbow(convTrajPts[i]);
		}
		flags[builtIDX] = true;
	}//buildFullTraj

	//calc primary trajectory length
	double MyGestTraj::calcTrajLength(eignVecTyp& _pts) {
		double res = 0;
		Eigen::Vector3d lastPoint = _pts[0];
		for (int i = 1; i < _pts.size(); ++i) {
			res += (lastPoint - _pts[i]).norm();
			lastPoint = _pts[i];
		}
		if (flags[closeTrajIDX]) { res += (_pts[0] - lastPoint).norm(); }
		return res;
	}//calcTrajLength

	std::vector<double> MyGestTraj::calcDistsFromStart(eignVecTyp& _pts) {
		std::vector<double> res(0);
		res.push_back(0);
		Eigen::Vector3d lastPoint = _pts[0];
		for (int i = 1; i < _pts.size(); ++i) {
			res.push_back(res[i-1] + (lastPoint - _pts[i]).norm());
			lastPoint = _pts[i];
		}
		if (flags[closeTrajIDX]) { res.push_back(res[_pts.size() - 1] + (lastPoint - _pts[0]).norm()); }

		return res;
	}

	 //sets required info for points array - points and dist between pts, length, etc
	void MyGestTraj::setPts(eignVecTyp& tmpAra) {
		convTrajPts.clear();
		for (int i = 0; i < tmpAra.size(); ++i) {//1 frame of trajtarget for every src traj value
			convTrajPts.push_back(tmpAra[i]);
		}
		trajLen = calcTrajLength(convTrajPts);
		trajPtDistFromSt = calcDistsFromStart(convTrajPts);
		perPtSpace = trajLen / convTrajPts.size();
	}//setPts	

	 //subdivide, tuck, respace, resample, etc. pts of this curve
	void MyGestTraj::processPts(int numPts) {
		for (int i = 0; i < IKSolve->params->trajNumReps; ++i) {
			debugTrajPts = procPts(_tuck, debugTrajPts, .5f, flags[closeTrajIDX]);
			debugTrajPts = procPts(_tuck, debugTrajPts, -.5f, flags[closeTrajIDX]);
			debugTrajPts = procPts(_tuck, debugTrajPts, .5f, flags[closeTrajIDX]);
			debugTrajPts = procPts(_tuck, debugTrajPts, -.5f, flags[closeTrajIDX]);
			debugTrajPts = procPts(_subdivide, debugTrajPts, 2, flags[closeTrajIDX]);
		}		//smooth curve - J4
		double debugLen = calcTrajLength(debugTrajPts);
		std::vector<double> ptsDistFromSt = calcDistsFromStart(debugTrajPts);
		debugTrajPts = equiDist(debugTrajPts, debugLen, ptsDistFromSt);
		debugLen = calcTrajLength(debugTrajPts);
		ptsDistFromSt = calcDistsFromStart(debugTrajPts);
		debugTrajPts = resample(debugTrajPts, debugLen, ptsDistFromSt, numPts, flags[closeTrajIDX]);

	}

	eignVecTyp MyGestTraj::procPts(int _typ, eignVecTyp& _pts, double val, bool wrap) {
		eignVecTyp tmp(0); // temporary array
		switch (_typ) {
		case _subdivide: {
			for (int i = 0; i < _pts.size() - 1; ++i) { 
				tmp.push_back(_pts[i]); 
				for (int j = 1; j<val; ++j) {	tmp.push_back(std::move(interpVec(_pts[i], _pts[i + 1], j / val)));	} 
			}
			tmp.push_back(_pts[_pts.size() - 1]);
			return tmp; }
		case _tuck: {
			if (wrap) {				
				tmp.push_back(std::move(tuck(_pts[0],_pts[(_pts.size() - 1)], _pts[1], val)));
				for (int i = 1; i < _pts.size() - 1; ++i) {	tmp.push_back(std::move(tuck(_pts[i], _pts[(i - 1)], _pts[(i + 1)], val)));	}
				tmp.push_back(std::move(tuck(_pts[(_pts.size() - 1)], _pts[(_pts.size() - 2)], _pts[0], val)));
			}
			else {
				tmp.push_back(_pts[0]);
				for (int i = 1; i < _pts.size() - 1; ++i) {	tmp.push_back(std::move(tuck(_pts[i], _pts[(i - 1)], _pts[(i + 1)], val)));	}
				tmp.push_back(_pts[_pts.size() - 1]);
			}
			return tmp; }
		default: {break; }
		}
		return tmp;
	}

	//uses length of trajectory of _pts to respace points
	eignVecTyp MyGestTraj::equiDist(eignVecTyp& _pts, double _len, std::vector<double>& _ptsDistFromSt) {
		eignVecTyp tmp(0);
		double ratio = _len / (1.0f * _pts.size()), curDist = 0;					 //new distance between each vertex, iterative dist travelled so far			 
		for (int i = 0; i<_pts.size(); ++i) {
			tmp.push_back(at(curDist / _len, _len, _pts, _ptsDistFromSt));
			curDist += ratio;
		}
		tmp.push_back(_pts[_pts.size() - 1]);
		return tmp;
	}//equiDist

	 //uses length of trajectory of _pts to resample points based on location along length - assumes equidistant
	eignVecTyp MyGestTraj::resample(eignVecTyp& _pts, double _len, std::vector<double>& _ptsDistFromSt, int numPts, bool wrap) {
		eignVecTyp tmp(0);
		//if wrap then end at first point after wrap, otherwise end at last point
		int num = (wrap ? _pts.size() : _pts.size() - 1);
		double ratio = num / (1.0f * (numPts - 1)), f;
		int idx, newIdx = 0;
		double curDist = 0;
		for (double i = 0; i<_pts.size() - 1; i += ratio) {
			//gets every point that is -ratio- dist along trajectory.
			//tmp.push_back(at(curDist / _len, _len, _pts, _ptsDistFromSt));
			idx = (int)i;
			f = i - idx;
			tmp.push_back(std::move(interpVec(_pts[i], _pts[i + 1], f)));
			newIdx++;
		}
		if (wrap) {
			if ((tmp[newIdx - 1] - tmp[0]).norm() > ratio) {			//want to only add another point if last 2 points are further than ratio apart otherwise NOP - initial point is last point
				f = (tmp[newIdx - 1] - tmp[0]).norm() - ratio;
				tmp.push_back(std::move(interpVec(tmp[newIdx - 1], tmp[0], f)));
			} else { }
		}
		else { tmp.push_back(_pts[_pts.size() - 1]); }			//always add last point if open line/loop - want to preserve end point

		return tmp;
	}//equiDist

	 //copy the source info from passed trajectory - should make this trajectory ready for buildTrajFromData call, except for setting center and radius of circle and plane of inscribed circle, all from symbol
	void MyGestTraj::copySrcInfo(std::shared_ptr<gestureIKApp::MyGestTraj> _src) {
		avgLoc << _src->avgLoc;				//in matlab space so will not change 
		ctrPoint << parentSymbol->ptrCtrPt;
		double newScaleAmt = parentSymbol->sclAmt / _src->parentSymbol->sclAmt;
		for (int i = 0; i < numFlags; ++i) {		flags[i] = _src->flags[i];	}
		//copy convTrajPts into this traj's convTrajPts
		eignVecTyp tmpCtrToPtVecs(0);
		for (int i = 0; i < _src->convTrajPts.size(); ++i) {
			tmpCtrToPtVecs.push_back(_src->convTrajPts[i] - _src->ctrPoint);//vector of displacement in _src traj
			tmpCtrToPtVecs[i] *= newScaleAmt;
		}
		//rotate vectors to lie on plane normal to parentSymbol->ptrPlaneNorm and displace from ctrPoint by tmpCtrToPtVecs 
		eignVecTyp tmpPts(0);
		for (int i = 0; i < tmpCtrToPtVecs.size(); ++i) {
			Eigen::Vector3d tmp = tmpCtrToPtVecs[i],
				yVec(0, -1, 0), R = parentSymbol->ptrPlaneNorm.cross(yVec).normalized(), S = parentSymbol->ptrPlaneNorm.cross(R).normalized();
			tmpPts.push_back(ctrPoint + (tmp(2) * R) + (tmp(1) * S));

		}
		setPts(tmpPts);
		//build actual tracking data trajectory - before this is called, _myParent's ctrPoint, elbowPt, and normal vex need to be calced
		buildFullTraj();
	}//copySrcInfo

	/**
	* using arc length parameterisation this will return a point along the curve at a
	* particular fraction of the length of the curve (0,1 will return endpoints, .5 will return halfway along curve)
	* @param t fraction of curve length we are interested in returning a point - should be 0-1
	* @return point @t along curve
	*/
	Eigen::Vector3d MyGestTraj::at(double t,  double _ttllen, eignVecTyp& _pts, std::vector<double>& _distAtEachPt) {
		if (t<0) { std::cout << "In at : t=" << t << " needs to be [0,1]"<< std::endl; return _pts[0]; }
		else if (t>1) { std::cout << "In at : t=" << t << " needs to be [0,1]"<< std::endl; return (flags[closeTrajIDX] ? _pts[0] : _pts[_pts.size() - 1]); }//last point on closed trajectory is first point
		double dist = t * _ttllen, s;		
		for (int i = 0; i < _distAtEachPt.size() - 1; ++i) {										//built off d_pts so that it will get wrap for closed curve
			if (_distAtEachPt[i + 1] >= dist) {													//if current distance along arclength > dist we want for point
				s = ((dist - _distAtEachPt[i]) / (_distAtEachPt[i + 1] - _distAtEachPt[i]));					//needs to stay between 0 and 1 (since interpolation functions between pts will be 0-1 based), so normalize by distance d_pts[i]
				
				return interpVec(_pts[i], _pts[((i + 1) % _pts.size())], s);
			}
		}
		return _pts[0];
	}//at	


	void MyGestTraj::drawDebugTraj(dart::renderer::RenderInterface* mRI, const Eigen::Ref<const Eigen::Vector3d>& clr) {
		Eigen::Vector3d ballSz(0.004, 0.004, 0.004);
		if (flags[connTrajIDX]) {
			mRI->setPenColor(Eigen::Vector3d(.1,.1,.9));
			ballSz << 0.002, 0.002, 0.002;
		}
		else {
			mRI->setPenColor(Eigen::Vector3d(clr));
		}
		for (int i = 0; i < trajTargets.size(); ++i) {
			mRI->pushMatrix();
			mRI->translate(trajTargets[i][0]);				//ptr finger trajectory location
			mRI->drawEllipsoid(ballSz);
			mRI->popMatrix();
		}
		if (!flags[debugIDX]) {								//don't draw this if not debugging from main window (i.e. don't draw when drawing all symbol trajs for letter
			for (int i = 0; i < trajTargets.size(); ++i) {
				mRI->pushMatrix();
				mRI->translate(trajTargets[i][1]);				//elbow
				mRI->drawEllipsoid(ballSz);
				mRI->popMatrix();
			}
		}
		//draw center
		mRI->setPenColor(Eigen::Vector3d(.9, .1, .9));
		mRI->pushMatrix();
		mRI->translate(ctrPoint);				//ptr finger trajectory location
		mRI->drawEllipsoid(1.5 * ballSz);
		mRI->popMatrix();

		//mRI->pushMatrix();
		//if (flags[connTrajIDX]) {
		//	mRI->setPenColor(Eigen::Vector3d(.5, .3, .9));
		//}
		//else {
		//	Eigen::Vector3d dClr(clr);
		//	dClr(1) = 1 - dClr(1);
		//	mRI->setPenColor(dClr);
		//}	
		//for (int i = 0; i < debugTrajPts.size(); ++i) {
		//	mRI->pushMatrix();
		//	mRI->translate((debugTrajPts[i] + Eigen::Vector3d(0, 0, -0.4)));				//ptr finger trajectory location
		//	mRI->drawEllipsoid(Eigen::Vector3d(0.005, 0.005, 0.005));
		//	mRI->popMatrix();
		//}
		//mRI->popMatrix();
	}//drawDebugTraj


	 //trajectory file expected to have 1 column for every tracked marker, 1 row for every sample.  needs to be resampled for frames/playback speed (?)
	 //srcTrajData holds x,y,timing data mapped to z,y and separate timing ara.
	void MyGestTraj::readTrajFile() {//read trajectory data from filename for this trajectory
									 //cout << "Now Reading :" << filename << " trajectory file "<< std::endl;
		srcTrajData.clear();
		srtTrajTiming.clear();
		std::ifstream  trajData(filename);
		std::string line, cell;
		Eigen::Vector3d dat(0, 0, 0), tmpDat(0, 0, 0), lastDat(0, 0, 0);
		double lastTime = 0;
		int i = 0;
		//int lastTime = 0;
		while (std::getline(trajData, line)) {
			int idx = 2;
			//Eigen::Vector3d dat(0, 0, 0), tmpDat(0,0,0), lastDat(0,0,0);
			std::stringstream  ss(line);
			while (std::getline(ss, cell, ',')) {
				//put first and 2nd in idx 2 and 1 of dat, respectively, and put 3rd col in sep construct
				if (idx != 0) { dat(idx--) = stod(cell); }
				else { dat(0) = 0;		srtTrajTiming.push_back(stod(cell)); }
			}
			//NOTE Trajectory timing info from matlab includes time span between ending and beginning trajectories.  might need to normalize for this
			srcTrajData.push_back(std::move(dat));
			//debug output below : 
			if (flags[debugIDX]) {
				tmpDat << 0, dat(1), dat(2);
				std::cout << "i:" << i++ << "\tdat:(" << buildStrFrmEigV3d(dat) << "\tdist travelled:" << ((tmpDat - lastDat).norm()) << "\ttiming:" << srtTrajTiming.back() << " timing diff " << (srtTrajTiming.back() - lastTime) << std::endl;
				lastTime = srtTrajTiming.back();
				lastDat << tmpDat;
			}
		}
		flags[loadedIDX] = true;
		//cout << "Finished Reading :" << filename << " trajectory file.   srcTrajData holds : "<< srcTrajData.size()<<" points."<< std::endl;
		//cout << std::endl;
	}//readTrajFile

	std::ostream& operator<<(std::ostream& out, MyGestTraj& traj) {
		out << "Traj name " << traj.name << "\t# src traj points : " << traj.srcTrajData.size() << std::endl;
		return out;
	}
}  // namespace gestureIKApp
