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

#include "apps/gestureIK/IKSolver.h"
#include "apps/gestureIK/GestIKParams.h"

using namespace dart::dynamics;
using namespace dart::utils;
using namespace dart::math;

//using 200 iterations
//using .01 alpha
//using 1 constraint
namespace gestureIKApp {

	//TODO if doing left hand - reverse trajectories in z direction and then reverse display via scale(1,1,-1)

	IKSolver::IKSolver(dart::dynamics::SkeletonPtr _skel) :
		trkMarkers(nullptr), reach(0), //bicepLen(0),frArmLen(0), handLen(0), 
		drawCrclCtr(0, 0, 0), drawElbowCtr(0, 0, 0), tstRShldrSt(0, 0, 0), elbowShldrNormal(0,0,0),
		params(nullptr), drawShldrMrkrName(""), drawElbowMrkrName(""), skelPtr(_skel), trajectory(nullptr), mrkrWts(), numDofs(_skel->getNumDofs()), normDist(nullptr), uniDist(nullptr),
		initPose(_skel->getPositions()), newPose(), lastNewPose(), grads(), mC(), mJ(){
		//load sim parameters from default file
		//build params file from xml
		params = std::make_shared<gestureIKApp::GestIKParams>();
		loadIKParams();

		// Set joint limits
		for (size_t i = 0; i < skelPtr->getNumJoints(); ++i) {
			skelPtr->getJoint(i)->setPositionLimitEnforced(true);
		}
		//init objs
		mJ = Eigen::MatrixXd::Zero(3* trkMarkers->size(), numDofs);
		mC.setZero(3* trkMarkers->size());
		newPose = skelPtr->getPositions();
		lastNewPose = newPose;
		//normal distribution randomiser
		normDist = std::make_shared<std::normal_distribution<double> >(0, 1.0);
		uniDist = std::make_shared<std::uniform_real_distribution<double> >(0, 1.0);
	}
	IKSolver::~IKSolver(){}

	//reset reach, 
	void IKSolver::setAllArmMrkrDims() {
		std::string _finger(params->getIKPtrFingerName()), _wrist(params->getIKPtrWristName()), _elbow(params->getIKPtrElbowName()), _shldr(params->getIKPtrShldrName());
		//reset skeleton to be in initial pose
		setPoseAndCompSkelKin(initPose);
		//approx distance skel can reach without changing posture too much (rest dist from scap to ptrFinger_r) - !!!depends on skeleton arm being straight to start!!!
		reach = (skelPtr->getMarker(_finger)->getWorldPosition() - skelPtr->getMarker(_shldr)->getWorldPosition()).norm(); //only works because arm is straight
		//double bicepLen = (skelPtr->getMarker(_elbow)->getWorldPosition() - skelPtr->getMarker(_shldr)->getWorldPosition()).norm();
		//double frArmLen = (skelPtr->getMarker(_wrist)->getWorldPosition() - skelPtr->getMarker(_elbow)->getWorldPosition()).norm();
		//double handLen = (skelPtr->getMarker(_finger)->getWorldPosition() - skelPtr->getMarker(_wrist)->getWorldPosition()).norm();
		//double streach = (bicepLen + frArmLen + handLen);
		//std::cout << "bicep len : " << bicepLen << " forearm len : " << frArmLen << " hand len : " << handLen << " sum of lengths : " << streach << " reach : " << reach << "\n";
		//start location of shoulder in world
		tstRShldrSt = skelPtr->getMarker(_shldr)->getWorldPosition();
		drawShldrMrkrName.assign( _shldr);
		drawElbowMrkrName.assign(_elbow);
	}//setAllArmMrkrDims

	//load/reload all params values from xml
	void IKSolver::loadIKParams() {
		//read parameters in from xml file  "dflt_gestik_params.xml"
		std::stringstream ss;
		ss << appFilePath << baseXMLFileName;
		GestIKParser::readGestIKXML(ss.str(), params);
		std::cout << "Loaded params : " << *params << "\n";

		//loadMarkerLocs();
		//read marker locations from file defined in IK_params file
		ss.str("");
		ss << appFilePath << params->markerXMLFileName;
		GestIKParser::readMarkerLocsXML(ss.str(), params);
		std::cout << "Markers loaded from file name : " << ss.str() << "\n";

		//create markers from XML file - need to modify old markers instead if exist
		createMarkersXML();
		//TODO change to use names of markers from XML file, to support drawing with left hand
		//setAllArmMrkrDims("ptrFinger_r", "ptrWrist_r", "ptrElbow_r", "right_scapula");
		setAllArmMrkrDims();
		trkMarkers = std::make_shared<trkMrkMap>();
		//set all tracked markers (these are solved for in IKSolver)
		for (int i = 0; i < trkedMrkrNames.size(); ++i) {
			Marker* m = skelPtr->getMarker(trkedMrkrNames[i]);
			trkMarkers->insert(std::make_pair(trkedMrkrNames[i], std::make_shared<trackedMarker>(m)));
			mrkrWts.insert(std::make_pair(trkedMrkrNames[i], 1.0));		//equal weights of 1
		}
		//ptr finger 2x as important as standard, elbow 1/2 as important
		//mrkrWts["ptrFinger_r"] *= 2.0;
		//mrkrWts["ptrElbow_r"] *= .1;
		//set fixed markers to be fixed to their current position
		for (int i = 0; i < fixedMrkrNames.size(); ++i) {
			(*trkMarkers)[fixedMrkrNames[i]]->fixToLoc(true);
		}

	}
	
	//void IKSolver::loadMarkerLocs() {
	//	//read marker locations from file defined in IK_params file
	//	std::stringstream ss;
	//	ss << appFilePath << params->markerXMLFileName;
	//	GestIKParser::readMarkerLocsXML(ss.str(), params);
	//	std::cout << "Markers loaded from file name : " << ss.str() << "\n";
	//}//loadMarkerLocs
	
	//set marker names and locations on skeleton from xml file - if marker already exists then modify, otherwise create new
	void IKSolver::createMarkersXML() {
		for (std::vector<markerXMLData>::iterator it = params->markerLocs.begin(); it != params->markerLocs.end(); ++it) {
			BodyNode* bNode = skelPtr->getBodyNode(it->bnodeName);
			//check if marker exists, otherwise make new one
			Marker* m = skelPtr->getMarker(it->markerName);
			if (m == nullptr) {//m doesn't exist, add it
				m = new Marker(it->markerName, it->offset, bNode);
				bNode->addMarker(m);
			} else {//m exists, modify its offset based on values from xml
				m->setLocalPosition(it->offset);
			}
		}
	}//createMarkersXML

	//set default values for pointer finger avg loc and elbow avg loc, and plane normals for ptr finger inscribed circle and elbow inscribed circle
	//called after every time params is loaded or reloaded
	void IKSolver::setSampleCenters() {
		//build pointer center and guess at elbow center
		drawCrclCtr = skelPtr->getMarker(drawShldrMrkrName)->getWorldPosition();
		//move forward (in x dir) by reachPct
		drawCrclCtr(0) += (params->IK_reachPct * reach);
		drawCrclCtr(1) += (params->IK_ctrYOffset);
		//instead of winging it for elbow, IK to draw center and update elbow location - use this estimate as start loc
		drawElbowCtr = drawCrclCtr;
		drawElbowCtr(0) *= .5;
		drawElbowCtr(1) -= (reach * params->IK_elbowScale);
		//normal point from elbow to shoulder as arm is extended to ~average position
		elbowShldrNormal = (tstRShldrSt - drawElbowCtr).normalized();
		std::cout << "In IKSolve : arm reach : " << reach << " elbow scale : "<< params->IK_elbowScale<<"\tcenter " << buildStrFrmEigV3d(drawCrclCtr) << " elbow center " << buildStrFrmEigV3d(drawElbowCtr) << " and rad of test circle " << params->IK_drawRad << "\n";
		(*trkMarkers)[trkedMrkrNames[0]]->setTarPos(drawCrclCtr);
		(*trkMarkers)[trkedMrkrNames[1]]->setTarPos(drawElbowCtr);
		//solve IK
		solve();
		//recalc elbow center after IKing ptr to circle center location - improves guess
		drawElbowCtr = skelPtr->getMarker(drawElbowMrkrName)->getWorldPosition();
		std::cout << "Sample Elbow Location being updated after IKing to Ctr point in IKSolver" << buildStrFrmEigV3d(drawElbowCtr) << "\n";
		//normal point from elbow to shoulder as arm is extended to ~average position
		elbowShldrNormal = (tstRShldrSt - drawElbowCtr).normalized();
	}//setSampleCenters

	//compare current marker locations with target marker locations and return sq error
	//only determines error in tracked markers
	double IKSolver::getPoseError() {
		double totSqError = 0;
		Eigen::VectorXd diffRes;
		diffRes.setZero(3 * trkMarkers->size());
		int cI3 = 0;
		for (int i = 0; i < trkedMrkrNames.size(); ++i) {
			diffRes.segment<3>(cI3) = ((mrkrWts[trkedMrkrNames[i]]) * (*trkMarkers)[trkedMrkrNames[i]]->getCnstrntVec());			//gets vector from target to marker
			cI3 += 3;
		}
		return diffRes.squaredNorm();
	}


	void IKSolver::setTrkMrkrs(eignVecTyp& _tarPosAra) {
		//update target positions from passed values
		for (int i = 0; i < trkedMrkrNames.size(); ++i) {
			(*trkMarkers)[trkedMrkrNames[i]]->setTarPos(_tarPosAra[i]);
		}
	}

	//called for old symbol solves
	void IKSolver::solve(eignVecTyp& _tarPosAra) {
		setTrkMrkrs(_tarPosAra);
		solve();
	}

	//_tarPosAra is a frame's worth of target positions
	void IKSolver::solve()	{
		//update target positions from passed values
		double poseError = 999, poseErrorLast = 1000, poseErrorLastGood;
		//int adaptTS;
		int i = 0;
		Eigen::VectorXd modGrads;
		while ((i < params->IK_solveIters) && (poseError > params->IK_maxSqError)){
			grads = calcGradient();
			double mult = 10.0;
			while (mult > .1) {
				modGrads = mult * params->IK_alpha * grads;
				do {
					lastNewPose = newPose;
					newPose = skelPtr->getPositions() - modGrads;
					setPoseAndCompSkelKin(newPose);
					poseErrorLastGood = poseErrorLast;						//the last good "last cycle" pose error (i.e. keeping track of the most recent "last pose error" that triggered an iteration)
					poseErrorLast = poseError;
					poseError = getPoseError();
					//std::cout << "Iter : " << i << " adaptTS : "<< adaptTS <<" pose error : " << poseError << "\n";
					//++adaptTS;
				} while (poseError < poseErrorLast);		//repeat while improving
				//std::cout << "Iter : " << i << " adaptTS : " << adaptTS << " pose error : " << poseError << "\n";
				//after do-while, will have done 1 iteration too many (which had increased delta poseError)
				mult *= .5;
				newPose = lastNewPose;
				setPoseAndCompSkelKin(newPose);
				poseError = poseErrorLast;									//last cycle which was undone
				poseErrorLast = poseErrorLastGood;							//last "last cycle" pose error which was > current cycle pose error
			}// while (mult > .5);
			++i;
		}
		//std::cout << "Total Iters : " << i << " pose error : " << poseError << "\n";
	}//solve

	//only want to solve constraint eqs for single hierarchical branch from ptrFinger_r to root
	Eigen::VectorXd IKSolver::calcGradient() {		
		mC.setZero(3 * trkMarkers->size());
		mJ.setZero(3 * trkMarkers->size(), numDofs);
		Joint* joint;
		Eigen::Matrix4d jointToChild, parentToJoint, WrldTojnt, ttlDisp, ttlDeriv;
		BodyNode *node, *nodeNext;
		int cIter = 0, cI3;
		for (trkMrkMap::iterator it = trkMarkers->begin(); it != trkMarkers->end(); ++it) {
			cI3 = 3 * cIter;
			Marker* tarMrkr = it->second->m;
			mC.segment<3>(cI3) = it->second->getCnstrntVec();				//tarMrkr->getWorldPosition() - _tarPosAra[ci];			
			Eigen::Vector4d offset(0, 0, 0, 0);
			offset << tarMrkr->getLocalPosition(), 1;
			BodyNode *node = tarMrkr->getBodyNode();
			do {
				joint = node->getParentJoint();
				jointToChild = joint->getTransformFromChildBodyNode().inverse().matrix();
				parentToJoint = Eigen::Matrix4d::Identity();
				WrldTojnt = Eigen::Matrix4d::Identity();
				int numDegree = joint->getNumDofs();
				nodeNext = node->getParentBodyNode();
				if (nodeNext != NULL) {
					parentToJoint = joint->getTransformFromParentBodyNode().matrix();
					WrldTojnt = nodeNext->getTransform().matrix() * parentToJoint;
				}
				for (int i = 0; i < numDegree; i++) {
					ttlDeriv.setIdentity();
					for (int j = 0; j < numDegree; j++) {
						ttlDeriv *= (j != i) ? joint->getTransform(j).matrix() : joint->getTransformDerivative(j);
					}
					Eigen::Vector4d jCol = WrldTojnt * ttlDeriv * jointToChild * offset;
					mJ.block<3, 1>(cI3, joint->getIndexInSkeleton(i)) = jCol.head(3);
				}
				ttlDisp.setIdentity();
				for (int j = 0; j < numDegree; j++) { ttlDisp *= joint->getTransform(j).matrix(); }
				offset = parentToJoint * ttlDisp * jointToChild * offset;
				node = nodeNext;
			} while (node != NULL);

			++cIter;
		}//while		
		Eigen::VectorXd grad = 2 * mJ.transpose() * mC;
		return grad;
	}
	//draw each trajectory target (and fixed targets if onlyTraj == false
	void IKSolver::drawTrkMrkrs(dart::renderer::RenderInterface* mRI, bool onlyTraj) {
		if (onlyTraj) {	for (trkMrkMap::iterator it = trkMarkers->begin(); it != trkMarkers->end(); ++it) { if (!it->second->IsFixed()) { it->second->draw(mRI); } }}
		else {			for (trkMrkMap::iterator it = trkMarkers->begin(); it != trkMarkers->end(); ++it) { it->second->draw(mRI); }	}
	}

////////////////////
// tracked marker functions
////////////////////

	//draw this trackedMarker's target position
	void trackedMarker::draw(dart::renderer::RenderInterface* mRI) {
		mRI->pushMatrix();
		mRI->setPenColor(Eigen::Vector3d(.25, .25, .75));

		mRI->translate(tarPos);
		mRI->drawEllipsoid(Eigen::Vector3d(0.03, 0.03, 0.03));
		mRI->popMatrix();
	}

	std::ostream& operator<<(std::ostream& out, trackedMarker& tmrk) {
		out << "Trked M ID :" << tmrk.ID << "\tM Name : " << tmrk.m->getName() << "\tBody Node name : " << tmrk.m->getBodyNode()->getName() << "\n";
		out << "\tIs Fixed : " << (tmrk.isFixed ? "True " : "False") << "\tOrig Pos : " << buildStrFrmEigV3d(tmrk.origPos) << "\tCurrent Target Pos : " << buildStrFrmEigV3d(tmrk.tarPos) << "\n";
		return out;
	}

}