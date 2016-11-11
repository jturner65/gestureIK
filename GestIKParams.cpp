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

#include "apps/gestureIK/GestIKParams.h"

using namespace std;
using namespace Eigen;

namespace gestureIKApp {

	GestIKParams::GestIKParams() : defaultVals(), dataCapNumExamples(100), dataCapTestTrainThresh(.5),
		IK_reachPct(.75), IK_drawRad(.2), IK_solveIters(100), IK_alpha(0.01), IK_maxSqError(0.0001), trajLenThresh(0.01),
		trajNumReps(5), trajDistMult(.1), trajDesiredVel(.03), trajNev4OvPct(.2),
		flags(numFlags, false)
	{
		cout << "GestIK params ctor\n";
		setDefaultVals();																//set defaults from hardcoded values - overridden by xml data if necessary
		setCurrentValsAsDefault();
	}

	GestIKParams::~GestIKParams() {}

	void GestIKParams::setParamValFromXMLStr(const std::string& _name, const std::string& s) {
		//cout << "Setting value : " << s << " from xml for " << _name << "\n";
		//doubles
		if (_name.compare("IK_alpha") == 0) { IK_alpha = stod(s);      return; }
		if (_name.compare("IK_maxSqError") == 0) { IK_maxSqError = stod(s);      return; }
		if (_name.compare("IK_reachPct") == 0) { IK_reachPct = stod(s);      return; }
		if (_name.compare("IK_drawRad") == 0) { IK_drawRad = stod(s);      return; }
		if (_name.compare("dataCapTestTrainThresh") == 0) { dataCapTestTrainThresh = stod(s);      return; }
		if (_name.compare("trajLenThresh") == 0) { trajLenThresh = stod(s);      return; }
		if (_name.compare("trajDistMult") == 0) { trajDistMult = stod(s);      return; }
		if (_name.compare("trajDesiredVel") == 0) { trajDesiredVel = stod(s); return; }
		if (_name.compare("trajNev4OvPct") == 0) { trajNev4OvPct = stod(s); return; }
		//ints
		if (_name.compare("dataCapNumExamples") == 0) { dataCapNumExamples = stoi(s);			return; }
		if (_name.compare("IK_solveIters") == 0) { IK_solveIters = stoi(s);			return; }
		if (_name.compare("trajNumReps") == 0) { trajNumReps = stoi(s);			return; }
		//strings
		//if (_name.compare("skelFileName") == 0) { skelFileName = std::string(s);	return; }
		//boolean
		//if (_name.compare("useGBP") == 0) { flags[IDX_useGBP] = (s.compare("TRUE") == 0 ? true : false);        return; }

	}	//set up reasonable default values to be used if XML is unavailable or unused.  the default values will be used whenever reset is called
	void GestIKParams::setDefaultVals() {
		//use as template - set any class variables to reasonable defaults
		//fMaxSDMult = 4;						//fMaxSDMult is magic number from Unity code that multiples the sd for the per-step priors for the force limits- UI enterable? hardcoded to 4.0 in unity code
		dataCapNumExamples = 100;
		dataCapTestTrainThresh = .5;
		IK_reachPct = .75;
		IK_solveIters = 100;
		IK_alpha = 0.01;
		IK_drawRad = .2;
		IK_maxSqError = 0.0001;
		trajLenThresh = 0.01;
		trajDistMult = 0.1;
		trajNumReps = 5;
		trajDesiredVel = .03;
		trajNev4OvPct = .2;
		flags[IDX_useLeftHand] = false;
		defaultVals = accumulateVals();			//set default values as current param vals
	}//setDefaultVals
	//allow current values from UI to be set as defaults 
	void GestIKParams::setCurrentValsAsDefault() {
		defaultVals = accumulateVals();
	}//setCurrentValsAsDefault

	vector<double> GestIKParams::accumulateVals() {		//grab all locals that are modifiable by UI or used elsewhere
		vector<double> res;
		int idx = 0;
		res.push_back(dataCapNumExamples);
		res.push_back(dataCapTestTrainThresh);
		res.push_back(IK_reachPct);
		res.push_back(IK_solveIters);
		res.push_back(IK_alpha);
		res.push_back(IK_maxSqError);
		res.push_back(trajLenThresh);
		res.push_back(trajDistMult);
		res.push_back(trajNumReps);
		res.push_back(trajDesiredVel);
		res.push_back(trajNev4OvPct);

		return res;
	}

	void GestIKParams::distributeVals(vector<double>& vals) {//copy all UI values to their appropriate lcl variables (from UI or file)
		//int idx = 0, lIdx = 0, hIdx = 0, lfIdx = 0, hfIdx = 0;
		//armStdScale = vals[idx++];				//add slider for fMaxSDMult
		//TODO no UI currently modifies values so only reading xml file should adjust IK/datacollection params
	}
	void GestIKParams::resetValues() {//set some reasonable defaults in here that we can return to them if not loaded from file
		distributeVals(defaultVals);
		flags[IDX_useLeftHand] = false;
	}

	std::ostream& operator<<(std::ostream& out, GestIKParams& p) {//for dbug output 
		out << "GestIK Params values : \n";
		out << "dataCapNumExamples : " << p.dataCapNumExamples << "\t| dataCapTestTrainThresh : " << p.dataCapTestTrainThresh << "\t| IK_reachPct : " << p.IK_reachPct << "\t| IK_solveIters : " << p.IK_solveIters << "\n";
		out << "IK_alpha  : " << p.IK_alpha << "\t| IK_drawRad  : " << p.IK_drawRad << "\t| IK_maxSqError  : " << p.IK_maxSqError << "\t| trajLenThresh : " << p.trajLenThresh << "\n";
		out << "trajDistMult : " << p.trajDistMult << "\t| trajNumReps : " << p.trajNumReps << "\t| trajDesiredVel : " << p.trajDesiredVel <<"\t| trajNev4OvPct : "<< p.trajNev4OvPct <<"\n";
		out << "Use Left Hand to Draw : " << (p.flags[p.IDX_useLeftHand] ? "True" : "False")  << "\n";
		return out;
	}

}
