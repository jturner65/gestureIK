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

#include <chrono>
#include "apps/gestureIK/GestIKParams.h"

namespace gestureIKApp {

	GestIKParams::GestIKParams() : defaultVals(), dataCapNumExamples(100), //dataCapTestTrainThresh(.5),
		IK_reachPct(.75), IK_drawRad(.2), IK_solveIters(100), IK_alpha(0.01), IK_maxSqError(0.0001), IK_elbowScale(0.36), IK_fastTrajMult(1.5), IK_ctrYOffset(0),
		trajLenThresh(0.01), trajRandCtrStd(.1), trajRandSclStd(.1), trajRandCtrStdScale_X(1), trajRandCtrStdScale_Y(1), trajRandCtrStdScale_Z(1),
		trajNumReps(5), trajDistMult(.1), trajDesiredVel(.03), trajNev4OvPct(.2), rnd_camThet(.789), win_Width(800), win_Height(800), numLetters(26), numTotSymPerLtr(0), ltrIdxStSave(0), fixedClipLen(16), origZoom(.65f),
		clipCountOffset(0), dataType(MULT_8),
		bkgR(1.0), bkgG(1.0), bkgB(1.0), bkgA(1.0), dateFNameOffset(""),
		flags(numFlags, false)
	{
		std::cout << "GestIK params ctor"<<std::endl;
		setDefaultVals();																//set defaults from hardcoded values - overridden by xml data if necessary
	}

	GestIKParams::~GestIKParams() {}

	void GestIKParams::setParamValFromXMLStr(const std::string& _name, const std::string& s) {
		//cout << "Setting value : " << s << " from xml for " << _name << std::endl;
		//doubles
		if (_name.compare("IK_alpha") == 0) { IK_alpha = stod(s);      return; }
		if (_name.compare("IK_maxSqError") == 0) { IK_maxSqError = stod(s);      return; }
		if (_name.compare("IK_elbowScale") == 0) { IK_elbowScale = stod(s);      return; }
		if (_name.compare("IK_fastTrajMult") == 0) { IK_fastTrajMult = stod(s);      return; }		
		if (_name.compare("IK_reachPct") == 0) { IK_reachPct = stod(s);      return; }
		if (_name.compare("IK_drawRad") == 0) { IK_drawRad = stod(s);      return; }
		if (_name.compare("IK_ctrYOffset") == 0) { IK_ctrYOffset = stod(s);      return; }

		//if (_name.compare("dataCapTestTrainThresh") == 0) { dataCapTestTrainThresh = stod(s);      return; }
		if (_name.compare("trajLenThresh") == 0) { trajLenThresh = stod(s);      return; }
		if (_name.compare("trajDistMult") == 0) { trajDistMult = stod(s);      return; }
		if (_name.compare("trajDesiredVel") == 0) { trajDesiredVel = stod(s); return; }
		if (_name.compare("trajNev4OvPct") == 0) { trajNev4OvPct = stod(s); return; }
		if (_name.compare("rnd_camThet") == 0) { rnd_camThet = stod(s); return; }
		
		if (_name.compare("trajRandCtrStd") == 0) { trajRandCtrStd = stod(s); return; }
		if (_name.compare("trajRandSclStd") == 0) { trajRandSclStd = stod(s); return; }
		if (_name.compare("trajRandCtrStdScale_X") == 0) { trajRandCtrStdScale_X = stod(s); return; }
		if (_name.compare("trajRandCtrStdScale_Y") == 0) { trajRandCtrStdScale_Y = stod(s); return; }
		if (_name.compare("trajRandCtrStdScale_Z") == 0) { trajRandCtrStdScale_Z = stod(s); return; }

		if (_name.compare("bkgR") == 0) { bkgR = stod(s); return; }
		if (_name.compare("bkgG") == 0) { bkgG = stod(s); return; }
		if (_name.compare("bkgB") == 0) { bkgB = stod(s); return; }
		if (_name.compare("bkgA") == 0) { bkgA = stod(s); return; }
		//ints
		if (_name.compare("dataCapNumExamples") == 0) { dataCapNumExamples = stoi(s);			return; }
		if (_name.compare("IK_solveIters") == 0) { IK_solveIters = stoi(s);			return; }
		if (_name.compare("trajNumReps") == 0) { trajNumReps = stoi(s);			return; }
		if (_name.compare("win_Width") == 0) { win_Width = stoi(s);			return; }
		if (_name.compare("win_Height") == 0) { win_Height = stoi(s);			return; }
		if (_name.compare("numLetters") == 0) { numLetters = stoi(s);			return; }
		if (_name.compare("numTotSymPerLtr") == 0) { numTotSymPerLtr = stoi(s);			return; }
		if (_name.compare("ltrIdxStSave") == 0) { ltrIdxStSave = stoi(s);	if (ltrIdxStSave >= numLetters) { ltrIdxStSave  = numLetters-1;} 		return; }
		if (_name.compare("fixedClipLen") == 0) { fixedClipLen = stoi(s);			return; }
		if (_name.compare("clipCountOffset") == 0) { clipCountOffset = stoi(s); return; }
		//enum
		if (_name.compare("dataType") == 0) {	dataType = static_cast<DataType>(stoi(s)); return;	}

		//floats 
		if (_name.compare("origZoom") == 0) { origZoom = stof(s);			return; }
		//strings
		//if (_name.compare("skelFileName") == 0) { skelFileName = std::string(s);	return; }
		//boolean
		//std::string sDest(s);
		//std::transform(s.begin(), s.end(), sDest.begin(), toupper);
		if (_name.compare("IDX_useLeftHand") == 0) { flags[IDX_useLeftHand] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_genRandLtrs") == 0) { flags[IDX_genRandLtrs] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_regenNotAppend") == 0) { flags[IDX_regenNotAppend] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_chgTrajDbgClrs") == 0) { flags[IDX_chgTrajDbgClrs] = (s.compare("TRUE") == 0 ? true : false);        return; }

	}	//setParamValFromXMLStr

	//set up reasonable default values to be used if XML is unavailable or unused.  the default values will be used whenever reset is called
	void GestIKParams::setDefaultVals() {
		//use as template - set any class variables to reasonable defaults
		//fMaxSDMult = 4;						//fMaxSDMult is magic number from Unity code that multiples the sd for the per-step priors for the force limits- UI enterable? hardcoded to 4.0 in unity code
		dataCapNumExamples = 100;
		//dataCapTestTrainThresh = .5;

		IK_reachPct = .75;
		IK_solveIters = 100;
		IK_alpha = 0.01;
		IK_drawRad = .2;
		IK_maxSqError = 0.0001;
		IK_elbowScale = 0.36;
		IK_fastTrajMult = 1.5;
		IK_ctrYOffset = 0;

		trajLenThresh = 0.01;
		trajRandCtrStd = .1;
		trajRandSclStd = .1;

		trajRandCtrStdScale_X = 1; 
		trajRandCtrStdScale_Y = 1; 
		trajRandCtrStdScale_Z = 1;

		trajDistMult = 0.1;
		trajNumReps = 5;
		trajDesiredVel = .03;
		trajNev4OvPct = .2;
		rnd_camThet = .789;

		win_Width = 800;
		win_Height = 800;

		numLetters = 26;
		numTotSymPerLtr = 40;

		ltrIdxStSave = 0;
		fixedClipLen = 16;
		origZoom = .65f;
		clipCountOffset = 0;
		
		dataType = MULT_8;

		bkgR = 1.0;
		bkgG = 1.0;
		bkgB = 1.0;
		bkgA = 1.0;

		flags[IDX_useLeftHand] = false;
		flags[IDX_regenNotAppend] = true;
		flags[IDX_genRandLtrs] = true;
		flags[IDX_chgTrajDbgClrs] = false;
		defaultVals = accumulateVals();			//set default values as current param vals
	}//setDefaultVals
	//set current date as file name offset to keep files unique
	void GestIKParams::setDateFNameOffset() {
		std::stringstream ss;
		std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
		std::time_t now_c = std::chrono::system_clock::to_time_t(now);
		ss << std::put_time(std::localtime(&now_c), "%y%m%d%H");
		dateFNameOffset = ss.str();
		std::cout << "dateFNameOffset : " << dateFNameOffset << std::endl;
	}


	//allow current values from UI to be set as defaults 
	void GestIKParams::setCurrentValsAsDefault() {
		defaultVals = accumulateVals();
	}//setCurrentValsAsDefault

	std::vector<double> GestIKParams::accumulateVals() {		//grab all locals that are modifiable by UI or used elsewhere
		std::vector<double> res;
		res.push_back(dataCapNumExamples);
		//res.push_back(dataCapTestTrainThresh);

		res.push_back(IK_reachPct);
		res.push_back(IK_solveIters);
		res.push_back(IK_drawRad);
		res.push_back(IK_alpha);
		res.push_back(IK_maxSqError);
		res.push_back(IK_elbowScale);
		res.push_back(IK_fastTrajMult);
		res.push_back(IK_ctrYOffset);

		res.push_back(trajLenThresh);
		res.push_back(trajRandCtrStd);
		res.push_back(trajRandSclStd);

		res.push_back(trajRandCtrStdScale_X);
		res.push_back(trajRandCtrStdScale_Y);
		res.push_back(trajRandCtrStdScale_Z);

		res.push_back(trajDistMult);
		res.push_back(trajNumReps);
		res.push_back(trajDesiredVel);
		res.push_back(trajNev4OvPct);
		res.push_back(rnd_camThet);

		res.push_back(win_Width);
		res.push_back(win_Height);

		res.push_back(numLetters);
		res.push_back(numTotSymPerLtr);

		res.push_back(ltrIdxStSave);
		res.push_back(fixedClipLen);
		res.push_back(origZoom);
		res.push_back(clipCountOffset);
		res.push_back(dataType);

		res.push_back(bkgR);
		res.push_back(bkgG);
		res.push_back(bkgB);
		res.push_back(bkgA);

		//represent boolean flags as vals
		for (int i = 0; i < numFlags; ++i) {
			res.push_back((flags[i] ? 1 : 0));
		}
		return res;
	}//accumulateVals

	void GestIKParams::distributeVals(std::vector<double>& vals) {//copy all UI values to their appropriate lcl variables (from UI or file)
		int idx = 0;
		dataCapNumExamples = (int)floor(vals[idx++] + .5);
		//dataCapTestTrainThresh = vals[idx++];

		IK_reachPct = vals[idx++];
		IK_solveIters = (unsigned int)floor(vals[idx++] + .5);
		IK_drawRad = vals[idx++];
		IK_alpha = vals[idx++];
		IK_maxSqError = vals[idx++];
		IK_elbowScale = vals[idx++];
		IK_fastTrajMult = vals[idx++];
		IK_ctrYOffset = vals[idx++];

		trajLenThresh = vals[idx++];
		trajRandCtrStd = vals[idx++];
		trajRandSclStd = vals[idx++];

		trajRandCtrStdScale_X = vals[idx++];
		trajRandCtrStdScale_Y = vals[idx++];
		trajRandCtrStdScale_Z = vals[idx++];
		
		trajDistMult = vals[idx++];
		trajNumReps = (int)floor(vals[idx++] + .5);
		trajDesiredVel = vals[idx++];
		trajNev4OvPct = vals[idx++];
		rnd_camThet = vals[idx++];

		win_Width = (int)floor(vals[idx++] + .5);
		win_Height = (int)floor(vals[idx++] + .5);

		numLetters = (int)floor(vals[idx++] + .5);
		numTotSymPerLtr = (int)floor(vals[idx++] + .5);

		ltrIdxStSave = (int)floor(vals[idx++] + .5);
		fixedClipLen = (int)floor(vals[idx++] + .5);
		origZoom = vals[idx++];
		clipCountOffset = (int)floor(vals[idx++] + .5);
		dataType = static_cast<DataType>(((int)floor(vals[idx++] + .5) % 3));
		bkgR = vals[idx++];
		bkgG = vals[idx++];
		bkgB = vals[idx++];
		bkgA = vals[idx++];
		//represent boolean flags as vals
		for (int i = 0; i < numFlags; ++i) {flags[i] = (vals[idx++] == 1.0);}
	}//distributeVals
	void GestIKParams::resetValues() {//set some reasonable defaults in here that we can return to them if not loaded from file 
		distributeVals(defaultVals);
	}

	std::ostream& operator<<(std::ostream& out, GestIKParams& p) {//for dbug output 
		out << "GestIK Params values : "<<"\tdateFNameOffset : "<< p.dateFNameOffset<< std::endl;
		out << "Letter data type : " << DataType2str[p.dataType] << "\tdataCapNumExamples : " << p.dataCapNumExamples << "\t rnd_camThet : " << p.rnd_camThet;// << "\t| dataCapTestTrainThresh : " << p.dataCapTestTrainThresh
		out << "\t| IK_reachPct : " << p.IK_reachPct << "\t| IK_solveIters : " << p.IK_solveIters << std::endl;
		out << "IK_alpha  : " << p.IK_alpha << "\t| IK_drawRad  : " << p.IK_drawRad << "\t| IK_maxSqError  : " << p.IK_maxSqError << "\t| IK_elbowScale  : " << p.IK_elbowScale << "\t| IK_fastTrajMult  : " << p.IK_fastTrajMult << "\t| IK_ctrYOffset  : " << p.IK_ctrYOffset << std::endl;
		out << "trajLenThresh : " << p.trajLenThresh << "\t| trajDistMult : " << p.trajDistMult  << "\ttrajNumReps : " << p.trajNumReps << "\t| trajDesiredVel : " << p.trajDesiredVel << "\t| trajNev4OvPct : " << p.trajNev4OvPct << "\t| win_Width : " << p.win_Width << "\t| win_Height : " << p.win_Height << "\t| orig zoom : " << p.origZoom << std::endl;
		out << "clipCountOffset : " << p.clipCountOffset << "\t#letters (out of 26) to load : "<<p.numLetters <<"\t| traj rand scale x,y,z ("   << p.trajRandCtrStdScale_X<<", "<<p.trajRandCtrStdScale_Y << ", " << p.trajRandCtrStdScale_Z <<")\t| bkg clr (rgba) : (" << p.bkgR << ", " << p.bkgG << ", " << p.bkgB << ", " << p.bkgA << ")"<< std::endl;
		out << "Generate left hand data : " << (p.flags[p.IDX_useLeftHand] ? "True" : "False") << "\tGenerate Random Letters : " << (p.flags[p.IDX_genRandLtrs] ? "True" : "False") << "\tSave Non-random (File-based) letter IK (set to false to ignore these to add more random sequences): " << (p.flags[p.IDX_regenNotAppend] ? "True" : "False") << std::endl;
		out << "Display trajectories with different colors : " << (p.flags[p.IDX_chgTrajDbgClrs] ? "True" : "False")  << std::endl;
		return out;
	}

}
