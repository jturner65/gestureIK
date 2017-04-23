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

	GestIKParams::GestIKParams() : defaultVals(), markerLocs(),
		IK_reachPct(.75), IK_drawRad(.2), IK_solveIters(100), IK_alpha(0.01), IK_maxSqError(0.0001), IK_elbowScale(0.36), IK_fastTrajMult(1.5), IK_ctrYOffset(0),
		trajLenThresh(0.01), trajRandCtrStd(.1), trajRandSclStd(.1), trajRandCtrStdScale_X(1), trajRandCtrStdScale_Y(1), trajRandCtrStdScale_Z(1),
		trajNumReps(5), trajDistMult(.1), trajDesiredVel(.03), trajNev4OvPct(.2), rnd_camThet(.35), rnd_camZoom(.05), rnd_camTrans(.25), rnd_headClrBnd(.2), rnd_headDimPct(.1), rnd_handClrBnd(.2),rnd_handDimPct(.1),
		win_Width(800), win_Height(800), numTotSymPerLtr(0), fixedClipLen(16), mBlurPreFrames(2), mBlurPostFrames(2), dbgTrajDrawn(3),  origZoom(.65f),
		dataType(VAR_VEL), bkgR(1.0), bkgG(1.0), bkgB(1.0), bkgA(1.0), dateFNameOffset(""), baseOutDir(""), srcLtrDir(""), markerXMLFileName(""), flags(numFlags, false)
	{
		std::cout << "GestIK params ctor"<<std::endl;
		setDefaultVals();																//set defaults from hardcoded values - overridden by xml data if necessary
	}

	GestIKParams::~GestIKParams() {}

	void GestIKParams::setParamValFromXMLStr(const std::string& _name, const std::string& s) {
		//cout << "Setting value : " << s << " from xml for " << _name << "\n";
		//doubles
		if (_name.compare("IK_alpha") == 0) { IK_alpha = stod(s);      return; }
		if (_name.compare("IK_maxSqError") == 0) { IK_maxSqError = stod(s);      return; }
		if (_name.compare("IK_elbowScale") == 0) { IK_elbowScale = stod(s);      return; }
		if (_name.compare("IK_fastTrajMult") == 0) { IK_fastTrajMult = stod(s);      return; }		
		if (_name.compare("IK_reachPct") == 0) { IK_reachPct = stod(s);      return; }
		if (_name.compare("IK_drawRad") == 0) { IK_drawRad = stod(s);      return; }
		if (_name.compare("IK_ctrYOffset") == 0) { IK_ctrYOffset = stod(s);      return; }

		if (_name.compare("trajLenThresh") == 0) { trajLenThresh = stod(s);   return; }
		if (_name.compare("trajDistMult") == 0) { trajDistMult = stod(s);  return; }
		if (_name.compare("trajDesiredVel") == 0) { trajDesiredVel = stod(s); return; }
		if (_name.compare("trajNev4OvPct") == 0) { trajNev4OvPct = stod(s); return; }

		if (_name.compare("rnd_camThet") == 0) { rnd_camThet = stod(s); return; }
		if (_name.compare("rnd_camZoom") == 0) { rnd_camZoom = stod(s);	return; }
		if (_name.compare("rnd_camTrans") == 0) { rnd_camTrans = stod(s); return; }

		if (_name.compare("rnd_headClrBnd") == 0) { rnd_headClrBnd = stod(s); return; }
		if (_name.compare("rnd_headDimPct") == 0) { rnd_headDimPct = stod(s); return; }
		if (_name.compare("rnd_handClrBnd") == 0) { rnd_handClrBnd = stod(s); return; }
		if (_name.compare("rnd_handDimPct") == 0) { rnd_handDimPct = stod(s); return; }

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
		if (_name.compare("IK_solveIters") == 0) { IK_solveIters = stoi(s);			return; }
		if (_name.compare("trajNumReps") == 0) { trajNumReps = stoi(s);			return; }
		if (_name.compare("win_Width") == 0) { win_Width = stoi(s);			return; }
		if (_name.compare("win_Height") == 0) { win_Height = stoi(s);			return; }
		if (_name.compare("numTotSymPerLtr") == 0) { numTotSymPerLtr = stoi(s);			return; }
		if (_name.compare("fixedClipLen") == 0) { fixedClipLen = stoi(s);			return; }
		if (_name.compare("mBlurPreFrames") == 0) { mBlurPreFrames = stoi(s);			return; }
		if (_name.compare("mBlurPostFrames") == 0) { mBlurPostFrames = stoi(s);			return; }
		if (_name.compare("dbgTrajDrawn") == 0) { dbgTrajDrawn = stoi(s);			return; }
		
		//enum
		if (_name.compare("dataType") == 0) {	dataType = static_cast<DataType>(stoi(s)); return;	}
		//floats 
		if (_name.compare("origZoom") == 0) { origZoom = stof(s);			return; }
		//strings
		if (_name.compare("baseOutDir") == 0) { baseOutDir = std::string(s);	return; }
		if (_name.compare("srcLtrDir") == 0) { srcLtrDir = std::string(s);	return; }
		if (_name.compare("markerXMLFileName") == 0) { markerXMLFileName = std::string(s);	return; }
		//boolean
		//std::string sDest(s);
		//std::transform(s.begin(), s.end(), sDest.begin(), toupper);
		if (_name.compare("IDX_useLeftHand") == 0) { flags[IDX_useLeftHand] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_chgTrajDbgClrs") == 0) { flags[IDX_chgTrajDbgClrs] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_rndCamOrient") == 0) {	flags[IDX_rndCamOrient] = (s.compare("TRUE") == 0 ? true : false);        return; }          
		if (_name.compare("IDX_rndCamLoc") == 0) {		flags[IDX_rndCamLoc] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_rndHeadDims") == 0) {	flags[IDX_rndHeadDims] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_rndHeadClr") == 0) {		flags[IDX_rndHeadClr] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_rndHandShape") == 0) {	flags[IDX_rndHandShape] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_rndHandDims") == 0) {	flags[IDX_rndHandDims] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_rndHandClr") == 0) { flags[IDX_rndHandClr] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_useMotionBlur") == 0) { flags[IDX_useMotionBlur] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_useOutputDir") == 0) { flags[IDX_useOutputDir] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_saveHandCOMVals") == 0) { flags[IDX_saveHandCOMVals] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_alwaysSaveImgs") == 0) { flags[IDX_alwaysSaveImgs] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("IDX_useCustSrcLtrs") == 0) { flags[IDX_useCustSrcLtrs] = (s.compare("TRUE") == 0 ? true : false);        return; }

	}	//setParamValFromXMLStr
	//construct and return name of directory holding letter sequences or csv's of com locations in screen space
	std::string GestIKParams::getDataOutputDir(bool isCSV) {
		std::stringstream ss;
		ss << getOutputBaseDir(); 
		if (flags[IDX_useMotionBlur]) {			ss << "blur_";		}
		else if (isCSV) { ss << "COMVals_"; }
		ss << DataType2strAbbrev[dataType] << "_" << dateFNameOffset << "/"; 		
		return ss.str();
	}//getDataOutputDir

	 //set current date as file name offset to keep files unique
	void GestIKParams::setDateFNameOffset() {
		std::stringstream ss;
		std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
		std::time_t now_c = std::chrono::system_clock::to_time_t(now);
		//changed to include minute, exlude year
		ss << std::put_time(std::localtime(&now_c), "%m%d%H%M");
		dateFNameOffset = ss.str();
		std::cout << "dateFNameOffset : " << dateFNameOffset << "\n";
	}//setDateFNameOffset

	//save values of marker locations and body nodes they are attached to from XML - this will be then applied to loaded skeleton in IKSolver 
	void GestIKParams::setMarkerLocVals(const std::string& bName, const std::string& offsetStr, const std::string& mName) {
		markerLocs.push_back(markerXMLData(bName, offsetStr, mName));
	}

	//set up reasonable default values to be used if XML is unavailable or unused.  the default values will be used whenever reset is called
	void GestIKParams::setDefaultVals() {
		//use as template - set any class variables to reasonable defaults
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
		rnd_camThet = .35;
		rnd_camZoom = .05;
		rnd_camTrans = .25;
		rnd_headClrBnd = .2;
		rnd_headDimPct = .1;
		rnd_handClrBnd = .2;
		rnd_handDimPct = .1;

		win_Width = 800;
		win_Height = 800;
		numTotSymPerLtr = 40;
		fixedClipLen = 16;
		mBlurPreFrames = 2;
		mBlurPostFrames = 2;
		dbgTrajDrawn = 3;

		origZoom = .65f;
		
		dataType = VAR_VEL;

		bkgR = 1.0;
		bkgG = 1.0;
		bkgB = 1.0;
		bkgA = 1.0;
		//default output directory 
		baseOutDir = DART_ROOT_PATH"apps/gestureIK/frames/";
		//default is base directory - ALWAYS APPENDS DART ROOT PATH when consumed so directory does not need to be specified within xml file
		srcLtrDir = "apps/gestureIK/sourceLetters/";
		markerXMLFileName = "";
	
		for (int i = 0; i < numFlags; ++i) { flags[i] = false; }
		flags[IDX_alwaysSaveImgs] = true;//default this to true

		defaultVals = accumulateVals();			//set default values as current param vals
	}//setDefaultVals

	//allow current values from UI to be set as defaults 
	void GestIKParams::setCurrentValsAsDefault() {
		defaultVals = accumulateVals();
	}//setCurrentValsAsDefault

	//save all param vals into single array
	std::vector<double> GestIKParams::accumulateVals() {		
		std::vector<double> res;

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
		res.push_back(rnd_camZoom);
		res.push_back(rnd_camTrans);
		res.push_back(rnd_headClrBnd);
		res.push_back(rnd_headDimPct);
		res.push_back(rnd_handClrBnd);
		res.push_back(rnd_handDimPct);

		res.push_back(win_Width);
		res.push_back(win_Height);
		res.push_back(numTotSymPerLtr);
		res.push_back(fixedClipLen);
		res.push_back(mBlurPreFrames);
		res.push_back(mBlurPostFrames);
		res.push_back(dbgTrajDrawn);

		res.push_back(origZoom);
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
		rnd_camZoom = vals[idx++];
		rnd_camTrans = vals[idx++];
		rnd_headClrBnd = vals[idx++];
		rnd_headDimPct = vals[idx++];
		rnd_handClrBnd = vals[idx++];
		rnd_handDimPct = vals[idx++];

		win_Width = (int)floor(vals[idx++] + .5);
		win_Height = (int)floor(vals[idx++] + .5);
		numTotSymPerLtr = (int)floor(vals[idx++] + .5);
		fixedClipLen = (int)floor(vals[idx++] + .5);
		mBlurPreFrames = (int)floor(vals[idx++] + .5);
		mBlurPostFrames = (int)floor(vals[idx++] + .5);
		dbgTrajDrawn = (int)floor(vals[idx++] + .5);		

		origZoom = vals[idx++];
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
		out << "Letter data type : " << DataType2str[p.dataType] << "\t# total Symbols per letter : " << p.numTotSymPerLtr << "\t| IK_reachPct : " << p.IK_reachPct << "\t| IK_solveIters : " << p.IK_solveIters << "\n";
		out << "Random Env/Skel vals : rnd_camThet : " << p.rnd_camThet << "\t rnd_camZoom : " << p.rnd_camZoom << "\t rnd_camTrans : " << p.rnd_camTrans << "\n";
		out << "\t rnd_headClrBnd : " << p.rnd_headClrBnd << "\t rnd_headDimPct : " << p.rnd_headDimPct << "\t rnd_handClrBnd : " << p.rnd_handClrBnd << "\t rnd_handDimPct : " << p.rnd_handDimPct << "\n";
		out << "IK_alpha  : " << p.IK_alpha << "\t| IK_drawRad  : " << p.IK_drawRad << "\t| IK_maxSqError  : " << p.IK_maxSqError << "\t| IK_elbowScale  : " << p.IK_elbowScale << "\t| IK_fastTrajMult  : ";
		out << p.IK_fastTrajMult << "\t| IK_ctrYOffset  : " << p.IK_ctrYOffset << "\n";
		out << "trajLenThresh : " << p.trajLenThresh << "\t| trajDistMult : " << p.trajDistMult << "\ttrajNumReps : " << p.trajNumReps << "\t| trajDesiredVel : " << p.trajDesiredVel << "\t| trajNev4OvPct : " << p.trajNev4OvPct;
		out << "\t| win_Width : " << p.win_Width << "\t| win_Height : " << p.win_Height << "\t| orig zoom : " << p.origZoom << "\n";
		out << "\t| traj rand scale x,y,z (" << p.trajRandCtrStdScale_X << ", " << p.trajRandCtrStdScale_Y << ", " << p.trajRandCtrStdScale_Z << ")";
		out << "\t| bkg clr(rgba) : (" << p.bkgR << ", " << p.bkgG << ", " << p.bkgB << ", " << p.bkgA << ")\n";
		out << "Motion Blur : " << (p.flags[p.IDX_useMotionBlur] ? "True" : "False");
		if (p.flags[p.IDX_useMotionBlur]) { 	out << " : Preframes used to build blur : " << (p.mBlurPreFrames) << " : Postframes used to build blur : " << (p.mBlurPostFrames) << "\n";	}
		out << "\tUse custom directory for source trajectories : " << (p.flags[p.IDX_useCustSrcLtrs] ? "True : Dir Specified : " : "False") << (p.flags[p.IDX_useCustSrcLtrs] ? p.getSrcLtrsPath() : "") << "\n";
		out << "\tUse XML-Specified Output Dir : " << (p.flags[p.IDX_useOutputDir] ? "True : Dir Specified : " : "False") << (p.flags[p.IDX_useOutputDir] ? p.baseOutDir : "") << "\n";
		out << "\tFile to use for marker locations specification : " << p.markerXMLFileName << "\n\n";
		out << "Always Save Screenshots : " << (p.flags[p.IDX_alwaysSaveImgs] ? "True" : "False") << "\n\n";
		out << "Generate data focused on left hand (TODO) : " << (p.flags[p.IDX_useLeftHand] ? "True" : "False") << "\tDisplay trajs with different colors : " << (p.flags[p.IDX_chgTrajDbgClrs] ? "True\n" : "False\n") ;
		out << "Save COM-related values of hand and elbow : " << (p.flags[p.IDX_saveHandCOMVals] ? "True\n" : "False\n") ;
		out << "Randomize :\n";
		out << "\tCamera Orientation : " << (p.flags[p.IDX_rndCamOrient] ? "True" : "False") << "\t Camera Location / Zoom : " << (p.flags[p.IDX_rndCamLoc] ? "True" : "False");
		out << "\tHead Dimensions: " << (p.flags[p.IDX_rndHeadDims] ? "True" : "False") << "\t  Head Color: " << (p.flags[p.IDX_rndHeadClr] ? "True\n" : "False\n") ;
		out << "\tHand Shape : " << (p.flags[p.IDX_rndHandShape] ? "True" : "False") << "\t  Hand Dimensions : " << (p.flags[p.IDX_rndHandDims] ? "True" : "False");
		out << "\tHand Color : " << (p.flags[p.IDX_rndHandClr] ? "True\n" : "False\n") ;
		return out;
	}

}
