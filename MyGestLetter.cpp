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


#include "apps/gestureIK/MyGestLetter.h"
#include "apps/gestureIK/IKSolver.h"

namespace gestureIKApp {

	MyGestLetter::MyGestLetter(const std::string& _ltrName, unsigned int _ltrNum):IKSolve(nullptr), 
		curIDX(_ltrNum), numFileSymbols(0), numTotSymbols(0), srcSymbols(0), exampleSymbols(0),
		curSymbol(nullptr), ltrName(_ltrName), fileName(""), uni(nullptr), flags(numFlags)
	{
		std::stringstream ss;
		ss << lettersPath << "ltr_" << ltrName << "/ltr_" << ltrName << ".xml";
		//name of xml file holding instances of this letter
		fileName = ss.str();
	}
	MyGestLetter::~MyGestLetter() {	}

	//build file name used by screen capture
	std::string MyGestLetter::getSymbolFileName() {
		std::stringstream ss;
		ss << IKSolve->params->dateFNameOffset << "_" << curSymbol->name << "_" << DataType2strAbbrev[IKSolve->params->dataType];
		return ss.str();
	}

	std::string MyGestLetter::buildSymbolName(int count) {
		std::stringstream ss;
		ss << ltrName << "_" << buildStrFromInt(count);
		return ss.str();
	}// buildSymbolName
	//symbol file describing the trajectories that make up this symbol - called from xml parser
	//per symbol list of trajectory file names
	void MyGestLetter::buildFileSymbolTrajs(std::vector< std::vector< std::string > >& trajFileNames) {
		exampleSymbols.clear();
		srcSymbols.clear();
		//std::stringstream ss;
		for (int i = 0; i < numFileSymbols; ++i) {
			const std::string name = buildSymbolName(i);

			//std::allocate_shared<MyGestSymbol>(Eigen::aligned_allocator <MyGestSymbol>(), name, i);
			//symbols.push_back(std::make_shared<MyGestSymbol>(name, i));
			exampleSymbols.push_back(std::allocate_shared<MyGestSymbol>(Eigen::aligned_allocator <MyGestSymbol>(), IKSolve, name, i));
			//set shared ptr ref to self
			exampleSymbols[i]->setSolver(IKSolve);
			exampleSymbols[i]->buildTrajsFromFile(trajFileNames[i], exampleSymbols[i]);
			//srcSymbols are only source trajectory symbols
			srcSymbols.push_back(exampleSymbols[i]);
		}
		numTotSymbols = numFileSymbols;
		//build distribution of potential symbols to draw from to find random symbol idx to ik to 
		buildUniDist();
	}//readLetterFile

	//generate random symbols from the file-based symbols already read in so that there are _totNumDesSymb present - note this is total desired, so # to add is _totNumDesSymb - numFileSymbols
	//TODO use this to only hold debug/example symbols intended to illustrate distributions - all actual symbols are generated on the fly
	void MyGestLetter::buildRandExSymbolTrajs(int _totNumDesSymb) {
		if (_totNumDesSymb <= numTotSymbols) {
			std::cout << "Requested " << _totNumDesSymb << " random and file based symbols, but already have " << numTotSymbols << " made, so doing nothing." << std::endl;
			return;
		}
		std::shared_ptr<gestureIKApp::MyGestSymbol> tmpPtr;
		for (int i = numTotSymbols; i < _totNumDesSymb; ++i) {		//adding symbols
			tmpPtr = buildRandSymbol(i);
			exampleSymbols.push_back(tmpPtr);
		}
		numTotSymbols = _totNumDesSymb;
		//rebuild new distribution of potential symbols to draw from to find random symbol idx to ik to 
		//buildUniDist();
	}//buildRandExSymbolTrajs

	std::shared_ptr<gestureIKApp::MyGestSymbol> MyGestLetter::buildRandSymbol(int idx) {
		std::shared_ptr<gestureIKApp::MyGestSymbol> tmpPtr(nullptr);
		//std::stringstream ss;
		int srcSymbolIDX;
		bool isDone = false;
		bool isFast = (2 * (*uni)(mtrn_gen)) > numFileSymbols;		//mtrn_gen is uniform from 0 to numTotSymbols-1, so this should be ~50% TODO replace with xml boolean, if used (? is fast traj considered?)
		//uncomment to equally represent each source symbol
		//srcSymbolIDX = idx % numFileSymbols;						//just move ahead 1 symbol - this will have each letter equally represented (except those that have too many trajectories or otherwise fail in mySymbol::buildRandomSymbol
		do {
			//comment out if we want to equally represent each source symbol
			srcSymbolIDX = (*uni)(mtrn_gen);						//random idx of source symbol
			const std::string name = buildSymbolName(idx);
			//tmpPtr = std::make_shared<MyGestSymbol>(name, srcSymbolIDX);
			tmpPtr = std::allocate_shared<MyGestSymbol>(Eigen::aligned_allocator <MyGestSymbol>(), IKSolve, name, srcSymbolIDX);
			tmpPtr->setSolver(IKSolve);
			//isDone returns false if there are issues with the random symbol build - i.e. if the source symbol has too many trajectories or other problems
			isDone = tmpPtr->buildRandomSymbol(srcSymbols[srcSymbolIDX], tmpPtr, isFast);
			//uncomment to equally represent each source symbol
			//srcSymbolIDX = (srcSymbolIDX + 1) % numFileSymbols;
		} while (!isDone);
		return tmpPtr;
	}//buildRandSymbol

	//sets specific index in symbol list for letter to draw - used to let myWindow control which symbols to draw (for train and test data)
	void MyGestLetter::buildSymbolAndSolveIK(int symIdx, bool solveIK, bool disp) {
		//generating random symbol here - symIdx is only specified to determine if all symbols of this letter have been drawn - need to maintain a count instead
		curSymbol = buildRandSymbol(symIdx);
		curSymbol->initSymbolIK();
		//if solveIK solve all IK frames here, initially, if we wish to enable motion blur, and save skel states in vector, so we can display buffer accumulated results 
		if (solveIK) {//only solve for all frames if using motion blur
			bool doneCurLetter = false;
			while (!doneCurLetter) {
				doneCurLetter = curSymbol->solve();
			}
			curSymbol->curFrame = 0;		//reset to 0 so can be used again for setIKSkelPose
		}
		if (disp) { std::cout << "MyGestLetter::buildSymbolAndSolveIK : Specified " << symIdx << "th symbol to use for ltr idx : " << curIDX << " : " << ltrName << " with : "<< curSymbol->numTrajFrames <<" frames "<< std::endl; }
	}//buildSymbolAndSolveIK

	//draw all trajectory components of all symbols of current letter being used for IK
	void MyGestLetter::drawSymbolTrajDist(dart::renderer::RenderInterface* mRI) {
		for (int i = 0; i < exampleSymbols.size(); ++i) { exampleSymbols[i]->drawTrajs(mRI);		}
	}
	
	//draw all trajectory components of current symbol being used for IK
	void MyGestLetter::drawLetter(dart::renderer::RenderInterface* mRI) {
		curSymbol->drawTrajs(mRI);
	}

	std::ostream& operator<<(std::ostream& out, MyGestLetter& ltr) {
		out << "Letter :" << ltr.ltrName << "\tfile name : " << ltr.fileName << std::endl;
		out << "\t# symbol files present : " << ltr.numFileSymbols << "\t total # symbols of this letter present : " << ltr.numTotSymbols << std::endl;
		return out;
	}

}  // namespace gestureIKApp
