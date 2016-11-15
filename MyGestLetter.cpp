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

	MyGestLetter::MyGestLetter(std::string _ltrName):IKSolve(nullptr), curSymbolIDX(0), numSymbols(0), symbols(0), ltrName(_ltrName), fileName(""), uni(nullptr), flags(numFlags){
		stringstream ss;
		ss.str("");
		ss << lettersPath << "ltr_" << ltrName<<"/ltr_"<<ltrName<<".xml";
		//name of xml file holding instances of this letter
		fileName = ss.str();
	}
	
	MyGestLetter::~MyGestLetter() {	}
	
	//symbol file describing the trajectories that make up this symbol - called from xml parser
	//per symbol list of trajectory file names
	void MyGestLetter::buildSymbolTrajs(vector< vector< std::string > >& trajFileNames){
		symbols.clear();
		stringstream ss;
		for (int i = 0; i < numSymbols; ++i) {
			ss.str("");
			ss << ltrName << "_" << i;
			symbols.push_back(std::make_shared<MyGestSymbol>(ss.str()));
			//set shared ptr ref to self
			symbols[i]->_self = symbols[i];
			symbols[i]->setSolver(IKSolve);
			symbols[i]->readTrajFiles(trajFileNames[i], symbols[i]);
		}
		//build distribution of potential symbols to draw from to find random symbol idx to ik to 
		uni = make_shared< uniform_int_distribution<int> > (0, (numSymbols-1));
	}//readLetterFile

	//solve IK on current letter - get current symbol, cycle through all trajectories until drawn
	bool MyGestLetter::solve() {
		bool finished = symbols[curSymbolIDX]->solve();
		if (finished) {
			cout << "Finished drawing letter : " << symbols[curSymbolIDX]->name << "\n";
		}
		return finished;
	}

	//sets random index in symbol list for letter to draw
	void MyGestLetter::setRandSymbolIdx(int idx) {
		curIDX = idx;
		curSymbolIDX = (*uni)(mtrn_gen);
		symbols[curSymbolIDX]->initSymbolIK();
		cout << "Curr Rand symbol to use for ltr idx : " << curIDX << " : " << ltrName << " : " << curSymbolIDX << "\n";
	}

	//sets specific index in symbol list for letter to draw - used to let myWindow control which symbols to draw (for train and test data)
	void MyGestLetter::setSymbolIdx(int idx, int symIdx) {
		curIDX = idx;
		curSymbolIDX = symIdx;
		symbols[curSymbolIDX]->initSymbolIK();
		cout << "Specified symbol to use for ltr idx : " << curIDX << " : " << ltrName << " : " << curSymbolIDX << "\n";
	}

	//set reference to IK solver - set in all trajectories
	void MyGestLetter::setSolver(std::shared_ptr<gestureIKApp::IKSolver> _slv) {
		IKSolve = _slv;
		for (int i = 0; i<symbols.size(); ++i) { symbols[i]->setSolver(IKSolve); }
	}

	//set flag values for all symbols of this letter
	void MyGestLetter::setSymbolFlags(int idx, bool val) {
		for (int i = 0; i < symbols.size(); ++i) { symbols[i]->setFlags(idx, val); }
		
	}

	//draw all trajectory components of current symbol being used for IK
	void MyGestLetter::drawLetter(dart::renderer::RenderInterface* mRI) {
		symbols[curSymbolIDX]->drawTrajs(mRI);
	}

	std::ostream& operator<<(std::ostream& out, MyGestLetter& ltr) {
		out << "Letter :" << ltr.ltrName << "\tfile name : " << ltr.fileName << "\n";
		out << "\t# symbols presents : " << ltr.numSymbols << "\n";

		return out;
	}

}  // namespace gestureIKApp
