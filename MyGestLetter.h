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
#ifndef APPS_GESTURE_MYGESTLETTER_H_
#define APPS_GESTURE_MYGESTLETTER_H_

#include <Eigen/StdVector>
#include <memory>
#include <Eigen/Dense>
#include "apps/gestureIK/GestGlbls.h"
#include "apps/gestureIK/MyGestSymbol.h"

#include "dart/dart.h"
namespace dart {
	namespace renderer {
		class RenderInterface;
	}  // namespace renderer
} // namespace dart

/*
this class represents a class of letter.  it will consist of a collection of symbols - manifestations of the letter
it is responsible for determining which symbol should be used when its letter is called for

omniglot is collected amazon turk data
symbols in BPL omniglot data have x,y data and time in milliseconds, individually recorded for each component trajectory
alphabet is #22, latin, in dataset

each symbol will be recorded in xml file with # of component trajectories, names of each trajectory file.
each trajectory file will be csv file of location of primary trajectory x,y points and velocities
*/
namespace gestureIKApp {
	class IKSolver;
	class MyGestSymbol;
	//collection of 1 or more trajectories making up a symbol
	class MyGestLetter {
	public:
		MyGestLetter(const std::string& _ltrName);
		virtual ~MyGestLetter();

		//data from matlab will be 3 cols per point : (x,y, timing data), where timing is how long between consecutive points in milliseconds.
		//data needs to be projected only plane with normal pointing at shoulder, and scaled to fit desired circle.
		//average location needs to be managed at symbol level.

		//solve IK on the current Letter - returns true if finished
		bool solve();
		//build a uniform distribution engine to be able to select randomly from all the symbols that represent this letter (read from file and desired from random generation)
		void buildUniDist() {
			uni = std::make_shared< std::uniform_int_distribution<int> >(0, (numTotSymbols - 1));
		}
		//build the symbols that are built from file descriptions
		void buildFileSymbolTrajs(std::vector< std::vector< std::string > >& trajFileNames);
		//generate random symbols from the file-based symbols already read in so that there are _totNumDesSymb present (# randomized * # from files, probably)
		void buildRandomSymbolTrajs(int _totNumDesSymb, int partitionForTest);
		//set solver for this trajectory
		void setSolver(std::shared_ptr<gestureIKApp::IKSolver> _slv);
		//set symbol to draw to be random entry in symbols list
		void setRandSymbolIdx(int idx, bool disp);
		//turn on/off testing of letter quality
		void setTestLtrQual(bool val) {	setSymbolFlags(testLtrQualIDX, val);	}
		//set symbol and letter idx
		void setSymbolIdx(int idx, int symIdx, bool disp);
		//set flags of all subordinate symbols
		void setSymbolFlags(int idx, bool val);
		//draw all letters to test range of values from randomization
		void drawAllSymbols(dart::renderer::RenderInterface* mRI);
		//draw all components of trajectory of current symbol
		void drawLetter(dart::renderer::RenderInterface* mRI);

		std::string getCurSymbolName() { return symbols[curSymbolIDX]->name; }
		int getCurSymbolFrame() { return symbols[curSymbolIDX]->curFrame; }
		//limit training data to 16 frame-long complete clips
		inline void limitTrainTo16(bool val) {
			for (int i = 0; i < symbols.size(); ++i) { symbols[i]->limitTrainTo16(val); }
		}

		friend std::ostream& operator<<(std::ostream& out, MyGestLetter& ltr);


	public :	//variables
		std::shared_ptr<gestureIKApp::IKSolver> IKSolve;						//ref to ik solver
		//idx in letters array (0 == a, 1 == b etc)
		unsigned int curIDX;
		//current symbol index for debug drawing all letters
		unsigned int curDrawSymIDX;
		//current symbol idx used
		unsigned int curSymbolIDX;
		//# of examples of this letter from files
		unsigned int numFileSymbols;
		//# of total symbols of this letter (# randomized * # from files, probably)
		unsigned int numTotSymbols;
		//instances of this letter
		std::vector<std::shared_ptr<gestureIKApp::MyGestSymbol> > symbols;		
		
		std::string ltrName;													//letter name
		std::string fileName;													//xml filename source for the symbols and trajectories of this letter
		std::shared_ptr<std::uniform_int_distribution<int> > uni; // guaranteed unbiased

		std::vector<bool> flags;					//state flags of trajectory
		static const unsigned int debugIDX = 0,		//debug mode
			testLtrQualIDX = 1;						//test all symbols to make sure no abberrant trajectories

		static const unsigned int numFlags = 2;


	};
} // namespace gestureIKApp

#endif // #ifndef APPS_GESTURE_MYGESTLETTER_H_
