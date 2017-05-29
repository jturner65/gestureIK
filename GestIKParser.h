/*
* Copyright (c) 2015, Georgia Tech Research Corporation
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

#ifndef APPS_GESTURE_GESITIKPARSER_H_
#define APPS_GESTURE_GESITIKPARSER_H_

#include <vector>
#include <string>
#include <memory>
#include <iostream>

#include <Eigen/Dense>
#include <tinyxml2.h>
#include "dart/utils/Parser.h"
#include "apps/gestureIK/GestIKParams.h"
#include "apps/gestureIK/MyGestLetter.h"

namespace gestureIKApp {
	//any necessary hyperparams needed to be loaded from XML file
	/**
	Letter source trajectory files have specific, nested format. 

	main file : ltr_<char>.xml 
	
	this file holds <char>'s letter name, # of examples, and other optional config data, 
	as well as the counts and filenames (with absolute path embedded, unfortunately) of each example's 
	component trajectory files.

	trajectory file : ltr_<char>_cpy_<exNum>_trj_<trjNum>.csv
	
	holds a trajectory of an example of a letter. 
	<trjNum> is the trajectory in the sequence of trajectories comprising this example
	<exNum> is a particular example of the letter; <char> is the letter.

	ex : 
	ltr_a.xml
	ltr_a_cpy_0_trj_0.csv
	ltr_a_cpy_0_trj_1.csv
	ltr_a_cpy_1_trj_0.csv
	...

	means : a's main descriptor xml, a's example #0 component traj's 0 and 1, and a's example #1 component traj 1
	*/

	class GestIKParser {
	public:
		//read configuration for general IK solve and trajectory building
		static void readGestIKXML(const std::string& _filename, std::shared_ptr<GestIKParams> params);
		//read specific letter file and populate MyGestLetter object
		static void readGestLetterXML(std::shared_ptr<MyGestLetter> _ltr);
		//read symbol trajectory files data
		static std::vector< std::vector< std::string > >  readGestSymbolsXML(std::shared_ptr<MyGestLetter> _ltr, tinyxml2::XMLElement* _xmlElement);
		//read in marker mappings for skeleton
		static void readMarkerLocsXML(const std::string& _filename, std::shared_ptr<GestIKParams> params);
	};

}//namespace gestureIKApp 
#endif  // APPS_GESTURE_GestIKParams_H_
