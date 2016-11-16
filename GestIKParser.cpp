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

#include "apps/gestureIK/GestIKParser.h"

using namespace std;
using namespace Eigen;
using namespace dart::utils;
namespace gestureIKApp {
	//read info from filename, set params value controlling IK sim and data collection variables
	void GestIKParser::readGestIKXML(const std::string& _filename, std::shared_ptr<GestIKParams>& params) {
		//_filename includes path
		cout << "Read params from file name : " << _filename << "\n";
		// Load xml and create Document
		tinyxml2::XMLDocument _configFile;
		try { dart::utils::openXMLFile(_configFile, _filename.c_str()); }
		catch (std::exception const& e) { cout << "readGestIKXML: LoadFile  " << _filename << " Fails: " << e.what() << ".\n"; }

		std::string pname;
		tinyxml2::XMLElement* paramElem = NULL;
		paramElem = _configFile.FirstChildElement("GestIKParams");
		if (paramElem == NULL) { cout << "readGestIKXML:Params Config file " << _filename << " does not contain <GestIKParams> as an element.\n";				return; }
		else {//read in configuration params
			ElementEnumerator parameters(paramElem, "parameter");
			while (parameters.next()) {
				tinyxml2::XMLElement* pElem = parameters.get();
				assert(pElem != NULL);
				pname = getValueString(pElem, "name");
				std::string v = "val", s = pElem->FirstChildElement(v.c_str())->GetText();
				params->setParamValFromXMLStr(pname, s);				
			}
			params->setCurrentValsAsDefault();			//initializes important values and sets defaults
		}
	}//readGestIKXML

	//take file name from _ltr and read in all information pertaining to representative symbols and component trajectories
	void GestIKParser::readGestLetterXML(std::shared_ptr<MyGestLetter> _ltr) {
		// Load xml and create Document
		tinyxml2::XMLDocument _configFile;
		try { dart::utils::openXMLFile(_configFile, _ltr->fileName.c_str()); }
		catch (std::exception const& e) {					cout << "readGestLetterXML : LoadFile  " << _ltr->fileName << " Fails: " << e.what() << ".\n"; }

		// letterData element
		tinyxml2::XMLElement* configElement = NULL;
		configElement = _configFile.FirstChildElement("letterData");
		if (configElement == NULL) {						cout << "Letter Config file " << _ltr->fileName  << " does not contain Required <letterData> as an element.\n";			return; }
		// skeleton data
		tinyxml2::XMLElement* ltrElement = NULL;
		ltrElement = configElement->FirstChildElement("letter");
		if (ltrElement == NULL) {							cout << "Unknown Letter - Letter Element missing in file " << _ltr->fileName << ".\n";			return; }
		std::string ltr = ltrElement->GetText();
		cout<<"Letter being read "<<ltr<<" in file : "<< _ltr->fileName << ".\n";

		tinyxml2::XMLElement* numSymbolElem = NULL;
		numSymbolElem = configElement->FirstChildElement("symbolExampleCounts");
		if (numSymbolElem == NULL) {						cout << "<symbolExampleCounts> missing in file " << _ltr->fileName << ".\n"; return; }
		_ltr->numFileSymbols = getValueInt(configElement, "symbolExampleCounts");

		tinyxml2::XMLElement* symbolsElem = NULL;
		symbolsElem = configElement->FirstChildElement("symbols");
		if (symbolsElem == NULL) {						cout << "<symbols> Element missing in file " << _ltr->fileName << ".\n"; return; }

		tinyxml2::XMLElement* firstSymbolElem = NULL;
		firstSymbolElem = symbolsElem->FirstChildElement("symbol");
		if (firstSymbolElem == NULL) { cout << "<symbol> Element missing in <symbols> tag in file " << _ltr->fileName << ".\n"; return; }
		readGestSymbolsXML(_ltr, firstSymbolElem);

	}//readGestLetterXML

	void GestIKParser::readGestSymbolsXML(std::shared_ptr<MyGestLetter> _ltr, tinyxml2::XMLElement* _xmlElement) {
		vector< vector< std::string > > trajFileNames;			//all trajectory file names for all symbols of this ltr
		tinyxml2::XMLElement* symElem = _xmlElement;
		for (int i = 0; i < _ltr->numFileSymbols; ++i) {
			tinyxml2::XMLElement* numTrajsElem = NULL;
			numTrajsElem = symElem->FirstChildElement("trajCounts");
			if (numTrajsElem == NULL) { cout << "<trajCounts> missing in file " << _ltr->fileName << " symbol def # "<<i<<".\n"; return; }

			int numTrajs = getValueInt(symElem, "trajCounts");
			vector< std::string > tmpSymbolTrajs;
			tmpSymbolTrajs.resize(numTrajs);
			numTrajsElem = numTrajsElem->NextSiblingElement("trajectory");
			for (int j = 0; j < numTrajs; ++j) {
				tmpSymbolTrajs[j] = numTrajsElem->GetText();
				//cout << "\tRead " << i << "th symbol's " << j << "th trajectory : " << tmpSymbolTrajs[j] << "\n";
				numTrajsElem = numTrajsElem->NextSiblingElement("trajectory");
			}
			trajFileNames.push_back(std::move(tmpSymbolTrajs));
			symElem = symElem->NextSiblingElement("symbol");
		}//for each symbol
		_ltr->buildFileSymbolTrajs(trajFileNames);
	}
}
