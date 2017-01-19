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

#include <iostream>
#include <vector>

#include "dart/dart.h"

#include "apps/gestureIK/MyWindow.h"

int main(int argc, char* argv[]) {
	// create and initialize the world
	//dart::simulation::WorldPtr myWorld = dart::utils::SkelParser::readWorld(DART_DATA_PATH"skel/fullbody1.skel");
	dart::simulation::WorldPtr myWorld = dart::utils::SkelParser::readWorld(DART_ROOT_PATH"apps/gestureIK/fullbody1.skel");
	assert(myWorld != nullptr);

	Eigen::Vector3d gravity(0.0, -9.81, 0.0);
	myWorld->setGravity(gravity);

	dart::dynamics::SkeletonPtr biped = myWorld->getSkeleton("fullbody1");
	//configure initial pose
	biped->getDof("j_thigh_left_z")->setPosition(  0.15);
	biped->getDof("j_shin_left")->setPosition(    -0.40);
	biped->getDof("j_heel_left_1")->setPosition(   0.25);
	biped->getDof("j_thigh_right_z")->setPosition( 0.15);
	biped->getDof("j_shin_right")->setPosition(   -0.40);
	biped->getDof("j_heel_right_1")->setPosition(  0.25);
	biped->getDof("j_abdomen_2")->setPosition(     0.00);

	// create a window and link it to the world
	MyWindow window(std::make_shared<IKSolver>(biped));
	window.setWorld(myWorld);

	std::cout << "\nKey Commands:"<< std::endl;
	std::cout << "\t` : reset camera loc" << std::endl;
	std::cout << "\tc : screen capture 1 entire cycle of trajectory" << std::endl;
	std::cout << "\td : debug mode on/off" << std::endl;
	std::cout << "\tv : markers on/off" << std::endl;
	std::cout << "\ts : turn current symbol trajectory display on/off." << std::endl;
	std::cout << "\tb : load letters and build letter list." << std::endl;

	std::cout << "Left click: rotate camera" << std::endl;
	std::cout << "Right click: pan camera" << std::endl;
	std::cout << "Shift + Left click: zoom camera" << std::endl;

	std::cout << "Sample Trajectories : " << std::endl;
	std::cout << "\t1,2,3,4 : follow trajectory for sample objects : circle, triangle, square, star." << std::endl;
	std::cout << "\tr : restart current sample object with randomization." << std::endl;
	std::cout << "\tw : capture sample symbols Triangle/Square/Star to disk." << std::endl;
	std::cout << "\ty : reset sample object endpoints to non-random locations." << std::endl;

	std::cout << std::endl;
	std::cout << "Letter Trajectories : " << std::endl;
	std::cout << "\ta : capture letter symbols a-z to disk." << std::endl;
	std::cout << "\tf : test letter symbols a-z following screen cap protocol without saving." << std::endl;
	std::cout << "\tp : pause between rendering IK Frames for letters." << std::endl;
	std::cout << "\tShift + <letter> : follow random human trajectory for <letter> " << std::endl;

	glutInit(&argc, argv);
	//window.initWindow(320, 240, "Gesture IK");
	//window.initWindow(800, 800, "Gesture IK");
	window.initCustWindow("Gesture IK");
	glutMainLoop();

	return 0;
}

