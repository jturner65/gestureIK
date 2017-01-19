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

#ifndef APPS_GESTURE_GLBLS_H_
#define APPS_GESTURE_GLBLS_H_

#include <string>
#include <sstream>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <random>
#include "dart/dart.h"

namespace gestureIKApp {
	class trackedMarker;
	class MyGestTraj;


	/** Helper types for STL containers with fixed size vectorizable Eigen memory allocators - from MRPT. */
	template <class TYPE1, class TYPE2 = TYPE1>
	struct aligned_containers {
		typedef std::pair<TYPE1, TYPE2> pair_t;
		typedef std::vector<TYPE1, Eigen::aligned_allocator<TYPE1> > vector_t;
		typedef std::vector<std::vector<TYPE1, Eigen::aligned_allocator<TYPE1> > > vec_vec_t;
		typedef std::deque<TYPE1, Eigen::aligned_allocator<TYPE1> > deque_t;
		typedef std::list<TYPE1, Eigen::aligned_allocator<TYPE1> > list_t;
		typedef std::map<TYPE1, TYPE2, std::less<TYPE1>, Eigen::aligned_allocator<std::pair<const TYPE1, TYPE2> > > map_t;
		typedef std::multimap<TYPE1, TYPE2, std::less<TYPE1>, Eigen::aligned_allocator<std::pair<const TYPE1, TYPE2> > > multimap_t;
	};

	typedef gestureIKApp::aligned_containers<Eigen::Vector3d>::vector_t eignVecTyp;
	typedef gestureIKApp::aligned_containers<Eigen::Vector3d>::vec_vec_t eignVecVecTyp;
	//map keyed by name of all markers that are being tracked by IK
	typedef std::map<std::string, std::shared_ptr<trackedMarker>> trkMrkMap;

	//map keyed by name of trajectories, holding shared ptrs of trajectories
	typedef std::map<std::string, std::shared_ptr<MyGestTraj>> trajMap;

	//data being generated - variation to explore the LSTM's capabilities and quirks - the LSTM's window is 16 frames long
	//const_vel uses the same per-frame displacement for all trajectories of all letters - duration can be any value
	//FIXED_16 uses training data that is restricted to 16 frames, and multiple of 8 (but no less than 16) frame testing data
	//mult_8 uses multiple of 8 (but no less than 16) frame training and testing data
	enum DataType { CONST_VEL, FIXED_16, MULT_8};
	static const char* DataType2str[] = { "Constant Velocity", "16-frame", "Mult-8 Train, Mult-8 Test" };
	static const char* DataType2strAbbrev[] = { "CONST_VEL", "FIXED_16", "MULT_8" };

	//////////////////////////
	//for randomization stuff
	/////////////////////////////

	static std::random_device randDevice;			// only used once to initialise (seed) engine
	static std::mt19937 mtrn_gen(randDevice());    // random-number engine used (Mersenne-Twister in this case)	

	//end randomization stuff

	//global variables and functions
	//static const float origZoom = .65f;
	static const Eigen::Vector3d origMTrans(27.36, -370.1, -200.00);
	//const Eigen::Quaterniond origTrackBallQ(0.78389, 0, -0.6209, 0);
	static const Eigen::Quaterniond origTrackBallQ(0.5*sqrt(2), 0, -0.5*sqrt(2), 0);

	//radial locations of verts for sample shapes
	static std::array<double, 3> const triCrnrConsts{ DART_PI_HALF, DART_PI_HALF + (2.0 * DART_PI / 3.0), DART_PI_HALF + (4.0 * DART_PI / 3.0) };
	static std::array<double, 4> const sqCrnrConsts{ .25*DART_PI,.75*DART_PI, 1.25*DART_PI,1.75*DART_PI };
	static std::array<double, 5> const strCrnrConsts{ DART_PI_HALF, DART_PI_HALF + (2.0 * 1.256637061435917), DART_PI_HALF + (4.0 * 1.256637061435917), DART_PI_HALF + (1.256637061435917), DART_PI_HALF + (3.0 * 1.256637061435917) };

	////base value of # of trajectories to capture for training and testing data
	//static const int dataCap = 100;
	////threshold between testing and training data
	//static const double testTrainThresh = .5;

	//location of generated(temporary) csv files
	static std::string const appFilePath = DART_ROOT_PATH"apps/gestureIK/";
	//location of generated(temporary) csv files
	static std::string const csvFilePath = DART_ROOT_PATH"apps/gestureIK/csvs/";
	//location of frame captures
	static std::string const framesFilePath = DART_ROOT_PATH"apps/gestureIK/frames/";
	//location of BPL letters
	static std::string const lettersPath = DART_ROOT_PATH"apps/gestureIK/sourceLetters/";

	//names of generated trajectories - TODO replace with reading in from XML file
	static std::array<std::string, 4> const trajNames{ "circle", "triangle" , "square", "star" };

	//precalced sqrt 2
	static const double sqrt2 = sqrt(2);

	//names of markers that we are actually solving for in IK
	static std::array<std::string, 7> const trkedMrkrNames{ "ptrFinger_r", "ptrElbow_r" , "right_scapula", "left_scapula" , "head_right","head_left", "abdomen" };
	//names of markers we are holding fixed (solving for original positions only)
	static std::array<std::string, 5> const fixedMrkrNames{ "right_scapula", "left_scapula" , "head_right", "head_left", "abdomen" };

	//colors to use for trajectory rendering
	static std::array<Eigen::Vector3d, 7> const trajColors{ Eigen::Vector3d(0.1,0.1,0.1), Eigen::Vector3d(0.8,0.1,0.1), Eigen::Vector3d(0.1,0.8,0.1),Eigen::Vector3d(0.1,0.1,0.8),Eigen::Vector3d(0.1,0.8,0.8), Eigen::Vector3d(0.8,0.8,0.1),Eigen::Vector3d(0.8,0.1,0.8) };

	//int, float , double, eigen vector3d to string utilities
	static inline std::string buildStrFromInt(int val, int numDigs=5) {
		std::stringstream ss;
		if (val == 0) {
			for (int i = numDigs; i > 1; --i) { ss << "0"; }
		}
		else {
			float comp = pow(10, numDigs - 1);
			for (int i = numDigs; i > 0; --i) {
				if (val < comp) { ss << "0"; comp /= 10; }
				else { break; }
			}
		}
		ss << val;
		//std::cout << "val : " << val << "\tstr:-" << ss.str() << "-"<<std::endl;				
		return ss.str();// label = ss.str();
	}//buildLabel
	static inline std::string buildStrFromFloat(float val, const char* fmt = "%.4f") {
		char buf[256];
		std::stringstream ss;
		sprintf(buf, fmt, val);
		ss << buf;
		return ss.str();// label = ss.str();
	}//buildLabel
	static inline std::string buildStrFromDbl(double val, const char* fmt = "%.4f") {
		char buf[256];
		std::stringstream ss;
		sprintf(buf, fmt, val);
		ss << buf;
		return ss.str();// label = ss.str();
	}//buildLabel
	static inline std::string buildStrFrmEigV3d(const Eigen::Ref<const Eigen::Vector3d>& vec, int numDigs = 4) {
		std::stringstream ss;
		ss.str("");
		ss << "%." << numDigs << "f\0";
		const std::string tmp = ss.str();
		const char * fmt = tmp.c_str();
		ss.str("");
		ss << buildStrFromDbl(vec(0), fmt) << "," << buildStrFromDbl(vec(1), fmt) << "," << buildStrFromDbl(vec(2), fmt); 
		return ss.str();
	}


	///////////////////////////
	//		interpolation utilities and constants
	///////////////////////////

	//% of distance to use for "sweep-back" point in 4 point neville
	static const double nevPct = .2;

	static const int 	//point processing flags
		_subdivide = 0,
		_tuck = 1;

	//interpolation routines
	//static inline Eigen::Vector3d interpVec(const Eigen::Ref<const Eigen::Vector3d>& A, const Eigen::Ref<const Eigen::Vector3d>& B, double t) { return ((1 - t) * A) + (t * B); }
	static inline Eigen::Vector3d interpVec(const Eigen::Ref<const Eigen::Vector3d>& A, const Eigen::Ref<const Eigen::Vector3d>& B, double t) { return (A + (t * (B - A))); }
	//tuck/untuck based on sign of t
	static inline Eigen::Vector3d tuck(const Eigen::Ref<const Eigen::Vector3d>& A, const Eigen::Ref<const Eigen::Vector3d>& B, const Eigen::Ref<const Eigen::Vector3d>& C, double t) {return interpVec(A, interpVec(B, C, .5), t);}

	static Eigen::Vector3d GenQInterp(const Eigen::Ref<const Eigen::Vector3d>& A, double a, const Eigen::Ref<const Eigen::Vector3d>& B, double b,
		const Eigen::Ref<const Eigen::Vector3d>& C, double c, double t) {
		Eigen::Vector3d res = interpVec(interpVec(A, B, (t - a) / (b - a)), interpVec(B, C, (t - b) / (c - b)), (t - a) / (c - a));
		return res;
	}

	static Eigen::Vector3d GenQInterp(const Eigen::Ref<const Eigen::Vector3d>& A, double a, const Eigen::Ref<const Eigen::Vector3d>& B, double b,
		const Eigen::Ref<const Eigen::Vector3d>& C, double c, const Eigen::Ref<const Eigen::Vector3d>& D, double d, double t) {
		Eigen::Vector3d p = GenQInterp(A, a, B, b, C, c, t),
			q = GenQInterp(B, b, C, c, D, d, t);
		auto res = interpVec(p, q, (t - a) / (d - a));
		return res;
	}

	static Eigen::Vector3d QInterp(const Eigen::Ref<const Eigen::Vector3d>& A, const Eigen::Ref<const Eigen::Vector3d>& B, const Eigen::Ref<const Eigen::Vector3d>& C, const Eigen::Ref<const Eigen::Vector3d>& D, double t) {
		return  GenQInterp(A, 0, B, 0.25f, C, 0.75f, D, 1, t);
	}


}//namespace gestureIKApp 
#endif  // APPS_GESTURE_GLBLS_H_
