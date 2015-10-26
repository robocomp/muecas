/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

 ////muecashead////////
#define MAX_X 2700
#define MIN_X -4900

#define MAX_Y 2700
#define MIN_Y -4900

#define MAX_Z 2700
#define MIN_Z -4900

#define MAX_TILT 1200
#define MIN_TILT -1400

#include <rcdraw/rcdraw.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>

#include <genericworker.h>
#include <flycapture/FlyCapture2.h>

/**
       \brief
       @author authorname
*/
using namespace std;
using namespace cv;
class SpecificWorker : public GenericWorker
{
Q_OBJECT
	public:
		SpecificWorker(MapPrx& mprx);	
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		
		//muecashead
		RoboCompJointMotor::MotorParamsList getAllMotorParams();
		void  getAllMotorState(RoboCompJointMotor::MotorStateMap& mstateMap);
		void  setPosition(const RoboCompJointMotor::MotorGoalPosition& goal);
				
		//motor emotion
		void move    (float pos, const string& Name);
		void moveneckimitation(float newpitch, float newroll, float newyaw);
		
		float normalize(float X, float A, float B, float C, float D);	
	
	public slots:
		void compute(); 
		void habla();	
		void moveNeck(double);
		void moveFaceElems(double l);
		
	private:
		
		QMutex *mutex_memory;
		bool headmoving;
		RoboCompCommonHead::THeadParams headParams;
		RoboCompJointMotor::MotorParamsList mplFaulhaber,mplMuecas, mplAll;
		RoboCompJointMotor::MotorList mlFaulhaber,mlMuecas,mlAll;
		RoboCompJointMotor::MotorStateMap stateFaulhaber,stateMuecas,stateAll;
		RoboCompIMU::DataImu dataImu;

		std::string neckMotorName;
		std::string tiltMotorName;
		std::string leftPanMotorName;
		std::string rightPanMotorName;
		std::string mouthMotorName;
		std::string eyeBLeftUPMotorName;
		std::string eyeBLeftYAWMotorName;
		std::string eyeBRightUPMotorName;
		std::string eyeBRightYAWMotorName;
		float joy_x_antiguo;
		float joy_y_antiguo;
		float joy_z_antiguo;
						
	/*	RoboCompCamera::TCamParams paramsCamera;
		RoboCompCamera::imgType imgCam;  
		RoboCompDifferentialRobot::TBaseState bState;
		RoboCompCommonHead::THeadState hState;
	*/		
	QImage *qImageRGB;
	FlyCapture2::Camera **ppCameras;
	
};

#endif