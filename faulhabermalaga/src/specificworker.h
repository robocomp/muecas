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

#include <genericworker.h>
#include <canbus/faulhaberApi.h>
#include "servo.h"
/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
	void setPosition(const MotorGoalPosition& goal);
	void setVelocity(const MotorGoalVelocity& goal);
	void setZeroPos(const string& name);
	void setSyncPosition(const MotorGoalPositionList& listGoals);
	void setSyncVelocity(const MotorGoalVelocityList& listGoals);
	void setSyncZeroPos();
	
	MotorParams getMotorParams(const string& motor);
	MotorState getMotorState(const string& motor);
	MotorStateMap getMotorStateMap(const MotorList& mList);
	void getAllMotorState(MotorStateMap& mstateMap);
	MotorParamsList getAllMotorParams();
	BusParams getBusParams();
	
	/*
	void stopAllMotors();
	void stopMotor(const string& motor);
	void releaseBrakeAllMotors();
	void releaseBrakeMotor(const string& motor);
	void enableBrakeAllMotors();
	void enableBrakeMotor(const string& motor); */
private:
	void initialiazeMotors();
	float truncatePosition(QString name,float position);
	
private:
		//Parameters
	RoboCompJointMotor::BusParams busParams;
	RoboCompJointMotor::MotorParamsList motorParamsList;
	QHash<QString, RoboCompJointMotor::MotorParams> mParams;
	//Exceptions
	RoboCompJointMotor::HardwareFailedException hFailed;
	RoboCompJointMotor::UnknownMotorException uFailed;
	//Motors
	FaulHaberApi *faulhaber;
	QHash<QString, Servo*> motorsName;
	QHash<int,Servo*> motorsId;
	QHash<QString,int> name2id;
	QMutex * memory_mutex;
	
public slots:
	void compute();
};

#endif
