/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <phidget21.h>

static float Acc[3];
static float Gyr[3];
static float Mag[3];
static float Ori[3];


class SpecificWorker : public GenericWorker
{
Q_OBJECT
	public:
		SpecificWorker(MapPrx& mprx);	
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);

		Magnetic getMagneticFields();
		void resetImu();
		Orientation getOrientation();
		DataImu getDataImu();
		Gyroscope getAngularVel();
		Acceleration getAcceleration();
		
	public slots:
		void compute(); 	

	private:
		void init();
		CPhidgetSpatialHandle spatial;
		static int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count);
		static int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr);
		static int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr);
		static int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown);
		int display_properties(CPhidgetHandle phid);
};
#endif

