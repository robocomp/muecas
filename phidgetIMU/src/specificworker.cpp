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
#include "specificworker.h"


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	init();
	timer.start(Period);

	return true;
}

void SpecificWorker::init()
{
	int result;
	const char *err;
	
	spatial = 0;
	CPhidgetSpatial_create(&spatial);
	CPhidget_open((CPhidgetHandle)spatial, -1);
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 3000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		qFatal("fary IMU");
	}
	display_properties((CPhidgetHandle)spatial);
	CPhidgetSpatial_setDataRate(spatial, 16);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)spatial, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)spatial, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);
	printf("Reading.....\n");
	
}

void SpecificWorker::compute()
{
	//qDebug() << Ori[0] << Ori[1] << Ori[2];
}

//callback that will run if the Spatial is attached to the computer
int  SpecificWorker::AttachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d attached!", serialNo);
	return 0;
}

//callback that will run if the Spatial is detached from the computer
int  SpecificWorker::CCONV DetachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d detached! \n", serialNo);
	return 0;
}

//callback that will run if the Spatial generates an error
int  SpecificWorker::CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown)
{
	printf("Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
int CCONV SpecificWorker::SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
	Acc[0] = -data[count-1]->acceleration[0];
	Acc[1] = -data[count-1]->acceleration[2];
	Acc[2] =  data[count-1]->acceleration[1];
	Gyr[0] = -data[count-1]->angularRate[0]*M_PI/180.;
	Gyr[1] = -data[count-1]->angularRate[2]*M_PI/180.;
	Gyr[2] =  data[count-1]->angularRate[1]*M_PI/180.;
	Mag[0] = -data[count-1]->magneticField[0];
	Mag[1] = -data[count-1]->magneticField[2];
	Mag[2] =  data[count-1]->magneticField[1];

	
	Ori[0] =  atan2(Acc[2], -Acc[1]);
	Ori[1] = 0;
	Ori[2] = -atan2(Acc[0], -Acc[1]);
	for (int o=0; o<3; o++)
	{
		while(Ori[o] > +M_PI) Ori[o]-=2.*M_PI;
		while(Ori[o] < -M_PI) Ori[o]+=2.*M_PI;
	}
// 	printf("Acc: (%f, %f, %f)\n", W->Acc[0], W->Acc[1], W->Acc[2]);
// 	printf("Ori: (%f, %f, %f)\n", W->Ori[0], W->Ori[1], W->Ori[2]);
	return 0;
}


//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
// of the attached Spatial.
int SpecificWorker::display_properties(CPhidgetHandle phid)
{
	int serialNo, version;
	const char* ptr;
	int numAccelAxes, numGyroAxes, numCompassAxes, dataRateMax, dataRateMin, dataRate;

	CPhidget_getDeviceType(phid, &ptr);
	CPhidget_getSerialNumber(phid, &serialNo);
	CPhidget_getDeviceVersion(phid, &version);
	CPhidgetSpatial_getAccelerationAxisCount((CPhidgetSpatialHandle)phid, &numAccelAxes);
	CPhidgetSpatial_getGyroAxisCount((CPhidgetSpatialHandle)phid, &numGyroAxes);
	CPhidgetSpatial_getCompassAxisCount((CPhidgetSpatialHandle)phid, &numCompassAxes);
	CPhidgetSpatial_getDataRateMax((CPhidgetSpatialHandle)phid, &dataRateMax);
	CPhidgetSpatial_getDataRateMin((CPhidgetSpatialHandle)phid, &dataRateMin);
	CPhidgetSpatial_getDataRate((CPhidgetSpatialHandle)phid, &dataRate);
	

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("Number of Accel Axes: %i\n", numAccelAxes);
	printf("Number of Gyro Axes: %i\n", numGyroAxes);
	printf("Number of Compass Axes: %i\n", numCompassAxes);
	printf("datarate> Max: %d  Min: %d Current %d\n", dataRateMax, dataRateMin, dataRate
	);

	return 0;
}



////////////////////////////////////////////////////7
/////////
/////////////////////////////////////////////////////

void SpecificWorker::resetImu()
{
	mutex->lock();
	for (int i=0; i<3; ++i)
	{
		Mag[i] = 0.f;
		Acc[i] = 0.f;
		Gyr[i] = 0.f;
	}
	mutex->unlock();
}

Orientation SpecificWorker::getOrientation()
{
	Orientation d;
	mutex->lock();
	d.Pitch = Ori[0];
	d.Yaw   = Ori[1];
	d.Roll  = Ori[2];
	mutex->unlock();
	return d;
}

DataImu SpecificWorker::getDataImu()
{
	QMutexLocker ml(mutex);
	DataImu d;
	d.acc = getAcceleration();
	d.gyro = getAngularVel();
	d.mag = getMagneticFields();
	d.rot = getOrientation();
	return d;
}

Gyroscope SpecificWorker::getAngularVel()
{
	Gyroscope d;
	mutex->lock();
	d.XGyr = Gyr[0];
	d.YGyr = Gyr[1];
	d.ZGyr = Gyr[2];
	mutex->unlock();
	return d;	
}

Magnetic SpecificWorker::getMagneticFields()
{
	Magnetic d;
	mutex->lock();
	d.XMag= Mag[0];
	d.YMag= Mag[1];
	d.ZMag= Mag[2];
	mutex->unlock();
	return d;	
}

Acceleration SpecificWorker::getAcceleration()
{
	Acceleration d;
	mutex->lock();
	d.XAcc = Acc[0];
	d.YAcc = Acc[1];
	d.ZAcc = Acc[2];
	mutex->unlock();
	return d;
}
