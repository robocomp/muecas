/****************************************************************************
#                   Robolab component model prototype                       #
#                             Copyright (C) 2009                            #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/

#include <QtCore>
#include <QtGui>

#include <Ice/Ice.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>

#include "config.h"

#include "worker.h"
#include "imuI.h"

using namespace std;
using namespace RoboCompIMU;

int result;
const char *err;
CPhidgetSpatialHandle spatial;
Worker *W;

class IMUComp : public RoboComp::Application
{
private:
	void initialize();
public:
	virtual int run(int, char*[]);
	string device;
};

void IMUComp::initialize()
{
}


//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d attached!", serialNo);
	return 0;
}

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d detached! \n", serialNo);
	return 0;
}

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown)
{
	printf("Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
	W->Acc[0] = -data[count-1]->acceleration[0];
	W->Acc[1] = -data[count-1]->acceleration[2];
	W->Acc[2] =  data[count-1]->acceleration[1];
	W->Gyr[0] = -data[count-1]->angularRate[0]*M_PI/180.;
	W->Gyr[1] = -data[count-1]->angularRate[2]*M_PI/180.;
	W->Gyr[2] =  data[count-1]->angularRate[1]*M_PI/180.;
	W->Mag[0] = -data[count-1]->magneticField[0];
	W->Mag[1] = -data[count-1]->magneticField[2];
	W->Mag[2] =  data[count-1]->magneticField[1];

	
	W->Ori[0] =  atan2(W->Acc[2], -W->Acc[1]);
	W->Ori[1] = 0;
	W->Ori[2] = -atan2(W->Acc[0], -W->Acc[1]);
	for (int o=0; o<3; o++)
	{
		while(W->Ori[o] > +M_PI) W->Ori[o]-=2.*M_PI;
		while(W->Ori[o] < -M_PI) W->Ori[o]+=2.*M_PI;
	}
// 	printf("Acc: (%f, %f, %f)\n", W->Acc[0], W->Acc[1], W->Acc[2]);
// 	printf("Ori: (%f, %f, %f)\n", W->Ori[0], W->Ori[1], W->Ori[2]);
	return 0;
}


//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
// of the attached Spatial.
int display_properties(CPhidgetHandle phid)
{
	int serialNo, version;
	const char* ptr;
	int numAccelAxes, numGyroAxes, numCompassAxes, dataRateMax, dataRateMin;

	CPhidget_getDeviceType(phid, &ptr);
	CPhidget_getSerialNumber(phid, &serialNo);
	CPhidget_getDeviceVersion(phid, &version);
	CPhidgetSpatial_getAccelerationAxisCount((CPhidgetSpatialHandle)phid, &numAccelAxes);
	CPhidgetSpatial_getGyroAxisCount((CPhidgetSpatialHandle)phid, &numGyroAxes);
	CPhidgetSpatial_getCompassAxisCount((CPhidgetSpatialHandle)phid, &numCompassAxes);
	CPhidgetSpatial_getDataRateMax((CPhidgetSpatialHandle)phid, &dataRateMax);
	CPhidgetSpatial_getDataRateMin((CPhidgetSpatialHandle)phid, &dataRateMin);

	

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("Number of Accel Axes: %i\n", numAccelAxes);
	printf("Number of Gyro Axes: %i\n", numGyroAxes);
	printf("Number of Compass Axes: %i\n", numCompassAxes);
	printf("datarate> Max: %d  Min: %d\n", dataRateMax, dataRateMin);

	return 0;
}


int IMUComp::run(int argc, char* argv[])
{
	QCoreApplication a(argc, argv);
	int status=EXIT_SUCCESS;

	string proxy;
	initialize();

	Worker *worker = new Worker();
	W = worker;

	CPhidgetSpatialHandle spatial = 0;
	CPhidgetSpatial_create(&spatial);
	CPhidget_open((CPhidgetHandle)spatial, -1);
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 3000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}
	display_properties((CPhidgetHandle)spatial);
	CPhidgetSpatial_setDataRate(spatial, 16);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)spatial, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)spatial, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);
	printf("Reading.....\n");

	try
	{
		// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapter = communicator()->createObjectAdapter("IMUComp");
		IMUI *imuI = new IMUI(worker);
		adapter->add(imuI, communicator()->stringToIdentity("imu"));
		adapter->activate();
		cout << SERVER_FULL_NAME " started" << endl;
		a.exec();
		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;
		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;
	}
	return status;
}

int main(int argc, char* argv[])
{
	bool hasConfig = false;
	string arg;
	IMUComp app;

	for (int i = 1; i < argc; ++i)
	{
		arg = argv[i];
		if ( arg.find ( "--Ice.Config=", 0 ) != string::npos )
			hasConfig = true;
	}

	if ( hasConfig )
		return app.main( argc, argv );
	else
		return app.main(argc, argv, "config");
}
