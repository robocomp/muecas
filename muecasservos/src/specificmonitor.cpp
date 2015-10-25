
/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "specificmonitor.h"
/**
* \brief Default constructor
*/
SpecificMonitor::SpecificMonitor(GenericWorker *_worker,Ice::CommunicatorPtr _communicator):GenericMonitor(_worker, _communicator)
{
	ready = false;
}
/**
* \brief Default destructor
*/
SpecificMonitor::~SpecificMonitor()
{

}

void SpecificMonitor::run()
{
	initialize();
	ready = true;
	forever
	{
		//rDebug("specific monitor run");
		this->sleep(period);
	}
}

/**
 * \brief Reads components parameters and checks set integrity before signaling the Worker thread to start running
 * There can be four (4) types of parameteres:
 *		(1) Ice parameters
 *		(2) Nexus (configuration) parameters	
 *		(3) Local component parameters read at start
 *		(4) Local parameters read from other running component
 *
 */
void SpecificMonitor::initialize()
{
	rInfo("Starting monitor ...");
	initialTime=QTime::currentTime();
	RoboCompCommonBehavior::ParameterList params;
	//readPConfParams(params);
	readConfig(params);
	if(!sendParamsToWorker(params))
	{
		rError("Error reading config parameters. Exiting");
		killYourSelf();
	}
	state = RoboCompCommonBehavior::Running;
}
bool SpecificMonitor::sendParamsToWorker(RoboCompCommonBehavior::ParameterList params)
{
	if(checkParams(params))
	{
		//Set params to worker
		worker->setParams(params);
		return true;
	}
	else
	{
		rError("Incorrect parameters");
		return false;			//Change when implemented
		//return true;
	}

}
///Local Component parameters read at start
///Reading parameters from config file or passed in command line, with Ice machinery
///We need to supply a list of accepted values to each call
void SpecificMonitor::readConfig(RoboCompCommonBehavior::ParameterList &params )
{
	//Read params from config file
	//Example
	    //RoboCompCommonBehavior::Parameter aux;
	    //aux.editable = true;
	    //configGetString( "DRobot.Device", aux.value,"/dev/ttyUSB0");
	    //params["DRobot.Device"] = aux;
	    RoboCompCommonBehavior::Parameter aux;
	configGetString( "", "muecasJoint.NumMotors", aux.value, "0" );	
	int num_motors=0;
	num_motors = QString::fromStdString(aux.value).toInt();
	if(num_motors <= 0) 
	  qFatal("Monitor::initialize - Zero motors found. Exiting..." );

	params["muecasJoint.NumMotors"] = aux;
	configGetString( "", "muecasJoint.Device", aux.value, "/dev/ttyUSB0" );	
	params["muecasJoint.Device"] = aux;

	configGetString( "", "muecasJoint.BaudRate", aux.value, "" );
	params["muecasJoint.BaudRate"] = aux;
	
	configGetString( "", "muecasJoint.BasicPeriod", aux.value, "100" );
	params["muecasJoint.BasicPeriod"] = aux;
	
	std::string paramsStr;
	for (int i=0; i<num_motors; i++)
	{
		std::string s= QString::number(i).toStdString();
		configGetString( "", "muecasJoint.Params_" + s, aux.value , "");
		params["muecasJoint.Params_" + s] = aux;
		QStringList list = QString::fromStdString(aux.value).split(",");
		if (list.size() != 7) qFatal("Error reading motor. Only %d parameters for motor %d.", list.size(), i);
		
		aux.value=list[0].toStdString();
		params["muecasJoint.Params_" + s +".name"]= aux;
		aux.value=list[1].toStdString();
		params["muecasJoint.Params_" + s +".busId"]= aux;
		aux.value=list[2].toStdString();
		params["muecasJoint.Params_" + s +".invertedSign"]= aux;
		aux.value=list[3].toStdString();
		params["muecasJoint.Params_" + s +".minPos"]= aux;
		aux.value=list[4].toStdString();
		params["muecasJoint.Params_" + s +".maxPos"]= aux;
		aux.value=list[5].toStdString();
		params["muecasJoint.Params_" + s +".zeroPos"]= aux;
		aux.value=list[6].toStdString();	
		params["muecasJoint.Params_" + s +".maxVelocity"]= aux;
	}
}

//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList params)
{
	
	bool correct = true;
		if(QString::fromStdString(params["muecasJoint.NumMotors"].value).toFloat() < 5)
		correct = false;
	//Check parameters
	//Example
// 	    if(l["DRobot.Handler"].value != "Robex" and l["DRobot.Handler"].value != "Gazebo" and l["DRobot.Handler"].value != "Player")
// 		    correct = false;
	
	//copy parameters
// 	if(correct)
// 		config_params = l;
	return correct;
}

