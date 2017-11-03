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
 
 #include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{
		memory_mutex = new QMutex();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

void SpecificWorker::initialiazeMotors()
{
	///Create servos instances in a QMap indexed by name
	std::cout << "JointMotor::FaulHaber::FaulHaber - Motor Map creation with " << busParams.numMotors << " motors: " << std::endl;
	for (int i = 0; i < busParams.numMotors; i++)
	{
		QString name = QString::fromStdString(motorParamsList.operator[](i).name);
		motorsName[name] = new Servo( motorParamsList.operator[](i) );
		motorsId[motorParamsList.operator[](i).busId] = motorsName[name];
		
		//steps_range , max_degrees, steps_speed_range, max_speed_rads  ¿¿Deberían venir en los parámetros?
		motorsName[name]->setMotorRanges(motorParamsList.operator[](i).stepsRange, motorParamsList.operator[](i).maxDegrees, 0, 0.f);  
		qDebug()<<"\t" << name ; 
	}


	///Initialize motor params
	foreach( Servo *s, motorsName )
	{
		RoboCompJointMotor::MotorParams &params = s->params;
		std::cout << "JointMotor::FaulHaber::FaulHaber - Configuration data of motor " << params.name  << std::endl;
		qDebug() << "	" << "busId" << params.busId;
		faulhaber->Init_Node(params.busId);
		//rInfo("Init node");
		///Change Mode		
		faulhaber->enableControlPositionMode(params.busId);
		//rInfo("enable control");
		faulhaber->enableCommandMode(params.busId);
		//rInfo("enable command");
		if(params.busId == 3)
			faulhaber->setZero(3);
	}
	
	//send all motor to home position (end of track)
	rInfo("Sending home");
	foreach( Servo *s, motorsName )
	{
		RoboCompJointMotor::MotorParams &params = s->params;
		//rInfo("Sending home");
		faulhaber->goHome(params.busId);
		if(params.busId != 3)
			faulhaber->goHome(params.busId);
	}
	rInfo("waiting");
	
	sleep(15);
	rInfo("salgo");
	
	
	int positions[busParams.numMotors];
	int ids[busParams.numMotors];
	
	for(int i=0;i<busParams.numMotors;i++)
	  ids[i] = motorParamsList[i].busId;
	
//	int nohome,nohome_one;	
//	do{
//	  nohome = 0;
//	  for(int i=0;i<busParams.numMotors;i++){
//	    positions[i] = faulhaber->getPosition(ids[i]); 
// 	    if (positions[i]>3)
// 	      nohome_one = 1;
// 	    else
// 	      nohome_one = 0;
// 	    nohome = nohome + nohome_one;
// 	    usleep(Period*10);
// 	  }
// 	  qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3] << nohome;
// 	} while (nohome>0);
	  sleep(1);
	  for(int i=0;i<busParams.numMotors;i++)
	    positions[i] = faulhaber->getPosition(ids[i]); 

	  int cuenta=0;
	  while  (((positions[0]< -1) || (positions[1]< -1) || (positions[2]< -1) || (positions[3]< -30)) && (cuenta<30)){
	    for(int i=0;i<busParams.numMotors;i++)
	      positions[i] = faulhaber->getPosition(ids[i]); 
	    usleep(500000);
	    cuenta++;
	    qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];
	  }
	  qDebug() << "Cuenta:" << cuenta;
	  if (cuenta>=30)
	    qDebug() << "ERROR: no he llegado al home";
	  //read all motor positions
// 	  faulhaber->syncGetPosition(busParams.numMotors,&ids[0],&positions[0]);
// 	  rInfo("syncGetPosition");
// 	  qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];

  	  //qDebug() << faulhaber->getPosition(21) << faulhaber->getPosition(22) << faulhaber->getPosition(23)  << faulhaber->getPosition(20) ;



	  qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];
	  sleep(1);
	
	
	rInfo("In home");
	
	  
	//send all motor to zero position
	for(int i=0;i < busParams.numMotors;i++)
		positions[i] = motorParamsList[i].zeroPos;
	faulhaber->syncSetPosition(busParams.numMotors,&ids[0],&positions[0]);
	sleep(4);
	
	foreach( Servo *s, motorsName )
	{
		RoboCompJointMotor::MotorParams &params = s->params;
		faulhaber->setZero(params.busId);
		params.zeroPos = 0.f;
	}
	qDebug()<<"fin initialize";

		  faulhaber->syncGetPosition(busParams.numMotors,&ids[0],&positions[0]);
	  //rInfo("syncGetPosition");
	  qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];
	
}
	

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	busParams.device = params["Faulhaber.Device"].value;
	busParams.numMotors = QString::fromStdString(params["Faulhaber.NumMotors"].value).toInt();
	busParams.baudRate = QString::fromStdString(params["Faulhaber.BaudRate"].value).toInt();
	busParams.basicPeriod = QString::fromStdString(params["Faulhaber.BasicPeriod"].value).toInt();
	Period = busParams.basicPeriod *1000;
	
	for (int i=0; i<busParams.numMotors; i++)
	{
		std::string s= QString::number(i).toStdString();
		RoboCompJointMotor::MotorParams mpar;
		mpar.name = params["Faulhaber.Params_" + s +".name"].value;
		mpar.busId = QString::fromStdString(params["Faulhaber.Params_" + s +".busId"].value).toUShort();
		mpar.invertedSign = QString::fromStdString(params["Faulhaber.Params_" + s +".invertedSign"].value).contains("true");
		mpar.minPos = QString::fromStdString(params["Faulhaber.Params_" + s +".minPos"].value).toFloat();
		mpar.maxPos = QString::fromStdString(params["Faulhaber.Params_" + s +".maxPos"].value).toFloat();
		mpar.zeroPos = QString::fromStdString(params["Faulhaber.Params_" + s +".zeroPos"].value).toFloat();
		mpar.maxVelocity = QString::fromStdString(params["Faulhaber.Params_" + s +".maxVelocity"].value).toFloat();
		mpar.stepsRange = QString::fromStdString(params["Faulhaber.Params_" + s +".stepsToRadsRatio"].value).toFloat();
		mpar.maxDegrees = QString::fromStdString(params["Faulhaber.Params_" + s +".maxDegrees"].value).toFloat();
		motorParamsList.push_back(mpar);
		mParams[QString::fromStdString(mpar.name)] = mpar;
		name2id[QString::fromStdString(mpar.name)] = mpar.busId;
	}
	
	
	//Creacion de faulhaberApi
	faulhaber = new FaulHaberApi(QString::fromStdString(busParams.device),busParams.baudRate);

	initialiazeMotors();

	timer.start(Period);

	return true;
	
}

///
/// Servant methods
///

void SpecificWorker::compute( )
{

	int positions[busParams.numMotors];
	int ids[busParams.numMotors];
	for(int i=0;i<busParams.numMotors;i++)
		ids[i] = motorParamsList[i].busId;
	forever
	{
// 		rDebug("compute");
//  		mutex->lock();
//   		faulhaber->syncGetPosition(busParams.numMotors,&ids[0],&positions[0]);
//   		qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];
//  		sleep(1);
//   		mutex->unlock();
// 
// 		
// 		qDebug() << faulhaber->getPosition(9) << faulhaber->getPosition(10) << faulhaber->getPosition(11)  << faulhaber->getPosition(8) ;
// 		

		
		for(int i=0;i<busParams.numMotors;i++)
		{
			float pos = 0.f;
			if(ids[i] == 5 or ids[i] == 6 or ids[i] == 7)
				pos = faulhaber->convertir_Pasos_Radianes(positions[i],mParams[name2id.key(ids[i])].invertedSign );
			else if(ids[i] == 1 or ids[i] == 4 or ids[i] == 2 or ids[i] == 3)
				pos = motorsName[name2id.key(ids[i])]->steps2Rads(positions[i]);
			
			memory_mutex->lock();
				Servo::TMotorData &data = motorsId[ids[i]]->data;
				data.currentPosRads = pos;
				data.isMoving = fabs(data.antPosRads - data.currentPosRads) > 0.01;
				data.antPosRads = data.currentPosRads;
			memory_mutex->unlock();
		}
		
		
	/*		
		faulhaber->syncGetPosition(busParams.numMotors,&ids[0],&positions[0]);
		qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];
		
		for(int i=0;i < busParams.numMotors;i++){
			positions_d[i] = positions[i] + 1000;
			if(positions_d[i]>5000)
			  positions_d[i]=0;
		}
		//faulhaber->syncSetPosition(busParams.numMotors,&ids[0],&positions_d[0]);
		qDebug() << "positions_d" << positions_d[0] << positions_d[1] << positions_d[2] << positions_d[3];
		sleep(4);*/
		
		
		
		usleep(Period);
	}
}

void SpecificWorker::setPosition(const MotorGoalPosition& goalPosition)
{
	qDebug()<<"esto es setposicion in worker start";
	QString name = QString::fromStdString(goalPosition.name);
	if( mParams.contains( name ) )
	{
		Servo *servo = motorsName[name];
		int busId = name2id[name];
/*		if(busId == 6 or busId == 7 or busId == 8)
		{
			uFailed.what = std::string("Exception: FaulhaberComp::setPosition:: Movement not allowed, neck must move together") + goalPosition.name;
			throw uFailed;
			return;
		}*/
		
		float position = truncatePosition(name,goalPosition.position);
		int pInt = 0;
		
		if(busId == 5 or busId == 6)	//leftEye, rightEye
		{
			pInt = faulhaber->convertir_Radianes_Pasos(position,servo->params.invertedSign);
			if(pInt < -2500)
				pInt = -2500;
			if(pInt > 2500)
				pInt = 2500;
		}
		else if(busId == 7)	//tilt
		{
			pInt = faulhaber->convertir_Radianes_Pasos(position,servo->params.invertedSign);
			if(position < -1400)
				position = -1400;
			if(position > 1200)
				position = 1200;
		}
		else if(busId == 1 or busId == 4 or busId == 2 or busId == 3)
			pInt = servo->rads2Steps(position);

		qDebug()<<"setting position"<<pInt<<"of motor"<<name<<busId;
		mutex->lock();
			faulhaber->setPosition(busId,pInt);
		mutex->unlock();
	}
	else
	{
		uFailed.what = std::string("Exception: FaulhaberComp::setPosition:: Unknown motor name") + goalPosition.name;
		throw uFailed;
	}
	qDebug()<<"done:setting position of motor"<<name;
	qDebug()<<"esto es setposicion in worker end";
	
}


void SpecificWorker::setSyncPosition(const MotorGoalPositionList& goalPosList)
{
	qDebug()<<"esto es setsyncposicion in worker now";
	bool correct = true;
	int ids[goalPosList.size()];
	int positions[goalPosList.size()];
	float position = 0;
	qDebug()<<"esto es setsyncposicion comienzo";
	
	for(uint i=0;i<goalPosList.size();i++)
	{
		QString name = QString::fromStdString(goalPosList[i].name);
	
		qDebug()<<"esto es setsyncposicion in worker comenzando";
		if ( !motorsName.contains(name))
		{
			qDebug()<<"esto es setsyncposicion de todo";
			correct = false;
			break;
		}
		else
		{
			Servo *servo = motorsName[name];
			ids[i] = name2id[name];
	
			qDebug()<<"esto es setsyncposicion in nothing";
			position = truncatePosition(name,goalPosList[i].position);
			if(ids[i] == 5 or ids[i] == 6)
			{
				position = faulhaber->convertir_Radianes_Pasos(position,servo->params.invertedSign);
				if(position < -2000)
					position = -2000;
				if(position > 2000)
					position = 2000;
			qDebug()<<"esto es setsyncposicion in worker limit";
				
			}

			else if(ids[i] == 8)
				
				position = servo->rads2Steps(position);
			else if(ids[i] == 7)	//tilt
			{
				position =  faulhaber->convertir_Radianes_Pasos(position,servo->params.invertedSign);
				if(position < -800)
					position = -800;
				if(position > 800)
					position = 800;
			}
			
		
			else if(ids[i] == 1 or ids[i] == 4 or ids[i] == 2 or ids[i] == 3)
				position = servo->rads2Steps(position);
			
			positions[i] = position;
				qDebug()<<"esto es setsyncposicion in deadland";
		}
		qDebug()<<"esto es setsyncposicion in worker intermedio00";
	}
	if(correct)
	{
		////CHAPUZA para que los motores del cuello tengan que moverse obligatoriamente los 3 a la vez.
		int sum=0;
		for(uint i=0;i< goalPosList.size();i++)
		{
			if(ids[i] == 4 or ids[i] == 2 or ids[i] == 3)
				sum += ids[i];
			
			qDebug()<<"esto es setsyncposicion in worker intermedio1";
		}
		if(sum == 0 or sum == 9)
		{
			qDebug()<<"esto es setsyncposicion in worker intermedio2";
			mutex->lock();
			faulhaber->syncSetPosition(goalPosList.size(),&ids[0],&positions[0]);
			mutex->unlock();
		}
	}
	else
	{
		uFailed.what = std::string("Exception: FaulhaberComp::setSyncPosition:: Unknown motor name");
		throw uFailed;
	}
	qDebug()<<"done:setting sync position of motors";
	
	qDebug()<<"esto es setsyncposicion in worker dead";
}

MotorParams SpecificWorker::getMotorParams(const std::string& motor)
{
	RoboCompJointMotor::MotorParams mp;
	QString name = QString::fromStdString(motor);
	if ( mParams.contains( name ) )
	{
		mp = mParams[name];
	}
	else
	{
		uFailed.what = std::string("Exception: FaulhaberComp::getMotorParams:: Unknown motor name") + motor;
		throw uFailed;
	}
	return mp;
}
MotorState SpecificWorker::getMotorState(const std::string& motor)
{
	RoboCompJointMotor::MotorState state;
	QString name = QString::fromStdString(motor);
	if ( mParams.contains( name ) )
	{
		memory_mutex->lock();
		state.pos = motorsName[name]->data.currentPosRads;
		state.v = motorsName[name]->data.currentVelocityRads;
		state.p = motorsName[name]->data.currentPos;
		state.temperature = motorsName[name]->data.temperature;
		state.isMoving = motorsName[name]->data.isMoving;
		state.vel = motorsName[name]->data.currentVelocityRads;
		memory_mutex->unlock();
	}
	else
	{
		uFailed.what = std::string("Exception: FaulhaberComp::getMotorState:: Unknown motor name") + motor;
		throw uFailed;
	}
	return state;
}
void SpecificWorker::getAllMotorState(MotorStateMap& mstateMap)
{
	RoboCompJointMotor::MotorState state;
	memory_mutex->lock();
	foreach( Servo *s, motorsName)
	{
		state.pos = s->data.currentPosRads;
//		state.v = s->data.currentVelocityRads;
		state.p = s->data.currentPos;
		state.isMoving = s->data.isMoving;
//		state.temperature = s->data.temperature;
		mstateMap[s->params.name] = state ;
	}
	memory_mutex->unlock();
}
MotorParamsList SpecificWorker::getAllMotorParams()
{
	return motorParamsList;
}
BusParams SpecificWorker::getBusParams()
{
	return busParams;
}

MotorStateMap SpecificWorker::getMotorStateMap(const MotorList& motorList){
	RoboCompJointMotor::MotorStateMap stateMap;
	RoboCompJointMotor::MotorState state;
	foreach(std::string motor, motorList)
	{
		QString name = QString::fromStdString(motor);
		if ( mParams.contains( name ) )
		{
			memory_mutex->lock();
			state.pos = motorsName[name]->data.currentPosRads;
//			state.v = motorsName[name]->data.currentVelocityRads;
			state.p = motorsName[name]->data.currentPos;
//			state.temperature = motorsName[name]->data.temperature;
			state.isMoving = motorsName[name]->data.isMoving;
//			state.vel = motorsName[name]->data.currentVelocityRads;
			memory_mutex->unlock();
		}
		else
		{
			uFailed.what = std::string("Exception: FaulhaberComp::getMotorParams:: Unknown motor name") + motor;
			throw uFailed;
		}
	}
	return stateMap;
}


void SpecificWorker::setZeroPos(const std::string& name)
{
	RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}

void SpecificWorker::setSyncZeroPos()
{
	RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}

void SpecificWorker::setVelocity(const MotorGoalVelocity& goalVelocity)
{
	RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}
void SpecificWorker::setSyncVelocity(const MotorGoalVelocityList& listGoals)
{
	RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}

//Comprueba la posicion pasada con los rangos máximos de movimiento y trunca el valor en caso necesario.
float SpecificWorker::truncatePosition(QString name,float position)
{
	float p = position;
	if(position > mParams[name].maxPos )
		p = mParams[name].maxPos;
	else if( position < mParams[name].minPos )
		p = mParams[name].minPos;
	return p;
}



/*

void SpecificWorker::stopAllMotors(){
}
void SpecificWorker::stopMotor(const string& motor){
}
void SpecificWorker::releaseBrakeAllMotors(){
}
void SpecificWorker::releaseBrakeMotor(const string& motor){
}
void SpecificWorker::enableBrakeAllMotors(){
}
void SpecificWorker::enableBrakeMotor(const string& motor){
}
*/
