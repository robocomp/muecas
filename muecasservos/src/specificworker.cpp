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
		qDebug() << __FUNCTION__ << "Constructor";
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute()
{
	//qDebug()<<"compute";
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	busParams.device = params["muecasJoint.Device"].value;
	busParams.numMotors = QString::fromStdString(params["muecasJoint.NumMotors"].value).toInt();
	busParams.baudRate = QString::fromStdString(params["muecasJoint.BaudRate"].value).toInt();
	busParams.basicPeriod = QString::fromStdString(params["muecasJoint.BasicPeriod"].value).toInt();
			
	for (int i=0; i<busParams.numMotors; i++)
	{
		std::string s= QString::number(i).toStdString();
		RoboCompJointMotor::MotorParams mpar;
		mpar.name = params["muecasJoint.Params_" + s +".name"].value;
		mpar.busId = QString::fromStdString(params["muecasJoint.Params_" + s +".busId"].value).toUShort();
		mpar.invertedSign = QString::fromStdString(params["muecasJoint.Params_" + s +".invertedSign"].value).contains("true");
		mpar.minPos = QString::fromStdString(params["muecasJoint.Params_" + s +".minPos"].value).toFloat();
		mpar.maxPos = QString::fromStdString(params["muecasJoint.Params_" + s +".maxPos"].value).toFloat();
		mpar.zeroPos = QString::fromStdString(params["muecasJoint.Params_" + s +".zeroPos"].value).toFloat();
		mpar.maxVelocity = QString::fromStdString(params["muecasJoint.Params_" + s +".maxVelocity"].value).toFloat();
		motorParamsList.push_back(mpar);
		
		QString name = QString::fromStdString(mpar.name);
		mParams[name] = mpar;
		name2id[name] = mpar.busId;
		
		//servos
		motorsName[name] = new Servo( mpar );
		motorsId[mpar.busId] = motorsName[name];
		//TODO: Revisar los parametros de cada motor
		//setMotorRanges(float step_range, float degrees_range, float speed_step_range, float max_speed_rads);
		motorsName[name]->setMotorRanges(200, 360.f,0,0);
	}
	//Creacion de acceso dispositivo
	device.setName (QString::fromStdString(busParams.device) );
	device.setBaudRate ( QSerialPort::BAUD9600);//cambiar por valor leido de los parametros

	if ( device.open ( QIODevice::ReadWrite ) == false )
	{
		rError("Failed to open: "+QString::fromStdString(busParams.device));
		qFatal("Error opening device");
	}
	rInfo("Device "+QString::fromStdString(busParams.device)+" is open");
	
	timer.start(Period);
	return true;
}

void SpecificWorker::setPosition(const MotorGoalPosition& goal)
{
	
	QString name = QString::fromStdString(goal.name);
	if( mParams.contains( name ) )
	{
		Servo *servo = motorsName[name];
		int busId = name2id[name];
		
		
		float position = truncatePosition(name,goal.position);
		int pInt = 0;
		
	// pInt = servo->rads2Steps(position);
			pInt = servo->convertir_Radianes_Pasos(position,servo->params.invertedSign);
			
		qDebug()<<"setting position"<<pInt<<"of motor"<<name<<busId;
	
		Servo::TMotorData &data = motorsId[busId]->data;
		data.currentPosRads = position;
	}
	else
	{
		uFailed.what = std::string("Exception: muecasJointComp::setPosition:: Unknown motor name") + goal.name;
		throw uFailed;
	}
	qDebug()<<"done:setting position of motor"<<name;
	
}

void SpecificWorker::setVelocity(const MotorGoalVelocity& goal)
{
	RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}

void SpecificWorker::setZeroPos(const std::string& name)
{
	RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}

void SpecificWorker::setSyncPosition(const MotorGoalPositionList& listGoals){
	bool correct = true;
	int ids[ listGoals.size()];
	int positions[ listGoals.size()];
	float position = 0;
	
	for(uint i=0;i< listGoals.size();i++)
	{
		QString name = QString::fromStdString( listGoals[i].name);
		if ( !motorsName.contains(name))
		{
			correct = false;
			break;
		}
		else
		{
			Servo *servo = motorsName[name];
			ids[i] = name2id[name];
			position = truncatePosition(name, listGoals[i].position);
			positions[i] = servo->rads2Steps(position);
		}
	}
	if(correct)
	{
		for(uint i=0;i< listGoals.size();i++)
			setPosition( listGoals[i]);
	}
	else
	{
		uFailed.what = std::string("Exception: muecasJointComp::setSyncPosition:: Unknown motor name");
		throw uFailed;
	}
	qDebug()<<"done:setting sync position of motors";
}
void SpecificWorker::setSyncVelocity(const MotorGoalVelocityList& listGoals){
		RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}
void SpecificWorker::setSyncZeroPos(){
		RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}
MotorParams SpecificWorker::getMotorParams(const std::string& motor){
	RoboCompJointMotor::MotorParams mp;
	QString name = QString::fromStdString(motor);
	if ( mParams.contains( name ) )
	{
		mp = mParams[name];
	}
	else
	{
		uFailed.what = std::string("Exception: muecasJointComp::getMotorParams:: Unknown motor name") + motor;
		throw uFailed;
	}
	return mp;
	//return 0;
}
MotorState SpecificWorker::getMotorState(const std::string& motor){
	RoboCompJointMotor::MotorState state;
	QString name = QString::fromStdString(motor);
	if ( mParams.contains( name ) )
	{
		state.pos = motorsName[name]->data.currentPosRads;
		state.v = motorsName[name]->data.currentVelocityRads;
		state.p = motorsName[name]->data.currentPos;
		state.temperature = motorsName[name]->data.temperature;
		state.isMoving = motorsName[name]->data.isMoving;
		state.vel = motorsName[name]->data.currentVelocityRads;
	}
	else
	{
		uFailed.what = std::string("Exception: muecasJointComp::getMotorState:: Unknown motor name") + motor;
		throw uFailed;
	}
	return state;
	//return 0;
}
MotorStateMap SpecificWorker::getMotorStateMap(const MotorList& mList){
		RoboCompJointMotor::MotorStateMap stateMap;
	RoboCompJointMotor::MotorState state;
	foreach(std::string motor, mList)
	{
		QString name = QString::fromStdString(motor);
		if ( mParams.contains( name ) )
		{
			state.pos = motorsName[name]->data.currentPosRads;
			state.v = motorsName[name]->data.currentVelocityRads;
			state.p = motorsName[name]->data.currentPos;
			state.temperature = motorsName[name]->data.temperature;
			state.isMoving = motorsName[name]->data.isMoving;
			state.vel = motorsName[name]->data.currentVelocityRads;
		}
		else
		{
			uFailed.what = std::string("Exception: muecasJointComp::getMotorParams:: Unknown motor name") + motor;
			throw uFailed;
		}
	}
	return stateMap;
	
	//return 0;
}
void SpecificWorker::getAllMotorState(MotorStateMap& mstateMap){
	RoboCompJointMotor::MotorState state;
	foreach( Servo *s, motorsName)
	{
		state.pos = s->data.currentPosRads;
		state.v = s->data.currentVelocityRads;
		state.p = s->data.currentPos;
		state.isMoving = s->data.isMoving;
		state.temperature = s->data.temperature;
		mstateMap[s->params.name] = state ;
	}
}
MotorParamsList SpecificWorker::getAllMotorParams(){
	return motorParamsList;
	//return 0;
}
BusParams SpecificWorker::getBusParams(){
		return busParams;
	//return 0;
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
/**
* \brief Send command to the device
* @param cmd Command to send
* @return true if command was sent successfully, else return false
*/
bool SpecificWorker::sendCommand(QString cmd, char *buf, int totalread)
{
	if(device.isOpen())
	{
		// Send command

		device.write(cmd.toLatin1().data(),cmd.size());
		qDebug() << "reading ";
		int numRecibido = device.read(buf, totalread);
		if ( numRecibido != totalread )
		{
			printf("[10] Device: Ack error: <comand(%d) ", cmd.size());
			return false;
		}
		else
			return true;
	}
	else
	{
		rError("Fatal error, motion deviced closed");
		return false;
	}
}


bool SpecificWorker::setPositionservo(int busId, int  pasos)
{   
	//COMPROBAR QUE EL RANGO PEDIDO ES VALIDO. SI NO LO ES DEVOLVER FALSE
	
	// TAMBIEN CAMBIAR PARA QUE ACEPTE RADIANES
	
	  //pasos are converted to quarters of microsenconds
	  //o pasos = 0 qms
	  //255 pasos = 2000*4=8000 qms
	   qDebug() <<  "pasos " << pasos; 
	  //para probar algo
 	  int qms = pasos * (8000/255);
 qDebug() <<  "qms " << qms; 
	  QString cmd;

	  cmd.resize(4);
	  char buf[1];   
	   
      cmd[0] = (0x84);
	  cmd[1] = busId;
      cmd[2] = (qms & 0x7F);
      cmd[3] = ((qms >> 7) & 0x7F);
      bool res = sendCommand(cmd,buf,0);
	  return res;

	 
	   
//  	QString cmd;
//  	cmd.resize(9);
//  	cmd.fill('0');
//  	char buf[5], id[2],p[3];
//  	
//  	sprintf(id,"%d",busId);
//  	cmd[0] = 'A';
//  	cmd[1] = id[0];
//  	cmd[2] = ':';
//  	
// 	sprintf(p,"%03d",pasos);
//  	cmd[3] = p[0];
//  	cmd[4] = p[1];
//  	cmd[5] = p[2];
//  	cmd[6] = id[0];
//  	cmd[7] = '*';
//  	cmd[8] ='\0';
//  	
//  	sendCommand(cmd,buf,3);
}

// bool SpecificWorker::setMultiplePositions(int busId, int  &pasos)
// {   
// }

bool SpecificWorker::getPosition(int busId, float  &rads)
{   
	//natively returns 2 bytes in quarters of microsenconds
	//we have to transform them to rads taking into account the ZEROPOSITON to set it to 0 rads.
	
	QString cmd;

	cmd.resize(2);
	char buf[2];   
	   
    cmd[0] = (0x90);
	cmd[1] = busId;
    bool res = sendCommand(cmd,buf,2);
	uint16_t *r = (uint16_t *)buf;
	rads = (*r) * (2.f*M_PI / 8000);
	//now we apply the offset ZEROPOS
	return res;
}

bool SpecificWorker::setSpeed(int busId, float  radsg)
{   
	//SPEED always positive
	//CHECK SPEED LIMITS
	//Speed given in quarters of microsenconds per ten millisconds (qms/tms)
	//Conversion from rads sg to qms/tm
	//Assuming 8000 microsenconds = 2PI rads, 1 rad will take 8000/2PI
	//rads to qms = radsg * (8000/2PI)
	//qms/sg to qms/tms = 1/100 so radsg*(8000/2PI)/100 is what we need
	
	int qms = (int)(radsg * (8000.f/2.f*M_PI)/100.f);
	QString cmd;

	cmd.resize(4);
	char buf[1];   
	   
    cmd[0] = (0x87);
	cmd[1] = busId;
    cmd[2] = (qms & 0x7F);
    cmd[3] = ((qms >> 7) & 0x7F);
    bool res = sendCommand(cmd,buf,0);
	return res;
}

// bool SpecificWorker::setAcceleration(int busId, int  &pasos)
// {   
// }

bool SpecificWorker::getMovingState(int busId, bool &state)
{   
	QString cmd;

	cmd.resize(3);
	char buf[1];   
	   
    cmd[0] = (0xAA);
	cmd[1] = busId;
    cmd[2] = (0x13);
    bool res = sendCommand(cmd,buf,1);
	state = (buf[0] == true);
	return res;
}

// bool SpecificWorker::getErrors(int busId, int  &pasos)
// {   
// }
// 
// bool SpecificWorker::goHome(int busId, int  &pasos)
// {   
// }


/**
* \brief Update all motors position
*/
void SpecificWorker::update()
{
	///Servos hasn't got encoder so we can't read their real positions
}
