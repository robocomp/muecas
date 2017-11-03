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
double Prior_FE[5] = {-1.,-1.,-1.,-1.,-1.};

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{
  qDebug() << __FUNCTION__ << "init worker";
	mutex = new QMutex(QMutex::Recursive);
	mutex_memory = new QMutex(QMutex::Recursive);
	
	////READ FROM CONFIG!!!
	neckMotorName ="neck";
	
	tiltMotorName ="tilt";
	leftPanMotorName = "leftPan";
	rightPanMotorName = "rightPan";
	
	mouthMotorName = "mouth";
	eyeBLeftUPMotorName = "eyeBLeftUP";
	eyeBLeftYAWMotorName= "eyeBLeftYAW";
	eyeBRightUPMotorName= "eyeBRightUP";
	eyeBRightYAWMotorName = "eyeBRightYAW";
	
	/////////////////camera ///////////////////////////////////
	
// 	try
// 	{
// 		paramsCamera = camera_proxy->getCamParams();
// 	}
// 	catch ( const Ice::Exception& ex )
// 	{
// 		qDebug ( " Could not obtain camera parameters... \n" );
// 		std::cout << ex<<std::endl;
// 		exit(-1);
// 		
// 	}
// 		imgSize = paramsCamera.width * paramsCamera.height ;
// 	
// 	qDebug() << "Focal: " << paramsCamera.focal;
// 	qDebug() << "Width " <<  paramsCamera.width;
// 	qDebug() << "Height " << paramsCamera.height;
// 	
	
	////////////////////faulhaber y motor///////////////////////////////
	
	qDebug() << __FUNCTION__ << "Read params from faulhaber and muecasJoint";
	try
	{
		mplFaulhaber = jointmotor2_proxy->getAllMotorParams();
		mplAll = mplFaulhaber;
		foreach( RoboCompJointMotor::MotorParams param, mplFaulhaber)
		{
			qDebug() << QString::fromStdString(param.name);
		}
		qDebug() << __FUNCTION__ << "Connection to Faulhaber OK";
	}
	catch(Ice::Exception &e)
	{	std::cout << e<<std::endl;	}
	
	try
	{		
		mplMuecas =jointmotor1_proxy->getAllMotorParams();
		foreach (RoboCompJointMotor::MotorParams motor, mplMuecas)
			mplAll.push_back(motor);
		qDebug() << __FUNCTION__ << "Connection to SERVOS OK";
	}
	catch(Ice::Exception &e)
	{	std::cout << e<<std::endl; }

// 	//Now checking for motors in JointMotor that match localParams
// 	int cont=0;
// 	QVector<RoboCompJointMotor::MotorParams> qmp = QVector<RoboCompJointMotor::MotorParams>::fromStdVector( mplFaulhaber );
// 	foreach( RoboCompJointMotor::MotorParams qmp, mplFaulhaber)
// 	{
// 		if (qmp.name == neckMotorName  or  qmp.name == tiltMotorName  or qmp.name == leftPanMotorName or  qmp.name == rightPanMotorName )
// 		{
// 			headParams.motorsParams[qmp.name] = qmp;
// 			cont++;
// 		}
// 		else
// 			qDebug() << "muecasHead::Monitor::Initialize() - No required motor found in running JointMotor: "<<qmp.name.c_str();
// 	}
	
// 	if(cont != 4)
// 	{
// 		qDebug() << "muecasHead::Monitor::Initialize() - Required motors not found in running JointMotor";
// 		qFatal("error initializing head motors");
// 	}
// 	headParams.model = "NT2P";

	qDebug() << "muecasHead() - constructor Ok";
	
	////////////////////////////////////////////////////////////////////////////
		
 	qImageRGB = new QImage(640,480, QImage::Format_Indexed8);
// 	Rgbcv = new RCDraw(paramsCamera.width,paramsCamera.height, qImageRGB, imgframe);
// 	
// 	qImageRGB2 = new QImage(paramsCamera.width,paramsCamera.height, QImage::Format_Indexed8);
// 	Rgbcv2 = new RCDraw(paramsCamera.width,paramsCamera.height, qImageRGB2, imgframe2);
// 	
// 	cvImage = cvCreateImage(cvSize(320,240), 8, 1);
// 	cvImage2 = cvCreateImage(cvSize(320,240), 8, 1);
// 	cvrgb = cvCreateImage(cvSize(320,240), 8, 1);
// 	cvrgb2 = cvCreateImage(cvSize(320,240), 8, 1);
	
	///////////////////////////////////////////////////////////////////////
	 
	connect(neckpitch, SIGNAL(valueChanged(double)),this, SLOT(moveNeck(double)));
	connect(neckroll, SIGNAL(valueChanged(double)),this, SLOT(moveNeck(double)));
	connect(neckyaw, SIGNAL(valueChanged(double)),this, SLOT(moveNeck(double)));
	connect(eyetilt, SIGNAL(valueChanged(double)),this, SLOT(moveNeck(double)));
	connect(lefteyepan, SIGNAL(valueChanged(double)),this, SLOT(moveNeck(double)));
	connect(righteyepan, SIGNAL(valueChanged(double)),this, SLOT(moveNeck(double)));
	connect(lefteyebrowpitch, SIGNAL(valueChanged(double)),this, SLOT(moveFaceElems(double)));
	connect(righteyebrowpitch, SIGNAL(valueChanged(double)),this, SLOT(moveFaceElems(double)));
	connect(lefteyebrowroll, SIGNAL(valueChanged(double)),this, SLOT(moveFaceElems(double)));
	connect(righteyebrowroll, SIGNAL(valueChanged(double)),this, SLOT(moveFaceElems(double)));
	
	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));	
	connect(ButtonSpeech, SIGNAL(clicked()),this, SLOT(habla()));
	
	timer.start(100); //is in ms
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	timer.start(Period);

	return true;
};

void SpecificWorker::compute( )
{
////////////////motores///////////////////////////////////////
	try
	{
		//qDebug() << "Talking to head";
		jointmotor2_proxy->getAllMotorState(stateFaulhaber);
	}
	catch(Ice::Exception &ex)
	{	std::cout << ex<<std::endl; }

	try
	{
		jointmotor1_proxy->getAllMotorState(stateMuecas);
	}
	catch(Ice::Exception &ex)
	{ std::cout << ex<<std::endl;	}
	
	
	try
	{
		dataImu = imu_proxy->getDataImu();
		lcdNumberAccX->display(dataImu.acc.XAcc);
		lcdNumberAccY->display(dataImu.acc.YAcc);
		lcdNumberAccZ->display(dataImu.acc.ZAcc);
		lcdNumberMagX->display(dataImu.mag.XMag);
		lcdNumberMagY->display(dataImu.mag.YMag);
		lcdNumberMagZ->display(dataImu.mag.ZMag);
		lcdNumberGyrX->display(dataImu.gyro.XGyr);
		lcdNumberGyrY->display(dataImu.gyro.YGyr);
		lcdNumberGyrZ->display(dataImu.gyro.ZGyr);
	}
	catch(Ice::Exception &ex)
	{ std::cout << ex<<std::endl;	}
	
	mutex_memory->lock();
		stateAll = stateFaulhaber;
		std::pair<std::string,RoboCompJointMotor::MotorState> pair;
		foreach (pair,stateMuecas)
			stateAll[pair.first] = pair.second;
	mutex_memory->unlock();
	
	///////////camara/////////////////////////////////////////////////
// 	try{
// 		
// 			camera_proxy->getYImage(5, imgCam, hState, bState);
//   			memcpy(cvImage->imageData, &imgCam[0], imgSize);	
//  			memcpy(cvImage2->imageData, &imgCam[imgSize], imgSize);	
//   	}
//   	catch(const Ice::Exception& e)
//   	{
//   		qDebug()<<"Error reading camera images";
//   	}
// 
//  	cvFlip(cvImage, cvImage, 0);
//  	cvFlip(cvImage2, cvImage2, 0);
//  	memcpy (  qImageRGB->bits(), cvImage->imageData, paramsCamera.width*paramsCamera.height );
// 	memcpy (  qImageRGB2->bits(), cvImage2->imageData, paramsCamera.width*paramsCamera.height );

	 //IMAGE
// 		FlyCapture2::Error error;
// 		FlyCapture2::Image rawImageL, rawImageR;	
// 		 
// 		error = ppCameras[0]->RetrieveBuffer( &rawImageL );
// 		 qDebug() << "start cap";
// 		if ( error != FlyCapture2::PGRERROR_OK )
// 		{
// 			std::cout << "capture error" << std::endl;
// 		}
//  		error = ppCameras[1]->RetrieveBuffer( &rawImageR );
//  		if ( error != FlyCapture2::PGRERROR_OK )
//  		{
//  			std::cout << "capture error" << std::endl;
//  		}
// 		
// 		// convert to rgb
// 	  //FlyCapture2::Image rgbImage;
//     //rawImageL.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
// 		// convert to OpenCV Mat
// 		unsigned int rowBytes = (double)rawImageL.GetReceivedDataSize()/(double)rawImageL.GetRows();       
// 		cv::Mat imageL = cv::Mat(rawImageL.GetRows(), rawImageL.GetCols(), CV_8UC1, rawImageL.GetData(),rowBytes);
// 		cv::flip(imageL, imageL, 0);
// 		const QImage qimageL(imageL.data, imageL.cols, imageL.rows, imageL.step, QImage::Format_Indexed8);
// 		labelLeft->setPixmap(QPixmap::fromImage(qimageL));
// 		cv::Mat imageR = cv::Mat(rawImageR.GetRows(), rawImageR.GetCols(), CV_8UC1, rawImageR.GetData(),rowBytes);
// 		cv::flip(imageR, imageR, 0);
// 		const QImage qimageR(imageR.data, imageR.cols, imageR.rows, imageR.step, QImage::Format_Indexed8);
// 		labelRight->setPixmap(QPixmap::fromImage(qimageR));
// 		
// 		cv::StereoBM sbm;
// // 		sbm.state->SADWindowSize = 9;
// // 		sbm.state->numberOfDisparities = 112;
// // 		sbm.state->preFilterSize = 5;
// // 		sbm.state->preFilterCap = 61;
// // 		sbm.state->minDisparity = -39;
// // 		sbm.state->textureThreshold = 507;
// // 		sbm.state->uniquenessRatio = 0;
// // 		sbm.state->speckleWindowSize = 0;
// // 		sbm.state->speckleRange = 8;
// // 		sbm.state->disp12MaxDiff = 1;
// 		Mat disp, disp_norm, disp_norm_scaled;
// 		sbm(imageR, imageL, disp);
// 		cv::normalize( disp, disp_norm, 0, 255, NORM_MINMAX, CV_8U, Mat() );
// 		//cv::convertScaleAbs( disp_norm, disp_norm_scaled );
// 		cv::imshow("disp",disp_norm);
// 		cv::waitKey(10);
//     // resize the label to fit the image
//     //labelRight->resize(label->pixmap()->size());
// 		// 		cv::imshow("image", image);
// 		// 		cv::waitKey(10); 
}



void SpecificWorker::habla()
{
  try
	{
			speech_proxy->say(textEdit->toPlainText().toStdString(),true);
	}
	catch (Ice::Exception e) 
	{		qDebug()<<"Error talking to Speech: "<<e.what();} 
}

////////////////////

RoboCompJointMotor::MotorParamsList SpecificWorker::getAllMotorParams()
{
	return mplAll;
}
void  SpecificWorker::getAllMotorState(RoboCompJointMotor::MotorStateMap& mstateMap)
{
	mutex_memory->lock();
		mstateMap = stateAll;
	mutex_memory->unlock();
}
void  SpecificWorker::setPosition(const RoboCompJointMotor::MotorGoalPosition& goal)
{
	QMutexLocker lock(mutex);
	try
	{
		jointmotor2_proxy->setPosition( goal );
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{ 		throw ex;	}
	catch( RoboCompJointMotor::HardwareFailedException & ex)
	{		throw ex;	}
}

void SpecificWorker::move(float pos,  const string& Name )
{
	RoboCompJointMotor::MotorGoalPosition g;
 	g.name = Name;
 	g.maxSpeed = 0;
 	g.position = pos;
	QMutexLocker lock(mutex);
 	try
 	{
 		jointmotor1_proxy->setPosition( g );
 	}
 	catch( RoboCompJointMotor::OutOfRangeException &ex )
 	{
 		throw ex;
 	}
 	catch(Ice::Exception &ex)
 	{
 		throw ex;
 	}	
}

void SpecificWorker::moveneckimitation(float newpitch, float newroll, float newyaw)
{
		float rotations_eyeA = 0;
		float rotations_eyeB = 0;
		
		float eyestilt = 0;
		float ROTATE = newyaw;
		
		float POS_Zz = newpitch;
		float POS_Xx = -newpitch + newroll;
		float POS_Yy = -newpitch - newroll;
		
		int pasos_rotate = ROTATE * 17960.f / 6.28;
		int pasos_zz = POS_Zz * 17960.f / 6.28; 
		int pasos_xx = POS_Xx * 17960.f / 6.28;
		int pasos_yy = POS_Yy * 17960.f / 6.28;
		
		qDebug()<<"pasos"<<pasos_zz<<pasos_xx<<pasos_yy<<pasos_rotate;
		
			
			// limites de los recorridos de los motores
			if ( pasos_xx > MIN_X && pasos_xx < MAX_X && pasos_yy > MIN_Y && pasos_yy < MAX_Y && pasos_zz > MIN_Z && pasos_zz < MAX_Z && pasos_rotate > MIN_TILT &&  pasos_rotate < MAX_TILT)
			{
				qDebug()<<"stting positios"<<"front"<<POS_Zz<<"right"<<POS_Xx<<"left"<<POS_Yy;
				RoboCompJointMotor::MotorGoalPosition p_goal;
				RoboCompJointMotor::MotorGoalPositionList list;
				
				p_goal.name = "neckRight";
				p_goal.position = POS_Xx;
				list.push_back(p_goal);
				
				p_goal.name = "neckLeft";
				p_goal.position = POS_Yy;
				list.push_back(p_goal);
				
				p_goal.name = "neckFront";
				p_goal.position = POS_Zz;
				list.push_back(p_goal);

				
				p_goal.name = "neck";
				p_goal.position = ROTATE;
				list.push_back(p_goal);
				
				
				p_goal.name = "rightPan";
				p_goal.position = rotations_eyeA;
				list.push_back(p_goal);
				
							
				p_goal.name = "leftPan";
				p_goal.position = rotations_eyeB;
				list.push_back(p_goal);
				
				
				p_goal.name = "tilt";
				p_goal.position = eyestilt;
				list.push_back(p_goal);
			
				mutex->lock();
				try
				{
					jointmotor2_proxy->setSyncPosition( list );
				}
				catch( Ice::Exception & ex)
				{
					std::cout<< ex.what()<< std::endl;
				}
				mutex->unlock();
			}
		
}


void SpecificWorker::moveNeck(double l)
{
		float rotations_eyeA = righteyepan->value();
		float rotations_eyeB = lefteyepan->value();
		
		float eyestilt = eyetilt->value();
		float ROTATE = neckyaw->value();
		
		float POS_Zz = neckpitch->value();
		float POS_Xx = -neckpitch->value() + neckroll->value();
		float POS_Yy = -neckpitch->value() - neckroll->value();
		
		int pasos_rotate = ROTATE * 17960.f / 6.28;
		int pasos_zz = POS_Zz * 17960.f / 6.28; 
		int pasos_xx = POS_Xx * 17960.f / 6.28;
		int pasos_yy = POS_Yy * 17960.f / 6.28;
		
		qDebug()<<"pasos"<<pasos_zz<<pasos_xx<<pasos_yy<<pasos_rotate;
		
			
			// limites de los recorridos de los motores
			if ( pasos_xx > MIN_X && pasos_xx < MAX_X && pasos_yy > MIN_Y && pasos_yy < MAX_Y && pasos_zz > MIN_Z && pasos_zz < MAX_Z && pasos_rotate > MIN_TILT &&  pasos_rotate < MAX_TILT)
			{
				qDebug()<< __FUNCTION__ << "Setting position" << "front" << POS_Zz << "right" << POS_Xx << "left" << POS_Yy;
				RoboCompJointMotor::MotorGoalPosition p_goal;
				RoboCompJointMotor::MotorGoalPositionList list;
				
				p_goal.name = "neckRight";
				p_goal.position = POS_Xx;
				list.push_back(p_goal);
				
				p_goal.name = "neckLeft";
				p_goal.position = POS_Yy;
				list.push_back(p_goal);
				
				p_goal.name = "neckFront";
				p_goal.position = POS_Zz;
				list.push_back(p_goal);

				p_goal.name = "neck";
				p_goal.position = ROTATE;
				list.push_back(p_goal);
								
				p_goal.name = "rightPan";
				p_goal.position = rotations_eyeA;
				list.push_back(p_goal);
				
				p_goal.name = "leftPan";
				p_goal.position = rotations_eyeB;
				list.push_back(p_goal);
				
				p_goal.name = "tilt";
				p_goal.position = eyestilt;
				list.push_back(p_goal);
			
				mutex->lock();
					try
					{
						jointmotor2_proxy->setSyncPosition( list );
					}
					catch( Ice::Exception & ex)
					{
						std::cout<< ex.what()<< std::endl;
					}
				mutex->unlock();
			}
}

void SpecificWorker::moveFaceElems(double l)
{
	move(mouth->value(), mouthMotorName);
	
	move(-lefteyebrowroll->value(), eyeBLeftYAWMotorName );
	
	move(righteyebrowroll->value(), eyeBRightYAWMotorName );
    
	move(lefteyebrowpitch->value(), eyeBLeftUPMotorName );
	
	move(righteyebrowpitch->value(), eyeBRightUPMotorName );

}


float SpecificWorker::normalize(float X, float A, float B, float C, float D)
{
	return ((D-C)*(X-A)/(B-A))+C;
}



/*
float SpecificWorker::euclideanDistance3D(Vector3DF p1, Vector3DF p2)
{
 // return Tomilimeter(sqrt(pow(p1.X - p2.X,2)*1000 + pow(p1.Y - p2.Y,2)*1000 + pow(p1.Z - p2.Z,2)*1000)); 
  return Tomilimeter(sqrt(pow(p1.X - p2.X,2) + pow(p1.Y - p2.Y,2) + pow(p1.Z - p2.Z,2)));
}*/

/*
	FlyCapture2::Error error;
	FlyCapture2::CameraInfo camInfo;

	// Connect the camera
	FlyCapture2::BusManager bus;
	uint nCams;
	bus.GetNumOfCameras(&nCams);
	FlyCapture2::PGRGuid pGuid;
	error = bus.GetCameraFromIndex(0,&pGuid);
	if ( error != FlyCapture2::PGRERROR_OK )
  {
      error.PrintErrorTrace();
			return false;
  }
  cout << "Number of cameras detected: " << nCams << endl; 
  if ( nCams < 1 )
    {
        cout << "Insufficient number of cameras... press Enter to exit." << endl; ;
        cin.ignore();
        return false;
    }
  ppCameras = new FlyCapture2::Camera*[nCams];
	// Connect to all detected cameras and attempt to set them to a common video mode and frame rate
  for ( unsigned int i = 0; i < nCams; i++)
  {
		ppCameras[i] = new FlyCapture2::Camera();

    FlyCapture2::PGRGuid guid;
    error = bus.GetCameraFromIndex( i, &guid );
    if (error != FlyCapture2::PGRERROR_OK)
    {
            error.PrintErrorTrace();
            return false;
    }

		// Connect to a camera
		error = ppCameras[i]->Connect( &guid );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return false;
		}
		// Set all cameras to a specific mode and frame rate so they
    // can be synchronized
    error = ppCameras[i]->SetVideoModeAndFrameRate( FlyCapture2::VIDEOMODE_640x480Y8, FlyCapture2::FRAMERATE_30 );
    if (error != FlyCapture2::PGRERROR_OK)
		{
        error.PrintErrorTrace();
        cout << "Error starting cameras. " << endl;
        cout << "This example requires cameras to be able to set to 640x480 Y8 at 30fps. " << endl;
        cout << "If your camera does not support this mode, please edit the source code and recompile the application. " << endl;
        cout << "Press Enter to exit. " << endl;
        cin.ignore();
        return false;
    }
  }
 	qDebug() << "start cap";
  //error = FlyCapture2::Camera::StartSyncCapture( nCams, (const FlyCapture2::Camera**)ppCameras );
		qDebug() << "start cap";
	error = ppCameras[0]->StartCapture();
	if (error != FlyCapture2::PGRERROR_OK)
  {
     error.PrintErrorTrace();
     cout << "Error starting cameras. " << endl;
     cout << "This example requires cameras to be able to set to 640x480 Y8 at 30fps. " << endl;
     cout << "If your camera does not support this mode, please edit the source code and recompile the application. " << endl;
     cout << "Press Enter to exit. " << endl;
     cin.ignore();
     return false;
  }
	error = ppCameras[1]->StartCapture();
	if (error != FlyCapture2::PGRERROR_OK)
  {
     error.PrintErrorTrace();
     cout << "Error starting cameras. " << endl;
     cout << "This example requires cameras to be able to set to 640x480 Y8 at 30fps. " << endl;
     cout << "If your camera does not support this mode, please edit the source code and recompile the application. " << endl;
     cout << "Press Enter to exit. " << endl;
     cin.ignore();
     return false;
  }*/
