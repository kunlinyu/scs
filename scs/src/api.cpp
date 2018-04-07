#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <Eigen/Eigen>
#include <GL/glut.h>
#include <ode/ode.h>

#include "mytime.h"
#include "simulation.h"
#include "draw.h"
#include "common.h"
#include "car.h"
#include "api.h"
#include "track.h"

	pFunc AiFunc = NULL;		// AI callback function
	pFunc DisplayFunc [CUSTOMWINDOWNUM];	// Display callback function
	void (*InitFunc)() = NULL;		// Init callback function
	char TrackName[255] = {0};		// track file's name

	Eigen::Vector3d CurrentSpeed;
	Eigen::Vector3d LastSpeed;

void sGetGraph (unsigned char graph[GRAPH_HEIGHT][GRAPH_WIDTH])
{
	static unsigned char image[GRAPH_HEIGHT][GRAPH_WIDTH][4];
	static int first = 1;
	if (first) {
		first = 0;
		srand((unsigned int)time(NULL));
	}
	glutSetWindow (WinCar);
#ifdef WIN32
	glReadPixels(0,0,GRAPH_WIDTH,GRAPH_HEIGHT,GL_RGBA,GL_UNSIGNED_BYTE,image);
#else
	glReadPixels(0,0,GRAPH_WIDTH,GRAPH_HEIGHT,GL_BGRA,GL_UNSIGNED_BYTE,image);
#endif

	for (int i=0; i<GRAPH_HEIGHT; i++)
		for (int j=0; j<GRAPH_WIDTH; j++) {
			graph[i][j] = (image[i][j][0]+image[i][j][1]+image[i][j][2])/3;	// gray
			if (graph[i][j]<250 && graph[i][j]>5)	// noise
				graph[i][j] += rand()%10-5;
			else if (graph[i][j]>=250)
				graph[i][j] -= rand()%5;
			else if (graph[i][j]<=5)
				graph[i][j] += rand()%5;
		}

}

void sGetLine (unsigned char line[GRAPH_WIDTH])
{
	static unsigned char image[GRAPH_WIDTH][4];
	glutSetWindow (WinCar);
#ifdef WIN32
	glReadPixels(0,0,GRAPH_WIDTH,1,GL_RGBA,GL_UNSIGNED_BYTE,image);
#else
	glReadPixels(0,0,GRAPH_WIDTH,1,GL_BGRA,GL_UNSIGNED_BYTE,image);
#endif

	for (int i=0; i<GRAPH_WIDTH; i++) {
		line[i] = (image[i][0]+image[i][1]+image[i][2])/3;
		if (line[i]<250 && line[i]>5)	// noise
			line[i] += rand()%10-5;
		else if (line[i]>=250)
			line[i] -= rand()%5;
		else if (line[i]<=5)
			line[i] += rand()%5;
	}
}

void sSetMotor (int duty)
{
	if (drivemode==debug) return;
	sSetMotorL (duty);
	sSetMotorR (duty);
}

void sSetMotorL (int duty)
{
	if (drivemode==debug) return;
	if (duty> 500) {
		duty = 500;
		printf ("SetMotorL: Too high!\n");
	}
	if (duty<-500) {
		duty =-500;
		printf ("SetMotorL: Too low!\n");
	}
	MotorDutyL = duty;
}

void sSetMotorR (int duty)
{
	if (drivemode==debug) return;
	if (duty> 500) {
		duty = 500;
		printf ("SetMotorR: Too high!\n");
	}
	if (duty<-500) {
		duty =-500;
		printf ("SetMotorR: Too low!\n");
	}
	MotorDutyR = duty;
}

void sSetServoDir (int dir)
{
	if (drivemode==debug) return;
	if (!car_obj.GetCarDirection())
		ServoDir = dir;
	else	ServoDir = -dir;

	if (ServoDir> 100) {
		ServoDir = 100;
		printf("SetServoDir: Too high!\n");
	}
	if (ServoDir<-100) {
		ServoDir =-100;
		printf("SetServoDir: Too low!\n");
	}
}

void sEnableReverse ()
{
	if (car_obj.GetCarReverseFlag())	car_obj.SetCarReverseFlag(0);
	else			car_obj.SetCarReverseFlag(1);
	printf("INIT: Reverse enabled.\n");
}

void sEnableMiddleLine ()
{
	if (MiddleLineFlag)	MiddleLineFlag = 0;
	else			MiddleLineFlag = 1;
	printf("INIT: Middle line enabled.\n");
}

void sSetMu (double mu)
{
	Mu = mu;
	printf("MU: set Mu==%lf\n",Mu);
}

void sEnableCustomWindow (int num)
{
	CustomWindowFlag = 1;
	CustomWindowNum = num;
	if (CustomWindowNum>=CUSTOMWINDOWNUM)
		CustomWindowNum = CUSTOMWINDOWNUM-1;
	printf("INIT: Custom window enabled.\n");
}

void sEnableCustomWindow ()
{
	sEnableCustomWindow (1);
}

void sSetDisplayFunc (void f(), int num)
{
	if (num>=CUSTOMWINDOWNUM) {
		num = 0;
		printf("DISPLAY: window num too high.\n");
	}
	DisplayFunc[num] = f;
	printf("INIT: Display function have been set.\n");
}

void sSetDisplayFunc (void f())
{
	sSetDisplayFunc (f,0);
}

double sGetASpeedL ()
{
	Eigen::Vector3d vl(dBodyGetAngularVel (car_obj.GetWheelBL()->body));
	Eigen::Vector3d pl(dBodyGetPosition   (car_obj.GetWheelBL()->body));
	Eigen::Vector3d pr(dBodyGetPosition   (car_obj.GetWheelBR()->body));
	Eigen::Vector3d v = pl - pr;
	return vl.dot(v) / v.norm () ;
}

double sGetASpeedR ()
{
	Eigen::Vector3d vr(dBodyGetAngularVel (car_obj.GetWheelBR()->body));
	Eigen::Vector3d pl(dBodyGetPosition   (car_obj.GetWheelBL()->body));
	Eigen::Vector3d pr(dBodyGetPosition   (car_obj.GetWheelBR()->body));
	Eigen::Vector3d v = pl - pr;
	return vr.dot(v) / v.norm () ;
}

double sGetASpeed ()
{
	return (sGetASpeedL () + sGetASpeedR ()) /2.0;
}
double sGetSpeedL ()
{ return sGetASpeedL () * car_obj.GetWheelRadius(); }

double sGetSpeedR ()
{ return sGetASpeedR () * car_obj.GetWheelRadius(); }

double sGetSpeed ()
{ return sGetASpeed () * car_obj.GetWheelRadius();}

void scsMainLoop (int * argc, char *argv[])
{
	printf(	"\n"VERSION" MainLoop\n");
	
	if (car_obj.GetCarReverseFlag())
		if (car_obj.GetCarDirection())	car_obj.SetCarDirection(false);
		else				car_obj.SetCarDirection(true);
	
	dInitODE2 (0);
	ResetSimulation ();

	glutInit (argc,argv);
	DrawInit ();

	printf("\nRolling...\n");
	glutMainLoop ();

	DestroySimulation ();
	dCloseODE ();
}

void sSetCar (CarType ct)
{
	if (ct!=camera && ct!=balance && ct!= electromagnetic) {
		ct = camera;
		printf("SETCAR: car type error.\n");
	}
	cartype = ct;
	if (cartype == camera)	car_obj.SetCarDirection(true);
	else					car_obj.SetCarDirection(false);
	if (cartype == electromagnetic)
		MiddleLineFlag = 1;
	printf("INIT: Car type have been set to be ");
	switch (cartype) {
		case camera: printf("camera.\n"); break;
		case balance: printf("balance.\n"); break;
		case electromagnetic: printf("electromagnetic.\n"); break;
	}
}

void sSetAiFunc (void f())
{
	AiFunc = f;
	printf("INIT: AI function have been set.\n");
}

void sSetInitFunc (void f())
{
	InitFunc = f;
	printf("INIT: Initialize function have been set.\n");
}

void sSetTrack (const char * s)
{
	strncpy(TrackName,s,255);
	printf("INIT: Track file's name have been set: \"%s\"\n",s);
}

void sEnableRoute ()
{
	RouteFlag = 1;
	printf("INIT: Route enabled.\n");
}

void sEnablePath (double security)
{
	PathFlag = 0;
	PathSecurity = security;
	printf("INIT: Path disabled. (security == %lf)\n",PathSecurity);
}

void sEnablePath ()
{
	PathFlag = 0;
	printf("INIT: Path disabled. (security == %lf)\n",PathSecurity);
}

void sSetCamera (Eigen::Vector3d camerapos)
{
	CameraPos = camerapos;
	printf("CAMERA: new position ");
//	CameraPos.print ();  // TODO
}

void sSetCCD (Eigen::Vector3d camerapos)
{
	CameraPos = camerapos;
	printf("CAMERA: new position ");
//	CameraPos.print ();  // TODO
}

void sSetDepressionAngle (double depressionangle)
{
	DepressionAngle = depressionangle;
	if (DepressionAngle<0.0) DepressionAngle = 0.0;
	if (DepressionAngle>90.0)DepressionAngle = 90.0;
	printf("DEPRESSION: %6.3lf\n",DepressionAngle);
}

void sSetBatteryPosition (Eigen::Vector3d pos)
{
  car_obj.SetBatteryPos(pos);
	printf("BATTERY: new position ");
//	pos.print ();  // TODO
}

Eigen::Vector3d sGetAngularSpeed ()
{
	Eigen::Vector3d av (dBodyGetAngularVel(car_obj.GetChassis()->body));
	return car_obj.ToCarCoo(av);
}

Eigen::Vector3d sGetAcc ()
{
	Eigen::Vector3d acc = CurrentSpeed - LastSpeed;
	acc /= CurrentStepTime;
	acc.z() = acc.z() + 9.8;
	return car_obj.ToCarCoo(acc);
}

Eigen::Vector3d sGetPosition ()
{
	Eigen::Vector3d pos(car_obj.GetPosition());
	return pos;
}

Eigen::Vector3d sGetDirection ()
{
	return car_obj.CarY();
}

Eigen::Vector3d sGetMagnetic (Eigen::Vector3d pos)
{
	Eigen::Vector3d mag;
	Eigen::Vector3d CarPos(car_obj.GetPosition());
  /*
  // TODO: inductance_number
	if (car_obj.InductanceNumber() >= INDUCTANCE_NUM_MAX) return mag;
	InductancePos[car_obj.InductanceNumber()] = pos;
	pos = ToWorldCoo(pos) + CarPos;
	for (int i=0; i<PathNum-1; i++) {
		if ((Middle[i+0] - pos).norm() > 3.0 &&
        (Middle[i+1] - pos).norm() > 3.0) continue;
		Eigen::Vector3d r1	= pos - Middle[i];
		Eigen::Vector3d r2	= pos - Middle[i+1];
		Eigen::Vector3d l	= Middle[i+1] - Middle[i];
		Eigen::Vector3d dir	= l.cross(r1).normalized();
		double up	= l.dot(r1) / l.cross(r1).norm() / r1.norm();
		double down	= l.dot(r2) / l.cross(r2).norm() / r2.norm();
		mag += dir * (up - down);
	}
	Magnetic[car_obj.InductanceNumber()] = ToCarCoo (mag);
  car_obj.SetInductanceNumber(car_obj.InductanceNumber()+1);
	return ToCarCoo (mag);
  */
}

int sGetReedSwitch ()
{
	Eigen::Vector3d pos(car_obj.GetPosition());
	Eigen::Vector3d v1 = ELineR - ELineL;	// on track
	Eigen::Vector3d v2 = pos - ELineL;
	if (v1.dot(v2) < 0) return 0;

	v1 *= -1;		// on track
	v2 = pos - ELineR;
	if (v1.dot(v2) < 0) return 0;

					// in oval
	if (((pos-ELineL).norm()+(pos-ELineR).norm())>sqrt(TRACK_WIDTH_OUT*TRACK_WIDTH_OUT+5.0*5.0*CONTROLTIME*CONTROLTIME))
		return 0;
	return 1;
}
