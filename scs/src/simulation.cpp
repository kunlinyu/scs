#include <stdlib.h>
#include <iostream>
#include <time.h>

#include <Eigen/Eigen>
#include <ode/ode.h>

#include "simulation.h"
#include "track.h"
#include "car.h"
#include "draw.h"
#include "api.h"
#include "ui.h"

#define ITERS	10
#define MU	1.7
#define SLIP	0.7

	dWorldID	world;
	dSpaceID	space;
	dJointGroupID	contactgroup = NULL;

	int MotorDutyL = 0;
	int MotorDutyR = 0;
	double AngularSpeedL = 0.0;
	double AngularSpeedR = 0.0;
	int ServoDir = 0;

	double Mu = MU;
	double Slip = SLIP;
	double CurrentStepTime = 0.0;

  Car car_obj;
  Track track;

void ResetSimulation ()
{
	static int first = 1;

	// destroy world if it exists
	if (first) {
		first = 0;
	}else	DestroySimulation ();


	// create world
	world = dWorldCreate ();
	//space = dHashSpaceCreate (0);
	//space = dSimpleSpaceCreate (0);
	space = dSweepAndPruneSpaceCreate (0, dSAP_AXES_XYZ);

	// physical things
	contactgroup = dJointGroupCreate (0);
	dWorldSetGravity (world, 0, 0, -9.8);
	dWorldSetCFM (world, 1e-2);
	dWorldSetERP (world, 0.8);
	dWorldSetQuickStepNumIterations (world, ITERS);
	dCreatePlane (space, 0, 0, 1, 0);
	printf("SIM: Physical constant created!\n");

	switch (cartype) {
		case camera:
		case electromagnetic:	car_obj.MakeCar (0.0, 0.0, world, space);	break;
		case balance:		car_obj.MakeBalanceCar (0.0, 0.0, world, space);break;
	}
	printf("SIM: Car created!\n");
	
	track.MakeTrack (space,TrackName);
	printf("SIM: Track created!\n");
	printf("SIM: Virtual world created!\n");

	if (InitFunc) InitFunc ();
	ClearRoute ();
}

void DestroySimulation() {
  // TODO: destroy car ?
	track.DestroyTrack ();
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);
	printf("SIM: Virtual world destroyed!\n\n\n");
}

// This function was writter according to demos of ode
// It will be call in every step
static void CollideCallback (void *data, dGeomID o1, dGeomID o2)
{
	static int first = 1;
	dBodyID b1 = dGeomGetBody (o1);
	dBodyID b2 = dGeomGetBody (o2);
	if (b1 && b2 && dAreConnected (b1, b2)) return;
	
	if (first) {
		first = 0;
		srand((unsigned int)time(NULL));
	}
	const int N = 10;
	dContact contact[N];
	int n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
	if (n<=0) return;
	for (int i=0; i<n; i++) {
		contact[i].surface.mu		= Mu	+ rand()%100/2000.0;
		contact[i].surface.slip1	= Slip	+ rand()%100/2000.0;
		contact[i].surface.slip2	= Slip	+ rand()%100/2000.0;
		contact[i].surface.soft_erp	= 0.4;
		contact[i].surface.soft_cfm	= 0.01;
		contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
		dJointID c = dJointCreateContact (world,contactgroup,contact+i);
		dJointAttach (c,dGeomGetBody(o1),dGeomGetBody(o2));
	}
}

// When the car cross the end line, calc time and round.
static void timer (double time) {
	static int first = 1;
	double AverageSpeed;
	static double dir = -1.0;
	static double Time = 0.0;
	static Eigen::Vector3d last(0,0,-1);
	static int Round = 0;
	Eigen::Vector3d pos = car_obj.GetPosition();
	Eigen::Vector3d v1 = track.ELineL() - pos;
	Eigen::Vector3d v2 = track.ELineR() - pos;

	Time += time;		// at the different side of end line;
	if (v1.cross(v2).z() * last.z() > 0) return;
	last = v1.cross(v2);
	
	v1 = track.ELineR() - track.ELineL();	// on track
	v2 = pos - track.ELineL();
	if (v1.dot(v2) < 0) return;

	v1 *= -1;		// on track
	v2 = pos - track.ELineR();
	if (v1.dot(v2) < 0) return;

	if (first) {
		first = 0;
		AverageSpeed = track.EndLineDistance() / Time;
	} else	AverageSpeed = track.TotalLength() / Time;

	if (AverageSpeed > 10.0) {
		printf("Abandoned result\n");
		Round --;
	}
	else	printf("Round: %d\tTime: %5.2f\tAveSpeed: %5.2f\t",Round,Time,AverageSpeed);
	
	if (track.TotalLength() / Time < 1.0)	printf("loser...\n");
	else if (AverageSpeed < 2.0)	printf("Hurry up!\n");
	else if (AverageSpeed < 3.0)	printf("Good!\n");
	else if (AverageSpeed < 4.0)	printf("Awesome!\n");
	else if (AverageSpeed < 5.0)	printf("Amazing!\n");
	else if (AverageSpeed <10.0)	printf("That's impossible...\n");
	
	Time = 0.0;
	Round ++;
}

double sign(double x)
{
	if (x>0)	return 1.0;
	else		return -1.0;
}

static void motor ()
{
	double fmax = 0.1;
	double Pl = MotorDutyL/10.0;
	double Pr = MotorDutyR/10.0;
	double v0l = Pl/fmax;
	double v0r = Pr/fmax;
	if (Pl==0.0) v0l += 0.01;
	if (Pr==0.0) v0r += 0.01;
	double Aspeedl = sGetASpeedL () + sign(sGetASpeedL ())*fabs(v0l);
	double Aspeedr = sGetASpeedR () + sign(sGetASpeedR ())*fabs(v0r);

	double torquel = sign(Pl)*fabs(Pl/Aspeedl)+sign(Pl!=0.0?Pl:Aspeedl)-Aspeedl*0.0001-sign(Aspeedl)*0.01;
	double torquer = sign(Pr)*fabs(Pr/Aspeedr)+sign(Pr!=0.0?Pr:Aspeedr)-Aspeedr*0.0001-sign(Aspeedr)*0.01;

  car_obj.SetTorque(torquel, torquer);
	
	//dJointSetHingeParam (joint_bl_,dParamVel,AngularSpeedL);
	//dJointSetHingeParam (joint_br_,dParamVel,AngularSpeedR);
}

static void servo () {
  car_obj.SetServo(ServoDir);
}

void step (double stepsize)
{
	timer (stepsize);
	LastSpeed = LastSpeed*0.0 + CurrentSpeed*1.0;
	Eigen::Vector3d v(car_obj.GetLinearVel());
	CurrentSpeed = CurrentSpeed*0.9 + v*0.1;
	CurrentStepTime = stepsize;

	motor ();
	if (cartype!=balance) {
		servo ();
	}
	
	dSpaceCollide (space,0,&CollideCallback);
	dWorldQuickStep (world,stepsize);
	dJointGroupEmpty (contactgroup);
}
