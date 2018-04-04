#ifndef _CAR_H_
#define _CAR_H_

#include <Eigen/Eigen>
#include <ode/common.h>
#include "track.h"

#define A_WHEEL_RADIUS		0.025	// wheel radius
#define A_WHEEL_FRONT_WIDTH	0.025	// wheel width
#define A_WHEEL_BACK_WIDTH	0.025	// wheel width

#define B_WHEEL_RADIUS		0.03	// wheel radius
#define B_WHEEL_FRONT_WIDTH	0.025	// wheel width
#define B_WHEEL_BACK_WIDTH	0.04	// wheel width

#define INDUCTANCE_NUM		128	// max number of inductance

#define STARTZ	(TRACK_HEIGHT + 0.05)	// starting height of chassis
#define FMAX	1.0	// car engine fmax

typedef struct sObject {
	dBodyID body;
	dGeomID geom;
} * sObjectID;

extern sObjectID Chassis;
extern sObjectID Wheel_FL, Wheel_FR, Wheel_BL, Wheel_BR;
extern dJointID Joint_FL, Joint_FR, Joint_BL, Joint_BR;
extern int CarReverseFlag;
extern int CarDirection;
extern Eigen::Vector3d BatteryPos;
extern Eigen::Vector3d BatteryPosR;
extern Eigen::Vector3d BatteryPosB;
extern double WheelRadius;
extern double CarWidth;
extern Eigen::Vector3d InductancePos [INDUCTANCE_NUM];
extern Eigen::Vector3d Magnetic [INDUCTANCE_NUM];
extern int Ip;

void MakeCar (double x, double y, dWorldID world, dSpaceID space);
void ResetCar ();
void MakeBalanceCar (double x, double y, dWorldID world, dSpaceID space);
void DrawCar ();
void DrawBalanceCar ();
void SetCar(double speed, double turn);
void SetBalanceCar(double speedL, double speedR);
Eigen::Vector3d CarX ();
Eigen::Vector3d CarY ();
Eigen::Vector3d CarZ ();
Eigen::Vector3d ToCarCoo (Eigen::Vector3d v);
Eigen::Vector3d ToWorldCoo (Eigen::Vector3d v);

#endif
