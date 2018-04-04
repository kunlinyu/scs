#ifndef _DRAW_H_
#define _DRAW_H_

#include <Eigen/Eigen>

#define CUSTOMWINDOWNUM 16

typedef void (*pFunc) ();

enum ViewType {
	over,
	car,
	bird,
	birdf,
	hard,
	back,
	overf,
	soft,
	mouse,
};

enum DriveMode {
	play,ai,debug,
};

enum CarType {
	camera,electromagnetic,balance,
};

extern CarType cartype;
extern int RouteFlag;
extern int CustomWindowFlag;
extern int CustomWindowNum;
extern int PathFlag;
extern int WinCustom[CUSTOMWINDOWNUM];
extern int WinCar;
extern int WinGod;
extern Eigen::Vector3d ViewPoint;
extern Eigen::Vector3d CameraPos;
extern double DepressionAngle;
extern ViewType viewtype;
extern DriveMode drivemode;
extern double VirtualSpeed;
extern double distance;
extern double pitch;
extern double height;
extern double h;
extern double hpr[3];

void ClearViewVar ();
void DrawInit();

#endif
