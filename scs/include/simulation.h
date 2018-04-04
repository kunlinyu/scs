#ifndef _SIMULATION_H_
#define _SIMULATION_H_ 


#include <Eigen/Eigen>
#include <ode/common.h>

extern dWorldID	world;
extern dSpaceID	space;
extern dJointGroupID	contactgroup;

extern int MotorDutyL;
extern int MotorDutyR;
extern int ServoDir;
extern int MotorDutyL;
extern int MotorDutyR;
extern double AngularSpeedL;
extern double AngularSpeedR;
extern int ServoDir;
extern double Mu;
extern double Slip;
extern double CurrentStepTime;

void ResetSimulation();
void DestroySimulation();
void step(double stepsize);

#endif
