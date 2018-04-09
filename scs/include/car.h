#ifndef _CAR_H_
#define _CAR_H_

#include <Eigen/Eigen>
#include <ode/common.h>
#include "track.h"

#define A_WHEEL_RADIUS 0.025  // wheel radius
#define A_WHEEL_FRONT_WIDTH 0.025  // wheel width
#define A_WHEEL_BACK_WIDTH 0.025  // wheel width

#define B_WHEEL_RADIUS 0.03  // wheel radius
#define B_WHEEL_FRONT_WIDTH 0.025  // wheel width
#define B_WHEEL_BACK_WIDTH 0.04  // wheel width

#define INDUCTANCE_NUM_MAX 128  // max number of inductance

#define STARTZ (TRACK_HEIGHT + 0.05)  // starting height of chassis
#define FMAX 1.0  // car engine fmax

typedef struct Object {
 dBodyID body;
 dGeomID geom;
} * ObjectPtr;

class Car {
 public:
  Car();
  void MakeCar(double x, double y, dWorldID world, dSpaceID space);
  void ResetCar();
  void MakeBalanceCar(double x, double y, dWorldID world, dSpaceID space);
  void DrawCar();
  void DrawBalanceCar();
  void SetCar(double speed, double turn);  // TODO: implement
  void SetBalanceCar(double speedL, double speedR);  // TODO: implement

  Eigen::Vector3d CarX();
  Eigen::Vector3d CarY();
  Eigen::Vector3d CarZ();
  Eigen::Vector3d ToCarCoo(Eigen::Vector3d v);
  Eigen::Vector3d ToWorldCoo(Eigen::Vector3d v);

  double GetASpeedL();
  double GetASpeedR();
  double GetASpeed();
  double GetSpeedL();
  double GetSpeedR();
  double GetSpeed();
  void SetTorque(double l, double r);

  Eigen::Vector3d GetPosition() { return Eigen::Vector3d(dBodyGetPosition(Chassis->body)); }
  Eigen::Vector3d GetAngularVel() { return Eigen::Vector3d(dBodyGetAngularVel(Chassis->body)); }
  Eigen::Vector3d GetLinearVel() { return Eigen::Vector3d(dBodyGetLinearVel(Chassis->body)); }
  dJointID GetJointBL() { return Joint_BL; }
  dJointID GetJointBR() { return Joint_BR; }
  dJointID GetJointFL() { return Joint_FL; }
  dJointID GetJointFR() { return Joint_FR; }
  ObjectPtr GetWheelBL() { return Wheel_BL; }
  ObjectPtr GetWheelBR() { return Wheel_BR; }
  ObjectPtr GetWheelFL() { return Wheel_FL; }
  ObjectPtr GetWheelFR() { return Wheel_FR; }
  bool GetCarReverseFlag() { return CarReverseFlag; }
  void SetCarReverseFlag(double flag) { CarReverseFlag = flag; }
  bool GetCarDirection() { return CarDirection; }
  void SetCarDirection(bool dir) { CarDirection = dir; }
  int InductanceNumber() { return inductance_number; }
  void SetInductanceNumber(int n) { inductance_number = n; }  // TODO remove
  double GetCarWidth() { return CarWidth; }
  double GetWheelRadius() { return WheelRadius; }
  void SetBatteryPos(Eigen::Vector3d pos) {
    BatteryPos = pos;
    BatteryPosR = pos;
    BatteryPosB = pos;
  }

 private:
  void TransformMatrix(double matrix[16],Eigen::Vector3d pos, const double R[12]);
  void DrawChassis();
  void DrawBattery();
  void DrawWheel();
  void DrawBalanceWheel();
  void DrawInductance();
  void DestroyObject (ObjectPtr obj);
  void DestroyCar();

 private:
  static constexpr double ChassisLength = 0.26;  // chassis length
  static constexpr double ChassisWidth = 0.11;  // chassis width
  static constexpr double ChassisHeight = 0.005; // chassis height
  static constexpr double ChassisMass = 0.5;  // chassis mass

  static constexpr double WheelLR  = 0.135; // Distance between left and right wheel
  static constexpr double WheelFront = 0.08;  // Distance between front wheel and middle
  static constexpr double WheelBack = 0.12;  // Distance between back wheel and middle
  static constexpr double WheelMass = 0.03;  // wheel mass

  static constexpr double BatteryLength = 0.13;
  static constexpr double BatteryWidth = 0.04;
  static constexpr double BatteryHeight = 0.02;
  static constexpr double BatteryMass = 0.3;

  ObjectPtr Wheel_FL;
  ObjectPtr Wheel_FR;
  ObjectPtr Wheel_BL;
  ObjectPtr Wheel_BR;
  ObjectPtr Chassis;
  ObjectPtr Battery;

  // Joint between wheels and chassis
  dJointID Joint_FL;
  dJointID Joint_FR;
  dJointID Joint_BL;
  dJointID Joint_BR;

  bool CarReverseFlag;
  bool CarDirection;

  Eigen::Vector3d BatteryPos;
  Eigen::Vector3d BatteryPosR;
  Eigen::Vector3d BatteryPosB;

  double WheelRadius;
  double WheelFrontWidth;
  double WheelBackWidth;
  double WheelWidth;

  double CarWidth;

  Eigen::Vector3d InductancePos [INDUCTANCE_NUM_MAX]; // save every inductance's position
  Eigen::Vector3d Magnetic[INDUCTANCE_NUM_MAX]; // save the magnetic of every inductance
  int inductance_number;  // inductance pointer
};


#endif
