#ifndef _CAR_H_
#define _CAR_H_

#include <Eigen/Eigen>
#include <ode/common.h>
#include "track.h"

#define A_WHEEL_RADIUS 0.025       // wheel radius
#define A_WHEEL_FRONT_WIDTH 0.025  // wheel width
#define A_WHEEL_BACK_WIDTH 0.025   // wheel width

#define B_WHEEL_RADIUS 0.03        // wheel radius
#define B_WHEEL_FRONT_WIDTH 0.025  // wheel width
#define B_WHEEL_BACK_WIDTH 0.04    // wheel width

#define INDUCTANCE_NUM_MAX 128     // max number of inductance

#define STARTZ (TRACK_HEIGHT + 0.05)  // starting height of chassis
#define FMAX 1.0                      // car engine fmax

typedef struct Object {
 dBodyID body;
 dGeomID geom;
} * ObjectPtr;

class Car {
 public:
  Car();
  void MakeCar(double x, double y, dWorldID world, dSpaceID space);
  void MakeBalanceCar(double x, double y, dWorldID world, dSpaceID space);
  void DrawCar();
  void DrawBalanceCar();  // TODO: merge to DrawCar
  void SetCar(double speed, double turn);  // TODO: implement
  void SetBalanceCar(double speedL, double speedR);  // TODO: implement

  Eigen::Vector3d CarX();
  Eigen::Vector3d CarY();
  Eigen::Vector3d CarZ();
  Eigen::Vector3d ToCarCoo(Eigen::Vector3d v);  // TODO: use Isometry
  Eigen::Vector3d ToWorldCoo(Eigen::Vector3d v);

  double GetASpeedL();
  double GetASpeedR();
  double GetASpeed();
  double GetSpeedL();
  double GetSpeedR();
  double GetSpeed();

  void SetTorque(double l, double r);
  void SetServo(double dir);

  Eigen::Vector3d GetPosition() { return Eigen::Vector3d(dBodyGetPosition(chassis_->body)); }
  Eigen::Vector3d GetAngularVel() { return Eigen::Vector3d(dBodyGetAngularVel(chassis_->body)); }
  Eigen::Vector3d GetLinearVel() { return Eigen::Vector3d(dBodyGetLinearVel(chassis_->body)); }

  bool GetCarDirection() { return car_direction_; }
  void SetCarDirection(bool dir) { car_direction_ = dir; }

  int InductanceNumber() { return inductance_number; }
  void SetInductanceNumber(int n) { inductance_number = n; }  // TODO remove

  double GetCarWidth() { return CarWidth; }
  void SetBattery(Eigen::Vector3d pos) { battery_position_ = pos; }

 private:
  void TransformMatrix(double matrix[16],Eigen::Vector3d pos, const double R[12]);
  void DrawChassis();
  void Drawbattery_();
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

  static constexpr double battery_Length = 0.13;
  static constexpr double battery_Width = 0.04;
  static constexpr double battery_Height = 0.02;
  static constexpr double battery_Mass = 0.3;

  ObjectPtr wheel_fl_;
  ObjectPtr wheel_fr_;
  ObjectPtr wheel_bl_;
  ObjectPtr wheel_br_;
  ObjectPtr chassis_;
  ObjectPtr battery_;

  // Joint between wheels and chassis
  dJointID joint_fl_;
  dJointID joint_fr_;
  dJointID joint_bl_;
  dJointID joint_br_;

  bool car_direction_;

  Eigen::Vector3d battery_position_;

  double wheel_radius_;
  double wheel_front_width_;
  double wheel_back_width_;

  double CarWidth;

  Eigen::Vector3d InductancePos [INDUCTANCE_NUM_MAX]; // save every inductance's position
  Eigen::Vector3d Magnetic[INDUCTANCE_NUM_MAX]; // save the magnetic of every inductance
  int inductance_number;  // inductance pointer
};

#endif
