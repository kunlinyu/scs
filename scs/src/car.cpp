#include <Eigen/Eigen>
#include <ode/ode.h>
#include <GL/freeglut.h>
#include "car.h"
#include "draw.h"

Car::Car() :
	chassis_(NULL),
	car_direction_(1) {
	battery_position_  = Eigen::Vector3d(0,-0.05,0.02);

	wheel_radius_	= B_WHEEL_RADIUS;
	wheel_front_width_	= B_WHEEL_FRONT_WIDTH;
	wheel_back_width_	= B_WHEEL_BACK_WIDTH;

	CarWidth = ChassisWidth + B_WHEEL_BACK_WIDTH;
  inductance_number = 0;
}

void Car::DestroyObject (ObjectPtr obj) {
	if (!obj) return;
	dBodyDestroy (obj->body);
	dGeomDestroy (obj->geom);
	free (obj);
}

void Car::DestroyCar(){
	DestroyObject(chassis_);
	DestroyObject(wheel_fl_);
	DestroyObject(wheel_fr_);
	DestroyObject(wheel_bl_);
	DestroyObject(wheel_br_);
	DestroyObject(battery_);
}

void Car::MakeCar(double x, double y, dWorldID world, dSpaceID space)  // TODO: move into car
{
	dMass m;
	dQuaternion q;

	//if (chassis_) DestroyCar ();

	if (cartype == electromagnetic) {
		wheel_radius_	= A_WHEEL_RADIUS;	// wheel radius
		wheel_front_width_	= A_WHEEL_FRONT_WIDTH;	// wheel width
		wheel_back_width_	= A_WHEEL_BACK_WIDTH;
		CarWidth	= ChassisWidth + A_WHEEL_BACK_WIDTH;
	}

	// chassis body
	chassis_ = (ObjectPtr)malloc(sizeof(Object));
	chassis_->body = dBodyCreate (world);
	dBodySetPosition (chassis_->body,x,y,STARTZ);
	dMassSetBoxTotal (&m,ChassisMass,ChassisWidth,ChassisLength,ChassisHeight);
	dBodySetMass (chassis_->body,&m);
	chassis_->geom = dCreateBox (space,ChassisWidth,ChassisLength,ChassisHeight);
	dGeomSetBody (chassis_->geom,chassis_->body);

	// battery_ body
	battery_ = (ObjectPtr)malloc(sizeof(Object));
	battery_->body = dBodyCreate (world);
	if (car_direction_) {
    battery_position_ = Eigen::Vector3d(0,0.05,0.02);
  }
  dBodySetPosition(battery_->body, x+battery_position_ .x(), y+battery_position_ .y(), STARTZ+battery_position_ .z());
	dMassSetBoxTotal(&m,battery_Mass,battery_Length,battery_Width,battery_Height*5);
	dBodySetMass(battery_->body,&m);
	battery_->geom = dCreateBox (space,battery_Length,battery_Width,battery_Height);
	dGeomSetBody (battery_->geom,battery_->body);

	// battery_ joint
	dJointID JointChassisbattery_ = dJointCreateFixed(world,0);
	dJointAttach (JointChassisbattery_,chassis_->body,battery_->body);
	dJointSetFixed (JointChassisbattery_);

	// wheel bodies
#define WHEEL(WHICH); 								\
	wheel_##WHICH = (ObjectPtr)malloc(sizeof(Object));			\
	wheel_##WHICH->body = dBodyCreate (world);				\
	dQFromAxisAndAngle (q,0,1,0,M_PI/2.0);					\
	dBodySetQuaternion (wheel_##WHICH->body,q);				\
	dMassSetCylinder (&m,1,1,wheel_radius_,wheel_width);			\
	dMassAdjust (&m,WheelMass);						\
	dBodySetMass (wheel_##WHICH->body,&m);					\
	wheel_##WHICH->geom = dCreateCylinder (space,wheel_radius_,wheel_width);	\
	dGeomSetBody (wheel_##WHICH->geom,wheel_##WHICH->body);

	double wheel_width = wheel_front_width_;
	WHEEL(fl_);
	WHEEL(fr_);
	wheel_width = wheel_back_width_;
	WHEEL(bl_);
	WHEEL(br_);
#undef WHEEL

	if (!car_direction_) {
		dBodySetPosition (wheel_fl_->body, x-0.5*WheelLR, y+WheelFront, STARTZ+0.01);
		dBodySetPosition (wheel_fr_->body, x+0.5*WheelLR, y+WheelFront, STARTZ+0.01);
		dBodySetPosition (wheel_bl_->body, x-0.5*WheelLR, y-WheelBack , STARTZ+0.01);
		dBodySetPosition (wheel_br_->body, x+0.5*WheelLR, y-WheelBack , STARTZ+0.01);
	}
	else {
		dBodySetPosition (wheel_bl_->body, x-0.5*WheelLR, y+WheelBack , STARTZ+0.01);
		dBodySetPosition (wheel_br_->body, x+0.5*WheelLR, y+WheelBack , STARTZ+0.01);
		dBodySetPosition (wheel_fl_->body, x-0.5*WheelLR, y-WheelFront, STARTZ+0.01);
		dBodySetPosition (wheel_fr_->body, x+0.5*WheelLR, y-WheelFront, STARTZ+0.01);
	}


	// front wheel hinge2s
#define JOINT_F(WHICH);										\
	joint_f##WHICH = dJointCreateHinge2 (world,0);						\
	dJointAttach (joint_f##WHICH, chassis_->body, wheel_f##WHICH->body);			\
	const double *pos_F##WHICH = dBodyGetPosition (wheel_f##WHICH->body);			\
	dJointSetHinge2Anchor(joint_f##WHICH,pos_F##WHICH[0],pos_F##WHICH[1],pos_F##WHICH[2]);	\
	dJointSetHinge2Axis1 (joint_f##WHICH,0,-0.1,1);						\
	dJointSetHinge2Axis2 (joint_f##WHICH,1,0,0);						\
	dJointSetHinge2Param (joint_f##WHICH,dParamFMax,FMAX);					\
	dJointSetHinge2Param (joint_f##WHICH,dParamFMax2,0);					\
	dJointSetHinge2Param (joint_f##WHICH,dParamVel,0);					\
	dJointSetHinge2Param (joint_f##WHICH,dParamSuspensionERP,0.8);				\
	dJointSetHinge2Param (joint_f##WHICH,dParamSuspensionCFM,1e-5);				

	JOINT_F(l_);
	JOINT_F(r_);
#undef JOINT_F

	// back wheel hinges
#define JOINT_B(WHICH);										\
	joint_b##WHICH = dJointCreateHinge (world,0);						\
	dJointAttach (joint_b##WHICH, chassis_->body, wheel_b##WHICH->body);			\
	const double *pos_B##WHICH = dBodyGetPosition(wheel_b##WHICH->body);			\
	dJointSetHingeAnchor(joint_b##WHICH,pos_B##WHICH[0],pos_B##WHICH[1],pos_B##WHICH[2]);	\
	dJointSetHingeAxis  (joint_b##WHICH,1,0,0);						\
	dJointSetHingeParam (joint_b##WHICH,dParamFMax,FMAX);					\
	dJointSetHingeParam (joint_b##WHICH,dParamVel,0);					\
	dJointSetHingeParam (joint_b##WHICH,dParamSuspensionERP,0.8);				\
	dJointSetHingeParam (joint_b##WHICH,dParamSuspensionCFM,1e-5);

	JOINT_B(l_);
	JOINT_B(r_);
#undef JOINT_B
}


void Car::MakeBalanceCar (double x, double y, dWorldID world, dSpaceID space) {
	dMass m;
	dQuaternion q;
	battery_position_ = Eigen::Vector3d(0,-0.01,-0.05);

	// chassis body
	chassis_ = (ObjectPtr)malloc(sizeof(Object));
	chassis_->body = dBodyCreate (world);
	dQFromAxisAndAngle (q,1,0,0,M_PI/2.0);
	dBodySetQuaternion (chassis_->body,q);
	dBodySetPosition (chassis_->body,x,y,STARTZ+0.1);
	dMassSetBoxTotal (&m,ChassisMass/2.0,ChassisWidth,ChassisLength,ChassisHeight);
	dBodySetMass (chassis_->body,&m);
	chassis_->geom = dCreateBox (space,ChassisWidth,ChassisLength,ChassisHeight);
	dGeomSetBody (chassis_->geom,chassis_->body);

	// battery_ body
	battery_ = (ObjectPtr)malloc(sizeof(Object));
	battery_->body = dBodyCreate (world);
	dQFromAxisAndAngle (q,1,0,0,M_PI/2.0);
	dBodySetQuaternion (battery_->body,q);
	dBodySetPosition (battery_->body, x+battery_position_.x(), y+battery_position_.y(), STARTZ+battery_position_.z()+0.1);
	dMassSetBoxTotal (&m,battery_Mass,battery_Length,battery_Width,battery_Height*5);
	dBodySetMass (battery_->body,&m);
	battery_->geom = dCreateBox (space,battery_Length,battery_Width,battery_Height);
	dGeomSetBody (battery_->geom,battery_->body);

	// battery_ hinge
	dJointID j = dJointCreateFixed(world,0);
	dJointAttach (j,chassis_->body,battery_->body);
	dJointSetFixed (j);


	// wheel bodies
#define WHEEL(WHICH); 								\
	wheel_##WHICH = (ObjectPtr)malloc(sizeof(Object));			\
	wheel_##WHICH->body = dBodyCreate (world);				\
	dQFromAxisAndAngle (q,0,1,0,M_PI/2.0);					\
	dBodySetQuaternion (wheel_##WHICH->body,q);				\
	dMassSetCylinder (&m,1,1,wheel_radius_,wheel_width);			\
	dMassAdjust (&m,WheelMass);						\
	dBodySetMass (wheel_##WHICH->body,&m);					\
	wheel_##WHICH->geom = dCreateCylinder (space,wheel_radius_,wheel_width);	\
	dGeomSetBody (wheel_##WHICH->geom,wheel_##WHICH->body);

	double wheel_width = wheel_back_width_;
	WHEEL(bl_);
	WHEEL(br_);
#undef WHEEL

	dBodySetPosition (wheel_bl_->body, x-0.5*WheelLR, y, STARTZ+0.1-WheelBack);
	dBodySetPosition (wheel_br_->body, x+0.5*WheelLR, y, STARTZ+0.1-WheelBack);

	// back wheel hinges
#define JOINT_B(WHICH);										\
	joint_b##WHICH = dJointCreateHinge (world,0);						\
	dJointAttach (joint_b##WHICH, chassis_->body, wheel_b##WHICH->body);			\
	const double *pos_B##WHICH = dBodyGetPosition(wheel_b##WHICH->body);			\
	dJointSetHingeAnchor(joint_b##WHICH,pos_B##WHICH[0],pos_B##WHICH[1],pos_B##WHICH[2]);	\
	dJointSetHingeAxis  (joint_b##WHICH,1,0,0);						\
	dJointSetHingeParam (joint_b##WHICH,dParamFMax,FMAX);					\
	dJointSetHingeParam (joint_b##WHICH,dParamVel,0);					\
	dJointSetHingeParam (joint_b##WHICH,dParamSuspensionERP,0.8);				\
	dJointSetHingeParam (joint_b##WHICH,dParamSuspensionCFM,1e-5);

	JOINT_B(l_);
	JOINT_B(r_);
#undef JOINT_B
}

// The definition of matrix between ode & opengl are diferrent.
void Car::TransformMatrix (double matrix[16],Eigen::Vector3d pos, const double R[12]) {
	matrix[0]  = R[0];
	matrix[1]  = R[4];
	matrix[2]  = R[8];
	matrix[3]  = 0;
	matrix[4]  = R[1];
	matrix[5]  = R[5];
	matrix[6]  = R[9];
	matrix[7]  = 0;
	matrix[8]  = R[2];
	matrix[9]  = R[6];
	matrix[10] = R[10];
	matrix[11] = 0;
	matrix[12] = pos.x();
	matrix[13] = pos.y();
	matrix[14] = pos.z();
	matrix[15] = 1;
}

void Car::DrawChassis () {
	static Eigen::Vector3d pp,p,c;
	Eigen::Vector3d pos(dGeomGetPosition (chassis_->geom));
	Eigen::Vector3d speed (dBodyGetLinearVel (chassis_->body));
	const double * R = dGeomGetRotation (chassis_->geom);
	static double r=1.0,l;
	double matrix[16];

	TransformMatrix (matrix,pos,R);
	glPushMatrix ();
	glMultMatrixd (matrix);
	glTranslated (0.0,0.0,ViewPoint.z()/1000.0);

	glScaled (ChassisWidth,ChassisLength,ChassisHeight);
	glColor3f (0.2f,0.2f,0.3f);
	glutSolidCube (1);

	glPopMatrix ();
}

void Car::Drawbattery_ ()
{
	Eigen::Vector3d pos(dGeomGetPosition (battery_->geom));
	const double * R = dGeomGetRotation (battery_->geom);
	GLdouble matrix[16];

	glColor3f (0.6f,0.6f,0.9f);

	TransformMatrix (matrix,pos,R);
	glPushMatrix ();
	glMultMatrixd (matrix);

	glScaled (battery_Length,battery_Width,battery_Height);
	glutSolidCube (1);

	glPopMatrix ();
}


void Car::DrawWheel ()
{
	Eigen::Vector3d pos;
	const double * R;
	double matrix[16];
	Eigen::Vector3d speed;
	double l;

  const double *tmp;
#define WHEEL(WHICH)					\
  tmp = dGeomGetPosition(wheel_##WHICH->geom); \
  pos = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]); \
  tmp = dBodyGetLinearVel(wheel_##WHICH->body); \
  speed = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]); \
		l = speed.norm() / 10.0 * 2.5;								\
		R	= dGeomGetRotation(wheel_##WHICH->geom);\
	TransformMatrix(matrix,pos,R);			\
	glPushMatrix();					\
	glTranslated (0.0,0.0,ViewPoint.z()/1000.0);	\
	glMultMatrixd (matrix);				\
	glTranslated (0.0f,0.0f,-wheel_width/2.0);	\
	glColor3d (0.4f,0.2f,0.2f);			\
	glLineWidth (1);				\
glEnable(GL_POLYGON_OFFSET_FILL);			\
glPolygonOffset(1.0f, 1.0f);				\
	glutSolidCylinder(wheel_radius_,wheel_width,18,4);\
glDisable(GL_POLYGON_OFFSET_FILL);			\
	glColor3f (0.3f,0.1f,0.1f);			\
	glutWireCylinder(wheel_radius_,wheel_width,18,4);\
	glPopMatrix();

	double wheel_width = wheel_front_width_;
	WHEEL(fl_);
	WHEEL(fr_);
	wheel_width = wheel_back_width_;
	WHEEL(bl_);
	WHEEL(br_);
#undef WHEEL
}

void Car::DrawBalanceWheel ()
{
	Eigen::Vector3d pos;
	const double * R;
	double matrix[16];

	glColor3f (0.9f,0.2f,0.2f);

  const double *tmp;
#define WHEEL(WHICH)					\
  tmp = dGeomGetPosition(wheel_##WHICH->geom); \
  pos = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]); \
	R	= dGeomGetRotation(wheel_##WHICH->geom);\
	TransformMatrix(matrix,pos,R);			\
	glPushMatrix();					\
	glTranslated (0.0,0.0,ViewPoint.z()/1000.0);	\
	glMultMatrixd (matrix);				\
	glTranslated(0.0f,0.0f,-wheel_width/2.0);	\
	glColor3f (0.4f,0.2f,0.2f);			\
	glLineWidth (1);				\
glEnable(GL_POLYGON_OFFSET_FILL);			\
glPolygonOffset(1.0f, 1.0f);				\
	glutSolidCylinder(wheel_radius_,wheel_width,18,4);\
glDisable(GL_POLYGON_OFFSET_FILL);			\
	glColor3f (0.3f,0.1f,0.1f);			\
	glutWireCylinder(wheel_radius_,wheel_width,18,4);\
	glPopMatrix();

  double wheel_width = wheel_back_width_;
	WHEEL(bl_);
	WHEEL(br_);
#undef WHEEL
}

void Car::DrawInductance() {
	Eigen::Vector3d pos(dGeomGetPosition(chassis_->geom));
	const double * R = dGeomGetRotation(chassis_->geom);
	double matrix[16];

	TransformMatrix (matrix,pos,R);
	glPushMatrix ();
	glMultMatrixd (matrix);

	for (int i = 0; i < inductance_number; i++) {
		glPushMatrix ();
		glTranslated (InductancePos[i].x(),InductancePos[i].y(),InductancePos[i].z());

		glColor3f (0.9f,0.2f,0.2f);
		glLineWidth (2);
		Magnetic[i] /= 100.0;
		glBegin (GL_LINES);
		glVertex3d (0,0,0);
		glVertex3d (Magnetic[i].x(),Magnetic[i].y(),Magnetic[i].z());
		glEnd ();

		glColor3f (0.1f,0.1f,0.2f);
		glutSolidSphere (0.005f,10,10);
		glPopMatrix ();
	}
	glPopMatrix ();
}

void Car::DrawCar() {
	DrawChassis();
	Drawbattery_();
	DrawWheel();
	DrawInductance();
}

void Car::DrawBalanceCar() {
	DrawChassis();
	Drawbattery_();
	DrawBalanceWheel();
}

Eigen::Vector3d Car::CarX ()
{
	Eigen::Vector3d Left (dBodyGetPosition(wheel_bl_->body));
	Eigen::Vector3d Right(dBodyGetPosition(wheel_br_->body));

	return (Right - Left).normalized();  // TODO(yukunlin): CHECK
}

Eigen::Vector3d Car::CarY ()
{
	if (cartype != balance) {
		Eigen::Vector3d PosF(dBodyGetPosition (wheel_fl_->body));
		Eigen::Vector3d PosB(dBodyGetPosition (wheel_bl_->body));
		Eigen::Vector3d dir = PosF - PosB;
		dir.normalize();
		if (car_direction_) dir *= -1;
		return dir;
	} else {
		Eigen::Vector3d PosL(dBodyGetPosition (wheel_bl_->body));
		Eigen::Vector3d PosR(dBodyGetPosition (wheel_br_->body));
		Eigen::Vector3d PosM(GetPosition());
		Eigen::Vector3d PosB = (PosL + PosR) / 2;
		Eigen::Vector3d Front = PosM - PosB;
		Eigen::Vector3d Y = Front.cross(PosR-PosL);
		Y.normalize();
		return Y;
	}
}

Eigen::Vector3d Car::CarZ ()
{
	return CarX().cross(CarY()).normalized();
}

Eigen::Vector3d Car::ToCarCoo (Eigen::Vector3d v)
{
	Eigen::Vector3d v1;
	v1.x() = v.dot(CarX());
	v1.y() = v.dot(CarY());
	v1.z() = v.dot(CarZ());
	return v1;
}

Eigen::Vector3d Car::ToWorldCoo (Eigen::Vector3d v)
{
	Eigen::Vector3d v1 = v.x()*CarX() + v.y()*CarY() + v.z()*CarZ();
	return v1;
}

double Car::GetASpeedL() {
	Eigen::Vector3d vl(dBodyGetAngularVel(wheel_bl_->body));
	Eigen::Vector3d pl(dBodyGetPosition  (wheel_bl_->body));
	Eigen::Vector3d pr(dBodyGetPosition  (wheel_br_->body));
	Eigen::Vector3d v = pl - pr;
	return vl.dot(v) / v.norm();
}

double Car::GetASpeedR() {
	Eigen::Vector3d vr(dBodyGetAngularVel(wheel_br_->body));
	Eigen::Vector3d pl(dBodyGetPosition  (wheel_bl_->body));
	Eigen::Vector3d pr(dBodyGetPosition  (wheel_br_->body));
	Eigen::Vector3d v = pl - pr;
	return vr.dot(v) / v.norm();
}

double Car::GetASpeed() { return (GetASpeedL() + GetASpeedR()) / 2.0; }
double Car::GetSpeedL() { return GetASpeedL() * wheel_radius_; }
double Car::GetSpeedR() { return GetASpeedR() * wheel_radius_; }
double Car::GetSpeed() { return GetASpeed() * wheel_radius_; }

void Car::SetTorque(double l, double r) {
	dJointAddHingeTorque(joint_bl_, l);
	dJointAddHingeTorque(joint_br_, r);
}

void Car::SetServo(double dir) {
	double curturn_L = dJointGetHinge2Angle1(joint_fl_);
	double curturn_R = dJointGetHinge2Angle1(joint_fr_);
	dJointSetHinge2Param (joint_fl_, dParamVel, (dir / 400 * M_PI-curturn_L) * 10);
	dJointSetHinge2Param (joint_fr_, dParamVel, (dir / 400 * M_PI-curturn_R) * 10);
}





