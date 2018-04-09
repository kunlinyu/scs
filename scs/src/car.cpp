#include <Eigen/Eigen>
#include <ode/ode.h>
#include <GL/freeglut.h>
#include "car.h"
#include "draw.h"

Car::Car() :
	Chassis(NULL),
	CarReverseFlag(0),
	CarDirection(1) {
	BatteryPos  = Eigen::Vector3d(0,-0.05,0.02);
	BatteryPosR = Eigen::Vector3d(0,0.05,0.02);
	BatteryPosB = Eigen::Vector3d(0,-0.01,-0.05);

	WheelRadius	= B_WHEEL_RADIUS;
	WheelFrontWidth	= B_WHEEL_FRONT_WIDTH;
	WheelBackWidth	= B_WHEEL_BACK_WIDTH;
	WheelWidth = B_WHEEL_FRONT_WIDTH;

	CarWidth = 0.11 + B_WHEEL_BACK_WIDTH;
  inductance_number = 0;
}

void Car::DestroyObject (ObjectPtr obj) {
	if (!obj) return;
	dBodyDestroy (obj->body);
	dGeomDestroy (obj->geom);
	free (obj);
}

void Car::DestroyCar(){
	DestroyObject(Chassis);
	DestroyObject(Wheel_FL);
	DestroyObject(Wheel_FR);
	DestroyObject(Wheel_BL);
	DestroyObject(Wheel_BR);
	DestroyObject(Battery);
}

void Car::MakeCar(double x, double y, dWorldID world, dSpaceID space)
{
	dMass m;
	dQuaternion q;

	//if (Chassis) DestroyCar ();

	if (cartype == electromagnetic) {
		WheelRadius	= A_WHEEL_RADIUS;	// wheel radius
		WheelFrontWidth	= A_WHEEL_FRONT_WIDTH;	// wheel width
		WheelBackWidth	= A_WHEEL_BACK_WIDTH;
		WheelWidth	= A_WHEEL_FRONT_WIDTH;
		CarWidth	= ChassisWidth + A_WHEEL_BACK_WIDTH;
	}

	// chassis body
	Chassis = (ObjectPtr)malloc(sizeof(Object));
	Chassis->body = dBodyCreate (world);
	dBodySetPosition (Chassis->body,x,y,STARTZ);
	dMassSetBoxTotal (&m,ChassisMass,ChassisWidth,ChassisLength,ChassisHeight);
	dBodySetMass (Chassis->body,&m);
	Chassis->geom = dCreateBox (space,ChassisWidth,ChassisLength,ChassisHeight);
	dGeomSetBody (Chassis->geom,Chassis->body);

	// Battery body
	Battery = (ObjectPtr)malloc(sizeof(Object));
	Battery->body = dBodyCreate (world);
	if (!CarDirection)
		dBodySetPosition (Battery->body, x+BatteryPos .x(), y+BatteryPos .y(), STARTZ+BatteryPos .z());
	else	dBodySetPosition (Battery->body, x+BatteryPosR.x(), y+BatteryPosR.y(), STARTZ+BatteryPosR.z());
	dMassSetBoxTotal (&m,BatteryMass,BatteryLength,BatteryWidth,BatteryHeight*5);
	dBodySetMass (Battery->body,&m);
	Battery->geom = dCreateBox (space,BatteryLength,BatteryWidth,BatteryHeight);
	dGeomSetBody (Battery->geom,Battery->body);

	// Battery joint
	dJointID JointChassisBattery = dJointCreateFixed(world,0);
	dJointAttach (JointChassisBattery,Chassis->body,Battery->body);
	dJointSetFixed (JointChassisBattery);

	// wheel bodies
#define WHEEL(WHICH); 								\
	Wheel_##WHICH = (ObjectPtr)malloc(sizeof(Object));			\
	Wheel_##WHICH->body = dBodyCreate (world);				\
	dQFromAxisAndAngle (q,0,1,0,M_PI/2.0);					\
	dBodySetQuaternion (Wheel_##WHICH->body,q);				\
	dMassSetCylinder (&m,1,1,WheelRadius,WheelWidth);			\
	dMassAdjust (&m,WheelMass);						\
	dBodySetMass (Wheel_##WHICH->body,&m);					\
	Wheel_##WHICH->geom = dCreateCylinder (space,WheelRadius,WheelWidth);	\
	dGeomSetBody (Wheel_##WHICH->geom,Wheel_##WHICH->body);

	WheelWidth = WheelFrontWidth;
	WHEEL(FL);
	WHEEL(FR);
	WheelWidth = WheelBackWidth;
	WHEEL(BL);
	WHEEL(BR);
#undef WHEEL

	if (!CarDirection) {
		dBodySetPosition (Wheel_FL->body, x-0.5*WheelLR, y+WheelFront, STARTZ+0.01);
		dBodySetPosition (Wheel_FR->body, x+0.5*WheelLR, y+WheelFront, STARTZ+0.01);
		dBodySetPosition (Wheel_BL->body, x-0.5*WheelLR, y-WheelBack , STARTZ+0.01);
		dBodySetPosition (Wheel_BR->body, x+0.5*WheelLR, y-WheelBack , STARTZ+0.01);
	}
	else {
		dBodySetPosition (Wheel_BL->body, x-0.5*WheelLR, y+WheelBack , STARTZ+0.01);
		dBodySetPosition (Wheel_BR->body, x+0.5*WheelLR, y+WheelBack , STARTZ+0.01);
		dBodySetPosition (Wheel_FL->body, x-0.5*WheelLR, y-WheelFront, STARTZ+0.01);
		dBodySetPosition (Wheel_FR->body, x+0.5*WheelLR, y-WheelFront, STARTZ+0.01);
	}


	// front wheel hinge2s
#define JOINT_F(WHICH);										\
	Joint_F##WHICH = dJointCreateHinge2 (world,0);						\
	dJointAttach (Joint_F##WHICH, Chassis->body, Wheel_F##WHICH->body);			\
	const double *pos_F##WHICH = dBodyGetPosition (Wheel_F##WHICH->body);			\
	dJointSetHinge2Anchor(Joint_F##WHICH,pos_F##WHICH[0],pos_F##WHICH[1],pos_F##WHICH[2]);	\
	dJointSetHinge2Axis1 (Joint_F##WHICH,0,-0.1,1);						\
	dJointSetHinge2Axis2 (Joint_F##WHICH,1,0,0);						\
	dJointSetHinge2Param (Joint_F##WHICH,dParamFMax,FMAX);					\
	dJointSetHinge2Param (Joint_F##WHICH,dParamFMax2,0);					\
	dJointSetHinge2Param (Joint_F##WHICH,dParamVel,0);					\
	dJointSetHinge2Param (Joint_F##WHICH,dParamSuspensionERP,0.8);				\
	dJointSetHinge2Param (Joint_F##WHICH,dParamSuspensionCFM,1e-5);				

	JOINT_F(L);
	JOINT_F(R);
#undef JOINT_F

	// back wheel hinges
#define JOINT_B(WHICH);										\
	Joint_B##WHICH = dJointCreateHinge (world,0);						\
	dJointAttach (Joint_B##WHICH, Chassis->body, Wheel_B##WHICH->body);			\
	const double *pos_B##WHICH = dBodyGetPosition(Wheel_B##WHICH->body);			\
	dJointSetHingeAnchor(Joint_B##WHICH,pos_B##WHICH[0],pos_B##WHICH[1],pos_B##WHICH[2]);	\
	dJointSetHingeAxis  (Joint_B##WHICH,1,0,0);						\
	dJointSetHingeParam (Joint_B##WHICH,dParamFMax,FMAX);					\
	dJointSetHingeParam (Joint_B##WHICH,dParamVel,0);					\
	dJointSetHingeParam (Joint_B##WHICH,dParamSuspensionERP,0.8);				\
	dJointSetHingeParam (Joint_B##WHICH,dParamSuspensionCFM,1e-5);

	JOINT_B(L);
	JOINT_B(R);
#undef JOINT_B
}


void Car::MakeBalanceCar (double x, double y, dWorldID world, dSpaceID space) {
	dMass m;
	dQuaternion q;

	// chassis body
	Chassis = (ObjectPtr)malloc(sizeof(Object));
	Chassis->body = dBodyCreate (world);
	dQFromAxisAndAngle (q,1,0,0,M_PI/2.0);
	dBodySetQuaternion (Chassis->body,q);
	dBodySetPosition (Chassis->body,x,y,STARTZ+0.1);
	dMassSetBoxTotal (&m,ChassisMass/2.0,ChassisWidth,ChassisLength,ChassisHeight);
	dBodySetMass (Chassis->body,&m);
	Chassis->geom = dCreateBox (space,ChassisWidth,ChassisLength,ChassisHeight);
	dGeomSetBody (Chassis->geom,Chassis->body);

	// Battery body
	Battery = (ObjectPtr)malloc(sizeof(Object));
	Battery->body = dBodyCreate (world);
	dQFromAxisAndAngle (q,1,0,0,M_PI/2.0);
	dBodySetQuaternion (Battery->body,q);
	dBodySetPosition (Battery->body, x+BatteryPosB.x(), y+BatteryPosB.y(), STARTZ+BatteryPosB.z()+0.1);
	dMassSetBoxTotal (&m,BatteryMass,BatteryLength,BatteryWidth,BatteryHeight*5);
	dBodySetMass (Battery->body,&m);
	Battery->geom = dCreateBox (space,BatteryLength,BatteryWidth,BatteryHeight);
	dGeomSetBody (Battery->geom,Battery->body);

	// Battery hinge
	dJointID j = dJointCreateFixed(world,0);
	dJointAttach (j,Chassis->body,Battery->body);
	dJointSetFixed (j);


	// wheel bodies
#define WHEEL(WHICH); 								\
	Wheel_##WHICH = (ObjectPtr)malloc(sizeof(Object));			\
	Wheel_##WHICH->body = dBodyCreate (world);				\
	dQFromAxisAndAngle (q,0,1,0,M_PI/2.0);					\
	dBodySetQuaternion (Wheel_##WHICH->body,q);				\
	dMassSetCylinder (&m,1,1,WheelRadius,WheelWidth);			\
	dMassAdjust (&m,WheelMass);						\
	dBodySetMass (Wheel_##WHICH->body,&m);					\
	Wheel_##WHICH->geom = dCreateCylinder (space,WheelRadius,WheelWidth);	\
	dGeomSetBody (Wheel_##WHICH->geom,Wheel_##WHICH->body);

	WheelWidth = WheelBackWidth;
	WHEEL(BL);
	WHEEL(BR);
#undef WHEEL

	dBodySetPosition (Wheel_BL->body, x-0.5*WheelLR, y, STARTZ+0.1-WheelBack);
	dBodySetPosition (Wheel_BR->body, x+0.5*WheelLR, y, STARTZ+0.1-WheelBack);

	// back wheel hinges
#define JOINT_B(WHICH);										\
	Joint_B##WHICH = dJointCreateHinge (world,0);						\
	dJointAttach (Joint_B##WHICH, Chassis->body, Wheel_B##WHICH->body);			\
	const double *pos_B##WHICH = dBodyGetPosition(Wheel_B##WHICH->body);			\
	dJointSetHingeAnchor(Joint_B##WHICH,pos_B##WHICH[0],pos_B##WHICH[1],pos_B##WHICH[2]);	\
	dJointSetHingeAxis  (Joint_B##WHICH,1,0,0);						\
	dJointSetHingeParam (Joint_B##WHICH,dParamFMax,FMAX);					\
	dJointSetHingeParam (Joint_B##WHICH,dParamVel,0);					\
	dJointSetHingeParam (Joint_B##WHICH,dParamSuspensionERP,0.8);				\
	dJointSetHingeParam (Joint_B##WHICH,dParamSuspensionCFM,1e-5);

	JOINT_B(L);
	JOINT_B(R);
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
	Eigen::Vector3d pos(dGeomGetPosition (Chassis->geom));
	Eigen::Vector3d speed (dBodyGetLinearVel (Chassis->body));
	const double * R = dGeomGetRotation (Chassis->geom);
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

void Car::DrawBattery ()
{
	Eigen::Vector3d pos(dGeomGetPosition (Battery->geom));
	const double * R = dGeomGetRotation (Battery->geom);
	GLdouble matrix[16];

	glColor3f (0.6f,0.6f,0.9f);

	TransformMatrix (matrix,pos,R);
	glPushMatrix ();
	glMultMatrixd (matrix);

	glScaled (BatteryLength,BatteryWidth,BatteryHeight);
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
  tmp = dGeomGetPosition(Wheel_##WHICH->geom); \
  pos = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]); \
  tmp = dBodyGetLinearVel(Wheel_##WHICH->body); \
  speed = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]); \
		l = speed.norm() / 10.0 * 2.5;								\
		R	= dGeomGetRotation(Wheel_##WHICH->geom);\
	TransformMatrix(matrix,pos,R);			\
	glPushMatrix();					\
	glTranslated (0.0,0.0,ViewPoint.z()/1000.0);	\
	glMultMatrixd (matrix);				\
	glTranslated (0.0f,0.0f,-WheelWidth/2.0);	\
	glColor3d (0.4f,0.2f,0.2f);			\
	glLineWidth (1);				\
glEnable(GL_POLYGON_OFFSET_FILL);			\
glPolygonOffset(1.0f, 1.0f);				\
	glutSolidCylinder(WheelRadius,WheelWidth,18,4);\
glDisable(GL_POLYGON_OFFSET_FILL);			\
	glColor3f (0.3f,0.1f,0.1f);			\
	glutWireCylinder(WheelRadius,WheelWidth,18,4);\
	glPopMatrix();

	WheelWidth = WheelFrontWidth;
	WHEEL(FL);
	WHEEL(FR);
	WheelWidth = WheelBackWidth;
	WHEEL(BL);
	WHEEL(BR);
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
  tmp = dGeomGetPosition(Wheel_##WHICH->geom); \
  pos = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]); \
	R	= dGeomGetRotation(Wheel_##WHICH->geom);\
	TransformMatrix(matrix,pos,R);			\
	glPushMatrix();					\
	glTranslated (0.0,0.0,ViewPoint.z()/1000.0);	\
	glMultMatrixd (matrix);				\
	glTranslated(0.0f,0.0f,-WheelWidth/2.0);	\
	glColor3f (0.4f,0.2f,0.2f);			\
	glLineWidth (1);				\
glEnable(GL_POLYGON_OFFSET_FILL);			\
glPolygonOffset(1.0f, 1.0f);				\
	glutSolidCylinder(WheelRadius,WheelWidth,18,4);\
glDisable(GL_POLYGON_OFFSET_FILL);			\
	glColor3f (0.3f,0.1f,0.1f);			\
	glutWireCylinder(WheelRadius,WheelWidth,18,4);\
	glPopMatrix();

	WHEEL(BL);
	WHEEL(BR);
#undef WHEEL
}

void Car::DrawInductance() {
	Eigen::Vector3d pos(dGeomGetPosition (Chassis->geom));
	const double * R = dGeomGetRotation (Chassis->geom);
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
	DrawChassis ();
	DrawBattery ();
	DrawWheel ();
	DrawInductance ();
}

void Car::DrawBalanceCar ()
{
	DrawChassis ();
	DrawBattery ();
	DrawBalanceWheel ();
}

Eigen::Vector3d Car::CarX ()
{
	Eigen::Vector3d Left (dBodyGetPosition(Wheel_BL->body));
	Eigen::Vector3d Right(dBodyGetPosition(Wheel_BR->body));

	return (Right - Left).normalized();  // TODO(yukunlin): CHECK
}

Eigen::Vector3d Car::CarY ()
{
	if (cartype != balance) {
		Eigen::Vector3d PosF(dBodyGetPosition (Wheel_FL->body));
		Eigen::Vector3d PosB(dBodyGetPosition (Wheel_BL->body));
		Eigen::Vector3d dir = PosF - PosB;
		dir.normalize();
		if (CarDirection) dir *= -1;
		return dir;
	} else {
		Eigen::Vector3d PosL(dBodyGetPosition (Wheel_BL->body));
		Eigen::Vector3d PosR(dBodyGetPosition (Wheel_BR->body));
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
	Eigen::Vector3d vl(dBodyGetAngularVel(Wheel_BL->body));
	Eigen::Vector3d pl(dBodyGetPosition  (Wheel_BL->body));
	Eigen::Vector3d pr(dBodyGetPosition  (Wheel_BR->body));
	Eigen::Vector3d v = pl - pr;
	return vl.dot(v) / v.norm();
}

double Car::GetASpeedR() {
	Eigen::Vector3d vr(dBodyGetAngularVel(Wheel_BR->body));
	Eigen::Vector3d pl(dBodyGetPosition  (Wheel_BL->body));
	Eigen::Vector3d pr(dBodyGetPosition  (Wheel_BR->body));
	Eigen::Vector3d v = pl - pr;
	return vr.dot(v) / v.norm();
}

double Car::GetASpeed() { return (GetASpeedL() + GetASpeedR()) / 2.0; }
double Car::GetSpeedL() { return GetASpeedL() * WheelRadius; }
double Car::GetSpeedR() { return GetASpeedR() * WheelRadius; }
double Car::GetSpeed() { return GetASpeed() * WheelRadius; }
