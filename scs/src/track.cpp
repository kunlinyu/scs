#include <math.h>
#include <malloc.h>
#include <string.h>

#include <Eigen/Eigen>
#include <GL/glut.h>
#include <ode/ode.h>

#include "track.h"
#include "common.h"
#include "car.h"
#include "draw.h"
#include "simulation.h"

#define TYPE_BARRIER 0x01
#define TYPE_DASH    0x02
#define TYPE_SLOP    0x04


Track::Track() {
  for (int i = 0; i < MAXSEGMENT; i++) {
    type[i] = 0;
  }
  TriNum = 0;
  PointNum = 0;
  TotalLength = 0.0;
  PathSecurity = 0.0;
  MiddleLineFlag = 0;
  TrackReverseFlag = 0;
  ElevationAngle = ELEVATION_ANGLE;
}

void Track::ReadTrack (const char * fname) {
  FILE *fin;
  char line[256];
  char s[256] = {0};
  char c = 'A';

  if (strlen(fname)<1) {
    printf ("\tTRACK: There is no track file.\n");
    getchar ();
    exit (1);
  }
  fin = fopen(fname,"r");
  if (!fin) {
    printf ("\tTRACK: Can not open track file \"%s\".\n",fname);
    getchar ();
    exit (1);
  }

  printf ("\tTRACK: Reading track file \"%s\".\n",fname);
  fgets (line,255,fin);
  sscanf(line,"%lf%c",&EndLineDistance,&c);

  EndLineDistance /= 100.0;
  printf("\tTRACK: EndLineDistance: %5.2f.\n",EndLineDistance);
  int i = 0;
  do {
    if (fgets (line,255,fin)==NULL) {i++; break;}
    if (strlen(line)<3) continue;
    if (i>=MAXSEGMENT) {
      printf("\tTRACK: The Track is too long to draw.\n");
      break;
    }
    memset (s,0,sizeof(s));
    sscanf (line,"%lf%lf%s",&track[i*2],&track[i*2+1],s);
    if (strlen(s)>1 && s[0]=='/' && s[1]=='/');
    else {
      for (int j=0; j<(int)strlen(s); j++) {
        switch (s[j]) {
          case '+': type[i] |= TYPE_BARRIER;	break;
          case '.': type[i] |= TYPE_DASH;	break;
          case '^': type[i] |= TYPE_SLOP;	break;
          default : printf("TRACK: Unrecognized track type \"%c\".\n",s[j]);
        }
      }
    }
    i++;
  } while (track[(i-1)*2]!=0);
  Segment = i-1;
  fclose(fin);
}

void Track::Reverse() {
  for (int i=0; i<Segment; i++)
    track[i*2+1] *= -1;
  for (int i=0; i<Segment/2; i++) {
    double t = track[i*2];
    track[i*2] = track[(Segment-i-1)*2];
    track[(Segment-i-1)*2] = t;
    t = track[i*2+1];
    track[i*2+1] = track[(Segment-i-1)*2+1];
    track[(Segment-i-1)*2+1] = t;
    unsigned char c = type[i];
    type[i] = type[Segment-i-1];
    type[Segment-i-1] = c;
  }
  EndLineDistance *= -1.0;
}

void Track::CalcPoint_OUT() {
  Eigen::Vector3d dir(0,1,0);	// current direction
  TotalLength = 0.0;
  if (TrackReverseFlag) {
    LftOut[0] = Eigen::Vector3d(-TRACK_WIDTH_OUT/2.0,2.0,TRACK_HEIGHT);
    RgtOut[0] = Eigen::Vector3d( TRACK_WIDTH_OUT/2.0,2.0,TRACK_HEIGHT);
  }
  else {
    LftOut[0] = Eigen::Vector3d (-TRACK_WIDTH_OUT/2.0,0.0,TRACK_HEIGHT);
    RgtOut[0] = Eigen::Vector3d ( TRACK_WIDTH_OUT/2.0,0.0,TRACK_HEIGHT);
  }
  DashLO[0] = LftOut[0];
  DashRO[0] = RgtOut[0];

  double SlopDir = 1.0;
  int p = 1;	// pointer
  int dp = 0;	// dash pointer
  if (cartype == balance)	ElevationAngle = ELEVATION_ANGLE_B;

  for (int i=0; i<Segment; i++) {		// Calc for each segment
    double length = track[i*2]/100.0;
    double degree = track[i*2];
    double R = track[i*2+1]/100.0;
    if (fabs(R) < TRACK_WIDTH_OUT/2.0) {	// straight
      if ((type[i]&TYPE_BARRIER) >0) {	// barrier
        Eigen::Vector3d up(0,0,BARRIER_HEIGHT);
        LftOut[p] = LftOut[p-1] + up + dir*BARRIER_HEIGHT;	// move up
        RgtOut[p] = RgtOut[p-1] + up + dir*BARRIER_HEIGHT;
        p ++;
        length -= 2*BARRIER_HEIGHT;
        if (length<0.0) length = 0.0;
        TotalLength += BARRIER_HEIGHT;
      }
      if ((type[i] & TYPE_SLOP) >0) {
        dir.z() = SlopDir * atan(ElevationAngle*DEG_TO_RAD/2.0);
        LftOut[p] = LftOut[p-1] + TRANSITION_LENGTH * dir;
        RgtOut[p] = RgtOut[p-1] + TRANSITION_LENGTH * dir;
        p ++;
        length -= 2*TRANSITION_LENGTH;
        if (length<0.0) length = 0.0;
        TotalLength += TRANSITION_LENGTH;
        dir.z() = SlopDir * atan(ElevationAngle*DEG_TO_RAD);
      } else {
        dir.z() = 0.0;
      }
      LftOut[p] = LftOut[p-1] + length * dir;	// move forward
      RgtOut[p] = RgtOut[p-1] + length * dir;
      TotalLength += length;
      if ((type[i]&TYPE_DASH)>0) {
        DashLO[dp] = LftOut[p];
        DashRO[dp] = RgtOut[p];
        dp ++;
      }
      p ++;
      if ((type[i]&TYPE_BARRIER) >0) {
        Eigen::Vector3d down(0,0,-BARRIER_HEIGHT);
        LftOut[p] = LftOut[p-1] + down + dir*BARRIER_HEIGHT;	// move up
        RgtOut[p] = RgtOut[p-1] + down + dir*BARRIER_HEIGHT;
        p ++;
        TotalLength += BARRIER_HEIGHT;
      }
      if ((type[i] & TYPE_SLOP) >0) {
        dir.z() = SlopDir * atan(ElevationAngle*DEG_TO_RAD/2.0);
        LftOut[p] = LftOut[p-1] + TRANSITION_LENGTH * dir;
        RgtOut[p] = RgtOut[p-1] + TRANSITION_LENGTH * dir;
        p ++;
        TotalLength += TRANSITION_LENGTH;
        SlopDir *= -1.0;
      } else {
        dir.z() = 0.0;
      }
      if (p>=MAXPOINT) {
        printf("\tTRACK: The Track is too long to draw.\n");
        goto StopCalcPoint;
      }
    }
    else {  // curve
      int StepNum = (int)fabs(R*degree*DEG_TO_RAD/TRACK_STEP);
      double StepDegree = degree*DEG_TO_RAD/StepNum;
      double LenL = (fabs(R)-TRACK_WIDTH_OUT/2.0)*sin(StepDegree/2.0)*2.0;
      double LenR = (fabs(R)+TRACK_WIDTH_OUT/2.0)*sin(StepDegree/2.0)*2.0;

      if ((type[i]&TYPE_BARRIER) >0) {	// barrier
        Eigen::Vector3d up(0,0,BARRIER_HEIGHT);
        LftOut[p] = LftOut[p-1] + up;	// move down
        RgtOut[p] = RgtOut[p-1] + up;
        p ++;
      }

      if (R < 0.0) {			// swap curve's direction
        double temp = LenL;
        LenL = LenR;
        LenR = temp;
        StepDegree = -StepDegree;
        R = -R;
      }
      for (int j=0; j<StepNum; j++) {	// calc each step
        Eigen::AngleAxis<double> rot(StepDegree / 2.0, Eigen::Vector3d::UnitZ());
        // turn the first half
        dir = rot * dir;
        LftOut[p] = LftOut[p-1] + LenL * dir;	// move forward
        RgtOut[p] = RgtOut[p-1] + LenR * dir;
        // turn the last half
        dir = rot * dir;
        TotalLength += fabs(StepDegree*R);
        if ((type[i]&TYPE_DASH)>0) {
          DashLO[dp] = LftOut[p];
          DashRO[dp] = RgtOut[p];
          dp ++;
        }
        p ++;
        if (p>=MAXPOINT) {
          printf("\tTRACK: The Track is too long to draw.\n");
          goto StopCalcPoint;
        }
      }
      if ((type[i]&TYPE_BARRIER) >0) {
        Eigen::Vector3d down(0,0,-BARRIER_HEIGHT);
        LftOut[p] = LftOut[p-1] + down;	// move up
        RgtOut[p] = RgtOut[p-1] + down;
        p ++;
        if (p>=MAXPOINT) {
          printf("\tTRACK: The Track is too long to draw.\n");
          goto StopCalcPoint;
        }
      }
    }
  }
StopCalcPoint :
  // try to connect tobe a circle
  if (	LftOut[p-1].x()<0 && RgtOut[p-1].x()>0 &&	// position limit
      LftOut[p-1].y()<LftOut[0].y() && RgtOut[p-1].y()<RgtOut[0].y() &&
      (LftOut[p-1] - LftOut[0]).norm() < 0.3 &&
      (RgtOut[p-1] - RgtOut[0]).norm() < 0.3 &&  // distance limit
      fabs(LftOut[p-1].x()-RgtOut[p-1].x())>0.9*TRACK_WIDTH_OUT)	// direction limit
  {
    LftOut[p] = LftOut[0];
    RgtOut[p] = RgtOut[0];
    TotalLength += (LftOut[p]-LftOut[p-1]+RgtOut[p]-RgtOut[p-1]).norm()/2.0;
    p ++;
    printf ("\tTRACK: Connect to be a circle automatically!\n");
  } else {
    printf ("\tTRACK: Can not connect to be a circle!\n");
  }
  PointNum = p;
  DashNum = dp;
  printf("\tTRACK: TotalLength of track: %5.2f.\n",TotalLength);
  return ;
}

#define DASHSTEP 20

void Track::InsertDash() {
  for (int i=DashNum-1; i>=0; i--) {
    DashLO[DASHSTEP*i] = DashLO[i];
    DashRO[DASHSTEP*i] = DashRO[i];
  }
  for (int i=0; i<DashNum-1; i++)
    for (int j=1; j<DASHSTEP; j++) {
      DashLO[i*DASHSTEP+j] = (DashLO[(i+1)*DASHSTEP]-DashLO[i*DASHSTEP])*j/DASHSTEP + DashLO[i*DASHSTEP];
      DashRO[i*DASHSTEP+j] = (DashRO[(i+1)*DASHSTEP]-DashRO[i*DASHSTEP])*j/DASHSTEP + DashRO[i*DASHSTEP];
    }
  DashNum = DashNum*DASHSTEP - DASHSTEP + 1;
}

void Track::CalcDash_IN() {
  for (int i=0; i<DashNum; i++) {
    Eigen::Vector3d v = DashRO[i] - DashLO[i];
    v.normalize ();
    DashLI[i] = DashLO[i] + v * LINE_WIDTH*1.1;
    DashRI[i] = DashRO[i] - v * LINE_WIDTH*1.1;
  }
  for (int i=0; i<DashNum; i++) {
    Eigen::Vector3d v = DashRO[i] - DashLO[i];
    v.normalize ();
    DashLO[i] -= v * 0.001;
    DashRO[i] += v * 0.001;
  }
}

void Track::save(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3) {
  static int first = 1;
  static int i = 0;
  static int r = 0;
  if (r != TrackReverseFlag) {
    r = TrackReverseFlag;
    first = 1;
    free (vertices);
    i = 0;
  }
  if (first) {
    first = 0;
    vertices = (double*)malloc(PointNum*6*3*3*sizeof(double));	// 6 points(2 triangle), 3 dimension, 3 surface
  }
  vertices[i++] = v1.x();
  vertices[i++] = v1.y();
  vertices[i++] = v1.z();

  vertices[i++] = v2.x();
  vertices[i++] = v2.y();
  vertices[i++] = v2.z();

  vertices[i++] = v3.x();
  vertices[i++] = v3.y();
  vertices[i++] = v3.z();

  TriNum += 3;
  if (i>PointNum*6*3*3) {
    printf("TRACK: save error!\n");
    getchar();
    exit (1);
  }
}

void Track::CalcMesh() {
  vertices = (double*)malloc(PointNum*6*3*3*sizeof(double));
  TriNum = 0;
  for (int i=0; i<PointNum-1; i++) {
    save (LftOut[i],RgtOut[i],RgtOut[i+1]);
    save (RgtOut[i+1],LftOut[i+1],LftOut[i]);
  }

  for (int i=0; i<PointNum-1; i++) {
    Eigen::Vector3d L0 = Eigen::Vector3d(LftOut[i+0].x(), LftOut[i+0].y(), 0.0);
    Eigen::Vector3d L1 = Eigen::Vector3d(LftOut[i+1].x(), LftOut[i+1].y(), 0.0);	// do not create profile if it is not very high
    if (LftOut[i].z()>2*TRACK_HEIGHT+0.001 || LftOut[i+1].z()>2*TRACK_HEIGHT+0.001)
      save (LftOut[i],LftOut[i+1],L1);
    if (LftOut[i].z()>2*TRACK_HEIGHT+0.001)
      save (L1,L0,LftOut[i]);
  }
  for (int i=0; i<PointNum-1; i++) {
    Eigen::Vector3d R0 = Eigen::Vector3d(RgtOut[i+0].x(), RgtOut[i+0].y(), 0.0);
    Eigen::Vector3d R1 = Eigen::Vector3d(RgtOut[i+1].y(), RgtOut[i+1].y(), 0.0);
    if (RgtOut[i].z()>2*TRACK_HEIGHT+0.001)
      save (RgtOut[i],R0,R1);
    if (RgtOut[i].z()>2*TRACK_HEIGHT+0.001 || RgtOut[i+1].z()>2*TRACK_HEIGHT+0.001)
      save (R1,RgtOut[i+1],RgtOut[i]);
  }

  indices = (dTriIndex*)malloc(TriNum*sizeof(dTriIndex));
  for (int i=0; i<TriNum; i++)
    indices[i] = i;
}

void Track::MakeMesh(dSpaceID space) {
  dMatrix3 R;
  MeshData = dGeomTriMeshDataCreate ();

  dGeomTriMeshDataBuildDouble (
      MeshData,
      vertices,
      3 * sizeof(double),
      TriNum,
      indices,
      TriNum,
      3 * sizeof(dTriIndex)
      );
  dGeomID track_mesh = dCreateTriMesh (space, MeshData, 0, 0, 0);
  dGeomTriMeshEnableTC (track_mesh, dCylinderClass, 0);
  dGeomTriMeshEnableTC (track_mesh, dBoxClass, 0);
  dGeomSetPosition (track_mesh, 0.0, 0.0, 0.0);
  dRSetIdentity (R);
  dGeomSetRotation (track_mesh, R);
}

void Track::CalcPoint_IN() {
  for (int i=0; i<PointNum; i++) {
    Eigen::Vector3d v = RgtOut[i] - LftOut[i];
    v.normalize ();
    LftIn[i] = LftOut[i] + v * LINE_WIDTH;
    RgtIn[i] = RgtOut[i] - v * LINE_WIDTH;
  }
}

void Track::CalcPoint_Middle() {
  for (int i=0; i<PathNum; i++)
    Middle[i] = (SecureL[i]+SecureR[i])/2.0;
}

void Track::CalcSecure() {
  int j = 0;
  for (int i=0; i<PointNum-1; i++,j++) {
    SecureL[j] = LftOut[i];
    SecureR[j] = RgtOut[i];

    Eigen::Vector3d v1 = LftOut[i] - RgtOut[i];
    Eigen::Vector3d v2 = LftOut[i+1] - RgtOut[i+1];
    if (v1!=v2) continue;
    double distance = (LftOut[i] - LftOut[i+1]).norm();
    int StepNum = (int)(distance/TRACK_STEP);
    Eigen::Vector3d dir = LftOut[i+1] - LftOut[i];
    dir.normalize ();
    for (int k=0; k<StepNum; k++,j++) {
      SecureL[j+1] = SecureL[j] + dir * distance/(StepNum+1.0);
      SecureR[j+1] = SecureR[j] + dir * distance/(StepNum+1.0);
    }
  }
  PathNum = j;

  if (PathSecurity > (TRACK_WIDTH_OUT-car_obj.GetCarWidth())/2.0) {
    PathSecurity = (TRACK_WIDTH_OUT-car_obj.GetCarWidth())/2.0-0.001;
    printf ("PATH: The security of path is too high.\n");
  }
  if (PathSecurity < -car_obj.GetCarWidth()/2.0) {
    PathSecurity = -car_obj.GetCarWidth()/2.0;
    printf ("PATH: The security of path is too low.\n");
  }
  for (int i=0; i<PathNum; i++) {
    Eigen::Vector3d v = SecureR[i] - SecureL[i];
    v.normalize ();
    SecureL[i] += v * (PathSecurity+car_obj.GetCarWidth()/2.0);
    SecureR[i] -= v * (PathSecurity+car_obj.GetCarWidth()/2.0);
  }
}

void Track::MakeTrack (dSpaceID space, const char * fname)
{
  static int first = 1;
  static int r = 1;
  if (first) {
    first = 0;
    ReadTrack (fname);
    Reverse ();
  }
  if (r!=TrackReverseFlag) {
    r = TrackReverseFlag;
    Reverse ();
    CalcPoint_OUT ();
    CalcMesh ();
    MakeMesh (space);
    CalcPoint_IN ();
    InsertDash ();
    CalcDash_IN ();
    CalcSecure ();
    CalcPoint_Middle ();
  }
  else {
    MakeMesh (space);
  }
}

void Track::DestroyTrack ()
{
  dGeomTriMeshDataDestroy (MeshData);
}

void Track::DrawEndLine() {
  double len = EndLineDistance;
  int WinID = glutGetWindow ();
  while (len<0.0) len += TotalLength;
  while (len>TotalLength) len -= TotalLength;

  for (int i=1; i<PointNum; i++) {
    Eigen::Vector3d p = (LftOut[i-1]+RgtOut[i-1]) / 2.0;
    Eigen::Vector3d n = (LftOut[i  ]+RgtOut[i  ]) / 2.0;
    len -= (n-p).norm();
    if (len<0.0) {
      Eigen::AngleAxis<double> rot(M_PI_2, Eigen::Vector3d::UnitZ());
      Eigen::Vector3d dir = (rot * (RgtOut[i] - LftOut[i])).normalized();
      ELineL = LftOut[i] + dir*len;
      ELineR = RgtOut[i] + dir*len;
      break;
    }
  }

  glColor3f (0.0f,0.0f,0.0f);
  glPushMatrix ();
  if (WinID==WinGod) {
    epsilon = ViewPoint.z()/500.0;
  } else {
    epsilon = 0.001;
  }
  glTranslated(ELineL.x(),ELineL.y(),ELineL.z() + epsilon);
  epsilon = 0.001;
  Eigen::Vector3d v = ELineR - ELineL;
  double theta = atan(v.y()/v.x());
  if (v.x()<0)	if (v.y()>0)	theta += M_PI;
  else		theta -= M_PI;
  glRotated (theta*RAD_TO_DEG,0,0,1);
  glRectd (TRACK_WIDTH_OUT/8.0,0,TRACK_WIDTH_OUT*3.0/8.0,LINE_WIDTH);
  glRectd (TRACK_WIDTH_OUT*5.0/8.0,0,TRACK_WIDTH_OUT*7.0/8.0,LINE_WIDTH);
  glPopMatrix ();
}

void Track::DrawMiddleLine() {
  glColor3f (0,0,0);
  glLineWidth (1);
  glBegin (GL_LINES);
  epsilon = ViewPoint.z() / 500.0;
  for (int i=0; i<PathNum; i++)
    glVertex3d (Middle[i].x(),Middle[i].y(),Middle[i].z()+epsilon);
  epsilon = 0.001;
  glEnd ();
}

void Track::DrawTrack() {
  int WinID = glutGetWindow ();

  glColor3f (0.6f,0.6f,0.6f);
  glBegin (GL_QUAD_STRIP);
  for (int i=0; i<PointNum; i++) {				// left profile
    if (LftOut[i].z()<0.02) {
      glEnd();
      glBegin (GL_QUAD_STRIP);
      continue;
    }
    glVertex3d (LftOut[i].x(),LftOut[i].y(),LftOut[i].z() - epsilon);
    glVertex3d (LftOut[i].x(),LftOut[i].y(),0);
  }
  glEnd ();

  glColor3f (0.6f,0.6f,0.6f);
  glBegin (GL_QUAD_STRIP);
  for (int i=0; i<PointNum; i++) {				// right profile
    if (RgtOut[i].z()<0.02) {
      glEnd();
      glBegin (GL_QUAD_STRIP);
      continue;
    }
    glVertex3d (RgtOut[i].x(),RgtOut[i].y(),RgtOut[i].z() - epsilon);
    glVertex3d (RgtOut[i].x(),RgtOut[i].y(),0);
  }
  glEnd ();

  glColor3f (0.0f,0.0f,0.0f);
  glBegin (GL_QUAD_STRIP);
  epsilon = 0.001;
  for (int i=0; i<PointNum; i++) {				// border
    glVertex3d (LftOut[i].x(),LftOut[i].y(),LftOut[i].z()-epsilon);
    glVertex3d (RgtOut[i].x(),RgtOut[i].y(),RgtOut[i].z()-epsilon);
  }
  glEnd ();

  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(-1.0f, -1.0f);
  glColor3d (0.9,0.0,0.0);
  glBegin (GL_QUAD_STRIP);
  double len = 0.05;
  for (int i=0; i<DashNum; i++) {					// dash left
    double dlen;
    if (i>0) dlen = (DashLI[i] - DashLI[i-1]).norm ();
    else	dlen = 0.0;
    len += dlen;
    if (dlen>TRACK_STEP/5) {
      len = 0.05;
      glEnd ();
      glBegin (GL_QUAD_STRIP);
    }
    if (((int)(len/0.1))%2)	glColor3d (0.66,0.66,0.66);
    else			glColor3d (0.0,0.0,0.0);
    glVertex3d (DashLO[i].x(),DashLO[i].y(),DashLO[i].z() - epsilon);
    glVertex3d (DashLI[i].x(),DashLI[i].y(),DashLI[i].z());
  }
  glEnd ();


  glColor3d (0.66,0.66,0.66);
  glBegin (GL_QUAD_STRIP);
  len = 0.05;
  for (int i=0; i<DashNum; i++) {					// dash right
    double dlen;
    if (i>0) dlen = (DashRI[i] - DashRI[i-1]).norm ();
    else	dlen = 0.0;
    len += dlen;
    if (dlen>TRACK_STEP/5) {
      len = 0.05;
      glEnd ();
      glBegin (GL_QUAD_STRIP);
    }
    if (((int)(len/0.1))%2)	glColor3d (0.66,0.66,0.66);
    else			glColor3d (0.0,0.0,0.0);
    glVertex3d (DashRO[i].x(),DashRO[i].y(),DashRO[i].z() - epsilon);
    glVertex3d (DashRI[i].x(),DashRI[i].y(),DashRI[i].z());
  }
  glEnd ();
  glDisable(GL_POLYGON_OFFSET_FILL);

  glColor3f (0.66f,0.66f,0.66f);
  glBegin (GL_QUAD_STRIP);
  if (WinID==WinGod) {
    epsilon = ViewPoint.z()/1000.0;
  } else {
    epsilon = 0.0;
  }
  for (int i=0; i<PointNum; i++) {				// track surface (cover the black border)
    if (fabs(LftIn[i].z() - TRACK_HEIGHT - BARRIER_HEIGHT)<0.001 && cartype == balance)
      glColor3f (0.0f,0.0f,0.0f);
    else
      glColor3f (0.66f,0.66f,0.66f);
    glVertex3d (LftIn[i].x(),LftIn[i].y(),LftIn[i].z()+epsilon);
    glVertex3d (RgtIn[i].x(),RgtIn[i].y(),RgtIn[i].z()+epsilon);
  }
  epsilon = 0.001;
  glEnd ();

  glColor3f (0.6f,0.6f,0.6f);
  glBegin (GL_QUAD_STRIP);
  for (int i=0; i<PointNum; i++) {				// track button
    glVertex3d (LftOut[i].x(),LftOut[i].y(),0.0);
    glVertex3d (RgtOut[i].x(),RgtOut[i].y(),0.0);
  }
  glEnd ();

  DrawEndLine ();

  if (WinID!=WinCar && viewtype!=car && MiddleLineFlag)	DrawMiddleLine();
}
