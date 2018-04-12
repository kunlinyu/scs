#ifndef _TRACK_H_
#define _TRACK_H_

#include <Eigen/Eigen>
#include <ode/common.h>

#define TRACK_WIDTH_OUT		0.45
#define TRACK_WIDTH_IN		(TRACK_WIDTH_OUT-2*LINE_WIDTH)
#define TRACK_STEP		0.05
#define TRACK_HEIGHT		0.01

#define LINE_WIDTH		0.025
#define LINE_LENGTH		0.10

#define BARRIER_HEIGHT		0.005
#define ELEVATION_ANGLE		15
#define ELEVATION_ANGLE_B	12
#define TRANSITION_LENGTH	0.10

#define MAXPOINT		10000
#define MAXSEGMENT		1000

class Track {
 public:
  Track();
  void MakeTrack(dSpaceID space, const char * fname);
  void DestroyTrack ();
  void DrawTrack();

 public:
  int PointNum;
  double TotalLength;
  double EndLineDistance;
  double PathSecurity;
  int PathNum;
  Eigen::Vector3d ELineL;
  Eigen::Vector3d ELineR;
  int ConnectFlag;
  int MiddleLineFlag;
  int TrackReverseFlag;

 private:
  void ReadTrack(const char * fname);
  void Reverse();
  void CalcPoint_OUT();
  void InsertDash();
  void CalcDash_IN();
  void save(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3);
  void CalcMesh();
  void MakeMesh(dSpaceID space);
  void CalcPoint_IN();
  void CalcPoint_Middle();
  void CalcSecure();
  void DrawEndLine();
  void DrawMiddleLine();

 private:
	int TriNum;
	dTriMeshDataID MeshData;
	double track[MAXSEGMENT];
	unsigned char type[MAXSEGMENT];
	double *vertices;
	dTriIndex *indices;
	int Segment;
	double ElevationAngle;
  double epsilon;

	Eigen::Vector3d LftOut	[MAXPOINT];
	Eigen::Vector3d RgtOut	[MAXPOINT];
	Eigen::Vector3d LftIn	[MAXPOINT];
	Eigen::Vector3d RgtIn	[MAXPOINT];
	Eigen::Vector3d SecureL	[MAXPOINT];
	Eigen::Vector3d SecureR	[MAXPOINT];
	Eigen::Vector3d Middle	[MAXPOINT];
  Eigen::Vector3d Path[MAXPOINT];
	double Cur		[MAXPOINT];
	double Limit	[MAXPOINT];
	Eigen::Vector3d DashLO	[MAXPOINT];
	Eigen::Vector3d DashLI	[MAXPOINT];
	Eigen::Vector3d DashRO	[MAXPOINT];
	Eigen::Vector3d DashRI	[MAXPOINT];

	int DashNum = 0;

};

#endif
