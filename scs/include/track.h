#ifndef _TRACK_H_
#define _TRACK_H_

#include <Eigen/Eigen>
#include <ode/common.h>

#define TRACK_WIDTH_OUT  0.45
#define TRACK_WIDTH_IN   (TRACK_WIDTH_OUT-2*LINE_WIDTH)
#define TRACK_STEP       0.05
#define TRACK_HEIGHT     0.01

#define LINE_WIDTH       0.025
#define LINE_LENGTH      0.10

#define BARRIER_HEIGHT     0.005
#define ELEVATION_ANGLE    15
#define ELEVATION_ANGLE_B  12
#define TRANSITION_LENGTH  0.10

#define MAXPOINT    10000
#define MAXSEGMENT  1000

using Eigen::Vector3d;

class Track {
 public:
  Track();
  void MakeTrack(dSpaceID space, const char * fname);
  void DestroyTrack ();
  void DrawTrack();
  double PathSecurity() const { return path_security_; }
  void SetPathSecurity(double security) { path_security_ = security; }
  double TotalLength() const { return total_length_; }
  void SetMiddleLine(bool flag) { middle_line_flag_ = flag; }
  bool TrackReverseFlag() const { return track_reverse_flag_; }
  void SetTrackReverseFlag(bool flag) { track_reverse_flag_ = flag; }
  double EndLineDistance() const { return end_line_distance_; }
  Vector3d ELineL() const { return end_line_l_; }
  Vector3d ELineR() const { return end_line_r_; }

 private:
  void ReadTrack(const char * fname);
  void Reverse();
  void CalcPoint_OUT();
  void InsertDash();
  void CalcDash_IN();
  void save(Vector3d v1, Vector3d v2, Vector3d v3);
  void CalcMesh();
  void MakeMesh(dSpaceID space);
  void CalcPoint_IN();
  void CalcPoint_Middle();
  void CalcSecure();
  void DrawEndLine();
  void DrawMiddleLine();

 private:
  Vector3d end_line_l_;
  Vector3d end_line_r_;
  bool track_reverse_flag_;
  double total_length_;
  bool middle_line_flag_;
  int PathNum;
  int ConnectFlag;
  double path_security_;
  int PointNum;
  int TriNum;
  double end_line_distance_;
  dTriMeshDataID MeshData;
  double track[MAXSEGMENT];
  unsigned char type[MAXSEGMENT];
  double *vertices;
  dTriIndex *indices;
  int Segment;
  double ElevationAngle;
  double epsilon;
  int DashNum = 0;

  Vector3d LftOut [MAXPOINT];
  Vector3d RgtOut [MAXPOINT];
  Vector3d LftIn [MAXPOINT];
  Vector3d RgtIn [MAXPOINT];
  Vector3d SecureL [MAXPOINT];
  Vector3d SecureR [MAXPOINT];
  Vector3d Middle [MAXPOINT];
  Vector3d Path[MAXPOINT];
  double Cur  [MAXPOINT];
  double Limit [MAXPOINT];
  Vector3d DashLO [MAXPOINT];
  Vector3d DashLI [MAXPOINT];
  Vector3d DashRO [MAXPOINT];
  Vector3d DashRI [MAXPOINT];
};

#endif
