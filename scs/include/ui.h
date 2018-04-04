#ifndef _UI_H_
#define _UI_H_

#include <Eigen/Eigen>

#define GLUT_WHEEL_UP	3
#define GLUT_WHEEL_DOWN	4

typedef struct sRoute {
	Eigen::Vector3d pos;
	sRoute * next;
	double speed;
} * sRouteID;

extern sRouteID head;

void Play ();
void Key (unsigned char cmd, int x, int y);
void SpecialKeyPress (int key, int x, int y);
void SpecialKeyUp(int key, int x, int y);
void Mouse (int button, int state, int x, int y);
void motion (int x, int y);
void ClearRoute ();
void Free (sRouteID head);

#endif
