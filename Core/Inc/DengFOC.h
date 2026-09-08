#ifndef __DENGFOC_H__
#define __DENGFOC_H__

#include "main.h"
#include <math.h>

#ifndef PI
#define PI M_PI
#endif

float _electricalAngle(float shaft_angle, int pole_pairs);
float _normalizeAngle(float angle);
void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq,float Ud, float angle_el);
float velocityOpenloop(float target_velocity);

#endif
