#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"
#include <math.h>

void AS5600_Init(void);
uint16_t AS5600_GetRawAngle(void);
float AS5600_GetAngle_Without_Track(void);
float AS5600_GetAngle(void);

#endif
