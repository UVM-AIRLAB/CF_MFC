#ifndef __CONTROLLER_MFC_H__
#define __CONTROLLER_MFC_H__

#include "stabilizer_types.h" //Important parameters such as , ATTITUDERATE, POSITIONRATE, and Loop rates
#include "math3d.h"


void controllerOutOfTreeInit(void);
bool controllerOutOfTreeTest(void);
void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)

#endif //__CONTROLLER_MFC_H__