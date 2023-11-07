#ifndef __POSITION_CONTROLLER_MFC_H__
#define __POSITION_CONTROLLER_MFC_H__

//Saturation thrust limits should go here

#include "controller_MFC.h"
#include "stabilizer.h"
#include "math3d.h"
#include "log.h"
#include "param.h"

// Postion controller will run a normal PID on X & Y but MFC on Z 

// A position controller calculate the thrust, roll, pitch to approach
// a 3D position setpoint
void positionControllerInit();
void positionControllerResetAllPID();
void positionControllerResetAllfilters();
void positionController(float* thrust, attitude_t *attitude, const setpoint_t *setpoint,
                                                             const state_t *state);
void velocityController(float* thrust, attitude_t *attitude, const Axis3f *setpoint_velocity,
                                                             const state_t *state);


#endif // __POSITION_CONTROLLER_MFC_H__