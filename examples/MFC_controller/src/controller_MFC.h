#ifndef __CONTROLLER_MFC_H__
#define __CONTROLLER_MFC_H__

#include "stabilizer_types.h" //Important parameters such as , ATTITUDERATE, POSITIONRATE, and Loop rates
#include "math3d.h"

typedef struct 
{
    // XY PD Gains
    float kp_xy; // P
    float kd_xy; // D

    // Z PD Gains
    float kp_z;  // P
    float kd_z;  // D

    // Roll PD Gains
    float kp_rp;  // P
    float kd_rp;  // D

    // Yaw PD Gains
    float kp_y;   // P
    float kd_y;   // D

    // Beta Parameters
    float beta_xy;
    float beta_z;
    float beta_rp;
    float beta_y;

    // Error Variables
    float prev_x;
    float prev_y;
    float prev_z;
    float prev_roll;
    float prev_pitch;
    float prev_yaw;

    // Logging Parameters
    float cmd_thrust;
    float cmd_roll;
    float cmd_pitch;
    float cmd_yaw;

}; CONTROLLER_MFC_t

void controllerOutOfTreeInit(CONTROLLER_MFC_t* self);
bool controllerOutOfTreeTest(CONTROLLER_MFC_t* self);
void controllerOutOfTree(CONTROLLER_MFC_t* self, control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)

#endif //__CONTROLLER_MFC_H__