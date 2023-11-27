#ifndef __POSITION_CONTROLLER_MFC_H__
#define __POSITION_CONTROLLER_MFC_H__

//Saturation thrust limits should go here

#include "controller_MFC.h"
#include "stabilizer.h"
#include "math3d.h"
#include "log.h"
#include "param.h"

// Postion controller will run a normal PID on X & Y but MFC on Z 
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
    float kp_yaw;   // P
    float kd_yaw;   // D

    // Beta Parameters
    float beta_xy;
    float beta_z;
    float beta_rp;
    float beta_yaw;
    
    //Not sure if these are needed

    // Error Variables
    float prev_x_error;
    float prev_y_error;
    float prev_z_error;
    float prev_roll_error;
    float prev_pitch_error;
    float prev_yaw_error;

    // Reference Derivitive
    float prev_x_reference;
    float prev_y_reference;

    float z_ref_diff;
    float pref_z_ref_diff;
    float prev_z_reference;
    float prev_z_reference_2;

    float prev_roll_reference;
    float prev_pitch_reference;
    float prev_yaw_reference;

    // These variables are the outputs of the PD controller
    float mfc_x;
    float mfc_y;
    float mfc_z;
    float mfc_roll;
    float mfc_pitch;
    float mfc_yaw;

    // // Logging Parameters
    // float cmd_thrust;
    // float cmd_roll;
    // float cmd_pitch;
    // float cmd_yaw;

}; CONTROLLER_MFC_t

// A position controller calculate the thrust, roll, pitch to approach
// a 3D position setpoint
void positionControllerMFCInit(CONTROLLER_MFC_t* self);
void positionControllerMFC(CONTROLLER_MFC_t* self, float* thrust, attitude_t *attitude, const setpoint_t *setpoint,
                                                             const state_t *state);
void positionControllerResetAllfilters();
void positionControllerResetAllPID();                                                       
// I don't think we need this but keeping it in 
void velocityController(float* thrust, attitude_t *attitude, const Axis3f* setpoint_velocity,
                                                             const state_t *state) 
//Needs: i) states ii) betas iii) KF constants iv) DT v) control input
void positionControllerMFCFilter(const state_t *state)


#endif // __POSITION_CONTROLLER_MFC_H__