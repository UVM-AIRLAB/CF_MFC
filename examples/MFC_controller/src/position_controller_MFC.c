/*
 *
 * Copyright (c) 2019 Ewoud Smeur and Evghenii Volodscoi
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This control algorithm is the model free control (MFC) outer loop controller to control the 
 * postion of the Crazyflie.
 */

#include "position_controller_MFC.h"

// Going to hardcode gains 

static CONTROLLER_MFC_t g_self = 
{
    //XY PD Gains
    .kp_xy = 8.2314;    //P
    .kd_xy = 5.6000;    //D

    //Z PD Gains
    .kp_z = 14.6314;    //P
    .kd_z = 9.6000;     //D

    //Roll and Pitch PD Gains
    .kp_rp = 4816;      //P
    .kd_rp = 140;       //D

    //Yaw PD Gains
    .kp_yaw = 6744;     //P
    .kd_yaw = 180;      //D

    //Beta Parameters
    .beta_xy = 3.0;
    .beta_z = 2.0;
    .beta_rp = 10.0;
    .beta_yaw = 50.0;

    //Error Helpers
    .prev_x_error = 0;
    .prev_y_error  = 0;
    .prev_z_error  = 0;
    .prev_roll_error  = 0;
    .prev_pitch_error  = 0;
    .prev_yaw_error  = 0;
}

void positionControllerMFCInit(void)
{
    // Do some initalization call here
}

void positionControllerMFC(float* thrust, attitude_t *attitude, const setpoint_t *setpoint, const state_t *state)
{
    //Do a standard position controller call / add in position controller logic but switch for z
    
}


                                                        
