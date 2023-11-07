/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 */


/*
IDEAS:
Should the Kalman Filter for parameter estimation be a single function?\
** Hardcode the pole placement **


Needed:
- Struct containing the controller gains, beta values
- Kalman Filter for F estimation
- Error calculation function
- Control effort calculation function
- 

*/

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "MFCCONTROLLER"
#include "debug.h"

// CONTROLLER CALLS
/*
MFC will be used on the i) Z ii) XY iii) Attitude. Phase i will require a standard attitude controller call from the default library with a 
modfied position controller.

Phase i Requirements:
- Controller Header
- PID Controller Header
- Attitude Controller Header
-> Modified Position Controller / MFC Controller Header
*/

// Move the includes to the the top of the file if you want to
#include "controller.h"
#include "controller_pid.h"
#include "attitude_controller.h"
#include "controller_MFC.h"
#include "position_controller_MFC.h"


#define ATTITUDE_UPDATE_DT (float)(1.0f/ATTITUDE_RATE) // Line from controller_pid.c for updating attitude 

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

while(1) {
    vTaskDelay(M2T(2000));
  }
}

// #1 Initialization

/*
Intialization will require the initialization of 
i) Attitude Controller
ii) Modified Position Controller / MFC Controller
*/

static bool initTest;

void controllerOutOfTreeInit() {
  /*
  TODO:
    Write an intialization function for a modified position PID controller with the Z direction referencing an MFC structure
  */
  attitudeControllerInit(ATTITUDE_UPDATE_DT); 

  initTest = true;
}

// #2 Controller Test
bool controllerOutOfTreeTest() {
  // Return true once the initialization is finished

  initTest &= attitudeControllerTest(); // MUST PASS ATTITUDE CONTROLLER TEST AS WELL
  return initTest;
}

// #3 Controller Update Functions

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Implement your controller here...

  // Call the PID controller instead in this example to make it possible to fly
  controllerPid(control, setpoint, sensors, state, tick);
}

// #4 Logging Parameters