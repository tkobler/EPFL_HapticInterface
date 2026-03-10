/*
 * Copyright (C) 2021 EPFL-REHAssist (Rehabilitation and Assistive Robotics Group).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "haptic_controller.h"
#include "communication.h"
#include "drivers/adc.h"
#include "drivers/incr_encoder.h"
#include "drivers/hall.h"
#include "drivers/callback_timers.h"
#include "lib/utils.h"
#include "torque_regulator.h"

#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 350 // Default control loop period [us].

volatile uint32_t  hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_encoderPaddleAngle; // Paddle angle measured by the incremental encoder [deg].
volatile float32_t hapt_motorTorque; // Motor torque [N.m].

volatile float32_t spring_k = 0.01; //
volatile float32_t zero_angle = 0; //
volatile float32_t relative_angle;

// Velocity and acceleration computation
volatile float32_t paddle_velocity_1;   // velocity with 1 sample differentiation
volatile float32_t paddle_velocity_2;   // velocity with 2 samples differentiation

volatile float32_t paddle_acc_1;        // acceleration with 1 sample differentiation
volatile float32_t paddle_acc_2;        // acceleration with 2 samples differentiation

float32_t prev_angle_1 = 0;
float32_t prev_angle_2 = 0;

float32_t prev_velocity_1 = 0;
float32_t prev_velocity_2 = 0;



void hapt_Update(void);

/**
  * @brief Initializes the haptic controller.
  */
void hapt_Init(void)
{
    hapt_timestamp = 0;
    hapt_motorTorque = 0.0f;

    // Make the timers call the update function periodically.
    cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

    // Share some variables with the computer.
    comm_monitorUint32Func("timestep [us]", cbt_GetHapticControllerPeriod,
                           cbt_SetHapticControllerPeriod);
    comm_monitorFloat("motor_torque [N.m]", (float32_t*)&hapt_motorTorque, READWRITE);
    comm_monitorFloat("encoder_paddle_pos [deg]", (float32_t*)&hapt_encoderPaddleAngle, READONLY);
    comm_monitorFloat("hall_voltage [V]", (float32_t*)&hapt_hallVoltage, READONLY);

    // added display constant
    comm_monitorFloat("spring constant", (float32_t*)&spring_k, READWRITE);
    comm_monitorFloat("zero_angle", (float32_t*)&zero_angle, READWRITE);
    comm_monitorFloat("relative_angle", (float32_t*)&relative_angle, READONLY);

    comm_monitorFloat("vel_1sample", (float32_t*)&paddle_velocity_1, READONLY);
    comm_monitorFloat("vel_2sample", (float32_t*)&paddle_velocity_2, READONLY);
    comm_monitorFloat("acc_1sample", (float32_t*)&paddle_acc_1, READONLY);
    comm_monitorFloat("acc_2sample", (float32_t*)&paddle_acc_2, READONLY);

}

/**
  * @brief Updates the haptic controller state.
  */

void hapt_Update()
{
    float32_t motorShaftAngle; // [deg].

    // Compute the dt (uncomment if you need it).
    float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f; // [s].

    // Increment the timestamp.
    hapt_timestamp += cbt_GetHapticControllerPeriod();
    
    // Get the Hall sensor voltage.
    hapt_hallVoltage = hall_GetVoltage();

    // Get the encoder position.
    motorShaftAngle = enc_GetPosition();
    hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;

    // Compute the motor torque, and apply it.
    //hapt_motorTorque = 0.0f;
    relative_angle = hapt_hallVoltage* 48.7132 -96.7068;
    //hapt_motorTorque = -relative_angle*spring_k/ REDUCTION_RATIO;
    torq_SetTorque(hapt_motorTorque);

    // Compute velocity and acceleration
    float32_t angle;
    //angle = hapt_encoderPaddleAngle;
    angle = relative_angle;

    // ----- Velocity (1 sampling period)
    paddle_velocity_1 = (angle - prev_angle_1) / dt;
    // ----- Velocity (2 sampling periods)
    paddle_velocity_2 = (angle - prev_angle_2) / (2*dt);

    // ----- Acceleration (1 sampling period)
    paddle_acc_1 = (paddle_velocity_1 - prev_velocity_1) / dt;
    // ----- Acceleration (2 sampling periods)
    paddle_acc_2 = (paddle_velocity_1 - prev_velocity_2) / (2*dt);

    prev_angle_2 = prev_angle_1;
    prev_angle_1 = angle;
    prev_velocity_2 = prev_velocity_1;
    prev_velocity_1 = paddle_velocity_1;
}

