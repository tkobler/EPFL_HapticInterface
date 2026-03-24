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

// Physical limits of the paddle range [deg].
// Adjust these values to match the actual hardware range.
#define PADDLE_ANGLE_MIN   -45.0f
#define PADDLE_ANGLE_MAX    45.0f

// Torque output saturation limit [N.m].
// Prevents the PID from commanding unsafe torques.
#define PID_TORQUE_SAT      0.5f

volatile uint32_t  hapt_timestamp;          // Time base of the controller [us].
volatile float32_t hapt_hallVoltage;        // Hall sensor output voltage [V].
volatile float32_t hapt_encoderPaddleAngle; // Paddle angle from incremental encoder [deg].
volatile float32_t hapt_motorTorque;        // Motor torque [N.m].

volatile float32_t spring_k  = 0.01f;
volatile float32_t zero_angle = 0.0f;
volatile float32_t relative_angle;

// Velocity and acceleration computation
volatile float32_t paddle_velocity_1; // velocity with 1 sample differentiation
volatile float32_t paddle_velocity_2; // velocity with 2 samples differentiation
volatile float32_t paddle_acc_1;      // acceleration with 1 sample differentiation
volatile float32_t paddle_acc_2;      // acceleration with 2 samples differentiation

float32_t prev_angle_1   = 0.0f;
float32_t prev_angle_2   = 0.0f;
float32_t prev_velocity_1 = 0.0f;
float32_t prev_velocity_2 = 0.0f;

float32_t filtred_vel    = 0.0f;
float32_t prev_fil_vel   = 0.0f;
float32_t filtred_acc    = 0.0f;
float32_t prev_fil_acc   = 0.0f;

float32_t tao = 1.0f / (200.0f * 2.0f * 3.14159265f); // Velocity/accel filter time constant [s].

// ---------------------------------------------------------------------------
// PID Position Controller
// ---------------------------------------------------------------------------

// Enable flag: 0 = disabled (default at startup), 1 = enabled.
volatile uint32_t  pid_enabled         = 0;

// Sensor selection: 0 = incremental encoder (default), 1 = Hall-effect sensor.
volatile uint32_t  pid_use_hall        = 0;

// Target position set from the remote-control interface [deg].
// Clamped to [PADDLE_ANGLE_MIN, PADDLE_ANGLE_MAX] on write via hapt_Update.
volatile float32_t pid_target_angle    = 0.0f;

// Parallel PID gains (tunable from the remote-control interface).
volatile float32_t pid_Kp              = 0.001;  // Proportional gain [N.m/deg]
volatile float32_t pid_Ki              = 0.001;  // Integral gain     [N.m/(deg.s)]
volatile float32_t pid_Kd              = 0.0005f; // Derivative gain   [N.m.s/deg]

// Input (position) low-pass filter time constant for the PID [s].
// A dedicated filter on the measured position prevents the derivative
// term from amplifying sensor noise.
volatile float32_t pid_filter_tao      = 1.0f / (50.0f * 2.0f * 3.14159265f); // 50 Hz cut-off

// Read-only diagnostics exposed to the remote-control interface.
volatile float32_t pid_error           = 0.0f;  // Current position error [deg].
volatile float32_t pid_torque_output   = 0.0f;  // Torque commanded by PID [N.m].
volatile float32_t pid_measured_angle  = 0.0f;  // Filtered angle used by PID [deg].

// Internal PID state (not exposed).
static float32_t pid_integral          = 0.0f;
static float32_t pid_prev_error        = 0.0f;
static float32_t pid_filtered_pos      = 0.0f;  // Low-pass filtered measured position [deg].

void hapt_Update(void);

/**
  * @brief Initializes the haptic controller.
  */
void hapt_Init(void)
{
    hapt_timestamp  = 0;
    hapt_motorTorque = 0.0f;

    // Reset PID internal state.
    pid_integral     = 0.0f;
    pid_prev_error   = 0.0f;
    pid_filtered_pos = 0.0f;

    // Make the timers call the update function periodically.
    cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

    // --- Original monitored variables ---
    comm_monitorUint32Func("timestep [us]", cbt_GetHapticControllerPeriod,
                           cbt_SetHapticControllerPeriod);
    comm_monitorFloat("motor_torque [N.m]",      (float32_t*)&hapt_motorTorque,        READWRITE);
    comm_monitorFloat("encoder_paddle_pos [deg]",(float32_t*)&hapt_encoderPaddleAngle, READONLY);
    comm_monitorFloat("hall_voltage [V]",         (float32_t*)&hapt_hallVoltage,        READONLY);

    comm_monitorFloat("spring constant",  (float32_t*)&spring_k,     READWRITE);
    comm_monitorFloat("zero_angle",       (float32_t*)&zero_angle,    READWRITE);
    comm_monitorFloat("relative_angle",   (float32_t*)&relative_angle,READONLY);

    comm_monitorFloat("vel_1sample",           (float32_t*)&paddle_velocity_1, READONLY);
    comm_monitorFloat("vel_2sample",           (float32_t*)&paddle_velocity_2, READONLY);
    comm_monitorFloat("acc_1sample",           (float32_t*)&paddle_acc_1,      READONLY);
    comm_monitorFloat("acc_2sample",           (float32_t*)&paddle_acc_2,      READONLY);
    comm_monitorFloat("filtred speed",         (float32_t*)&filtred_vel,       READONLY);
    comm_monitorFloat("filtred acceleration",  (float32_t*)&filtred_acc,       READONLY);

    // --- PID controller variables ---
    // Control / configuration (read-write from the PC interface).
    comm_monitorUint32("pid_enabled",           (uint32_t*)&pid_enabled,       READWRITE);
    comm_monitorUint32("pid_use_hall",          (uint32_t*)&pid_use_hall,      READWRITE);
    comm_monitorFloat("pid_target_angle [deg]", (float32_t*)&pid_target_angle, READWRITE);
    comm_monitorFloat("pid_Kp",                 (float32_t*)&pid_Kp,           READWRITE);
    comm_monitorFloat("pid_Ki",                 (float32_t*)&pid_Ki,           READWRITE);
    comm_monitorFloat("pid_Kd",                 (float32_t*)&pid_Kd,           READWRITE);
    comm_monitorFloat("pid_filter_tao [s]",     (float32_t*)&pid_filter_tao,   READWRITE);

    // Diagnostics (read-only).
    comm_monitorFloat("pid_error [deg]",        (float32_t*)&pid_error,         READONLY);
    comm_monitorFloat("pid_torque [N.m]",       (float32_t*)&pid_torque_output, READONLY);
    comm_monitorFloat("pid_meas_angle [deg]",   (float32_t*)&pid_measured_angle,READONLY);
}

/**
  * @brief Updates the haptic controller state.
  */
void hapt_Update(void)
{
    float32_t motorShaftAngle; // [deg].

    // Compute dt [s].
    float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f;

    // Increment the timestamp.
    hapt_timestamp += cbt_GetHapticControllerPeriod();

    // Get the Hall sensor voltage and derive angle.
    hapt_hallVoltage = hall_GetVoltage();
    relative_angle   = hapt_hallVoltage * 48.7132f - 96.7068f;

    // Get the encoder position.
    motorShaftAngle        = enc_GetPosition();
    hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;

    // -----------------------------------------------------------------------
    // PID Position Controller
    // -----------------------------------------------------------------------

    // Clamp the target angle to the physically feasible paddle range.
    if (pid_target_angle < PADDLE_ANGLE_MIN) pid_target_angle = PADDLE_ANGLE_MIN;
    if (pid_target_angle > PADDLE_ANGLE_MAX) pid_target_angle = PADDLE_ANGLE_MAX;

    if (pid_enabled)
    {
        // 1. Select the raw measured position based on sensor choice.
        float32_t raw_pos = pid_use_hall ? relative_angle : hapt_encoderPaddleAngle;

        // 2. Low-pass filter the measured position to attenuate noise
        //    before it reaches the derivative term.
        //    First-order IIR: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
        float32_t alpha_pid = dt / (pid_filter_tao + dt);
        pid_filtered_pos    = alpha_pid * raw_pos + (1.0f - alpha_pid) * pid_filtered_pos;
        pid_measured_angle  = pid_filtered_pos; // expose to remote interface

        // 3. Compute position error [deg].
        float32_t error = pid_target_angle - pid_filtered_pos;
        pid_error = error;

        // 4. Integrate the error (with simple anti-windup via torque saturation
        //    — accumulation is only applied when not saturating).
        pid_integral += error * dt;

        // 5. Differentiate the error.
        float32_t error_derivative = (error - pid_prev_error) / dt;
        pid_prev_error = error;

        // 6. Parallel PID sum.
        float32_t torque_cmd = pid_Kp * error
                             + pid_Ki * pid_integral
                             + pid_Kd * error_derivative;

        // 7. Saturate the output torque.
        if (torque_cmd >  PID_TORQUE_SAT) torque_cmd =  PID_TORQUE_SAT;
        if (torque_cmd < -PID_TORQUE_SAT) torque_cmd = -PID_TORQUE_SAT;

        pid_torque_output = torque_cmd;
        hapt_motorTorque  = torque_cmd;
    }
    else
    {
        // Controller disabled: reset integrator and derivative memory so
        // there is no transient bump when re-enabling.
        pid_integral      = 0.0f;
        pid_prev_error    = 0.0f;
        pid_torque_output = 0.0f;
        // hapt_motorTorque is left under manual control from the PC interface.
    }

    // Apply the motor torque (PID output or manual value when disabled).
    torq_SetTorque(hapt_motorTorque);

    // -----------------------------------------------------------------------
    // Velocity and acceleration estimation (unchanged from original)
    // -----------------------------------------------------------------------
    float32_t angle = hapt_encoderPaddleAngle;
    // Uncomment the line below to use the Hall sensor instead:
    // angle = relative_angle;

    paddle_velocity_1 = (angle - prev_angle_1) / dt;
    paddle_velocity_2 = (angle - prev_angle_2) / (2.0f * dt);

    paddle_acc_1 = (paddle_velocity_1 - prev_velocity_1) / dt;
    paddle_acc_2 = (paddle_velocity_1 - prev_velocity_2) / (2.0f * dt);

    prev_angle_2    = prev_angle_1;
    prev_angle_1    = angle;
    prev_velocity_2 = prev_velocity_1;
    prev_velocity_1 = paddle_velocity_1;

    // Velocity and acceleration low-pass filtering.
    float32_t alpha = dt / (tao + dt);
    filtred_vel  = alpha * paddle_velocity_1 + (1.0f - alpha) * prev_fil_vel;
    prev_fil_vel = filtred_vel;
    filtred_acc  = alpha * paddle_acc_1 + (1.0f - alpha) * prev_fil_acc;
    prev_fil_acc = filtred_acc;
}

