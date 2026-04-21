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
#define PADDLE_ANGLE_MIN   -45.0f
#define PADDLE_ANGLE_MAX    45.0f

// Torque output saturation limit [N.m].
#define PID_TORQUE_SAT      0.5f

volatile uint32_t  hapt_timestamp;          // Time base of the controller [us].
volatile float32_t hapt_hallVoltage;        // Hall sensor output voltage [V].
volatile float32_t hapt_encoderPaddleAngle; // Paddle angle from incremental encoder [deg].
volatile float32_t hapt_motorTorque;        // Motor torque [N.m].

volatile float32_t spring_k  = 0.01f;
volatile float32_t zero_angle = 0.0f;
volatile float32_t relative_angle;

// -----------------------------------------------------------------------
// Velocity and acceleration estimation
// -----------------------------------------------------------------------
volatile float32_t paddle_velocity_1;
volatile float32_t paddle_velocity_2;
volatile float32_t paddle_acc_1;
volatile float32_t paddle_acc_2;

float32_t prev_angle_1   = 0.0f;
float32_t prev_angle_2   = 0.0f;
float32_t prev_velocity_1 = 0.0f;
float32_t prev_velocity_2 = 0.0f;

float32_t filtred_vel    = 0.0f;
float32_t prev_fil_vel   = 0.0f;
float32_t filtred_acc    = 0.0f;
float32_t prev_fil_acc   = 0.0f;

float32_t comp_filtred_vel_dry    = 0.0f;
float32_t comp_prev_fil_vel_dry   = 0.0f;
float32_t comp_filtred_vel_visc    = 0.0f;
float32_t comp_prev_fil_vel_visc   = 0.0f;

// ---------------------------------------------------------------------------
// PID Position Controller
// ---------------------------------------------------------------------------
volatile uint32_t  pid_enabled         = 0;
volatile uint32_t  pid_use_hall        = 0;
volatile float32_t pid_target_angle    = 0.0f;
volatile float32_t pid_Kp              = 0.001f;
volatile float32_t pid_Ki              = 0.001f;
volatile float32_t pid_Kd              = 0.0005f;
volatile float32_t pid_filter_tao      = 1.0f / (50.0f * 2.0f * 3.14159265f);
volatile float32_t pid_error           = 0.0f;
volatile float32_t pid_torque_output   = 0.0f;
volatile float32_t pid_measured_angle  = 0.0f;

static float32_t pid_integral          = 0.0f;
static float32_t pid_prev_error        = 0.0f;
static float32_t pid_filtered_pos      = 0.0f;

void hapt_Update(void);

// -----------------------------------------------------------------------
// Dry friction estimation
// -----------------------------------------------------------------------
volatile uint32_t stiction_mode  = 0;
volatile float32_t stiction_step  = 0.001f;
volatile float32_t stiction_found = 0.0f;
volatile float32_t vel_threshold  = 5.0f;

static float32_t stict_torque = 0.0f;
static uint32_t  stict_hold   = 0;

// -----------------------------------------------------------------------
// Friction and gravity compensation
// -----------------------------------------------------------------------
#define PADDLE_MGL_DEFAULT  0.01896f
#define PI_F                3.14159265f
#define DEG2RAD             (PI_F / 180.0f)

volatile uint32_t comp_dry_enabled      = 1;
volatile uint32_t comp_viscous_enabled  = 1;
volatile uint32_t comp_gravity_enabled  = 1;

float32_t tao_dry  = 1.0f / (100.0f * 2.0f * 3.14159265f);
float32_t tao_visc = 1.0f / (200.0f * 2.0f * 3.14159265f);

// NOTE: tao for the general velocity/accel filter is now derived from
// wall_filter_fc each control cycle so it tracks changes made via the
// remote interface in real time.
float32_t tao; // updated every cycle from wall_filter_fc

volatile float32_t comp_Fc_ccw         = 0.0009f;
volatile float32_t comp_Fc_cw          = 0.0006f;
volatile float32_t comp_b              = 0.0000029943f;
volatile float32_t comp_mgl            = PADDLE_MGL_DEFAULT;
volatile float32_t comp_vel_deadband   = 3.0f;
volatile float32_t comp_torque_total   = 0.0f;

// ---------------------------------------------------------------------------
// Virtual Wall Parameters
// ---------------------------------------------------------------------------
// Wall trigger angle (symmetric) [deg].
#define WALL_ANGLE          15.0f

// Virtual spring stiffness K [N.m/deg]: restoring torque per degree of penetration.
// Tunable from the remote-control interface.
volatile float32_t wall_K = 0.005f;

// Virtual damping coefficient B [N.m/(deg/s)]: dissipative torque proportional
// to the penetration velocity. Tunable from the remote-control interface.
volatile float32_t wall_B = 0.0001f;

// Wall filter cut-off frequency [Hz]: shared first-order IIR applied to both
// position and velocity before the wall law, to suppress quantisation noise.
// Tunable from the remote-control interface.
volatile float32_t wall_filter_fc = 200.0f;   // [Hz] – initial value

// Read-only diagnostics.
volatile float32_t wall_torque      = 0.0f;   // Torque produced by the wall this cycle [N.m].
volatile float32_t wall_penetration = 0.0f;   // Current wall penetration depth [deg] (>= 0).

// Internal wall filter state.
static float32_t wall_filtered_angle = 0.0f;
static float32_t wall_filtered_vel   = 0.0f;

// -----------------------------------------------------------------------
// KB Plot Sweep
// -----------------------------------------------------------------------
#define KB_NUM_POINTS 10

// Sweep control – all READWRITE from the remote interface.
//
//   kb_sweep_enabled    : write 1 to start (or restart) the sweep,
//                         0 to abort at any time.
//   kb_vibration_detect : write 1 the moment you feel sustained vibration;
//                         the code records the current (B, K) pair, clears
//                         this flag automatically, increments B, resets K
//                         to 0 and starts the next ramp.
//   kb_K_sweep_rate     : how fast K rises [N.m/deg per second].
//                         Default 0.002 => ~0.1 N.m/deg in ~50 s at 350 us.
//                         Increase if the sweep is too slow for your patience.
//   kb_B_start          : damping value used for the very first point.
//   kb_B_step           : how much B is incremented after each recorded point.
volatile uint32_t  kb_sweep_enabled    = 0;
volatile uint32_t  kb_vibration_detect = 0;
volatile float32_t kb_K_sweep_rate     = 0.002f;  // [N.m/deg/s]
volatile float32_t kb_B_start          = 0.0f;    // [N.m/(deg/s)]
volatile float32_t kb_B_step           = 0.0001f; // [N.m/(deg/s)]

// Sweep diagnostics – READONLY.
volatile uint32_t  kb_point_index      = 0;    // Points stored so far (0–10).
volatile float32_t kb_K_current        = 0.0f; // K currently being swept [N.m/deg].
volatile float32_t kb_B_current        = 0.0f; // B fixed for this sweep [N.m/(deg/s)].

// Stored results: 10 K values and 10 B values – READONLY.
volatile float32_t kb_K_results[KB_NUM_POINTS] = {0.0f};
volatile float32_t kb_B_results[KB_NUM_POINTS] = {0.0f};

// Experiment mode selector – READWRITE.
//
//   0 = user-controlled (no auto-configuration)
//   1 = A : no velocity filter,  sampling period = 350 us
//   2 = B : no velocity filter,  sampling period = 10 000 us (10 ms)
//   3 = C : 50 Hz LP filter,     sampling period = 350 us
//   4 = D : 50 Hz LP filter,     sampling period = 350 us,
//           friction & gravity compensation DISABLED
//
// Writing a mode value configures wall_filter_fc, the timer period and
// (for mode D) the compensation enable flags in a single step so you can
// move between the four experimental conditions without manual tuning.
volatile uint32_t  kb_experiment_mode  = 0;

// Internal sweep tracking state.
static uint32_t kb_sweep_was_enabled = 0;
static uint32_t kb_prev_exp_mode     = 0;


/**
  * @brief Initializes the haptic controller.
  */
void hapt_Init(void)
{
    hapt_timestamp  = 0;
    hapt_motorTorque = 0.0f;

    // Initialise tao from the default wall_filter_fc.
    tao = 1.0f / (wall_filter_fc * 2.0f * PI_F);

    // Reset PID internal state.
    pid_integral     = 0.0f;
    pid_prev_error   = 0.0f;
    pid_filtered_pos = 0.0f;

    // Reset wall filter state.
    wall_filtered_angle = 0.0f;
    wall_filtered_vel   = 0.0f;

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
    comm_monitorUint32("pid_enabled",           (uint32_t*)&pid_enabled,       READWRITE);
    comm_monitorUint32("pid_use_hall",          (uint32_t*)&pid_use_hall,      READWRITE);
    comm_monitorFloat("pid_target_angle [deg]", (float32_t*)&pid_target_angle, READWRITE);
    comm_monitorFloat("pid_Kp",                 (float32_t*)&pid_Kp,           READWRITE);
    comm_monitorFloat("pid_Ki",                 (float32_t*)&pid_Ki,           READWRITE);
    comm_monitorFloat("pid_Kd",                 (float32_t*)&pid_Kd,           READWRITE);
    comm_monitorFloat("pid_filter_tao [s]",     (float32_t*)&pid_filter_tao,   READWRITE);
    comm_monitorFloat("pid_error [deg]",        (float32_t*)&pid_error,         READONLY);
    comm_monitorFloat("pid_torque [N.m]",       (float32_t*)&pid_torque_output, READONLY);
    comm_monitorFloat("pid_meas_angle [deg]",   (float32_t*)&pid_measured_angle,READONLY);

    // --- Friction & gravity compensation ---
    comm_monitorUint32("comp_dry_enabled",     (uint32_t*)&comp_dry_enabled,     READWRITE);
    comm_monitorUint32("comp_viscous_enabled", (uint32_t*)&comp_viscous_enabled, READWRITE);
    comm_monitorUint32("comp_gravity_enabled", (uint32_t*)&comp_gravity_enabled, READWRITE);
    comm_monitorFloat("comp_Fc_cw [N.m]",      (float32_t*)&comp_Fc_cw,          READWRITE);
    comm_monitorFloat("comp_Fc_ccw [N.m]",     (float32_t*)&comp_Fc_ccw,         READWRITE);
    comm_monitorFloat("comp_b [N.m.s/deg]",    (float32_t*)&comp_b,              READWRITE);
    comm_monitorFloat("comp_mgl [N.m]",        (float32_t*)&comp_mgl,            READWRITE);
    comm_monitorFloat("comp_vel_deadband [d/s]",(float32_t*)&comp_vel_deadband,  READWRITE);
    comm_monitorFloat("comp_velocityy",        (float32_t*)&comp_filtred_vel_visc,READONLY);
    comm_monitorFloat("comp_torque_total [N.m]",(float32_t*)&comp_torque_total,  READONLY);

    // -----------------------------------------------------------------------
    // Virtual Wall – tunable parameters (remote-control interface)
    // -----------------------------------------------------------------------
    // Spring stiffness: higher = stiffer wall, but risks instability.
    comm_monitorFloat("wall_K [N.m/deg]",      (float32_t*)&wall_K,           READWRITE);

    // Damping factor: adds energy dissipation to stabilise high-K walls.
    comm_monitorFloat("wall_B [N.m/(deg/s)]",  (float32_t*)&wall_B,           READWRITE);

    // Filter cut-off frequency: lower = smoother velocity, more phase lag;
    // higher = more responsive, but noisier.
    comm_monitorFloat("wall_filter_fc [Hz]",   (float32_t*)&wall_filter_fc,   READWRITE);

    // Read-only diagnostics.
    comm_monitorFloat("wall_torque [N.m]",     (float32_t*)&wall_torque,      READONLY);
    comm_monitorFloat("wall_penetration [deg]",(float32_t*)&wall_penetration, READONLY);

    // -----------------------------------------------------------------------
    // KB Plot Sweep – control, diagnostics and stored results
    // -----------------------------------------------------------------------

    // Sweep control.
    comm_monitorUint32("kb_sweep_enabled",           (uint32_t*)&kb_sweep_enabled,    READWRITE);
    comm_monitorUint32("kb_vibration_detect",        (uint32_t*)&kb_vibration_detect, READWRITE);
    comm_monitorFloat("kb_K_sweep_rate [N.m/deg/s]", (float32_t*)&kb_K_sweep_rate,    READWRITE);
    comm_monitorFloat("kb_B_start [N.m/(d/s)]",      (float32_t*)&kb_B_start,         READWRITE);
    comm_monitorFloat("kb_B_step  [N.m/(d/s)]",      (float32_t*)&kb_B_step,          READWRITE);

    // Sweep diagnostics.
    comm_monitorUint32("kb_point_index",              (uint32_t*)&kb_point_index,      READONLY);
    comm_monitorFloat("kb_K_current [N.m/deg]",       (float32_t*)&kb_K_current,       READONLY);
    comm_monitorFloat("kb_B_current [N.m/(d/s)]",     (float32_t*)&kb_B_current,       READONLY);

    // Stored K results (one per point, 0–9).
    comm_monitorFloat("kb_K_result_0",  (float32_t*)&kb_K_results[0],  READONLY);
    comm_monitorFloat("kb_K_result_1",  (float32_t*)&kb_K_results[1],  READONLY);
    comm_monitorFloat("kb_K_result_2",  (float32_t*)&kb_K_results[2],  READONLY);
    comm_monitorFloat("kb_K_result_3",  (float32_t*)&kb_K_results[3],  READONLY);
    comm_monitorFloat("kb_K_result_4",  (float32_t*)&kb_K_results[4],  READONLY);
    comm_monitorFloat("kb_K_result_5",  (float32_t*)&kb_K_results[5],  READONLY);
    comm_monitorFloat("kb_K_result_6",  (float32_t*)&kb_K_results[6],  READONLY);
    comm_monitorFloat("kb_K_result_7",  (float32_t*)&kb_K_results[7],  READONLY);
    comm_monitorFloat("kb_K_result_8",  (float32_t*)&kb_K_results[8],  READONLY);
    comm_monitorFloat("kb_K_result_9",  (float32_t*)&kb_K_results[9],  READONLY);

    // Stored B results (one per point, 0–9).
    comm_monitorFloat("kb_B_result_0",  (float32_t*)&kb_B_results[0],  READONLY);
    comm_monitorFloat("kb_B_result_1",  (float32_t*)&kb_B_results[1],  READONLY);
    comm_monitorFloat("kb_B_result_2",  (float32_t*)&kb_B_results[2],  READONLY);
    comm_monitorFloat("kb_B_result_3",  (float32_t*)&kb_B_results[3],  READONLY);
    comm_monitorFloat("kb_B_result_4",  (float32_t*)&kb_B_results[4],  READONLY);
    comm_monitorFloat("kb_B_result_5",  (float32_t*)&kb_B_results[5],  READONLY);
    comm_monitorFloat("kb_B_result_6",  (float32_t*)&kb_B_results[6],  READONLY);
    comm_monitorFloat("kb_B_result_7",  (float32_t*)&kb_B_results[7],  READONLY);
    comm_monitorFloat("kb_B_result_8",  (float32_t*)&kb_B_results[8],  READONLY);
    comm_monitorFloat("kb_B_result_9",  (float32_t*)&kb_B_results[9],  READONLY);

    // Experiment mode selector.
    comm_monitorUint32("kb_experiment_mode",         (uint32_t*)&kb_experiment_mode,  READWRITE);
}

/**
  * @brief Updates the haptic controller state.
  */
void hapt_Update(void)
{
    float32_t motorShaftAngle;

    // Compute dt [s].
    float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f;

    // Recompute tao from the (possibly updated) wall_filter_fc each cycle.
    // This makes the filter cut-off immediately responsive to remote changes.
    tao = 1.0f / (wall_filter_fc * 2.0f * PI_F);

    // -----------------------------------------------------------------------
    // Experiment mode – apply settings once on each mode change
    //
    //   Mode 1 (A): no velocity filter  (fc = 1 MHz -> alpha ≈ 1),  350 us
    //   Mode 2 (B): no velocity filter,                             10 ms
    //   Mode 3 (C): 50 Hz LP filter,                                350 us
    //   Mode 4 (D): 50 Hz LP filter,  compensation OFF,             350 us
    // -----------------------------------------------------------------------
    if (kb_experiment_mode != kb_prev_exp_mode)
    {
        kb_prev_exp_mode = kb_experiment_mode;
        switch (kb_experiment_mode)
        {
            case 1: /* A – no filter, 350 us */
                wall_filter_fc = 1.0e6f;
                cbt_SetHapticControllerPeriod(350);
                break;
            case 2: /* B – no filter, 10 ms */
                wall_filter_fc = 1.0e6f;
                cbt_SetHapticControllerPeriod(10000);
                break;
            case 3: /* C – 50 Hz filter, 350 us */
                wall_filter_fc = 50.0f;
                cbt_SetHapticControllerPeriod(350);
                break;
            case 4: /* D – 50 Hz filter, no compensation, 350 us */
                wall_filter_fc       = 50.0f;
                cbt_SetHapticControllerPeriod(350);
                comp_dry_enabled     = 0;
                comp_viscous_enabled = 0;
                comp_gravity_enabled = 0;
                break;
            default:
                break;
        }
        // tao will be refreshed at the top of the next cycle from the new wall_filter_fc.
    }

    // Increment the timestamp.
    hapt_timestamp += cbt_GetHapticControllerPeriod();

    // Get the Hall sensor voltage and derive angle.
    hapt_hallVoltage = hall_GetVoltage();
    relative_angle   = hapt_hallVoltage * 48.7132f - 96.7068f - zero_angle;

    // Get the encoder position.
    motorShaftAngle         = enc_GetPosition();
    hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;

    // -----------------------------------------------------------------------
    // Velocity and acceleration estimation
    // -----------------------------------------------------------------------
    float32_t angle = hapt_encoderPaddleAngle;

    paddle_velocity_1 = (angle - prev_angle_1) / dt;
    paddle_velocity_2 = (angle - prev_angle_2) / (2.0f * dt);

    paddle_acc_1 = (paddle_velocity_1 - prev_velocity_1) / dt;
    paddle_acc_2 = (paddle_velocity_1 - prev_velocity_2) / (2.0f * dt);

    prev_angle_2    = prev_angle_1;
    prev_angle_1    = angle;
    prev_velocity_2 = prev_velocity_1;
    prev_velocity_1 = paddle_velocity_1;

    // General velocity/acceleration low-pass filter (cut-off = wall_filter_fc).
    float32_t alpha  = dt / (tao + dt);
    filtred_vel  = alpha * paddle_velocity_1 + (1.0f - alpha) * prev_fil_vel;
    prev_fil_vel = filtred_vel;
    filtred_acc  = alpha * paddle_acc_1 + (1.0f - alpha) * prev_fil_acc;
    prev_fil_acc = filtred_acc;

    // -----------------------------------------------------------------------
    // PID Position Controller
    // -----------------------------------------------------------------------
    if (pid_target_angle < PADDLE_ANGLE_MIN) pid_target_angle = PADDLE_ANGLE_MIN;
    if (pid_target_angle > PADDLE_ANGLE_MAX) pid_target_angle = PADDLE_ANGLE_MAX;

    if (pid_enabled)
    {
        float32_t raw_pos = pid_use_hall ? relative_angle : hapt_encoderPaddleAngle;

        float32_t alpha_pid = dt / (pid_filter_tao + dt);
        pid_filtered_pos    = alpha_pid * raw_pos + (1.0f - alpha_pid) * pid_filtered_pos;
        pid_measured_angle  = pid_filtered_pos;

        float32_t error = pid_target_angle - pid_filtered_pos;
        pid_error = error;

        pid_integral += error * dt;

        float32_t error_derivative = (error - pid_prev_error) / dt;
        pid_prev_error = error;

        float32_t torque_cmd = pid_Kp * error
                             + pid_Ki * pid_integral
                             + pid_Kd * error_derivative;

        if (torque_cmd >  PID_TORQUE_SAT) torque_cmd =  PID_TORQUE_SAT;
        if (torque_cmd < -PID_TORQUE_SAT) torque_cmd = -PID_TORQUE_SAT;

        pid_torque_output = torque_cmd;
        hapt_motorTorque  = torque_cmd;
    }
    else
    {
        pid_integral      = 0.0f;
        pid_prev_error    = 0.0f;
        pid_torque_output = 0.0f;
    }

    // -----------------------------------------------------------------------
    // Dry friction estimation
    // -----------------------------------------------------------------------
    if (stiction_mode != 0 && !pid_enabled)
    {
        stict_hold++;
        if (stict_hold >= 10)
        {
            stict_hold = 0;
            float32_t dir = (stiction_mode == 1) ? 1.0f : -1.0f;

            if (fabsf(filtred_vel) > vel_threshold)
            {
                stiction_found   = stict_torque;
                stict_torque     = 0.0f;
                stiction_mode    = 0;
                hapt_motorTorque = 0.0f;
            }
            else
            {
                stict_torque    += dir * stiction_step;
                hapt_motorTorque = stict_torque;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Friction and gravity compensation
    // -----------------------------------------------------------------------
    float32_t tau_comp = 0.0f;
    hapt_motorTorque   = 0.0f;

    float32_t alpha_dry  = dt / (tao_dry  + dt);
    float32_t alpha_visc = dt / (tao_visc + dt);

    comp_filtred_vel_dry   = alpha_dry  * paddle_velocity_1 + (1.0f - alpha_dry)  * comp_prev_fil_vel_dry;
    comp_prev_fil_vel_dry  = comp_filtred_vel_dry;
    comp_filtred_vel_visc  = alpha_visc * paddle_velocity_1 + (1.0f - alpha_visc) * comp_prev_fil_vel_visc;
    comp_prev_fil_vel_visc = comp_filtred_vel_visc;

    // 1. Dry friction compensation
    if (comp_dry_enabled)
    {
        if (comp_filtred_vel_dry > comp_vel_deadband)
            tau_comp += comp_Fc_ccw;
        else if (comp_filtred_vel_dry < -comp_vel_deadband)
            tau_comp -= comp_Fc_cw;
    }

    // 2. Viscous friction compensation
    if (comp_viscous_enabled)
    {
        tau_comp += comp_b * comp_filtred_vel_visc / 15.0f;
    }

    // 3. Gravity compensation:  τ = m·g·L · sin(θ)
    if (comp_gravity_enabled)
    {
        float32_t theta_rad = hapt_encoderPaddleAngle * DEG2RAD;
        tau_comp += comp_mgl * sinf(theta_rad) / REDUCTION_RATIO;
    }

    comp_torque_total = tau_comp;
    hapt_motorTorque += tau_comp;

    // -----------------------------------------------------------------------
    // KB Plot Sweep – auto-ramp K for a fixed B; user signals instability
    //
    // Procedure:
    //   1. Set kb_B_start and kb_B_step, then write 1 to kb_sweep_enabled.
    //   2. The code starts with B = kb_B_start and K = 0, then increases K
    //      at kb_K_sweep_rate [N.m/deg/s] every control cycle.
    //   3. When you feel sustained vibration, write 1 to kb_vibration_detect.
    //   4. The controller stores the (B, K) pair, increments B by kb_B_step,
    //      resets K to 0 and begins the next ramp automatically.
    //   5. After 10 points, kb_sweep_enabled is cleared automatically.
    //      Results are in kb_K_result_0 … kb_K_result_9 (K_unstable) and
    //      kb_B_result_0 … kb_B_result_9 (the matching B value).
    //
    // To abort at any time: write 0 to kb_sweep_enabled.
    // -----------------------------------------------------------------------

    // Detect rising edge of kb_sweep_enabled to (re-)initialise the sweep.
    if (kb_sweep_enabled && !kb_sweep_was_enabled)
    {
        kb_point_index = 0;
        kb_K_current   = 0.0f;
        kb_B_current   = kb_B_start;
        wall_K         = 0.0f;
        wall_B         = kb_B_current;
    }
    kb_sweep_was_enabled = kb_sweep_enabled;

    if (kb_sweep_enabled && (kb_point_index < KB_NUM_POINTS))
    {
        if (kb_vibration_detect)
        {
            // Record the (B, K_unstable) pair for the current point.
            kb_K_results[kb_point_index] = kb_K_current;
            kb_B_results[kb_point_index] = kb_B_current;
            kb_point_index++;

            kb_vibration_detect = 0; // Acknowledge flag – user must re-arm.

            if (kb_point_index >= KB_NUM_POINTS)
            {
                // All 10 points collected – stop sweep automatically.
                kb_sweep_enabled = 0;
            }
            else
            {
                // Advance to the next damping value and restart K from zero.
                kb_B_current += kb_B_step;
                kb_K_current  = 0.0f;
                wall_K        = 0.0f;
                wall_B        = kb_B_current;
            }
        }
        else
        {
            // Ramp K upward at the configured rate.
            kb_K_current += kb_K_sweep_rate * dt;
            wall_K        = kb_K_current;
            wall_B        = kb_B_current;
        }
    }

    // -----------------------------------------------------------------------
    // Virtual Viscoelastic Walls at φ_wall = ±15°
    //
    // Model:  τ_wall = −K · δ  −  B · δ̇
    //
    //   δ   = signed penetration beyond the wall boundary [deg]
    //         δ > 0 when inside the positive wall (angle > +15°)
    //         δ < 0 when inside the negative wall (angle < −15°)
    //   δ̇   = filtered velocity [deg/s], used only when the paddle is
    //         actively penetrating the wall (δ and δ̇ have the same sign)
    //         to avoid the damper pulling the paddle deeper during rebound.
    //
    // The wall filter (shared cut-off wall_filter_fc) is applied to both
    // the measured angle and the velocity estimate so that high-frequency
    // quantisation noise does not create torque spikes at high K values.
    // -----------------------------------------------------------------------

    // Update the wall's dedicated low-pass filters.
    // alpha is already computed above from tao = 1/(wall_filter_fc · 2π).
    wall_filtered_angle = alpha * hapt_encoderPaddleAngle
                        + (1.0f - alpha) * wall_filtered_angle;
    wall_filtered_vel   = alpha * paddle_velocity_1
                        + (1.0f - alpha) * wall_filtered_vel;

    float32_t tau_wall = 0.0f;

    if (wall_filtered_angle > WALL_ANGLE)
    {
        // ---- Positive wall ----
        float32_t delta = wall_filtered_angle - WALL_ANGLE;   // penetration depth [deg], > 0
        wall_penetration = delta;

        // Spring term: always pushes back.
        tau_wall = -wall_K * delta;

        // Damper term: only when penetrating (positive velocity while in positive wall).
        // During rebound (negative velocity) the damper is inactive so it does not
        // resist the natural exit, which would destabilise the interaction.
        if (wall_filtered_vel > 0.0f)
            tau_wall -= wall_B * wall_filtered_vel;
    }
    else if (wall_filtered_angle < -WALL_ANGLE)
    {
        // ---- Negative wall ----
        float32_t delta = wall_filtered_angle + WALL_ANGLE;   // penetration depth [deg], < 0
        wall_penetration = -delta;                             // report as positive magnitude

        // Spring term: always pushes back.
        tau_wall = -wall_K * delta;

        // Damper term: only when penetrating (negative velocity in negative wall).
        if (wall_filtered_vel < 0.0f)
            tau_wall -= wall_B * wall_filtered_vel;
    }
    else
    {
        // Paddle is between the two walls – no wall torque.
        wall_penetration = 0.0f;
    }

    wall_torque = tau_wall;
    hapt_motorTorque += tau_wall;

    // Apply the final motor torque.
    torq_SetTorque(hapt_motorTorque);
}
