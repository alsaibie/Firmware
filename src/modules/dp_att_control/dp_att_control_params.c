/****************************************************************************
 *
 *   Copyright (c) 2016-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file dolphin_att_control_params.c
 * Parameters for Dolphin attitude controller.
 *
 * Adopted from mc_att_control_params.c
 * Modified by:
 * @author Ali AlSaibie <alsaibie@gatech.edu>
 */

/**
 * Roll P gain x 100
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 30
 * @decimal 2
 * @increment 0.05
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ROLL_P, 15.0f);

/**
 * Roll feed forward x100
 *
 * Feed forward weight for manual roll control. 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ROLL_FF, 1.0f);

/**
 * Roll rate P gain x100
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 5
 * @increment 0.05
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ROLLRATE_P, 2.5f);

/**
 * Roll rate I gain x100
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.1
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ROLLRATE_I, 10.0f);

/**
 * Roll rate integrator limit x100
 *
 * Roll rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large roll moment trim changes.
 *
 * @min 0.0
 * @max 120
 * @decimal 2
 * @increment 0.1
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_RR_INT_LIM, 100.0f);

/**
 * Roll rate D gain x100
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.02
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ROLLRATE_D, 0.01f);

/**
 * Roll rate feedforward x100
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @max 30.0
 * @decimal 2
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ROLLRATE_FF, 10.0f);

/**
 * Pitch P gain x100
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 50
 * @decimal 2
 * @increment 0.5
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_PITCH_P, 2.50f);

/**
 * Pitch rate P gain x100
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 5
 * @increment 0.05
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_PITCHRATE_P, 2.5f);

/**
 * Pitch rate I gain x100
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @max 50
 * @decimal 3
 * @increment 0.1
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_PITCHRATE_I, 5.0f);

/**
 * Pitch rate integrator limit x100
 *
 * Pitch rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large pitch moment trim changes.
 *
 * @min 0.0
 * @max 100
 * @decimal 2
 * @increment 1.0
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_PR_INT_LIM, 50.0f);

/**
 * Pitch rate D gain x100
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.1
 * @decimal 5
 * @increment 0.005
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_PITCHRATE_D, 0.001f);

/**
 * Pitch rate feedforward x100
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.05
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_PITCHRATE_FF, 0.05f);

/**
 * Yaw P gain x100
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 50
 * @decimal 2
 * @increment 0.1
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_YAW_P, 2.5f);

/**
 * Yaw rate P gain x100
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 4.0
 * @decimal 4
 * @increment 0.001
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_YAWRATE_P, 2.5f);

/**
 * Yaw rate I gain x100
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @max 50
 * @decimal 2
 * @increment 0.1
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_YAWRATE_I, 5.0f);

/**
 * Yaw rate integrator limit x100
 *
 * Yaw rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 1.0
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_YR_INT_LIM, 50.0f);

/**
 * Yaw rate D gain x100
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 5
 * @increment 0.0005
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_YAWRATE_D, 0.001f);

/**
 * Yaw rate feedforward x100
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_YAWRATE_FF, 5.0f);


/**
 * Max roll rate
 *
 * Limit for roll rate, has effect for large rotations in autonomous mode, to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ROLLRATE_MAX, 360.0f);

/**
 * Max pitch rate
 *
 * Limit for pitch rate, has effect for large rotations in autonomous mode, to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_PITCHRATE_MAX, 360.0f);

/**
 * Max yaw rate
 *
 * A value of significantly over 120 degrees per second can already lead to mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_YAWRATE_MAX, 360.0f);

/**
 * Max roll rate in auto mode
 *
 * Limit for yaw rate, has effect for large rotations in autonomous mode,
 * to avoid large control output and mixer saturation. A value of significantly
 * over 60 degrees per second can already lead to mixer saturation.
 * A value of 30 degrees / second is recommended to avoid very audible twitches.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ROLLRAUTO_MAX, 180.0f);

/**
 * Max acro roll rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1000.0
 * @decimal 1
 * @increment 5
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ACRO_R_MAX, 360.0f);

/**
 * Max acro pitch rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1000.0
 * @decimal 1
 * @increment 5
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ACRO_P_MAX, 360.0f);

/**
 * Max acro yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1000.0
 * @decimal 1
 * @increment 5
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ACRO_Y_MAX, 360.0f);
/**
 * Acro mode Expo factor for Pitch and Yaw.
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ACRO_EXPO, 0.69f);

/**
 * Acro mode Expo factor for Roll.
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ACRO_EXPO_R, 0.69f);

/**
 * Acro mode SuperExpo factor for Pitch and Yaw.
 *
 * SuperExpo factor for refining the input curve shape tuned using MC_ACRO_EXPO.
 *
 * 0 Pure Expo function
 * 0.7 resonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ACRO_SUPEXPO, 0.7f);

/**
 * Acro mode SuperExpo factor for Roll.
 *
 * SuperExpo factor for refining the input curve shape tuned using MC_ACRO_EXPO_Y.
 *
 * 0 Pure Expo function
 * 0.7 resonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ACRO_SUPEXPOR, 0.7f);

/**
 * Threshold for Rattitude mode
 *
 * Manual input needed in order to override attitude control rate setpoints
 * and instead pass manual stick inputs as rate setpoints
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_RATT_TH, 1.0f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_INT32(DP_BAT_SCALE_EN, 0);

/**
 * TPA P Breakpoint
 *
 * Throttle PID Attenuation (TPA)
 * Magnitude of throttle setpoint at which to begin attenuating roll/pitch P gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_TPA_BREAK_P, 1.0f);

/**
 * TPA I Breakpoint
 *
 * Throttle PID Attenuation (TPA)
 * Magnitude of throttle setpoint at which to begin attenuating roll/pitch I gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_TPA_BREAK_I, 1.0f);

/**
 * TPA D Breakpoint
 *
 * Throttle PID Attenuation (TPA)
 * Magnitude of throttle setpoint at which to begin attenuating roll/pitch D gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_TPA_BREAK_D, 1.0f);

/**
 * TPA Rate P
 *
 * Throttle PID Attenuation (TPA)
 * Rate at which to attenuate roll/pitch P gain
 * Attenuation factor is 1.0 when throttle magnitude is below the setpoint
 * Above the setpoint, the attenuation factor is (1 - rate * (throttle - breakpoint) / (1.0 - breakpoint))
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_TPA_RATE_P, 0.0f);

/**
 * TPA Rate I
 *
 * Throttle PID Attenuation (TPA)
 * Rate at which to attenuate roll/pitch I gain
 * Attenuation factor is 1.0 when throttle magnitude is below the setpoint
 * Above the setpoint, the attenuation factor is (1 - rate * (throttle - breakpoint) / (1.0 - breakpoint))
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_TPA_RATE_I, 0.0f);

/**
 * TPA Rate D
 *
 * Throttle PID Attenuation (TPA)
 * Rate at which to attenuate roll/pitch D gain
 * Attenuation factor is 1.0 when throttle magnitude is below the setpoint
 * Above the setpoint, the attenuation factor is (1 - rate * (throttle - breakpoint) / (1.0 - breakpoint))
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_TPA_RATE_D, 0.0f);

/**
 * Thrust Factor
 *
 * @unit na
 * @min 0
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_THRUST_FACTOR, 0.25f);

/**
 * Idle Speed - as % of Max PWM
 *
 * @unit na
 * @min 0
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_IDLE_SPEED, 0.15f);

/**
 * Cutoff frequency for the low pass filter on the D-term in the rate controller
 *
 * The D-term uses the derivative of the rate and thus is the most susceptible to noise.
 * Therefore, using a D-term filter allows to decrease the driver-level filtering, which
 * leads to reduced control latency and permits to increase the P gains.
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 1000
 * @decimal 0
 * @increment 10
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_DTERM_CUTOFF, 25.0f);


/**
 * Normalized thrust to normalized voltage polynomial c0 coefficient
 *
 * @unit na
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_V_HZ_N_C0, 1.68E-01f);

/**
 * Normalized thrust to normalized voltage polynomial c1 coefficient
 *
 * @unit na
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_V_HZ_N_C1, -2.21E-01f);

/**
 * Normalized thrust to normalized voltage polynomial c2 coefficient
 *
 * @unit na
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_V_HZ_N_C2, 1.01f);

/**
 * Thrust to Voltage polynomial c0 coefficient
 *
 * @unit na
 * @decimal 2
 * @group Dolphin Attitude Control
 */

PARAM_DEFINE_FLOAT(DP_V_T_C0, 1.57E+00f);

/**
 * Thrust to Voltage polynomial c1 coefficient
 *
 * @unit na
 * @decimal 2
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_V_T_C1, 3.11E+00f);

/**
 * Thrust to Voltage polynomial c2 coefficient
 *
 * @unit na
 * @decimal 2
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_V_T_C2, 3.51E-01f);

/**
 * Torque to Voltage polynomial c2 coefficient
 *
 * @unit na
 * @decimal 4
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_V_TORQ_C2, 2.182E+04f);

/**
 * Rotors Radius in meter
 *
 * @unit m
 * @min 0
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_FLOAT(DP_ROTOR_R, 0.03f);