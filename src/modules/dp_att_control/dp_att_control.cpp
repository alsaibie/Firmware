/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the mc att module and fw wing app and it is designed for underwater robots.
 *
 * All the acknowledgments and credits for the mc and fw wing app are reported in those files.
 *
 * @author Ali AlSaibie <alsaibie@gatech.edu>
 *
 */
#include <float.h>
#include "dp_att_control.hpp"

/**
 * Dolphin attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int dp_att_control_main(int argc, char *argv[]);

namespace dp_att_control {

    DolphinAttitudeControl *g_control;
}

DolphinAttitudeControl::DolphinAttitudeControl() :

        _task_should_exit(false),
        _task_running(false),
        _control_task(-1),

        /* subscriptions */
        _ctrl_state_sub(-1),
        _v_att_sp_sub(-1),
        _v_control_mode_sub(-1),
        _params_sub(-1),
        _manual_control_sp_sub(-1),
        _armed_sub(-1),
        _vehicle_status_sub(-1),
        _motor_limits_sub(-1),
        _battery_status_sub(-1),
        _sensor_correction_sub(-1),

        /* gyro selection */
        _gyro_count(1),
        _selected_gyro(0),

        /* publications */
        _v_rates_sp_pub(nullptr),
        _actuators_0_pub(nullptr),
        _controller_status_pub(nullptr),
        _rates_sp_id(nullptr),
        _actuators_id(nullptr),

        _actuators_0_circuit_breaker_enabled(false),

        _ctrl_state{},
        _v_att_sp{},
        _v_rates_sp{},
        _manual_control_sp{},
        _v_control_mode{},
        _actuators{},
        _armed{},
        _vehicle_status{},
        _motor_limits{},
        _controller_status{},
        _battery_status{},
        _sensor_gyro{},
        _sensor_correction{},

        _saturation_status{},

        /* performance counters */
        _loop_perf(perf_alloc(PC_ELAPSED, "dp_att_control")),
        _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")){
  for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
    _sensor_gyro_sub[i] = -1;
  }

  _vehicle_status.is_rotary_wing = true;

  _params.att_p.zero();
  _params.rate_p.zero();
  _params.rate_i.zero();
  _params.rate_int_lim.zero();
  _params.rate_d.zero();
  _params.rate_ff.zero();
  _params.yaw_ff = 0.0f;
  _params.roll_rate_max = 0.0f;
  _params.pitch_rate_max = 0.0f;
  _params.yaw_rate_max = 0.0f;
  _params.dp_rate_max.zero();
  _params.auto_rate_max.zero();
  _params.acro_rate_max.zero();
  _params.rattitude_thres = 1.0f;
  _params.bat_scale_en = 0;

  _params.board_rotation = 0;

  _params.board_offset[0] = 0.0f;
  _params.board_offset[1] = 0.0f;
  _params.board_offset[2] = 0.0f;
  _params.thrust_factor = 0.0f;
  _params.idle_speed = 0.0f;
  _rates_prev.zero();
  _rates_sp.zero();
  _rates_sp_prev.zero();
  _rates_int.zero();
  _thrust_sp = 0.0f;
  _att_control.zero();
  _mixed_att_control.zero();

  for (int k = 0; k < _rotor_count; k++) {
    _rotors[k].roll_scale = _dolphin_x_table[k][0];
    _rotors[k].pitch_scale = _dolphin_x_table[k][1];
    _rotors[k].yaw_scale = _dolphin_x_table[k][2];
    _rotors[k].thrust_scale = _dolphin_x_table[k][3];
    _rotors[k].out_scale = _dolphin_x_table[k][4];
  }

  _I.identity();
  _board_rotation.identity();

  _params_handles.roll_p = param_find("DP_ROLL_P");
  _params_handles.roll_rate_p = param_find("DP_ROLLRATE_P");
  _params_handles.roll_rate_i = param_find("DP_ROLLRATE_I");
  _params_handles.roll_rate_integ_lim = param_find("DP_RR_INT_LIM");
  _params_handles.roll_rate_d = param_find("DP_ROLLRATE_D");
  _params_handles.roll_rate_ff = param_find("DP_ROLLRATE_FF");
  _params_handles.pitch_p = param_find("DP_PITCH_P");
  _params_handles.pitch_rate_p = param_find("DP_PITCHRATE_P");
  _params_handles.pitch_rate_i = param_find("DP_PITCHRATE_I");
  _params_handles.pitch_rate_integ_lim = param_find("DP_PR_INT_LIM");
  _params_handles.pitch_rate_d = param_find("DP_PITCHRATE_D");
  _params_handles.pitch_rate_ff = param_find("DP_PITCHRATE_FF");
  _params_handles.tpa_breakpoint_p = param_find("DP_TPA_BREAK_P");
  _params_handles.tpa_breakpoint_i = param_find("DP_TPA_BREAK_I");
  _params_handles.tpa_breakpoint_d = param_find("DP_TPA_BREAK_D");
  _params_handles.tpa_rate_p = param_find("DP_TPA_RATE_P");
  _params_handles.tpa_rate_i = param_find("DP_TPA_RATE_I");
  _params_handles.tpa_rate_d = param_find("DP_TPA_RATE_D");
  _params_handles.yaw_p = param_find("DP_YAW_P");
  _params_handles.yaw_rate_p = param_find("DP_YAWRATE_P");
  _params_handles.yaw_rate_i = param_find("DP_YAWRATE_I");
  _params_handles.yaw_rate_integ_lim = param_find("DP_YR_INT_LIM");
  _params_handles.yaw_rate_d = param_find("DP_YAWRATE_D");
  _params_handles.yaw_rate_ff = param_find("DP_YAWRATE_FF");
  _params_handles.yaw_ff = param_find("DP_YAW_FF");
  _params_handles.roll_rate_max = param_find("DP_ROLLRATE_MAX");
  _params_handles.pitch_rate_max = param_find("DP_PITCHRATE_MAX");
  _params_handles.yaw_rate_max = param_find("DP_YAWRATE_MAX");
  _params_handles.yaw_auto_max = param_find("DP_YAWRAUTO_MAX");
  _params_handles.acro_roll_max = param_find("DP_ACRO_R_MAX");
  _params_handles.acro_pitch_max = param_find("DP_ACRO_P_MAX");
  _params_handles.acro_yaw_max = param_find("DP_ACRO_Y_MAX");
  _params_handles.rattitude_thres = param_find("DP_RATT_TH");
  _params_handles.roll_tc = param_find("DP_ROLL_TC");
  _params_handles.pitch_tc = param_find("DP_PITCH_TC");
  _params_handles.thrust_factor = param_find("DP_THRUST_FACTOR");
  _params_handles.idle_speed = param_find("DP_IDLE_SPEED");
  _params_handles.bat_scale_en = param_find("DP_BAT_SCALE_EN");

  /* rotations */
  _params_handles.board_rotation = param_find("SENS_BOARD_ROT");

  /* rotation offsets */
  _params_handles.board_offset[0] = param_find("SENS_BOARD_X_OFF");
  _params_handles.board_offset[1] = param_find("SENS_BOARD_Y_OFF");
  _params_handles.board_offset[2] = param_find("SENS_BOARD_Z_OFF");



  /* fetch initial parameter values */
  parameters_update();

  /* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
  for (unsigned i = 0; i < 3; i++) {
    // used scale factors to unity
    _sensor_correction.gyro_scale_0[i] = 1.0f;
    _sensor_correction.gyro_scale_1[i] = 1.0f;
    _sensor_correction.gyro_scale_2[i] = 1.0f;
  }

}

DolphinAttitudeControl::~DolphinAttitudeControl() {
  if (_control_task != -1) {
    /* task wakes up every 100ms or so at the longest */
    _task_should_exit = true;

    /* wait for a second for the task to quit at our request */
    unsigned i = 0;

    do {
      /* wait 20ms */
      usleep(20000);

      /* if we have given up, kill it */
      if (++i > 50) {
        px4_task_delete(_control_task);
        break;
      }
    } while (_control_task != -1);
  }

  dp_att_control::g_control = nullptr;
}

int
DolphinAttitudeControl::parameters_update() {
  float v;

  float roll_tc, pitch_tc;

  param_get(_params_handles.roll_tc, &roll_tc);
  param_get(_params_handles.pitch_tc, &pitch_tc);

  /* roll gains */
  param_get(_params_handles.roll_p, &v);
  _params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
  param_get(_params_handles.roll_rate_p, &v);
  _params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
  param_get(_params_handles.roll_rate_i, &v);
  _params.rate_i(0) = v;
  param_get(_params_handles.roll_rate_integ_lim, &v);
  _params.rate_int_lim(0) = v;
  param_get(_params_handles.roll_rate_d, &v);
  _params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
  param_get(_params_handles.roll_rate_ff, &v);
  _params.rate_ff(0) = v;

  /* pitch gains */
  param_get(_params_handles.pitch_p, &v);
  _params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
  param_get(_params_handles.pitch_rate_p, &v);
  _params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
  param_get(_params_handles.pitch_rate_i, &v);
  _params.rate_i(1) = v;
  param_get(_params_handles.pitch_rate_integ_lim, &v);
  _params.rate_int_lim(1) = v;
  param_get(_params_handles.pitch_rate_d, &v);
  _params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
  param_get(_params_handles.pitch_rate_ff, &v);
  _params.rate_ff(1) = v;

  param_get(_params_handles.tpa_breakpoint_p, &_params.tpa_breakpoint_p);
  param_get(_params_handles.tpa_breakpoint_i, &_params.tpa_breakpoint_i);
  param_get(_params_handles.tpa_breakpoint_d, &_params.tpa_breakpoint_d);
  param_get(_params_handles.tpa_rate_p, &_params.tpa_rate_p);
  param_get(_params_handles.tpa_rate_i, &_params.tpa_rate_i);
  param_get(_params_handles.tpa_rate_d, &_params.tpa_rate_d);

  /* yaw gains */
  param_get(_params_handles.yaw_p, &v);
  _params.att_p(2) = v;
  param_get(_params_handles.yaw_rate_p, &v);
  _params.rate_p(2) = v;
  param_get(_params_handles.yaw_rate_i, &v);
  _params.rate_i(2) = v;
  param_get(_params_handles.yaw_rate_integ_lim, &v);
  _params.rate_int_lim(2) = v;
  param_get(_params_handles.yaw_rate_d, &v);
  _params.rate_d(2) = v;
  param_get(_params_handles.yaw_rate_ff, &v);
  _params.rate_ff(2) = v;

  param_get(_params_handles.yaw_ff, &_params.yaw_ff);

  /* angular rate limits */
  param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
  _params.dp_rate_max(0) = math::radians(_params.roll_rate_max);
  param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
  _params.dp_rate_max(1) = math::radians(_params.pitch_rate_max);
  param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
  _params.dp_rate_max(2) = math::radians(_params.yaw_rate_max);

  /* auto angular rate limits */
  param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
  _params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
  param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
  _params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
  param_get(_params_handles.yaw_auto_max, &_params.yaw_auto_max);
  _params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

  /* manual rate control scale and auto mode roll/pitch rate limits */
  param_get(_params_handles.acro_roll_max, &v);
  _params.acro_rate_max(0) = math::radians(v);
  param_get(_params_handles.acro_pitch_max, &v);
  _params.acro_rate_max(1) = math::radians(v);
  param_get(_params_handles.acro_yaw_max, &v);
  _params.acro_rate_max(2) = math::radians(v);

  /* stick deflection needed in rattitude mode to control rates not angles */
  param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);


  param_get(_params_handles.bat_scale_en, &_params.bat_scale_en);

  _actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

  /* rotation of the autopilot relative to the body */
  param_get(_params_handles.board_rotation, &(_params.board_rotation));

  /* fine adjustment of the rotation */
  param_get(_params_handles.board_offset[0], &(_params.board_offset[0]));
  param_get(_params_handles.board_offset[1], &(_params.board_offset[1]));
  param_get(_params_handles.board_offset[2], &(_params.board_offset[2]));

  param_get(_params_handles.thrust_factor, &_params.thrust_factor);
  param_get(_params_handles.idle_speed, &_params.idle_speed);

  return OK;
}

void
DolphinAttitudeControl::parameter_update_poll() {
  bool updated;

  /* Check if parameters have changed */
  orb_check(_params_sub, &updated);

  if (updated) {
    struct parameter_update_s param_update;
    orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
    parameters_update();
  }
}

void
DolphinAttitudeControl::vehicle_control_mode_poll() {
  bool updated;

  /* Check if vehicle control mode has changed */
  orb_check(_v_control_mode_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
  }
}

void
DolphinAttitudeControl::vehicle_manual_poll() {
  bool updated;

  /* get pilots inputs */
  orb_check(_manual_control_sp_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
  }
}

void
DolphinAttitudeControl::vehicle_attitude_setpoint_poll() {
  /* check if there is a new setpoint */
  bool updated;
  orb_check(_v_att_sp_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
  }
}

void
DolphinAttitudeControl::vehicle_rates_setpoint_poll() {
  /* check if there is a new setpoint */
  bool updated;
  orb_check(_v_rates_sp_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
  }
}

void
DolphinAttitudeControl::arming_status_poll() {
  /* check if there is a new setpoint */
  bool updated;
  orb_check(_armed_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
  }
}

void
DolphinAttitudeControl::vehicle_status_poll() {
  /* check if there is new status information */
  bool vehicle_status_updated;
  orb_check(_vehicle_status_sub, &vehicle_status_updated);

  if (vehicle_status_updated) {
    orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

    /* set correct uORB ID, depending on if vehicle is VTOL or not */
    if (!_rates_sp_id) {
      if (_vehicle_status.is_vtol) {
        _rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
        _actuators_id = ORB_ID(actuator_controls_virtual_mc);

      } else {
        _rates_sp_id = ORB_ID(vehicle_rates_setpoint);
        _actuators_id = ORB_ID(actuator_controls_0);
      }
    }
  }
}

void
DolphinAttitudeControl::vehicle_motor_limits_poll() {
  /* check if there is a new message */
  bool updated;
  orb_check(_motor_limits_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
    _saturation_status.value = _motor_limits.saturation_status;
  }
}

void
DolphinAttitudeControl::battery_status_poll() {
  /* check if there is a new message */
  bool updated;
  orb_check(_battery_status_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
  }
}

void
DolphinAttitudeControl::control_state_poll() {
  /* check if there is a new message */
  bool updated;
  orb_check(_ctrl_state_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
  }
}

void
DolphinAttitudeControl::sensor_correction_poll() {
  /* check if there is a new message */
  bool updated;
  orb_check(_sensor_correction_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
  }

  /* update the latest gyro selection */
  if (_sensor_correction.selected_gyro_instance < _gyro_count) {
    _selected_gyro = _sensor_correction.selected_gyro_instance;
  }
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode), ctrl_state (attitude)
 * Output: '_rates_sp' vector, '_thrust_sp'
 * TODO: Explicitly Pass Inputs and Outputs. Make it self-contained.
 */
void
DolphinAttitudeControl::control_attitude(float dt) {
  vehicle_attitude_setpoint_poll();

  _thrust_sp = _v_att_sp.thrust;

  /* construct attitude setpoint rotation matrix */
  math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
  math::Matrix<3, 3> R_sp = q_sp.to_dcm();

  /* get current rotation matrix from control state quaternions */
  math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
  math::Matrix<3, 3> R = q_att.to_dcm();

  /* all input data is ready, run controller itself */

  /* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
  math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
  math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

  /* axis and sin(angle) of desired rotation */
  math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z); //e_R is vector between current and desired

  /* calculate angle error */
  float e_R_z_sin = e_R.length(); // Vector Geometric length not array length
  float e_R_z_cos = R_z * R_sp_z; // Dot Product

  /* calculate weight for yaw control TODO: Understand this */
  float yaw_w = R_sp(2, 2) * R_sp(2, 2);

  /* calculate rotation matrix after roll/pitch only rotation */
  math::Matrix<3, 3> R_rp;

  if (e_R_z_sin > 0.0f) {
    /* get axis-angle representation */
    float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
    math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

    e_R = e_R_z_axis * e_R_z_angle;

    /* cross product matrix for e_R_axis */
    math::Matrix<3, 3> e_R_cp;
    e_R_cp.zero();
    e_R_cp(0, 1) = -e_R_z_axis(2);
    e_R_cp(0, 2) = e_R_z_axis(1);
    e_R_cp(1, 0) = e_R_z_axis(2);
    e_R_cp(1, 2) = -e_R_z_axis(0);
    e_R_cp(2, 0) = -e_R_z_axis(1);
    e_R_cp(2, 1) = e_R_z_axis(0);

    /* rotation matrix for roll/pitch only rotation */
    R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

  } else {
    /* zero roll/pitch rotation */
    R_rp = R;
  }

  /* R_rp and R_sp has the same Z axis, calculate yaw error */
  math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
  math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
  e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

  if (e_R_z_cos < 0.0f) {
    /* for large thrust vector rotations use another rotation method:
     * calculate angle and axis for R -> R_sp rotation directly */
    math::Quaternion q_error;
    q_error.from_dcm(R.transposed() * R_sp);
    math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag() * 2.0f : -q_error.imag() * 2.0f;

    /* use fusion of Z axis based rotation and direct rotation */
    float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
    e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
  }

  /* calculate angular rates setpoint */
  _rates_sp = _params.att_p.emult(e_R); // Multiply by p gain, no I nor D for angular error

  /* limit rates */
  for (int i = 0; i < 3; i++) {
    if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
        !_v_control_mode.flag_control_manual_enabled) {
      _rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));

    } else {
      _rates_sp(i) = math::constrain(_rates_sp(i), -_params.dp_rate_max(i), _params.dp_rate_max(i));
    }
  }

  /* feed forward yaw setpoint rate */
  _rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;

}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
math::Vector<3>
DolphinAttitudeControl::pid_attenuations(float tpa_breakpoint, float tpa_rate) {
  /* throttle pid attenuation factor */
  float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
  tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

  math::Vector<3> pidAttenuationPerAxis;
  pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
  pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
  pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

  return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
DolphinAttitudeControl::control_attitude_rates(float dt) {
  /* reset integral if disarmed */
  if (!_armed.armed || !_vehicle_status.is_rotary_wing) {
    _rates_int.zero();
  }

  /* get transformation matrix from sensor/board to body frame */
  get_rot_matrix((enum Rotation) _params.board_rotation, &_board_rotation);

  /* fine tune the rotation */
  math::Matrix<3, 3> board_rotation_offset;
  board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _params.board_offset[0],
                                   M_DEG_TO_RAD_F * _params.board_offset[1],
                                   M_DEG_TO_RAD_F * _params.board_offset[2]);
  _board_rotation = board_rotation_offset * _board_rotation;

  // get the raw gyro data and correct for thermal errors
  math::Vector<3> rates;

  if (_selected_gyro == 0) {
    rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
    rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
    rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

  } else if (_selected_gyro == 1) {
    rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
    rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
    rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

  } else if (_selected_gyro == 2) {
    rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
    rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
    rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

  } else {
    rates(0) = _sensor_gyro.x;
    rates(1) = _sensor_gyro.y;
    rates(2) = _sensor_gyro.z;
  }

  // rotate corrected measurements from sensor to body frame
  rates = _board_rotation * rates;

  // correct for in-run bias errors
  rates(0) -= _ctrl_state.roll_rate_bias;
  rates(1) -= _ctrl_state.pitch_rate_bias;
  rates(2) -= _ctrl_state.yaw_rate_bias;

  math::Vector<3> rates_p_scaled = _params.rate_p.emult(pid_attenuations(_params.tpa_breakpoint_p, _params.tpa_rate_p));
  //math::Vector<3> rates_i_scaled = _params.rate_i.emult(pid_attenuations(_params.tpa_breakpoint_i, _params.tpa_rate_i));
  math::Vector<3> rates_d_scaled = _params.rate_d.emult(pid_attenuations(_params.tpa_breakpoint_d, _params.tpa_rate_d));

  /* angular rates error */
  math::Vector<3> rates_err = _rates_sp - rates;

  _att_control = rates_p_scaled.emult(rates_err) +
                 _rates_int +
                 rates_d_scaled.emult(_rates_prev - rates) / dt +
                 _params.rate_ff.emult(_rates_sp);

  _rates_sp_prev = _rates_sp;
  _rates_prev = rates;


  /* update integral only if motors are providing enough thrust to be effective */
  if (_thrust_sp > MIN_TAKEOFF_THRUST) {
    for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
      // Check for positive control saturation
      bool positive_saturation =
              ((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
              ((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
              ((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

      // Check for negative control saturation
      bool negative_saturation =
              ((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
              ((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
              ((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

      // prevent further positive control saturation
      if (positive_saturation) {
        rates_err(i) = math::min(rates_err(i), 0.0f);

      }

      // prevent further negative control saturation
      if (negative_saturation) {
        rates_err(i) = math::max(rates_err(i), 0.0f);

      }

      // Perform the integration using a first order method and do not propaate the result if out of range or invalid
      float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

      if (PX4_ISFINITE(rate_i) && rate_i > -_params.rate_int_lim(i) && rate_i < _params.rate_int_lim(i)) {
        _rates_int(i) = rate_i;

      }
    }
  }

  /* explicitly limit the integrator state */
  for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
    _rates_int(i) = math::constrain(_rates_int(i), -_params.rate_int_lim(i), _params.rate_int_lim(i));

  }
}

/*
 * Mixer
 * Input: '_att_control' vector, mixing table
 * Output: '_mixed_att_control' vector
 */
bool DolphinAttitudeControl::mix_control_output(math::Vector<3> &att_control, float thrust_sp,
                                                math::Vector<4> &mixed_att_control) {

  /***
   * Mixing strategy: adopted from mixer_multirotor.cpp
   * 1) Mix pitch, yaw and thrust without roll. And calculate max and min outputs
   * 2) Shift all outputs to minimize out of range violations, min or max. If not enough room to shift all outputs, then scale back pitch/yaw.
   * 3) If there is violation on both upper and lower end then shift such that the violation is equal on both sides.
   * 4) Mix in roll and see if it leads to limit violation, scale back output to allow for roll.
   * 5) Scale all output to range [-1, 1]
   * */

  float roll = math::constrain(att_control(0), -1.0f, 1.0f);
  float pitch = math::constrain(att_control(1), -1.0f, 1.0f);
  float yaw = math::constrain(att_control(2), -1.0f, 1.0f);
  float thrust = math::constrain(thrust_sp, -1.0f, 1.0f);
  float min_out = 1.0f;
  float max_out = -1.0f;

  /*** TODO: Understand how to translate the code to scale to forward and reverse motion.
   * I think the attitude is independent and will scale just fine with motor reverses, well assuming bidirectional
   * equality in thrust power per rotor. But here in att control I should just receive thrust command as a range from
   * -1 to 1 TODO: scale the thrust input early on to match -1 to 1 for bidirectional thrust.
   * */

  // thrust boost parameters AA: This is to limit the max increase/decrease of thrust to account for saturation
  float thrust_increase_factor = 1.5f;
  float thrust_decrease_factor = 0.75f;

  float outputs[4];

  /*** perform initial mix pass yielding unbounded outputs, ignore roll
  */
  for (unsigned i = 0; i < _rotor_count; i++) {
    float out = pitch * _rotors[i].pitch_scale +
                yaw * _rotors[i].yaw_scale +
                thrust * _rotors[i].thrust_scale;

    out *= _rotors[i].out_scale;

    /* calculate min and max output values AA: is in lowest and highest motors outputs */
    if (out < min_out) {
      min_out = out;
    }

    if (out > max_out) {
      max_out = out;
    }
    outputs[i] = out;
  }

  float boost = 0.0f;              // value added to demanded thrust (can also be negative)
  float pitch_yaw_scale = 1.0f;    // scale for demanded pitch and yaw
  float low_bound = -1.0f;
  float upp_bound = 1.0f;


  /*** Now we check if the outputs violate the bounds */
  // TODO review the math
  /* If things are fine - for completeness sake */
  if (max_out < upp_bound && min_out > low_bound) {
    // Keep calm and move on
  }
    // If min is out of bound
  else if (min_out < low_bound && max_out < upp_bound) {

    // In this case we need to increase thrust to bring motors within bound.
    float max_thrust_diff = (float) fabs(thrust * thrust_increase_factor - thrust);

    // if amount out of bound is less than gap between max and upper bound - shift up to make min = lower_bound
    if (-(min_out - low_bound) <= (upp_bound - max_out)) {

      if (max_thrust_diff >= -(min_out - low_bound)) {
        boost = -(min_out - low_bound);
      } else {

        boost = max_thrust_diff;
        pitch_yaw_scale = ((thrust + boost + 1) / (thrust - min_out));
      }
    }
      // shift max increase possible and scale back pitch_yaw
    else {
      boost = math::constrain(-(min_out - low_bound) - (1.0f - max_out) / 2.0f, 0.0f, max_thrust_diff); //TODO: FIX
      PX4_INFO("Shift back pitch_yaw Boost Value Positive: %f", (double) boost);
      pitch_yaw_scale = ((thrust + boost + 1) / (thrust - min_out));
    }

  }
    // if max is out of bound
  else if (max_out > upp_bound && min_out > low_bound) {

    float max_thrust_diff = (float) fabs(thrust - thrust_decrease_factor * thrust);

    // if amount out of bound is less than gap between min and lower bound - shift down to make max = upper_bound
    if ((max_out - upp_bound) <= -(low_bound - min_out)) {

      if (max_thrust_diff >= (max_out - upp_bound)) {
        boost = -(max_out - upp_bound);
      } else {
        boost = -max_thrust_diff;
        pitch_yaw_scale = (1 - (thrust + boost)) / (max_out - thrust);
      }
    }
      // shift max decrease possible and scale back pitch_yaw
    else {
      boost = math::constrain(-(max_out - 1.0f - min_out) / 2.0f, (float) -max_thrust_diff, 0.0f);
      pitch_yaw_scale = (1 - (thrust + boost)) / (max_out - thrust);
    }
  }
    // if both are out of bound
  else if (max_out > upp_bound && min_out < low_bound) {
    // Scale back so that both violations are equal
    boost = math::constrain(-(max_out - 1.0f + min_out) / 2.0f, (float) -fabs(thrust_decrease_factor * thrust - thrust),
                            (float) fabs(thrust_increase_factor * thrust - thrust));
    pitch_yaw_scale = (thrust + boost) / (thrust - (min_out - low_bound));
  } else {
    // I should never get here!
  }

//  PX4_INFO("New Roll Value %f", (double) roll);
  float thrust_reduction = 0.0f;
  float thrust_increase = 0.0f;
  float roll_scale_2 = 1.0f;
  float pitch_yaw_mix[_rotor_count] = {0.0f, 0.0f, 0.0f, 0.0f};
  /*** Mix now with boost, pitch_yaw scale and roll */
  for (unsigned i = 0; i < _rotor_count; i++) {
    pitch_yaw_mix[i] = (pitch * _rotors[i].pitch_scale + yaw * _rotors[i].yaw_scale) * pitch_yaw_scale;
    float out = pitch_yaw_mix[i] +
                roll * _rotors[i].roll_scale +
                (thrust + thrust_increase - thrust_reduction) + boost;
    out *= _rotors[i].out_scale;

    if (thrust >= 0.0f) {
      if (out > 1.0f) {
        // Thrust Positive and Output with roll exceeds upper bound: reduce thrust and scale back roll
        // Max prop reduction
        float prop_reduction = fminf(0.15f, out - 1.0f);
        thrust_reduction = fmaxf(thrust_reduction, prop_reduction);
        // roll scaled back s.t out = 1.0f TODO: Change this, I need a function to scale back roll, not to recalculate it.
        roll_scale_2 =
                (1.0f - (pitch_yaw_mix[i] + (thrust - thrust_reduction) + boost)) / (roll * _rotors[i].roll_scale);
//        PX4_INFO("M%i +Thrust +Out Roll Scale 2: %f", i, (double) roll_scale_2);
      } else if (out < -1.0f) {
        // Roll scaled back s.t. out = -1.0f
        roll_scale_2 =
                (-1.0f - (pitch_yaw_mix[i] + (thrust - thrust_reduction) + boost)) / (roll * _rotors[i].roll_scale);
//        PX4_INFO("+Thrust -Out Roll Scale 2: %f", (double) roll_scale_2);
      }
    } else if (thrust < 0.0f) {
      if (out > 1.0f) {
        // Scale back roll
        roll_scale_2 =
                (1.0f - (pitch_yaw_mix[i] + (thrust + thrust_increase) + boost)) / (roll * _rotors[i].roll_scale);
//        PX4_INFO("-Thrust +Out Roll Scale 2: %f", (double) roll_scale_2);

      } else if (out < -1.0f) {
        // Thrust negative and output with roll violates lower bound: increase thrust and scale back roll 50/50
        float prop_increase = fminf(0.15f, -(out + 1.0f));
        thrust_increase = fmaxf(thrust_increase, prop_increase);
        // roll scaled back s.t out = 1.0f
        roll_scale_2 =
                (-1.0f - (pitch_yaw_mix[i] + (thrust + thrust_increase) + boost)) / (roll * _rotors[i].roll_scale);
//        PX4_INFO("-Thrust -Out Roll Scale 2: %f", (double) roll_scale_2);
      }
    }

    roll = roll * roll_scale_2;
  }

  // Apply collective thrust reduction/increase (one shall be zero), the maximum for one prop
  thrust = thrust - thrust_reduction + thrust_increase;

  // add roll and scale outputs to range idle_speed...1
  for (unsigned i = 0; i < _rotor_count; i++) {
    outputs[i] = pitch_yaw_mix[i] +
                 roll * _rotors[i].roll_scale * roll_scale_2 +
                 thrust + boost;

    /*
     * TODO: After evaluating my own thrusters, change this to suit model better. for now keep as is.
     * TODO: Confirm where thrust_factor is set, if it doesn't cascade to here, then just set it here or make a
     * module param.
     *
      implement simple model for static relationship between applied motor pwm and motor thrust
      model: thrust = (1 - _thrust_factor) * PWM + _thrust_factor * PWM^2
      this model assumes normalized input / output in the range [0,1] so this is the right place
      to do it as at this stage the outputs are in that range.
      // TODO: This model needs to change to reflect the face that I'm using a bidrectional thrust. A reverse scaler must be used that is different than the forward one, since non-symmetrical thrust
      // TODO: I need to split this, if an output (+) use one equation, if (-) use equation for reverse thrust and
      add sign.
     */
    auto _thrust_factor = _params.thrust_factor;
    auto _idle_speed = _params.idle_speed;

    if (_thrust_factor > 0.0f) {
      float _output = outputs[i];
      if(_output > 0.0f){
        _output = -(1.0f - _thrust_factor) /
                  (2.0f * _thrust_factor) + sqrtf((1.0f - _thrust_factor) *
                                                  (1.0f - _thrust_factor) /
                                                  (4.0f * _thrust_factor * _thrust_factor) + (_output / _thrust_factor));
        _output = math::constrain(_idle_speed + (_output * (1.0f - _idle_speed)), 0.0f, 1.0f);
        outputs[i] = _output;
      }
      else if (_output < 0.0f){
        _output = - _output; // Work with positive
        _output = -(1.0f - _thrust_factor) /
                  (2.0f * _thrust_factor) + sqrtf((1.0f - _thrust_factor) *
                                                  (1.0f - _thrust_factor) /
                                                  (4.0f * _thrust_factor * _thrust_factor) + (_output / _thrust_factor));
        _output = math::constrain(_idle_speed + (_output * (1.0f - _idle_speed)), 0.0f, 1.0f);
        outputs[i] = - _output; // Bring back the sign
      }

    }



  }


  /* TODO: Incorporate slew rate limiting
   * slew rate limiting and saturation checking */
//  for (unsigned i = 0; i < _rotor_count; i++) {
//    bool clipping_high = false;
//    bool clipping_low = false;
//
//    // check for saturation against static limits
//    if (outputs[i] > 0.99f) {
//      clipping_high = true;
//
//    } else if (outputs[i] < _idle_speed + 0.01f) {
//      clipping_low = true;
//
//    }
//
//    // check for saturation against slew rate limits
//    if (_delta_out_max > 0.0f) {
//      float delta_out = outputs[i] - _outputs_prev[i];
//
//      if (delta_out > _delta_out_max) {
//        outputs[i] = _outputs_prev[i] + _delta_out_max;
//        clipping_high = true;
//
//      } else if (delta_out < -_delta_out_max) {
//        outputs[i] = _outputs_prev[i] - _delta_out_max;
//        clipping_low = true;
//
//      }
//    }
//
//    _outputs_prev[i] = outputs[i];


  /* Copy outputs to premixed att control output */
  mixed_att_control(0) = (outputs[0] + 1.0f) / 2.0f;
  mixed_att_control(1) = (outputs[1] + 1.0f) / 2.0f;
  mixed_att_control(2) = (outputs[2] + 1.0f) / 2.0f;
  mixed_att_control(3) = (outputs[3] + 1.0f) / 2.0f;
//  PX4_INFO("Mixed Output: %f, %f, %f, %f", (double) mixed_att_control(0),
//           (double) mixed_att_control(1),
//           (double) mixed_att_control(2),
//           (double) mixed_att_control(3));
  return true;
}

void
DolphinAttitudeControl::task_main_trampoline(int argc, char *argv[]) {
  dp_att_control::g_control->task_main();
}

void
DolphinAttitudeControl::task_main() {

  /*
   * do subscriptions
   */
  _v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
  _v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
  _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
  _v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
  _params_sub = orb_subscribe(ORB_ID(parameter_update));
  _manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
  _armed_sub = orb_subscribe(ORB_ID(actuator_armed));
  _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
  _motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
  _battery_status_sub = orb_subscribe(ORB_ID(battery_status));

  _gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);

  if (_gyro_count == 0) {
    _gyro_count = 1;
  }

  for (unsigned s = 0; s < _gyro_count; s++) {
    _sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
  }

  _sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));

  /* initialize parameters cache */
  parameters_update();

  /* wakeup source: gyro data from sensor selected by the sensor app */
  px4_pollfd_struct_t poll_fds = {};
  poll_fds.events = POLLIN;

  while (!_task_should_exit) {

    poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

    /* wait for up to 100ms for data */
    int pret = px4_poll(&poll_fds, 1, 100);

    /* timed out - periodic check for _task_should_exit */
    if (pret == 0) {
      continue;
    }

    /* this is undesirable but not much we can do - might want to flag unhappy status */
    if (pret < 0) {
      warn("mc att ctrl: poll error %d, %d", pret, errno);
      /* sleep a bit before next try */
      usleep(100000);
      continue;
    }

    perf_begin(_loop_perf);

    /* run controller on gyro changes */
    if (poll_fds.revents & POLLIN) {
      static uint64_t last_run = 0;
      float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
      last_run = hrt_absolute_time();

      /* guard against too small (< 2ms) and too large (> 20ms) dt's */
      if (dt < 0.002f) {
        dt = 0.002f;

      } else if (dt > 0.02f) {
        dt = 0.02f;
      }

      /* copy gyro data */
      orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);

      /* check for updates in other topics */
      parameter_update_poll();
      vehicle_control_mode_poll();
      arming_status_poll();
      vehicle_manual_poll();
      vehicle_status_poll();
      vehicle_motor_limits_poll();
      battery_status_poll();
      control_state_poll();
      sensor_correction_poll();


      if (_v_control_mode.flag_control_attitude_enabled) {

        //normal attitude control
        control_attitude(dt);
        //control_attitude(

        /* publish attitude rates setpoint */
        _v_rates_sp.roll = _rates_sp(0);
        _v_rates_sp.pitch = _rates_sp(1);
        _v_rates_sp.yaw = _rates_sp(2);
        _v_rates_sp.thrust = _thrust_sp;
        _v_rates_sp.timestamp = hrt_absolute_time();

        if (_v_rates_sp_pub != nullptr) {
          orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

        } else if (_rates_sp_id) {
          _v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
        }

      } else {
        /* attitude controller disabled, poll rates setpoint topic */
        if (_v_control_mode.flag_control_manual_enabled) {
          /* manual rates control - ACRO mode */
          _rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x,
                                      _manual_control_sp.r).emult(_params.acro_rate_max);
          _thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_DOLPHIN);
          PX4_INFO("Manual Control Thrust %f, Rates: %f, %f, %f", (double)_thrust_sp*100, (double)_rates_sp(0)*100,
                   (double)_rates_sp(1) * 100,
                   (double)_rates_sp(2) * 100);

          /* publish attitude rates setpoint */
          _v_rates_sp.roll = _rates_sp(0);
          _v_rates_sp.pitch = _rates_sp(1);
          _v_rates_sp.yaw = _rates_sp(2);
          _v_rates_sp.thrust = _thrust_sp;
          _v_rates_sp.timestamp = hrt_absolute_time();

          if (_v_rates_sp_pub != nullptr) {
            orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

          } else if (_rates_sp_id) {
            _v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
          }

        } else {
          /* attitude controller disabled, poll rates setpoint topic TODO: fix comment, doesn't make sense */
          vehicle_rates_setpoint_poll();
          _rates_sp(0) = _v_rates_sp.roll;
          _rates_sp(1) = _v_rates_sp.pitch;
          _rates_sp(2) = _v_rates_sp.yaw;
          _thrust_sp = _v_rates_sp.thrust;
        }
      }

      /* should be enabled by default if controlling vehicle*/
      if (_v_control_mode.flag_control_rates_enabled) {

        control_attitude_rates(dt); // TODO: Fix such that the function explicitly takes input gives output.

        /* Now we mix */

        mix_control_output(_att_control, _thrust_sp, _mixed_att_control);

        /* publish actuator controls TODO: Mix first then publish*/
        _actuators.control[0] = (PX4_ISFINITE(_mixed_att_control(0))) ? _mixed_att_control(0) : 0.0f;
        _actuators.control[1] = (PX4_ISFINITE(_mixed_att_control(1))) ? _mixed_att_control(1) : 0.0f;
        _actuators.control[2] = (PX4_ISFINITE(_mixed_att_control(2))) ? _mixed_att_control(2) : 0.0f;
        _actuators.control[3] = (PX4_ISFINITE(_mixed_att_control(3))) ? _mixed_att_control(3) : 0.0f;
        _actuators.timestamp = hrt_absolute_time();
        _actuators.timestamp_sample = _ctrl_state.timestamp;

        /* scale effort by battery status TODO: Verify this works with my setup*/
        if (_params.bat_scale_en && _battery_status.scale > 0.0f) {
          for (int i = 0; i < 4; i++) {
            _actuators.control[i] *= _battery_status.scale;
          }
        }

        _controller_status.roll_rate_integ = _rates_int(0);
        _controller_status.pitch_rate_integ = _rates_int(1);
        _controller_status.yaw_rate_integ = _rates_int(2);
        _controller_status.timestamp = hrt_absolute_time();

        if (!_actuators_0_circuit_breaker_enabled) {
          if (_actuators_0_pub != nullptr) {

            orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
            perf_end(_controller_latency_perf);

          } else if (_actuators_id) {
            _actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
          }

        }

        /* publish controller status */
        if (_controller_status_pub != nullptr) {
          orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

        } else {
          _controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
        }
      }
    }

    // TODO: review, is this necessary? This is a termination routine, I need to modify it for my use,

    perf_end(_loop_perf);
  }

  _control_task = -1;

}


int
DolphinAttitudeControl::start() {
  ASSERT(_control_task == -1);

  /* start the task */
  _control_task = px4_task_spawn_cmd("dp_att_control",
                                     SCHED_DEFAULT,
                                     SCHED_PRIORITY_ATTITUDE_CONTROL,
                                     1700,
                                     (px4_main_t) &DolphinAttitudeControl::task_main_trampoline,
                                     nullptr);

  if (_control_task < 0) {
    warn("task start failed");
    return -errno;
  }

  return OK;
}


int dp_att_control_main(int argc, char *argv[]) {
  if (argc < 2) {
    warnx("usage: dp_att_control {start|stop|status}");
    return 1;
  }

  if (!strcmp(argv[1], "start")) {

    if (dp_att_control::g_control != nullptr) {
      warnx("already running");
      return 1;
    }

    dp_att_control::g_control = new DolphinAttitudeControl;

    if (dp_att_control::g_control == nullptr) {
      warnx("alloc failed");
      return 1;
    }

    if (OK != dp_att_control::g_control->start()) {
      delete dp_att_control::g_control;
      dp_att_control::g_control = nullptr;
      warnx("start failed");
      return 1;
    }

    return 0;
  }

  if (!strcmp(argv[1], "stop")) {
    if (dp_att_control::g_control == nullptr) {
      warnx("not running");
      return 1;
    }

    delete dp_att_control::g_control;
    dp_att_control::g_control = nullptr;
    return 0;
  }

  if (!strcmp(argv[1], "status")) {
    if (dp_att_control::g_control) {
      warnx("running");
      return 0;

    } else {
      warnx("not running");
      return 1;
    }
  }

  warnx("unrecognized command");
  return 1;
}
