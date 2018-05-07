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
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 *
 * @modified by Ali AlSaibie <ali@gatech.edu>
 */



#include "dp_pos_control.hpp"

static int _control_task = -1;			/**< task handle for sensor task */

using matrix::Eulerf;
using matrix::Quatf;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int dp_pos_control_main(int argc, char *argv[]);


namespace dp_control
{
    DolphinPositionControl	*dp_control = nullptr;
}

DolphinPositionControl::DolphinPositionControl() :
        SuperBlock(nullptr, "DPC"),
/* performance counters */
        _loop_perf(perf_alloc(PC_ELAPSED, "dp pos control")),
        _reset_pos_sp(true),
        _reset_alt_sp(true),
        _do_reset_alt_pos_flag(true),
        _mode_auto(false),
        _pos_hold_engaged(false),
        _alt_hold_engaged(false),
        _run_pos_control(true),
        _run_alt_control(true),
        _reset_yaw_sp(true)
{
  _parameter_handles.throttle_idle      = param_find("DPC_THR_IDLE");
  _parameter_handles.speed_control_mode = param_find("DPC_SP_CTRL_MODE");
  _parameter_handles.throttle_min       = param_find("DPC_THR_MIN");
  _parameter_handles.throttle_max       = param_find("DPC_THR_MAX");

  _parameter_handles.throttle_cruise    = param_find("DPC_THR_CRUISE");

  /* fetch initial parameter values */
  parameters_update();
}

DolphinPositionControl::~DolphinPositionControl()
{
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

  dp_control::dp_control = nullptr;
}

int
DolphinPositionControl::parameters_update()
{

  param_get(_parameter_handles.speed_control_mode, &(_parameters.speed_control_mode));


  param_get(_parameter_handles.throttle_idle, &(_parameters.throttle_idle));
  param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
  param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
  param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));

  /* Update and publish the navigation capabilities */
//  _dp_pos_ctrl_status.landing_slope_angle_rad = 0;
//  _dp_pos_ctrl_status.landing_horizontal_slope_displacement = 0;
//  _dp_pos_ctrl_status.landing_flare_length = 0;
  dp_pos_ctrl_status_publish();

  return OK;
}

void
DolphinPositionControl::vehicle_control_mode_poll()
{
  bool updated;
  orb_check(_control_mode_sub, &updated);

  if (updated) {
    orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
  }
}

void
DolphinPositionControl::manual_control_setpoint_poll()
{
  bool manual_updated;
  orb_check(_manual_control_sub, &manual_updated);

  if (manual_updated) {
    orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
  }
}

void
DolphinPositionControl::control_state_poll()
{
  bool ctrl_state_updated;
  orb_check(_ctrl_state_sub, &ctrl_state_updated);

  if (ctrl_state_updated) {
    orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
  }
}

void
DolphinPositionControl::position_setpoint_triplet_poll()
{
  bool pos_sp_triplet_updated;
  orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

  if (pos_sp_triplet_updated) {
    orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
  }
}

void DolphinPositionControl::dp_pos_ctrl_status_publish()
{
  _dp_pos_ctrl_status.timestamp = hrt_absolute_time();

  if (_dp_pos_ctrl_status_pub != nullptr) {
    orb_publish(ORB_ID(fw_pos_ctrl_status), _dp_pos_ctrl_status_pub, &_dp_pos_ctrl_status);

  } else {
    _dp_pos_ctrl_status_pub = orb_advertise(ORB_ID(fw_pos_ctrl_status), &_dp_pos_ctrl_status);
  }
}

bool
DolphinPositionControl::control_position(const math::Vector<2> &current_position,
                                             const math::Vector<3> &ground_speed, const position_setpoint_triplet_s &pos_sp_triplet)
{
  // TODO: Take care of this later.
//  float dt = 0.01; // Using non zero value to a avoid division by zero
//
//  if (_control_position_last_called > 0) {
//    dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
//  }
//
//  _control_position_last_called = hrt_absolute_time();
//
  bool setpoint = true;
//
//  if (_control_mode.flag_control_auto_enabled && pos_sp_triplet.current.valid) {
//    /* AUTONOMOUS FLIGHT */
//
//    _control_mode_current = AUV_POSCTRL_MODE_AUTO;
//
//    /* get circle mode */
//    bool was_circle_mode = _dp_control.circle_mode();
//
//    /* current waypoint (the one currently heading for) */
//    math::Vector<2> curr_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);
//
//    /* previous waypoint */
//    math::Vector<2> prev_wp = curr_wp;
//
//    if (pos_sp_triplet.previous.valid) {
//      prev_wp(0) = (float)pos_sp_triplet.previous.lat;
//      prev_wp(1) = (float)pos_sp_triplet.previous.lon;
//    }
//
//    math::Vector<2> ground_speed_2d = {ground_speed(0), ground_speed(1)};
//
//    float mission_throttle = _parameters.throttle_cruise;
//
//    /* Just control the throttle */
//    if (_parameters.speed_control_mode == 1) {
//      /* control the speed in closed loop */
//
//      float mission_target_speed = _parameters.dpspeed_trim;
//
//      if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
//          _pos_sp_triplet.current.cruising_speed > 0.1f) {
//        mission_target_speed = _pos_sp_triplet.current.cruising_speed;
//      }
//
//      //Compute airspeed control out and just scale it as a constant
//      mission_throttle = _parameters.throttle_speed_scaler
//                         * pid_calculate(&_speed_ctrl, mission_target_speed, _ctrl_state.x_vel, _ctrl_state.x_acc, dt);
//
//      // Constrain throttle between min and max
//      mission_throttle = math::constrain(mission_throttle, _parameters.throttle_min, _parameters.throttle_max);
//
//    } else {
//      /* Just control throttle in open loop */
//      if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle) &&
//          _pos_sp_triplet.current.cruising_throttle > 0.01f) {
//
//        mission_throttle = _pos_sp_triplet.current.cruising_throttle;
//      }
//    }
//
//    if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
//      _att_sp.roll_body = 0.0f;
//      _att_sp.pitch_body = 0.0f;
//      _att_sp.yaw_body = 0.0f;
//      _att_sp.thrust = 0.0f;
//
//    } else if ((pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION)
//               || (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF)) {
//
//      /* waypoint is a plain navigation waypoint or the takeoff waypoint, does not matter */
//      _dp_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);
//      _att_sp.roll_body = _dp_control.nav_roll();
//      _att_sp.pitch_body = 0.0f;
//      _att_sp.yaw_body = _dp_control.nav_bearing();
//      _att_sp.fw_control_yaw = true;
//      _att_sp.thrust = mission_throttle;
//
//    } else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
//
//      /* waypoint is a loiter waypoint so we want to stop*/
//      _dp_control.navigate_loiter(curr_wp, current_position, pos_sp_triplet.current.loiter_radius,
//                                   pos_sp_triplet.current.loiter_direction, ground_speed_2d);
//
//      _att_sp.roll_body = _dp_control.nav_roll();
//      _att_sp.pitch_body = 0.0f;
//      _att_sp.yaw_body = _dp_control.nav_bearing();
//      _att_sp.fw_control_yaw = true;
//      _att_sp.thrust = 0.0f;
//    }
//
//    if (was_circle_mode && !_dp_control.circle_mode()) {
//      /* just kicked out of loiter, reset integrals */
//      _att_sp.yaw_reset_integral = true;
//    }
//
//  } else {
//    _control_mode_current = AUV_POSCTRL_MODE_OTHER;
//
//    _att_sp.roll_body = 0.0f;
//    _att_sp.pitch_body = 0.0f;
//    _att_sp.yaw_body = 0.0f;
//    _att_sp.fw_control_yaw = true;
//    _att_sp.thrust = 0.0f;
//
//    /* do not publish the setpoint */
//    setpoint = false;
//  }

  return setpoint;
}

void
DolphinPositionControl::task_main()
{
  _control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
  _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
  _global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
  _manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
  _params_sub = orb_subscribe(ORB_ID(parameter_update));
  _pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

  /* rate limit control mode updates to 5Hz */
  orb_set_interval(_control_mode_sub, 200);

  /* rate limit position updates to 50 Hz */
  orb_set_interval(_global_pos_sub, 20);

  /* abort on a nonzero return value from the parameter init */
  if (parameters_update()) {
    /* parameter setup went wrong, abort */
    warnx("aborting startup due to errors.");
    _task_should_exit = true;
  }

  /* wakeup source(s) */
  px4_pollfd_struct_t fds[2];

  /* Setup of loop */
  fds[0].fd = _params_sub;
  fds[0].events = POLLIN;
  fds[1].fd = _global_pos_sub;
  fds[1].events = POLLIN;

  _task_running = true;

  hrt_abstime t_prev = 0;

  while (!_task_should_exit) {

    /* wait for up to 500ms for data */
    int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

    /* timed out - periodic check for _task_should_exit, etc. */
    if (pret == 0) {
      continue;
    }

    /* this is undesirable but not much we can do - might want to flag unhappy status */
    if (pret < 0) {
      warn("poll error %d, %d", pret, errno);
      continue;
    }

    /* check vehicle control mode for changes to publication state */
    vehicle_control_mode_poll();

    /* only update parameters if they changed */
    if (fds[0].revents & POLLIN) {
      /* read from param to clear updated flag */
      struct parameter_update_s update;
      orb_copy(ORB_ID(parameter_update), _params_sub, &update);

      /* update parameters from storage */
      parameters_update();
    }
    hrt_abstime t = hrt_absolute_time();
    float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
    t_prev = t;

    /* set dt for control blocks */
    setDt(dt);

    /* only run controller if position changed */
    if (fds[1].revents & POLLIN) {
      perf_begin(_loop_perf);

      /* load local copies */
      orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

      if (!_control_mode.flag_control_position_enabled || !_control_mode.flag_control_manual_enabled) {
        _pos_hold_engaged = false;
      }

      if (!_control_mode.flag_control_altitude_enabled || !_control_mode.flag_control_manual_enabled) {
        _alt_hold_engaged = false;
      }

      /* generate attitude setpoint from manual controls */
      if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {

        generate_attitude_setpoint(dt);

      } else {
        _reset_yaw_sp = true;
        _att_sp.yaw_sp_move_rate = 0.0f;
      }

//      // handle estimator reset events. we only adjust setpoins for manual modes
//      if (_control_mode.flag_control_manual_enabled) {
//
//        // adjust navigation waypoints in position control mode
//        if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
//            && _global_pos.lat_lon_reset_counter != _pos_reset_counter) {
//        }
//      }
//
//      // update the reset counters in any case
//      _pos_reset_counter = _global_pos.lat_lon_reset_counter;
//
//      control_state_poll();
//      manual_control_setpoint_poll();
//      position_setpoint_triplet_poll();
//
//      math::Vector<3> ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
//      math::Vector<2> current_position((float)_global_pos.lat, (float)_global_pos.lon);
//
//      /*
//       * Attempt to control position, on success (= sensors present and not in manual mode),
//       * publish setpoint.
//       */
//      if (control_position(current_position, ground_speed, _pos_sp_triplet)) {
//        _att_sp.timestamp = hrt_absolute_time();
//
//        Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
//        q.copyTo(_att_sp.q_d);
//        _att_sp.q_d_valid = true;
//
//        if (!_control_mode.flag_control_offboard_enabled ||
//            _control_mode.flag_control_position_enabled ||
//            _control_mode.flag_control_velocity_enabled ||
//            _control_mode.flag_control_acceleration_enabled) {
//
//          /* lazily publish the setpoint only once available */
//          if (_attitude_sp_pub != nullptr) {
//            /* publish the attitude setpoint */
//            orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &_att_sp);
//
//          } else {
//            /* advertise and publish */
//            _attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
//          }
//        }
//
//        /* XXX check if radius makes sense here */
//        float turn_distance = _parameters.l1_distance; //_dp_control.switch_distance(100.0f);
//
//        /* lazily publish navigation capabilities */
//        if ((hrt_elapsed_time(&_dp_pos_ctrl_status.timestamp) > 1000000)
//            || (fabsf(turn_distance - _dp_pos_ctrl_status.turn_distance) > FLT_EPSILON
//                && turn_distance > 0)) {
//
//          /* set new turn distance */
//          _dp_pos_ctrl_status.turn_distance = turn_distance;
//
//          _dp_pos_ctrl_status.nav_roll = _dp_control.nav_roll();
//          _dp_pos_ctrl_status.nav_pitch = 0.0f;
//          _dp_pos_ctrl_status.nav_bearing = _dp_control.nav_bearing();
//
//          _dp_pos_ctrl_status.target_bearing = _dp_control.target_bearing();
//          _dp_pos_ctrl_status.xtrack_error = _dp_control.crosstrack_error();
//
//          math::Vector<2> curr_wp((float)_pos_sp_triplet.current.lat, (float)_pos_sp_triplet.current.lon);
//          _dp_pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(current_position(0), current_position(1), curr_wp(0),
//                                                                       curr_wp(1));
//
//          dp_pos_ctrl_status_publish();
//        }
//      }

      perf_end(_loop_perf);
    }
  }

  _task_running = false;

  warnx("exiting.\n");

  _control_task = -1;
}

void
DolphinPositionControl::task_main_trampoline(int argc, char *argv[])
{
  dp_control::dp_control = new DolphinPositionControl();

  if (dp_control::dp_control == nullptr) {
    warnx("OUT OF MEM");
    return;
  }

  /* only returns on exit */
  dp_control::dp_control->task_main();
  delete dp_control::dp_control;
  dp_control::dp_control = nullptr;
}

int
DolphinPositionControl::start()
{
  ASSERT(_control_task == -1);
  warn("Starting by marco");

  /* start the task */
  _control_task = px4_task_spawn_cmd("dp_pos_ctrl",
                                     SCHED_DEFAULT,
                                     SCHED_PRIORITY_POSITION_CONTROL,
                                     1700,
                                     (px4_main_t)&DolphinPositionControl::task_main_trampoline,
                                     nullptr);
  warn("done");

  if (_control_task < 0) {
    warn("task start failed");
    return -errno;
  }

  return OK;
}

void DolphinPositionControl::generate_attitude_setpoint(float dt) {

  //TODO: Change orientation. Roll replaces yaw.
  if (_reset_yaw_sp) {
    _reset_yaw_sp = false;
    _att_sp.yaw_body = _yaw;
  }

  /* control throttle directly if no climb rate controller is active */
//  if (!_control_mode.flag_control_climb_rate_enabled) {
//    float thr_val = throttle_curve(_manual.z, _params.thr_hover);
//    _att_sp.thrust = math::min(thr_val, _manual_thr_max.get());
//
//  }
//
//  _att_sp.yaw_body = _manual.y * _params.man_tilt_max;
//  _att_sp.pitch_body = -_manual.x * _params.man_tilt_max;

  // construct attitude setpoint rotation matrix. modify the setpoints for yaw
  // and pitch such that they reflect the user's intention even if a roll error
  // (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
  // from the pure euler angle setpoints will lead to unexpected attitude behaviour from
  // the user's view as the euler angle sequence uses the  yaw setpoint and not the current
  // heading of the vehicle.

  // calculate our current yaw error
  float yaw_error = _wrap_pi(_att_sp.yaw_body - _yaw);

  // compute the vector obtained by rotating a z unit vector by the rotation
  // given by the roll and pitch commands of the user
  math::Vector<3> zB = {0, 0, 1};
  math::Matrix<3, 3> R_sp_yaw_pitch;
  R_sp_yaw_pitch.from_euler(0, _att_sp.pitch_body, yaw_error);
  math::Vector<3> z_yaw_pitch_sp = R_sp_yaw_pitch * zB;


  // transform the vector into a new frame which is rotated around the z axis
  // by the current yaw error. this vector defines the desired tilt when we look
  // into the direction of the desired heading
  math::Matrix<3, 3> R_roll_correction;
  R_roll_correction.from_euler(0.0f, 0.0f, -_att_sp.roll_body);
  z_yaw_pitch_sp = R_roll_correction * z_yaw_pitch_sp;

  // use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
  // R_tilt is computed from_euler; only true if cos(roll) not equal zero
  // -> valid if roll is not +-pi/2;

  //TODO: REVIEW AND CORRECT
  _att_sp.roll_body = -asinf(z_yaw_pitch_sp(1));
  _att_sp.pitch_body = atan2f(z_yaw_pitch_sp(0), z_yaw_pitch_sp(2));

  /* copy quaternion setpoint to attitude setpoint topic */
  matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
  q_sp.copyTo(_att_sp.q_d);
  _att_sp.q_d_valid = true;

}

int dp_pos_control_main(int argc, char *argv[])
{
  if (argc < 2) {
    warnx("usage: dp_pos_control {start|stop|status}");
    return 1;
  }

  if (!strcmp(argv[1], "start")) {

    if (dp_control::dp_control != nullptr) {
      warnx("already running");
      return 1;
    }

    if (OK != DolphinPositionControl::start()) {
      warn("start failed");
      return 1;
    }

    /* avoid memory fragmentation by not exiting start handler until the task has fully started */
    while (dp_control::dp_control == nullptr || !dp_control::dp_control->task_running()) {
      usleep(50000);
      printf(".");
      fflush(stdout);
    }

    printf("\n");

    return 0;
  }

  if (!strcmp(argv[1], "stop")) {
    if (dp_control::dp_control == nullptr) {
      warnx("not running");
      return 1;
    }

    delete dp_control::dp_control;
    dp_control::dp_control = nullptr;
    return 0;
  }

  if (!strcmp(argv[1], "status")) {
    if (dp_control::dp_control) {
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
