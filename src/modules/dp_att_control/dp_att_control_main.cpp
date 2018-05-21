/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file dp_att_control_main.cpp
 * Dolphion attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

/**
 *
 * This module is a modification of the mc att module and fw wing app and it is designed for underwater robots.
 *
 * All the acknowledgments and credits for the mc and fw wing app are reported in those files.
 *
 * @author Ali AlSaibie <ali@alsaibie.com>
 *
 */

#include "dp_att_control.hpp"
#include "AttitudeController.hpp"
#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <systemlib/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

using namespace matrix;


int DolphinAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
TODO: FIX
This implements the dolphin attitude and rate controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has two loops: a P loop for angular error and a PID loop for angular rate error.

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dp_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

DolphinAttitudeControl::DolphinAttitudeControl() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "dp_att_control"))
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}

	controller = new AttitudeController();

	update_parameters();
}

void
DolphinAttitudeControl::update_states() {

	/* Attitude */
	Controller::States states;
	generic_poll(ORB_ID(vehicle_attitude), _v_att_sub, _v_att);
	states.att.q = Quatf(_v_att.q);

	/* Rates */
	generic_poll(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], _sensor_gyro);
	generic_poll(ORB_ID(sensor_bias), _sensor_bias_sub, _sensor_bias);
	generic_poll(ORB_ID(sensor_correction),_sensor_correction_sub, _sensor_correction);
	generic_poll(ORB_ID(multirotor_motor_limits), _motor_limits_sub, _motor_limits);

	// get the raw gyro data and correct for thermal errors
	if (_selected_gyro == 0) {
		states.rates.rpy(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
		states.rates.rpy(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
		states.rates.rpy(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		states.rates.rpy(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
		states.rates.rpy(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
		states.rates.rpy(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		states.rates.rpy(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
		states.rates.rpy(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
		states.rates.rpy(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

	} else {
		states.rates.rpy(0) = _sensor_gyro.x;
		states.rates.rpy(1) = _sensor_gyro.y;
		states.rates.rpy(2) = _sensor_gyro.z;
	}

	// rotate corrected measurements from sensor to body frame
	states.rates.rpy = _board_rotation * states.rates.rpy;

	// correct for in-run bias errors
	states.rates.rpy(0) -= _sensor_bias.gyro_x_bias;
	states.rates.rpy(1) -= _sensor_bias.gyro_y_bias;
	states.rates.rpy(2) -= _sensor_bias.gyro_z_bias;

	/* Power */
	generic_poll(ORB_ID(battery_status), _battery_status_sub, _battery_status);
	//TODO: pass correct power parameter to controller

	/* Control Mode */
	generic_poll(ORB_ID(vehicle_control_mode), _v_control_mode_sub, _v_control_mode);
	//TODO: pass correct control mode to controller
	controller->updateStates(states);
}

void
DolphinAttitudeControl::run()
{
	/*
	 * do subscriptions
	 */
	parameter_subscribe_unsubscribe(true);

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	const hrt_abstime task_start = hrt_absolute_time();
	hrt_abstime last_run = task_start;
	float dt_accumulator = 0.f;
	int loop_counter = 0;

	while (!should_exit()) {

		poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for should_exit() */
		if (pret == 0) { continue; }

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on gyro changes */
		if (poll_fds.revents & POLLIN) {
			const hrt_abstime now = hrt_absolute_time();
			float dt = (now - last_run) / 1e6f;
			last_run = now;

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;
			} else if (dt > 0.02f) {
				dt = 0.02f; }

			/* check for updates in other topics */
			generic_poll(ORB_ID(vehicle_status), _vehicle_status_sub, _vehicle_status);
			update_parameters();

            /*
             * Control Step 1: Compute Attitude Rates - Using switch on nav states rather than control modes for explicitness
             * Case 1: Acro mode
             * Case 2: Attitude Control
             * Case 3: Offboard rate_sp
             */
			bool publish_rate_sp = false;

			if(_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {_control_mode.is_armed = true;}
			else {_control_mode.is_armed = false;}

            switch (_vehicle_status.nav_state){
                case vehicle_status_s::NAVIGATION_STATE_ACRO: {
					/* ACRO mode - Rate Setpoint from Manual Input */
					generic_poll(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, _manual_control_sp);

					/* Update Control Mode */
					_control_mode.mode = Controller::CONTROL_MODE::Acro;
					controller->updateControlMode(_control_mode);

					/* Control Rates */
					_rates_sp.rpy = Vector3f(_manual_control_sp.y, _manual_control_sp.x, _manual_control_sp.r);
					_rates_sp.thrust = _manual_control_sp.z;
					controller->updateRateSetpoint(_rates_sp);
					controller->controlRates(dt);

					publish_rate_sp = true;

					break;
				}

                case vehicle_status_s::NAVIGATION_STATE_MANUAL:
                case vehicle_status_s::NAVIGATION_STATE_STAB:
                case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
                case vehicle_status_s::NAVIGATION_STATE_POSCTL:
                case vehicle_status_s::NAVIGATION_STATE_ALTCTL: {
					/* Grab Attitude Setpoint */
					generic_poll(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, _v_att_sp);

					/* Update Control Mode */
					_control_mode.mode = Controller::CONTROL_MODE::Manual;
					controller->updateControlMode(_control_mode);

					/* Control Attitude */
					_att_sp.q = _v_att_sp.q_d;
					controller->updateAttitudeSetpoint(_att_sp);
					controller->controlAttitude(dt)

					/* Control Rates */
					_rates_sp = controller->getRateSetpoint();
					controller->updateRateSetpoint(_rates_sp);
					controller->controlRates(dt); //TODO: pass armed bool flag

					publish_rate_sp = true;
					break;
				}

                case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: {
                	/* Add any independent attitude controller nav state here */
					/* Poll rates setpoint topic */
					generic_poll(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, _v_rates_sp);

					/* Update Control Mode */
					_control_mode.mode = Controller::CONTROL_MODE::Auto;
					controller->updateControlMode(_control_mode);

					/* Control Rates */
					_rates_sp.rpy(0) = _v_rates_sp.roll;
					_rates_sp.rpy(1) = _v_rates_sp.pitch;
					_rates_sp.rpy(2) = _v_rates_sp.yaw;
					_rates_sp.thrust = _v_rates_sp.thrust;

					controller->updateRateSetpoint(_rates_sp);
					controller->controlRates(dt);

					break;
				}

				case vehicle_status_s::NAVIGATION_STATE_TERMINATION: {
					controller->resetSetpoints();
					publish_rate_sp = true;
					break;
				}

                default:
                    /* Undefined navigation state - skip */
                    break;
            }

            if(publish_rate_sp){
				generic_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, _v_rates_sp);
            }

			/* Publish Actuator Controls */
            publish_output();

            /* publish controller status */
            generic_publish(ORB_ID(rate_ctrl_status), _controller_status_pub, _rate_ctrl_status);

            /* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
            if ( _vehicle_status.nav_state != vehicle_status_s::ARMING_STATE_ARMED || (now - task_start) < 3300000) {
                dt_accumulator += dt;
                ++loop_counter;

                if (dt_accumulator > 1.f) {
                    const float loop_update_rate = (float)loop_counter / dt_accumulator;
                    _loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
                    dt_accumulator = 0;
                    loop_counter = 0;
                    //TODO: move to controller
                    _lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
                    _lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
                    _lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
                }
            }
		}

		perf_end(_loop_perf);
	}

	parameter_subscribe_unsubscribe(false);
}

/* Parameter update calls */

void
DolphinAttitudeControl::parameter_subscribe_unsubscribe(bool subscribe){

	if(subscribe){
		_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
		_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
		_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
		_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
		_params_sub = orb_subscribe(ORB_ID(parameter_update));
		_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
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
		_sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));
	}
	else{
		orb_unsubscribe(_v_att_sub);
		orb_unsubscribe(_v_att_sp_sub);
		orb_unsubscribe(_v_rates_sp_sub);
		orb_unsubscribe(_v_control_mode_sub);
		orb_unsubscribe(_params_sub);
		orb_unsubscribe(_manual_control_sp_sub);
		orb_unsubscribe(_vehicle_status_sub);
		orb_unsubscribe(_motor_limits_sub);
		orb_unsubscribe(_battery_status_sub);
		for (unsigned s = 0; s < _gyro_count; s++) {
			orb_unsubscribe(_sensor_gyro_sub[s]);
		}
		orb_unsubscribe(_sensor_correction_sub);
		orb_unsubscribe(_sensor_bias_sub);
	}
}

template <typename _msg_in_struct_T>
bool DolphinAttitudeControl::generic_poll(orb_id_t msg_id, int _msg_sub, _msg_in_struct_T &_msg){

	bool updated = false;
	orb_check(_msg_sub, &updated);
	if (updated) {
		orb_copy(msg_id, _msg_sub, &_msg);

		/* Handle Special Updates */
		if (msg_id == ORB_ID(multirotor_motor_limits)){
			_saturation_status.value = _motor_limits.saturation_status;
		}
		if(msg_id == ORB_ID(parameter_update)){
			updateParams();
		}
		if(msg_id == ORB_ID(sensor_correction)){
			if (_sensor_correction.selected_gyro_instance < _gyro_count) {
				_selected_gyro = _sensor_correction.selected_gyro_instance;
			}
		}
	}

	return updated;
}

template <typename _msg_out_struct_T>
void DolphinAttitudeControl::generic_publish(orb_id_t msg_id, orb_advert_t _msg_pub, _msg_out_struct_T &_msg){

	/* Handle Special Updates */
	bool do_publish = true;
	if (msg_id == ORB_ID(vehicle_rates_setpoint)){
		/* publish attitude rates setpoint */
		_v_rates_sp.roll = _rates_sp.rpy(0);
		_v_rates_sp.pitch = _rates_sp.rpy(1);
		_v_rates_sp.yaw = _rates_sp.rpy(2);
		_v_rates_sp.thrust = _rates_sp.thrust;
		_v_rates_sp.timestamp = hrt_absolute_time();
	}
	if(msg_id == ORB_ID(actuator_controls_0)){
		if (_actuators_0_circuit_breaker_enabled) {do_publish = false;}
	}
	if(msg_id == ORB_ID(rate_ctrl_status)){
		Controller::ControllerStatus _ctrl_status = controller->getControllerStatus();
		_rate_ctrl_status.timestamp = hrt_absolute_time();
		_rate_ctrl_status.rollspeed = _ctrl_status.rates_prev(0);
		_rate_ctrl_status.pitchspeed = _ctrl_status.rates_prev(1);
		_rate_ctrl_status.yawspeed = _ctrl_status.rates_prev(2);
		_rate_ctrl_status.rollspeed_integ = _ctrl_status.rates_int(0);
		_rate_ctrl_status.pitchspeed_integ = _ctrl_status.rates_int(1);
		_rate_ctrl_status.yawspeed_integ = _ctrl_status.rates_int(2);
	}

	if(do_publish) {
		if (_msg_pub != nullptr) {
			orb_publish(msg_id, _msg_pub, &_msg);
		} else if (msg_id) {
			_msg_pub = orb_advertise(msg_id, &_msg);
		}
	}
}

void DolphinAttitudeControl::publish_output(){

	/* publish actuator controls */
	Controller::Outputs compensated_u = controller->getCompensatedControlOutput();

	_actuators.control[0] = (PX4_ISFINITE(compensated_u.actuator(0))) ? compensated_u.actuator(0) : 0.0f;
	_actuators.control[1] = (PX4_ISFINITE(compensated_u.actuator(1))) ? compensated_u.actuator(1) : 0.0f;
	_actuators.control[2] = (PX4_ISFINITE(compensated_u.actuator(2))) ? compensated_u.actuator(2) : 0.0f;
	_actuators.control[3] = (PX4_ISFINITE(compensated_u.actuator(3))) ? compensated_u.actuator(3) : 0.0f;
	_actuators.timestamp = hrt_absolute_time();
	_actuators.timestamp_sample = _sensor_gyro.timestamp;

	/* Publish Actuator Controls */
	generic_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, _actuators);
}

void
DolphinAttitudeControl::update_parameters()
{
	/* Store some of the parameters in a more convenient way & precompute often-used values */

	if(generic_poll(ORB_ID(parameter_update), _params_sub, _param_update)) {
		Controller::Limits limits;
		Controller::Gains gains;
		/* roll gains */
		limits.rate_int_lim
		gains.att_p(0) = _roll_p.get();
		gains.rate_p(0) = _roll_rate_p.get();
		gains.rate_i(0) = _roll_rate_i.get();
		limits.rate_int_lim(0) = _roll_rate_integ_lim.get();
		gains.rate_d(0) = _roll_rate_d.get();
		gains.rate_ff(0) = _roll_rate_ff.get();

		/* pitch gains */
		gains.att_p(1) = _pitch_p.get();
		gains.rate_p(1) = _pitch_rate_p.get();
		gains.rate_i(1) = _pitch_rate_i.get();
		limits.rate_int_lim(1) = _pitch_rate_integ_lim.get();
		gains.rate_d(1) = _pitch_rate_d.get();
		gains.rate_ff(1) = _pitch_rate_ff.get();

		/* yaw gains */
		gains.att_p(2) = _yaw_p.get();
		gains.rate_p(2) = _yaw_rate_p.get();
		gains.rate_i(2) = limits.rate_int_lim(2) = _yaw_rate_integ_lim.get();
		gains.rate_d(2) = _yaw_rate_d.get();
		gains.rate_ff(2) = _yaw_rate_ff.get();

		if (fabsf(gains.lp_filters_d[0].get_cutoff_freq() - _d_term_cutoff_freq.get()) > 0.01f) {
			gains.lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
			gains.lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
			gains.lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		}

		/* angular rate limits */

		limits.manual_rate_max(0) = math::radians(_roll_rate_max.get());
		limits.manual_rate_max(1) = math::radians(_pitch_rate_max.get());
		limits.manual_rate_max(2) = math::radians(_yaw_rate_max.get());

		/* auto angular rate limits */
		limits.auto_rate_max(0) = math::radians(_roll_rate_max.get());
		limits.auto_rate_max(1) = math::radians(_pitch_rate_max.get());
		limits.auto_rate_max(2) = math::radians(_yaw_auto_max.get());

		/* manual rate control acro mode rate limits and expo */
		limits.acro_rate_max(0) = math::radians(_acro_roll_max.get());
		limits.acro_rate_max(1) = math::radians(_acro_pitch_max.get());
		limits.acro_rate_max(2) = math::radians(_acro_yaw_max.get());

		controller->updateGains(gains);
		controller->updateLimits(limits);


		_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

		/* get transformation matrix from sensor/board to body frame */
		_board_rotation = get_rot_matrix((enum Rotation) _board_rotation_param.get());

		/* fine tune the rotation */
		Dcmf board_rotation_offset(Eulerf(
				M_DEG_TO_RAD_F * _board_offset_x.get(),
				M_DEG_TO_RAD_F * _board_offset_y.get(),
				M_DEG_TO_RAD_F * _board_offset_z.get()));
		_board_rotation = board_rotation_offset * _board_rotation;
	}
}

/* App Initializations */

int DolphinAttitudeControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("dp_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ATTITUDE_CONTROL,
					   1700,
					   (px4_main_t)&run_trampoline,
					   (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

DolphinAttitudeControl *DolphinAttitudeControl::instantiate(int argc, char *argv[])
{
	return new DolphinAttitudeControl();
}

int DolphinAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int dp_att_control_main(int argc, char *argv[])
{
	return DolphinAttitudeControl::main(argc, argv);
}
