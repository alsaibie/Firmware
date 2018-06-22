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
using namespace AController;


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
	_loop_perf(perf_alloc(PC_ELAPSED, "dp_att_control")),
	_shared_lp_filters_d{{initial_update_rate_hz, 50.f},
                       {initial_update_rate_hz, 50.f},
                       {initial_update_rate_hz, 50.f}} // will be initialized correctly when params are loaded
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.msg.gyro_scale_0[i] = 1.0f;
		_sensor_correction.msg.gyro_scale_1[i] = 1.0f;
		_sensor_correction.msg.gyro_scale_2[i] = 1.0f;
	}

	_params.msg_id = ORB_ID(parameter_update);
	_v_att.msg_id = ORB_ID(vehicle_attitude);
	_v_att_sp.msg_id = ORB_ID(vehicle_attitude_setpoint);
	_v_rates_sp.msg_id = ORB_ID(vehicle_rates_setpoint);
	_v_control_mode.msg_id = ORB_ID(vehicle_control_mode);
	_manual_control_sp.msg_id = ORB_ID(manual_control_setpoint);
	_vehicle_status.msg_id = ORB_ID(vehicle_status);
	_motor_limits.msg_id = ORB_ID(multirotor_motor_limits);
	_battery_status.msg_id = ORB_ID(battery_status);
	_sensor_gyro.msg_id = ORB_ID(sensor_gyro);
	_sensor_correction.msg_id = ORB_ID(sensor_correction);
	_sensor_bias.msg_id = ORB_ID(sensor_bias);
	_actuators.msg_id = ORB_ID(actuator_controls_0);
	_rate_ctrl_status.msg_id = ORB_ID(rate_ctrl_status);


	controller = new AttitudeController();

	controller->setFilter(_shared_lp_filters_d);

	update_parameters(true);
}

template <typename _struct_T>
void
DolphinAttitudeControl::parameter_subscribe(_struct_T &uorb_msg) {
	uorb_msg.msg_sub = orb_subscribe(uorb_msg.msg_id);
}
template <typename _struct_T>
void
DolphinAttitudeControl::parameter_unsubscribe(_struct_T &uorb_msg) {
	orb_unsubscribe(uorb_msg.msg_sub);
}

void
DolphinAttitudeControl::parameter_subscribe_unsubscribe(bool subscribe){

	if(subscribe){
		parameter_subscribe(_params);
		parameter_subscribe(_v_att);
		parameter_subscribe(_v_att_sp);
		parameter_subscribe(_v_rates_sp);
		parameter_subscribe(_v_control_mode);
		parameter_subscribe(_manual_control_sp);
		parameter_subscribe(_vehicle_status);
		parameter_subscribe(_motor_limits);
		parameter_subscribe(_battery_status);

		_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);
		if (_gyro_count == 0) {
			_gyro_count = 1;
		}
		for (unsigned s = 0; s < _gyro_count; s++) {
			_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
		}

		parameter_subscribe(_sensor_correction);
		parameter_subscribe(_sensor_bias);
	}
	else{
		parameter_unsubscribe(_params);
		parameter_unsubscribe(_v_att);
		parameter_unsubscribe(_v_att_sp);
		parameter_unsubscribe(_v_rates_sp);
		parameter_unsubscribe(_v_control_mode);
		parameter_unsubscribe(_manual_control_sp);
		parameter_unsubscribe(_vehicle_status);
		parameter_unsubscribe(_motor_limits);
		parameter_unsubscribe(_battery_status);

		for (unsigned s = 0; s < _gyro_count; s++) {
			orb_unsubscribe(_sensor_gyro_sub[s]);
		}
		parameter_unsubscribe(_sensor_correction);
		parameter_unsubscribe(_sensor_bias);
	}
}

template <typename _struct_T>
bool
DolphinAttitudeControl::parameter_poll(_struct_T &uorb_msg){

	bool updated = false;
	/* In case of gyro, update to the right sub */
	if (uorb_msg.msg_id == _sensor_gyro.msg_id){
		uorb_msg.msg_sub = _sensor_gyro_sub[_selected_gyro];
	}

	orb_check(uorb_msg.msg_sub, &updated);
	if (updated) {
		orb_copy(uorb_msg.msg_id, uorb_msg.msg_sub, &uorb_msg.msg);
		/* Handle Special Updates */
		if (uorb_msg.msg_id == _motor_limits.msg_id){
			_saturation_status.value = _motor_limits.msg.saturation_status;
		}
		else if(uorb_msg.msg_id == _sensor_correction.msg_id){
			if (_sensor_correction.msg.selected_gyro_instance < _gyro_count) {
				_selected_gyro = _sensor_correction.msg.selected_gyro_instance;
			}
		}
        else if(uorb_msg.msg_id == _params.msg_id){
            updateParams();
        }
	}
	return updated;
}
template <typename _struct_T>
void DolphinAttitudeControl::parameter_publish(_struct_T &urob_msg){

	/* Handle Special Updates */
	if (urob_msg.msg_id == _v_rates_sp.msg_id){
		/* publish attitude rates setpoint */
        AController::Setpoints::Rates controller_rates_sp = controller->getRateSetpoint();
		_v_rates_sp.msg.roll = controller_rates_sp.rpy(0);
		_v_rates_sp.msg.pitch = controller_rates_sp.rpy(1);
		_v_rates_sp.msg.yaw = controller_rates_sp.rpy(2);
		_v_rates_sp.msg.thrust = controller_rates_sp.thrust;
		_v_rates_sp.msg.timestamp = hrt_absolute_time();
//        PX4_INFO("Rates SP %f, %f, %f, %f", (double)_v_rates_sp.msg.roll, (double)_v_rates_sp.msg.pitch,
//                 (double)_v_rates_sp.msg.yaw, (double)_v_rates_sp.msg.thrust);
		if (_v_rates_sp.msg_pub != nullptr) {
			orb_publish(_v_rates_sp.msg_id, _v_rates_sp.msg_pub, &_v_rates_sp.msg);
		} else {
			_v_rates_sp.msg_pub = orb_advertise(_v_rates_sp.msg_id, &_v_rates_sp.msg);
		}
	}

	else if(urob_msg.msg_id == _actuators.msg_id && !_actuators_0_circuit_breaker_enabled){

		/* publish actuator controls */

        AController::Outputs mixed_u = controller->getMixedControlOutput();
		_actuators.msg.control[0] = (PX4_ISFINITE(mixed_u.actuator(0))) ? mixed_u.actuator(0) : 0.0f;
		_actuators.msg.control[1] = (PX4_ISFINITE(mixed_u.actuator(1))) ? mixed_u.actuator(1) : 0.0f;
		_actuators.msg.control[2] = (PX4_ISFINITE(mixed_u.actuator(2))) ? mixed_u.actuator(2) : 0.0f;
		_actuators.msg.control[3] = (PX4_ISFINITE(mixed_u.actuator(3))) ? mixed_u.actuator(3) : 0.0f;
		_actuators.msg.timestamp = hrt_absolute_time();
		_actuators.msg.timestamp_sample = _sensor_gyro.msg.timestamp;
//		PX4_INFO("Actuator Values %f, %f, %f, %f", (double)_actuators.msg.control[0], (double)_actuators.msg.control[1],
//				 (double)_actuators.msg.control[2], (double)_actuators.msg.control[3]);
		if (_actuators.msg_pub != nullptr) {
			orb_publish(_actuators.msg_id, _actuators.msg_pub, &_actuators.msg);
		} else {
			_actuators.msg_pub = orb_advertise(_actuators.msg_id, &_actuators.msg);
		}
	}

	else if(urob_msg.msg_id == _rate_ctrl_status.msg_id){
        AController::ControllerStatus _ctrl_status = controller->getControllerStatus();
		_rate_ctrl_status.msg.timestamp = hrt_absolute_time();
		_rate_ctrl_status.msg.rollspeed = _ctrl_status.rates_prev(0);
		_rate_ctrl_status.msg.pitchspeed = _ctrl_status.rates_prev(1);
		_rate_ctrl_status.msg.yawspeed = _ctrl_status.rates_prev(2);
		_rate_ctrl_status.msg.rollspeed_integ = _ctrl_status.rates_int(0);
		_rate_ctrl_status.msg.pitchspeed_integ = _ctrl_status.rates_int(1);
		_rate_ctrl_status.msg.yawspeed_integ = _ctrl_status.rates_int(2);

		if (_rate_ctrl_status.msg_pub != nullptr) {
			orb_publish(_rate_ctrl_status.msg_id, _rate_ctrl_status.msg_pub, &_rate_ctrl_status.msg);
		} else {
            _rate_ctrl_status.msg_pub = orb_advertise(_rate_ctrl_status.msg_id, &_rate_ctrl_status.msg);
		}
	}

	/* Generic Publishes */
	else{
		if (urob_msg.msg_pub != nullptr) {
			orb_publish(urob_msg.msg_id, urob_msg.msg_pub, &urob_msg.msg);
		} else if (urob_msg.msg_id) {
			urob_msg.msg_pub = orb_advertise(urob_msg.msg_id, &urob_msg.msg);
		}
	}
}

void
DolphinAttitudeControl::update_states() {

	/* Attitude */
	AController::States controller_states;
	parameter_poll(_v_att);
	controller_states.att.q = Quatf(_v_att.msg.q);

//    PX4_INFO("att q %f, %f, %f, %f", (double)_v_att.msg.q[0], (double)_v_att.msg.q[1], (double)_v_att.msg.q[2],
//             (double)_v_att.msg.q[3]);

	/* Rates */
	parameter_poll(_sensor_gyro);
	parameter_poll(_sensor_bias);
	parameter_poll(_sensor_correction);
	parameter_poll(_motor_limits);

	// get the raw gyro data and correct for thermal errors
	if (_selected_gyro == 0) {
		controller_states.rates.rpy(0) = (_sensor_gyro.msg.x - _sensor_correction.msg.gyro_offset_0[0]) * _sensor_correction.msg.gyro_scale_0[0];
		controller_states.rates.rpy(1) = (_sensor_gyro.msg.y - _sensor_correction.msg.gyro_offset_0[1]) * _sensor_correction.msg.gyro_scale_0[1];
		controller_states.rates.rpy(2) = (_sensor_gyro.msg.z - _sensor_correction.msg.gyro_offset_0[2]) * _sensor_correction.msg.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		controller_states.rates.rpy(0) = (_sensor_gyro.msg.x - _sensor_correction.msg.gyro_offset_1[0]) * _sensor_correction.msg.gyro_scale_1[0];
		controller_states.rates.rpy(1) = (_sensor_gyro.msg.y - _sensor_correction.msg.gyro_offset_1[1]) * _sensor_correction.msg.gyro_scale_1[1];
		controller_states.rates.rpy(2) = (_sensor_gyro.msg.z - _sensor_correction.msg.gyro_offset_1[2]) * _sensor_correction.msg.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		controller_states.rates.rpy(0) = (_sensor_gyro.msg.x - _sensor_correction.msg.gyro_offset_2[0]) * _sensor_correction.msg.gyro_scale_2[0];
		controller_states.rates.rpy(1) = (_sensor_gyro.msg.y - _sensor_correction.msg.gyro_offset_2[1]) * _sensor_correction.msg.gyro_scale_2[1];
		controller_states.rates.rpy(2) = (_sensor_gyro.msg.z - _sensor_correction.msg.gyro_offset_2[2]) * _sensor_correction.msg.gyro_scale_2[2];

	} else {
		controller_states.rates.rpy(0) = _sensor_gyro.msg.x;
		controller_states.rates.rpy(1) = _sensor_gyro.msg.y;
		controller_states.rates.rpy(2) = _sensor_gyro.msg.z;
	}

	// rotate corrected measurements from sensor to body frame
	controller_states.rates.rpy = _board_rotation * controller_states.rates.rpy;

	// correct for in-run bias errors
	controller_states.rates.rpy(0) -= _sensor_bias.msg.gyro_x_bias;
	controller_states.rates.rpy(1) -= _sensor_bias.msg.gyro_y_bias;
	controller_states.rates.rpy(2) -= _sensor_bias.msg.gyro_z_bias;

	/* Power */
	parameter_poll(_battery_status);
	//TODO: Add more battery parameters, like nominal voltage and such. Also add it in uwsim_sensor
	controller_states.power.bat_scale_en = _bat_scale_en.get();
	controller_states.power.bat_scale_mA = _battery_status.msg.remaining;

	/* And update to controller */
	controller->updateStates(controller_states);
}

void
DolphinAttitudeControl::update_parameters(bool force)
{
	/* Store some of the parameters in a more convenient way & precompute often-used values */
    const float rate_gain_scale_ = .01f; //TODO: move as param?

    if(parameter_poll(_params) || force) {
	    PX4_INFO("Hello");
		AController::Limits limits = controller->getLimits();
		AController::Gains gains  = controller->getGains();
        AController::Constants constants;

		/* roll gains */
		gains.att_p(0) = _roll_p.get();
		gains.rate_p(0) = _roll_rate_p.get() * rate_gain_scale_;
		gains.rate_i(0) = _roll_rate_i.get() * rate_gain_scale_;
		limits.rate_int_lim(0) = _roll_rate_integ_lim.get() * rate_gain_scale_;
		gains.rate_d(0) = _roll_rate_d.get() * rate_gain_scale_;
		gains.rate_ff(0) = _roll_rate_ff.get() * rate_gain_scale_;
        gains.att_ff(0) = _roll_ff.get();

		/* pitch gains */
		gains.att_p(1) = _pitch_p.get();
		gains.rate_p(1) = _pitch_rate_p.get() * rate_gain_scale_;
		gains.rate_i(1) = _pitch_rate_i.get() * rate_gain_scale_;
		limits.rate_int_lim(1) = _pitch_rate_integ_lim.get() * rate_gain_scale_;
		gains.rate_d(1) = _pitch_rate_d.get() * rate_gain_scale_;
		gains.rate_ff(1) = _pitch_rate_ff.get() * rate_gain_scale_;

		/* yaw gains */
		gains.att_p(2) = _yaw_p.get();
		gains.rate_p(2) = _yaw_rate_p.get() * rate_gain_scale_;
		gains.rate_i(2) = limits.rate_int_lim(2) = _yaw_rate_integ_lim.get() * rate_gain_scale_;
		gains.rate_d(2) = _yaw_rate_d.get() * rate_gain_scale_;
		gains.rate_ff(2) = _yaw_rate_ff.get() * rate_gain_scale_;

		/* low pass filter*/
		if (fabsf(_shared_lp_filters_d[0].get_cutoff_freq() - _d_term_cutoff_freq.get()) > 0.01f) {
			_shared_lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
			_shared_lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
			_shared_lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		}

		/* angular rate limits */

		limits.manual_rate_max(0) = math::radians(_roll_rate_max.get());
		limits.manual_rate_max(1) = math::radians(_pitch_rate_max.get());
		limits.manual_rate_max(2) = math::radians(_yaw_rate_max.get());

		/* auto angular rate limits */
		limits.auto_rate_max(0) = math::radians(_roll_auto_max.get());
		limits.auto_rate_max(1) = math::radians(_pitch_rate_max.get());
		limits.auto_rate_max(2) = math::radians(_yaw_rate_max.get());

		/* manual rate control acro mode rate limits and expo */
		limits.acro_rate_max(0) = math::radians(_acro_roll_max.get());
		limits.acro_rate_max(1) = math::radians(_acro_pitch_max.get());
		limits.acro_rate_max(2) = math::radians(_acro_yaw_max.get());

		/* Expo Rates */
		limits.acro_expo_py = math::radians(_acro_expo_py.get());
		limits.acro_expo_r = math::radians(_acro_expo_r.get());
		limits.acro_superexpo_py = math::radians(_acro_superexpo_py.get());
		limits.acro_superexpo_r = math::radians(_acro_superexpo_r.get());

        constants.Vn_norm_coefficients(0) = _c0_Vn_norm.get();
        constants.Vn_norm_coefficients(1) = _c1_Vn_norm.get();
        constants.Vn_norm_coefficients(2) = _c2_Vn_norm.get();
        PX4_INFO("Vn norm Coefficients: %f, %f, %f", (double)constants.Vn_norm_coefficients(0), (double)constants.Vn_norm_coefficients(1),
                 (double)constants.Vn_norm_coefficients(2));

        constants.VT_coefficients(0) = _c0_VT.get();
        constants.VT_coefficients(1) = _c1_VT.get();
        constants.VT_coefficients(2) = _c2_VT.get();
        PX4_INFO("VT Coefficients: %f, %f, %f", (double)constants.VT_coefficients(0), (double)constants.VT_coefficients(1),
                 (double)constants.VT_coefficients(2));

        constants.VTorq_coefficients(0) = 0.0f;
        constants.VTorq_coefficients(1) = 0.0f;
        constants.VTorq_coefficients(2) = _c2_VTORQ.get();
        PX4_INFO("VT Coefficients: %f, %f, %f", (double)constants.VTorq_coefficients(0), (double)constants.VTorq_coefficients(1),
                 (double)constants.VTorq_coefficients(2));

    	constants.rotor_radius = _rotor_radius.get();

		controller->updateGains(gains);
		controller->updateLimits(limits);
		controller->updateConstants(constants);

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
			parameter_poll(_vehicle_status);
			update_parameters();

			/*
			 * Update States
			 * */
            update_states();

            /*
             * Control Step 1: Compute Attitude Rates - Using switch on nav states rather than control modes for explicitness
             * Case 1: Acro mode
             * Case 2: Attitude Control
             * Case 3: Offboard rate_sp
             */
			bool publish_rate_sp = false;

			AController::ControlMode _controller_mode;

			if(_vehicle_status.msg.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {_controller_mode.is_armed = true;}
			else {_controller_mode.is_armed = false;}

            switch (_vehicle_status.msg.nav_state){
                case vehicle_status_s::NAVIGATION_STATE_ACRO: {
					/* ACRO mode - Rate Setpoint from Manual Input */
					parameter_poll(_manual_control_sp);

					_controller_mode.mode = AController::CONTROL_MODE::Acro;
					controller->updateControlMode(_controller_mode);

					AController::Setpoints::Rates controller_rates_sp;
					controller_rates_sp.rpy = Vector3f(_manual_control_sp.msg.y, _manual_control_sp.msg.x, _manual_control_sp.msg.r);
					controller_rates_sp.thrust = 2.0f * _manual_control_sp.msg.z - 1.0f;
					controller->updateRateSetpoint(controller_rates_sp);
					controller->controlRates(dt);

					publish_rate_sp = true;

					break;
				}

                case vehicle_status_s::NAVIGATION_STATE_MANUAL:
                case vehicle_status_s::NAVIGATION_STATE_STAB:
                case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
                case vehicle_status_s::NAVIGATION_STATE_POSCTL:
                case vehicle_status_s::NAVIGATION_STATE_ALTCTL: {
					parameter_poll(_v_att_sp);

					_controller_mode.mode = AController::CONTROL_MODE::Manual;
					controller->updateControlMode(_controller_mode);

					AController::Setpoints::Attitude controller_att_sp;
					controller_att_sp.q = _v_att_sp.msg.q_d;

					controller->updateAttitudeSetpoint(controller_att_sp);
					controller->controlAttitude(dt);


                    AController::Setpoints::Rates controller_rates_sp;
					controller_rates_sp = controller->getRateSetpoint();
					controller_rates_sp.thrust = _v_att_sp.msg.thrust;

					controller->updateRateSetpoint(controller_rates_sp);
					controller->controlRates(dt);

					publish_rate_sp = true;
					break;
				}

                case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: {
                	/* Add any independent attitude controller nav state here */
					parameter_poll(_v_rates_sp);

					_controller_mode.mode = AController::CONTROL_MODE::Auto;
					controller->updateControlMode(_controller_mode);

                    AController::Setpoints::Rates controller_rates_sp;
					controller_rates_sp.rpy(0) = _v_rates_sp.msg.roll;
					controller_rates_sp.rpy(1) = _v_rates_sp.msg.pitch;
					controller_rates_sp.rpy(2) = _v_rates_sp.msg.yaw;
					controller_rates_sp.thrust = _v_rates_sp.msg.thrust;

					controller->updateRateSetpoint(controller_rates_sp);
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

            if(publish_rate_sp){ parameter_publish(_v_rates_sp); }

			/* Publish Actuator Controls */
			parameter_publish(_actuators);

            /* publish controller status */
            parameter_publish(_rate_ctrl_status);

            /* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
            if ( _vehicle_status.msg.nav_state != vehicle_status_s::ARMING_STATE_ARMED || (now - task_start) < 3300000) {
                dt_accumulator += dt;
                ++loop_counter;

                if (dt_accumulator > 1.f) {
                    const float loop_update_rate = (float)loop_counter / dt_accumulator;
                    _loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
                    dt_accumulator = 0;
                    loop_counter = 0;
                    //TODO: move to controller
					_shared_lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
					_shared_lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
					_shared_lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
                }
            }
		}

		perf_end(_loop_perf);
	}

	parameter_subscribe_unsubscribe(false);
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
