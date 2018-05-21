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

	PRINT_MODULE_USAGE_NAME("dp_att_control_", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

DolphinAttitudeControl::DolphinAttitudeControl() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "dp_att_control_")),
	_lp_filters_d{
	{initial_update_rate_hz, 50.f},
	{initial_update_rate_hz, 50.f},
	{initial_update_rate_hz, 50.f}} // will be initialized correctly when params are loaded
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	_vehicle_status.is_rotary_wing = true;

	/* initialize quaternions in messages to be valid */
	_v_att.q[0] = 1.f;
	_v_att_sp.q_d[0] = 1.f;

	_rates_prev.zero();
	_rates_prev_filtered.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}

	_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
	_actuators_id = ORB_ID(actuator_controls_0);

	parameters_updated();
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
DolphinAttitudeControl::control_attitude(float dt)
{
	_thrust_sp = _v_att_sp.thrust;

	/* prepare yaw weight from the ratio between roll/pitch and yaw gains */
	Vector3f attitude_gain = _attitude_p;
	const float pitch_yaw_gain = (attitude_gain(1) + attitude_gain(2)) / 2.f;
	const float roll_w = math::constrain(attitude_gain(2) / pitch_yaw_gain, 0.f, 1.f);
	attitude_gain(0) = pitch_yaw_gain;

	/* get estimated and desired vehicle attitude */
	Quatf q(_v_att.q);
	Quatf qd(_v_att_sp.q_d);

	/* ensure input quaternions are exactly normalized because acosf(1.00001) == NaN */
	q.normalize();
	qd.normalize();

	/* calculate reduced desired attitude neglecting vehicle's roll to prioritize pitch and yaw */
	Vector3f e_z = q.dcm_x();
	Vector3f e_z_d = qd.dcm_x();
	Quatf qd_red(e_z, e_z_d);

	if (abs(qd_red(2)) > (1.f - 1e-5f) || abs(qd_red(3)) > (1.f - 1e-5f)) {
		/* In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction */
		qd_red = qd;

	} else {
		/* transform rotation from current to desired thrust vector into a world frame reduced desired attitude */
		qd_red *= q;
	}

	/* mix full and reduced desired attitude */
	Quatf q_mix = qd * qd_red.inversed();
	q_mix *= math::signNoZero(q_mix(0));
	/* catch numerical problems with the domain of acosf and asinf */
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(1) = math::constrain(q_mix(1), -1.f, 1.f);
	qd = Quatf(cosf(roll_w * acosf(q_mix(0))), sinf(roll_w * asinf(q_mix(1))), 0, 0) * qd_red;

	/* quaternion attitude control law, qe is rotation from q to qd */
	Quatf qe = q.inversed() * qd;

	/* using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	 * also taking care of the antipodal unit quaternion ambiguity */
	Vector3f eq = 2.f * math::signNoZero(qe(0)) * qe.imag();

	/* calculate angular rates setpoint */
	_rates_sp = eq.emult(attitude_gain);

	/* Feed forward the roll setpoint rate. We need to apply the roll rate in the body frame.
	 * We infer the body x axis by taking the first column of R.transposed (== q.inversed)
	 * because it's the rotation axis for body roll and multiply it by the rate and gain. */
	// TODO: Change this to full FF model based control.
	Vector3f roll_feedforward_rate = q.inversed().dcm_x();
	roll_feedforward_rate *= _v_att_sp.roll_sp_move_rate * _roll_rate_ff.get();
	_rates_sp += roll_feedforward_rate;

	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
		    !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_auto_rate_max(i), _auto_rate_max(i));

		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_dp_rate_max(i), _dp_rate_max(i));
		}
	}

}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
Vector3f
DolphinAttitudeControl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	Vector3f pidAttenuationPerAxis;
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
DolphinAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_v_control_mode.flag_armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
	}

	// get the raw gyro data and correct for thermal errors
	Vector3f rates;

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
	rates(0) -= _sensor_bias.gyro_x_bias;
	rates(1) -= _sensor_bias.gyro_y_bias;
	rates(2) -= _sensor_bias.gyro_z_bias;

	Vector3f rates_p_scaled = _rate_p.emult(pid_attenuations(_tpa_breakpoint_p.get(), _tpa_rate_p.get()));
	Vector3f rates_i_scaled = _rate_i.emult(pid_attenuations(_tpa_breakpoint_i.get(), _tpa_rate_i.get()));
	Vector3f rates_d_scaled = _rate_d.emult(pid_attenuations(_tpa_breakpoint_d.get(), _tpa_rate_d.get()));

	/* angular rates error */
	Vector3f rates_err = _rates_sp - rates;

	/* apply low-pass filtering to the rates for D-term */
	Vector3f rates_filtered(
		_lp_filters_d[0].apply(rates(0)),
		_lp_filters_d[1].apply(rates(1)),
		_lp_filters_d[2].apply(rates(2)));

	_att_control = rates_p_scaled.emult(rates_err) +
		       _rates_int -
		       rates_d_scaled.emult(rates_filtered - _rates_prev_filtered) / dt +
		       _rate_ff.emult(_rates_sp);

	_rates_prev = rates;
	_rates_prev_filtered = rates_filtered;

	/* TODO: Adjust for dolphin
	 * update integral only if motors are providing enough thrust to be effective */
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

			// Perform the integration using a first order method and do not propagate the result if out of range or invalid
			float rate_i = _rates_int(i) + rates_i_scaled(i) * rates_err(i) * dt;

			if (PX4_ISFINITE(rate_i) && rate_i > -_rate_int_lim(i) && rate_i < _rate_int_lim(i)) {
				_rates_int(i) = rate_i;
			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_rate_int_lim(i), _rate_int_lim(i));
	}
}

bool
DolphinAttitudeControl::mix_control_output(matrix::Vector3f &att_control, float thrust_sp, matrix::Vector<float, 4> & mixed_att_control) {

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
//      ROS_INFO("Shift back pitch_yaw Boost Value Positive: %f", (double) boost);
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
        auto _thrust_factor = 1.0f;
        auto _idle_speed = .1f;

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
//  mixed_att_control(0) = (outputs[0] + 1.0f) / 2.0f;
//  mixed_att_control(1) = (outputs[1] + 1.0f) / 2.0f;
//  mixed_att_control(2) = (outputs[2] + 1.0f) / 2.0f;
//  mixed_att_control(3) = (outputs[3] + 1.0f) / 2.0f;
    mixed_att_control(0) = (0+ 1.0f) / 2.0f;
    mixed_att_control(1) = (0 + 1.0f) / 2.0f;
    mixed_att_control(2) = (0+ 1.0f) / 2.0f;
    mixed_att_control(3) = (0 + 1.0f) / 2.0f;
//  PX4_INFO("Mixed Output: %f, %f, %f, %f", (double) mixed_att_control(0),
//           (double) mixed_att_control(1),
//           (double) mixed_att_control(2),
//           (double) mixed_att_control(3));
    return true;

}


bool
DolphinAttitudeControl::actuator_dynamics_compensation(actuator_controls_s &actuator_controls, float dt)
{

    return true;
}

void
DolphinAttitudeControl::battery_power_compensation(actuator_controls_s &actuator_controls, float dt)
{
	/* scale effort by battery status TODO: change this with my model */
	if (_bat_scale_en.get() && _battery_status.scale > 0.0f) {
		for (int i = 0; i < 4; i++) {
			actuator_controls.control[i] *= _battery_status.scale;
		}
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

			generic_poll(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], _sensor_gyro);
			generic_poll(ORB_ID(parameter_update), _params_sub, _param_update);
			generic_poll(ORB_ID(vehicle_status), _vehicle_status_sub, _vehicle_status);

            /*
             * Control Step 1: Compute Attitude Rates - Using switch on nav states rather than control modes for explicitness
             * Case 1: Acro mode
             * Case 2: Attitude Control
             * Case 3: Offboard rate_sp
             */

            switch (_vehicle_status.nav_state){
                case vehicle_status_s::NAVIGATION_STATE_ACRO: {
					/* ACRO mode */
					generic_poll(ORB_ID(vehicle_control_mode), _v_control_mode_sub, _v_control_mode);
					generic_poll(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, _manual_control_sp);
					Vector3f man_rate_sp;
//					Vector3f man_rate_sp(
//							math::superexpo(_manual_control_sp.y, _acro_expo_rp.get(), _acro_superexpo_rp.get()),
//							math::superexpo(-_manual_control_sp.x, _acro_expo_rp.get(), _acro_superexpo_rp.get()),
//							math::superexpo(_manual_control_sp.r, _acro_expo_y.get(), _acro_superexpo_y.get()));
					_rates_sp = man_rate_sp.emult(_acro_rate_max);
					_thrust_sp = _manual_control_sp.z;

					generic_publish(_rates_sp_id, _v_rates_sp_pub, _v_rates_sp);
					break;
				}
                case vehicle_status_s::NAVIGATION_STATE_MANUAL:
                case vehicle_status_s::NAVIGATION_STATE_STAB:
                case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
                case vehicle_status_s::NAVIGATION_STATE_POSCTL:
                case vehicle_status_s::NAVIGATION_STATE_ALTCTL: {
					/* nav states requiring att control here (rate sp generation) */
					generic_poll(ORB_ID(vehicle_control_mode), _v_control_mode_sub, _v_control_mode);
					generic_poll(ORB_ID(vehicle_attitude_setpoint), _v_att_sub, _v_att);
					generic_poll(ORB_ID(vehicle_rates_setpoint), _v_att_sp_sub, _v_att_sp);

					control_attitude(dt);


					generic_publish(_rates_sp_id, _v_rates_sp_pub, _v_rates_sp);

					break;
				}
                case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: {
                	/* Add any independent attitude controller nav state here */
					/* Poll rates setpoint topic */
					generic_poll(ORB_ID(vehicle_rates_setpoint), _v_att_sp_sub, _v_att_sp);

					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
					break;
				}
				case vehicle_status_s::NAVIGATION_STATE_TERMINATION: {
					_rates_sp.zero();
					_rates_int.zero();
					_thrust_sp = 0.0f;
					_att_control.zero();
					break;
				}
                default:
                    /* Undefined navigation state - skip */
                    break;
            }

            /*
             * Control Step 2 - Attitude Rates Control
             */
            generic_poll(ORB_ID(sensor_bias), _sensor_bias_sub, _sensor_bias);
			generic_poll(ORB_ID(sensor_correction),_sensor_correction_sub, _sensor_correction);
			generic_poll(ORB_ID(multirotor_motor_limits), _motor_limits_sub, _motor_limits);

            control_attitude_rates(dt);

            /*
             * Control Step 3 - Mix Output (TODO: move to mixer once I figure out how)
             *
             */

            mix_control_output(_att_control, _thrust_sp, _mixed_att_control);

            /* publish actuator controls */
            _actuators.control[0] = (PX4_ISFINITE(_mixed_att_control(0))) ? _mixed_att_control(0) : 0.0f;
            _actuators.control[1] = (PX4_ISFINITE(_mixed_att_control(1))) ? _mixed_att_control(1) : 0.0f;
            _actuators.control[2] = (PX4_ISFINITE(_mixed_att_control(2))) ? _mixed_att_control(2) : 0.0f;
            _actuators.control[3] = (PX4_ISFINITE(_mixed_att_control(3))) ? _mixed_att_control(3) : 0.0f;
            _actuators.timestamp = hrt_absolute_time();
            _actuators.timestamp_sample = _sensor_gyro.timestamp;


            /*
             * Control step 4 - Thruster Dynamics & Battery Voltage Compensation
             */
			actuator_dynamics_compensation(_actuators, dt);
			generic_poll(ORB_ID(battery_status), _battery_status_sub, _battery_status);
			battery_power_compensation(_actuators, dt);

			/* Publish Actuator Controls */
            generic_publish(_actuators_id, _actuators_0_pub, _actuators);

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

/*
 * UTILITIES
 */

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
void DolphinAttitudeControl::generic_poll(orb_id_t msg_id, int _msg_sub, _msg_in_struct_T &_msg){

	bool updated;
	orb_check(_msg_sub, &updated);
	if (updated) {
		orb_copy(msg_id, _msg_sub, &_msg);

		/* Handle Special Updates */
		if (msg_id == ORB_ID(multirotor_motor_limits)){
			_saturation_status.value = _motor_limits.saturation_status;
		}
		if(msg_id == ORB_ID(parameter_update)){
			updateParams();
			parameters_updated();
		}
		if(msg_id == ORB_ID(sensor_correction)){
			if (_sensor_correction.selected_gyro_instance < _gyro_count) {
				_selected_gyro = _sensor_correction.selected_gyro_instance;
			}
		}
		if(msg_id == ORB_ID(multirotor_motor_limits)){
			_saturation_status.value = _motor_limits.saturation_status;
		}

	}
}

template <typename _msg_out_struct_T>
void DolphinAttitudeControl::generic_publish(orb_id_t msg_id, orb_advert_t _msg_pub, _msg_out_struct_T &_msg){

	/* Handle Special Updates */
	bool do_publish = true;
	if (msg_id == _rates_sp_id){
		/* publish attitude rates setpoint */
		_v_rates_sp.roll = _rates_sp(0);
		_v_rates_sp.pitch = _rates_sp(1);
		_v_rates_sp.yaw = _rates_sp(2);
		_v_rates_sp.thrust = _thrust_sp;
		_v_rates_sp.timestamp = hrt_absolute_time();
	}
	if(msg_id == _actuators_id){
		if (_actuators_0_circuit_breaker_enabled) {do_publish = false;}
	}
	if(msg_id == ORB_ID(rate_ctrl_status)){
		_rate_ctrl_status.timestamp = hrt_absolute_time();
		_rate_ctrl_status.rollspeed = _rates_prev(0);
		_rate_ctrl_status.pitchspeed = _rates_prev(1);
		_rate_ctrl_status.yawspeed = _rates_prev(2);
		_rate_ctrl_status.rollspeed_integ = _rates_int(0);
		_rate_ctrl_status.pitchspeed_integ = _rates_int(1);
		_rate_ctrl_status.yawspeed_integ = _rates_int(2);
	}

	if(do_publish) {
		if (_msg_pub != nullptr) {
			orb_publish(msg_id, _msg_pub, &_msg);
		} else if (msg_id) {
			_msg_pub = orb_advertise(msg_id, &_msg);
		}
	}
}

void
DolphinAttitudeControl::parameters_updated()
{
	/* Store some of the parameters in a more convenient way & precompute often-used values */

	/* roll gains */
	_attitude_p(0) = _roll_p.get();
	_rate_p(0) = _roll_rate_p.get();
	_rate_i(0) = _roll_rate_i.get();
	_rate_int_lim(0) = _roll_rate_integ_lim.get();
	_rate_d(0) = _roll_rate_d.get();
	_rate_ff(0) = _roll_rate_ff.get();

	/* pitch gains */
	_attitude_p(1) = _pitch_p.get();
	_rate_p(1) = _pitch_rate_p.get();
	_rate_i(1) = _pitch_rate_i.get();
	_rate_int_lim(1) = _pitch_rate_integ_lim.get();
	_rate_d(1) = _pitch_rate_d.get();
	_rate_ff(1) = _pitch_rate_ff.get();

	/* yaw gains */
	_attitude_p(2) = _yaw_p.get();
	_rate_p(2) = _yaw_rate_p.get();
	_rate_i(2) = _yaw_rate_i.get();
	_rate_int_lim(2) = _yaw_rate_integ_lim.get();
	_rate_d(2) = _yaw_rate_d.get();
	_rate_ff(2) = _yaw_rate_ff.get();

	if (fabsf(_lp_filters_d[0].get_cutoff_freq() - _d_term_cutoff_freq.get()) > 0.01f) {
		_lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		_lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		_lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		_lp_filters_d[0].reset(_rates_prev(0));
		_lp_filters_d[1].reset(_rates_prev(1));
		_lp_filters_d[2].reset(_rates_prev(2));
	}

	/* angular rate limits */
	_dp_rate_max(0) = math::radians(_roll_rate_max.get());
	_dp_rate_max(1) = math::radians(_pitch_rate_max.get());
	_dp_rate_max(2) = math::radians(_yaw_rate_max.get());

	/* auto angular rate limits */
	_auto_rate_max(0) = math::radians(_roll_rate_max.get());
	_auto_rate_max(1) = math::radians(_pitch_rate_max.get());
	_auto_rate_max(2) = math::radians(_yaw_auto_max.get());

	/* manual rate control acro mode rate limits and expo */
	_acro_rate_max(0) = math::radians(_acro_roll_max.get());
	_acro_rate_max(1) = math::radians(_acro_pitch_max.get());
	_acro_rate_max(2) = math::radians(_acro_yaw_max.get());

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* get transformation matrix from sensor/board to body frame */
	_board_rotation = get_rot_matrix((enum Rotation)_board_rotation_param.get());

	/* fine tune the rotation */
	Dcmf board_rotation_offset(Eulerf(
			M_DEG_TO_RAD_F * _board_offset_x.get(),
			M_DEG_TO_RAD_F * _board_offset_y.get(),
			M_DEG_TO_RAD_F * _board_offset_z.get()));
	_board_rotation = board_rotation_offset * _board_rotation;
}

/* App Initializations */

int DolphinAttitudeControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("dp_att_control_",
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
