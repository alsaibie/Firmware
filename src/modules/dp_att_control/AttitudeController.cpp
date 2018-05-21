//
// Created by alsaibie on 5/18/18.
//

#include "AttitudeController.hpp"

#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

AttitudeController::AttitudeController()
{

    /* Zero States, Setpoints and Outputs */
    _state.att.q.zero();
    _state.att.q(0) = 1.0;
    _state.rates.rpy.zero();
    _state.power.bat_scale_mA = 0;
    _state.power.bat_scale_en = false;
    _att_sp.q.zero();
    _att_sp.q(0) = 1.0;
    _rates_sp.rpy.zero();
    _rates_sp.roll_move_rate = 0;
    _rates_sp.thrust = 0;
    _rates_actuator_u.zero();
    _mixed_actuator_u.zero();
    _compensated_u.actuator.zero();

    _gains.lp_filters_d[3] = {{initial_update_rate_hz, 50.f},
                             {initial_update_rate_hz, 50.f},
                             {initial_update_rate_hz, 50.f}};
}

void
AttitudeController::resetSetpoints() {

    _rates_sp.rpy.zero();
    _rates_sp.thrust = 0.0f;
    _ctrl_status.rates_int.zero();
    _rates_actuator_u.zero();
    _mixed_actuator_u.zero();
    _compensated_u.actuator.zero();
}

void AttitudeController::controlAttitude(const float &dt) {

    /* prepare yaw weight from the ratio between roll/pitch and yaw gains */
    Vector3f attitude_gain = _gains.att_p;
    const float pitch_yaw_gain = (attitude_gain(1) + attitude_gain(2)) / 2.f;
    const float roll_w = math::constrain(attitude_gain(2) / pitch_yaw_gain, 0.f, 1.f);
    attitude_gain(0) = pitch_yaw_gain;

    Quatf q(_state.att.q);
    Quatf qd(_att_sp.q);
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
    _rates_sp.rpy = eq.emult(attitude_gain);

    /* Feed forward the roll setpoint rate. We need to apply the roll rate in the body frame.
     * We infer the body x axis by taking the first column of R.transposed (== q.inversed)
     * because it's the rotation axis for body roll and multiply it by the rate and gain. */
    // TODO: Change this to full FF model based control.
    Vector3f roll_feedforward_rate = q.inversed().dcm_x();
    roll_feedforward_rate *= _rates_sp.roll_move_rate * _gains.rate_ff(0);
    _rates_sp.rpy += roll_feedforward_rate;

    /* limit rates */
    for (int i = 0; i < 3; i++) {
        if (_mode.mode == Controller::CONTROL_MODE::Auto){
            _rates_sp.rpy(i) = math::constrain(_rates_sp.rpy(i), -_limits.auto_rate_max(i), _limits.acro_rate_max(i));

        } else {
            _rates_sp.rpy(i) = math::constrain(_rates_sp.rpy(i), -_limits.manual_rate_max(i), _limits.manual_rate_max(i));
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
AttitudeController::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
    /* throttle pid attenuation factor */
    float tpa = 1.0f - tpa_rate * (fabsf(_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
    tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

    Vector3f pidAttenuationPerAxis;
    pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
    pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
    pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

    return pidAttenuationPerAxis;
}
void
AttitudeController::limitSaturation() {

}

void AttitudeController::mixOutput() {
    /***
  * Mixing strategy: adopted from mixer_multirotor.cpp
  * 1) Mix pitch, yaw and thrust without roll. And calculate max and min outputs
  * 2) Shift all outputs to minimize out of range violations, min or max. If not enough room to shift all outputs, then scale back pitch/yaw.
  * 3) If there is violation on both upper and lower end then shift such that the violation is equal on both sides.
  * 4) Mix in roll and see if it leads to limit violation, scale back output to allow for roll.
  * 5) Scale all output to range [-1, 1]
  * */

    float roll = math::constrain(_rates_actuator_u(0), -1.0f, 1.0f);
    float pitch = math::constrain(_rates_actuator_u(1), -1.0f, 1.0f);
    float yaw = math::constrain(_rates_actuator_u(2), -1.0f, 1.0f);
    float thrust = math::constrain(_rates_sp.thrust, -1.0f, 1.0f);
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

    /*** perform initial mix pass yielding unbounded outputs, ignore roll */
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

    /* Copy outputs to premixed att control output */
//  _mixed_actuator_u(0) = (outputs[0] + 1.0f) / 2.0f;
//  _mixed_actuator_u(1) = (outputs[1] + 1.0f) / 2.0f;
//  _mixed_actuator_u(2) = (outputs[2] + 1.0f) / 2.0f;
//  _mixed_actuator_u(3) = (outputs[3] + 1.0f) / 2.0f;
    _mixed_actuator_u(0) = (0+ 1.0f) / 2.0f;
    _mixed_actuator_u(1) = (0 + 1.0f) / 2.0f;
    _mixed_actuator_u(2) = (0+ 1.0f) / 2.0f;
    _mixed_actuator_u(3) = (0 + 1.0f) / 2.0f;

}

void AttitudeController::compensateThrusterDynamics(const float &dt) {


    /* Battery Voltage Compensation */
    /* scale effort by battery status TODO: change this with my model */
    if (_state.power.bat_scale_en && _state.power.bat_scale_mA > 0.0f) {
        for (int i = 0; i < 4; i++) {
            actuator_controls.control[i] *= _state.power.bat_scale_mA;
        }
    }
}

void AttitudeController::controlRates(const float &dt) {
    /* reset integral if disarmed */
    if (!_mode.is_armed) {
        _ctrl_status.rates_int.zero();
    }

    if(_mode.mode == Controller::CONTROL_MODE::Acro) {
        Vector3f(
                //TODO: Change limit scaling to py and roll
        math::superexpo(_rates_sp.rpy(0), _limits.acro_expo_rp, _limits.acro_superexpo_rp),
        math::superexpo(_rates_sp.rpy(1), _limits.acro_expo_rp, _limits.acro_superexpo_rp),
        math::superexpo(_rates_sp.rpy(2), _limits.acro_expo_y, _limits.acro_superexpo_y));
        _rates_sp.rpy = _rates_sp.rpy.emult(_limits.acro_rate_max);
    }

    Vector3f rates_p_scaled = _gains.rate_p.emult(pid_attenuations(_gains._tpa_breakpoint_p, _gains._tpa_rate_p));
    Vector3f rates_i_scaled = _gains.rate_i.emult(pid_attenuations(_gains._tpa_breakpoint_i, _gains._tpa_rate_i));
    Vector3f rates_d_scaled = _gains.rate_d.emult(pid_attenuations(_gains._tpa_breakpoint_d, _gains._tpa_rate_d));

    /* angular rates error */
    Vector3f rates_err = _rates_sp.rpy - _state.rates.rpy;

    /* apply low-pass filtering to the rates for D-term */
    Vector3f rates_filtered(
            _gains.lp_filters_d[0].apply(_state.rates.rpy(0)),
            _gains.lp_filters_d[1].apply(_state.rates.rpy(1)),
            _gains.lp_filters_d[2].apply(_state.rates.rpy(2)));

    _rates_actuator_u = rates_p_scaled.emult(rates_err) +
            _ctrl_status.rates_int -
                   rates_d_scaled.emult(rates_filtered - _ctrl_status.rates_prev_filtered) / dt +
                   _gains.rate_ff.emult(_rates_sp.rpy);

    _ctrl_status.rates_prev = _state.rates.rpy;
    _ctrl_status.rates_prev_filtered = rates_filtered;

    limitSaturation();
    /* TODO: Adjust for dolphin
     * update integral only if motors are providing enough thrust to be effective */
//    if (_rates_sp.thrust > MIN_TAKEOFF_THRUST) {
//        for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
//            // Check for positive control saturation
//            bool positive_saturation =
//                    ((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
//                    ((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
//                    ((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);
//
//            // Check for negative control saturation
//            bool negative_saturation =
//                    ((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
//                    ((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
//                    ((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);
//
//            // prevent further positive control saturation
//            if (positive_saturation) {
//                rates_err(i) = math::min(rates_err(i), 0.0f);
//            }
//
//            // prevent further negative control saturation
//            if (negative_saturation) {
//                rates_err(i) = math::max(rates_err(i), 0.0f);
//            }
//
//            // Perform the integration using a first order method and do not propagate the result if out of range or invalid
//            float rate_i = _rates_int(i) + rates_i_scaled(i) * rates_err(i) * dt;
//
//            if (PX4_ISFINITE(rate_i) && rate_i > -_rate_int_lim(i) && rate_i < _rate_int_lim(i)) {
//                _rates_int(i) = rate_i;
//            }
//        }
//    }

    /* explicitly limit the integrator state */
    for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
        _ctrl_status.rates_int(i) = math::constrain(_ctrl_status.rates_int(i),
                                                    -_ctrl_status.rate_int_lim(i), _ctrl_status.rate_int_lim(i));
    }

    mixOutput();
    compensateThrusterDynamics(dt);
}








