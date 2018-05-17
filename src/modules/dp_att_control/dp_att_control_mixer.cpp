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

/** Dolphin Mixer
 * Input: att_control_
 * Output: mixed_att_output_**/
bool
DolphinAttitudeControl::mix_control_output(matrix::Vector3f &att_control, float thrust, matrix::Vector<float, 4> & mixed_att_control) {

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

}
