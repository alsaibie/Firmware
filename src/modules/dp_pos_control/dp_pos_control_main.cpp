/****************************************************************************
 *
 *   Copyright (c) 2017 Ali AlSaibie. All rights reserved.
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
 * @file
 *
 * @author  by Ali AlSaibie <ali@alsaibie.com>
 */

#include "dp_pos_control.hpp"

#define STICK_DZ 0.10f

#define APPLY_DZ(x) (fabs(x) > STICK_DZ ? x : 0.0f )

int DolphinPositionControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }
    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
TODO: FIX


)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("dp_pos_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

DolphinPositionControl::DolphinPositionControl() :
        ModuleParams(nullptr),
        _loop_perf(perf_alloc(PC_ELAPSED, "dp_pos_control"))
{

    /* Initialize variables */
    _params.msg_id          = ORB_ID(parameter_update);
    _v_att.msg_id           = ORB_ID(vehicle_attitude);
    _v_att_sp.msg_id        = ORB_ID(vehicle_attitude_setpoint);
    _v_rates_sp.msg_id      = ORB_ID(vehicle_rates_setpoint);
    _v_control_mode.msg_id  = ORB_ID(vehicle_control_mode);
    _manual_control_sp.msg_id   = ORB_ID(manual_control_setpoint);
    _vehicle_status.msg_id      = ORB_ID(vehicle_status);
    _battery_status.msg_id      = ORB_ID(battery_status);
    _local_pos.msg_id           = ORB_ID(vehicle_local_position);
    _local_pos_sp.msg_id        = ORB_ID(vehicle_local_position_setpoint);


    controller = new PositionController();
    update_parameters(true);
}



template <typename _struct_T>
void
DolphinPositionControl::parameter_subscribe(_struct_T&uorb_msg) {
    uorb_msg.msg_sub = orb_subscribe(uorb_msg.msg_id);
}
template <typename _struct_T>
void
DolphinPositionControl::parameter_unsubscribe(_struct_T &uorb_msg) {
    orb_unsubscribe(uorb_msg.msg_sub);
}

void
DolphinPositionControl::parameter_subscribe_unsubscribe(bool subscribe){

    if(subscribe){
        parameter_subscribe(_params);
        parameter_subscribe(_v_att);
        parameter_subscribe(_v_att_sp);
        parameter_subscribe(_v_rates_sp);
        parameter_subscribe(_v_control_mode);
        parameter_subscribe(_manual_control_sp);
        parameter_subscribe(_vehicle_status);
        parameter_subscribe(_battery_status);
        parameter_subscribe(_local_pos);
        parameter_subscribe(_local_pos_sp);

    }
    else{
        parameter_unsubscribe(_params);
        parameter_unsubscribe(_v_att);
        parameter_unsubscribe(_v_att_sp);
        parameter_unsubscribe(_v_rates_sp);
        parameter_unsubscribe(_v_control_mode);
        parameter_unsubscribe(_manual_control_sp);
        parameter_unsubscribe(_vehicle_status);
        parameter_unsubscribe(_battery_status);
        parameter_unsubscribe(_local_pos);
        parameter_unsubscribe(_local_pos_sp);

    }
}

template <typename _struct_T>
bool
DolphinPositionControl::parameter_poll(_struct_T &uorb_msg){

    bool updated = false;

    orb_check(uorb_msg.msg_sub, &updated);
    if (updated) {
        orb_copy(uorb_msg.msg_id, uorb_msg.msg_sub, &uorb_msg.msg);
        /* Handle Special Updates */
        if(uorb_msg.msg_id == _params.msg_id){
            updateParams();
        }

    }
    return updated;
}
template <typename _struct_T>
void DolphinPositionControl::parameter_publish(_struct_T &urob_msg){

    /* Handle Special Updates */
    if(urob_msg.msg_id == _v_att_sp.msg_id){
        Controller::Outputs _att_d = controller->getDesiredAttitude();
        _v_att_sp.msg.timestamp = hrt_absolute_time();
        _v_att_sp.msg.q_d[0] = _att_d.orientation_q(0);
        _v_att_sp.msg.q_d[1] = _att_d.orientation_q(1);
        _v_att_sp.msg.q_d[2] = _att_d.orientation_q(2);
        _v_att_sp.msg.q_d[3] = _att_d.orientation_q(3);
        _v_att_sp.msg.thrust = _att_d.thrust;

        if (_v_att_sp.msg_pub != nullptr) {
            orb_publish(_v_att_sp.msg_id, _v_att_sp.msg_pub, &_v_att_sp.msg);
        } else {
            _v_att_sp.msg_pub = orb_advertise(_v_att_sp.msg_id, &_v_att_sp.msg);
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
DolphinPositionControl::update_parameters(bool force)
{
    /* Store some of the parameters in a more convenient way & precompute often-used values */

    if(parameter_poll(_params) || force) {

        Controller::Limits limits = controller->getLimits();
        Controller::Gains gains  = controller->getGains();
        //TODO: Define parameters
        limits.max_att_angle(0) = math::radians(_max_roll_angle.get());
        limits.max_att_angle(1) = math::radians(_max_tilt_angle.get());
        limits.max_att_angle(2) = math::radians(_max_tilt_angle.get());


        controller->updateGains(gains);
        controller->updateLimits(limits);

    }
}

void
DolphinPositionControl::update_states() {

    /* Position */
    Controller::States controller_states {};
    parameter_poll(_local_pos);
    parameter_poll(_v_att);
    controller_states.pos.p(0) = _local_pos.msg.x;
    controller_states.pos.p(1) = _local_pos.msg.y;
    controller_states.pos.p(2) = _local_pos.msg.z;
    controller_states.vel.v(0) = _local_pos.msg.vx;
    controller_states.vel.v(1) = _local_pos.msg.vy;
    controller_states.vel.v(2) = _local_pos.msg.vz;

    controller_states.att.orientation_q = Quatf(_v_att.msg.q);

    /* Power */
    parameter_poll(_battery_status);
    //TODO: Add more battery parameters, like nominal voltage and such. Also add it in uwsim_sensor
    controller_states.power.bat_scale_mA = _battery_status.msg.remaining;

    /* And update to controller */
    controller->updateStates(controller_states);
}


void
DolphinPositionControl::run()
{

    parameter_subscribe_unsubscribe(true);

    /* wakeup source */
    px4_pollfd_struct_t poll_fds = {};
    poll_fds.events = POLLIN;

    const hrt_abstime task_start = hrt_absolute_time();
    hrt_abstime last_run = task_start;
//    float dt_accumulator = 0.f;
//    int loop_counter = 0;

    while (!should_exit()) {

//        poll_fds.fd = _local_pos_sub;
        poll_fds.fd = _v_att.msg_sub; // TODO: Change to _local_pos_sub, and make local_pos publish.

        /* wait for up to 100ms for data */
        int pret = px4_poll(&poll_fds, 1, 100);
        /* timed out - periodic check for should_exit() */
        if (pret == 0) {
//            continue;
            // TODO: why isn't this behaving properly?
        }
        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
            PX4_ERR("poll error %d, %d", pret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;
        }

        perf_begin(_loop_perf);

        const hrt_abstime now = hrt_absolute_time();
        float dt = (now - last_run) / 1e6f;
        last_run = now;

        /* guard against too small (< 2ms) and too large (> 20ms) dt's */
        if (dt < 0.002f) {
            dt = 0.002f;
        } else if (dt > 0.02f) {
            dt = 0.02f;
        }


        parameter_poll(_vehicle_status);
        update_parameters();

        /*
         * Update States
         * */
        update_states();

        Controller::ControlMode _controller_mode;

        if(_vehicle_status.msg.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {_controller_mode.is_armed = true;}
        else {_controller_mode.is_armed = false;}

        switch (_vehicle_status.msg.nav_state){
            case vehicle_status_s::NAVIGATION_STATE_ACRO:
                /* Do Nothing - attitude controller will handle */
                break;
            case vehicle_status_s::NAVIGATION_STATE_MANUAL:
            case vehicle_status_s::NAVIGATION_STATE_STAB:
            {
                /* Stabilized - Attitude Control Only */
                _controller_mode.mode = Controller::CONTROL_MODE::Manual;
                controller->updateControlMode(_controller_mode);


                parameter_poll(_manual_control_sp);
                Controller::Setpoints::Attitude att_sp;
                att_sp.thrust = 2.0f * APPLY_DZ(_manual_control_sp.msg.z - 0.5f);
                att_sp.orientation = Vector3f(APPLY_DZ(-_manual_control_sp.msg.y),
                                              APPLY_DZ(_manual_control_sp.msg.x),
                                              APPLY_DZ(-_manual_control_sp.msg.r));

                controller->updateAttitudeSetpoint(att_sp);

                /* If unarmed or reference reset requested
                 * TODO: Find a good way to intercept an assigned keypress,
                 * */
                if(!_controller_mode.is_armed){ controller->resetReferenceState(); }

                controller->controlAttitude(dt);

                break;
            }
            case vehicle_status_s::NAVIGATION_STATE_POSCTL:
            {
                /* This is velocity control */
                PX4_INFO("Position MODE");
                // TODO: Can't switch here without proper position feedback. GPS enough?
                controller->controlPosition(dt);
                break;
            }
            default:
                /* Nothing */
                break;
        }

        parameter_publish(_v_att_sp);
        perf_end(_loop_perf);
    }

    parameter_subscribe_unsubscribe(false);
}

/* Standard App Initializations */

int DolphinPositionControl::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("dp_pos_control",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_POSITION_CONTROL,
                                  1700,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);
    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }
    return 0;
}

DolphinPositionControl *DolphinPositionControl::instantiate(int argc, char *argv[])
{
    return new DolphinPositionControl();
}

int DolphinPositionControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int dp_pos_control_main(int argc, char *argv[])
{
    return DolphinPositionControl::main(argc, argv);
}

