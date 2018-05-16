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
    parameters_updated();
}



/**
 * Position controller  - attitude only
 * Input:
 * Output:
 */
void
DolphinPositionControl::control_position_attitude(float dt)
{
    vehicle_attitude_setpoint_poll();
}

/**
 * Position controller - full state
 * Input:
 * Output:
 */
void
DolphinPositionControl::control_position_full(float dt)
{
    vehicle_attitude_setpoint_poll();

}


void
DolphinPositionControl::run()
{

    /*
     * do subscriptions
     */
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _battery_status_sub = orb_subscribe(ORB_ID(battery_status));
    _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    _v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    _v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    /* wakeup source */
    px4_pollfd_struct_t poll_fds = {};
    poll_fds.events = POLLIN;

    const hrt_abstime task_start = hrt_absolute_time();
    hrt_abstime last_run = task_start;
//    float dt_accumulator = 0.f;
//    int loop_counter = 0;

    while (!should_exit()) {

//        poll_fds.fd = _local_pos_sub;
        poll_fds.fd = _v_att_sub; // TODO: Change to _local_pos_sub, and make local_pos publish.

        /* wait for up to 100ms for data */
        int pret = px4_poll(&poll_fds, 1, 100);
        /* timed out - periodic check for should_exit() */
        if (pret == 0) {
//            continue;
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


        vehicle_status_poll();

        switch (_vehicle_status.nav_state){
            case vehicle_status_s::NAVIGATION_STATE_ACRO:
                /* Do Nothing - attitude controller will handle */
                PX4_INFO("ACRO MODE");
                break;
            case vehicle_status_s::NAVIGATION_STATE_MANUAL:
            case vehicle_status_s::NAVIGATION_STATE_STAB:
                /* Stabilized - Attitude Control Only */
                control_position_attitude(dt);
                PX4_INFO("Stabilized MODE");
                break;
            case vehicle_status_s::NAVIGATION_STATE_POSCTL:
                /* Full State Position Control*/
                control_position_full(dt);
                PX4_INFO("Position MODE");
                // TODO: Can't switch here without proper position feedback. GPS enough?
                break;
            default:
                /* Nothing */
                break;
        }

        perf_end(_loop_perf);
    }

    orb_unsubscribe(_params_sub);
    orb_unsubscribe(_battery_status_sub);
    orb_unsubscribe(_v_att_sub);
    orb_unsubscribe(_v_att_sp_sub);
    orb_unsubscribe(_v_control_mode_sub);
    orb_unsubscribe(_manual_control_sp_sub);
    orb_unsubscribe(_v_rates_sp_sub);
    orb_unsubscribe(_vehicle_status_sub);
    orb_unsubscribe(_local_pos_sub);
}

/* Parameter update calls */
void
DolphinPositionControl::parameters_updated()
{
    /* Store some of the parameters in a more convenient way & precompute often-used values */
}

void
DolphinPositionControl::parameter_update_poll()
{
    bool updated;

    /* Check if parameters have changed */
    orb_check(_params_sub, &updated);

    if (updated) {
        struct parameter_update_s param_update;
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
        updateParams();
        parameters_updated();
    }
}

void
DolphinPositionControl::vehicle_control_mode_poll()
{
    bool updated;

    /* Check if vehicle control mode has changed */
    orb_check(_v_control_mode_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
    }
}

void
DolphinPositionControl::vehicle_manual_poll()
{
    bool updated;

    /* get pilots inputs */
    orb_check(_manual_control_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
    }
}

void
DolphinPositionControl::vehicle_attitude_setpoint_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_v_att_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
    }
}

void
DolphinPositionControl::vehicle_rates_setpoint_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_v_rates_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
    }
}

void
DolphinPositionControl::vehicle_status_poll()
{
    /* check if there is new status information */
    bool updated;
    orb_check(_vehicle_status_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
    }
}


void
DolphinPositionControl::battery_status_poll()
{
    /* check if there is a new message */
    bool updated;
    orb_check(_battery_status_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
    }
}

void
DolphinPositionControl::vehicle_attitude_poll()
{
    /* check if there is a new message */
    bool updated;
    orb_check(_v_att_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
    }
}

/* App Initializations */

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

