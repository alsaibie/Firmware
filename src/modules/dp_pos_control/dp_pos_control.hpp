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
 * @author Ali AlSaibie
 */
#pragma once

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <cfloat>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <systemlib/pid/pid.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/uORB.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

using matrix::Dcmf;




/**
 * Dolphin position control app start / stop handling function
 * TODO: Write up a description of the inputs and outputs here in brief
 */
extern "C" __EXPORT int dp_pos_control_main(int argc, char *argv[]);

class DolphinPositionControl : public ModuleBase<DolphinPositionControl>, public ModuleParams
{
public:
    DolphinPositionControl();

    virtual ~DolphinPositionControl() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static DolphinPositionControl *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

private:

    /**
     * Position controllers
     */
    void		control_position_attitude(float dt);
    void		control_position_full(float dt);

    /**
     * initialize some vectors/matrices from parameters
     */
    void		parameters_updated();

    /**
     * Check for parameter update and handle it.
     */
    void		parameter_update_poll();
    void		battery_status_poll();
    void		vehicle_attitude_poll();
    void		vehicle_attitude_setpoint_poll();
    void		vehicle_control_mode_poll();
    void		vehicle_manual_poll();
    void		vehicle_rates_setpoint_poll();
    void		vehicle_status_poll();

private:

    /**
     * Throttle PID attenuation.
     */
    matrix::Vector3f pid_attenuations(float tpa_breakpoint, float tpa_rate);


    int		_params_sub{-1};		    /**< parameter updates subscription */
    int		_battery_status_sub{-1};	/**< battery status subscription */
    int		_v_att_sub{-1};			    /**< vehicle attitude subscription */
    int		_v_att_sp_sub{-1};		    /**< vehicle attitude setpoint subscription */
    int		_v_control_mode_sub{-1};	/**< vehicle control mode subscription */
    int		_manual_control_sp_sub{-1};	/**< manual control setpoint subscription */
    int		_v_rates_sp_sub{-1};		/**< vehicle rates setpoint subscription */
    int		_vehicle_status_sub{-1};	/**< vehicle status subscription */
    int		_local_pos_sub{-1};			    /**< vehicle local position */


    orb_advert_t	_v_att_sp_pub{nullptr};		/**< attitude setpoint publication */
//    orb_advert_t	_controller_status_pub{nullptr};	/**< controller status publication */

    orb_id_t _att_sp_id{nullptr};		/**< pointer to correct rates setpoint uORB metadata structure */

    bool		_actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */

    struct battery_status_s			    _battery_status {};	/**< battery status */
    struct vehicle_attitude_s		    _v_att {};		        /**< vehicle attitude */
    struct vehicle_attitude_setpoint_s	_v_att_sp {};		    /**< vehicle attitude setpoint */
    struct vehicle_control_mode_s		_v_control_mode {};	    /**< vehicle control mode */
    struct manual_control_setpoint_s	_manual_control_sp {};	/**< manual control setpoint */
    struct vehicle_rates_setpoint_s		_v_rates_sp {};		    /**< vehicle rates setpoint */
    struct vehicle_status_s			    _vehicle_status {};	    /**< vehicle status */
    struct vehicle_local_position_s		_local_pos {};		    /**< vehicle local position */

    perf_counter_t	_loop_perf;			/**< loop performance counter */

    static constexpr const float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
    float _loop_update_rate_hz{initial_update_rate_hz};          /**< current rate-controller loop update rate in [Hz] */

    DEFINE_PARAMETERS(
    (ParamFloat<px4::params::DPC_THR_CRUISE>) _thrust_cruise,
    (ParamFloat<px4::params::DPC_THR_MAX>) _thrust_max,
    (ParamFloat<px4::params::DPC_THR_MIN>) _thrust_min,
    (ParamFloat<px4::params::DPC_THR_IDLE>) _thrust_idle,
    (ParamInt<px4::params::DPC_SP_CTRL_MODE>) _speed_ctrl_mode
    )
};