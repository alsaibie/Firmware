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
#include <uORB/topics/vehicle_local_position_setpoint.h>

#include <uORB/uORB.h>

#include "PositionController.hpp"

namespace Dolphin {
    template <typename _struct_T>
    struct uorb_msg{
        orb_id_t        msg_id {nullptr};
        orb_advert_t	msg_pub {nullptr};
        int             msg_sub {-1};
        _struct_T msg {};
    };
}

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
     * Controller
     */

    PositionController *controller;

    /**
	 * initialize some vectors/matrices from parameters
	 */
    template <typename _struct_T>
    void            parameter_subscribe(_struct_T &uorb_msg);
    template <typename _struct_T>
    void            parameter_unsubscribe(_struct_T &uorb_msg);
    void            parameter_subscribe_unsubscribe(bool subscribe);

    /**
     * Check for msgs updates and handle it. TODO: specify type, remove unnecessary template
     */
    template <typename _struct_T>
    bool        parameter_poll(_struct_T &uorb_msg);

    /**
     * Publish msgs
     */
    template <typename _struct_T>
    void        parameter_publish(_struct_T &urob_msg);

    /**
     * Update States and parameters
     */

    void        update_states();
    void        update_parameters(bool force = false);

    Dolphin::uorb_msg<parameter_update_s>                       _params {};
    Dolphin::uorb_msg<battery_status_s>			                _battery_status {};	/**< battery status */
    Dolphin::uorb_msg<vehicle_attitude_s>		                _v_att {};		        /**< vehicle attitude */
    Dolphin::uorb_msg<vehicle_attitude_setpoint_s>	            _v_att_sp {};		    /**< vehicle attitude setpoint */
    Dolphin::uorb_msg<vehicle_control_mode_s>		            _v_control_mode {};	    /**< vehicle control mode */
    Dolphin::uorb_msg<manual_control_setpoint_s>	            _manual_control_sp {};	/**< manual control setpoint */
    Dolphin::uorb_msg<vehicle_rates_setpoint_s>		            _v_rates_sp {};		    /**< vehicle rates setpoint */
    Dolphin::uorb_msg<vehicle_status_s>			                _vehicle_status {};	    /**< vehicle status */
    Dolphin::uorb_msg<vehicle_local_position_s>		            _local_pos {};		    /**< vehicle local position */
    Dolphin::uorb_msg<vehicle_local_position_setpoint_s>		_local_pos_sp {};		    /**< vehicle local position setpoint */

    perf_counter_t	_loop_perf;			/**< loop performance counter */

    DEFINE_PARAMETERS(
    (ParamFloat<px4::params::DPC_THR_CRUISE>) _thrust_cruise,
    (ParamFloat<px4::params::DPC_THR_MAX>) _thrust_max,
    (ParamFloat<px4::params::DPC_THR_MIN>) _thrust_min,
    (ParamFloat<px4::params::DPC_THR_IDLE>) _thrust_idle,
    (ParamInt<px4::params::DPC_SP_CTRL_MODE>) _speed_ctrl_mode,
    (ParamFloat<px4::params::DPC_MAX_TILT>) _max_tilt_angle,
    (ParamFloat<px4::params::DPC_MAX_ROLL>) _max_roll_angle
    );
};