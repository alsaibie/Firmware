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
 *
 * This module is a modification of the mc att module and fw wing app and it is designed for underwater robots.
 *
 * All the acknowledgments and credits for the mc and fw wing app are reported in those files.
 *
 * @author Ali AlSaibie <ali@alsaibie.com>
 *
 */

#include <lib/mixer/mixer.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include "AttitudeController.hpp"
/**
 * Dolphin attitude control app start / stop handling function
 */
extern "C" __EXPORT int dp_att_control_main(int argc, char *argv[]);

#define MAX_GYRO_COUNT 3
namespace Dolphin {
    template <typename _struct_T>
    struct uorb_msg{
        orb_id_t        msg_id {nullptr};
        orb_advert_t	msg_pub {nullptr};
        int             msg_sub {-1};
        _struct_T msg {};
    };
}

class DolphinAttitudeControl : public ModuleBase<DolphinAttitudeControl>, public ModuleParams
{
public:
	DolphinAttitudeControl();

	virtual ~DolphinAttitudeControl() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static DolphinAttitudeControl *instantiate(int argc, char *argv[]);

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
    AController::AttitudeController *controller;
    //TODO: cleanup comments
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

	int		_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */

	unsigned _gyro_count{1};
	int _selected_gyro{0};


	bool		_actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */

    Dolphin::uorb_msg<parameter_update_s>           _params {};
    Dolphin::uorb_msg<multirotor_motor_limits_s>    _motor_limits {};
    Dolphin::uorb_msg<vehicle_attitude_s>           _v_att {};		        /**< vehicle attitude */
    Dolphin::uorb_msg<vehicle_attitude_setpoint_s>  _v_att_sp {};		    /**< vehicle attitude setpoint */
    Dolphin::uorb_msg<vehicle_rates_setpoint_s>     _v_rates_sp {};		    /**< vehicle rates setpoint */
    Dolphin::uorb_msg<manual_control_setpoint_s>    _manual_control_sp {};	/**< manual control setpoint */
    Dolphin::uorb_msg<vehicle_control_mode_s>       _v_control_mode {};	    /**< vehicle control mode */
    Dolphin::uorb_msg<actuator_controls_s>          _actuators {};		    /**< actuator controls */
    Dolphin::uorb_msg<vehicle_status_s>             _vehicle_status {};	    /**< vehicle status */
    Dolphin::uorb_msg<battery_status_s>             _battery_status {};	    /**< battery status */
    Dolphin::uorb_msg<sensor_gyro_s>                _sensor_gyro {};	    /**< gyro data before thermal correctons and ekf bias estimates are applied */
    Dolphin::uorb_msg<sensor_correction_s>          _sensor_correction {};	/**< sensor thermal corrections */
    Dolphin::uorb_msg<sensor_bias_s>                _sensor_bias {};	    /**< sensor in-run bias corrections */
    Dolphin::uorb_msg<rate_ctrl_status_s>           _rate_ctrl_status {};

	MultirotorMixer::saturation_status _saturation_status{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */
    math::LowPassFilter2p _shared_lp_filters_d[3];                      /**< low-pass filters for D-term (roll, pitch & yaw) */
	static constexpr const float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
	float _loop_update_rate_hz{initial_update_rate_hz};          /**< current rate-controller loop update rate in [Hz] */

	matrix::Dcmf _board_rotation;			    /**< rotation matrix for the orientation that the board is mounted */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DP_ROLL_P>) _roll_p,
		(ParamFloat<px4::params::DP_ROLLRATE_P>) _roll_rate_p,
		(ParamFloat<px4::params::DP_ROLLRATE_I>) _roll_rate_i,
		(ParamFloat<px4::params::DP_RR_INT_LIM>) _roll_rate_integ_lim,
		(ParamFloat<px4::params::DP_ROLLRATE_D>) _roll_rate_d,
		(ParamFloat<px4::params::DP_ROLLRATE_FF>) _roll_rate_ff,
        (ParamFloat<px4::params::DP_ROLL_FF>) _roll_ff,					    /**< roll control feed-forward */

		(ParamFloat<px4::params::DP_PITCH_P>) _pitch_p,
		(ParamFloat<px4::params::DP_PITCHRATE_P>) _pitch_rate_p,
		(ParamFloat<px4::params::DP_PITCHRATE_I>) _pitch_rate_i,
		(ParamFloat<px4::params::DP_PR_INT_LIM>) _pitch_rate_integ_lim,
		(ParamFloat<px4::params::DP_PITCHRATE_D>) _pitch_rate_d,
		(ParamFloat<px4::params::DP_PITCHRATE_FF>) _pitch_rate_ff,

        (ParamFloat<px4::params::DP_YAW_P>) _yaw_p,
		(ParamFloat<px4::params::DP_YAWRATE_P>) _yaw_rate_p,
		(ParamFloat<px4::params::DP_YAWRATE_I>) _yaw_rate_i,
		(ParamFloat<px4::params::DP_YR_INT_LIM>) _yaw_rate_integ_lim,
		(ParamFloat<px4::params::DP_YAWRATE_D>) _yaw_rate_d,
		(ParamFloat<px4::params::DP_YAWRATE_FF>) _yaw_rate_ff,

        (ParamFloat<px4::params::DP_DTERM_CUTOFF>) _d_term_cutoff_freq,		/**< Cutoff frequency for the D-term filter */

		(ParamFloat<px4::params::DP_TPA_BREAK_P>) _tpa_breakpoint_p,		/**< Throttle PID Attenuation breakpoint */
		(ParamFloat<px4::params::DP_TPA_BREAK_I>) _tpa_breakpoint_i,		/**< Throttle PID Attenuation breakpoint */
		(ParamFloat<px4::params::DP_TPA_BREAK_D>) _tpa_breakpoint_d,		/**< Throttle PID Attenuation breakpoint */
		(ParamFloat<px4::params::DP_TPA_RATE_P>) _tpa_rate_p,				/**< Throttle PID Attenuation slope */
		(ParamFloat<px4::params::DP_TPA_RATE_I>) _tpa_rate_i,				/**< Throttle PID Attenuation slope */
		(ParamFloat<px4::params::DP_TPA_RATE_D>) _tpa_rate_d,				/**< Throttle PID Attenuation slope */

		(ParamFloat<px4::params::DP_ROLLRATE_MAX>) _roll_rate_max,
		(ParamFloat<px4::params::DP_PITCHRATE_MAX>) _pitch_rate_max,
		(ParamFloat<px4::params::DP_YAWRATE_MAX>) _yaw_rate_max,
		(ParamFloat<px4::params::DP_ROLLRAUTO_MAX>) _roll_auto_max,

		(ParamFloat<px4::params::DP_ACRO_R_MAX>) _acro_roll_max,
		(ParamFloat<px4::params::DP_ACRO_P_MAX>) _acro_pitch_max,
		(ParamFloat<px4::params::DP_ACRO_Y_MAX>) _acro_yaw_max,
		(ParamFloat<px4::params::DP_ACRO_EXPO>) _acro_expo_py,				/**< expo stick curve shape (pitch & yaw) */
		(ParamFloat<px4::params::DP_ACRO_EXPO_R>) _acro_expo_r,				/**< expo stick curve shape (roll) */
		(ParamFloat<px4::params::DP_ACRO_SUPEXPO>) _acro_superexpo_py,			/**< superexpo stick curve shape (pitch & yaw) */
		(ParamFloat<px4::params::DP_ACRO_SUPEXPOR>) _acro_superexpo_r,			/**< superexpo stick curve shape (roll) */

		(ParamFloat<px4::params::DP_RATT_TH>) _rattitude_thres,

         /* Constants */
        (ParamFloat<px4::params::DP_V_HZ_N_C0>) _c0_Vn_norm,
        (ParamFloat<px4::params::DP_V_HZ_N_C1>) _c1_Vn_norm,
        (ParamFloat<px4::params::DP_V_HZ_N_C2>) _c2_Vn_norm,
        (ParamFloat<px4::params::DP_V_T_C0>) _c0_VT,
        (ParamFloat<px4::params::DP_V_T_C1>) _c1_VT,
        (ParamFloat<px4::params::DP_V_T_C2>) _c2_VT,
        (ParamFloat<px4::params::DP_V_TORQ_C2>) _c2_VTORQ,
        (ParamFloat<px4::params::DP_ROTOR_R>) _rotor_radius,

		(ParamBool<px4::params::DP_BAT_SCALE_EN>) _bat_scale_en,

		(ParamInt<px4::params::SENS_BOARD_ROT>) _board_rotation_param,

		(ParamFloat<px4::params::SENS_BOARD_X_OFF>) _board_offset_x,
		(ParamFloat<px4::params::SENS_BOARD_Y_OFF>) _board_offset_y,
		(ParamFloat<px4::params::SENS_BOARD_Z_OFF>) _board_offset_z

	)
};

