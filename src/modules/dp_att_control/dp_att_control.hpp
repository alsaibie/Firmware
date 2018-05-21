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
     AttitudeController *controller;

     Controller::States _states {};
     Controller::Setpoints::Attitude _att_sp {};
     Controller::Setpoints::Rates _rates_sp {};
     Controller::ControlMode _control_mode {};

	/**
	 * initialize some vectors/matrices from parameters
	 */
    void            parameter_subscribe_unsubscribe(bool subscribe);

	/**
	 * Check for msgs updates and handle it. TODO: specify type, remove unnecessary template
	 */
	template <typename _msg_in_struct_T>
    bool        generic_poll(orb_id_t msg_id, int _msg_sub, _msg_in_struct_T &_msg);
	/**
	 * Publish msgs
	 */
    template <typename _msg_out_struct_T>
    void        generic_publish(orb_id_t msg_id, orb_advert_t _msg_pub, _msg_out_struct_T &_msg);

    /**
     * Update States, setpoints and parameters
     */

    void        update_states();
    void        update_attitude_setpoint();
    void        update_rates_setpoint();
    void        update_parameters();

    /**
     * Publish Control Output
     */
    void        publish_output();



	int		_v_att_sub{-1};			    /**< vehicle attitude subscription */
	int		_v_att_sp_sub{-1};		    /**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub{-1};		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub{-1};	/**< vehicle control mode subscription */
	int		_params_sub{-1};		    /**< parameter updates subscription */
	int		_manual_control_sp_sub{-1};	/**< manual control setpoint subscription */
	int		_vehicle_status_sub{-1};	/**< vehicle status subscription */
	int		_motor_limits_sub{-1};		/**< motor limits subscription */
	int		_battery_status_sub{-1};	/**< battery status subscription */
	int		_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */
	int		_sensor_correction_sub{-1};	/**< sensor thermal correction subscription */
	int		_sensor_bias_sub{-1};		/**< sensor in-run bias correction subscription */


	unsigned _gyro_count{1};
	int _selected_gyro{0};

	orb_advert_t	_v_rates_sp_pub{nullptr};		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub{nullptr};		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub{nullptr};	/**< controller status publication */

	bool		_actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */

	struct parameter_update_s           _param_update {};
    struct multirotor_motor_limits_s     _motor_limits = {};
    struct vehicle_attitude_s		    _v_att {};		        /**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp {};		    /**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp {};		    /**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp {};	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode {};	    /**< vehicle control mode */
	struct actuator_controls_s		    _actuators {};		    /**< actuator controls */
	struct vehicle_status_s			    _vehicle_status {};	    /**< vehicle status */
	struct battery_status_s			    _battery_status {};	    /**< battery status */
	struct sensor_gyro_s			    _sensor_gyro {};	    /**< gyro data before thermal correctons and ekf bias estimates are applied */
	struct sensor_correction_s		    _sensor_correction {};	/**< sensor thermal corrections */
	struct sensor_bias_s			    _sensor_bias {};	    /**< sensor in-run bias corrections */
    struct rate_ctrl_status_s           _rate_ctrl_status {};

	MultirotorMixer::saturation_status _saturation_status{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

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

		(ParamFloat<px4::params::DP_YAW_FF>) _yaw_ff,					    /**< yaw control feed-forward */

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
		(ParamFloat<px4::params::DP_YAWRAUTO_MAX>) _yaw_auto_max,

		(ParamFloat<px4::params::DP_ACRO_R_MAX>) _acro_roll_max,
		(ParamFloat<px4::params::DP_ACRO_P_MAX>) _acro_pitch_max,
		(ParamFloat<px4::params::DP_ACRO_Y_MAX>) _acro_yaw_max,
//		(ParamFloat<px4::params::DP_ACRO_EXPO>) _acro_expo_rp,				/**< expo stick curve shape (roll & pitch) */
//		(ParamFloat<px4::params::DP_ACRO_EXPO_Y>) _acro_expo_y,				/**< expo stick curve shape (yaw) */
//		(ParamFloat<px4::params::DP_ACRO_SUPEXPO>) _acro_superexpo_rp,			/**< superexpo stick curve shape (roll & pitch) */
//		(ParamFloat<px4::params::DP_ACRO_SUPEXPOY>) _acro_superexpo_y,			/**< superexpo stick curve shape (yaw) */

		(ParamFloat<px4::params::DP_RATT_TH>) _rattitude_thres,

		(ParamBool<px4::params::DP_BAT_SCALE_EN>) _bat_scale_en,

		(ParamInt<px4::params::SENS_BOARD_ROT>) _board_rotation_param,

		(ParamFloat<px4::params::SENS_BOARD_X_OFF>) _board_offset_x,
		(ParamFloat<px4::params::SENS_BOARD_Y_OFF>) _board_offset_y,
		(ParamFloat<px4::params::SENS_BOARD_Z_OFF>) _board_offset_z

	)
};

