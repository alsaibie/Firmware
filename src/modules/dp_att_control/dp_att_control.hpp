/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @author Ali AlSaibie <alsaibie@gatech.edu>
 *
 */

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f
#define MANUAL_THROTTLE_MAX_DOLPHIN	0.9f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

#define MAX_GYRO_COUNT 3

class DolphinAttitudeControl
{
public:
		DolphinAttitudeControl();
	~DolphinAttitudeControl();

	int start();
	bool task_running() { return _task_running; }

private:
    bool	_task_should_exit{false};		/**< if true, task_main() should exit */
    bool	_task_running{false};			/**< if true, task is running in its mainloop */
    int		_control_task;			/**< task handle */

    int		_ctrl_state_sub;		/**< control state subscription */
    int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
    int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
    int		_v_control_mode_sub;	/**< vehicle control mode subscription */
    int		_params_sub;			/**< parameter updates subscription */
    int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
    int		_armed_sub;				/**< arming status subscription */
    int		_vehicle_status_sub;	/**< vehicle status subscription */
    int 	_motor_limits_sub;		/**< motor limits subscription */
    int 	_battery_status_sub;	/**< battery status subscription */
    int	_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */
    int	_sensor_correction_sub;	/**< sensor thermal correction subscription */

    unsigned _gyro_count;
    int _selected_gyro;

    orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
    orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
    orb_advert_t	_controller_status_pub;	/**< controller status publication */

    orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */
    orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

    bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

    struct control_state_s				_ctrl_state;		/**< control state */
    struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
    struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
    struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
    struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
    struct actuator_controls_s			_actuators;			/**< actuator controls */
    struct actuator_armed_s				_armed;				/**< actuator arming status */
    struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
    struct multirotor_motor_limits_s	_motor_limits;		/**< motor limits */
    struct mc_att_ctrl_status_s 		_controller_status; /**< controller status */
    struct battery_status_s				_battery_status;	/**< battery status */
    struct sensor_gyro_s			_sensor_gyro;		/**< gyro data before thermal correctons and ekf bias estimates are applied */
    struct sensor_correction_s		_sensor_correction;		/**< sensor thermal corrections */

    union {
        struct {
            uint16_t motor_pos	: 1; // 0 - true when any motor has saturated in the positive direction
            uint16_t motor_neg	: 1; // 1 - true when any motor has saturated in the negative direction
            uint16_t roll_pos	: 1; // 2 - true when a positive roll demand change will increase saturation
            uint16_t roll_neg	: 1; // 3 - true when a negative roll demand change will increase saturation
            uint16_t pitch_pos	: 1; // 4 - true when a positive pitch demand change will increase saturation
            uint16_t pitch_neg	: 1; // 5 - true when a negative pitch demand change will increase saturation
            uint16_t yaw_pos	: 1; // 6 - true when a positive yaw demand change will increase saturation
            uint16_t yaw_neg	: 1; // 7 - true when a negative yaw demand change will increase saturation
            uint16_t thrust_pos	: 1; // 8 - true when a positive thrust demand change will increase saturation
            uint16_t thrust_neg	: 1; // 9 - true when a negative thrust demand change will increase saturation
        } flags;
        uint16_t value;
    } _saturation_status;

    perf_counter_t	_loop_perf;			/**< loop performance counter */
    perf_counter_t	_controller_latency_perf;

    math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
    math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
    math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
    math::Vector<3>		_rates_int;		/**< angular rates integral error */
    float				_thrust_sp;		/**< thrust setpoint */
    math::Vector<3>		_att_control;	/**< attitude control vector */
    math::Vector<4>		_mixed_att_control;	/**< Mixed attitude control vector */

    math::Matrix<3, 3>  _I;				/**< identity matrix */

    math::Matrix<3, 3>	_board_rotation = {};	/**< rotation matrix for the orientation that the board is mounted */

    struct {
        param_t roll_p;
        param_t roll_rate_p;
        param_t roll_rate_i;
        param_t roll_rate_integ_lim;
        param_t roll_rate_d;
        param_t roll_rate_ff;
        param_t pitch_p;
        param_t pitch_rate_p;
        param_t pitch_rate_i;
        param_t pitch_rate_integ_lim;
        param_t pitch_rate_d;
        param_t pitch_rate_ff;
        param_t tpa_breakpoint_p;
        param_t tpa_breakpoint_i;
        param_t tpa_breakpoint_d;
        param_t tpa_rate_p;
        param_t tpa_rate_i;
        param_t tpa_rate_d;
        param_t yaw_p;
        param_t yaw_rate_p;
        param_t yaw_rate_i;
        param_t yaw_rate_integ_lim;
        param_t yaw_rate_d;
        param_t yaw_rate_ff;
        param_t yaw_ff;
        param_t roll_rate_max;
        param_t pitch_rate_max;
        param_t yaw_rate_max;
        param_t yaw_auto_max;

        param_t acro_roll_max;
        param_t acro_pitch_max;
        param_t acro_yaw_max;
        param_t rattitude_thres;

        param_t roll_tc;
        param_t pitch_tc;

        param_t motion_type;
        param_t thrust_factor;
        param_t idle_speed;
        param_t bat_scale_en;

        param_t board_rotation;

        param_t board_offset[3];


    }		_params_handles;		/**< handles for interesting parameters */

    struct {
        math::Vector<3> att_p;					/**< P gain for angular error */
        math::Vector<3> rate_p;				/**< P gain for angular rate error */
        math::Vector<3> rate_i;				/**< I gain for angular rate error */
        math::Vector<3> rate_int_lim;			/**< integrator state limit for rate loop */
        math::Vector<3> rate_d;				/**< D gain for angular rate error */
        math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */
        float yaw_ff;						/**< yaw control feed-forward */

        float tpa_breakpoint_p;				/**< Throttle PID Attenuation breakpoint */
        float tpa_breakpoint_i;				/**< Throttle PID Attenuation breakpoint */
        float tpa_breakpoint_d;				/**< Throttle PID Attenuation breakpoint */
        float tpa_rate_p;					/**< Throttle PID Attenuation slope */
        float tpa_rate_i;					/**< Throttle PID Attenuation slope */
        float tpa_rate_d;					/**< Throttle PID Attenuation slope */

        float roll_rate_max;
        float pitch_rate_max;
        float yaw_rate_max;
        float yaw_auto_max;
        math::Vector<3> dp_rate_max;		/**< attitude rate limits in stabilized modes */
        math::Vector<3> auto_rate_max;		/**< attitude rate limits in auto modes */
        math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */
        float rattitude_thres;

        int motion_type;
        float thrust_factor;
        float idle_speed;

        int bat_scale_en;

        int board_rotation;

        float board_offset[3];

    }		_params;

    /**
     * Rotor Mixing scales
     */
    static const int _rotor_count{4};
    static const int _mix_length{5};
    struct  rotors_vector{
        float	roll_scale;	/**< scales roll for this rotor */
        float	pitch_scale;	/**< scales pitch for this rotor */
        float	yaw_scale;	/**< scales yaw for this rotor */
        float thrust_scale; /**< scales Thrust for this rotor */
        float	out_scale;	/**< scales total out for this rotor */
    } _rotors[_rotor_count];

    /** Mixing Table. Order: Rollscale, PitchScale, YawScale, ThrustScale, OutScale */
//    static constexpr float _dolphin_x_table[_rotor_count][_mix_length] = {
//            { -1.000000,  0.707107,  -0.707107, 1.000000, 1.000000 },
//            { -1.000000,  -0.707107,  0.707107, 1.000000, 1.000000 },
//            { 1.000000, 0.707107,   0.707107, 1.000000, 1.000000 },
//            { 1.000000, -0.707107, -0.707107, 1.000000, 1.000000 },
//    };
    static constexpr float _dolphin_x_table[_rotor_count][_mix_length] = {
            { -1.000000,  0.5,  -0.5, 1.000000, 1.000000 },
            { -1.000000,  -0.5,  0.5, 1.000000, 1.000000 },
            { 1.000000, 0.5,   0.5, 1.000000, 1.000000 },
            { 1.000000, -0.5, -0.5, 1.000000, 1.000000 },
    };



    /**
     * Update our local parameter cache.
     */
    int			parameters_update();

    /**
     * Check for parameter update and handle it.
     */
    void		parameter_update_poll();

    /**
     * Check for changes in vehicle control mode.
     */
    void		vehicle_control_mode_poll();

    /**
     * Check for changes in manual inputs.
     */
    void		vehicle_manual_poll();

    /**
     * Check for attitude setpoint updates.
     */
    void		vehicle_attitude_setpoint_poll();

    /**
     * Check for rates setpoint updates.
     */
    void		vehicle_rates_setpoint_poll();

    /**
     * Check for arming status updates.
     */
    void		arming_status_poll();

    /**
     * Attitude controller.
     */
    void		control_attitude(float dt);

    /**
     * Attitude rates controller.
     */
    void		control_attitude_rates(float dt);

    /**
     * Mix Control Output.
     */
//    void		mix_control_output(void);
public:
    bool    mix_control_output(math::Vector<3> &att_control, float thrust, math::Vector<4> & mixed_att_control);
private:
    /**
     * Throttle PID attenuation.
     */
    math::Vector<3> pid_attenuations(float tpa_breakpoint, float tpa_rate);

    /**
     * Check for vehicle status updates.
     */
    void		vehicle_status_poll();

    /**
     * Check for vehicle motor limits status.
     */
    void		vehicle_motor_limits_poll();

    /**
     * Check for battery status updates.
     */
    void		battery_status_poll();

    /**
     * Check for control state updates.
     */
    void		control_state_poll();

    /**
     * Check for sensor thermal correction updates.
     */
    void		sensor_correction_poll();

    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * Main attitude control task.
     */
    void		task_main();

};
