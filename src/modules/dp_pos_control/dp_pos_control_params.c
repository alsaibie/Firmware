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
 * @file dp_pos_control_params.c
 *
 * @author Ali AlSaibie <ali@alsaibie.com>
 */

/*
 * Controller parameters, accessible via MAVLink
 */

/**
 * Cruise throttle
 *
 * This is the throttle setting required to achieve the desired cruise speed.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Dolphin POS Control
 */
PARAM_DEFINE_FLOAT(DPC_THR_CRUISE, 0.1f);

/**
 * Throttle limit max
 *
 * This is the maximum throttle % that can be used by the controller.

 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Dolphin POS Control
 */
PARAM_DEFINE_FLOAT(DPC_THR_MAX, 0.9f);

/**
 * Throttle limit min
 *
 * This is the minimum throttle % that can be used by the controller.
 * Set to 0
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Dolphin POS Control
 */
PARAM_DEFINE_FLOAT(DPC_THR_MIN, 0.0f);

/**
 * Idle throttle
 *
 * This is the minimum throttle while on the ground, it should be 0
 *
 *
 * @unit norm
 * @min 0.0
 * @max 0.4
 * @decimal 2
 * @increment 0.01
 * @group DPC POS Control
 */
PARAM_DEFINE_FLOAT(DPC_THR_IDLE, 0.0f);

/**
 * Control mode for speed
 * TODO: Adapt this to my use
 *
 * This allows the user to choose between closed loop gps speed or open loop cruise throttle speed
 * @min 0
 * @max 1
 * @value 0 open loop control
 * @value 1 close the loop with gps speed
 * @group Dolphin Attitude Control
 */
PARAM_DEFINE_INT32(DPC_SP_CTRL_MODE, 0);

