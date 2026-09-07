/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file airship_yaw_rate.hpp
 *
 * Stick to yaw rate setpoint mapping and the mode gate of the yaw rate loop,
 * kept free of uORB I/O so they can be unit tested.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/vehicle_control_mode.h>

namespace airship_yaw_rate
{

/**
 * Map the yaw stick to a yaw rate setpoint [rad/s].
 *
 * Deadzone first (MAN_DEADZONE, as mc_att_control and lib/sticks apply it),
 * then linear scaling to max_rate; a non-finite stick reads as released.
 */
inline float setpointFromStick(float stick, float deadzone, float max_rate)
{
	if (!PX4_ISFINITE(stick)) {
		return 0.f;
	}

	return math::deadzone(stick, deadzone) * max_rate;
}

/**
 * Whether the yaw rate loop closes on the stick: manual modes with rate
 * control (Acro, Stabilized, Altitude, Position). Manual has rates off and
 * modes without pilot input have no setpoint source here yet; both keep the
 * torque passthrough.
 */
inline bool loopActive(const vehicle_control_mode_s &control_mode, bool manual_input_usable)
{
	return manual_input_usable && control_mode.flag_control_manual_enabled
	       && control_mode.flag_control_rates_enabled;
}

} // namespace airship_yaw_rate
