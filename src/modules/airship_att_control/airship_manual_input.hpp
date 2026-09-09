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
 * @file airship_manual_input.hpp
 *
 * Stick to thrust and torque setpoint mapping of the manual passthrough,
 * kept free of uORB I/O so it can be unit tested.
 */

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/manual_control_setpoint.h>

namespace airship_manual_input
{

/** NaN marks a manual channel without valid data: read it as released */
inline float finiteOr(float value, float fallback)
{
	return PX4_ISFINITE(value) ? value : fallback;
}

/**
 * Thrust setpoint per body axis: throttle forward, the pitch stick vertical.
 *
 * Stick forward descends (+pitch = stick forward, +z = down in FRD). The
 * pitch stick is also pitch torque in torque(): tilting pods serve this
 * thrust, elevators serve that torque; the channel an airframe cannot serve
 * is reported unallocated.
 */
inline matrix::Vector3f thrust(const manual_control_setpoint_s &sticks)
{
	return matrix::Vector3f{(finiteOr(sticks.throttle, -1.f) + 1.f) * 0.5f, 0.f, finiteOr(sticks.pitch, 0.f)};
}

/**
 * Torque setpoint per body axis: the sticks straight through.
 *
 * Stick forward is nose down: negative pitch rotation in FRD.
 */
inline matrix::Vector3f torque(const manual_control_setpoint_s &sticks)
{
	return matrix::Vector3f{finiteOr(sticks.roll, 0.f), -finiteOr(sticks.pitch, 0.f), finiteOr(sticks.yaw, 0.f)};
}

} // namespace airship_manual_input
