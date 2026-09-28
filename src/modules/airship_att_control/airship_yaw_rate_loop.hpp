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
 * @file airship_yaw_rate_loop.hpp
 *
 * The airship's yaw rate loop: a PI loop with setpoint feedforward on the yaw
 * axis of RateControl, its anti-windup from the allocator and its reset when
 * the loop opens, kept free of uORB I/O so it can be unit tested.
 */

#pragma once

#include <float.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/rate_control/rate_control.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/rate_ctrl_status.h>

class AirshipYawRateLoop
{
public:
	/** Per-axis saturation flags for RateControl::setSaturationStatus() */
	struct SaturationFlags {
		matrix::Vector<bool, 3> positive;
		matrix::Vector<bool, 3> negative;
	};

	/**
	 * Anti-windup from the allocator, decoded as in mc_rate_control: an axis
	 * the allocator could not serve stops integrating in that direction.
	 */
	static SaturationFlags saturationFlags(const control_allocator_status_s &control_allocator_status)
	{
		SaturationFlags flags{};

		if (!control_allocator_status.torque_setpoint_achieved) {
			for (size_t i = 0; i < 3; i++) {
				if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
					flags.positive(i) = true;

				} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
					flags.negative(i) = true;
				}
			}
		}

		return flags;
	}

	/**
	 * Yaw rate gains, integrator limit and feedforward (AS_YAWRATE_P,
	 * AS_YAWRATE_I, AS_YR_INT_LIM, AS_YAWRATE_FF)
	 */
	void setGains(float p, float i, float integrator_limit, float feedforward)
	{
		// Yaw axis only: roll and pitch stay stick passthrough, their gains are zero
		_rate_control.setPidGains(matrix::Vector3f(0.f, 0.f, p), matrix::Vector3f(0.f, 0.f, i), matrix::Vector3f());
		// The library integrator limit defaults to zero, which would silently
		// disable the I term
		_rate_control.setIntegratorLimit(matrix::Vector3f(0.f, 0.f, integrator_limit));
		// The torque that holds a turn against the hull's yaw damping, taken
		// from the setpoint so that the integrator does not have to build it
		// up during the turn and unwind it after
		_rate_control.setFeedForwardGain(matrix::Vector3f(0.f, 0.f, feedforward));
	}

	/** Anti-windup feedback from the control allocator */
	void setSaturation(const control_allocator_status_s &control_allocator_status)
	{
		const SaturationFlags flags = saturationFlags(control_allocator_status);
		_rate_control.setSaturationStatus(flags.positive, flags.negative);
	}

	/**
	 * Run one cycle with the loop closed on the stick.
	 * @return normalized yaw torque, zero if the loop produced a non-finite one
	 */
	float update(const matrix::Vector3f &rates, float yaw_rate_sp, float dt)
	{
		_closed = true;

		// No D term, so no angular acceleration (0 * NaN would poison the torque).
		// landed = false: AirshipLandDetector reports landed only when disarmed or
		// in AUTO_LAND, and neither passes yawRateLoopActive; windup while armed
		// on the ground is not handled yet.
		const matrix::Vector3f torque = _rate_control.update(rates, matrix::Vector3f(0.f, 0.f, yaw_rate_sp),
						matrix::Vector3f{}, dt, false);

		return PX4_ISFINITE(torque(2)) ? torque(2) : 0.f;
	}

	/**
	 * Keep the loop open for this cycle. On the first open cycle after a
	 * closed one the integrator is cleared: it only moves while the loop is
	 * closed, so it would otherwise carry into the next closed-loop entry.
	 * @return true on that first open cycle only
	 */
	bool open()
	{
		if (!_closed) {
			return false;
		}

		_closed = false;
		_rate_control.resetIntegral();
		return true;
	}

	/** Integrator state for logging */
	void getStatus(rate_ctrl_status_s &rate_ctrl_status) { _rate_control.getRateControlStatus(rate_ctrl_status); }

private:
	RateControl _rate_control; ///< yaw axis only: roll and pitch gains stay zero
	bool _closed{false}; ///< the loop was closed on the previous cycle
};
