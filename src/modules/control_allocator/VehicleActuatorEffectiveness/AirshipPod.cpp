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

#include "AirshipPod.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

void AirshipPod::setTiltRange(const float tilt_min, const float tilt_max)
{
	_tilt_min = tilt_min;
	_tilt_max = fmaxf(tilt_max, tilt_min);
	_tilt.setForcedValue(math::constrain(_tilt.getState(), _tilt_min, _tilt_max));
	_tilt_target = math::constrain(_tilt_target, _tilt_min, _tilt_max);
}

void AirshipPod::steer(const Vector2f &demand, const float dt)
{
	const float magnitude = demand.norm();
	const bool engaged = _mode == TiltMode::Steering || _mode == TiltMode::Holding;

	if (magnitude > kSteerEngage) {
		_mode = TiltMode::Steering;
		_tilt_target = steerTarget(demand, magnitude);

	} else if (engaged && magnitude > kSteerRelease) {
		// Inside the band an engaged target stands
		_mode = TiltMode::Holding;

	} else {
		// At or below release, a NaN demand, or inside the band with nothing
		// engaged: the tilt stays where it is
		_mode = TiltMode::Released;
		_tilt_target = _tilt.getState();
	}

	slewToTarget(dt);
}

void AirshipPod::park(const float dt)
{
	_mode = TiltMode::Parked;
	_tilt_target = math::constrain(0.f, _tilt_min, _tilt_max);
	slewToTarget(dt);
}

void AirshipPod::slewToTarget(const float dt)
{
	if (_slew_rate > 0.f) {
		_tilt.setSlewRate(_slew_rate);
		_tilt.update(_tilt_target, dt);

	} else {
		_tilt.setForcedValue(_tilt_target);
	}
}

float AirshipPod::steerTarget(const Vector2f &demand, const float magnitude) const
{
	// Both branches below bring the demand direction into the range, picking
	// an end when it lies outside, and neither moves for a gain below the
	// margin, scored by what the candidate realizes. They stay apart because
	// the rear cone is a deadband as well as a branch cut: straight back,
	// atan2 flips between +-180 deg on noise in the perpendicular component,
	// so the cone ranks the ends by the range alone where tracking the
	// demand would dither
	float tilt = atan2f(demand(1), demand(0));

	// Switching ends costs a full sweep of an end-stop servo: ignore smaller
	// advantages; the floor covers stick noise at low demand
	const float switch_margin = fmaxf(kEndSwitchMargin * magnitude, kSteerRelease);

	if (demand(0) < 0.f && fabsf(demand(1)) < switch_margin) {
		// Straight back: the perpendicular component is noise, so rank the
		// ends by how backward each points, a function of the range and the
		// committed end alone, which cannot dither. On a tie the ends point
		// equally backward, so keep the committed one
		const float rear_hi = math::constrain(M_PI_F, _tilt_min, _tilt_max);
		const float rear_lo = math::constrain(-M_PI_F, _tilt_min, _tilt_max);
		const float reverse_hi = -cosf(rear_hi);
		const float reverse_lo = -cosf(rear_lo);

		if (fabsf(reverse_hi - reverse_lo) > FLT_EPSILON) {
			tilt = reverse_hi > reverse_lo ? rear_hi : rear_lo;

		} else {
			tilt = _tilt_target >= 0.f ? rear_hi : rear_lo;
		}

		// An end barely past vertical realizes little, and the perpendicular
		// can cancel even that: a pick that realizes less than the margin of
		// the demand is not worth any move. An end that points forward
		// realizes nothing of a rear demand, so this also holds a range
		// with no backward end at all. With the seam switch below this
		// prices every cone pick: one that clears the margin from a target
		// on the other side of the range gains it all, because that target
		// realizes nothing
		if (realized(demand, tilt) <= switch_margin) {
			tilt = _tilt_target;
		}

	} else if (tilt < _tilt_min || tilt > _tilt_max) {
		// The tilt is circular but the range is a segment: the numerically
		// nearer bound can point away from the demand. Take the end that
		// realizes more, floored at zero (no reverse), and switch ends only
		// past the margin
		const float p_hi = fmaxf(0.f, realized(demand, _tilt_max));
		const float p_lo = fmaxf(0.f, realized(demand, _tilt_min));

		if (fmaxf(p_hi, p_lo) <= FLT_EPSILON) {
			// Neither end realizes any of the demand, and the projection peaks
			// once on the circle, so no angle between them does either: a sweep
			// would realize nothing, so the tilt stays
			tilt = _tilt_target;

		} else {
			const bool committed_hi = _tilt_target - _tilt_min > _tilt_max - _tilt_target;

			if (committed_hi) {
				tilt = p_lo > p_hi + switch_margin ? _tilt_min : _tilt_max;

			} else {
				tilt = p_hi > p_lo + switch_margin ? _tilt_max : _tilt_min;
			}
		}
	}

	// A target on the far side of the seam costs more than half the servo's
	// travel to reach, because an end-stop mount cannot wrap; on a range no
	// wider than a half turn no target is ever that far. Pay that only for a
	// gain past the margin, as the end switch above: score both by what they
	// realize, and floor the committed target, which gives up nothing when
	// it points away. On the in-range branch the target is the demand
	// direction itself
	if (fabsf(tilt - _tilt_target) > M_PI_F) {
		if (realized(demand, tilt) <= fmaxf(0.f, realized(demand, _tilt_target)) + switch_margin) {
			tilt = _tilt_target;
		}
	}

	return math::constrain(tilt, _tilt_min, _tilt_max);
}

float AirshipPod::servoSetpoint() const
{
	return -1.f + 2.f * (_tilt.getState() - _tilt_min) / (_tilt_max - _tilt_min);
}

void AirshipPod::setServoSetpoint(const float servo_sp)
{
	_tilt.setForcedValue(_tilt_min + (servo_sp + 1.f) * 0.5f * (_tilt_max - _tilt_min));
}

AirshipPod::Drive AirshipPod::drive(const Vector2f &demand, const float out_min, const float out_max) const
{
	const Vector2f axis = thrustAxis();
	Drive drive{};
	// non-reversible propeller: reverse only by tilting, the CA_R_REV pod bits are not honored
	drive.thrust = math::constrain(demand.dot(axis), math::max(out_min, 0.f), out_max);
	drive.achieved = axis * drive.thrust;

	if (heldByChoice()) {
		drive.withheld = demand - drive.achieved;
	}

	return drive;
}

bool AirshipPod::heldByChoice() const
{
	// A fixed mount withholds nothing by choice
	return canTilt() && (_mode == TiltMode::Holding || _mode == TiltMode::Released)
	       && fabsf(_tilt.getState() - _tilt_target) < kTiltSettledTolerance;
}
