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

using namespace matrix;

void AirshipPod::setTiltRange(const float tilt_min, const float tilt_max)
{
	_tilt_min = tilt_min;
	_tilt_max = fmaxf(tilt_max, tilt_min);
	_tilt.setForcedValue(math::constrain(_tilt.getState(), _tilt_min, _tilt_max));
	_tilt_sp = math::constrain(_tilt_sp, _tilt_min, _tilt_max);
}

void AirshipPod::steer(const Vector2f &force, const float dt)
{
	const float magnitude = sqrtf(force(0) * force(0) + force(1) * force(1));

	if (magnitude > kSteerEngage) {
		_mode = TiltMode::Steering;
		_tilt_sp = steerTarget(force, magnitude);

	} else if (!((_mode == TiltMode::Steering || _mode == TiltMode::Holding) && magnitude > kSteerRelease)) {
		// Below release (or a NaN demand): the tilt stays where it is
		_mode = TiltMode::Released;
		_tilt_sp = _tilt.getState();

	} else {
		// Inside the band the setpoint stands
		_mode = TiltMode::Holding;
	}

	slewToSetpoint(dt);
}

void AirshipPod::park(const float dt)
{
	_mode = TiltMode::Parked;
	_tilt_sp = math::constrain(0.f, _tilt_min, _tilt_max);
	slewToSetpoint(dt);
}

void AirshipPod::slewToSetpoint(const float dt)
{
	if (_slew_rate > 0.f) {
		_tilt.setSlewRate(_slew_rate);
		_tilt.update(_tilt_sp, dt);

	} else {
		_tilt.setForcedValue(_tilt_sp);
	}
}

float AirshipPod::steerTarget(const Vector2f &force, const float magnitude) const
{
	float tilt = atan2f(force(1), force(0));

	// Switching ends costs a full sweep of an end-stop servo: ignore smaller
	// advantages; the floor covers stick noise at low demand
	const float switch_margin = fmaxf(kEndSwitchMargin * magnitude, kSteerRelease);

	if (force(0) < 0.f && fabsf(force(1)) < switch_margin) {
		// Straight back, atan2 flips between +-180 deg on the sign of the
		// perpendicular component: pick the range end that realizes the
		// demand best, on a tie keep the committed end
		const float rear_hi = math::constrain(M_PI_F, _tilt_min, _tilt_max);
		const float rear_lo = math::constrain(-M_PI_F, _tilt_min, _tilt_max);
		const float reverse_hi = -cosf(rear_hi);
		const float reverse_lo = -cosf(rear_lo);

		if (fmaxf(reverse_hi, reverse_lo) <= FLT_EPSILON) {
			// No end points backward (e.g. -90..90): a sweep would realize
			// nothing, so the tilt stays
			tilt = _tilt_sp;

		} else if (fabsf(reverse_hi - reverse_lo) > FLT_EPSILON) {
			tilt = reverse_hi > reverse_lo ? rear_hi : rear_lo;

		} else {
			tilt = _tilt_sp >= 0.f ? rear_hi : rear_lo;
		}

	} else if (tilt < _tilt_min || tilt > _tilt_max) {
		// The tilt is circular but the range is a segment: the numerically
		// nearer bound can point away from the demand. Take the end that
		// realizes more, floored at zero (no reverse), and switch ends only
		// past the margin
		const float p_hi = fmaxf(0.f, force(0) * cosf(_tilt_max) + force(1) * sinf(_tilt_max));
		const float p_lo = fmaxf(0.f, force(0) * cosf(_tilt_min) + force(1) * sinf(_tilt_min));
		const bool committed_hi = _tilt_sp - _tilt_min > _tilt_max - _tilt_sp;

		if (committed_hi) {
			tilt = p_lo > p_hi + switch_margin ? _tilt_min : _tilt_max;

		} else {
			tilt = p_hi > p_lo + switch_margin ? _tilt_max : _tilt_min;
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
	setTilt(_tilt_min + (servo_sp + 1.f) * 0.5f * (_tilt_max - _tilt_min));
}

bool AirshipPod::heldInBand() const
{
	return (_mode == TiltMode::Holding || _mode == TiltMode::Released)
	       && fabsf(_tilt.getState() - _tilt_sp) < kTiltSettledTolerance;
}
