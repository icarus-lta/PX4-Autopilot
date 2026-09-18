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
 * @file AirshipPod.hpp
 *
 * One airship propulsion pod: a non-reversible propeller on an end-stop
 * tilt servo. Tilt 0 is thrust forward, positive tilts the thrust up [rad];
 * the force demand is (forward, up) in normalized pod units. Pure math, no
 * parameters, no uORB.
 */

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/slew_rate/SlewRate.hpp>

class AirshipPod
{
public:
	// The tilt follows the demand direction only above the engage floor, holds
	// its target between release and engage, and is released where it stands
	// below release (a direction near zero magnitude is noise) [normalized]
	static constexpr float kSteerEngage = 0.02f;
	static constexpr float kSteerRelease = 0.01f;

	// Share of the demand a range end must realize beyond the other before
	// the tilt switches ends (floored at kSteerRelease); also the half-width
	// of the straight-back cone where +-180 deg are one direction [-]
	static constexpr float kEndSwitchMargin = 0.05f;

	/** Tilt range [rad]; an inverted pair is a fixed mount at tilt_min. Pulls the tilt and its target into the range */
	void setTiltRange(float tilt_min, float tilt_max);

	/** Tilt slew rate limit [rad/s]; 0 = unlimited */
	void setTiltSlewRate(float slew_rate) { _slew_limited = slew_rate > 0.f; _tilt.setSlewRate(slew_rate); }

	bool canTilt() const { return _tilt_max - _tilt_min > kMinTiltSpan; }

	/** Steer the tilt toward a force demand (forward, up) and advance it by dt [s] */
	void steer(const matrix::Vector2f &demand, float dt);

	/** Park the tilt as close to level as the range allows */
	void park(float dt);

	float tilt() const { return _tilt.getState(); }	///< realized tilt [rad]

	/** Servo output [-1, 1] of the realized tilt over the range; valid only while canTilt() */
	float servoSetpoint() const;

	/** Read a clamped servo output back into the realized tilt; valid only while canTilt() */
	void setServoSetpoint(float servo_sp);

	/** What the pod does with a force demand (forward, up) at the tilt it has actually realized */
	struct Drive {
		float thrust;			///< motor command along the realized axis, never negative
		matrix::Vector2f achieved;	///< force delivered (forward, up)
		matrix::Vector2f withheld;	///< the whole residual the pod withholds by choice, clamp shortfall included
	};

	/**
	 * Project a force demand onto the axis the tilt has actually reached.
	 *
	 * Call after the tilt servo is written, so the axis is the angle the servo
	 * realizes rather than the one that was asked for.
	 * @param out_min, out_max motor output limits; a negative minimum is not
	 *        honored, because the propeller does not reverse
	 */
	Drive drive(const matrix::Vector2f &demand, float out_min, float out_max) const;

	/**
	 * Steered below the engage floor (holding or released) and settled at its
	 * target: what the pod withholds is by choice, not missing authority. A
	 * fixed mount, a parked, steering or still-slewing tilt reports its shortfall
	 */
	bool heldByChoice() const;

private:
	/** Unit vector (forward, up) the propeller thrusts along at the realized tilt */
	matrix::Vector2f thrustAxis() const { return matrix::Vector2f{cosf(tilt()), sinf(tilt())}; }

	static constexpr float kMinTiltSpan = 1e-3f;		///< below this range the pod is a fixed mount [rad]
	static constexpr float kTiltSettledTolerance = 1e-3f;	///< the tilt counts as at its target [rad]

	// Two independent facts in one value: whether the steer band is latched
	// (read in steer()) and whether this cycle's shortfall is by choice (read
	// in heldByChoice()). All four combinations occur; none is spare
	enum class TiltMode : uint8_t {
		Parked,		///< as close to level as the range allows
		Steering,	///< demand above the engage floor: the target follows its direction
		Holding,	///< demand between release and engage while engaged: the target stands
		Released	///< demand below release, NaN, or not engaged: the target is the tilt itself
	};

	float steerTarget(const matrix::Vector2f &demand, float magnitude) const;

	/** What a tilt realizes of a force demand: the demand's projection on the thrust axis at that tilt, signed */
	static float realized(const matrix::Vector2f &demand, float tilt) { return demand(0) * cosf(tilt) + demand(1) * sinf(tilt); }

	void slewToTarget(float dt);

	SlewRate<float> _tilt{};	///< realized tilt [rad]
	float _tilt_target{0.f};	///< tilt target the slew tracks [rad]
	float _tilt_min{0.f};		///< [rad]
	float _tilt_max{0.f};		///< [rad], >= _tilt_min
	bool _slew_limited{false};	///< false when the configured rate was 0, i.e. unlimited
	TiltMode _mode{TiltMode::Parked};
};
