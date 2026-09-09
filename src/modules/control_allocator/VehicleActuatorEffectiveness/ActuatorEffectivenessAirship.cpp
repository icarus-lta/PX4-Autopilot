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

#include "ActuatorEffectivenessAirship.hpp"

#include <float.h>

using namespace matrix;

// One guard, in radians, for declaring the tilt servos and writing them
static constexpr float kMinTiltSpan = 1e-3f;

bool
ActuatorEffectivenessAirship::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// The pod allocation is non-linear, so motors and tilts are declared
	// with zero effectiveness and computed in updateSetpoint(): motors 1/2 =
	// starboard/port thrust, optional motor 3 = tail yaw thruster, then the
	// tilt servos when the range is nonzero: starboard/port in independent
	// grouping, a single collective tilt. Control surfaces follow with
	// regular matrix effectiveness.
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});

	_has_tail = _param_ca_airship_tail.get();

	if (_has_tail) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
	}

	_independent = _param_ca_airship_grp.get() > 0;
	_first_tilt_idx = configuration.num_actuators_matrix[0];
	const bool has_tilt_range = tiltMax() - tiltMin() > kMinTiltSpan;
	_tilt_count = has_tilt_range ? (_independent ? 2 : 1) : 0;

	for (int i = 0; i < _tilt_count; i++) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	_first_control_surface_idx = configuration.num_actuators_matrix[0];
	const bool surfaces_added = _control_surfaces.addActuators(configuration);

	for (int axis = 0; axis < 3; axis++) {
		_surface_serves[axis] = false;

		for (int i = 0; i < _control_surfaces.count(); i++) {
			if (fabsf(_control_surfaces.config(i).torque(axis)) > FLT_EPSILON) {
				_surface_serves[axis] = true;
			}
		}
	}

	return surfaces_added;
}

void
ActuatorEffectivenessAirship::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max)
{
	// Normalized demands: forward/up thrust in units of the combined motor
	// maximum, roll/yaw in units of the maximum couple.
	const float thrust_forward = control_sp(ControlAxis::THRUST_X);
	const float thrust_up = -control_sp(ControlAxis::THRUST_Z);

	// Control surfaces are allocated by the matrix; the pods and the tail
	// serve the demand they leave unmet. The surface torque is credited
	// only by CA_AIRSHIP_CS_K: still-air surfaces deliver none of their
	// allocation, so at low credit the propulsors serve it instead.
	_surface_torque.setZero();

	for (int i = 0; i < _control_surfaces.count(); i++) {
		const int idx = _first_control_surface_idx + i;

		// Delivered torque in the allocator's model is effectiveness *
		// (setpoint - trim) after clipping, but this runs before
		// clipActuatorSetpoint(): apply the limits and trim locally so
		// saturated or trimmed surfaces are not credited torque they
		// cannot deliver. A CA_SVn_SLEW on a surface runs later still
		// and stays invisible to the credit.
		const float deflection = math::constrain(actuator_sp(idx), actuator_min(idx), actuator_max(idx))
					 - _control_surfaces.config(i).trim;
		_surface_torque += _control_surfaces.config(i).torque * deflection;
	}

	const float credit = _param_ca_airship_cs_k.get();

	const float yaw = control_sp(ControlAxis::YAW) - credit * _surface_torque(2);
	const float roll = control_sp(ControlAxis::ROLL) - credit * _surface_torque(0);

	const float tilt_min = tiltMin();
	const float tilt_max = tiltMax();

	// Per-pod force decomposition (0 = starboard, 1 = port)
	float fx[2] = {thrust_forward - yaw, thrust_forward + yaw};
	float fz[2] = {thrust_up - roll, thrust_up + roll};

	if (!_independent) {
		fx[0] = fx[1] = thrust_forward;
		fz[0] = fz[1] = thrust_up;
	}

	actuator_armed_s armed;

	if (_actuator_armed_sub.update(&armed)) {
		_armed = armed.armed;
	}

	// Clamp dt like the allocator's own scheduling guard (same 0.2 ms floor
	// as ControlAllocator::Run: it only bites above 5 kHz, and keeps a
	// zero-length interval from stalling the slew); the 100 ms ceiling,
	// looser than the allocator's 20 ms, only bounds the first step after
	// a scheduling gap
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_update_time) * 1e-6f, 2e-4f, 0.1f);
	_last_update_time = now;

	bool retargeted[2] {};	// the demand recomputed the tilt direction this update

	for (int i = 0; i < 2; i++) {
		// The range params may have narrowed at runtime: pull the held
		// state back in before anything uses it
		_tilt[i].setForcedValue(math::constrain(_tilt[i].getState(), tilt_min, tilt_max));
		_tilt_target[i] = math::constrain(_tilt_target[i], tilt_min, tilt_max);

		const float magnitude = sqrtf(fx[i] * fx[i] + fz[i] * fz[i]);

		// Engaged steering stands until the demand drops below the release
		// threshold; between release and engage the target holds unchanged
		const bool steering_holds = _tilt_steering[i] && magnitude > kTiltSteerRelease;

		if (!_armed) {
			// Disarmed, park the tilt as close to level as the range allows
			_tilt_steering[i] = false;
			_tilt_target[i] = math::constrain(0.f, tilt_min, tilt_max);

		} else if (magnitude > kTiltSteerEngage) {
			_tilt_steering[i] = true;
			retargeted[i] = true;
			_tilt_target[i] = steerTarget(fx[i], fz[i], magnitude, _tilt_target[i], tilt_min, tilt_max);

		} else if (!steering_holds) {
			// Released: hold the tilt where it is (inside the band the
			// engaged target stands)
			_tilt_steering[i] = false;
			_tilt_target[i] = _tilt[i].getState();
		}

		// The tilt is a physical state: rate-limit it toward the target
		if (_param_ca_airship_tlt_r.get() > 0.f) {
			_tilt[i].setSlewRate(math::radians(_param_ca_airship_tlt_r.get()));
			_tilt[i].update(_tilt_target[i], dt);

		} else {
			_tilt[i].setForcedValue(_tilt_target[i]);
		}
	}

	writeTiltServos(actuator_sp, actuator_min, actuator_max, tilt_min, tilt_max - tilt_min);

	float thrust[2];	// unclamped projection, read again by the held-share discount
	float achieved_x[2] {};
	float achieved_z[2] {};

	for (int i = 0; i < 2; i++) {
		const float cos_tilt = cosf(_tilt[i].getState());
		const float sin_tilt = sinf(_tilt[i].getState());

		// Project the demand onto the realized tilt: the feasible
		// component when the tilt is clamped, fixed or still slewing.
		thrust[i] = fx[i] * cos_tilt + fz[i] * sin_tilt;

		// The propellers are non-reversible: reverse thrust is reached by
		// tilting, never by a negative motor command (the CA_R_REV pod
		// bits are deliberately not honored here).
		actuator_sp(i) = math::constrain(thrust[i], math::max(actuator_min(i), 0.f), actuator_max(i));

		// The achieved wrench, for the demand left unmet on each axis
		achieved_x[i] = actuator_sp(i) * cos_tilt;
		achieved_z[i] = actuator_sp(i) * sin_tilt;
	}

	float achieved_yaw = 0.5f * (achieved_x[1] - achieved_x[0]);

	if (_has_tail) {
		// The tail thruster serves the yaw demand the pods leave unmet;
		// reverse authority comes from the motor configuration.
		actuator_sp(2) = math::constrain(yaw - achieved_yaw, actuator_min(2), actuator_max(2));
		achieved_yaw += actuator_sp(2);
	}

	// The pods produce no pitch torque
	_achieved_torque = Vector3f(0.5f * (achieved_z[1] - achieved_z[0]), 0.f, achieved_yaw);

	// A pod inside the steer band holds its direction on purpose (see
	// kTiltSteerEngage); what it projects away is the band's choice, not
	// missing authority, and reported as saturation it would freeze the rate
	// controller's integrator against the small steady torques the integral
	// exists to remove. Discount that part from the reported shortfall: a
	// swinging pod, a fixed mount or a tilt servo clamped short of its
	// target remain real (the servo write-back above keeps a clamped tilt
	// off its target). A held pod's demand sits inside the band, far below
	// the motor limit, so no motor-clamp check is needed.
	constexpr float kTiltSettled = 1e-3f; // rad
	float held_x[2] {};	// demand a held pod leaves unserved by choice
	float held_z[2] {};

	for (int i = 0; i < 2; i++) {
		const bool held = _tilt_count > 0 && _armed && !retargeted[i]
				  && fabsf(_tilt[i].getState() - _tilt_target[i]) < kTiltSettled;

		if (held) {
			held_x[i] = fx[i] - achieved_x[i];
			held_z[i] = fz[i] - achieved_z[i];
		}
	}

	// Mean pod force, in the same normalized units as the thrust demand
	const float achieved_forward = 0.5f * (achieved_x[0] + achieved_x[1]);
	const float achieved_up = 0.5f * (achieved_z[0] + achieved_z[1]);
	const float held_forward = 0.5f * (held_x[0] + held_x[1]);
	const float held_up = 0.5f * (held_z[0] + held_z[1]);

	// Collective pods project the same demand, so their held parts cancel
	// and the structural yaw/roll shortfall stands
	_held_torque = Vector3f(0.5f * (held_z[1] - held_z[0]), 0.f, 0.5f * (held_x[1] - held_x[0]));

	_unallocated_thrust[0] = saturationSign(discountHeld(thrust_forward - achieved_forward, held_forward));
	// No actuator produces lateral force: the demand is unserved as-is
	_unallocated_thrust[1] = saturationSign(control_sp(ControlAxis::THRUST_Y));
	// The pods work in up-positive terms; the Z control axis points down
	_unallocated_thrust[2] = saturationSign(-discountHeld(thrust_up - achieved_up, held_up));
	_unallocated_torque[0] = saturationSign(discountHeld(roll - _achieved_torque(0), _held_torque(0)));
	// The pods produce no pitch torque
	_unallocated_torque[1] = saturationSign(control_sp(ControlAxis::PITCH));
	_unallocated_torque[2] = saturationSign(discountHeld(yaw - _achieved_torque(2), _held_torque(2)));
}

float
ActuatorEffectivenessAirship::steerTarget(const float fx, const float fz, const float magnitude,
		const float committed, const float tilt_min, const float tilt_max)
{
	float tilt = atan2f(fz, fx);

	// Reaching the opposite range end costs a full sweep of an end-stop
	// servo, so both end selections below ignore advantages smaller than
	// this margin; the floor covers stick noise at low demand.
	const float commit_margin = fmaxf(kTiltRearCone * magnitude, kTiltSteerRelease);

	if (fx < 0.f && fabsf(fz) < commit_margin) {
		// Straight back, atan2 flips between +-180 deg on the sign of the
		// perpendicular component: pick the range end that realizes the
		// demand best, on a tie keep the committed end.
		const float rear_hi = math::constrain(M_PI_F, tilt_min, tilt_max);
		const float rear_lo = math::constrain(-M_PI_F, tilt_min, tilt_max);
		const float cos_hi = cosf(rear_hi);
		const float cos_lo = cosf(rear_lo);

		if (fabsf(cos_hi - cos_lo) > FLT_EPSILON) {
			tilt = cos_hi < cos_lo ? rear_hi : rear_lo;

		} else {
			tilt = committed >= 0.f ? rear_hi : rear_lo;
		}

	} else if (tilt < tilt_min || tilt > tilt_max) {
		// The tilt is circular but the range is a segment: for a target
		// outside it, the numerically nearer bound can point away from
		// the demand entirely (e.g. range -180..0, demand back and
		// slightly up). Choose the end that realizes more of the demand,
		// floored at zero since the motors cannot reverse, and switch
		// ends only past the commitment margin.
		const float p_hi = fmaxf(0.f, fx * cosf(tilt_max) + fz * sinf(tilt_max));
		const float p_lo = fmaxf(0.f, fx * cosf(tilt_min) + fz * sinf(tilt_min));
		const bool committed_hi = committed - tilt_min > tilt_max - committed;

		if (committed_hi) {
			tilt = p_lo > p_hi + commit_margin ? tilt_min : tilt_max;

		} else {
			tilt = p_hi > p_lo + commit_margin ? tilt_max : tilt_min;
		}
	}

	return math::constrain(tilt, tilt_min, tilt_max);
}

void
ActuatorEffectivenessAirship::writeTiltServos(ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max, const float tilt_min, const float tilt_span)
{
	// Write the tilt servos before projecting: the projection must use
	// the angle the servo output can actually realize. Realized means
	// after the min/max clamp - the generic CA_SVn_SLEW runs later and
	// is invisible to this model, so tilt slewing belongs in
	// CA_AIRSHIP_TLT_R. The declaration allocates tilt servos only for a
	// span above kMinTiltSpan; the span check also covers a parameter
	// change that has not been redeclared yet.
	if (_tilt_count == 0 || tilt_span <= kMinTiltSpan) {
		return;
	}

	for (int i = 0; i < _tilt_count; i++) {
		const int idx = _first_tilt_idx + i;
		const float tilt_sp = -1.f + 2.f * (_tilt[i].getState() - tilt_min) / tilt_span;
		actuator_sp(idx) = math::constrain(tilt_sp, actuator_min(idx), actuator_max(idx));
		_tilt[i].setForcedValue(tilt_min + (actuator_sp(idx) + 1.f) * 0.5f * tilt_span);
	}

	if (!_independent) {
		// The single collective servo drives both pods
		_tilt[1].setForcedValue(_tilt[0].getState());
	}
}

float
ActuatorEffectivenessAirship::discountHeld(float residual, float held_part)
{
	// Remove the held-by-choice share from a same-signed shortfall
	if (residual * held_part > 0.f) {
		return copysignf(fmaxf(fabsf(residual) - fabsf(held_part), 0.f), residual);
	}

	return residual;
}

float
ActuatorEffectivenessAirship::saturationSign(float shortfall)
{
	if (shortfall > FLT_EPSILON) {
		return 1.f;
	}

	if (shortfall < -FLT_EPSILON) {
		return -1.f;
	}

	return 0.f;
}

void
ActuatorEffectivenessAirship::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude
	// is not used. A shortfall the tilt steer band holds by choice reads as
	// achieved so the rate controller keeps integrating (see updateSetpoint).
	// Torque axes with control-surface effectiveness instead keep the matrix
	// residual, corrected for the uncredited share of the surface allocation
	// and reduced by what the pods and tail achieved; the held share is
	// discounted there too, so the band's choice never reads as saturation.
	// Pitch has nothing achieved or held to subtract: the pods produce none.
	const float uncredited = 1.f - _param_ca_airship_cs_k.get();

	for (int axis = 0; axis < 3; axis++) {
		if (_surface_serves[axis]) {
			status.unallocated_torque[axis] = discountHeld(status.unallocated_torque[axis]
							  + uncredited * _surface_torque(axis) - _achieved_torque(axis),
							  _held_torque(axis));

		} else {
			status.unallocated_torque[axis] = _unallocated_torque[axis];
		}

		// A lateral demand has no actuator to serve it and must not read as
		// allocated: it is reported like any other unserved axis
		status.unallocated_thrust[axis] = _unallocated_thrust[axis];
	}
}
