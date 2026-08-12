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

#include <mathlib/mathlib.h>

#include <float.h>

using namespace matrix;

// The tilt direction is the atan2 of the pod force demand, which is
// meaningless near zero magnitude: stick noise alone would slam the tilt
// between opposite directions. Steering therefore engages only above the
// stick-noise floor and releases at half of it; inside the band the last
// commanded direction stands, so a sign reversal there cannot retarget.
static constexpr float kTiltSteerEngage = 0.02f;
static constexpr float kTiltSteerRelease = 0.01f;

// Pointing (near-)straight back, +180 and -180 deg realize the same thrust
// direction at opposite ends of an end-stop servo: within this cone of the
// negative x axis the previously committed end is kept, so perpendicular
// noise cannot command a full-range sweep.
static constexpr float kTiltRearCone = 0.05f;

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
	const bool has_tilt_range = math::radians(_param_ca_airship_tlmax.get() - _param_ca_airship_tlmin.get()) >
				    kMinTiltSpan;
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
	_saturation_flags = {};

	// Normalized demands: forward/up thrust in units of the combined motor
	// maximum, roll/yaw in units of the maximum couple.
	const float thrust_forward = control_sp(ControlAxis::THRUST_X);
	const float thrust_up = -control_sp(ControlAxis::THRUST_Z);

	// Control surfaces are allocated by the matrix; the pods and the tail
	// serve the demand they leave unmet. The surface torque is credited
	// only by CA_AIRSHIP_CS_K: still-air surfaces deliver none of their
	// allocation, so at low credit the propulsors serve it instead.
	float surface_roll = 0.f;
	float surface_pitch = 0.f;
	float surface_yaw = 0.f;

	for (int i = 0; i < _control_surfaces.count(); i++) {
		const int idx = _first_control_surface_idx + i;

		// Delivered torque in the allocator's model is effectiveness *
		// (setpoint - trim) after clipping, but this runs before
		// clipActuatorSetpoint(): apply the limits and trim locally so
		// saturated or trimmed surfaces are not credited torque they
		// cannot deliver.
		const float deflection = math::constrain(actuator_sp(idx), actuator_min(idx), actuator_max(idx))
					 - _control_surfaces.config(i).trim;
		const Vector3f &torque = _control_surfaces.config(i).torque;
		surface_roll += torque(0) * deflection;
		surface_pitch += torque(1) * deflection;
		surface_yaw += torque(2) * deflection;
	}

	_surface_roll = surface_roll;
	_surface_pitch = surface_pitch;
	_surface_yaw = surface_yaw;

	const float credit = _param_ca_airship_cs_k.get();

	const float yaw = control_sp(ControlAxis::YAW) - credit * surface_yaw;
	const float roll = control_sp(ControlAxis::ROLL) - credit * surface_roll;

	const float tilt_min = math::radians(_param_ca_airship_tlmin.get());
	const float tilt_max = math::radians(_param_ca_airship_tlmax.get());
	const float tilt_span = tilt_max - tilt_min;

	// Per-pod force decomposition (0 = starboard, 1 = port)
	float fx[2] = {thrust_forward - yaw, thrust_forward + yaw};
	float fz[2] = {thrust_up - roll, thrust_up + roll};

	if (!_independent) {
		fx[0] = fx[1] = thrust_forward;
		fz[0] = fz[1] = thrust_up;
	}

	if (_actuator_armed_sub.updated()) {
		actuator_armed_s armed;

		if (_actuator_armed_sub.copy(&armed)) {
			_armed = armed.armed;
		}
	}

	// dt floor matches the allocator's own scheduling clamp: a larger
	// floor would inflate the slew step at fast gyro rates
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_update_time) * 1e-6f, 2e-4f, 0.1f);
	_last_update_time = now;

	for (int i = 0; i < 2; i++) {
		// The range params may have narrowed at runtime: pull the held
		// state back in before anything uses it
		_tilt[i] = math::constrain(_tilt[i], tilt_min, tilt_max);
		_tilt_target[i] = math::constrain(_tilt_target[i], tilt_min, tilt_max);

		const float magnitude = sqrtf(fx[i] * fx[i] + fz[i] * fz[i]);

		if (!_armed) {
			// Disarmed, park the tilt as close to level as the range allows
			_tilt_steering[i] = false;
			_tilt_target[i] = math::constrain(0.f, tilt_min, tilt_max);

		} else if (magnitude > kTiltSteerEngage) {
			_tilt_steering[i] = true;
			float tilt = atan2f(fz[i], fx[i]);

			// A (near-)straight-back demand is realizable at either end:
			// keep the end already committed to, so noise on the
			// perpendicular axis cannot flip the target across the range
			if (fx[i] < 0.f && fabsf(fz[i]) < kTiltRearCone * magnitude) {
				tilt = _tilt_target[i] >= 0.f ? M_PI_F : -M_PI_F;
			}

			_tilt_target[i] = math::constrain(tilt, tilt_min, tilt_max);

		} else if (!(_tilt_steering[i] && magnitude > kTiltSteerRelease)) {
			// Released below the band: hold the tilt where it is. Inside
			// the band the engaged target stands unchanged.
			_tilt_steering[i] = false;
			_tilt_target[i] = _tilt[i];
		}

		// The tilt is a physical state: rate-limit it toward the target
		if (_param_ca_airship_tlt_r.get() > 0.f) {
			const float max_step = math::radians(_param_ca_airship_tlt_r.get()) * dt;
			_tilt[i] += math::constrain(_tilt_target[i] - _tilt[i], -max_step, max_step);

		} else {
			_tilt[i] = _tilt_target[i];
		}
	}

	// Write the tilt servos before projecting: the projection must use
	// the angle the servo output can actually realize
	if (_tilt_count > 0 && tilt_span > kMinTiltSpan) {
		for (int i = 0; i < _tilt_count; i++) {
			const int idx = _first_tilt_idx + i;
			const float tilt_sp = -1.f + 2.f * (_tilt[i] - tilt_min) / tilt_span;
			actuator_sp(idx) = math::constrain(tilt_sp, actuator_min(idx), actuator_max(idx));
			_tilt[i] = tilt_min + (actuator_sp(idx) + 1.f) * 0.5f * tilt_span;
		}

		if (!_independent) {
			// The single collective servo drives both pods
			_tilt[1] = _tilt[0];
		}
	}

	float thrust[2];
	float cos_tilt[2];
	float sin_tilt[2];

	for (int i = 0; i < 2; i++) {
		cos_tilt[i] = cosf(_tilt[i]);
		sin_tilt[i] = sinf(_tilt[i]);

		// Project the demand onto the realized tilt: the feasible
		// component when the tilt is clamped, fixed or still slewing.
		thrust[i] = fx[i] * cos_tilt[i] + fz[i] * sin_tilt[i];
	}

	for (int i = 0; i < 2; i++) {
		// The propellers are non-reversible: reverse thrust is reached by
		// tilting, never by a negative motor command (the CA_R_REV pod
		// bits are deliberately not honored here).
		actuator_sp(i) = math::constrain(thrust[i], math::max(actuator_min(i), 0.f), actuator_max(i));
	}

	// Report the demand left unmet on each axis by the achieved wrench.
	const float achieved_x[2] = {actuator_sp(0) *cos_tilt[0], actuator_sp(1) *cos_tilt[1]};
	const float achieved_z[2] = {actuator_sp(0) *sin_tilt[0], actuator_sp(1) *sin_tilt[1]};
	float achieved_yaw = 0.5f * (achieved_x[1] - achieved_x[0]);

	if (_has_tail) {
		// The tail thruster serves the yaw demand the pods leave unmet;
		// reverse authority comes from the motor configuration.
		actuator_sp(2) = math::constrain(yaw - achieved_yaw, actuator_min(2), actuator_max(2));
		achieved_yaw += actuator_sp(2);
	}

	_achieved_roll = 0.5f * (achieved_z[1] - achieved_z[0]);
	_achieved_yaw = achieved_yaw;

	setSaturationFlag(thrust_forward - 0.5f * (achieved_x[0] + achieved_x[1]),
			  _saturation_flags.thrust_x_pos, _saturation_flags.thrust_x_neg);
	// No actuator produces lateral force: the demand is unserved as-is
	setSaturationFlag(control_sp(ControlAxis::THRUST_Y),
			  _saturation_flags.thrust_y_pos, _saturation_flags.thrust_y_neg);
	setSaturationFlag(-(thrust_up - 0.5f * (achieved_z[0] + achieved_z[1])),
			  _saturation_flags.thrust_z_pos, _saturation_flags.thrust_z_neg);
	setSaturationFlag(yaw - achieved_yaw, _saturation_flags.yaw_pos, _saturation_flags.yaw_neg);
	setSaturationFlag(roll - _achieved_roll,
			  _saturation_flags.roll_pos, _saturation_flags.roll_neg);
	// The pods produce no pitch torque
	setSaturationFlag(control_sp(ControlAxis::PITCH), _saturation_flags.pitch_pos, _saturation_flags.pitch_neg);
}

void
ActuatorEffectivenessAirship::setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag)
{
	if (coeff > FLT_EPSILON) {
		positive_flag = true;

	} else if (coeff < -FLT_EPSILON) {
		negative_flag = true;
	}
}

void
ActuatorEffectivenessAirship::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude
	// is not used. Torque axes with control-surface effectiveness instead
	// keep the matrix residual, corrected for the uncredited share of the
	// surface allocation and reduced by what the pods and tail achieved.
	const float uncredited = 1.f - _param_ca_airship_cs_k.get();

	if (_surface_serves[0]) {
		status.unallocated_torque[0] += uncredited * _surface_roll - _achieved_roll;

	} else if (_saturation_flags.roll_pos) {
		status.unallocated_torque[0] = 1.f;

	} else if (_saturation_flags.roll_neg) {
		status.unallocated_torque[0] = -1.f;

	} else {
		status.unallocated_torque[0] = 0.f;
	}

	if (_surface_serves[1]) {
		// The pods produce no pitch torque: the credited residual stands
		status.unallocated_torque[1] += uncredited * _surface_pitch;

	} else if (_saturation_flags.pitch_pos) {
		status.unallocated_torque[1] = 1.f;

	} else if (_saturation_flags.pitch_neg) {
		status.unallocated_torque[1] = -1.f;

	} else {
		status.unallocated_torque[1] = 0.f;
	}

	if (_surface_serves[2]) {
		status.unallocated_torque[2] += uncredited * _surface_yaw - _achieved_yaw;

	} else if (_saturation_flags.yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_saturation_flags.yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}

	if (_saturation_flags.thrust_x_pos) {
		status.unallocated_thrust[0] = 1.f;

	} else if (_saturation_flags.thrust_x_neg) {
		status.unallocated_thrust[0] = -1.f;

	} else {
		status.unallocated_thrust[0] = 0.f;
	}

	// A lateral demand has no actuator to serve it and must not read as
	// allocated: report it like any other unserved axis
	if (_saturation_flags.thrust_y_pos) {
		status.unallocated_thrust[1] = 1.f;

	} else if (_saturation_flags.thrust_y_neg) {
		status.unallocated_thrust[1] = -1.f;

	} else {
		status.unallocated_thrust[1] = 0.f;
	}

	if (_saturation_flags.thrust_z_pos) {
		status.unallocated_thrust[2] = 1.f;

	} else if (_saturation_flags.thrust_z_neg) {
		status.unallocated_thrust[2] = -1.f;

	} else {
		status.unallocated_thrust[2] = 0.f;
	}
}
