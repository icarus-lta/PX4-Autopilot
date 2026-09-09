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

ActuatorEffectivenessAirship::ActuatorEffectivenessAirship(ModuleParams *parent) :
	ModuleParams(parent),
	_control_surfaces(this)
{
	updateParams();
}

void
ActuatorEffectivenessAirship::updateParams()
{
	ModuleParams::updateParams();

	_grouping = _param_ca_airship_grp.get() > 0 ? Grouping::Independent : Grouping::Collective;
	_has_tail = _param_ca_airship_tail.get();

	const float tilt_min = math::radians(_param_ca_airship_tlmin.get());
	const float tilt_max = math::radians(_param_ca_airship_tlmax.get());
	const float slew_rate = math::radians(_param_ca_airship_tlt_r.get());

	for (AirshipPod &pod : _pods) {
		pod.setTiltRange(tilt_min, tilt_max);
		pod.setTiltSlewRate(slew_rate);
	}
}

bool
ActuatorEffectivenessAirship::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Non-linear allocation: zero effectiveness here, computed in updateSetpoint()
	for (int i = 0; i < NUM_PODS; i++) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
	}

	if (_has_tail) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
	}

	_first_tilt_idx = configuration.num_actuators_matrix[0];
	_num_tilt_servos = _pods[STARBOARD].canTilt() ? (_grouping == Grouping::Independent ? NUM_PODS : 1) : 0;

	for (int i = 0; i < _num_tilt_servos; i++) {
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

bool
ActuatorEffectivenessAirship::isArmed()
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	return _armed;
}

void
ActuatorEffectivenessAirship::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max)
{
	const float thrust_forward = control_sp(ControlAxis::THRUST_X);
	const float thrust_up = -control_sp(ControlAxis::THRUST_Z);

	_surface_torque.setZero();

	for (int i = 0; i < _control_surfaces.count(); i++) {
		const int idx = _first_control_surface_idx + i;

		// The allocator credits effectiveness * (setpoint - trim) after
		// clipping, but this runs before clipActuatorSetpoint(): clip and
		// trim locally so a saturated or trimmed surface is not credited
		// torque it cannot deliver. A CA_SVn_SLEW on a surface runs later
		// and stays invisible.
		const float deflection = math::constrain(actuator_sp(idx), actuator_min(idx), actuator_max(idx))
					 - _control_surfaces.config(i).trim;
		_surface_torque += _control_surfaces.config(i).torque * deflection;
	}

	const float credit = _param_ca_airship_cs_k.get();

	const float yaw = control_sp(ControlAxis::YAW) - credit * _surface_torque(2);
	const float roll = control_sp(ControlAxis::ROLL) - credit * _surface_torque(0);

	float fx[NUM_PODS] = {thrust_forward - yaw, thrust_forward + yaw};	// starboard, port
	float fz[NUM_PODS] = {thrust_up - roll, thrust_up + roll};

	if (_grouping == Grouping::Collective) {
		fx[STARBOARD] = fx[PORT] = thrust_forward;
		fz[STARBOARD] = fz[PORT] = thrust_up;
	}

	const bool armed = isArmed();

	for (int i = 0; i < NUM_PODS; i++) {
		if (armed) {
			_pods[i].steer(Vector2f{fx[i], fz[i]}, _dt);

		} else {
			_pods[i].park(_dt);
		}
	}

	writeTiltServos(actuator_sp, actuator_min, actuator_max);

	float achieved_x[NUM_PODS] {};
	float achieved_z[NUM_PODS] {};

	for (int i = 0; i < NUM_PODS; i++) {
		const Vector2f axis = _pods[i].thrustAxis();
		const float thrust = fx[i] * axis(0) + fz[i] * axis(1);

		// non-reversible propellers: reverse only by tilting, the CA_R_REV pod bits are not honored
		actuator_sp(i) = math::constrain(thrust, math::max(actuator_min(i), 0.f), actuator_max(i));

		achieved_x[i] = actuator_sp(i) * axis(0);
		achieved_z[i] = actuator_sp(i) * axis(1);
	}

	float achieved_yaw = 0.5f * (achieved_x[PORT] - achieved_x[STARBOARD]);

	if (_has_tail) {
		// the tail serves the yaw the pods leave; reverse authority comes from CA_R_REV
		actuator_sp(TAIL) = math::constrain(yaw - achieved_yaw, actuator_min(TAIL), actuator_max(TAIL));
		achieved_yaw += actuator_sp(TAIL);
	}

	// The pods produce no pitch torque
	_achieved_torque = Vector3f(0.5f * (achieved_z[PORT] - achieved_z[STARBOARD]), 0.f, achieved_yaw);

	float held_x[NUM_PODS] {};
	float held_z[NUM_PODS] {};

	for (int i = 0; i < NUM_PODS; i++) {
		if (_num_tilt_servos > 0 && _pods[i].heldInBand()) {
			held_x[i] = fx[i] - achieved_x[i];
			held_z[i] = fz[i] - achieved_z[i];
		}
	}

	const float achieved_forward = 0.5f * (achieved_x[STARBOARD] + achieved_x[PORT]);
	const float achieved_up = 0.5f * (achieved_z[STARBOARD] + achieved_z[PORT]);
	const float held_forward = 0.5f * (held_x[STARBOARD] + held_x[PORT]);
	const float held_up = 0.5f * (held_z[STARBOARD] + held_z[PORT]);

	_held_torque = Vector3f(0.5f * (held_z[PORT] - held_z[STARBOARD]), 0.f, 0.5f * (held_x[PORT] - held_x[STARBOARD]));

	_unallocated_thrust[0] = saturationSign(discountHeld(thrust_forward - achieved_forward, held_forward));
	_unallocated_thrust[1] = saturationSign(control_sp(ControlAxis::THRUST_Y));	// no lateral actuator
	_unallocated_thrust[2] = saturationSign(-discountHeld(thrust_up - achieved_up, held_up));	// z down
	_unallocated_torque[0] = saturationSign(discountHeld(roll - _achieved_torque(0), _held_torque(0)));
	_unallocated_torque[1] = saturationSign(control_sp(ControlAxis::PITCH));
	_unallocated_torque[2] = saturationSign(discountHeld(yaw - _achieved_torque(2), _held_torque(2)));
}

void
ActuatorEffectivenessAirship::writeTiltServos(ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max)
{
	// Written before the motors so the projection uses the clamped angle the
	// servo realizes; the generic CA_SVn_SLEW runs after this model, so tilt
	// slewing belongs to CA_AIRSHIP_TLT_R
	if (_num_tilt_servos == 0 || !_pods[STARBOARD].canTilt()) {
		return;
	}

	for (int i = 0; i < _num_tilt_servos; i++) {
		const int idx = _first_tilt_idx + i;
		actuator_sp(idx) = math::constrain(_pods[i].servoSetpoint(), actuator_min(idx), actuator_max(idx));
		_pods[i].setServoSetpoint(actuator_sp(idx));
	}

	if (_grouping == Grouping::Collective) {
		_pods[PORT].setTilt(_pods[STARBOARD].tilt());
	}
}

float
ActuatorEffectivenessAirship::discountHeld(float shortfall, float held)
{
	if (shortfall * held > 0.f) {
		return copysignf(fmaxf(fabsf(shortfall) - fabsf(held), 0.f), shortfall);
	}

	return shortfall;
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
	// positive or no saturation to the rate controller; the magnitude is not
	// used. Torque axes a control surface serves keep the matrix residual
	// (corrected for the uncredited surface share, less what the pods and
	// tail achieved and the held share).
	const float uncredited = 1.f - _param_ca_airship_cs_k.get();

	for (int axis = 0; axis < 3; axis++) {
		if (_surface_serves[axis]) {
			status.unallocated_torque[axis] = discountHeld(status.unallocated_torque[axis]
							  + uncredited * _surface_torque(axis) - _achieved_torque(axis),
							  _held_torque(axis));

		} else {
			status.unallocated_torque[axis] = _unallocated_torque[axis];
		}

		status.unallocated_thrust[axis] = _unallocated_thrust[axis];
	}
}
