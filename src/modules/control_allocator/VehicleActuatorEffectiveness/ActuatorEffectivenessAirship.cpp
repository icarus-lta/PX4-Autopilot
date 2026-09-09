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
	_num_tilt_servos = 0;

	if (_pods[STARBOARD].canTilt()) {
		_num_tilt_servos = _grouping == Grouping::Independent ? NUM_PODS : 1;	// one collective servo drives both pods
	}

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

void
ActuatorEffectivenessAirship::updateArmedState()
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}
}

Vector3f
ActuatorEffectivenessAirship::surfaceTorque(const ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max) const
{
	// The allocator credits effectiveness * (setpoint - trim) after clipping,
	// but this runs before clipActuatorSetpoint(): clip and trim locally so a
	// saturated or trimmed surface is not credited torque it cannot deliver.
	// A CA_SVn_SLEW on a surface runs later and stays invisible.
	Vector3f torque{};

	for (int i = 0; i < _control_surfaces.count(); i++) {
		const int idx = _first_control_surface_idx + i;
		const float deflection = math::constrain(actuator_sp(idx), actuator_min(idx), actuator_max(idx))
					 - _control_surfaces.config(i).trim;
		torque += _control_surfaces.config(i).torque * deflection;
	}

	return torque;
}

Vector<float, ActuatorEffectiveness::NUM_AXES>
ActuatorEffectivenessAirship::podWrench(const Vector2f &starboard, const Vector2f &port)
{
	Vector<float, NUM_AXES> wrench{};
	wrench(ControlAxis::ROLL) = 0.5f * (port(1) - starboard(1));
	wrench(ControlAxis::PITCH) = 0.f;	// the pods produce no pitch torque
	wrench(ControlAxis::YAW) = 0.5f * (port(0) - starboard(0));
	wrench(ControlAxis::THRUST_X) = 0.5f * (starboard(0) + port(0));
	wrench(ControlAxis::THRUST_Y) = 0.f;	// no lateral actuator
	wrench(ControlAxis::THRUST_Z) = -(0.5f * (starboard(1) + port(1)));	// z down
	return wrench;
}

void
ActuatorEffectivenessAirship::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max)
{
	_surface_torque = surfaceTorque(actuator_sp, actuator_min, actuator_max);
	const float credit = _param_ca_airship_cs_k.get();

	// The pods and the tail serve what the credited surfaces leave
	Vector<float, NUM_AXES> demand = control_sp;

	for (int axis = 0; axis < 3; axis++) {
		demand(axis) -= credit * _surface_torque(axis);
	}

	const Vector2f common{demand(ControlAxis::THRUST_X), -demand(ControlAxis::THRUST_Z)};
	const Vector2f differential = _grouping == Grouping::Independent
				      ? Vector2f{demand(ControlAxis::YAW), demand(ControlAxis::ROLL)} : Vector2f{};
	const Vector2f pod_demand[NUM_PODS] = {common - differential, common + differential};

	updateArmedState();

	for (int i = 0; i < NUM_PODS; i++) {
		if (_armed) {
			_pods[i].steer(pod_demand[i], _dt);

		} else {
			_pods[i].park(_dt);
		}
	}

	writeTiltServos(actuator_sp, actuator_min, actuator_max);

	Vector2f achieved[NUM_PODS] {};
	Vector2f held[NUM_PODS] {};

	for (int i = 0; i < NUM_PODS; i++) {
		const Vector2f thrust_axis = _pods[i].thrustAxis();
		const float thrust = pod_demand[i].dot(thrust_axis);

		// non-reversible propellers: reverse only by tilting, the CA_R_REV pod bits are not honored
		actuator_sp(i) = math::constrain(thrust, math::max(actuator_min(i), 0.f), actuator_max(i));
		achieved[i] = thrust_axis * actuator_sp(i);

		if (_pods[i].heldByChoice()) {
			held[i] = pod_demand[i] - achieved[i];
		}
	}

	Vector<float, NUM_AXES> achieved_wrench = podWrench(achieved[STARBOARD], achieved[PORT]);

	if (_has_tail) {
		// the tail serves the yaw the pods leave; reverse authority comes from CA_R_REV
		actuator_sp(TAIL) = math::constrain(demand(ControlAxis::YAW) - achieved_wrench(ControlAxis::YAW),
						    actuator_min(TAIL), actuator_max(TAIL));
		achieved_wrench(ControlAxis::YAW) += actuator_sp(TAIL);
	}

	const Vector<float, NUM_AXES> held_wrench = podWrench(held[STARBOARD], held[PORT]);

	for (int axis = 0; axis < NUM_AXES; axis++) {
		_unallocated_control(axis) = discountHeld(demand(axis) - achieved_wrench(axis), held_wrench(axis));
	}

	_achieved_torque = achieved_wrench.slice<3, 1>(0, 0);
	_held_torque = held_wrench.slice<3, 1>(0, 0);
}

void
ActuatorEffectivenessAirship::writeTiltServos(ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max)
{
	// Written before the motors so the projection uses the clamped angle the
	// servo realizes; the generic CA_SVn_SLEW runs after this model, so tilt
	// slewing belongs to CA_AIRSHIP_TLT_R. The count is the declared layout;
	// the span check also covers a range updateParams() collapsed without a
	// redeclaration, where servoSetpoint() would divide by zero
	if (_num_tilt_servos == 0 || !_pods[STARBOARD].canTilt()) {
		return;
	}

	for (int i = 0; i < _num_tilt_servos; i++) {
		const int idx = _first_tilt_idx + i;
		actuator_sp(idx) = math::constrain(_pods[i].servoSetpoint(), actuator_min(idx), actuator_max(idx));
		_pods[i].setServoSetpoint(actuator_sp(idx));
	}

	if (_grouping == Grouping::Collective) {
		_pods[PORT].setServoSetpoint(actuator_sp(_first_tilt_idx));
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
	// The rate controller gates on torque_setpoint_achieved, then reads only
	// the sign of each axis. Torque axes a control surface serves keep the
	// matrix residual: it credits the surface in full, so add back the share
	// CA_AIRSHIP_CS_K left uncredited (the pods were asked to serve it),
	// subtract what the pods and tail achieved and discount the held share.
	// The other axes report a sign only.
	const float uncredited = 1.f - _param_ca_airship_cs_k.get();

	for (int axis = 0; axis < 3; axis++) {
		if (_surface_serves[axis]) {
			status.unallocated_torque[axis] = discountHeld(status.unallocated_torque[axis]
							  + uncredited * _surface_torque(axis) - _achieved_torque(axis),
							  _held_torque(axis));

		} else {
			status.unallocated_torque[axis] = saturationSign(_unallocated_control(axis));
		}

		status.unallocated_thrust[axis] = saturationSign(_unallocated_control(ControlAxis::THRUST_X + axis));
	}
}
