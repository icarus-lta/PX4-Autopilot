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

bool
ActuatorEffectivenessAirship::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// The allocation is non-linear, so the actuators are declared with zero
	// effectiveness and computed in updateSetpoint(): motors 1/2 =
	// starboard/port thrust, optional motor 3 = tail yaw thruster,
	// followed by servos 1/2 = starboard/port tilt.
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});

	if (_param_ca_airship_aux.get()) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
	}

	configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});

	return true;
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
	const float yaw = control_sp(ControlAxis::YAW);
	const float roll = control_sp(ControlAxis::ROLL);

	const float tilt_min = math::radians(_param_ca_airship_tlmin.get());
	const float tilt_max = math::radians(_param_ca_airship_tlmax.get());
	const float tilt_span = tilt_max - tilt_min;
	const bool independent = _param_ca_airship_grp.get() > 0;

	// Per-pod force decomposition (0 = starboard, 1 = port)
	float fx[2] = {thrust_forward - yaw, thrust_forward + yaw};
	float fz[2] = {thrust_up - roll, thrust_up + roll};

	if (!independent) {
		fx[0] = fx[1] = thrust_forward;
		fz[0] = fz[1] = thrust_up;
	}

	float thrust[2];
	float cos_tilt[2];
	float sin_tilt[2];

	for (int i = 0; i < 2; i++) {
		const float magnitude = sqrtf(fx[i] * fx[i] + fz[i] * fz[i]);

		// Hold the previous tilt through (near-)zero thrust, where the
		// direction is undefined.
		if (magnitude > 1e-3f) {
			float tilt = atan2f(fz[i], fx[i]);

			// atan2 of negative zero yields -pi: prefer the +180 deg
			// reverse so the tilt goes up, through the range maximum.
			if (fabsf(fz[i]) < FLT_EPSILON && fx[i] < 0.f) {
				tilt = M_PI_F;
			}

			_tilt[i] = math::constrain(tilt, tilt_min, tilt_max);
		}

		cos_tilt[i] = cosf(_tilt[i]);
		sin_tilt[i] = sinf(_tilt[i]);

		// Project the demand onto the realized tilt: the feasible
		// component when the tilt is clamped or fixed.
		thrust[i] = fx[i] * cos_tilt[i] + fz[i] * sin_tilt[i];
	}

	const bool aux = _param_ca_airship_aux.get();
	const int tilt_index = aux ? 3 : 2;

	for (int i = 0; i < 2; i++) {
		// The propellers are non-reversible: reverse thrust is reached by
		// tilting, never by a negative motor command.
		actuator_sp(i) = math::constrain(thrust[i], math::max(actuator_min(i), 0.f), actuator_max(i));

		const float tilt_sp = (tilt_span > FLT_EPSILON) ? (-1.f + 2.f * (_tilt[i] - tilt_min) / tilt_span) : 0.f;
		actuator_sp(tilt_index + i) = math::constrain(tilt_sp, actuator_min(tilt_index + i),
					      actuator_max(tilt_index + i));
	}

	// Report the demand left unmet on each axis by the achieved wrench.
	const float achieved_x[2] = {actuator_sp(0) *cos_tilt[0], actuator_sp(1) *cos_tilt[1]};
	const float achieved_z[2] = {actuator_sp(0) *sin_tilt[0], actuator_sp(1) *sin_tilt[1]};
	float achieved_yaw = 0.5f * (achieved_x[1] - achieved_x[0]);

	if (aux) {
		// The tail thruster serves the yaw demand the pods leave unmet;
		// reverse authority comes from the motor configuration.
		actuator_sp(2) = math::constrain(yaw - achieved_yaw, actuator_min(2), actuator_max(2));
		achieved_yaw += actuator_sp(2);
	}

	setSaturationFlag(thrust_forward - 0.5f * (achieved_x[0] + achieved_x[1]),
			  _saturation_flags.thrust_x_pos, _saturation_flags.thrust_x_neg);
	setSaturationFlag(-(thrust_up - 0.5f * (achieved_z[0] + achieved_z[1])),
			  _saturation_flags.thrust_z_pos, _saturation_flags.thrust_z_neg);
	setSaturationFlag(yaw - achieved_yaw, _saturation_flags.yaw_pos, _saturation_flags.yaw_neg);
	setSaturationFlag(roll - 0.5f * (achieved_z[1] - achieved_z[0]),
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
	// is not used.
	if (_saturation_flags.roll_pos) {
		status.unallocated_torque[0] = 1.f;

	} else if (_saturation_flags.roll_neg) {
		status.unallocated_torque[0] = -1.f;

	} else {
		status.unallocated_torque[0] = 0.f;
	}

	if (_saturation_flags.pitch_pos) {
		status.unallocated_torque[1] = 1.f;

	} else if (_saturation_flags.pitch_neg) {
		status.unallocated_torque[1] = -1.f;

	} else {
		status.unallocated_torque[1] = 0.f;
	}

	if (_saturation_flags.yaw_pos) {
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

	status.unallocated_thrust[1] = 0.f;

	if (_saturation_flags.thrust_z_pos) {
		status.unallocated_thrust[2] = 1.f;

	} else if (_saturation_flags.thrust_z_neg) {
		status.unallocated_thrust[2] = -1.f;

	} else {
		status.unallocated_thrust[2] = 0.f;
	}
}
