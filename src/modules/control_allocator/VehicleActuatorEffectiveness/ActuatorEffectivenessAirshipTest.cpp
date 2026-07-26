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

#include <gtest/gtest.h>
#include "ActuatorEffectivenessAirship.hpp"

using namespace matrix;

// Actuator indices
static constexpr int MOTOR_STARBOARD = 0;
static constexpr int MOTOR_PORT = 1;
static constexpr int TILT_STARBOARD = 2;
static constexpr int TILT_PORT = 3;

static void setTiltRange(float tilt_min_deg = -180.f, float tilt_max_deg = 180.f)
{
	// Disable autosaving parameters to avoid busy loop in param_set()
	param_control_autosave(false);

	param_set(param_find("CA_AIRSHIP_TLMIN"), &tilt_min_deg);
	param_set(param_find("CA_AIRSHIP_TLMAX"), &tilt_max_deg);
}

static void runUpdateSetpoint(ActuatorEffectivenessAirship &airship, const Vector<float, 6> &control_sp,
			      ActuatorEffectiveness::ActuatorVector &actuator_sp)
{
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(0.f);
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
}

TEST(ActuatorEffectivenessAirshipTest, VectoredConfiguration)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);

	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators_matrix[0], 4);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::MOTORS], 2);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 2);
}

TEST(ActuatorEffectivenessAirshipTest, ForwardCruise)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, FullYawIsTheExactCouple)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// Equal thrust, starboard pod reversed: zero net force, maximum couple
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f); // +180 deg
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);      // forward
}

TEST(ActuatorEffectivenessAirshipTest, CollectiveClimb)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.5f); // +90 deg
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.5f);
}

TEST(ActuatorEffectivenessAirshipTest, CruiseWithYaw)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.6f;
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.2f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.4f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.8f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, SaturationClampsAndReports)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 1.f;
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// Starboard pod demand cancels to zero, port pod saturates at 2x
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f); // yaw saturated positive
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 1.f); // forward thrust saturated positive
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], 0.f);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, TiltHeldThroughZeroThrust)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	// Zero demand: motors stop, tilts hold their last direction
	control_sp.setZero();
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, MotorsStayUnidirectional)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);

	// The props are non-reversible: any demand, including full reverse,
	// must map to motor commands in [0, 1]. Reversal is done by the tilt.
	for (float thrust = -1.f; thrust <= 1.f; thrust += 0.5f) {
		for (float yaw = -1.f; yaw <= 1.f; yaw += 0.5f) {
			for (float roll = -1.f; roll <= 1.f; roll += 0.5f) {
				Vector<float, 6> control_sp{};
				control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = thrust;
				control_sp(ActuatorEffectiveness::ControlAxis::YAW) = yaw;
				control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = roll;
				ActuatorEffectiveness::ActuatorVector actuator_sp{};
				runUpdateSetpoint(airship, control_sp, actuator_sp);

				EXPECT_GE(actuator_sp(MOTOR_STARBOARD), 0.f);
				EXPECT_LE(actuator_sp(MOTOR_STARBOARD), 1.f);
				EXPECT_GE(actuator_sp(MOTOR_PORT), 0.f);
				EXPECT_LE(actuator_sp(MOTOR_PORT), 1.f);
			}
		}
	}
}

TEST(ActuatorEffectivenessAirshipTest, TiltRangeLimited)
{
	setTiltRange(-90.f, 90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// Reverse is unreachable: the starboard tilt clamps to the +90 deg limit
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f); // +90 deg = range maximum
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);      // 0 deg = range center
}
