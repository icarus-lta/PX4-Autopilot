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

// With the tail thruster (CA_AIRSHIP_AUX) the tilts shift by one
static constexpr int MOTOR_TAIL = 2;
static constexpr int AUX_TILT_STARBOARD = 3;
static constexpr int AUX_TILT_PORT = 4;

static void setTiltRange(float tilt_min_deg = -180.f, float tilt_max_deg = 180.f)
{
	// Disable autosaving parameters to avoid busy loop in param_set()
	param_control_autosave(false);

	int32_t grouping = 1;
	param_set(param_find("CA_AIRSHIP_GRP"), &grouping);
	int32_t aux = 0;
	param_set(param_find("CA_AIRSHIP_AUX"), &aux);
	param_set(param_find("CA_AIRSHIP_TLMIN"), &tilt_min_deg);
	param_set(param_find("CA_AIRSHIP_TLMAX"), &tilt_max_deg);
}

static void setAuxThruster()
{
	int32_t aux = 1;
	param_set(param_find("CA_AIRSHIP_AUX"), &aux);
}

static void setCollectiveMode()
{
	int32_t grouping = 0;
	param_set(param_find("CA_AIRSHIP_GRP"), &grouping);
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

	// Equal and opposite thrust vectors: zero net force, maximum yaw couple
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f); // +180 deg
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);      // forward
}

TEST(ActuatorEffectivenessAirshipTest, VerticalClimb)
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
	// and the starboard demand has no feasible component along it.
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f); // +90 deg = range maximum
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);      // 0 deg = range center
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f); // half the couple is missing
}

TEST(ActuatorEffectivenessAirshipTest, CollectiveModeCruiseAndClimb)
{
	setTiltRange();
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);

	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.5f); // +90 deg, both pods
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.5f);
}

TEST(ActuatorEffectivenessAirshipTest, FixedMountRejectsVertical)
{
	setTiltRange(0.f, 0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// Vertical demand has no component along the fixed forward mount
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], -1.f);
}

TEST(ActuatorEffectivenessAirshipTest, FixedMountDifferentialYaw)
{
	setTiltRange(0.f, 0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, CollectiveModeYawAndRollUnallocated)
{
	setTiltRange();
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 0.5f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// No differential thrust: both demands are left unmet
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], 1.f);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = -1.f;
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = -0.5f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], -1.f);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], -1.f);
}

TEST(ActuatorEffectivenessAirshipTest, CollectiveModeHasNoDifferential)
{
	// Generic-airship class: fixed mounts driven by one thrust command
	setTiltRange(0.f, 0.f);
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// No differential thrust: yaw is left to the fins and reported unmet
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);

	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
}

TEST(ActuatorEffectivenessAirshipTest, CollectiveModeReverseCruise)
{
	setTiltRange();
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// Full reverse cruise through the tilt, with non-negative motors
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f); // 180 deg
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, MotorLimitRespected)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(0.f);
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);
	actuator_max(MOTOR_STARBOARD) = 0.8f;
	actuator_max(MOTOR_PORT) = 0.8f;
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);

	// The configured motor limit caps the couple; the shortfall is reported
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.8f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.8f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, PitchTorqueUnallocated)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);

	// The pods cannot produce pitch torque in any mode
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::PITCH) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[1], 1.f);

	control_sp(ActuatorEffectiveness::ControlAxis::PITCH) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[1], -1.f);
}

TEST(ActuatorEffectivenessAirshipTest, AuxThrusterConfiguration)
{
	setTiltRange();
	setAuxThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators_matrix[0], 5);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::MOTORS], 3);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 2);
}

TEST(ActuatorEffectivenessAirshipTest, AuxTailServesCollectiveYaw)
{
	setTiltRange();
	setCollectiveMode();
	setAuxThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// The pods have no differential thrust: the tail takes the whole demand
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_TAIL), 1.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, AuxTailReverseNeedsConfiguration)
{
	setTiltRange();
	setCollectiveMode();
	setAuxThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};

	// Non-reversible tail: negative demand clamps to zero and is reported
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_TAIL), 0.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], -1.f);

	// Reversible tail (CA_R_REV): full reverse authority
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(0.f);
	actuator_min(MOTOR_TAIL) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_TAIL), -1.f);

	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, AuxTailIdleWithIndependentCouple)
{
	setTiltRange();
	setAuxThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// The couple serves the demand exactly: nothing left for the tail
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(AUX_TILT_STARBOARD), 1.f); // +180 deg
	EXPECT_FLOAT_EQ(actuator_sp(AUX_TILT_PORT), 0.f);
	EXPECT_NEAR(actuator_sp(MOTOR_TAIL), 0.f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, AuxTailTopsUpClampedRange)
{
	setTiltRange(0.f, 0.f);
	setAuxThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// Fixed mounts yield half the couple differentially; the tail tops up
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_TAIL), 0.5f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, AsymmetricTiltRange)
{
	// Mirror of the Cloudship preset: collective mode, reversible tail
	// thruster, tilts from 0 deg (forward) to +90 deg (up)
	setTiltRange(0.f, 90.f);
	setCollectiveMode();
	setAuxThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(0.f);
	actuator_min(MOTOR_TAIL) = -1.f;
	actuator_min(AUX_TILT_STARBOARD) = -1.f;
	actuator_min(AUX_TILT_PORT) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);

	// Cruise: 0 deg is the range minimum, so the servos sit at -1
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(AUX_TILT_STARBOARD), -1.f);
	EXPECT_FLOAT_EQ(actuator_sp(AUX_TILT_PORT), -1.f);

	// Climb: +90 deg is the range maximum, so the servos sit at +1
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(AUX_TILT_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(AUX_TILT_PORT), 1.f);

	// Reverse cruise is unreachable: the tilt clamps at +90 deg where the
	// demand has no feasible component, and the shortfall is reported
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.f, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_TAIL), 0.f, 1e-6f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], -1.f);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}
