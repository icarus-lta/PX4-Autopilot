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

// With the tail thruster (CA_AIRSHIP_TAIL) the tilts shift by one
static constexpr int MOTOR_TAIL = 2;
static constexpr int TAIL_TILT_STARBOARD = 3;
static constexpr int TAIL_TILT_PORT = 4;

// Surfaces declared by setSurfaces() follow the tilts
static constexpr int SURFACE_ELEVATOR = 4;
static constexpr int SURFACE_RUDDER = 5;

// Collective grouping registers a single tilt servo
static constexpr int COLLECTIVE_TILT = 2;
static constexpr int TAIL_COLLECTIVE_TILT = 3;
static constexpr int TAIL_COLLECTIVE_RUDDER = 5;

static void setTiltRange(float tilt_min_deg = -180.f, float tilt_max_deg = 180.f)
{
	// Disable autosaving parameters to avoid busy loop in param_set()
	param_control_autosave(false);

	int32_t grouping = 1;
	param_set(param_find("CA_AIRSHIP_GRP"), &grouping);
	int32_t tail = 0;
	param_set(param_find("CA_AIRSHIP_TAIL"), &tail);
	param_set(param_find("CA_AIRSHIP_TLMIN"), &tilt_min_deg);
	param_set(param_find("CA_AIRSHIP_TLMAX"), &tilt_max_deg);
	int32_t surface_count = 0;
	param_set(param_find("CA_SV_CS_COUNT"), &surface_count);
	float surface_credit = 1.f;
	param_set(param_find("CA_AIRSHIP_CS_K"), &surface_credit);
}

static void setSurfaceCredit(float credit)
{
	param_set(param_find("CA_AIRSHIP_CS_K"), &credit);
}

static void setTailThruster()
{
	int32_t tail = 1;
	param_set(param_find("CA_AIRSHIP_TAIL"), &tail);
}

static void setCollectiveMode()
{
	int32_t grouping = 0;
	param_set(param_find("CA_AIRSHIP_GRP"), &grouping);
}

static void setSurfaces()
{
	int32_t surface_count = 2;
	param_set(param_find("CA_SV_CS_COUNT"), &surface_count);
	int32_t elevator = 3;
	param_set(param_find("CA_SV_CS0_TYPE"), &elevator);
	float pitch_torque = 1.f;
	param_set(param_find("CA_SV_CS0_TRQ_P"), &pitch_torque);
	int32_t rudder = 4;
	param_set(param_find("CA_SV_CS1_TYPE"), &rudder);
	float yaw_torque = 1.f;
	param_set(param_find("CA_SV_CS1_TRQ_Y"), &yaw_torque);
}

// The actuator layout is decided when the actuators are declared, so every
// updateSetpoint() needs a preceding declaration, as in the allocator
static void declareActuators(ActuatorEffectivenessAirship &airship)
{
	ActuatorEffectiveness::Configuration configuration{};
	airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

static void runUpdateSetpoint(ActuatorEffectivenessAirship &airship, const Vector<float, 6> &control_sp,
			      ActuatorEffectiveness::ActuatorVector &actuator_sp)
{
	declareActuators(airship);
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
	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), 0.f);

	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), 0.5f); // +90 deg
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
	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), 1.f); // 180 deg
}

TEST(ActuatorEffectivenessAirshipTest, MotorLimitRespected)
{
	setTiltRange();
	ActuatorEffectivenessAirship airship(nullptr);
	declareActuators(airship);

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

TEST(ActuatorEffectivenessAirshipTest, TailThrusterConfiguration)
{
	setTiltRange();
	setTailThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators_matrix[0], 5);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::MOTORS], 3);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 2);
}

TEST(ActuatorEffectivenessAirshipTest, CollectiveSingleTiltConfiguration)
{
	setTiltRange();
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	// One tilt command registers one tilt servo
	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators_matrix[0], 3);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::MOTORS], 2);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 1);

	// The tail motor shifts the tilt but does not add servos
	setTailThruster();
	ActuatorEffectivenessAirship tail_airship(nullptr);
	ActuatorEffectiveness::Configuration tail_configuration{};
	EXPECT_TRUE(tail_airship.getEffectivenessMatrix(tail_configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(tail_configuration.num_actuators_matrix[0], 4);
	EXPECT_EQ(tail_configuration.num_actuators[(int)ActuatorType::MOTORS], 3);
	EXPECT_EQ(tail_configuration.num_actuators[(int)ActuatorType::SERVOS], 1);
}

TEST(ActuatorEffectivenessAirshipTest, TailServesCollectiveYaw)
{
	setTiltRange();
	setCollectiveMode();
	setTailThruster();
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

TEST(ActuatorEffectivenessAirshipTest, TailReverseNeedsConfiguration)
{
	setTiltRange();
	setCollectiveMode();
	setTailThruster();
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

TEST(ActuatorEffectivenessAirshipTest, TailIdleWithIndependentCouple)
{
	setTiltRange();
	setTailThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// The couple serves the demand exactly: nothing left for the tail
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_TILT_STARBOARD), 1.f); // +180 deg
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_TILT_PORT), 0.f);
	EXPECT_NEAR(actuator_sp(MOTOR_TAIL), 0.f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, TailTopsUpClampedRange)
{
	setTiltRange(0.f, 0.f);
	setTailThruster();
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
	// thruster, tilt range 0 deg (forward) to +90 deg (up)
	setTiltRange(0.f, 90.f);
	setCollectiveMode();
	setTailThruster();
	ActuatorEffectivenessAirship airship(nullptr);
	declareActuators(airship);

	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(0.f);
	actuator_min(MOTOR_TAIL) = -1.f;
	actuator_min(TAIL_COLLECTIVE_TILT) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);

	// Cruise: 0 deg is the range minimum, so the tilt servo sits at -1
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), -1.f);

	// Climb: +90 deg is the range maximum, so the tilt servo sits at +1
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), 1.f);

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

TEST(ActuatorEffectivenessAirshipTest, FixedMountConfiguration)
{
	setTiltRange(0.f, 0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// A zero tilt range allocates no tilt servos
	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators_matrix[0], 2);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::MOTORS], 2);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 0);
}

TEST(ActuatorEffectivenessAirshipTest, SurfaceConfiguration)
{
	setTiltRange();
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	// The elevator and rudder follow the tilts, with matrix effectiveness
	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators_matrix[0], 6);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::MOTORS], 2);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 4);
	EXPECT_FLOAT_EQ(configuration.effectiveness_matrices[0](1, SURFACE_ELEVATOR), 1.f);
	EXPECT_FLOAT_EQ(configuration.effectiveness_matrices[0](2, SURFACE_RUDDER), 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, SurfacesKeepTheirSetpoints)
{
	setTiltRange();
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_ELEVATOR) = 0.7f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// The matrix allocation of the surfaces is left untouched
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(SURFACE_ELEVATOR), 0.7f);
}

TEST(ActuatorEffectivenessAirshipTest, SurfacesServeFirst)
{
	setTiltRange();
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	// The rudder already carries 0.6 of the yaw demand from the matrix
	// pass; the pods serve the remaining 0.4 and the report is exact
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_RUDDER) = 0.6f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.4f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.4f);
	EXPECT_FLOAT_EQ(actuator_sp(SURFACE_RUDDER), 0.6f);

	control_allocator_status_s status{};
	status.unallocated_torque[2] = 0.4f; // matrix residual: demand minus rudder
	airship.getUnallocatedControl(0, status);
	EXPECT_NEAR(status.unallocated_torque[2], 0.f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, SurfaceCreditZeroPodsServeAll)
{
	setTiltRange();
	setSurfaces();
	setSurfaceCredit(0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// The rudder still deflects with its matrix allocation, but with no
	// credit the pods serve the whole demand: full yaw couple as in hover
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_RUDDER) = 0.6f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f); // +180 deg
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);      // forward
	EXPECT_FLOAT_EQ(actuator_sp(SURFACE_RUDDER), 0.6f);

	// The uncredited rudder share cancels against the solver residual:
	// the pods served everything, so nothing is unallocated
	control_allocator_status_s status{};
	status.unallocated_torque[2] = 0.4f; // matrix residual: demand minus rudder
	airship.getUnallocatedControl(0, status);
	EXPECT_NEAR(status.unallocated_torque[2], 0.f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, SurfaceCreditSharesProportionally)
{
	setTiltRange();
	setSurfaces();
	setSurfaceCredit(0.5f);
	ActuatorEffectivenessAirship airship(nullptr);

	// Half credit: of the rudder's 0.6 allocation only 0.3 is trusted,
	// so the pods serve the remaining 0.7 of the demand
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_RUDDER) = 0.6f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.7f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.7f);
	EXPECT_FLOAT_EQ(actuator_sp(SURFACE_RUDDER), 0.6f);

	control_allocator_status_s status{};
	status.unallocated_torque[2] = 0.4f; // matrix residual: demand minus rudder
	airship.getUnallocatedControl(0, status);
	EXPECT_NEAR(status.unallocated_torque[2], 0.f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, SurfaceCreditZeroReportsHoverPitch)
{
	setTiltRange();
	setSurfaces();
	setSurfaceCredit(0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::PITCH) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_ELEVATOR) = 0.75f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	// With no credit, the elevator's share is honestly reported on top
	// of the matrix residual: the whole pitch demand is unallocated
	control_allocator_status_s status{};
	status.unallocated_torque[1] = 0.25f; // matrix residual: demand minus elevator
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[1], 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, PitchDeferredToSurfaces)
{
	setTiltRange();
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::PITCH) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	// With an elevator, the matrix residual stands instead of the flag
	control_allocator_status_s status{};
	status.unallocated_torque[1] = 0.25f;
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[1], 0.25f);
}

TEST(ActuatorEffectivenessAirshipTest, TailAfterSurfaces)
{
	setTiltRange();
	setCollectiveMode();
	setTailThruster();
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	// The rudder carries 0.6 of the yaw demand and the collective pods
	// none: the tail serves only the remaining 0.4
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(TAIL_COLLECTIVE_RUDDER) = 0.6f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_TAIL), 0.4f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_RUDDER), 0.6f);

	control_allocator_status_s status{};
	status.unallocated_torque[2] = 0.4f; // matrix residual: demand minus rudder
	airship.getUnallocatedControl(0, status);
	EXPECT_NEAR(status.unallocated_torque[2], 0.f, 1e-6f);
}
