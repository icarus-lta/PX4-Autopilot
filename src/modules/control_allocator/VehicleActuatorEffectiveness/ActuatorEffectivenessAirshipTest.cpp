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
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_status.h>

using namespace matrix;

// Actuator indices
static constexpr int MOTOR_STARBOARD = ActuatorEffectivenessAirship::STARBOARD;
static constexpr int MOTOR_PORT = ActuatorEffectivenessAirship::PORT;
static constexpr int TILT_STARBOARD = 2;
static constexpr int TILT_PORT = 3;

// With the tail thruster (CA_AIRSHIP_TAIL) the tilts shift by one
static constexpr int MOTOR_TAIL = ActuatorEffectivenessAirship::TAIL;
static constexpr int TAIL_TILT_STARBOARD = 3;
static constexpr int TAIL_TILT_PORT = 4;

// Surfaces precede the tilts (_first_control_surface_idx); setSurfaces()
// declares an elevator then a rudder
static constexpr int SURFACE_FIRST = 2;
static constexpr int SURFACE_ELEVATOR = SURFACE_FIRST;
static constexpr int SURFACE_RUDDER = SURFACE_FIRST + 1;

// The tilts follow the surfaces, so their index counts the declared ones:
// two for setSurfaces(), one for the single-aileron configurations
static constexpr int SURFACES_TILT_STARBOARD = SURFACE_FIRST + 2;
static constexpr int SURFACES_TILT_PORT = SURFACE_FIRST + 3;
static constexpr int AILERON_TILT_STARBOARD = SURFACE_FIRST + 1;
static constexpr int AILERON_TILT_PORT = SURFACE_FIRST + 2;

// Collective grouping registers a single tilt servo
static constexpr int COLLECTIVE_TILT = 2;
static constexpr int TAIL_COLLECTIVE_TILT = 3;
static constexpr int TAIL_COLLECTIVE_RUDDER = 4;

// The allocator's dt ceiling: the largest step it hands the effectiveness [s]
static constexpr float kDt = 0.02f;

// The armed state is a persistent uORB sample the effectiveness reads on
// its next update; the publication lives for the whole run so the topic
// stays advertised (a subscriber only copies from an advertised topic)
static void publishArmed(bool armed)
{
	static uORB::Publication<vehicle_status_s> pub{ORB_ID(vehicle_status)};
	vehicle_status_s vehicle_status{};
	vehicle_status.timestamp = hrt_absolute_time();
	vehicle_status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
	pub.publish(vehicle_status);
}

static void resetAirshipParams(float tilt_min_deg = -180.f, float tilt_max_deg = 180.f)
{
	// Armed unless a test disarms: the tilts park until vehicle_status
	// reports armed
	publishArmed(true);

	// Disable autosaving parameters to avoid busy loop in param_set()
	param_control_autosave(false);

	// Start every test from the parameter defaults so nothing leaks between
	// tests: independent grouping over the full tilt range, no tail, no
	// surfaces, full surface credit, no tilt slew
	param_reset_all();

	int32_t grouping = 1;
	param_set(param_find("CA_AIRSHIP_GRP"), &grouping);
	param_set(param_find("CA_AIRSHIP_TLMIN"), &tilt_min_deg);
	param_set(param_find("CA_AIRSHIP_TLMAX"), &tilt_max_deg);
}

static void setSurfaceCredit(float credit)
{
	param_set(param_find("CA_AIRSHIP_CS_K"), &credit);
}

static void setTiltRate(float rate_deg_s)
{
	param_set(param_find("CA_AIRSHIP_TLT_R"), &rate_deg_s);
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

// Disarm for the scope of a test and restore the armed baseline on exit,
// even on an early failure
struct ScopedDisarm {
	ScopedDisarm() { publishArmed(false); }
	~ScopedDisarm() { publishArmed(true); }
};

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

	// Production limits: motors are non-reversible (no CA_R_REV) at
	// [0, 1], servos span the full [-1, 1]
	int32_t tail = 0;
	param_get(param_find("CA_AIRSHIP_TAIL"), &tail);
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(-1.f);

	for (int i = 0; i < (tail ? 3 : 2); i++) {
		actuator_min(i) = 0.f;
	}

	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);
	airship.allocateAuxilaryControls(kDt, 0, actuator_sp);
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
}

TEST(ActuatorEffectivenessAirshipTest, VectoredConfiguration)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators_matrix[0], 4);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::MOTORS], 2);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 2);
}

TEST(ActuatorEffectivenessAirshipTest, ForwardCruise)
{
	resetAirshipParams();
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
	resetAirshipParams();
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
	resetAirshipParams();
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
	resetAirshipParams();
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
	resetAirshipParams();
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
	resetAirshipParams();
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
	resetAirshipParams();
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

				SCOPED_TRACE(::testing::Message() << "thrust=" << thrust << " yaw=" << yaw << " roll=" << roll);
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
	resetAirshipParams(-90.f, 90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	// Reverse is unreachable at either end: the starboard tilt stays level
	// instead of sweeping to a limit that realizes nothing, and the motor
	// clamps off
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);      // 0 deg = range center
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f); // half the couple is missing
}

TEST(ActuatorEffectivenessAirshipTest, NoReverseEndHoldsTheTilt)
{
	// Symmetric -90..+90 deg range (the Cloudship geometry): straight back
	// is realizable at neither end, so the cone must not commit the tilt
	// to a 90 deg excursion that produces nothing and modulates the forward
	// thrust on the way back
	resetAirshipParams(-90.f, 90.f);
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), 0.f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	// The shortfall is real and reported: nothing was held by choice
	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], -1.f);
}

TEST(ActuatorEffectivenessAirshipTest, CollectiveModeCruiseAndClimb)
{
	resetAirshipParams();
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
	resetAirshipParams(0.f, 0.f);
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

	// The mirror: a downward demand is equally unreachable
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, FixedMountDifferentialYaw)
{
	resetAirshipParams(0.f, 0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, FixedMountAtAngleServesVertical)
{
	// TLMIN = TLMAX = 90 is outside the declared parameter ranges, but
	// nothing prevents it in storage: the empty range must degrade to a
	// fixed mount at that angle, projecting the demand onto it
	resetAirshipParams(90.f, 90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 0);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	// Forward demand has no component along the up-fixed mount
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, InvertedRangeIsAFixedMountAtTheMinimum)
{
	// TLMIN > TLMAX is outside the declared parameter ranges, but nothing
	// prevents it in storage: it declares no tilt servo, so the pods must
	// sit still at TLMIN rather than flip between the two limits every
	// update
	resetAirshipParams(30.f, -30.f);
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 0);

	// A down demand has no component along the 30 deg mount: motors off,
	// and identically so on the next update
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 0.5f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};

	for (int update = 0; update < 3; update++) {
		SCOPED_TRACE(::testing::Message() << "update=" << update);
		runUpdateSetpoint(airship, control_sp, actuator_sp);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	}

	// Forward demand projects onto the 30 deg mount
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f * cosf(math::radians(30.f)));
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f * cosf(math::radians(30.f)));
}

TEST(ActuatorEffectivenessAirshipTest, CollectiveModeYawAndRollUnallocated)
{
	resetAirshipParams();
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
	resetAirshipParams(0.f, 0.f);
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
	resetAirshipParams();
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
	resetAirshipParams();
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
	resetAirshipParams();
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

TEST(ActuatorEffectivenessAirshipTest, LateralThrustReportedUnserved)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// No airship actuator produces lateral force: the demand must be
	// reported unserved, not silently read as allocated
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Y) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[1], 1.f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Y) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[1], -1.f);
}

TEST(ActuatorEffectivenessAirshipTest, TailThrusterConfiguration)
{
	resetAirshipParams();
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
	resetAirshipParams();
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
	resetAirshipParams();
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
	resetAirshipParams();
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
	resetAirshipParams();
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

TEST(ActuatorEffectivenessAirshipTest, TailTopsUpFixedMountYaw)
{
	resetAirshipParams(0.f, 0.f);
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
	// An up-only tilt range: collective mode, reversible tail thruster,
	// 0 deg (forward) to +90 deg (up)
	resetAirshipParams(0.f, 90.f);
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

	// Descent is unreachable: neither range end realizes a downward demand,
	// so the tilt stays at the 0 deg floor, the motors project to zero and
	// the shortfall is reported
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 1.f;
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), -1.f);
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.f, 1e-6f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], 1.f);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 0.f);

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

	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], -1.f);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, FixedMountConfiguration)
{
	resetAirshipParams(0.f, 0.f);
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
	resetAirshipParams();
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	// The elevator and rudder precede the tilts, with matrix effectiveness
	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators_matrix[0], 6);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::MOTORS], 2);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 4);
	EXPECT_FLOAT_EQ(configuration.effectiveness_matrices[0](1, SURFACE_ELEVATOR), 1.f);
	EXPECT_FLOAT_EQ(configuration.effectiveness_matrices[0](2, SURFACE_RUDDER), 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, SurfaceIndicesDoNotMoveWithTheTiltCount)
{
	// The tilt count follows the tilt range, which the ground station's
	// actuator list cannot express. Declared after the surfaces it can only
	// add entries behind them, never shift them
	for (const float tilt_deg : {180.f, 0.f}) {
		resetAirshipParams(-tilt_deg, tilt_deg);
		setSurfaces();
		ActuatorEffectivenessAirship airship(nullptr);

		ActuatorEffectiveness::Configuration configuration{};
		EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
		EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], tilt_deg > 0.f ? 4 : 2);
		EXPECT_FLOAT_EQ(configuration.effectiveness_matrices[0](1, SURFACE_ELEVATOR), 1.f);
		EXPECT_FLOAT_EQ(configuration.effectiveness_matrices[0](2, SURFACE_RUDDER), 1.f);
	}
}

TEST(ActuatorEffectivenessAirshipTest, SurfacesKeepTheirSetpoints)
{
	resetAirshipParams();
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

TEST(ActuatorEffectivenessAirshipTest, SurfaceCreditScalesPodShare)
{
	resetAirshipParams();
	setSurfaces();

	// The rudder carries 0.6 of the yaw demand from the matrix pass but is
	// credited only by CA_AIRSHIP_CS_K: the pods serve the rest, reversing
	// the starboard pod, and the report is exact in every case
	struct {
		float credit;
		float motor;
	} cases[] = {{1.f, 0.4f}, {0.5f, 0.7f}, {0.f, 1.f}};

	for (const auto &c : cases) {
		SCOPED_TRACE(::testing::Message() << "credit=" << c.credit);
		setSurfaceCredit(c.credit);
		ActuatorEffectivenessAirship airship(nullptr);

		Vector<float, 6> control_sp{};
		control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
		ActuatorEffectiveness::ActuatorVector actuator_sp{};
		actuator_sp(SURFACE_RUDDER) = 0.6f;
		runUpdateSetpoint(airship, control_sp, actuator_sp);

		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), c.motor);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), c.motor);
		EXPECT_FLOAT_EQ(actuator_sp(SURFACES_TILT_STARBOARD), 1.f); // +180 deg
		EXPECT_FLOAT_EQ(actuator_sp(SURFACES_TILT_PORT), 0.f);      // forward
		EXPECT_FLOAT_EQ(actuator_sp(SURFACE_RUDDER), 0.6f); // the matrix allocation stands

		// The uncredited rudder share cancels against the solver residual:
		// the pods served everything, so nothing is unallocated
		control_allocator_status_s status{};
		status.unallocated_torque[2] = 0.4f; // matrix residual: demand minus rudder
		airship.getUnallocatedControl(0, status);
		EXPECT_NEAR(status.unallocated_torque[2], 0.f, 1e-6f);
	}
}

TEST(ActuatorEffectivenessAirshipTest, RollSurfaceCreditedAgainstDifferentialRoll)
{
	resetAirshipParams();
	int32_t surface_count = 1;
	param_set(param_find("CA_SV_CS_COUNT"), &surface_count);
	int32_t aileron = 1;
	param_set(param_find("CA_SV_CS0_TYPE"), &aileron);
	float roll_torque = 1.f;
	param_set(param_find("CA_SV_CS0_TRQ_R"), &roll_torque);
	ActuatorEffectivenessAirship airship(nullptr);

	// The aileron carries 0.6 of the roll demand from the matrix pass;
	// the pods serve the remaining 0.4 differentially, straight up and
	// down, and the report is exact
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_FIRST) = 0.6f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.4f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.4f);
	EXPECT_FLOAT_EQ(actuator_sp(AILERON_TILT_STARBOARD), -0.5f); // -90 deg, down
	EXPECT_FLOAT_EQ(actuator_sp(AILERON_TILT_PORT), 0.5f);       // +90 deg, up

	control_allocator_status_s status{};
	status.unallocated_torque[0] = 0.4f; // matrix residual: demand minus aileron
	airship.getUnallocatedControl(0, status);
	EXPECT_NEAR(status.unallocated_torque[0], 0.f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, RollSteerBandShortfallIsNotSaturation)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// Level pods deliver none of a roll demand below the steer floor: the
	// unmet demand is the band's own choice, not saturation
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 0.75f * AirshipPod::kSteerEngage;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], 0.f);

	// Over the floor the pods vector straight down and up and serve it exactly
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 2.f * AirshipPod::kSteerEngage;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), -0.5f); // -90 deg, down
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.5f);       // +90 deg, up
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 2.f * AirshipPod::kSteerEngage, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 2.f * AirshipPod::kSteerEngage, 1e-6f);
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, RollSurfaceServedBandShortfallIsNotSaturation)
{
	resetAirshipParams();
	int32_t surface_count = 1;
	param_set(param_find("CA_SV_CS_COUNT"), &surface_count);
	int32_t aileron = 1;
	param_set(param_find("CA_SV_CS0_TYPE"), &aileron);
	float roll_torque = 1.f;
	param_set(param_find("CA_SV_CS0_TRQ_R"), &roll_torque);
	setSurfaceCredit(0.5f);
	ActuatorEffectivenessAirship airship(nullptr);

	// An aileron serves roll, so the report takes the surface branch. Half
	// the aileron's allocation is credited, leaving half the demand for the
	// pods, which is inside the steer band: they hold level and deliver none
	Vector<float, 6> control_sp{};
	const float demand = AirshipPod::kSteerEngage;
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = demand;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_FIRST) = demand; // the matrix allocates the full demand to the unit aileron
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(AILERON_TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(AILERON_TILT_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	// The held half is the band's choice on this path too
	control_allocator_status_s status{};
	status.unallocated_torque[0] = 0.f; // matrix residual: demand minus aileron
	airship.getUnallocatedControl(0, status);
	EXPECT_NEAR(status.unallocated_torque[0], 0.f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, SurfaceCreditZeroReportsFullPitchDemand)
{
	resetAirshipParams();
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
	resetAirshipParams();
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
	resetAirshipParams();
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

TEST(ActuatorEffectivenessAirshipTest, SurfaceTrimNotCredited)
{
	resetAirshipParams();
	setSurfaces();
	float trim = 0.2f;
	param_set(param_find("CA_SV_CS1_TRIM"), &trim);
	ActuatorEffectivenessAirship airship(nullptr);

	// The rudder sits exactly at its trim: it delivers no torque, so the
	// pods must serve the whole demand and the report must not drift
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_RUDDER) = 0.2f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	control_allocator_status_s status{};
	status.unallocated_torque[2] = 1.f; // matrix residual: demand minus (rudder - trim)
	airship.getUnallocatedControl(0, status);
	EXPECT_NEAR(status.unallocated_torque[2], 0.f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, SaturatedSurfaceNotOverCredited)
{
	resetAirshipParams();
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	// The matrix over-allocated the rudder to 1.5, but only the clipped
	// 1.0 can ever be delivered: crediting the excess would push a
	// phantom negative demand into the pods
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_RUDDER) = 1.5f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(actuator_sp(SURFACE_RUDDER), 1.5f); // clipping stays the allocator's job
}

TEST(ActuatorEffectivenessAirshipTest, DemandAtReleaseThresholdDoesNotHoldSteering)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// Establish a steered tilt with a full yaw couple
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);

	// A demand exactly at the release threshold releases (the hold is a
	// strict comparison): the tilts stay where they are, in either sign
	for (const float sign : {1.f, -1.f}) {
		control_sp(ActuatorEffectiveness::ControlAxis::YAW) = sign * AirshipPod::kSteerRelease;
		runUpdateSetpoint(airship, control_sp, actuator_sp);
		EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
		EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
	}
}

TEST(ActuatorEffectivenessAirshipTest, SteerHysteresisBand)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// Engage steering above the engage threshold
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	// Inside the band the last commanded direction stands: a reversed
	// demand below the engage floor must not retarget the tilt
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) =
		-0.5f * (AirshipPod::kSteerEngage + AirshipPod::kSteerRelease);
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	// Below the release threshold the hold takes over
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.5f * AirshipPod::kSteerRelease;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	// A reversal above the engage floor is a real demand and retargets
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, NanDemandDoesNotCorruptTiltState)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// Commit the starboard tilt to the +180 deg end
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	// A NaN sample fails every steering comparison and must fall into the
	// hold branch, leaving the persistent tilt state finite
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = NAN;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_TRUE(PX4_ISFINITE(actuator_sp(TILT_STARBOARD)));

	// The original demand behaves as if the NaN never happened
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, RearDemandNoiseKeepsChosenEnd)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// A straight-back demand commits both tilts to the +180 deg end
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);

	// Perpendicular noise flips the sign of the small fz component: the
	// committed end must hold, not swing across the whole range
	for (int step = -2; step <= 2; step++) {
		control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 0.05f * AirshipPod::kEndSwitchMargin * step;
		runUpdateSetpoint(airship, control_sp, actuator_sp);
		EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
		EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 1.f);
	}
}

TEST(ActuatorEffectivenessAirshipTest, SlewKeepsCommittedEnd)
{
	resetAirshipParams();
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// Commit to the +180 deg end; the first slew step moves 1.8 deg (90 deg/s x kDt)
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	const float first = actuator_sp(TILT_STARBOARD);
	EXPECT_GT(first, 0.f);

	// Noise on the perpendicular axis while the servo is still slewing
	// must not re-decide the end: the tilt keeps moving the same way
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 0.1f * AirshipPod::kEndSwitchMargin;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_GT(actuator_sp(TILT_STARBOARD), first);
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = -0.1f * AirshipPod::kEndSwitchMargin;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_GT(actuator_sp(TILT_STARBOARD), first);
}

TEST(ActuatorEffectivenessAirshipTest, TiltServoLimitBoundsProjection)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);
	declareActuators(airship);

	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(-1.f);
	actuator_min(MOTOR_STARBOARD) = 0.f;
	actuator_min(MOTOR_PORT) = 0.f;
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);
	actuator_max(TILT_STARBOARD) = 0.5f; // output limit: +90 deg at most

	// Full reverse asks for +180 deg; the limited starboard servo stops
	// at +90 deg and the projection must use that realized angle
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);

	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.5f);
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, CloudshipMirrorSymmetricTilt)
{
	// Mirror of the Cloudship preset's tilt geometry: collective mode, tail
	// thruster, symmetric tilt range -90..+90 deg (level = servo 0). The
	// preset's reversible tail (CA_R_REV) is exercised by
	// TailReverseNeedsConfiguration; no yaw is demanded here
	resetAirshipParams(-90.f, 90.f);
	setCollectiveMode();
	setTailThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	// Cruise: level tilt sits at the servo center, as the old mixer did
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), 0.f);

	// Climb: +90 deg is the range maximum
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), 1.f);

	// Descent: the symmetric range restores downward vectoring
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), -1.f);
}

TEST(ActuatorEffectivenessAirshipTest, RearAnchorPicksReachableEnd)
{
	// Down-only range: straight back is realizable only at -180 deg,
	// so the anchor must not commit to the unreachable +180 end
	resetAirshipParams(-180.f, 0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), -1.f); // -180 deg
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), -1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, TiltRangeNarrowedAtRuntimeReclampsHeldTilt)
{
	resetAirshipParams();

	// Drive the parameter update the way the allocator does: a notification on
	// the parent cascades to every ModuleParams child
	struct ParamOwner : ModuleParams {
		ParamOwner() : ModuleParams(nullptr) {}
		using ModuleParams::updateParams;
	} owner;
	ActuatorEffectivenessAirship airship{&owner};

	// Commit to the +180 deg end with the full default range
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	// Collapse the range to a forward fixed mount mid-flight: the held
	// 180 deg state must be pulled into the new range, so a demand below
	// the steering threshold projects onto 0 deg, not onto a stale angle
	// no servo write-back can correct (no tilt servo exists any more)
	float zero = 0.f;
	param_set(param_find("CA_AIRSHIP_TLMIN"), &zero);
	param_set(param_find("CA_AIRSHIP_TLMAX"), &zero);
	owner.updateParams();

	const float below_release = 0.5f * AirshipPod::kSteerRelease;
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = below_release;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), below_release);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), below_release);
}

TEST(ActuatorEffectivenessAirshipTest, RestrictedRangeRearDemandPicksRealizingEnd)
{
	// Down-only range with a back-and-up demand OUTSIDE the rear cone:
	// the atan2 target (+169 deg) is out of range, and the numerically
	// nearer bound (0 deg, forward) realizes none of it. The -180 end
	// must be chosen: full reverse thrust, only the up share unmet.
	resetAirshipParams(-180.f, 0.f);
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -0.2f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), -1.f); // -180 deg
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 0.f);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], -1.f); // the up share is unreachable
}

TEST(ActuatorEffectivenessAirshipTest, RearConeBoundaryKeepsCommittedEnd)
{
	// Crossing the rear-cone boundary must not change the committed end:
	// inside the cone the anchor holds it, and outside the out-of-range
	// end selection must agree, so the boundary itself cannot retarget
	resetAirshipParams(-180.f, 0.f);
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	// Commit straight back to the only realizing end, -180 deg
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), -1.f);

	// Step the perpendicular component from inside the cone to outside
	for (int step = 0; step <= 3; step++) {
		SCOPED_TRACE(::testing::Message() << "step=" << step);
		control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) =
			-0.4f * AirshipPod::kEndSwitchMargin * step;
		runUpdateSetpoint(airship, control_sp, actuator_sp);
		EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), -1.f);
	}
}

TEST(ActuatorEffectivenessAirshipTest, NegativeEndCommitmentHeld)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// A down-and-back demand commits the starboard tilt to the negative
	// side of the range
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 0.2f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_LT(actuator_sp(TILT_STARBOARD), -0.9f); // ~ -169 deg

	// Straight back inside the cone must hold the -180 end, whichever
	// way the perpendicular noise points
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 0.1f * AirshipPod::kEndSwitchMargin;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), -1.f);
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = -0.1f * AirshipPod::kEndSwitchMargin;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), -1.f);
}

TEST(ActuatorEffectivenessAirshipTest, CollectiveClampedServoRealizedCopy)
{
	resetAirshipParams();
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);
	declareActuators(airship);

	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(-1.f);
	actuator_min(MOTOR_STARBOARD) = 0.f;
	actuator_min(MOTOR_PORT) = 0.f;
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);
	actuator_max(COLLECTIVE_TILT) = 0.5f; // +90 deg reachable at most

	// Full reverse asks for +180 deg; the clamped collective servo stops
	// at +90 deg and BOTH pods must project onto that realized angle
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);

	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), 0.5f);
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, TiltRateLimited)
{
	resetAirshipParams();
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// A full vertical demand asks for +90 deg, but each update moves the
	// tilt by 90 deg/s x kDt = 1.8 deg, 0.01 of the 360 deg servo span
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_NEAR(actuator_sp(TILT_STARBOARD), 0.01f, 1e-6f);

	// The tilt keeps slewing toward the target on the next update
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_NEAR(actuator_sp(TILT_STARBOARD), 0.02f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, DisarmParksTiltLevel)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// Armed (resetAirshipParams publishes it): full yaw couple
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	// Disarmed, the tilt parks level regardless of the demand and the
	// reverse projection clamps the motor off
	ScopedDisarm disarmed;

	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f); // parked at 0 deg, forward
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, SteerBandShortfallIsNotSaturation)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// Level pods at hover: a yaw demand below the steer floor is served by
	// the port pod alone, the starboard pod would need the reversal the band
	// deliberately withholds
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.75f * AirshipPod::kSteerEngage;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.75f * AirshipPod::kSteerEngage, 1e-6f);

	// The unmet half is the band's own choice, not saturation: the rate
	// controller's integrator must stay free to lift the demand over the floor
	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 0.f);

	// Over the floor the steering engages and the exact couple is served
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 2.f * AirshipPod::kSteerEngage;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, SteerBandVerticalShortfallIsNotSaturation)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// Level pods with a vertical demand below the steer floor, in both
	// directions: the pods hold level and deliver none of it, and the held
	// share is the band's choice on the thrust axis as on the torque axes
	for (const float sign : {-1.f, 1.f}) {
		SCOPED_TRACE(::testing::Message() << "sign=" << sign);
		Vector<float, 6> control_sp{};
		control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = sign * 0.75f *
				AirshipPod::kSteerEngage;
		ActuatorEffectiveness::ActuatorVector actuator_sp{};
		runUpdateSetpoint(airship, control_sp, actuator_sp);
		EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
		EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

		control_allocator_status_s status{};
		airship.getUnallocatedControl(0, status);
		EXPECT_FLOAT_EQ(status.unallocated_thrust[2], 0.f);
		EXPECT_FLOAT_EQ(status.unallocated_torque[0], 0.f);
	}
}

TEST(ActuatorEffectivenessAirshipTest, SwingShortfallIsSaturation)
{
	resetAirshipParams();
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// A full couple from level pods: the starboard tilt is steering toward
	// the rear but moves 1.8 deg per update, so the couple is not
	// delivered yet. That shortfall is transient saturation the rate
	// controller must not integrate against
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_GT(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_LT(actuator_sp(TILT_STARBOARD), 0.2f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, HysteresisBandReversalIsNotSaturation)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// Commit the starboard pod to the rear with a full couple
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	// A reversed demand inside the hysteresis band keeps the committed
	// directions, which now project both pods away: motors off, no torque.
	// That is the band holding by choice, not saturation
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) =
		-0.5f * (AirshipPod::kSteerEngage + AirshipPod::kSteerRelease);
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, SwingContinuedInBandIsSaturation)
{
	resetAirshipParams();
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// Start the starboard reversal with a full couple
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	const float first = actuator_sp(TILT_STARBOARD);
	EXPECT_GT(first, 0.f);

	// The demand drops into the band: the target stands and the tilt keeps
	// swinging toward it, so the couple is still not delivered. A moving
	// tilt is a real, transient shortfall
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) =
		0.5f * (AirshipPod::kSteerEngage + AirshipPod::kSteerRelease);
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_GT(actuator_sp(TILT_STARBOARD), first);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, FixedMountInBandShortfallStands)
{
	resetAirshipParams(0.f, 0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// Fixed forward pods cannot reverse at all: the half the starboard motor
	// cannot deliver is structural even below the steer floor
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.75f * AirshipPod::kSteerEngage;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, OnePodHeldInBandIsNotSaturation)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	// Cruise with a yaw demand just above the forward thrust: the port pod
	// steers and delivers, the starboard pod's tiny reverse demand sits in
	// the band and holds level. The starboard shortfall is the band's choice
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.3f;
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.3f + 0.5f * AirshipPod::kSteerEngage;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.6f + 0.5f * AirshipPod::kSteerEngage, 1e-6f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 0.f);
}

TEST(ActuatorEffectivenessAirshipTest, DisarmedShortfallStillReported)
{
	resetAirshipParams();
	ActuatorEffectivenessAirship airship(nullptr);

	ScopedDisarm disarmed;

	// Parked tilts are not the steer band's choice: a disarmed demand the
	// parked pods cannot realize is still reported honestly
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, CollectiveYawShortfallStandsInSteerBand)
{
	resetAirshipParams();
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	// Collective pods cannot yaw at all: a small demand below the steer
	// floor is structurally unserved and must still be reported
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.5f * AirshipPod::kSteerEngage;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	runUpdateSetpoint(airship, control_sp, actuator_sp);

	control_allocator_status_s status{};
	airship.getUnallocatedControl(0, status);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST(ActuatorEffectivenessAirshipTest, SurfaceServedBandShortfallIsNotSaturation)
{
	resetAirshipParams();
	setSurfaces();
	setSurfaceCredit(0.5f);
	ActuatorEffectivenessAirship airship(nullptr);

	// A rudder serves yaw, so the report takes the surface branch. Half the
	// rudder's allocation is credited, leaving half the demand for the pods,
	// which is inside the steer band: they hold level and the starboard
	// motor clips, so only half of that arrives
	Vector<float, 6> control_sp{};
	const float demand = AirshipPod::kSteerEngage;
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = demand;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_RUDDER) = demand; // the matrix allocates the full demand to the unit rudder
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_FLOAT_EQ(actuator_sp(SURFACES_TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.5f * demand, 1e-6f);

	// The held half is the band's choice on this path too
	control_allocator_status_s status{};
	status.unallocated_torque[2] = 0.f; // matrix residual: demand minus rudder
	airship.getUnallocatedControl(0, status);
	EXPECT_NEAR(status.unallocated_torque[2], 0.f, 1e-6f);
}

TEST(ActuatorEffectivenessAirshipTest, SurfaceServedSwingShortfallStands)
{
	resetAirshipParams();
	setSurfaces();
	setSurfaceCredit(0.5f);
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// Same path, but now the pod share is far above the band: the starboard
	// tilt is steering toward the rear and has not arrived, so the couple is
	// genuinely missing and must still be reported
	Vector<float, 6> control_sp{};
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	actuator_sp(SURFACE_RUDDER) = 0.6f;
	runUpdateSetpoint(airship, control_sp, actuator_sp);
	EXPECT_GT(actuator_sp(SURFACES_TILT_STARBOARD), 0.f);
	EXPECT_LT(actuator_sp(SURFACES_TILT_STARBOARD), 0.2f);

	control_allocator_status_s status{};
	status.unallocated_torque[2] = 0.4f; // matrix residual: demand minus rudder
	airship.getUnallocatedControl(0, status);
	EXPECT_GT(status.unallocated_torque[2], 0.1f);
}
