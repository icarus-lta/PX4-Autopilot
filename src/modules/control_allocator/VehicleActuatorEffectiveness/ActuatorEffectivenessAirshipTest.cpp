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

// The declared layout is the motors, then the control surfaces, then the tilt
// servos (ActuatorEffectivenessAirship::getEffectivenessMatrix). Only the
// motors sit at a fixed index; every later block moves with the counts ahead
// of it, so name each block by the rule rather than by a number. These are an
// INDEPENDENT model of the layout, pinned against the real Configuration by
// DeclaredLayoutMatchesTheIndexMap
static constexpr int numMotors(bool tail) { return tail ? 3 : 2; }
static constexpr int firstSurface(bool tail) { return numMotors(tail); }
static constexpr int firstTilt(bool tail, int num_surfaces) { return firstSurface(tail) + num_surfaces; }

static constexpr int MOTOR_STARBOARD = ActuatorEffectivenessAirship::STARBOARD;
static constexpr int MOTOR_PORT = ActuatorEffectivenessAirship::PORT;
static constexpr int MOTOR_TAIL = ActuatorEffectivenessAirship::TAIL;

// Independent grouping, no surfaces
static constexpr int TILT_STARBOARD = firstTilt(false, 0);
static constexpr int TILT_PORT = firstTilt(false, 0) + 1;
static constexpr int TAIL_TILT_STARBOARD = firstTilt(true, 0);
static constexpr int TAIL_TILT_PORT = firstTilt(true, 0) + 1;

// setSurfaces() declares an elevator then a rudder, ahead of the tilts
static constexpr int SURFACE_FIRST = firstSurface(false);
static constexpr int SURFACE_ELEVATOR = SURFACE_FIRST;
static constexpr int SURFACE_RUDDER = SURFACE_FIRST + 1;
static constexpr int SURFACES_TILT_STARBOARD = firstTilt(false, 2);
static constexpr int SURFACES_TILT_PORT = firstTilt(false, 2) + 1;

// setAileron() declares one surface
static constexpr int AILERON_TILT_STARBOARD = firstTilt(false, 1);
static constexpr int AILERON_TILT_PORT = firstTilt(false, 1) + 1;

// Collective grouping registers a single tilt servo
static constexpr int COLLECTIVE_TILT = firstTilt(false, 0);
static constexpr int TAIL_COLLECTIVE_TILT = firstTilt(true, 0);
static constexpr int TAIL_COLLECTIVE_RUDDER = firstSurface(true) + 1;

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

static void setAileron()	// one roll surface, unit effectiveness
{
	int32_t surface_count = 1;
	param_set(param_find("CA_SV_CS_COUNT"), &surface_count);
	int32_t aileron = 1;
	param_set(param_find("CA_SV_CS0_TYPE"), &aileron);
	float roll_torque = 1.f;
	param_set(param_find("CA_SV_CS0_TRQ_R"), &roll_torque);
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

static void setElevatorWithWeakYaw()	// one elevator whose yaw coupling the allocator drops as a weak row
{
	int32_t surface_count = 1;
	param_set(param_find("CA_SV_CS_COUNT"), &surface_count);
	int32_t elevator = 3;
	param_set(param_find("CA_SV_CS0_TYPE"), &elevator);
	float pitch_torque = 1.f;
	param_set(param_find("CA_SV_CS0_TRQ_P"), &pitch_torque);
	float yaw_torque = 0.03f;	// at most kMinEffectiveness: the allocator zeroes the yaw row
	param_set(param_find("CA_SV_CS0_TRQ_Y"), &yaw_torque);
}

static void setNoTorqueSurface(int32_t type)	// one surface of a type that takes no torque, with a yaw entry set anyway
{
	int32_t surface_count = 1;
	param_set(param_find("CA_SV_CS_COUNT"), &surface_count);
	param_set(param_find("CA_SV_CS0_TYPE"), &type);
	float yaw_torque = 1.f;
	param_set(param_find("CA_SV_CS0_TRQ_Y"), &yaw_torque);
}

// Disarm for the scope of a test and restore the armed baseline on exit,
// even on an early failure
struct ScopedDisarm {
	ScopedDisarm() { publishArmed(false); }
	~ScopedDisarm() { publishArmed(true); }
};

// The actuator layout is decided when the actuators are declared, so every
// updateSetpoint() needs a preceding declaration, as in the allocator
static ActuatorEffectiveness::Configuration declareActuators(ActuatorEffectivenessAirship &airship)
{
	ActuatorEffectiveness::Configuration configuration{};
	airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::CONFIGURATION_UPDATE);
	return configuration;
}

// The allocator's output limits for the declared layout: motors are
// non-reversible (no CA_R_REV) at [0, 1], servos span the full [-1, 1]. A
// test that needs one entry different starts from these and overrides it
static void productionLimits(ActuatorEffectivenessAirship &airship, ActuatorEffectiveness::ActuatorVector &actuator_min,
			     ActuatorEffectiveness::ActuatorVector &actuator_max)
{
	const ActuatorEffectiveness::Configuration configuration = declareActuators(airship);
	actuator_min.setAll(-1.f);

	for (int i = 0; i < configuration.num_actuators[(int)ActuatorType::MOTORS]; i++) {
		actuator_min(i) = 0.f;
	}

	actuator_max.setAll(1.f);
}

static void runUpdateSetpoint(ActuatorEffectivenessAirship &airship, const Vector<float, 6> &control_sp,
			      ActuatorEffectiveness::ActuatorVector &actuator_sp)
{
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	productionLimits(airship, actuator_min, actuator_max);
	airship.allocateAuxilaryControls(kDt, 0, actuator_sp);
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
}

// One allocator step with explicit output limits, for the cases that need
// something other than the production [0, 1] motors / [-1, 1] servos
static void runUpdateSetpoint(ActuatorEffectivenessAirship &airship, const Vector<float, 6> &control_sp,
			      ActuatorEffectiveness::ActuatorVector &actuator_sp,
			      const ActuatorEffectiveness::ActuatorVector &actuator_min,
			      const ActuatorEffectiveness::ActuatorVector &actuator_max)
{
	declareActuators(airship);
	airship.allocateAuxilaryControls(kDt, 0, actuator_sp);
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
}

// Every test starts from the parameter defaults and the armed state; the
// setpoints, the actuator vector and the status the report reads are members,
// so a test writes only what it is about. The airship itself stays a local
// built after the test's own parameter setters, as the allocator builds it
// after reading them
class ActuatorEffectivenessAirshipTest : public ::testing::Test
{
public:
	void SetUp() override { resetAirshipParams(); }

protected:
	/** Narrow or collapse the tilt range of the airship the test builds next */
	static void setTiltRange(float tilt_min_deg, float tilt_max_deg)
	{
		param_set(param_find("CA_AIRSHIP_TLMIN"), &tilt_min_deg);
		param_set(param_find("CA_AIRSHIP_TLMAX"), &tilt_max_deg);
	}

	/** One allocator step at the production limits with the fixture's setpoints */
	void run(ActuatorEffectivenessAirship &airship) { runUpdateSetpoint(airship, control_sp, actuator_sp); }

	/** The report the allocator would publish; on a surface-served axis seed status with the matrix residual first */
	void report(ActuatorEffectivenessAirship &airship) { airship.getUnallocatedControl(0, status); }

	Vector<float, 6> control_sp{};
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	control_allocator_status_s status{};
};

TEST_F(ActuatorEffectivenessAirshipTest, ForwardCruise)
{
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	run(airship);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, FullYawIsTheExactCouple)
{
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);

	// Equal and opposite thrust vectors: zero net force, maximum yaw couple
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f); // +180 deg
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);      // forward
}

TEST_F(ActuatorEffectivenessAirshipTest, VerticalClimb)
{
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	run(airship);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.5f); // +90 deg
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.5f);
}

TEST_F(ActuatorEffectivenessAirshipTest, CruiseWithYaw)
{
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.6f;
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.2f;
	run(airship);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.4f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.8f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, SaturationClampsAndReports)
{
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 1.f;
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);

	// Starboard pod demand cancels to zero, port pod saturates at 2x
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f); // yaw saturated positive
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 1.f); // forward thrust saturated positive
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], 0.f);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, TiltHeldThroughZeroThrust)
{
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	control_sp.setZero();
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, MotorsStayUnidirectional)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// The props are non-reversible: any demand, including full reverse,
	// must map to motor commands in [0, 1]. Reversal is done by the tilt.
	for (float thrust = -1.f; thrust <= 1.f; thrust += 0.5f) {
		for (float yaw = -1.f; yaw <= 1.f; yaw += 0.5f) {
			for (float roll = -1.f; roll <= 1.f; roll += 0.5f) {
				control_sp.setZero();
				control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = thrust;
				control_sp(ActuatorEffectiveness::ControlAxis::YAW) = yaw;
				control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = roll;
				actuator_sp.setZero();
				run(airship);

				SCOPED_TRACE(::testing::Message() << "thrust=" << thrust << " yaw=" << yaw << " roll=" << roll);
				EXPECT_GE(actuator_sp(MOTOR_STARBOARD), 0.f);
				EXPECT_LE(actuator_sp(MOTOR_STARBOARD), 1.f);
				EXPECT_GE(actuator_sp(MOTOR_PORT), 0.f);
				EXPECT_LE(actuator_sp(MOTOR_PORT), 1.f);
			}
		}
	}
}

TEST_F(ActuatorEffectivenessAirshipTest, TiltRangeLimitedReportsTheUnreachableHalf)
{
	setTiltRange(-90.f, 90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);

	// Reverse is unreachable at either end, so the starboard pod realizes
	// none of its share (that its tilt stays level is the pod suite's pin):
	// the motor clamps off and half the couple is a real shortfall
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f); // half the couple is missing
}

TEST_F(ActuatorEffectivenessAirshipTest, NoReverseEndReportsTheWholeDemand)
{
	// Symmetric -90..+90 deg range (the Cloudship geometry) cruising forward,
	// then asked for straight back, which is realizable at neither end: the
	// pods stand (the pod suite pins the tilt), the motors clamp off, and the
	// whole demand is a real shortfall, nothing held by choice
	setTiltRange(-90.f, 90.f);
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	run(airship);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], -1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, CollectiveModeCruiseAndClimb)
{
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), 0.f);

	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), 0.5f); // +90 deg
}

TEST_F(ActuatorEffectivenessAirshipTest, FixedMountRejectsVertical)
{
	setTiltRange(0.f, 0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	run(airship);

	// Vertical demand has no component along the fixed forward mount
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], -1.f);

	// The mirror: a downward demand is equally unreachable
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, FixedMountDifferentialYaw)
{
	setTiltRange(0.f, 0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, FixedMountAtAngleServesVertical)
{
	// TLMIN = TLMAX = 90 is outside the declared parameter ranges, but
	// nothing prevents it in storage: the empty range must degrade to a
	// fixed mount at that angle, projecting the demand onto it
	setTiltRange(90.f, 90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 0);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	// Forward demand has no component along the up-fixed mount
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	run(airship);
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, InvertedRangeIsAFixedMountAtTheMinimum)
{
	// TLMIN > TLMAX is outside the declared parameter ranges, but nothing
	// prevents it in storage: it declares no tilt servo, so the pods must
	// sit still at TLMIN rather than flip between the two limits every
	// update
	setTiltRange(30.f, -30.f);
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	ActuatorEffectiveness::Configuration configuration{};
	EXPECT_TRUE(airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 0);

	// A down demand has no component along the 30 deg mount: motors off,
	// and identically so on the next update
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 0.5f;

	for (int update = 0; update < 3; update++) {
		SCOPED_TRACE(::testing::Message() << "update=" << update);
		run(airship);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	}

	// Forward demand projects onto the 30 deg mount
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f * cosf(math::radians(30.f)));
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f * cosf(math::radians(30.f)));
}

TEST_F(ActuatorEffectivenessAirshipTest, CollectiveModeYawAndRollUnallocated)
{
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 0.5f;
	run(airship);

	// No differential thrust: both demands are left unmet
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], 1.f);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = -1.f;
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = -0.5f;
	run(airship);
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], -1.f);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], -1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, CollectiveModeHasNoDifferential)
{
	// Generic-airship class: fixed mounts driven by one thrust command
	setTiltRange(0.f, 0.f);
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);

	// No differential thrust: yaw is left to the fins and reported unmet
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);

	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
}

TEST_F(ActuatorEffectivenessAirshipTest, CollectiveModeReverseCruise)
{
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	run(airship);

	// Full reverse cruise through the tilt, with non-negative motors
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), 1.f); // 180 deg
}

TEST_F(ActuatorEffectivenessAirshipTest, MotorLimitRespected)
{
	ActuatorEffectivenessAirship airship(nullptr);
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	productionLimits(airship, actuator_min, actuator_max);
	actuator_max(MOTOR_STARBOARD) = 0.8f;
	actuator_max(MOTOR_PORT) = 0.8f;
	runUpdateSetpoint(airship, control_sp, actuator_sp, actuator_min, actuator_max);

	// The configured motor limit caps the couple; the shortfall is reported
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.8f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.8f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, PitchTorqueUnallocated)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// The pods cannot produce pitch torque in any mode
	control_sp(ActuatorEffectiveness::ControlAxis::PITCH) = 1.f;
	run(airship);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[1], 1.f);

	control_sp(ActuatorEffectiveness::ControlAxis::PITCH) = -1.f;
	run(airship);
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[1], -1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, LateralThrustReportedUnserved)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// No airship actuator produces lateral force: the demand must be
	// reported unserved, not silently read as allocated
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Y) = 1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[1], 1.f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Y) = -1.f;
	run(airship);
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[1], -1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, CollectiveSingleTiltConfiguration)
{
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

TEST_F(ActuatorEffectivenessAirshipTest, TailServesCollectiveYaw)
{
	setCollectiveMode();
	setTailThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);

	// The pods have no differential thrust: the tail takes the whole demand
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_TAIL), 1.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, TailReverseNeedsConfiguration)
{
	setCollectiveMode();
	setTailThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = -1.f;

	// Non-reversible tail: negative demand clamps to zero and is reported
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_TAIL), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], -1.f);

	// Reversible tail (CA_R_REV): full reverse authority
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	productionLimits(airship, actuator_min, actuator_max);
	actuator_min(MOTOR_TAIL) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_TAIL), -1.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, TailIdleWithIndependentCouple)
{
	setTailThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);

	// The couple serves the demand exactly: nothing left for the tail
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_TILT_STARBOARD), 1.f); // +180 deg
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_TILT_PORT), 0.f);
	EXPECT_NEAR(actuator_sp(MOTOR_TAIL), 0.f, 1e-6f);
}

TEST_F(ActuatorEffectivenessAirshipTest, TailTopsUpFixedMountYaw)
{
	setTiltRange(0.f, 0.f);
	setTailThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);

	// Fixed mounts yield half the couple differentially; the tail tops up
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_TAIL), 0.5f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, AsymmetricTiltRange)
{
	// An up-only tilt range: collective mode, reversible tail thruster,
	// 0 deg (forward) to +90 deg (up)
	setTiltRange(0.f, 90.f);
	setCollectiveMode();
	setTailThruster();
	ActuatorEffectivenessAirship airship(nullptr);
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	productionLimits(airship, actuator_min, actuator_max);
	actuator_min(MOTOR_TAIL) = -1.f;

	// Cruise: 0 deg is the range minimum, so the tilt servo sits at -1
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	runUpdateSetpoint(airship, control_sp, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), -1.f);

	// Descent is unreachable: neither range end realizes a downward demand,
	// so the tilt stays at the 0 deg floor, the motors project to zero and
	// the shortfall is reported
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), -1.f);
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.f, 1e-6f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], 1.f);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 0.f);

	// Climb: +90 deg is the range maximum, so the tilt servo sits at +1
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), 1.f);

	// Reverse cruise is unreachable: the tilt clamps at +90 deg where the
	// demand has no feasible component, and the shortfall is reported
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp, actuator_min, actuator_max);
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.f, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_TAIL), 0.f, 1e-6f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], -1.f);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, FixedMountConfiguration)
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

// The constants above model the declared layout; check the model against the
// layout the effectiveness really declares, in every shape the suite uses
static void expectDeclaredLayout(bool tail, int num_surfaces, int num_tilts)
{
	SCOPED_TRACE(::testing::Message() << "tail=" << tail << " surfaces=" << num_surfaces
		     << " tilts=" << num_tilts);
	ActuatorEffectivenessAirship airship(nullptr);
	const ActuatorEffectiveness::Configuration configuration = declareActuators(airship);
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::MOTORS], numMotors(tail));
	EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], num_surfaces + num_tilts);
	EXPECT_EQ(configuration.num_actuators_matrix[0], firstTilt(tail, num_surfaces) + num_tilts);
}

TEST_F(ActuatorEffectivenessAirshipTest, DeclaredLayoutMatchesTheIndexMap)
{
	expectDeclaredLayout(false, 0, 2);

	resetAirshipParams();
	setTailThruster();
	expectDeclaredLayout(true, 0, 2);

	resetAirshipParams();
	setSurfaces();
	expectDeclaredLayout(false, 2, 2);

	resetAirshipParams();
	setAileron();
	expectDeclaredLayout(false, 1, 2);

	resetAirshipParams();
	setCollectiveMode();
	expectDeclaredLayout(false, 0, 1);

	resetAirshipParams();
	setCollectiveMode();
	setTailThruster();
	setSurfaces();
	expectDeclaredLayout(true, 2, 1);
}

TEST_F(ActuatorEffectivenessAirshipTest, SettledPodsDoNotReportRoundingAsSaturation)
{
	// Both pods settled on a reachable demand and delivering it: the residual
	// is the rounding of atan2/cos/sin and the servo round trip, and must not
	// be published as saturation, which would stop the yaw integrator
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.05f;
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -0.60f;

	// resetAirshipParams leaves CA_AIRSHIP_TLT_R at 0, so each tilt reaches
	// its target in one update: there is nothing to settle, and the residual
	// under test is the round trip alone
	run(airship);

	// The hover-climb point of 4580bb6f3e, where the FLT_EPSILON band it
	// replaced reported a sign on 5.6 % of small yaw setpoints, first at
	// 0.0026. Which setpoints carry a residual past FLT_EPSILON is a libm
	// rounding pattern, so sweep the small-setpoint regime rather than pin
	// the points that failed that day. Every point is delivered exactly up
	// to rounding: both pods steer (|demand| ~ 0.6, above kSteerEngage), the
	// demand (0.05 -/+ yaw, 0.6) never enters the rear cone (its up component
	// is far above the switch margin) and stays inside the thrust clamp,
	// which the port pod reaches only at yaw = 0.75
	for (int i = 2; i <= 100; i++) {
		const float yaw_sp = 0.0005f * i;	// 0.001 .. 0.05
		control_sp(ActuatorEffectiveness::ControlAxis::YAW) = yaw_sp;
		run(airship);

		report(airship);
		EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f) << "yaw setpoint " << (double)yaw_sp;
	}
}

TEST_F(ActuatorEffectivenessAirshipTest, SurfaceServedSettledPodsDoNotReportRoundingAsSaturation)
{
	// The same point with a rudder serving yaw at half credit, the one case
	// in which the yaw rate loop is closed on a served axis: the pods realize
	// their half of the yaw through the same round trip, and its rounding
	// must not be published as a sign on the surface branch either
	setSurfaces();
	setSurfaceCredit(0.5f);
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.05f;
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -0.60f;
	run(airship);

	for (int i = 2; i <= 100; i++) {
		const float yaw_sp = 0.0005f * i;	// 0.001 .. 0.05
		control_sp(ActuatorEffectiveness::ControlAxis::YAW) = yaw_sp;
		actuator_sp(SURFACE_RUDDER) = yaw_sp;	// the matrix allocates the whole demand to the unit rudder
		run(airship);

		status = {};	// matrix residual: nothing, the rudder took it all
		report(airship);
		EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f) << "yaw setpoint " << (double)yaw_sp;
	}
}

TEST_F(ActuatorEffectivenessAirshipTest, SurfaceIndicesDoNotMoveWithTheTiltCount)
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

TEST_F(ActuatorEffectivenessAirshipTest, SurfacesKeepTheirSetpoints)
{
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	actuator_sp(SURFACE_ELEVATOR) = 0.7f;
	run(airship);

	// The matrix allocation of the surfaces is left untouched
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(SURFACE_ELEVATOR), 0.7f);
}

TEST_F(ActuatorEffectivenessAirshipTest, NoTorqueSurfaceTypesTakeNoTorque)
{
	// ActuatorEffectivenessControlSurfaces::updateParams() zeroes the TRQ
	// entries of the types controlSurfaceTakesTorque() rejects: such a
	// surface declares an empty column, serves no axis, and the collective
	// pods, which cannot yaw, report the whole demand as a sign rather than
	// the matrix residual
	setCollectiveMode();

	for (const int32_t type : {9, 10, 11, 16, 17, 18}) {	// flaps, airbrake, steering wheel, spoilers
		SCOPED_TRACE(::testing::Message() << "CA_SV_CS0_TYPE=" << type);
		setNoTorqueSurface(type);
		ActuatorEffectivenessAirship airship(nullptr);

		const ActuatorEffectiveness::Configuration configuration = declareActuators(airship);
		EXPECT_EQ(configuration.num_actuators[(int)ActuatorType::SERVOS], 2); // the surface and the collective tilt
		EXPECT_FLOAT_EQ(configuration.effectiveness_matrices[0](2, SURFACE_FIRST), 0.f);

		control_sp.setZero();
		control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
		actuator_sp.setZero();
		actuator_sp(SURFACE_FIRST) = 1.f;	// a deflected surface of this type is credited no yaw
		run(airship);

		status = {};	// residual left at 0: a credited surface would publish it
		report(airship);
		EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
	}
}

TEST_F(ActuatorEffectivenessAirshipTest, SurfaceCreditScalesPodShare)
{
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

		control_sp.setZero();
		control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
		actuator_sp.setZero();
		actuator_sp(SURFACE_RUDDER) = 0.6f;
		run(airship);

		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), c.motor);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), c.motor);
		EXPECT_FLOAT_EQ(actuator_sp(SURFACES_TILT_STARBOARD), 1.f); // +180 deg
		EXPECT_FLOAT_EQ(actuator_sp(SURFACES_TILT_PORT), 0.f);      // forward
		EXPECT_FLOAT_EQ(actuator_sp(SURFACE_RUDDER), 0.6f); // the matrix allocation stands

		// The uncredited rudder share cancels against the solver residual:
		// the pods served everything, so nothing is unallocated
		status = {};
		status.unallocated_torque[2] = 0.4f; // matrix residual: demand minus rudder
		report(airship);
		EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
	}
}

TEST_F(ActuatorEffectivenessAirshipTest, SurfaceSlewLagIsReportedOnTheServedAxis)
{
	// The matrix gives the rudder 0.6 of a unit yaw demand and the pods serve
	// the rest; CA_SVn_SLEW then holds the rudder at 0.3, so the allocator's
	// residual, rebuilt from the final setpoint, is 0.7 and 0.3 of the yaw is
	// really missing. The local demand alone sees none of it
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	actuator_sp(SURFACE_RUDDER) = 0.6f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.4f);

	status.unallocated_torque[2] = 0.7f;	// demand minus the slewed rudder
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, WeakYawCouplingIsNotSurfaceServed)
{
	// An elevator's yaw entry at or below kMinEffectiveness never reaches
	// the allocation: the allocator zeroes that row, allocates nothing on
	// yaw and hands back the whole demand as its residual, with no surface
	// term in it. The axis is therefore not surface-served, and the report
	// is the sign of what the pods left of the local demand, not that
	// residual plus the uncredited coupling. Tilting pods hold the small
	// counter-yaw by choice; fixed mounts fall short of it by force
	struct {
		float tilt_deg;	// tilt range +-tilt_deg
		float credit;
		float report;
	} cases[] = {
		{180.f, 1.f, 0.f}, {180.f, 0.5f, 0.f}, {180.f, 0.f, 0.f},
		{0.f, 1.f, -1.f}, {0.f, 0.5f, -1.f}, {0.f, 0.f, 0.f},
	};

	for (const auto &c : cases) {
		SCOPED_TRACE(::testing::Message() << "tilt=" << c.tilt_deg << " credit=" << c.credit);
		resetAirshipParams(-c.tilt_deg, c.tilt_deg);
		setElevatorWithWeakYaw();
		setSurfaceCredit(c.credit);
		ActuatorEffectivenessAirship airship(nullptr);

		control_sp.setZero();
		control_sp(ActuatorEffectiveness::ControlAxis::PITCH) = 0.5f;
		actuator_sp.setZero();
		actuator_sp(SURFACE_FIRST) = 0.5f;	// the matrix pass allocates the pitch to the elevator
		run(airship);

		// The credited coupling, -0.015 * credit of yaw, asks the port pod for
		// a reverse it does not give: the same outputs before and after the
		// threshold, which decides the report alone
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.015f * c.credit);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

		status = {};	// the zeroed row's residual is the yaw demand itself: 0
		report(airship);
		EXPECT_FLOAT_EQ(status.unallocated_torque[2], c.report);
	}
}

TEST_F(ActuatorEffectivenessAirshipTest, RollSurfaceCreditedAgainstDifferentialRoll)
{
	setAileron();
	ActuatorEffectivenessAirship airship(nullptr);

	// The aileron carries 0.6 of the roll demand from the matrix pass;
	// the pods serve the remaining 0.4 differentially, straight up and
	// down, and the report is exact
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 1.f;
	actuator_sp(SURFACE_FIRST) = 0.6f;
	run(airship);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.4f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.4f);
	EXPECT_FLOAT_EQ(actuator_sp(AILERON_TILT_STARBOARD), -0.5f); // -90 deg, down
	EXPECT_FLOAT_EQ(actuator_sp(AILERON_TILT_PORT), 0.5f);       // +90 deg, up

	status.unallocated_torque[0] = 0.4f; // matrix residual: demand minus aileron
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, RollSteerBandShortfallIsNotSaturation)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// Level pods deliver none of a roll demand below the steer floor: the
	// unmet demand is the band's own choice, not saturation
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 0.75f * AirshipPod::kSteerEngage;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], 0.f);

	// Over the floor the pods vector straight down and up and serve it exactly
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = 2.f * AirshipPod::kSteerEngage;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), -0.5f); // -90 deg, down
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.5f);       // +90 deg, up
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 2.f * AirshipPod::kSteerEngage, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 2.f * AirshipPod::kSteerEngage, 1e-6f);
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, RollSurfaceServedBandShortfallIsNotSaturation)
{
	setAileron();
	setSurfaceCredit(0.5f);
	ActuatorEffectivenessAirship airship(nullptr);

	// An aileron serves roll, so the report takes the surface branch. Half
	// the aileron's allocation is credited, leaving half the demand for the
	// pods, which is inside the steer band: they hold level and deliver none
	const float demand = AirshipPod::kSteerEngage;
	control_sp(ActuatorEffectiveness::ControlAxis::ROLL) = demand;
	actuator_sp(SURFACE_FIRST) = demand; // the matrix allocates the full demand to the unit aileron
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(AILERON_TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(AILERON_TILT_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	// The held half is the band's choice on this path too
	status.unallocated_torque[0] = 0.f; // matrix residual: demand minus aileron
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, PitchDeferredToSurfaces)
{
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::PITCH) = 1.f;
	actuator_sp(SURFACE_ELEVATOR) = 1.f;	// the matrix allocates the whole pitch to the unit elevator
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	// With an elevator the allocator's own residual is what is reported, as
	// a sign like every other axis: here 0.25, after a CA_SV0_SLEW held the
	// elevator at 0.75. The local demand, 1 - 1 x 1.0 = 0, would report 0
	status.unallocated_torque[1] = 0.25f;
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[1], 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, TailAfterSurfaces)
{
	setCollectiveMode();
	setTailThruster();
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	// The rudder carries 0.6 of the yaw demand and the collective pods
	// none: the tail serves only the remaining 0.4
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	actuator_sp(TAIL_COLLECTIVE_RUDDER) = 0.6f;
	run(airship);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_TAIL), 0.4f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_RUDDER), 0.6f);

	status.unallocated_torque[2] = 0.4f; // matrix residual: demand minus rudder
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, SurfaceTrimNotCredited)
{
	setSurfaces();
	float trim = 0.2f;
	param_set(param_find("CA_SV_CS1_TRIM"), &trim);
	ActuatorEffectivenessAirship airship(nullptr);

	// The rudder sits exactly at its trim: it delivers no torque, so the
	// pods must serve the whole demand and the report must not drift
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	actuator_sp(SURFACE_RUDDER) = 0.2f;
	run(airship);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	status.unallocated_torque[2] = 1.f; // matrix residual: demand minus (rudder - trim)
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, SaturatedSurfaceNotOverCredited)
{
	setSurfaces();
	ActuatorEffectivenessAirship airship(nullptr);

	// The matrix over-allocated the rudder to 1.5, but only the clipped
	// 1.0 can ever be delivered: crediting the excess would push a
	// phantom negative demand into the pods
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	actuator_sp(SURFACE_RUDDER) = 1.5f;
	run(airship);

	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(actuator_sp(SURFACE_RUDDER), 1.5f); // clipping stays the allocator's job
}

TEST_F(ActuatorEffectivenessAirshipTest, TiltServoLimitBoundsProjection)
{
	ActuatorEffectivenessAirship airship(nullptr);
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	productionLimits(airship, actuator_min, actuator_max);
	actuator_max(TILT_STARBOARD) = 0.5f; // output limit: +90 deg at most

	// Full reverse asks for +180 deg; the limited starboard servo stops
	// at +90 deg and the projection must use that realized angle
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp, actuator_min, actuator_max);

	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.5f);
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, CloudshipMirrorSymmetricTilt)
{
	// Mirror of the Cloudship preset's tilt geometry: collective mode, tail
	// thruster, symmetric tilt range -90..+90 deg (level = servo 0). The
	// preset's reversible tail (CA_R_REV) is exercised by
	// TailReverseNeedsConfiguration; no yaw is demanded here
	setTiltRange(-90.f, 90.f);
	setCollectiveMode();
	setTailThruster();
	ActuatorEffectivenessAirship airship(nullptr);

	// Cruise: level tilt sits at the servo center, as the old mixer did
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.5f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.5f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), 0.f);

	// Climb: +90 deg is the range maximum
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), 1.f);

	// Descent: the symmetric range restores downward vectoring
	control_sp.setZero();
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TAIL_COLLECTIVE_TILT), -1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, TiltRangeNarrowedAtRuntimeReclampsHeldTilt)
{
	// Drive the parameter update the way the allocator does: a notification on
	// the parent cascades to every ModuleParams child
	struct ParamOwner : ModuleParams {
		ParamOwner() : ModuleParams(nullptr) {}
		using ModuleParams::updateParams;
	} owner;
	ActuatorEffectivenessAirship airship{&owner};

	// Commit to the +180 deg end with the full default range
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	run(airship);
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
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), below_release);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), below_release);
}

TEST_F(ActuatorEffectivenessAirshipTest, CollapsedRangeWithoutRedeclarationLeavesTheTiltSlots)
{
	// The allocator redeclares the actuators after every parameter update,
	// so a collapsed range never reaches updateSetpoint() with tilt servos
	// still declared. writeTiltServos() guards that order anyway: with the
	// span at zero its servo mapping would divide by zero, and the NaN would
	// go out on the tilt slots and, read back into the pods, on the motors
	struct ParamOwner : ModuleParams {
		ParamOwner() : ModuleParams(nullptr) {}
		using ModuleParams::updateParams;
	} owner;
	ActuatorEffectivenessAirship airship{&owner};

	// The limits of the tilting layout, and one declared step that writes
	// both tilt slots
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	productionLimits(airship, actuator_min, actuator_max);
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 1.f);

	// Collapse the range and step again WITHOUT redeclaring
	float zero = 0.f;
	param_set(param_find("CA_AIRSHIP_TLMIN"), &zero);
	param_set(param_find("CA_AIRSHIP_TLMAX"), &zero);
	owner.updateParams();
	airship.allocateAuxilaryControls(kDt, 0, actuator_sp);
	airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);

	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f); // the stale slots are left as they were
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 1.f);
	EXPECT_TRUE(PX4_ISFINITE(actuator_sp(MOTOR_STARBOARD)));
	EXPECT_TRUE(PX4_ISFINITE(actuator_sp(MOTOR_PORT)));
}

TEST_F(ActuatorEffectivenessAirshipTest, RestrictedRangeRearDemandLeavesOnlyTheUpShare)
{
	// Down-only range with a back-and-up demand OUTSIDE the rear cone:
	// the atan2 target (+169 deg) is out of range, and the numerically
	// nearer bound (0 deg, forward) realizes none of it. The pods reach
	// the -180 end (the pod suite pins the pick): full reverse thrust,
	// and only the up share is unmet
	setTiltRange(-180.f, 0.f);
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -0.2f;
	run(airship);

	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 0.f);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], -1.f); // the up share is unreachable
}

TEST_F(ActuatorEffectivenessAirshipTest, CollectiveClampedServoRealizedCopy)
{
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	productionLimits(airship, actuator_min, actuator_max);
	actuator_max(COLLECTIVE_TILT) = 0.5f; // +90 deg reachable at most

	// Full reverse asks for +180 deg; the clamped collective servo stops
	// at +90 deg and BOTH pods must project onto that realized angle
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	runUpdateSetpoint(airship, control_sp, actuator_sp, actuator_min, actuator_max);

	EXPECT_FLOAT_EQ(actuator_sp(COLLECTIVE_TILT), 0.5f);
	EXPECT_NEAR(actuator_sp(MOTOR_STARBOARD), 0.f, 1e-6f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.f, 1e-6f);
}

TEST_F(ActuatorEffectivenessAirshipTest, TiltRateIsReadInDegreesPerSecond)
{
	// CA_AIRSHIP_TLT_R reaches the pods in radians: the one conversion the
	// pod suite cannot see. A full vertical demand asks for +90 deg, and
	// the first update moves the tilt by 90 deg/s x kDt = 1.8 deg, 0.01 of
	// the 360 deg servo span
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	run(airship);
	EXPECT_NEAR(actuator_sp(TILT_STARBOARD), 0.01f, 1e-6f);
}

TEST_F(ActuatorEffectivenessAirshipTest, DisarmParksTiltLevel)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// Armed (resetAirshipParams publishes it): full yaw couple
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	// Disarmed, the tilt parks level regardless of the demand and the
	// reverse projection clamps the motor off
	ScopedDisarm disarmed;

	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f); // parked at 0 deg, forward
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, SteerBandShortfallIsNotSaturation)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// Level pods at hover: a yaw demand below the steer floor is served by
	// the port pod alone, the starboard pod would need the reversal the band
	// deliberately withholds
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.75f * AirshipPod::kSteerEngage;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.75f * AirshipPod::kSteerEngage, 1e-6f);

	// The unmet half is the band's own choice, not saturation: the rate
	// controller's integrator must stay free to lift the demand over the floor
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 0.f);

	// Over the floor the steering engages and the exact couple is served
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 2.f * AirshipPod::kSteerEngage;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, OppositeSignedHoldIsNotDiscounted)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// Commit both tilts rearward, so the seam keeps them there afterwards
	Vector<float, 6> rear{};
	rear(ActuatorEffectiveness::ControlAxis::THRUST_X) = -1.f;
	runUpdateSetpoint(airship, rear, actuator_sp);

	// The port pod steers and is kept at the +180 deg end by the seam rule
	// (switching ends would gain less than the margin), so it delivers its
	// backward component and leaves its small down component unserved -- a
	// real shortfall; the starboard pod holds in the band with an up
	// component withheld by choice. On the vertical axis the held share and
	// the shortfall therefore point opposite ways. Only a same-signed share
	// may be discounted: subtract this one and the axis stops reporting a
	// saturation the rate integrator has to see
	Vector<float, 6> split{};
	split(ActuatorEffectiveness::ControlAxis::ROLL) = -0.023f;
	split(ActuatorEffectiveness::ControlAxis::YAW) = -0.12f;
	split(ActuatorEffectiveness::ControlAxis::THRUST_X) = -0.12f;
	split(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 0.007f;
	runUpdateSetpoint(airship, split, actuator_sp);

	// Pin the state described: both pods at the +180 deg end, the starboard
	// motor off (its up demand is perpendicular to the axis), the port motor
	// serving its backward component
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.24f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[2], 1.f);
	EXPECT_FLOAT_EQ(status.unallocated_torque[0], -1.f); // roll: same-signed held 0.008 against a 0.023 shortfall, the remainder is real
}

TEST_F(ActuatorEffectivenessAirshipTest, HeldShareDoesNotHideAClampedPod)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// Cruise with a yaw demand just past the forward thrust: the starboard
	// pod's tiny reverse share sits in the band and is withheld by choice,
	// while the port pod clamps at its motor maximum in the same direction.
	// Only the held share may be discounted; the clamped remainder is real
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.6f;
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.615f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f); // yaw: 0.115 short, 0.0075 held by choice, the rest clamped
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 1.f); // thrust: held share opposite-signed, not discounted
}

TEST_F(ActuatorEffectivenessAirshipTest, SteerBandVerticalShortfallIsNotSaturation)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// Level pods with a vertical demand below the steer floor, in both
	// directions: the pods hold level and deliver none of it, and the held
	// share is the band's choice on the thrust axis as on the torque axes
	for (const float sign : {-1.f, 1.f}) {
		SCOPED_TRACE(::testing::Message() << "sign=" << sign);
		control_sp.setZero();
		control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = sign * 0.75f *
				AirshipPod::kSteerEngage;
		actuator_sp.setZero();
		run(airship);
		EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
		EXPECT_FLOAT_EQ(actuator_sp(TILT_PORT), 0.f);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
		EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

		status = {};
		report(airship);
		EXPECT_FLOAT_EQ(status.unallocated_thrust[2], 0.f);
		EXPECT_FLOAT_EQ(status.unallocated_torque[0], 0.f);
	}
}

TEST_F(ActuatorEffectivenessAirshipTest, SwingShortfallIsSaturation)
{
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// A full couple from level pods: the starboard tilt is steering toward
	// the rear but moves 1.8 deg per update, so the couple is not
	// delivered yet. That shortfall is transient saturation the rate
	// controller must not integrate against
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);
	EXPECT_GT(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_LT(actuator_sp(TILT_STARBOARD), 0.2f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, HysteresisBandReversalIsNotSaturation)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// Commit the starboard pod to the rear with a full couple
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);

	// A reversed demand inside the hysteresis band keeps the committed
	// directions, which now project both pods away: motors off, no torque.
	// That is the band holding by choice, not saturation
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) =
		-0.5f * (AirshipPod::kSteerEngage + AirshipPod::kSteerRelease);
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 1.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_PORT), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, SwingContinuedInBandIsSaturation)
{
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// Start the starboard reversal with a full couple
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);
	const float first = actuator_sp(TILT_STARBOARD);
	EXPECT_GT(first, 0.f);

	// The demand drops into the band: the target stands and the tilt keeps
	// swinging toward it, so the couple is still not delivered. A moving
	// tilt is a real, transient shortfall
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) =
		0.5f * (AirshipPod::kSteerEngage + AirshipPod::kSteerRelease);
	run(airship);
	EXPECT_GT(actuator_sp(TILT_STARBOARD), first);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, FixedMountInBandShortfallStands)
{
	setTiltRange(0.f, 0.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// Fixed forward pods cannot reverse at all: the half the starboard motor
	// cannot deliver is structural even below the steer floor
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.75f * AirshipPod::kSteerEngage;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, OnePodHeldInBandIsNotSaturation)
{
	ActuatorEffectivenessAirship airship(nullptr);

	// Cruise with a yaw demand just above the forward thrust: the port pod
	// steers and delivers, the starboard pod's tiny reverse demand sits in
	// the band and holds level. The starboard shortfall is the band's choice
	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_X) = 0.3f;
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.3f + 0.5f * AirshipPod::kSteerEngage;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.6f + 0.5f * AirshipPod::kSteerEngage, 1e-6f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
	EXPECT_FLOAT_EQ(status.unallocated_thrust[0], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, DisarmedShortfallStillReported)
{
	ActuatorEffectivenessAirship airship(nullptr);

	ScopedDisarm disarmed;

	// Parked tilts are not the steer band's choice: a disarmed demand the
	// parked pods cannot realize is still reported honestly
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(TILT_STARBOARD), 0.f);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, CollectiveYawShortfallStandsInSteerBand)
{
	setCollectiveMode();
	ActuatorEffectivenessAirship airship(nullptr);

	// Collective pods cannot yaw at all: a small demand below the steer
	// floor is structurally unserved and must still be reported
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 0.5f * AirshipPod::kSteerEngage;
	run(airship);

	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, SurfaceServedBandShortfallIsNotSaturation)
{
	setSurfaces();
	setSurfaceCredit(0.5f);
	ActuatorEffectivenessAirship airship(nullptr);

	// A rudder serves yaw, so the report takes the surface branch. Half the
	// rudder's allocation is credited, leaving half the demand for the pods,
	// which is inside the steer band: they hold level and the starboard
	// motor clips, so only half of that arrives
	const float demand = AirshipPod::kSteerEngage;
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = demand;
	actuator_sp(SURFACE_RUDDER) = demand; // the matrix allocates the full demand to the unit rudder
	run(airship);
	EXPECT_FLOAT_EQ(actuator_sp(SURFACES_TILT_STARBOARD), 0.f);
	EXPECT_FLOAT_EQ(actuator_sp(MOTOR_STARBOARD), 0.f);
	EXPECT_NEAR(actuator_sp(MOTOR_PORT), 0.5f * demand, 1e-6f);

	// The held half is the band's choice on this path too
	status.unallocated_torque[2] = 0.f; // matrix residual: demand minus rudder
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 0.f);
}

TEST_F(ActuatorEffectivenessAirshipTest, SurfaceServedSwingShortfallStands)
{
	setSurfaces();
	setSurfaceCredit(0.5f);
	setTiltRate(90.f);
	ActuatorEffectivenessAirship airship(nullptr);

	// Same path, but now the pod share is far above the band: the starboard
	// tilt is steering toward the rear and has not arrived, so the couple is
	// genuinely missing and must still be reported
	control_sp(ActuatorEffectiveness::ControlAxis::YAW) = 1.f;
	actuator_sp(SURFACE_RUDDER) = 0.6f;
	run(airship);
	EXPECT_GT(actuator_sp(SURFACES_TILT_STARBOARD), 0.f);
	EXPECT_LT(actuator_sp(SURFACES_TILT_STARBOARD), 0.2f);

	status.unallocated_torque[2] = 0.4f; // matrix residual: demand minus rudder
	report(airship);
	EXPECT_FLOAT_EQ(status.unallocated_torque[2], 1.f);
}
