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

#include "airship_manual_input.hpp"

using namespace airship_manual_input;

static constexpr float kMaxRate = math::radians(15.f); // AS_YAWRATE_MAX default, as the module converts it
static constexpr float kDeadzone = 0.1f;

static manual_control_setpoint_s sticks(float roll, float pitch, float yaw, float throttle)
{
	manual_control_setpoint_s setpoint{};
	setpoint.roll = roll;
	setpoint.pitch = pitch;
	setpoint.yaw = yaw;
	setpoint.throttle = throttle;
	return setpoint;
}

TEST(AirshipManualInputTest, ThrottleMapsToForwardThrust)
{
	// The throttle stick [-1, 1] is the forward thrust [0, 1]; nothing lateral
	EXPECT_FLOAT_EQ(thrust(sticks(0.f, 0.f, 0.f, -1.f))(0), 0.f);
	EXPECT_FLOAT_EQ(thrust(sticks(0.f, 0.f, 0.f, 0.f))(0), 0.5f);
	EXPECT_FLOAT_EQ(thrust(sticks(0.f, 0.f, 0.f, 1.f))(0), 1.f);
	EXPECT_FLOAT_EQ(thrust(sticks(0.4f, 0.5f, -0.7f, 1.f))(1), 0.f);
}

TEST(AirshipManualInputTest, PitchStickIsVerticalThrustAndPitchTorque)
{
	// Stick forward (+pitch) descends, +z in FRD, and is nose down, a
	// negative pitch rotation
	const manual_control_setpoint_s forward = sticks(0.f, 0.5f, 0.f, 0.f);
	EXPECT_FLOAT_EQ(thrust(forward)(2), 0.5f);
	EXPECT_FLOAT_EQ(torque(forward)(1), -0.5f);
}

TEST(AirshipManualInputTest, RollAndYawPassStraightThrough)
{
	const manual_control_setpoint_s deflected = sticks(0.3f, 0.f, -0.7f, 0.f);
	EXPECT_FLOAT_EQ(torque(deflected)(0), 0.3f);
	EXPECT_FLOAT_EQ(torque(deflected)(2), -0.7f);
	EXPECT_FLOAT_EQ(torque(deflected)(1), 0.f);
}

TEST(AirshipManualInputTest, UnavailableChannelReadsAsReleased)
{
	// The message marks a channel without data as NaN: released, and the
	// throttle at idle rather than half
	const manual_control_setpoint_s unavailable = sticks(NAN, NAN, NAN, NAN);
	EXPECT_FLOAT_EQ(thrust(unavailable)(0), 0.f);
	EXPECT_FLOAT_EQ(thrust(unavailable)(2), 0.f);
	EXPECT_FLOAT_EQ(torque(unavailable)(0), 0.f);
	EXPECT_FLOAT_EQ(torque(unavailable)(1), 0.f);
	EXPECT_FLOAT_EQ(torque(unavailable)(2), 0.f);
}

TEST(AirshipManualInputTest, ReleasedStickCommandsZero)
{
	// Anything inside the deadzone, band edge included, is exactly zero
	EXPECT_FLOAT_EQ(yawRateSetpoint(0.f, kDeadzone, kMaxRate), 0.f);
	EXPECT_FLOAT_EQ(yawRateSetpoint(0.05f, kDeadzone, kMaxRate), 0.f);
	EXPECT_FLOAT_EQ(yawRateSetpoint(-0.099f, kDeadzone, kMaxRate), 0.f);
	EXPECT_FLOAT_EQ(yawRateSetpoint(kDeadzone, kDeadzone, kMaxRate), 0.f);
	EXPECT_FLOAT_EQ(yawRateSetpoint(-kDeadzone, kDeadzone, kMaxRate), 0.f);
}

TEST(AirshipManualInputTest, LinearAndContinuousOutsideDeadzone)
{
	// Just outside the band the output leaves zero continuously
	EXPECT_NEAR(yawRateSetpoint(0.1001f, kDeadzone, kMaxRate), 0.f, 1e-3f);
	// Mid travel is rescaled over the remaining range: (0.55 - 0.1) / 0.9 = 0.5
	EXPECT_NEAR(yawRateSetpoint(0.55f, kDeadzone, kMaxRate), 0.5f * kMaxRate, 1e-5f);
	// Full stick reaches the full rate, symmetric in sign
	EXPECT_FLOAT_EQ(yawRateSetpoint(1.f, kDeadzone, kMaxRate), kMaxRate);
	EXPECT_FLOAT_EQ(yawRateSetpoint(-1.f, kDeadzone, kMaxRate), -kMaxRate);
	EXPECT_FLOAT_EQ(yawRateSetpoint(-0.55f, kDeadzone, kMaxRate),
			-yawRateSetpoint(0.55f, kDeadzone, kMaxRate));
}

TEST(AirshipManualInputTest, NoDeadzoneIsPlainLinear)
{
	EXPECT_FLOAT_EQ(yawRateSetpoint(0.25f, 0.f, kMaxRate), 0.25f * kMaxRate);
}

TEST(AirshipManualInputTest, NonFiniteStickReadsAsReleased)
{
	EXPECT_FLOAT_EQ(yawRateSetpoint(NAN, kDeadzone, kMaxRate), 0.f);
	EXPECT_FLOAT_EQ(yawRateSetpoint(INFINITY, kDeadzone, kMaxRate), 0.f);
}

TEST(AirshipManualInputTest, ThePilotIsInCommandOnlyArmedWithValidSticks)
{
	vehicle_control_mode_s mode{};
	manual_control_setpoint_s valid = sticks(0.f, 0.f, 0.f, 0.f);
	valid.valid = true;

	EXPECT_FALSE(manualInputUsable(mode, valid));	// disarmed

	mode.flag_armed = true;
	EXPECT_TRUE(manualInputUsable(mode, valid));

	manual_control_setpoint_s lost = valid;
	lost.valid = false;				// the link dropped: last values stand, .valid clears
	EXPECT_FALSE(manualInputUsable(mode, lost));
	EXPECT_FALSE(manualInputUsable(mode, manual_control_setpoint_s{}));	// boot: nothing published yet
}

TEST(AirshipManualInputTest, WrenchPublishedOnlyWhereNoOtherPublisherOwnsIt)
{
	// the flags commander sets per mode (ModeUtil/control_mode.cpp, setpoint_types.cpp);
	// an airship is not rotary wing, so its Manual enables no rate or attitude control
	vehicle_control_mode_s mode{};
	mode.flag_control_manual_enabled = true;
	mode.flag_control_allocation_enabled = true;
	EXPECT_TRUE(wrenchPublished(mode));	// Manual

	mode.flag_control_rates_enabled = true;
	EXPECT_TRUE(wrenchPublished(mode));	// Acro

	mode.flag_control_attitude_enabled = true;
	EXPECT_TRUE(wrenchPublished(mode));	// Stabilized, Altitude, Position

	mode.flag_control_manual_enabled = false;
	EXPECT_TRUE(wrenchPublished(mode));	// Hold, Mission, Land, Descend: nothing else drives the airship

	mode = vehicle_control_mode_s{};
	mode.flag_control_offboard_enabled = true;
	mode.flag_control_allocation_enabled = true;
	mode.flag_control_rates_enabled = true;
	EXPECT_TRUE(wrenchPublished(mode));	// Offboard body rates: no consumer here, the sticks stay torque

	mode.flag_control_rates_enabled = false;
	EXPECT_FALSE(wrenchPublished(mode));	// Offboard thrust and torque: the offboard publisher owns the topics

	mode.flag_control_offboard_enabled = false;
	EXPECT_FALSE(wrenchPublished(mode));	// an external mode sending thrust and torque, no offboard flag

	mode = vehicle_control_mode_s{};
	mode.flag_control_offboard_enabled = true;
	EXPECT_FALSE(wrenchPublished(mode));	// Offboard actuator setpoints: no allocation

	EXPECT_FALSE(wrenchPublished(vehicle_control_mode_s{}));	// Termination: every flag cleared
}

TEST(AirshipManualInputTest, LoopClosesInManualRateModes)
{
	vehicle_control_mode_s mode{};
	mode.flag_control_manual_enabled = true;
	mode.flag_control_rates_enabled = true;
	EXPECT_TRUE(yawRateLoopActive(mode, true, true)); // Acro

	// Stabilized (and Altitude, Position): no heading loop exists yet, so
	// the stick still feeds the rate loop
	mode.flag_control_attitude_enabled = true;
	EXPECT_TRUE(yawRateLoopActive(mode, true, true));
}

TEST(AirshipManualInputTest, LoopStaysOpenWithoutRatesOrPilot)
{
	vehicle_control_mode_s mode{};
	mode.flag_control_manual_enabled = true;
	EXPECT_FALSE(yawRateLoopActive(mode, true, true)); // Manual: rates off, torque passthrough

	mode.flag_control_rates_enabled = true;
	mode.flag_control_manual_enabled = false;
	// Hold, Land, Descend: manual flag off, loop open; the sticks still pass through as torque
	// (manualInputUsable() is armed && valid, it does not read this flag)
	EXPECT_FALSE(yawRateLoopActive(mode, true, true));

	mode.flag_control_manual_enabled = true;
	EXPECT_FALSE(yawRateLoopActive(mode, false, true)); // disarmed or lost sticks
}

TEST(AirshipManualInputTest, SurfaceOnlyYawKeepsTheStickAsTorque)
{
	// Collective pods, no tail: yaw is the rudders', which need airspeed
	vehicle_control_mode_s mode{};
	mode.flag_control_manual_enabled = true;
	mode.flag_control_rates_enabled = true;
	EXPECT_FALSE(yawRateLoopActive(mode, true, false)); // Acro

	mode.flag_control_attitude_enabled = true;
	EXPECT_FALSE(yawRateLoopActive(mode, true, false)); // Stabilized
}

TEST(AirshipManualInputTest, PropulsiveYawFromPodsOrTail)
{
	// the airship allocator, no yaw surfaces
	EXPECT_FALSE(hasPropulsiveYaw(kCaAirframeAirship, 0, false, false, 1.f));	// collective, no tail
	EXPECT_TRUE(hasPropulsiveYaw(kCaAirframeAirship, 1, false, false, 1.f));	// independent pods: 2520
	EXPECT_TRUE(hasPropulsiveYaw(kCaAirframeAirship, 0, true, false, 1.f));	// a tail thruster: 2507
	EXPECT_TRUE(hasPropulsiveYaw(kCaAirframeAirship, 2, false, false, 1.f));	// the allocator reads any GRP > 0 as independent
}

TEST(AirshipManualInputTest, FullyCreditedYawSurfacesKeepTheLoopOpen)
{
	// rudders credited in full carry the yaw first; the pods and the tail get only what they cannot deliver
	EXPECT_FALSE(hasPropulsiveYaw(kCaAirframeAirship, 0, false, true, 1.f));	// 2500: collective pods, rudders
	EXPECT_FALSE(hasPropulsiveYaw(kCaAirframeAirship, 1, false, true, 1.f));	// independent pods with rudders at CS_K 1
	EXPECT_FALSE(hasPropulsiveYaw(kCaAirframeAirship, 0, true, true, 1.f));	// a tail with rudders at CS_K 1
}

TEST(AirshipManualInputTest, SurfaceCreditBelowOneClosesTheLoopOnlyWithPodsOrTail)
{
	// below full credit the pods or the tail get the uncredited share; with neither the yaw is the surfaces' alone
	EXPECT_TRUE(hasPropulsiveYaw(kCaAirframeAirship, 1, false, true, 0.5f));	// half the yaw left to the pods
	EXPECT_TRUE(hasPropulsiveYaw(kCaAirframeAirship, 0, true, true, 0.5f));	// half the yaw left to the tail
	EXPECT_TRUE(hasPropulsiveYaw(kCaAirframeAirship, 1, false, true, 0.999f));	// full credit only at 1
	EXPECT_TRUE(hasPropulsiveYaw(kCaAirframeAirship, 1, false, true, 0.f));	// all of it left to the pods
	EXPECT_FALSE(hasPropulsiveYaw(kCaAirframeAirship, 0, false, true, 0.5f));	// nothing to leave it to
}

TEST(AirshipManualInputTest, YawSurfacesAsTheAllocatorSeesThem)
{
	const int32_t rudders[2] {4, 4};
	const float half[2] {0.5f, 0.5f};
	EXPECT_TRUE(hasYawSurfaces(2, rudders, half));		// a rudder pair
	EXPECT_FALSE(hasYawSurfaces(0, rudders, half));		// no surfaces configured

	const float reversed[2] {-0.5f, 0.f};
	EXPECT_TRUE(hasYawSurfaces(2, rudders, reversed));		// a rudder reversed through its sign

	// the allocator's row rule, ActuatorEffectiveness::kMinEffectiveness (0.05)
	const float min_effectiveness = ActuatorEffectiveness::kMinEffectiveness;

	const float small[2] {min_effectiveness, -0.8f * min_effectiveness};
	EXPECT_FALSE(hasYawSurfaces(2, rudders, small));		// every entry at most the minimum: the allocator drops the row

	const float just_above[1] {min_effectiveness + 0.001f};
	EXPECT_TRUE(hasYawSurfaces(1, rudders, just_above));	// above it: the row is kept

	const int32_t flaps[2] {9, 11};
	EXPECT_FALSE(hasYawSurfaces(2, flaps, half));		// a flap and an airbrake get no torque

	const int32_t elevators[2] {3, 3};
	const float none[2] {0.f, 0.f};
	EXPECT_FALSE(hasYawSurfaces(2, elevators, none));		// elevators with no yaw entry

	const int32_t layout_2500[4] {3, 3, 4, 4};
	const float yaw_2500[4] {0.f, 0.f, 0.5f, 0.5f};
	EXPECT_TRUE(hasYawSurfaces(4, layout_2500, yaw_2500));	// 2500: the elevators first, then the rudders

	// there are no parameters past kControlSurfaceMaxCount: the last surface counts, one past it is not read
	int32_t many_rudders[kControlSurfaceMaxCount + 1];
	float yaw_at_last[kControlSurfaceMaxCount + 1] {};
	float yaw_past_last[kControlSurfaceMaxCount + 1] {};

	for (int32_t &surface_type : many_rudders) {
		surface_type = 4;
	}

	yaw_at_last[kControlSurfaceMaxCount - 1] = 0.5f;
	yaw_past_last[kControlSurfaceMaxCount] = 0.5f;
	EXPECT_TRUE(hasYawSurfaces(kControlSurfaceMaxCount, many_rudders, yaw_at_last));
	EXPECT_FALSE(hasYawSurfaces(kControlSurfaceMaxCount + 1, many_rudders, yaw_past_last));
}

TEST(AirshipManualInputTest, OnlyTorqueBearingSurfaceTypesTakeYaw)
{
	const float one_yaw[1] {0.5f};

	// flaps, the airbrake, the steering wheel and spoilers: the allocator zeroes their torque
	const int32_t no_torque[] {9, 10, 11, 16, 17, 18};

	for (const int32_t surface_type : no_torque) {
		const int32_t one[1] {surface_type};
		EXPECT_FALSE(hasYawSurfaces(1, one, one_yaw)) << "type " << surface_type;
	}

	// every other type keeps its yaw entry, (Not set) and a value past the list included
	const int32_t torque_bearing[] {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 13, 14, 15, 19};

	for (const int32_t surface_type : torque_bearing) {
		const int32_t one[1] {surface_type};
		EXPECT_TRUE(hasYawSurfaces(1, one, one_yaw)) << "type " << surface_type;
	}
}

TEST(AirshipManualInputTest, OtherAllocatorsKeepTheLoop)
{
	// the gazebo-classic Cloudship: custom rotors (CA_AIRFRAME 9), a tail rotor among them
	EXPECT_TRUE(hasPropulsiveYaw(9, 0, false, false, 1.f));
	EXPECT_TRUE(hasPropulsiveYaw(9, 0, false, true, 1.f));	// yaw surfaces are read only for the airship allocator
}

TEST(AirshipManualInputTest, AirshipAllocatorIsCaAirframeSixteen)
{
	// CA_AIRFRAME "16: Airship" in control_allocator/module.yaml is a hand copy
	// of this constant that no build step checks: renumbering the constant
	// fails here, renumbering the yaml does not
	EXPECT_EQ(kCaAirframeAirship, 16);
}
