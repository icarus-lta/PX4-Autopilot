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

TEST(AirshipManualInputTest, LoopClosesInManualRateModes)
{
	vehicle_control_mode_s mode{};
	mode.flag_control_manual_enabled = true;
	mode.flag_control_rates_enabled = true;
	EXPECT_TRUE(yawRateLoopActive(mode, true)); // Acro

	// Stabilized (and Altitude, Position): no heading loop exists yet, so
	// the stick still feeds the rate loop
	mode.flag_control_attitude_enabled = true;
	EXPECT_TRUE(yawRateLoopActive(mode, true));
}

TEST(AirshipManualInputTest, LoopStaysOpenWithoutRatesOrPilot)
{
	vehicle_control_mode_s mode{};
	mode.flag_control_manual_enabled = true;
	EXPECT_FALSE(yawRateLoopActive(mode, true)); // Manual: rates off, torque passthrough

	mode.flag_control_rates_enabled = true;
	mode.flag_control_manual_enabled = false;
	EXPECT_FALSE(yawRateLoopActive(mode, true)); // Hold, Land, Descend: manual input not mixed in

	mode.flag_control_manual_enabled = true;
	EXPECT_FALSE(yawRateLoopActive(mode, false)); // disarmed or lost sticks
}
