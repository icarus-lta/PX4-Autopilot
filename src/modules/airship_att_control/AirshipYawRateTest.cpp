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

#include "airship_yaw_rate.hpp"

using namespace airship_yaw_rate;

static constexpr float kMaxRate = 0.2618f; // 15 deg/s
static constexpr float kDeadzone = 0.1f;

TEST(AirshipYawRateTest, ReleasedStickCommandsZero)
{
	// Anything inside the deadzone, band edge included, is exactly zero
	EXPECT_FLOAT_EQ(setpointFromStick(0.f, kDeadzone, kMaxRate), 0.f);
	EXPECT_FLOAT_EQ(setpointFromStick(0.05f, kDeadzone, kMaxRate), 0.f);
	EXPECT_FLOAT_EQ(setpointFromStick(-0.099f, kDeadzone, kMaxRate), 0.f);
	EXPECT_FLOAT_EQ(setpointFromStick(kDeadzone, kDeadzone, kMaxRate), 0.f);
}

TEST(AirshipYawRateTest, LinearAndContinuousOutsideDeadzone)
{
	// Just outside the band the output leaves zero continuously
	EXPECT_NEAR(setpointFromStick(0.1001f, kDeadzone, kMaxRate), 0.f, 1e-3f);
	// Mid travel is rescaled over the remaining range: (0.55 - 0.1) / 0.9 = 0.5
	EXPECT_NEAR(setpointFromStick(0.55f, kDeadzone, kMaxRate), 0.5f * kMaxRate, 1e-5f);
	// Full stick reaches the full rate, symmetric in sign
	EXPECT_FLOAT_EQ(setpointFromStick(1.f, kDeadzone, kMaxRate), kMaxRate);
	EXPECT_FLOAT_EQ(setpointFromStick(-1.f, kDeadzone, kMaxRate), -kMaxRate);
	EXPECT_FLOAT_EQ(setpointFromStick(-0.55f, kDeadzone, kMaxRate),
			-setpointFromStick(0.55f, kDeadzone, kMaxRate));
}

TEST(AirshipYawRateTest, NoDeadzoneIsPlainLinear)
{
	EXPECT_FLOAT_EQ(setpointFromStick(0.25f, 0.f, kMaxRate), 0.25f * kMaxRate);
}

TEST(AirshipYawRateTest, NonFiniteStickReadsAsReleased)
{
	EXPECT_FLOAT_EQ(setpointFromStick(NAN, kDeadzone, kMaxRate), 0.f);
	EXPECT_FLOAT_EQ(setpointFromStick(INFINITY, kDeadzone, kMaxRate), 0.f);
}

TEST(AirshipYawRateTest, LoopClosesInManualRateModes)
{
	vehicle_control_mode_s mode{};
	mode.flag_control_manual_enabled = true;
	mode.flag_control_rates_enabled = true;
	EXPECT_TRUE(loopActive(mode, true)); // Acro

	// Stabilized (and Altitude, Position): no heading loop exists yet, so
	// the stick still feeds the rate loop
	mode.flag_control_attitude_enabled = true;
	EXPECT_TRUE(loopActive(mode, true));
}

TEST(AirshipYawRateTest, LoopStaysOpenWithoutRatesOrPilot)
{
	vehicle_control_mode_s mode{};
	mode.flag_control_manual_enabled = true;
	EXPECT_FALSE(loopActive(mode, true)); // Manual: rates off, torque passthrough

	mode.flag_control_rates_enabled = true;
	mode.flag_control_manual_enabled = false;
	EXPECT_FALSE(loopActive(mode, true)); // Hold, Land, Descend: no pilot input mixed in

	mode.flag_control_manual_enabled = true;
	EXPECT_FALSE(loopActive(mode, false)); // disarmed or lost sticks
}
