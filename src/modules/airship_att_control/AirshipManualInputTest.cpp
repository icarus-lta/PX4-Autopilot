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
	EXPECT_FLOAT_EQ(thrust(sticks(0.f, 0.f, 0.f, 1.f))(1), 0.f);
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
