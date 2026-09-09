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

#include "AirshipPod.hpp"

using namespace matrix;

static constexpr float kDt = 0.02f;

static AirshipPod pod(float tilt_min_deg, float tilt_max_deg, float slew_rate_deg_s = 0.f)
{
	AirshipPod pod;
	pod.setTiltRange(math::radians(tilt_min_deg), math::radians(tilt_max_deg));
	pod.setTiltSlewRate(math::radians(slew_rate_deg_s));
	return pod;
}

static Vector2f force(float forward, float up)
{
	return Vector2f{forward, up};
}

TEST(AirshipPodTest, SteerEngagesAboveTheFloor)
{
	AirshipPod full = pod(-180.f, 180.f);

	// Below the engage floor the direction is undefined: the tilt stays level
	full.steer(force(0.019f, 0.f), kDt);
	EXPECT_FLOAT_EQ(full.tilt(), 0.f);

	full.steer(force(0.f, 0.021f), kDt);
	EXPECT_FLOAT_EQ(full.tilt(), math::radians(90.f));
}

TEST(AirshipPodTest, DemandAtTheReleaseThresholdReleases)
{
	AirshipPod slewing = pod(-180.f, 180.f, 90.f);
	slewing.steer(force(0.f, 1.f), kDt);
	const float first_step = slewing.tilt();
	EXPECT_GT(first_step, 0.f);

	// Exactly at the release threshold the hold is not kept (strict
	// comparison): the target drops to the current tilt and the slew stops
	slewing.steer(force(0.f, AirshipPod::kSteerRelease), kDt);
	slewing.steer(force(0.f, AirshipPod::kSteerRelease), kDt);
	EXPECT_FLOAT_EQ(slewing.tilt(), first_step);
	EXPECT_TRUE(slewing.heldByChoice());
}

TEST(AirshipPodTest, ReversalInsideTheBandDoesNotRetarget)
{
	AirshipPod full = pod(-180.f, 180.f);
	full.steer(force(0.f, 0.5f), kDt);
	EXPECT_FLOAT_EQ(full.tilt(), math::radians(90.f));

	// A reversed demand between release and engage holds the target
	full.steer(force(0.f, -0.015f), kDt);
	EXPECT_FLOAT_EQ(full.tilt(), math::radians(90.f));
	EXPECT_TRUE(full.heldByChoice());

	// Above the engage floor it is a real demand
	full.steer(force(0.f, -0.5f), kDt);
	EXPECT_FLOAT_EQ(full.tilt(), math::radians(-90.f));
}

TEST(AirshipPodTest, HoldingKeepsSlewingAndReleaseFreezes)
{
	AirshipPod slewing = pod(-180.f, 180.f, 90.f);
	slewing.steer(force(-1.f, 0.f), kDt);
	const float first_step = slewing.tilt();
	EXPECT_NEAR(first_step, math::radians(1.8f), 1e-6f);

	// Inside the band the target stands, so the tilt keeps moving toward it
	slewing.steer(force(-0.015f, 0.f), kDt);
	EXPECT_NEAR(slewing.tilt(), math::radians(3.6f), 1e-6f);
	EXPECT_FALSE(slewing.heldByChoice()); // not settled yet

	// Released mid-slew: the tilt freezes where it is
	slewing.steer(force(0.f, 0.f), kDt);
	slewing.steer(force(0.f, 0.f), kDt);
	EXPECT_NEAR(slewing.tilt(), math::radians(3.6f), 1e-6f);
	EXPECT_TRUE(slewing.heldByChoice());
}

TEST(AirshipPodTest, NanDemandReleasesAndLeavesTheStateFinite)
{
	AirshipPod full = pod(-180.f, 180.f);
	full.steer(force(0.f, 1.f), kDt);

	full.steer(force(NAN, 0.f), kDt);
	EXPECT_TRUE(PX4_ISFINITE(full.tilt()));
	EXPECT_FLOAT_EQ(full.tilt(), math::radians(90.f));

	// The next real demand behaves as if the NaN never happened
	full.steer(force(0.f, 1.f), kDt);
	EXPECT_FLOAT_EQ(full.tilt(), math::radians(90.f));
}

TEST(AirshipPodTest, RearDemandFromLevelPicksThePositiveEnd)
{
	AirshipPod full = pod(-180.f, 180.f);
	full.steer(force(-1.f, 0.f), kDt);
	EXPECT_FLOAT_EQ(full.tilt(), M_PI_F);
	EXPECT_FLOAT_EQ(full.servoSetpoint(), 1.f);
}

TEST(AirshipPodTest, RearConeNoiseKeepsTheCommittedEnd)
{
	AirshipPod full = pod(-180.f, 180.f);
	full.steer(force(-1.f, 0.f), kDt);

	// Perpendicular noise inside the cone must not flip across the range
	for (const float up : {0.02f, -0.02f, 0.02f}) {
		full.steer(force(-1.f, up), kDt);
		EXPECT_FLOAT_EQ(full.tilt(), M_PI_F);
	}

	// Committed to the negative side, the tie keeps that side
	AirshipPod negative = pod(-180.f, 180.f);
	negative.steer(force(-1.f, -0.2f), kDt);
	EXPECT_LT(negative.tilt(), 0.f);
	negative.steer(force(-1.f, 0.02f), kDt);
	EXPECT_FLOAT_EQ(negative.tilt(), -M_PI_F);
}

TEST(AirshipPodTest, RearDemandOnADownOnlyRangePicksTheReachableEnd)
{
	AirshipPod down_only = pod(-180.f, 0.f);
	down_only.steer(force(-1.f, 0.f), kDt);
	EXPECT_FLOAT_EQ(down_only.tilt(), -M_PI_F);
}

TEST(AirshipPodTest, NoBackwardEndHoldsTheTilt)
{
	// -90..90: neither end points backward, so a rear demand realizes
	// nothing at either and the tilt stays where it is
	AirshipPod symmetric = pod(-90.f, 90.f);
	symmetric.steer(force(cosf(math::radians(30.f)), sinf(math::radians(30.f))), kDt);
	EXPECT_FLOAT_EQ(symmetric.tilt(), math::radians(30.f));

	symmetric.steer(force(-1.f, 0.f), kDt);
	EXPECT_FLOAT_EQ(symmetric.tilt(), math::radians(30.f));
}

TEST(AirshipPodTest, OutOfRangeDemandSwitchesEndsOnlyPastTheMargin)
{
	// Down-only range, level: straight up is unreachable at either end
	AirshipPod down_only = pod(-180.f, 0.f);
	down_only.steer(force(0.f, 1.f), kDt);
	EXPECT_FLOAT_EQ(down_only.tilt(), 0.f);

	// Up and slightly back: the -180 end realizes 0.04, less than the margin
	down_only.steer(force(-0.04f, 1.f), kDt);
	EXPECT_FLOAT_EQ(down_only.tilt(), 0.f);

	// Past the margin the other end wins
	down_only.steer(force(-0.3f, 1.f), kDt);
	EXPECT_FLOAT_EQ(down_only.tilt(), -M_PI_F);
}

TEST(AirshipPodTest, SlewMovesRateTimesDt)
{
	AirshipPod slewing = pod(-180.f, 180.f, 90.f);
	slewing.steer(force(0.f, 1.f), kDt);
	EXPECT_NEAR(slewing.tilt(), math::radians(90.f) * kDt, 1e-6f);

	AirshipPod unlimited = pod(-180.f, 180.f);
	unlimited.steer(force(0.f, 1.f), kDt);
	EXPECT_FLOAT_EQ(unlimited.tilt(), math::radians(90.f));
}

TEST(AirshipPodTest, ParkLevelsWithinTheRange)
{
	AirshipPod up_only = pod(0.f, 90.f);
	up_only.steer(force(0.f, 1.f), kDt);
	up_only.park(kDt);
	EXPECT_FLOAT_EQ(up_only.tilt(), 0.f);
	EXPECT_FALSE(up_only.heldByChoice());

	AirshipPod down_only = pod(-180.f, -30.f);
	down_only.park(kDt);
	EXPECT_FLOAT_EQ(down_only.tilt(), math::radians(-30.f));
}

TEST(AirshipPodTest, ServoRoundTripAndClampReadBack)
{
	AirshipPod full = pod(-180.f, 180.f);
	full.steer(force(-1.f, 0.f), kDt);
	EXPECT_FLOAT_EQ(full.servoSetpoint(), 1.f);

	// A servo output clamped at 0.5 realizes +90 deg
	full.setServoSetpoint(0.5f);
	EXPECT_FLOAT_EQ(full.tilt(), math::radians(90.f));
	EXPECT_FLOAT_EQ(full.servoSetpoint(), 0.5f);
}

TEST(AirshipPodTest, RangeNarrowingClampsTheStateAndTheTarget)
{
	AirshipPod full = pod(-180.f, 180.f);
	full.steer(force(-1.f, 0.f), kDt);
	EXPECT_FLOAT_EQ(full.tilt(), M_PI_F);

	// Collapsed to a forward fixed mount: the tilt follows, and holds there
	full.setTiltRange(0.f, 0.f);
	EXPECT_FALSE(full.canTilt());
	EXPECT_FLOAT_EQ(full.tilt(), 0.f);
	full.steer(force(0.f, 0.f), kDt);
	EXPECT_FLOAT_EQ(full.tilt(), 0.f);

	// An inverted pair is a fixed mount at the minimum
	full.setTiltRange(math::radians(30.f), math::radians(-30.f));
	EXPECT_FALSE(full.canTilt());
	EXPECT_FLOAT_EQ(full.tilt(), math::radians(30.f));
}

TEST(AirshipPodTest, HeldByChoiceTruthTable)
{
	AirshipPod full = pod(-180.f, 180.f);
	full.steer(force(0.f, 1.f), kDt);
	EXPECT_FALSE(full.heldByChoice()); // steering

	full.steer(force(0.f, 0.015f), kDt);
	EXPECT_TRUE(full.heldByChoice()); // holding, settled

	full.steer(force(0.f, 0.f), kDt);
	EXPECT_TRUE(full.heldByChoice()); // released, settled

	full.park(kDt);
	EXPECT_FALSE(full.heldByChoice()); // parked

	// A fixed mount withholds nothing by choice: holding and settled, but not held
	AirshipPod fixed = pod(0.f, 0.f);
	fixed.steer(force(0.f, 1.f), kDt);
	fixed.steer(force(0.f, 0.015f), kDt);
	EXPECT_FALSE(fixed.heldByChoice());
}
