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

#include <random>

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

TEST(AirshipPodTest, DriveProjectsOntoTheRealizedAxisAndNeverReverses)
{
	AirshipPod full = pod(-180.f, 180.f);
	full.steer(force(0.f, 1.f), kDt);			// +90 deg: thrust up

	const AirshipPod::Drive up = full.drive(force(0.6f, 0.8f), 0.f, 1.f);
	EXPECT_FLOAT_EQ(up.thrust, 0.8f);			// only the component along the axis
	EXPECT_NEAR(up.achieved(0), 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(up.achieved(1), 0.8f);
	EXPECT_NEAR(up.withheld(0), 0.f, 1e-6f);		// steering: nothing withheld by choice,
	EXPECT_NEAR(up.withheld(1), 0.f, 1e-6f);		// not even the 0.6 the axis cannot serve

	// A demand behind the axis cannot be pushed, and a CA_R_REV bit on a pod
	// motor does not buy reverse either: the propeller reverses only by tilting
	EXPECT_FLOAT_EQ(full.drive(force(0.f, -0.5f), 0.f, 1.f).thrust, 0.f);
	EXPECT_FLOAT_EQ(full.drive(force(0.f, -0.5f), -1.f, 1.f).thrust, 0.f);
	EXPECT_FLOAT_EQ(pod(0.f, 0.f).drive(force(-1.f, 0.f), -1.f, 1.f).thrust, 0.f);

	// Held by choice: the whole residual, clamp shortfall included, is withheld
	full.steer(force(0.f, 0.015f), kDt);
	ASSERT_TRUE(full.heldByChoice());
	const AirshipPod::Drive held = full.drive(force(-0.01f, -0.5f), 0.f, 1.f);
	EXPECT_NEAR(held.thrust, 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(held.withheld(0), -0.01f);
	EXPECT_FLOAT_EQ(held.withheld(1), -0.5f);	// the clamp shortfall, not just the unserved axis
}

TEST(AirshipPodTest, RearConeRanksTheEndsByTheRangeAlone)
{
	// Straight back on an asymmetric range, both ends pointing backward: the
	// more backward end wins whichever way the perpendicular noise leans,
	// because the cone is a deadband and the noise is not a signal
	const float lo = math::radians(-100.f);

	for (const float noise : {0.03f, -0.03f}) {
		AirshipPod pod_a = pod(-100.f, 99.f, 120.f);

		for (int i = 0; i < 9000; i++) {
			pod_a.steer(force(-1.f, noise), kDt);
		}

		EXPECT_NEAR(pod_a.tilt(), lo, 1e-5f) << "noise " << noise;
	}

	// A symmetric range has both ends on one axis: the committed end stands
	AirshipPod pod_s = pod(-180.f, 180.f, 120.f);

	for (int i = 0; i < 9000; i++) {
		pod_s.steer(force(-1.f, 0.03f), kDt);
	}

	EXPECT_NEAR(pod_s.tilt(), math::radians(180.f), 1e-5f);
}

TEST(AirshipPodTest, RearConePickThatRealizesNothingHolds)
{
	// One end barely past vertical (92 deg realizes 3.5 % of straight back)
	// and a perpendicular component that cancels it: the pick would realize
	// nothing, so it is not worth a move, even from the same side of the
	// range where no sweep is priced
	AirshipPod nearly_vertical = pod(-88.f, 92.f);
	nearly_vertical.steer(force(0.985f, 0.174f), kDt);
	EXPECT_NEAR(nearly_vertical.tilt(), math::radians(10.f), 1e-3f);

	nearly_vertical.steer(force(-1.f, -0.0349f), kDt);
	EXPECT_NEAR(nearly_vertical.tilt(), math::radians(10.f), 1e-3f);

	// With the perpendicular the other way the end realizes 7 %: worth it
	nearly_vertical.steer(force(-1.f, 0.0349f), kDt);
	EXPECT_FLOAT_EQ(nearly_vertical.tilt(), math::radians(92.f));
}

TEST(AirshipPodTest, RearConeNoiseDoesNotDither)
{
	// Straight back with the perpendicular flipping sign every cycle inside
	// the cone, on a wide range where both ends point backward: ranked by the
	// demand the pick would flip with the noise and the servo would strand
	// mid-range; ranked by the range it reaches an end and stays
	AirshipPod wide = pod(-100.f, 100.f, 120.f);
	float previous = wide.tilt();
	float previous_step = 0.f;
	int reversals = 0;

	for (int i = 0; i < 500; i++) {
		wide.steer(force(-1.f, (i % 2 == 0) ? 0.03f : -0.03f), kDt);
		const float step = wide.tilt() - previous;

		if (step * previous_step < 0.f) {
			reversals++;
		}

		previous_step = step;
		previous = wide.tilt();
	}

	EXPECT_EQ(reversals, 0);
	EXPECT_NEAR(wide.tilt(), math::radians(100.f), 1e-5f);
}

TEST(AirshipPodTest, AsymmetricRangeDoesNotSweepForLessThrust)
{
	// An asymmetric range wider than half a turn. The pod commits to the high
	// end, then a straight-back demand puts the rear cone's pick at the other
	// end, half a turn away. The seam check has to score that end by what it
	// realizes: the demand's own magnitude is not what a range end delivers
	AirshipPod pod_a = pod(-100.f, 99.f, 120.f);

	for (int i = 0; i < 8000; i++) {
		pod_a.steer(force(-0.2f, 1.f), kDt);		// out of range past the high end
	}

	EXPECT_NEAR(pod_a.tilt(), math::radians(99.f), 1e-5f);

	const Vector2f rear = force(-1.f, 0.03f);		// inside the rear cone
	const float at_high = rear.dot(Vector2f{cosf(math::radians(99.f)), sinf(math::radians(99.f))});
	const float at_low = rear.dot(Vector2f{cosf(math::radians(-100.f)), sinf(math::radians(-100.f))});
	EXPECT_GT(at_high, at_low);				// the committed end is the better one

	for (int i = 0; i < 8000; i++) {
		pod_a.steer(rear, kDt);
	}

	EXPECT_NEAR(pod_a.tilt(), math::radians(99.f), 1e-5f);	// it stands rather than sweep for less
}

TEST(AirshipPodTest, SeamCrossingNeedsToBeWorthTheSweep)
{
	// A full turn has no range end to switch at, so the seam is the only place
	// a small change in demand can cost a whole servo travel
	AirshipPod full = pod(-180.f, 180.f, 120.f);

	for (int i = 0; i < 3000; i++) {
		full.steer(force(-1.f, 0.09f), kDt);	// just outside the rear cone, near +180 deg
	}

	const float committed = full.tilt();
	EXPECT_GT(committed, math::radians(170.f));

	// The perpendicular component flips: the demand direction moves ~10 deg,
	// but the far-side target is half a turn away and the committed tilt still
	// realizes almost all of it, so the pod stands
	for (int i = 0; i < 500; i++) {
		full.steer(force(-1.f, -0.09f), kDt);
	}

	EXPECT_FLOAT_EQ(full.tilt(), committed);

	// A demand that genuinely needs the other side still pays for the sweep
	for (int i = 0; i < 3000; i++) {
		full.steer(force(-1.f, -1.f), kDt);
	}

	EXPECT_NEAR(full.tilt(), math::radians(-135.f), 1e-5f);
}

TEST(AirshipPodTest, UnrealizableOutOfRangeDemandHoldsTheTilt)
{
	// Up-only range: a back-and-down demand is out of range and realizes
	// nothing at either end, so the tilt stays instead of sweeping a full
	// servo travel for zero thrust (as NoBackwardEndHoldsTheTilt)
	AirshipPod up_only = pod(0.f, 90.f);
	up_only.steer(force(cosf(math::radians(60.f)), sinf(math::radians(60.f))), kDt);
	EXPECT_FLOAT_EQ(up_only.tilt(), math::radians(60.f));

	up_only.steer(force(-0.5f, -0.5f), kDt);
	EXPECT_FLOAT_EQ(up_only.tilt(), math::radians(60.f));
	EXPECT_FALSE(up_only.heldByChoice());	// steering: the shortfall is real and reported
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

TEST(AirshipPodTest, RearConeSwitchesEndsOnlyPastTheMargin)
{
	// A servo that just passes vertical on one side: the ends are 180 deg
	// apart, so no target is ever across the seam, and only the positive end
	// points backward, by 3.5 % of a straight-back demand
	AirshipPod nearly_vertical = pod(-88.f, 92.f);
	nearly_vertical.steer(force(0.f, -1.f), kDt);
	EXPECT_FLOAT_EQ(nearly_vertical.tilt(), math::radians(-88.f));

	// Straight back is inside the rear cone: the positive end realizes more,
	// but by less than the margin, and reaching it sweeps the whole range
	nearly_vertical.steer(force(-1.f, 0.f), kDt);
	EXPECT_FLOAT_EQ(nearly_vertical.tilt(), math::radians(-88.f));

	// The same from a tilt left mid-range by tracking: the pick realizes
	// less than the margin wherever it starts
	AirshipPod mid_range = pod(-88.f, 92.f);
	mid_range.steer(force(0.05f, -1.f), kDt);
	EXPECT_NEAR(mid_range.tilt(), math::radians(-87.14f), 1e-3f);
	mid_range.steer(force(-1.f, 0.f), kDt);
	EXPECT_NEAR(mid_range.tilt(), math::radians(-87.14f), 1e-3f);

	// Three degrees further past vertical the positive end realizes 8.7 %:
	// past the margin, the sweep is paid for
	AirshipPod past_vertical = pod(-85.f, 95.f);
	past_vertical.steer(force(0.f, -1.f), kDt);
	past_vertical.steer(force(-1.f, 0.f), kDt);
	EXPECT_FLOAT_EQ(past_vertical.tilt(), math::radians(95.f));
}

TEST(AirshipPodTest, MarginBoundsEverySweepAndEveryHold)
{
	// The one rule every branch of the tilt policy keeps, both ways: a sweep,
	// from one range end to the other, across the seam, or a rear-cone pick
	// that leaves the committed target's side of the range, is paid for only
	// by a gain past the margin; and a hold forgoes no more than the margin
	// against the best angle in the range, where inside the rear cone the
	// candidates are the ends and the perpendicular component, noise there,
	// is not counted. Sampled over
	// the parameter range of tilt ranges and their corners, over committed
	// targets and over demands, so no branch can drift from the rule
	// unnoticed: each fix to the policy so far was one branch pricing a move
	// by a yardstick its siblings did not use, and this sample catches every
	// one of them
	std::mt19937 rng(42);
	auto uniform = [&rng](float lo, float hi) { return lo + (hi - lo) * ((rng() >> 8) / 16777216.f); };
	auto realized = [](const Vector2f & demand, float tilt) { return fmaxf(0.f, demand(0) * cosf(tilt) + demand(1) * sinf(tilt)); };
	int sweeps = 0;

	for (int i = 0; i < 200000; i++) {
		// Draw in statements: argument evaluation order is unspecified
		const float lo_deg = i % 7 == 0 ? -180.f : (i % 11 == 0 ? 0.f : uniform(-180.f, 0.f));
		const float hi_deg = i % 5 == 0 ? 180.f : (i % 13 == 0 ? 0.f : uniform(0.f, 180.f));
		const float tilt_min = math::radians(lo_deg);
		const float tilt_max = math::radians(hi_deg);
		AirshipPod sampled;
		sampled.setTiltRange(tilt_min, tilt_max);

		// Commit a target somewhere, then ask for something else
		const float settle_forward = uniform(-1.2f, 1.2f);
		const float settle_up = uniform(-1.2f, 1.2f);
		sampled.steer(force(settle_forward, settle_up), kDt);
		const float committed = sampled.tilt();
		const float forward = uniform(-1.2f, 1.2f);
		const float up = uniform(-1.2f, 1.2f);
		const Vector2f demand = force(forward, up);
		sampled.steer(demand, kDt);
		const float target = sampled.tilt();

		ASSERT_GE(target, tilt_min);
		ASSERT_LE(target, tilt_max);

		if (demand.norm() <= AirshipPod::kSteerEngage) {
			continue;	// not steered
		}

		const float margin = fmaxf(AirshipPod::kEndSwitchMargin * demand.norm(), AirshipPod::kSteerRelease);
		const float direction = atan2f(demand(1), demand(0));
		const bool in_range = direction >= tilt_min && direction <= tilt_max;
		const bool in_cone = demand(0) < 0.f && fabsf(demand(1)) < margin;
		const bool committed_hi = committed - tilt_min > tilt_max - committed;
		// "At an end" is the contract's width; kTiltSettledTolerance is private.
		// Tracking an in-range direction is free even when it lands on the
		// other end of a narrow range: only an end picked for an out-of-range
		// direction is a switch
		const bool from_lo = fabsf(committed - tilt_min) < 1e-3f;
		const bool from_hi = fabsf(committed - tilt_max) < 1e-3f;
		const bool end_switch = !in_range && !in_cone
					&& ((from_lo && !from_hi && fabsf(target - tilt_max) < 1e-3f)
					    || (from_hi && !from_lo && fabsf(target - tilt_min) < 1e-3f));
		const bool cone_across = in_cone && (committed_hi ? target < committed : target > committed);

		if (end_switch || cone_across || fabsf(target - committed) > M_PI_F) {
			sweeps++;
			EXPECT_GT(realized(demand, target), realized(demand, committed) + margin - 1e-5f)
					<< "unpaid sweep, sample " << i << ": range " << lo_deg << ".." << hi_deg
					<< " deg, demand (" << demand(0) << ", " << demand(1) << "), "
					<< math::degrees(committed) << " -> " << math::degrees(target) << " deg";
		}

		// The projection peaks once around the circle: the best angle in the
		// range is the demand direction if it is inside, else the better end.
		// Inside the cone the ends are the only candidates and the
		// perpendicular is noise: what it is worth at either end is not owed
		float best = fmaxf(realized(demand, tilt_min), realized(demand, tilt_max));
		float owed = margin;

		if (in_cone) {
			owed += 2.f * fabsf(demand(1));

		} else if (in_range) {
			best = fmaxf(best, demand.norm());
		}

		EXPECT_LE(best - realized(demand, target), owed + 1e-5f)
				<< "left on the table, sample " << i << ": range " << lo_deg << ".." << hi_deg
				<< " deg, demand (" << demand(0) << ", " << demand(1) << "), "
				<< math::degrees(committed) << " -> " << math::degrees(target) << " deg";

		// A constant demand is a fixed point
		sampled.steer(demand, kDt);
		EXPECT_FLOAT_EQ(sampled.tilt(), target);
	}

	EXPECT_GT(sweeps, 1000) << "the sample must actually exercise sweeps";
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
	// A pod still slewing toward a target keeps that target across the
	// range change, so the clamp on the target is observable: without it
	// the tilt walks to the stale angle and servoSetpoint() leaves [-1, 1]
	AirshipPod slewing = pod(-180.f, 180.f, 90.f);
	slewing.steer(force(-1.f, 0.f), kDt);
	slewing.setTiltRange(0.f, math::radians(10.f));

	for (int i = 0; i < 200; i++) {
		slewing.steer(force(-0.015f, 0.f), kDt);	// in the band: the target stands
	}

	EXPECT_FLOAT_EQ(slewing.tilt(), math::radians(10.f));
	EXPECT_FLOAT_EQ(slewing.servoSetpoint(), 1.f);

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
