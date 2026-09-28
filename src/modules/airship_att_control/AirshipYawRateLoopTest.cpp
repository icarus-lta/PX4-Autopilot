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

#include "airship_yaw_rate_loop.hpp"

using namespace matrix;

static constexpr float kDt = 0.01f;

// I only, so the output is the integral from before the update
static AirshipYawRateLoop integratorOnly()
{
	AirshipYawRateLoop loop;
	loop.setGains(0.f, 1.f, 1.f, 0.f);
	return loop;
}

static float yawIntegral(AirshipYawRateLoop &loop)
{
	rate_ctrl_status_s status{};
	loop.getStatus(status);
	return status.yawspeed_integ;
}

static control_allocator_status_s unallocated(float roll, float pitch, float yaw)
{
	control_allocator_status_s status{};
	status.torque_setpoint_achieved = false;
	status.unallocated_torque[0] = roll;
	status.unallocated_torque[1] = pitch;
	status.unallocated_torque[2] = yaw;
	return status;
}

TEST(AirshipYawRateLoopTest, SaturationFlagsFollowTheShortfallSign)
{
	const AirshipYawRateLoop::SaturationFlags flags = AirshipYawRateLoop::saturationFlags(unallocated(1.f, -1.f, 0.f));
	EXPECT_TRUE(flags.positive(0));		// roll short upwards
	EXPECT_FALSE(flags.negative(0));
	EXPECT_FALSE(flags.positive(1));
	EXPECT_TRUE(flags.negative(1));		// pitch short downwards
	EXPECT_FALSE(flags.positive(2));	// yaw served
	EXPECT_FALSE(flags.negative(2));
}

TEST(AirshipYawRateLoopTest, NoSaturationWhereTheTorqueWasAchieved)
{
	// the allocator's own verdict gates the per-axis residuals
	control_allocator_status_s status = unallocated(1.f, -1.f, 1.f);
	status.torque_setpoint_achieved = true;
	const AirshipYawRateLoop::SaturationFlags flags = AirshipYawRateLoop::saturationFlags(status);

	for (int i = 0; i < 3; i++) {
		EXPECT_FALSE(flags.positive(i)) << "axis " << i;
		EXPECT_FALSE(flags.negative(i)) << "axis " << i;
	}
}

TEST(AirshipYawRateLoopTest, PositiveShortfallStopsPositiveIntegrationOnly)
{
	AirshipYawRateLoop loop = integratorOnly();
	loop.setSaturation(unallocated(0.f, 0.f, 1.f));

	// asking for more positive yaw than the allocator delivers does not wind up
	loop.update(Vector3f(), 1.f, kDt);
	EXPECT_FLOAT_EQ(yawIntegral(loop), 0.f);

	// the other direction still integrates
	loop.update(Vector3f(), -1.f, kDt);
	EXPECT_LT(yawIntegral(loop), 0.f);
}

TEST(AirshipYawRateLoopTest, NegativeShortfallStopsNegativeIntegrationOnly)
{
	AirshipYawRateLoop loop = integratorOnly();
	loop.setSaturation(unallocated(0.f, 0.f, -1.f));

	loop.update(Vector3f(), -1.f, kDt);
	EXPECT_FLOAT_EQ(yawIntegral(loop), 0.f);

	loop.update(Vector3f(), 1.f, kDt);
	EXPECT_GT(yawIntegral(loop), 0.f);
}

TEST(AirshipYawRateLoopTest, IntegratorWindsUpToItsLimit)
{
	// the library's limit defaults to zero, which would hold the integral at zero
	AirshipYawRateLoop loop;
	loop.setGains(0.f, 1.f, 0.3f, 0.f);

	for (int i = 0; i < 100; i++) {
		loop.update(Vector3f(), 1.f, kDt);
	}

	EXPECT_FLOAT_EQ(yawIntegral(loop), 0.3f);
	EXPECT_FLOAT_EQ(loop.update(Vector3f(), 1.f, kDt), 0.3f);
}

TEST(AirshipYawRateLoopTest, FeedforwardHoldsTheTurnInsteadOfTheIntegrator)
{
	AirshipYawRateLoop loop;
	loop.setGains(0.5f, 1.f, 1.f, 2.f);

	// tracking a 0.1 rad/s turn: no rate error, so the torque is the
	// feedforward alone and the integrator is not asked to hold the turn
	for (int i = 0; i < 3; i++) {
		EXPECT_FLOAT_EQ(loop.update(Vector3f(0.f, 0.f, 0.1f), 0.1f, kDt), 0.2f);
	}

	EXPECT_FLOAT_EQ(yawIntegral(loop), 0.f);

	// stick released: the turn's torque goes the same cycle and P brakes the rate
	EXPECT_FLOAT_EQ(loop.update(Vector3f(0.f, 0.f, 0.1f), 0.f, kDt), -0.05f);
}

TEST(AirshipYawRateLoopTest, NonFiniteTorqueReadsAsZero)
{
	AirshipYawRateLoop loop;
	loop.setGains(0.5f, 0.09f, 0.3f, 0.f);
	EXPECT_FLOAT_EQ(loop.update(Vector3f(), 1.f, kDt), 0.5f);	// P on a 1 rad/s error

	// a non-finite gyro sample: no torque rather than NaN into the allocator
	EXPECT_FLOAT_EQ(loop.update(Vector3f(0.f, 0.f, NAN), 1.f, kDt), 0.f);
}

TEST(AirshipYawRateLoopTest, OpeningClearsTheIntegratorOnce)
{
	AirshipYawRateLoop loop = integratorOnly();
	EXPECT_FALSE(loop.open());	// open from the start: nothing to clear

	for (int i = 0; i < 3; i++) {
		loop.update(Vector3f(), 1.f, kDt);
	}

	EXPECT_GT(yawIntegral(loop), 0.f);

	// the first open cycle clears the integral and says so, the next ones do not
	EXPECT_TRUE(loop.open());
	EXPECT_FLOAT_EQ(yawIntegral(loop), 0.f);
	EXPECT_FALSE(loop.open());

	// closing again starts from zero, and the next opening is reported again
	EXPECT_FLOAT_EQ(loop.update(Vector3f(), 1.f, kDt), 0.f);
	EXPECT_TRUE(loop.open());
	EXPECT_FLOAT_EQ(yawIntegral(loop), 0.f);
}
