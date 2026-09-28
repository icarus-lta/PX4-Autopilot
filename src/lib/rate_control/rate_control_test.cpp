/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
#include <lib/rate_control/rate_control.hpp>

using namespace matrix;

TEST(RateControlTest, AllZeroCase)
{
	RateControl rate_control;
	Vector3f torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.f, false);
	EXPECT_EQ(torque, Vector3f());
}

TEST(RateControlTest, ZeroIntegratorLimitDisablesI)
{
	// I only, the integrator limit left at its default of zero
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f());
	const Vector3f rate_sp(1.f, 1.f, 1.f);
	rate_ctrl_status_s status{};

	for (int i = 0; i < 10; i++) {
		rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.01f, false);
	}

	rate_control.getRateControlStatus(status);
	EXPECT_FLOAT_EQ(status.yawspeed_integ, 0.f);
	EXPECT_EQ(rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.01f, false), Vector3f());

	// with a limit the same cycles integrate (about 0.098 with the I factor)
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));

	for (int i = 0; i < 10; i++) {
		rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.01f, false);
	}

	rate_control.getRateControlStatus(status);
	EXPECT_GT(status.yawspeed_integ, 0.09f);

	// and an explicit zero limit clamps the integral back to zero
	rate_control.setIntegratorLimit(Vector3f());
	rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.01f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_FLOAT_EQ(status.yawspeed_integ, 0.f);
}

TEST(RateControlTest, PositiveSaturationBlocksPositiveIntegrationOnly)
{
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f());
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_ctrl_status_s status{};

	// yaw saturated positive, set as a whole vector the way mc_rate_control does
	Vector<bool, 3> positive;
	Vector<bool, 3> negative;
	positive(2) = true;
	rate_control.setSaturationStatus(positive, negative);

	// a positive yaw error does not integrate
	rate_control.update(Vector3f(), Vector3f(0.f, 0.f, 1.f), Vector3f(), 0.01f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_FLOAT_EQ(status.yawspeed_integ, 0.f);

	// a negative one unwinds
	rate_control.update(Vector3f(), Vector3f(0.f, 0.f, -1.f), Vector3f(), 0.01f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_LT(status.yawspeed_integ, 0.f);

	// the other axes carry no flag
	rate_control.update(Vector3f(), Vector3f(1.f, 1.f, 0.f), Vector3f(), 0.01f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_GT(status.rollspeed_integ, 0.f);
	EXPECT_GT(status.pitchspeed_integ, 0.f);
}

TEST(RateControlTest, NegativeSaturationBlocksNegativeIntegrationOnly)
{
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f());
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_ctrl_status_s status{};

	// the per-axis setter writes the same flag
	rate_control.setNegativeSaturationFlag(2, true);

	// a negative yaw error does not integrate
	rate_control.update(Vector3f(), Vector3f(0.f, 0.f, -1.f), Vector3f(), 0.01f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_FLOAT_EQ(status.yawspeed_integ, 0.f);

	// a positive one unwinds
	rate_control.update(Vector3f(), Vector3f(0.f, 0.f, 1.f), Vector3f(), 0.01f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_GT(status.yawspeed_integ, 0.f);
}

TEST(RateControlTest, LandedFlagGatesIntegration)
{
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f());
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	const Vector3f rate_sp(0.f, 0.f, 1.f);
	rate_ctrl_status_s status{};

	// landed: P acts, I does not move
	EXPECT_FLOAT_EQ(rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.01f, true)(2), 1.f);
	EXPECT_FLOAT_EQ(rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.01f, true)(2), 1.f);
	rate_control.getRateControlStatus(status);
	EXPECT_FLOAT_EQ(status.yawspeed_integ, 0.f);

	// not landed: I moves, and the next output carries it
	rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.01f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_GT(status.yawspeed_integ, 0.f);
	EXPECT_GT(rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.01f, false)(2), 1.f);
}

TEST(RateControlTest, ResetIntegralClearsStatus)
{
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f());
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_ctrl_status_s status{};

	rate_control.update(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f(), 0.01f, false);
	rate_control.getRateControlStatus(status);
	EXPECT_GT(status.rollspeed_integ, 0.f);
	EXPECT_GT(status.pitchspeed_integ, 0.f);
	EXPECT_GT(status.yawspeed_integ, 0.f);

	// one axis
	rate_control.resetIntegral(2);
	rate_control.getRateControlStatus(status);
	EXPECT_GT(status.rollspeed_integ, 0.f);
	EXPECT_GT(status.pitchspeed_integ, 0.f);
	EXPECT_FLOAT_EQ(status.yawspeed_integ, 0.f);

	// all axes, and the next output carries no integral
	rate_control.resetIntegral();
	rate_control.getRateControlStatus(status);
	EXPECT_FLOAT_EQ(status.rollspeed_integ, 0.f);
	EXPECT_FLOAT_EQ(status.pitchspeed_integ, 0.f);
	EXPECT_FLOAT_EQ(status.yawspeed_integ, 0.f);
	EXPECT_EQ(rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false), Vector3f());
}
