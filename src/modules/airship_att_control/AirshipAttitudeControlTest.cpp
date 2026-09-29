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

/**
 * @file AirshipAttitudeControlTest.cpp
 *
 * Drives AirshipAttitudeControl::Run() through uORB: how the module wires its
 * parameters, the gates in airship_manual_input.hpp and AirshipYawRateLoop to
 * what it publishes, which their own unit tests do not reach.
 */

#include <gtest/gtest.h>

#include <memory>

#include "airship_att_control.hpp"

static constexpr hrt_abstime kGyroInterval = 4000;	// [us], 250 Hz

struct Sticks {
	float roll{0.f};
	float pitch{0.f};
	float yaw{0.f};
	float throttle{-1.f};
	bool valid{true};
};

static void setInt(const char *name, int32_t value) { param_set(param_find(name), &value); }
static void setFloat(const char *name, float value) { param_set(param_find(name), &value); }

// The airship allocator with independently driven pods: yaw from differential thrust
static void setIndependentPods()
{
	setInt("CA_AIRFRAME", kCaAirframeAirship);
	setInt("CA_AIRSHIP_GRP", 1);
}

static vehicle_control_mode_s acro()
{
	vehicle_control_mode_s mode{};
	mode.flag_armed = true;
	mode.flag_control_manual_enabled = true;
	mode.flag_control_rates_enabled = true;
	mode.flag_control_allocation_enabled = true;
	return mode;
}

static vehicle_control_mode_s manual()
{
	vehicle_control_mode_s mode = acro();
	mode.flag_control_rates_enabled = false;
	return mode;
}

// Each test builds the module after its own parameter setters, because the
// module reads them at construction. Run() is called directly: without the
// work queue manager the WorkItem only logs that it could not attach, as in
// ManualControlTest
class AirshipAttitudeControlTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// Disable autosaving parameters to avoid busy loop in param_set()
		param_control_autosave(false);
		param_reset_all();

		// The yaw loop the expected values are computed from
		setFloat("AS_YAWRATE_P", 0.7f);
		setFloat("AS_YAWRATE_I", 0.09f);
		setFloat("AS_YR_INT_LIM", 0.3f);
		setFloat("AS_YAWRATE_FF", 0.f);
		setFloat("AS_YAWRATE_MAX", 15.f);
		setFloat("MAN_DEADZONE", 0.f);
	}

protected:
	void start()
	{
		_controller = std::make_unique<AirshipAttitudeControl>();
		drain();
	}

	/** One gyro sample in `mode`, not rotating; the sticks are published only when asked */
	void cycle(vehicle_control_mode_s mode, const Sticks &sticks, bool publish_sticks = true)
	{
		mode.timestamp = hrt_absolute_time();
		_mode_pub.publish(mode);

		if (publish_sticks) {
			manual_control_setpoint_s setpoint{};
			setpoint.timestamp = hrt_absolute_time();
			setpoint.valid = sticks.valid;
			setpoint.roll = sticks.roll;
			setpoint.pitch = sticks.pitch;
			setpoint.yaw = sticks.yaw;
			setpoint.throttle = sticks.throttle;
			_sticks_pub.publish(setpoint);
		}

		vehicle_angular_velocity_s gyro{};
		gyro.timestamp = hrt_absolute_time();
		gyro.timestamp_sample = _sample;
		_gyro_pub.publish(gyro);
		_sample += kGyroInterval;

		_controller->Run();
	}

	/** All tests share one process: a new subscription reads the previous test's last sample as new */
	void drain()
	{
		_torque_sub.update();
		_thrust_sub.update();
		_rates_sub.update();
		rate_ctrl_status_s status{};
		_status_sub.update(&status);
	}

	/** The wrench published as zero and the yaw loop left open: no rate setpoint, no loop status */
	void expectZeroWrench(const char *when)
	{
		SCOPED_TRACE(when);
		ASSERT_TRUE(_torque_sub.update());
		ASSERT_TRUE(_thrust_sub.update());

		for (int i = 0; i < 3; i++) {
			EXPECT_FLOAT_EQ(_torque_sub.get().xyz[i], 0.f) << "axis " << i;
			EXPECT_FLOAT_EQ(_thrust_sub.get().xyz[i], 0.f) << "axis " << i;
		}

		EXPECT_FALSE(_rates_sub.updated());
		EXPECT_FALSE(_status_sub.updated());
	}

	uORB::Publication<vehicle_control_mode_s> _mode_pub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<manual_control_setpoint_s> _sticks_pub{ORB_ID(manual_control_setpoint)};
	uORB::Publication<vehicle_angular_velocity_s> _gyro_pub{ORB_ID(vehicle_angular_velocity)};

	uORB::SubscriptionData<vehicle_torque_setpoint_s> _torque_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::SubscriptionData<vehicle_thrust_setpoint_s> _thrust_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::SubscriptionData<vehicle_rates_setpoint_s> _rates_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _status_sub{ORB_ID(rate_ctrl_status)};

	std::unique_ptr<AirshipAttitudeControl> _controller;
	hrt_abstime _sample{12345678};
};

TEST_F(AirshipAttitudeControlTest, OffboardThrustAndTorqueOwnsTheWrench)
{
	start();

	// Offboard sending thrust and torque: its own publisher writes both topics
	vehicle_control_mode_s offboard{};
	offboard.flag_armed = true;
	offboard.flag_control_offboard_enabled = true;
	offboard.flag_control_allocation_enabled = true;
	cycle(offboard, Sticks{});
	EXPECT_FALSE(_torque_sub.updated());
	EXPECT_FALSE(_thrust_sub.updated());

	// Acro publishes both
	cycle(acro(), Sticks{});
	EXPECT_TRUE(_torque_sub.updated());
	EXPECT_TRUE(_thrust_sub.updated());
}

TEST_F(AirshipAttitudeControlTest, AcroClosesTheLoopOnPropulsiveYaw)
{
	setIndependentPods();
	start();

	Sticks sticks;
	sticks.roll = 0.3f;
	sticks.yaw = 1.f;
	cycle(acro(), sticks);

	// Not rotating, integral still empty: P times the full-stick rate
	ASSERT_TRUE(_torque_sub.update());
	EXPECT_NEAR(_torque_sub.get().xyz[2], 0.7f * math::radians(15.f), 1e-6f);
	EXPECT_FLOAT_EQ(_torque_sub.get().xyz[0], 0.3f);	// roll stays passthrough

	ASSERT_TRUE(_rates_sub.update());
	EXPECT_FLOAT_EQ(_rates_sub.get().yaw, math::radians(15.f));
	EXPECT_FALSE(PX4_ISFINITE(_rates_sub.get().roll));
	EXPECT_FALSE(PX4_ISFINITE(_rates_sub.get().pitch));
}

TEST_F(AirshipAttitudeControlTest, FeedforwardReachesTheYawTorque)
{
	setIndependentPods();
	setFloat("AS_YAWRATE_FF", 1.f);
	start();

	Sticks sticks;
	sticks.yaw = 1.f;
	cycle(acro(), sticks);

	// P and FF on the same full-stick rate: 1.7 times it, where AS_YR_INT_LIM
	// in the feedforward's place would give 1.0 times it
	ASSERT_TRUE(_torque_sub.update());
	EXPECT_NEAR(_torque_sub.get().xyz[2], (0.7f + 1.f) * math::radians(15.f), 1e-5f);
}

TEST_F(AirshipAttitudeControlTest, ManualOpensTheLoopAndReportsItOnce)
{
	setIndependentPods();
	start();

	Sticks sticks;
	sticks.yaw = 1.f;

	for (int i = 0; i < 10; i++) {
		cycle(acro(), sticks);
	}

	rate_ctrl_status_s status{};
	ASSERT_TRUE(_status_sub.update(&status));
	ASSERT_GT(status.yawspeed_integ, 0.f);	// wound up while closed
	drain();

	// Manual: the stick is the torque again, and the cleared integrator is published once
	cycle(manual(), sticks);
	ASSERT_TRUE(_torque_sub.update());
	EXPECT_FLOAT_EQ(_torque_sub.get().xyz[2], 1.f);
	ASSERT_TRUE(_status_sub.update(&status));
	EXPECT_FLOAT_EQ(status.yawspeed_integ, 0.f);

	cycle(manual(), sticks);
	EXPECT_FALSE(_status_sub.updated());
}

TEST_F(AirshipAttitudeControlTest, CollectivePodsWithoutTailKeepTheStickAsTorque)
{
	// No differential thrust, no tail, no surfaces: nothing makes yaw at rest
	setInt("CA_AIRFRAME", kCaAirframeAirship);
	setInt("CA_AIRSHIP_GRP", 0);
	setInt("CA_AIRSHIP_TAIL", 0);
	setInt("CA_SV_CS_COUNT", 0);
	start();

	Sticks sticks;
	sticks.yaw = 1.f;
	cycle(acro(), sticks);

	ASSERT_TRUE(_torque_sub.update());
	EXPECT_FLOAT_EQ(_torque_sub.get().xyz[2], 1.f);
	EXPECT_FALSE(_rates_sub.updated());	// no rate loop, no rate setpoint
}

TEST_F(AirshipAttitudeControlTest, YawRuddersAtFullCreditKeepTheStick)
{
	// Two rudders credited in full carry the yaw; the module reads them
	// through its CA_SV_CSn parameter handles
	setIndependentPods();
	setInt("CA_AIRSHIP_TAIL", 0);
	setInt("CA_SV_CS_COUNT", 2);
	setInt("CA_SV_CS0_TYPE", 4);	// rudder
	setInt("CA_SV_CS1_TYPE", 4);
	setFloat("CA_SV_CS0_TRQ_Y", 0.5f);
	setFloat("CA_SV_CS1_TRQ_Y", 0.5f);
	setFloat("CA_AIRSHIP_CS_K", 1.f);
	start();

	Sticks sticks;
	sticks.yaw = 1.f;
	cycle(acro(), sticks);

	ASSERT_TRUE(_torque_sub.update());
	EXPECT_FLOAT_EQ(_torque_sub.get().xyz[2], 1.f);
}

TEST_F(AirshipAttitudeControlTest, UnusableSticksPublishZeroWrenchInAcro)
{
	setIndependentPods();
	start();

	// Acro on propulsive yaw: disarmed or with the sticks lost the yaw loop
	// stays open, so nothing overwrites the zeroed wrench
	Sticks sticks;
	sticks.roll = 0.5f;
	sticks.pitch = 0.5f;
	sticks.yaw = 0.5f;
	sticks.throttle = 0.5f;

	vehicle_control_mode_s disarmed = acro();
	disarmed.flag_armed = false;
	cycle(disarmed, sticks);
	expectZeroWrench("disarmed");

	// A lost link: manual_control publishes its last setpoint once, not valid
	sticks.valid = false;
	cycle(acro(), sticks);
	expectZeroWrench("sticks lost");

	cycle(acro(), sticks, false);
	expectZeroWrench("sticks lost, no new sample");
}

TEST_F(AirshipAttitudeControlTest, RatesSetpointOnlyOnNewSticks)
{
	setIndependentPods();
	start();

	Sticks sticks;
	sticks.yaw = 1.f;
	cycle(acro(), sticks);
	drain();

	// A gyro sample without a new stick sample still closes the loop, but has
	// no new setpoint to log
	cycle(acro(), sticks, false);
	EXPECT_TRUE(_torque_sub.updated());
	EXPECT_FALSE(_rates_sub.updated());
}
