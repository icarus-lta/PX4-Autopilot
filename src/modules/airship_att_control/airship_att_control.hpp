/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include "airship_yaw_rate.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/rate_control/rate_control.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class AirshipAttitudeControl : public ModuleBase, public ModuleParams,
	public px4::WorkItem
{
public:
	static Descriptor desc;

	AirshipAttitudeControl();

	virtual ~AirshipAttitudeControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	/**
	 * Check for parameter update and handle it.
	 */
	void parameter_update_poll();

	/** Push the gains into the rate controller and convert the max rate to rad/s */
	void parameters_updated();

	/** Manual thrust: throttle forward, pitch vertical; zero unless armed with usable sticks */
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);

	/** Stick torque passthrough, with the yaw axis closed on the stick rate where the mode asks for rates */
	void publishTorqueSetpoint(const vehicle_angular_velocity_s &angular_velocity, float dt, bool new_sticks);

	/**
	 * Close the yaw rate loop on the stick.
	 * @param publish_setpoint publish the rates setpoint (on a new stick sample)
	 * @return normalized yaw torque
	 */
	float controlYawRate(const vehicle_angular_velocity_s &angular_velocity, float dt, bool publish_setpoint);

	/** Anti-windup feedback from the control allocator */
	void updateSaturationStatus();

	/** Integrator state for logging */
	void publishRateControlStatus();

	// The sticks stay live in every armed mode because no other module
	// serves the airship outside manual, but lost or never-published input
	// (which keeps its last finite values and only clears .valid) must
	// read as released, not be flown indefinitely.
	bool manualInputUsable() const { return _vehicle_control_mode.flag_armed && _manual_control_setpoint.valid; }

	RateControl _rate_control; ///< yaw axis only: roll and pitch gains stay zero

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<vehicle_thrust_setpoint_s>    _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>    _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_rates_setpoint_s>     _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::PublicationMulti<rate_ctrl_status_s>      _rate_ctrl_status_pub{ORB_ID(rate_ctrl_status)};

	manual_control_setpoint_s       _manual_control_setpoint{};
	vehicle_control_mode_s          _vehicle_control_mode{};

	matrix::Vector3f _thrust_setpoint{};	///< last published thrust, mirrored into the rates setpoint
	hrt_abstime _last_run{0};
	float _yaw_rate_max{0.f};		///< AS_YAWRATE_MAX [rad/s]
	bool _yaw_loop_active{false};		///< the yaw rate loop closed on the previous cycle

	perf_counter_t _loop_perf;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AS_YAWRATE_P>) _param_as_yawrate_p,
		(ParamFloat<px4::params::AS_YAWRATE_I>) _param_as_yawrate_i,
		(ParamFloat<px4::params::AS_YR_INT_LIM>) _param_as_yr_int_lim,
		(ParamFloat<px4::params::AS_YAWRATE_MAX>) _param_as_yawrate_max,
		(ParamFloat<px4::params::MAN_DEADZONE>) _param_man_deadzone
	)
};
