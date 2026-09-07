/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file airship_att_control_main.cpp
 * Airship attitude controller.
 *
 * @author Anton Erasmus	<anton@flycloudline.com>
 */

#include "airship_att_control.hpp"

#include <float.h>
#include <px4_platform_common/defines.h>

using namespace matrix;

// NaN marks a manual channel without valid data: read it as released
static float finiteOr(float value, float fallback)
{
	return PX4_ISFINITE(value) ? value : fallback;
}

ModuleBase::Descriptor AirshipAttitudeControl::desc{task_spawn, custom_command, print_usage};

AirshipAttitudeControl::AirshipAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "airship_att_control"))
{
	_rate_ctrl_status_pub.advertise();
	parameters_updated();
}

AirshipAttitudeControl::~AirshipAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
AirshipAttitudeControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
AirshipAttitudeControl::parameter_update_poll()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}
}

void
AirshipAttitudeControl::parameters_updated()
{
	// Yaw axis only: roll and pitch stay stick passthrough, their gains are zero
	_rate_control.setPidGains(Vector3f(0.f, 0.f, _param_as_yawrate_p.get()),
				  Vector3f(0.f, 0.f, _param_as_yawrate_i.get()), Vector3f());
	// The library integrator limit defaults to zero, which would silently
	// disable the I term
	_rate_control.setIntegratorLimit(Vector3f(0.f, 0.f, _param_as_yr_int_lim.get()));
	_yaw_rate_max = math::radians(_param_as_yawrate_max.get());
}

void AirshipAttitudeControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.timestamp_sample = timestamp_sample;

	// zero actuators unless armed with usable manual input
	if (manualInputUsable()) {
		v_thrust_sp.xyz[0] = (finiteOr(_manual_control_setpoint.throttle, -1.f) + 1.f) * .5f;
		// Stick forward descends: pitch drives the elevators on finned
		// airships and vertical thrust on vectored ones.
		v_thrust_sp.xyz[2] = finiteOr(_manual_control_setpoint.pitch, 0.f);
	}

	_thrust_setpoint = Vector3f(v_thrust_sp.xyz);
	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}

void AirshipAttitudeControl::publishTorqueSetpoint(const vehicle_angular_velocity_s &angular_velocity, const float dt,
		const bool new_sticks)
{
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = angular_velocity.timestamp_sample;

	const bool manual_input_usable = manualInputUsable();
	const bool yaw_loop_active = airship_yaw_rate::loopActive(_vehicle_control_mode, manual_input_usable);

	// zero actuators unless armed with usable manual input
	if (manual_input_usable) {
		v_torque_sp.xyz[0] = finiteOr(_manual_control_setpoint.roll, 0.f);
		// Stick forward is nose down: negative pitch rotation in FRD
		v_torque_sp.xyz[1] = -finiteOr(_manual_control_setpoint.pitch, 0.f);
		// Yaw: rate loop where the mode asks for rates, otherwise the stick is the torque
		v_torque_sp.xyz[2] = yaw_loop_active ? controlYawRate(angular_velocity, dt, new_sticks)
				     : finiteOr(_manual_control_setpoint.yaw, 0.f);
	}

	if (!yaw_loop_active) {
		_rate_control.resetIntegral();

		if (_yaw_loop_active) {
			// log the reset once, so the integrator does not read as frozen
			publishRateControlStatus();
		}
	}

	_yaw_loop_active = yaw_loop_active;

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

float AirshipAttitudeControl::controlYawRate(const vehicle_angular_velocity_s &angular_velocity, const float dt,
		const bool publish_setpoint)
{
	updateSaturationStatus();

	const float yaw_rate_sp = airship_yaw_rate::setpointFromStick(_manual_control_setpoint.yaw,
				  _param_man_deadzone.get(), _yaw_rate_max);

	const Vector3f rates{angular_velocity.xyz};

	// No D term, so no angular acceleration (0 * NaN would poison the torque).
	// landed = false: AirshipLandDetector reports landed = !armed and landed
	// throughout AUTO_LAND, which would freeze the integrator in flight.
	const Vector3f torque = _rate_control.update(rates, Vector3f(0.f, 0.f, yaw_rate_sp), Vector3f{}, dt, false);

	if (publish_setpoint) {
		// Roll and pitch carry no rate loop: NaN marks them uncontrolled
		vehicle_rates_setpoint_s rates_sp{};
		rates_sp.roll = NAN;
		rates_sp.pitch = NAN;
		rates_sp.yaw = yaw_rate_sp;
		_thrust_setpoint.copyTo(rates_sp.thrust_body);
		rates_sp.timestamp = hrt_absolute_time();
		_vehicle_rates_setpoint_pub.publish(rates_sp);
	}

	publishRateControlStatus();

	return PX4_ISFINITE(torque(2)) ? torque(2) : 0.f;
}

void AirshipAttitudeControl::publishRateControlStatus()
{
	rate_ctrl_status_s rate_ctrl_status{};
	_rate_control.getRateControlStatus(rate_ctrl_status);
	rate_ctrl_status.timestamp = hrt_absolute_time();
	_rate_ctrl_status_pub.publish(rate_ctrl_status);
}

void AirshipAttitudeControl::updateSaturationStatus()
{
	// Anti-windup from the allocator, wired as in mc_rate_control: an axis
	// the allocator could not serve stops integrating in that direction.
	control_allocator_status_s control_allocator_status;

	if (_control_allocator_status_sub.update(&control_allocator_status)) {
		Vector<bool, 3> saturation_positive;
		Vector<bool, 3> saturation_negative;

		if (!control_allocator_status.torque_setpoint_achieved) {
			for (size_t i = 0; i < 3; i++) {
				if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
					saturation_positive(i) = true;

				} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
					saturation_negative(i) = true;
				}
			}
		}

		_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
	}
}

void
AirshipAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		/* refresh manual control and control mode before publishing */
		const bool new_sticks = _manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		publishThrustSetpoint(angular_velocity.timestamp_sample);
		publishTorqueSetpoint(angular_velocity, dt, new_sticks);

		parameter_update_poll();
	}

	perf_end(_loop_perf);
}

int AirshipAttitudeControl::task_spawn(int argc, char *argv[])
{
	AirshipAttitudeControl *instance = new AirshipAttitudeControl();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int AirshipAttitudeControl::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	return 0;
}

int AirshipAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AirshipAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the airship attitude and rate controller. Roll, pitch and
thrust are stick passthrough. In manual modes with rate control (Acro,
Stabilized, Altitude, Position) the yaw stick commands a yaw rate closed by a
PI loop whenever armed, on the ground included; in Manual and in modes without
pilot input the yaw stick is passed through as torque.

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("airship_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Airship attitude control app start / stop handling function
 */
extern "C" __EXPORT int airship_att_control_main(int argc, char *argv[])
{
	return ModuleBase::main(AirshipAttitudeControl::desc, argc, argv);
}
