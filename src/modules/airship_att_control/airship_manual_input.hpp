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
 * @file airship_manual_input.hpp
 *
 * How the airship reads the sticks: the thrust and torque passthrough and the
 * modes it is published in, the yaw stick as a rate setpoint and the gate of
 * the yaw rate loop, kept free of uORB I/O so they can be unit tested.
 */

#pragma once

#include <control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp>
#include <control_allocation/actuator_effectiveness/AirshipAirframe.hpp>
#include <control_allocation/actuator_effectiveness/ControlSurfaceType.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>

namespace airship_manual_input
{

/** NaN marks a manual channel without valid data: read it as released */
inline float finiteOr(float value, float fallback)
{
	return PX4_ISFINITE(value) ? value : fallback;
}

/**
 * Thrust setpoint per body axis: throttle forward, the pitch stick vertical.
 *
 * Stick forward descends (+pitch = stick forward, +z = down in FRD). The
 * pitch stick is also pitch torque in torque(): tilting pods serve this
 * thrust, elevators serve that torque; the channel an airframe cannot serve
 * is reported unallocated.
 */
inline matrix::Vector3f thrust(const manual_control_setpoint_s &sticks)
{
	return matrix::Vector3f{(finiteOr(sticks.throttle, -1.f) + 1.f) * 0.5f, 0.f, finiteOr(sticks.pitch, 0.f)};
}

/**
 * Torque setpoint per body axis: the sticks straight through.
 *
 * Stick forward is nose down: negative pitch rotation in FRD.
 */
inline matrix::Vector3f torque(const manual_control_setpoint_s &sticks)
{
	return matrix::Vector3f{finiteOr(sticks.roll, 0.f), -finiteOr(sticks.pitch, 0.f), finiteOr(sticks.yaw, 0.f)};
}

/**
 * Map the yaw stick to a yaw rate setpoint [rad/s].
 *
 * Deadzone first (MAN_DEADZONE, as mc_att_control and lib/sticks apply it),
 * then linear scaling to max_rate; a non-finite stick reads as released.
 */
inline float yawRateSetpoint(float stick, float deadzone, float max_rate)
{
	return math::deadzone(finiteOr(stick, 0.f), deadzone) * max_rate;
}

/**
 * Whether the pilot is in command: armed with valid manual input.
 *
 * The sticks stay live in every armed mode; lost or never-published input
 * keeps its last values and only clears .valid, so it must read as released.
 */
inline bool manualInputUsable(const vehicle_control_mode_s &control_mode, const manual_control_setpoint_s &sticks)
{
	return control_mode.flag_armed && sticks.valid;
}

/**
 * Whether this module publishes the thrust and torque setpoints: in a mode
 * with manual, rate or attitude control, the gate fw_rate_control and
 * uuv_att_control publish under. A mode that enables none of these either
 * has its own publisher of the two topics (Offboard or an external mode
 * sending thrust and torque) or runs no allocation (actuator setpoints,
 * Termination).
 */
inline bool wrenchPublished(const vehicle_control_mode_s &control_mode)
{
	return control_mode.flag_control_manual_enabled || control_mode.flag_control_rates_enabled
	       || control_mode.flag_control_attitude_enabled;
}

/** CA_AIRFRAME value of the airship allocator, the one whose parameters hasPropulsiveYaw() reads */
static constexpr int32_t kAirshipAllocator = kCaAirframeAirship;

/** Largest number of control surfaces (CA_SV_CS_COUNT) */
static constexpr int kMaxControlSurfaces = kControlSurfaceMaxCount;

/**
 * Whether the control surfaces take yaw in the allocator, from their
 * CA_SV_CSn_TYPE and CA_SV_CSn_TRQ_Y: the allocator gives flaps, airbrakes,
 * the steering wheel and spoilers no torque whatever their TRQ parameters say
 * (controlSurfaceTakesTorque()), and it drops an axis whose every entry is at
 * most ActuatorEffectiveness::kMinEffectiveness, so the yaw row survives only
 * if a surface of another type has a yaw entry above that. The
 * pseudo-inverse's dependent-axis drop is not mirrored: a yaw row that only
 * repeats a roll or pitch row still counts here, which keeps the loop open
 * and the stick as torque on such an airframe.
 */
inline bool hasYawSurfaces(int count, const int32_t types[], const float yaw_torques[])
{
	bool any = false;

	for (int i = 0; i < count && i < kMaxControlSurfaces; i++) {
		const auto surface_type = static_cast<ControlSurfaceType>(types[i]);

		if (controlSurfaceTakesTorque(surface_type) && fabsf(yaw_torques[i]) > ActuatorEffectiveness::kMinEffectiveness) {
			any = true;
		}
	}

	return any;
}

/**
 * Whether the propulsion gets yaw torque to make at rest, which the yaw rate
 * loop needs to close (AS_YAWRATE_MAX states the rule for the user). The inputs
 * are the airship allocator's configuration: pod_grouping (CA_AIRSHIP_GRP;
 * above 0 is the allocator's own test for independently driven pods), tail
 * (CA_AIRSHIP_TAIL, a tail thruster), yaw_surfaces (hasYawSurfaces() above)
 * and surface_credit (CA_AIRSHIP_CS_K).
 *
 * The pods and the tail serve only what the credited surfaces leave. At the
 * default credit of 1 that is only the yaw beyond what the surfaces deliver at
 * full deflection, so the surfaces carry the yaw first and the loop stays open
 * as on surfaces alone; surfaces too small for the stick's range leave the
 * rest to the pods as stick torque. Below 1 the pods get the uncredited share
 * of the yaw the loop asks for, and a small share leaves the loop little
 * authority at rest, where its integrator then winds with no shortfall
 * reported.
 *
 * On surfaces alone the stick stays torque in every mode, as fixed-wing Acro
 * does by default (FW_ACRO_YAW_EN). The loop has no airspeed scaling: at rest
 * its integrator would wind to AS_YR_INT_LIM with no shortfall reported, since
 * the surfaces do not saturate, and hold that rudder after the stick is
 * released, and in any mode it would cap the normalized yaw torque at
 * AS_YAWRATE_P plus AS_YAWRATE_FF, times AS_YAWRATE_MAX (in rad/s), plus
 * AS_YR_INT_LIM.
 *
 * Any other allocator keeps the loop: these parameters do not describe its
 * propulsion (the gazebo-classic Cloudship's custom rotor set, a tail rotor
 * among them).
 */
inline bool hasPropulsiveYaw(int32_t allocator, int32_t pod_grouping, bool tail, bool yaw_surfaces,
			     float surface_credit)
{
	if (allocator != kAirshipAllocator) {
		return true;
	}

	return (pod_grouping > 0 || tail) && !(yaw_surfaces && surface_credit >= 1.f);
}

/**
 * Whether the yaw rate loop closes on the stick: usable manual input
 * (manualInputUsable() above, passed by the caller) in a manual mode with rate
 * control (Acro, Stabilized, Altitude, Position), on an airframe whose
 * propulsion gets yaw to make (hasPropulsiveYaw() above, which says why yaw
 * from surfaces alone keeps the stick as torque). Manual has rates off and the
 * non-manual modes have no setpoint source here yet; both keep the torque
 * passthrough. Disarmed or with the sticks lost the loop stays open, so it
 * cannot overwrite the zeroed torque or wind up.
 */
inline bool yawRateLoopActive(const vehicle_control_mode_s &control_mode, bool manual_input_usable,
			      bool propulsive_yaw)
{
	return manual_input_usable && propulsive_yaw && control_mode.flag_control_manual_enabled
	       && control_mode.flag_control_rates_enabled;
}

} // namespace airship_manual_input
