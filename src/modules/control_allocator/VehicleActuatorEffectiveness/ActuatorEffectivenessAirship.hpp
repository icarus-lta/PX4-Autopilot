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
 * @file ActuatorEffectivenessAirship.hpp
 *
 * Actuator effectiveness for an airship with two side propulsion pods
 * symmetric about the CoG, each on a non-reversible propeller that tilts
 * about the body y axis (0 = thrust forward, positive = up), optionally a
 * tail yaw thruster (motor 3) and control surfaces.
 *
 * The pods are allocated in closed form in updateSetpoint(): the demand is
 * split into a per-pod force, index 0 = starboard and 1 = port, as
 * fx = thrust_x -/+ yaw and fz = thrust_up -/+ roll with thrust_up = -thrust_z,
 * yaw and roll net of the credited control-surface torque, all normalized.
 * The tilt is the direction of that force and the motor its projection onto
 * the realized tilt, so the pods produce no pitch torque and reach reverse
 * thrust only by tilting. Collective grouping (CA_AIRSHIP_GRP = 0) gives both
 * pods the same force on one tilt servo. Control surfaces are allocated by
 * the matrix; the pods and the tail serve what they leave unmet.
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>

class ActuatorEffectivenessAirship : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessAirship(ModuleParams *parent) : ModuleParams(parent), _control_surfaces(this) {}
	virtual ~ActuatorEffectivenessAirship() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
			    const ActuatorVector &actuator_max) override;

	/** No auxiliary controls: takes the allocator's cycle time for the tilt slew in updateSetpoint() */
	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override { _dt = dt; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	const char *name() const override { return "Airship"; }

	// The tilt direction is the atan2 of the pod force demand, which is
	// meaningless near zero magnitude: stick noise alone would slam the tilt
	// between opposite directions. Steering therefore engages only above the
	// stick-noise floor and releases at half of it; inside the band the last
	// commanded direction stands, so a sign reversal there cannot retarget.
	// What a held pod projects away is not reported as saturation.
	static constexpr float kTiltSteerEngage = 0.02f;
	static constexpr float kTiltSteerRelease = 0.01f;

	// Pointing (near-)straight back, +180 and -180 deg realize the same thrust
	// direction at opposite ends of an end-stop servo: inside this cone of the
	// negative x axis (|fz| < kTiltRearCone * |f|: a ratio, sin of the ~3 deg
	// half-angle; floored at kTiltSteerRelease for low demand) the end that
	// realizes the demand best is chosen and on a tie the committed end is
	// kept, so perpendicular noise cannot command a full-range sweep. The same
	// margin, in pod-force units, is the hysteresis for switching range ends
	// when the target falls outside the tilt range.
	static constexpr float kTiltRearCone = 0.05f;

private:
	/** +1, -1 or 0: the direction of a shortfall, as the rate controller reads it */
	static float saturationSign(float shortfall);
	static float discountHeld(float residual, float held_part);

	/**
	 * Tilt a pod should steer to for a force demand above the steer band
	 * @param fx, fz pod force demand (forward, up), magnitude its norm
	 * @param committed the pod's current tilt target [rad]
	 * @return the target within [tilt_min, tilt_max] [rad]
	 */
	static float steerTarget(float fx, float fz, float magnitude, float committed, float tilt_min, float tilt_max);

	/** Write the realized tilts to the servo outputs and read the clamped angles back */
	void writeTiltServos(ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
			     const ActuatorVector &actuator_max, float tilt_min, float tilt_span);

	float tiltMin() const { return math::radians(_param_ca_airship_tlmin.get()); }
	float tiltMax() const { return math::radians(_param_ca_airship_tlmax.get()); }

	SlewRate<float> _tilt[2] {};	///< realized tilt [rad], held through zero-thrust
	float _tilt_target[2] {};	///< commanded tilt [rad] the slew tracks; holds through the hysteresis band
	bool _tilt_steering[2] {};	///< per-pod hysteresis state of the direction hold

	bool _armed{false};		///< the tilts park until vehicle_status reports armed, as the tiltrotor holds its tilts
	float _dt{0.f};		///< allocator time step [s], handed in right before updateSetpoint()

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	ActuatorEffectivenessControlSurfaces _control_surfaces;

	// Actuator layout, decided when the actuators are declared
	int _first_control_surface_idx{0};
	int _first_tilt_idx{0};
	int _tilt_count{0};
	bool _has_tail{false};
	bool _independent{false};

	bool _surface_serves[3] {};	///< torque axes with control-surface effectiveness
	matrix::Vector3f _surface_torque{};	///< torque the clipped, trim-relative surface deflections can deliver
	matrix::Vector3f _achieved_torque{};	///< torque the pods and tail delivered (no pitch: the pods produce none)
	matrix::Vector3f _held_torque{};	///< torque a pod held inside the steer band leaves unserved by choice
	float _unallocated_torque[3] {};	///< sign of the roll/pitch/yaw shortfall left by the propulsors
	float _unallocated_thrust[3] {};	///< sign of the x/y/z force shortfall left by the propulsors

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_AIRSHIP_TLMIN>) _param_ca_airship_tlmin,
		(ParamFloat<px4::params::CA_AIRSHIP_TLMAX>) _param_ca_airship_tlmax,
		(ParamInt<px4::params::CA_AIRSHIP_GRP>) _param_ca_airship_grp,
		(ParamBool<px4::params::CA_AIRSHIP_TAIL>) _param_ca_airship_tail,
		(ParamFloat<px4::params::CA_AIRSHIP_CS_K>) _param_ca_airship_cs_k,
		(ParamFloat<px4::params::CA_AIRSHIP_TLT_R>) _param_ca_airship_tlt_r
	)
};
