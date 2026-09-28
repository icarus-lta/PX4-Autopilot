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
 * Airship with two side propulsion pods symmetric about the CoG (0 =
 * starboard, 1 = port), each a non-reversible propeller on a tilt servo
 * (AirshipPod), an optional tail yaw thruster (motor 3) and control
 * surfaces. The pod force is (thrust_x, -thrust_z) -/+ (yaw, roll), net of
 * the credited control-surface torque, all normalized; collective grouping
 * gives both pods the common force on one servo. Motors and tilts are
 * declared with zero effectiveness and computed in updateSetpoint(); the
 * pods produce no pitch torque and reverse only by tilting. The surfaces
 * are allocated by the matrix; the pods and the tail serve what they leave.
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"
#include "AirshipPod.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>

class ActuatorEffectivenessAirship : public ModuleParams, public ActuatorEffectiveness
{
public:
	static constexpr int NUM_PODS = 2;
	enum MotorIndex { STARBOARD = 0, PORT = 1, TAIL = 2 };

	/** Pod grouping: CA_AIRSHIP_GRP > 0 selects Independent, anything else Collective */
	enum class Grouping { Collective, Independent };

	ActuatorEffectivenessAirship(ModuleParams *parent);
	virtual ~ActuatorEffectivenessAirship() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
			    const ActuatorVector &actuator_max) override;

	/**
	 * No auxiliary controls: takes the allocator's cycle time for the tilt slew
	 * in updateSetpoint(). Because the flaps/spoiler path is not called, a
	 * CA_SV_CSn_TYPE of flap or spoiler (or any type controlSurfaceTakesTorque()
	 * rejects) is inert on an airship: the surface still occupies its slot, its
	 * torque is zero (controlSurfaceTakesTorque()), so it is credited nothing
	 * and its setpoint stays at CA_SV_CSn_TRIM; CA_SV_CSn_FLAP and
	 * CA_SV_CSn_SPOIL are ignored.
	 */
	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override { _dt = dt; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	const char *name() const override { return "Airship"; }

private:
	void updateParams() override;

	/** Refresh _armed from vehicle_status; the last sample holds between updates */
	void updateArmedState();

	/** Torque the clipped, trim-relative control-surface deflections deliver */
	matrix::Vector3f surfaceTorque(const ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
				       const ActuatorVector &actuator_max) const;

	/**
	 * Body wrench of a pod force pair: the mean is thrust (x forward, z down),
	 * half the difference the roll/yaw couple.
	 *
	 * The left inverse of the demand split in updateSetpoint() on the four axes
	 * the pods serve: podWrench() of that split reproduces ROLL, YAW, THRUST_X
	 * and THRUST_Z exactly. The sign flips are the pod frame (forward, up)
	 * against the body frame (x forward, z down).
	 */
	static matrix::Vector<float, NUM_AXES> podWrench(const matrix::Vector2f &starboard, const matrix::Vector2f &port);

	/** Write the tilt servos and read the clamped angles back; the collective servo drives both pods */
	void writeTiltServos(ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
			     const ActuatorVector &actuator_max);

	/*
	 * A shortfall this small is rounding, not saturation. The pods' achieved
	 * wrench comes through atan2f, cosf/sinf and the servo round trip, which
	 * carry a few ulps; publishing a sign quantizes that to 1 before the
	 * allocator's own achieved gate (norm_squared < 1e-6) can absorb it, and
	 * the rate controller would then stop integrating against exactly the
	 * small steady torques the integral exists to remove. Three orders above
	 * the measured noise and an order below the smallest real shortfall the
	 * tests assert. [normalized torque or thrust]
	 *
	 * Applied on every axis, the ones a surface serves included: with the
	 * yaw rate loop closed on independent pods at a CA_AIRSHIP_CS_K below 1
	 * they realize the served yaw through the same round trip, and a float32
	 * replay of that case leaves residuals of order 1e-7 on a few percent of
	 * the cycles. On the two shapes measured earlier the band never decided:
	 * fixed mounts (2500_generic_airship) carry no rounding at all, and a
	 * tilting airframe whose pods cannot realize the served axis reports the
	 * demand's own magnitude, three orders above it.
	 */
	static constexpr float kShortfallDeadband = 1e-4f;

	/** +1, -1 or 0: the direction of a shortfall, as the rate controller reads it */
	static float saturationSign(float shortfall);

	/**
	 * What an axis was asked for, less what the pods and the tail delivered,
	 * less the share they withheld by choice (see AirshipPod::heldByChoice):
	 * reported as saturation that share would freeze the rate integrator
	 * against the small steady torques the integral exists to remove. Only a
	 * same-signed share is removed, because the two pods can withhold and
	 * fall short in opposite directions -- one holding in the steer band
	 * while the other steers and falls short, floored by the non-reversible
	 * clamp or leaving a perpendicular residual at a range end. Subtracting
	 * the whole share would then flip the published sign and drive the
	 * integrator the wrong way.
	 */
	float shortfall(float asked, int axis) const;

	AirshipPod _pods[NUM_PODS] {};
	bool _armed{false};		///< the tilts park until vehicle_status reports armed
	float _dt{0.f};			///< allocator time step [s], handed in right before updateSetpoint()

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	ActuatorEffectivenessControlSurfaces _control_surfaces;

	// From the parameters
	Grouping _grouping{Grouping::Collective};
	bool _has_tail{false};

	// Actuator layout, decided when the actuators are declared
	int _first_control_surface_idx{0};
	int _first_tilt_idx{0};
	int _num_tilt_servos{0};
	bool _surface_serves[3] {};	///< torque axes whose matrix row the allocator keeps: a surface entry above kMinEffectiveness

	// What updateSetpoint() leaves for getUnallocatedControl(): the raw
	// quantities, so the subtraction happens once, where it is published
	matrix::Vector3f _surface_torque{};	///< torque the clipped, trim-relative surface deflections deliver
	matrix::Vector<float, NUM_AXES> _demand{};	///< what the pods and the tail were asked for: control_sp less the credited surface torque
	matrix::Vector<float, NUM_AXES> _achieved{};	///< what the pods and the tail delivered
	matrix::Vector<float, NUM_AXES> _held{};	///< what the pods withhold by choice (AirshipPod::heldByChoice)

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_AIRSHIP_TLMIN>) _param_ca_airship_tlmin,
		(ParamFloat<px4::params::CA_AIRSHIP_TLMAX>) _param_ca_airship_tlmax,
		(ParamInt<px4::params::CA_AIRSHIP_GRP>) _param_ca_airship_grp,
		(ParamBool<px4::params::CA_AIRSHIP_TAIL>) _param_ca_airship_tail,
		(ParamFloat<px4::params::CA_AIRSHIP_CS_K>) _param_ca_airship_cs_k,
		(ParamFloat<px4::params::CA_AIRSHIP_TLT_R>) _param_ca_airship_tlt_r
	)
};
