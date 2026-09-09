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

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>

class ActuatorEffectivenessAirship : public ModuleParams, public ActuatorEffectiveness
{
public:
	static constexpr int NUM_PODS = 2;
	enum MotorIndex { STARBOARD = 0, PORT = 1, TAIL = 2 };

	enum class Grouping : int32_t {
		// This matches with the parameter CA_AIRSHIP_GRP
		Collective = 0,
		Independent = 1,
	};

	ActuatorEffectivenessAirship(ModuleParams *parent);
	virtual ~ActuatorEffectivenessAirship() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
			    const ActuatorVector &actuator_max) override;

	/** No auxiliary controls: takes the allocator's cycle time for the tilt slew in updateSetpoint() */
	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override { _dt = dt; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	const char *name() const override { return "Airship"; }

protected:
	void updateParams() override;

private:
	/** Refresh the armed state from vehicle_status */
	bool isArmed();

	/** Torque the clipped, trim-relative control-surface deflections deliver */
	matrix::Vector3f surfaceTorque(const ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
				       const ActuatorVector &actuator_max) const;

	/** Body wrench of a pod force pair: the mean is thrust (x forward, z down), half the difference the roll/yaw couple */
	static matrix::Vector<float, NUM_AXES> podWrench(const matrix::Vector2f &starboard, const matrix::Vector2f &port);

	/** Write the tilt servos and read the clamped angles back; the collective servo drives both pods */
	void writeTiltServos(ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
			     const ActuatorVector &actuator_max);

	/**
	 * Remove from a shortfall the share a pod withheld by the steer band's
	 * choice: reported as saturation it would freeze the rate integrator
	 * against the small steady torques the integral exists to remove. Only
	 * a same-signed share is removed; a swinging, clamped or fixed tilt
	 * keeps reporting.
	 */
	static float discountHeld(float shortfall, float held);

	/** +1, -1 or 0: the direction of a shortfall, as the rate controller reads it */
	static float saturationSign(float shortfall);

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
	bool _surface_serves[3] {};	///< torque axes with control-surface effectiveness

	matrix::Vector3f _surface_torque{};	///< torque the clipped, trim-relative surface deflections deliver
	matrix::Vector3f _achieved_torque{};	///< torque the pods and tail delivered
	matrix::Vector3f _held_torque{};	///< torque a pod held inside the steer band leaves unserved by choice
	matrix::Vector<float, NUM_AXES> _unallocated_control{};	///< shortfall per axis less the held share

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_AIRSHIP_TLMIN>) _param_ca_airship_tlmin,
		(ParamFloat<px4::params::CA_AIRSHIP_TLMAX>) _param_ca_airship_tlmax,
		(ParamInt<px4::params::CA_AIRSHIP_GRP>) _param_ca_airship_grp,
		(ParamBool<px4::params::CA_AIRSHIP_TAIL>) _param_ca_airship_tail,
		(ParamFloat<px4::params::CA_AIRSHIP_CS_K>) _param_ca_airship_cs_k,
		(ParamFloat<px4::params::CA_AIRSHIP_TLT_R>) _param_ca_airship_tlt_r
	)
};
