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

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"

#include <drivers/drv_hrt.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

class ActuatorEffectivenessAirship : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessAirship(ModuleParams *parent) : ModuleParams(parent), _control_surfaces(this) {}
	virtual ~ActuatorEffectivenessAirship() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
			    const ActuatorVector &actuator_max) override;

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	const char *name() const override { return "Airship"; }

	// The tilt direction is the atan2 of the pod force demand, which is
	// meaningless near zero magnitude: stick noise alone would slam the tilt
	// between opposite directions. Steering therefore engages only above the
	// stick-noise floor and releases at half of it; inside the band the last
	// commanded direction stands, so a sign reversal there cannot retarget.
	static constexpr float kTiltSteerEngage = 0.02f;
	static constexpr float kTiltSteerRelease = 0.01f;

	// Pointing (near-)straight back, +180 and -180 deg realize the same thrust
	// direction at opposite ends of an end-stop servo: within this cone of the
	// negative x axis the previously committed end is kept, so perpendicular
	// noise cannot command a full-range sweep.
	static constexpr float kTiltRearCone = 0.05f;

private:
	struct SaturationFlags {
		bool roll_pos;
		bool roll_neg;
		bool pitch_pos;
		bool pitch_neg;
		bool yaw_pos;
		bool yaw_neg;
		bool thrust_x_pos;
		bool thrust_x_neg;
		bool thrust_y_pos;
		bool thrust_y_neg;
		bool thrust_z_pos;
		bool thrust_z_neg;
	};
	static void setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag);

	SaturationFlags _saturation_flags{};

	SlewRate<float> _tilt[2] {};	///< realized tilt [rad], held through zero-thrust
	float _tilt_target[2] {};	///< commanded tilt [rad] the slew tracks; holds through the hysteresis band
	bool _tilt_steering[2] {};	///< per-pod hysteresis state of the direction hold
	bool _armed{true};		///< assume armed until actuator_armed reports otherwise
	hrt_abstime _last_update_time{0};

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	ActuatorEffectivenessControlSurfaces _control_surfaces;

	// Actuator layout, decided when the actuators are declared
	int _first_control_surface_idx{0};
	int _first_tilt_idx{0};
	int _tilt_count{0};
	bool _has_tail{false};
	bool _independent{false};

	bool _surface_serves[3] {};	///< torque axes with control-surface effectiveness
	matrix::Vector3f _surface_torque{};	///< torque the clipped, trim-relative surface deflections can deliver
	float _achieved_roll{0.f};
	float _achieved_yaw{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_AIRSHIP_TLMIN>) _param_ca_airship_tlmin,
		(ParamFloat<px4::params::CA_AIRSHIP_TLMAX>) _param_ca_airship_tlmax,
		(ParamInt<px4::params::CA_AIRSHIP_GRP>) _param_ca_airship_grp,
		(ParamBool<px4::params::CA_AIRSHIP_TAIL>) _param_ca_airship_tail,
		(ParamFloat<px4::params::CA_AIRSHIP_CS_K>) _param_ca_airship_cs_k,
		(ParamFloat<px4::params::CA_AIRSHIP_TLT_R>) _param_ca_airship_tlt_r
	)
};
