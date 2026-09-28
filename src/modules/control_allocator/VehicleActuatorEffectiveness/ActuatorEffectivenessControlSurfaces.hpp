/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
#include "control_allocation/actuator_effectiveness/ControlSurfaceType.hpp"

#include <px4_platform_common/module_params.h>
#include <lib/slew_rate/SlewRate.hpp>

static constexpr float kSpoilersSlewRate = 0.5f; // slew rate for normalized spoilers setpoint [1/s]

class ActuatorEffectivenessControlSurfaces : public ModuleParams, public ActuatorEffectiveness
{
public:

	static constexpr int MAX_COUNT = kControlSurfaceMaxCount;

	using Type = ControlSurfaceType;

	/** Whether a surface of this type gets its TRQ parameters as torque (controlSurfaceTakesTorque()) */
	static constexpr bool takesTorque(Type surface_type) { return controlSurfaceTakesTorque(surface_type); }

	struct Params {
		Type type;

		matrix::Vector3f torque;
		float trim;
		float scale_flap;
		float scale_spoiler;
	};

	ActuatorEffectivenessControlSurfaces(ModuleParams *parent);
	virtual ~ActuatorEffectivenessControlSurfaces() = default;

	bool addActuators(Configuration &configuration);

	const char *name() const override { return "Control Surfaces"; }

	int count() const { return _count; }

	const Params &config(int idx) const { return _params[idx]; }

	void applyFlaps(float flaps_control, int first_actuator_idx, float dt, ActuatorVector &actuator_sp);
	void applySpoilers(float spoilers_control, int first_actuator_idx, float dt, ActuatorVector &actuator_sp);
	void applyLaunchLock(int first_actuator_idx, ActuatorVector &actuator_sp);

private:
	void updateParams() override;

	struct ParamHandles {
		param_t type;

		param_t torque[3];
		param_t trim;
		param_t scale_flap;
		param_t scale_spoiler;
	};
	ParamHandles _param_handles[MAX_COUNT];
	param_t _count_handle;
	param_t _param_handle_ca_cs_laun_lk;

	Params _params[MAX_COUNT] {};
	int32_t _count{0};
	int32_t _param_ca_cs_laun_lk{0};

	SlewRate<float> _flaps_setpoint_with_slewrate;
	SlewRate<float> _spoilers_setpoint_with_slewrate;
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_SV_FLAP_SLEW>) _param_ca_flap_slew
	)
};
