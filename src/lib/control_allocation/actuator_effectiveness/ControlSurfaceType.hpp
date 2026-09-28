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
 * @file ControlSurfaceType.hpp
 *
 * The control-surface types of CA_SV_CSn_TYPE and which of them take torque.
 * Header-only, so a controller can read the allocator's surface configuration
 * without depending on the effectiveness classes that consume it.
 */

#pragma once

#include <cstdint>

/** Largest number of control surfaces: the CA_SV_CS{n} parameter instances (CA_SV_CS_COUNT at most) */
static constexpr int kControlSurfaceMaxCount = 8;

/** Control-surface type; the values are those of CA_SV_CSn_TYPE */
enum class ControlSurfaceType : int32_t {
	LeftAileron = 1,
	RightAileron = 2,
	Elevator = 3,
	Rudder = 4,
	LeftElevon = 5,
	RightElevon = 6,
	LeftVTail = 7,
	RightVTail = 8,
	LeftFlap = 9,
	RightFlap = 10,
	Airbrake = 11,
	Custom = 12,
	LeftATail = 13,
	RightATail = 14,
	SingleChannelAileron = 15,
	SteeringWheel = 16,
	LeftSpoiler = 17,
	RightSpoiler = 18,
};

/**
 * Whether a surface of this type gets its CA_SV_CSn_TRQ_* parameters as
 * torque; flaps, airbrakes, the steering wheel and spoilers get none. The
 * switch is exhaustive without a default so that a type added to the enum
 * without a case fails the build (-Wswitch) instead of silently taking torque.
 */
constexpr bool controlSurfaceTakesTorque(ControlSurfaceType surface_type)
{
	switch (surface_type) {
	case ControlSurfaceType::LeftFlap:
	case ControlSurfaceType::RightFlap:
	case ControlSurfaceType::Airbrake:
	case ControlSurfaceType::SteeringWheel:
	case ControlSurfaceType::LeftSpoiler:
	case ControlSurfaceType::RightSpoiler:
		return false;

	case ControlSurfaceType::LeftAileron:
	case ControlSurfaceType::RightAileron:
	case ControlSurfaceType::Elevator:
	case ControlSurfaceType::Rudder:
	case ControlSurfaceType::LeftElevon:
	case ControlSurfaceType::RightElevon:
	case ControlSurfaceType::LeftVTail:
	case ControlSurfaceType::RightVTail:
	case ControlSurfaceType::Custom:
	case ControlSurfaceType::LeftATail:
	case ControlSurfaceType::RightATail:
	case ControlSurfaceType::SingleChannelAileron:
		break;
	}

	return true; // also a value outside the enum, such as 0 (not set)
}
