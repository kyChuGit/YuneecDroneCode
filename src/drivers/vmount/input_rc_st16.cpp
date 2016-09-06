/****************************************************************************
*
*   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file input_rc_st16.cpp
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "input_rc_st16.h"

#include <errno.h>
#include <px4_posix.h>
#include <px4_defines.h>


namespace vmount
{


InputRCSt16::InputRCSt16()
	: InputRC(0, 0, 0)
{
}

InputRCSt16::~InputRCSt16()
{
}


void InputRCSt16::_read_control_data_from_subscription(ControlData &control_data)
{
	manual_control_setpoint_s manual_control_setpoint;
	orb_copy(ORB_ID(manual_control_setpoint), _get_subscription_fd(), &manual_control_setpoint);
	control_data.type = ControlData::Type::Angle;

	//roll
	control_data.type_data.angle.is_speed[0] = false;
	control_data.type_data.angle.angles[0] = 0.f; //not assigned
	control_data.stabilize_axis[0] = false;

	//pitch
	if (manual_control_setpoint.aux3 > 0.5f) {
		control_data.type_data.angle.is_speed[1] = true;
		control_data.type_data.angle.angles[1] = manual_control_setpoint.aux2 * M_PI_F;

	} else {
		//convert the input range to [-90, -3] deg
		const float pitch_min = -3.f / 180.f;
		const float pitch_max = -90.f / 180.f;
		control_data.type_data.angle.angles[1] = ((manual_control_setpoint.aux2 + 1.f) / 2.f * (pitch_max - pitch_min)
				+ pitch_min) * M_PI_F;
		control_data.type_data.angle.is_speed[1] = false;
	}

	control_data.stabilize_axis[1] = true;

	//yaw
	control_data.type_data.angle.is_speed[2] = true;
	control_data.type_data.angle.angles[2] = manual_control_setpoint.aux1 * M_PI_F;

	if (manual_control_setpoint.aux4 > 0.5f) {
		control_data.stabilize_axis[2] = true;

	} else if (manual_control_setpoint.aux4 > -0.5f) {
		control_data.stabilize_axis[2] = false;

	} else {
		control_data.stabilize_axis[2] = false;
		control_data.type_data.angle.angles[2] = 0.f;
		control_data.type_data.angle.is_speed[2] = false;
	}

	control_data.gimbal_shutter_retract = false;
}


} /* namespace vmount */
