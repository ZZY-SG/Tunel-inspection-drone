/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualAcceleration.cpp
 */

#include "FlightTaskManualAcceleration.hpp"

using namespace matrix;

bool FlightTaskManualAcceleration::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitudeSmoothVel::activate(last_setpoint);

	_stick_acceleration_xy.resetPosition();

	// 接管时保留当前真实水平速度，避免引用 OFFBOARD 末帧速度 setpoint 引起的阶跃；
	// 同时做有限值检查和模长限幅，避免 EKF 瞬时速度异常偏大导致接管后继承过大速度目标
	Vector2f velocity_sp = _velocity.xy();

	if (!velocity_sp.isAllFinite()) {
		velocity_sp.setZero();

	} else {
		// 室内保护起点，可根据实际机型调整；不用 MPC_VEL_MANUAL(如 10 m/s) 这种普通人工上限直接当接管保护限幅
		const float takeover_velocity_limit = math::min(_param_mpc_vel_manual.get(), 2.0f);

		const float speed = velocity_sp.norm();

		if (speed > takeover_velocity_limit && speed > FLT_EPSILON) {
			velocity_sp *= takeover_velocity_limit / speed;
		}
	}

	_stick_acceleration_xy.resetVelocity(velocity_sp);

	// 接管时水平加速度目标强制为 0，不继承 OFFBOARD 末帧加速度、不使用瞬时 EKF 加速度，
	// 由 calculateDrag 阻尼逐渐刹车，速度归零后 lockPosition 自动锁当前点
	_stick_acceleration_xy.resetAcceleration(Vector2f(0.f, 0.f));

	return ret;
}

bool FlightTaskManualAcceleration::update()
{
	const vehicle_local_position_s vehicle_local_pos = _sub_vehicle_local_position.get();
	setMaxDistanceToGround(vehicle_local_pos.hagl_max_xy);
	bool ret = FlightTaskManualAltitudeSmoothVel::update();

	float max_hagl_ratio = 0.0f;

	if (PX4_ISFINITE(vehicle_local_pos.hagl_max_xy) && vehicle_local_pos.hagl_max_xy > FLT_EPSILON) {
		max_hagl_ratio = (vehicle_local_pos.dist_bottom) / vehicle_local_pos.hagl_max_xy;
	}

	// limit horizontal velocity near max hagl to decrease chance of larger gound distance jumps
	static constexpr float factor_threshold = 0.8f; // threshold ratio of max_hagl
	static constexpr float min_vel = 2.f; // minimum max-velocity near max_hagl

	if (max_hagl_ratio > factor_threshold) {
		const float vxy_max = math::min(vehicle_local_pos.vxy_max, _param_mpc_vel_manual.get());
		_stick_acceleration_xy.setVelocityConstraint(interpolate(max_hagl_ratio, factor_threshold, 1.f, vxy_max, min_vel));

	} else if (PX4_ISFINITE(vehicle_local_pos.vxy_max)) {
		_stick_acceleration_xy.setVelocityConstraint(vehicle_local_pos.vxy_max);
	}

	_stick_acceleration_xy.generateSetpoints(_sticks.getPitchRollExpo(), _yaw, _yaw_setpoint, _position,
			_velocity_setpoint_feedback.xy(), _deltatime);
	_stick_acceleration_xy.getSetpoints(_position_setpoint, _velocity_setpoint, _acceleration_setpoint);

	_constraints.want_takeoff = _checkTakeoff();

	// check if an external yaw handler is active and if yes, let it update the yaw setpoints
	_weathervane.update();

	if (_weathervane.isActive()) {
		_yaw_setpoint = NAN;

		// only enable the weathervane to change the yawrate when position lock is active (and thus the pos. sp. are NAN)
		if (Vector2f(_position_setpoint).isAllFinite()) {
			// vehicle is steady
			_yawspeed_setpoint += _weathervane.getWeathervaneYawrate();
		}
	}

	return ret;
}

void FlightTaskManualAcceleration::_ekfResetHandlerPositionXY(const matrix::Vector2f &delta_xy)
{
	_stick_acceleration_xy.addToPositionSetpoint(delta_xy);
}

void FlightTaskManualAcceleration::_ekfResetHandlerVelocityXY(const matrix::Vector2f &delta_vxy)
{
	_stick_acceleration_xy.resetVelocity(_velocity.xy());
}
