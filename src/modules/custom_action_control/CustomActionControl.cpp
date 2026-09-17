/****************************************************************************
 * SEARCH_TOP controller implementation.
 ****************************************************************************/

#include "CustomActionControl.hpp"

#include <geo/geo.h>
#include <lib/custom_action_protocol/CustomActionProtocol.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/log.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

using namespace time_literals;
using custom_action_protocol::Command;
using custom_action_protocol::Result;

namespace
{
constexpr hrt_abstime kRunInterval = 20_ms;
constexpr hrt_abstime kLocalPositionTimeout = 500_ms;
constexpr hrt_abstime kAttitudeTimeout = 200_ms;
constexpr hrt_abstime kActuatorTimeout = 200_ms;
constexpr hrt_abstime kControlModeTimeout = 1_s;
constexpr hrt_abstime kDirectActuatorTakeoverTimeout = 1_s;
constexpr hrt_abstime kStatusInterval = 200_ms;
constexpr hrt_abstime kTrimInvalidResetDelay = 250_ms;
constexpr hrt_abstime kPressMotorBlendTime = 300_ms;
constexpr float kMotorTrimFilterTimeConstant = 0.4f;
constexpr float kTrimLooseLimitMultiplier = 2.f;
constexpr uint8_t kMinimumMotorCount = 4;
constexpr uint8_t kContactThresholdFramesRequired = 3;
constexpr uint8_t kContactSensorCountRequired = 3;
const char *reasonName(uint8_t reason)
{
	switch (reason) {
	case custom_action_status_s::REASON_DIRECTION: return "direction";
	case custom_action_status_s::REASON_MAX_DISTANCE: return "max distance";
	case custom_action_status_s::REASON_TIMEOUT: return "timeout";
	case custom_action_status_s::REASON_SENSOR_TIMEOUT: return "sensor timeout";
	case custom_action_status_s::REASON_ESTIMATOR: return "estimator invalid";
	case custom_action_status_s::REASON_HEADING_RESET: return "heading reset";
	case custom_action_status_s::REASON_LAND: return "land";
	case custom_action_status_s::REASON_TOP_DISTANCE: return "top distance";
	case custom_action_status_s::REASON_ACTUATOR_TIMEOUT: return "direct actuator timeout";
	case custom_action_status_s::REASON_MOTOR_TRIM_MISMATCH: return "motor trim mismatch";
	default: return "none";
	}
}
}

ModuleBase::Descriptor CustomActionControl::desc{task_spawn, custom_command, print_usage};

CustomActionControl::CustomActionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

CustomActionControl::~CustomActionControl()
{
	ScheduleClear();
}

bool CustomActionControl::init()
{
	publishStatus(true);
	ScheduleOnInterval(kRunInterval);
	return true;
}

bool CustomActionControl::localStateValid() const
{
	const hrt_abstime now = hrt_absolute_time();
	return _local_position.timestamp != 0 && now >= _local_position.timestamp
	       && now - _local_position.timestamp <= kLocalPositionTimeout
	       && _local_position.xy_valid && _local_position.z_valid
	       && PX4_ISFINITE(_local_position.x) && PX4_ISFINITE(_local_position.y)
	       && PX4_ISFINITE(_local_position.z) && PX4_ISFINITE(_local_position.heading);
}

bool CustomActionControl::flightStateAllowsCustom() const
{
	return _param_enabled.get() && _vehicle_status.timestamp != 0
	       && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
	       && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	       && localStateValid();
}

bool CustomActionControl::sensorFresh(hrt_abstime now) const
{
	const hrt_abstime timeout = static_cast<hrt_abstime>(_param_sensor_timeout.get() * 1_s);

	if (_top_distance.valid_mask != 0x0F || _top_distance.timestamp == 0
	    || now < _top_distance.timestamp || now - _top_distance.timestamp > timeout) {
		return false;
	}

	for (uint8_t sensor = 0; sensor < 4; ++sensor) {
		if (_top_distance.timestamp_sample[sensor] == 0 || now < _top_distance.timestamp_sample[sensor]
		    || now - _top_distance.timestamp_sample[sensor] > timeout
		    || !PX4_ISFINITE(_top_distance.distance_m[sensor])) {
			return false;
		}
	}

	return true;
}

float CustomActionControl::minimumTopDistance() const
{
	float minimum_distance = INFINITY;

	for (float distance : _top_distance.distance_m) {
		minimum_distance = math::min(minimum_distance, distance);
	}

	return minimum_distance;
}

uint8_t CustomActionControl::topDistanceCountAtOrBelow(float threshold) const
{
	uint8_t count = 0;

	for (float distance : _top_distance.distance_m) {
		if (PX4_ISFINITE(distance) && distance <= threshold) {
			++count;
		}
	}

	return count;
}

void CustomActionControl::updateFilteredTopDistance()
{
	const float measured_distance = minimumTopDistance();

	if (!PX4_ISFINITE(measured_distance)) {
		return;
	}

	if (!PX4_ISFINITE(_filtered_top_distance)) {
		_filtered_top_distance = measured_distance;

	} else {
		_filtered_top_distance += _param_top_filter.get() * (measured_distance - _filtered_top_distance);
	}
}

bool CustomActionControl::isPrecontactState() const
{
	return _state == custom_action_status_s::STATE_SEARCH_TOP
	       || _state == custom_action_status_s::STATE_TOP_APPROACH
	       || _state == custom_action_status_s::STATE_CONTACT_VERIFY;
}

float CustomActionControl::verticalVelocityNed() const
{
	if (_state == custom_action_status_s::STATE_TOP_APPROACH) {
		return -_param_approach_velocity.get();
	}

	if (_state == custom_action_status_s::STATE_CONTACT_VERIFY) {
		return -_param_verify_velocity.get();
	}

	return -_param_top_velocity.get();
}

float CustomActionControl::constrainedTrimTime(float available_time_s) const
{
	const float minimum_time = math::max(_param_trim_time_min.get(), 0.05f);
	const float maximum_time = math::max(_param_trim_time_max.get(), minimum_time);
	const float ratio = math::constrain(_param_trim_ratio.get(), 0.01f, 1.f);
	return math::constrain(math::max(available_time_s, 0.f) * ratio, minimum_time, maximum_time);
}

void CustomActionControl::resetMotorTrimCandidate(MotorTrimCandidate &candidate)
{
	candidate = MotorTrimCandidate{};
}

void CustomActionControl::resetMotorTrim()
{
	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		_motor_trim[i] = 0.f;
		_press_entry_output[i] = 0.f;
		_press_base_output[i] = 0.f;
	}

	resetMotorTrimCandidate(_search_loose_trim);
	resetMotorTrimCandidate(_search_strict_trim);
	resetMotorTrimCandidate(_approach_strict_trim);
	_motor_trim_source = MotorTrimSource::None;
	_motor_trim_mask = 0;
	_press_output_mask = 0;
	_motor_trim_valid = false;
	_motor_output_limited = false;
	_pressure_ramp_complete = false;
	_active_pressure_gain = 0.f;
	_limiting_motor = 0;
	_search_trim_required_s = 0.f;
	_approach_trim_required_s = 0.f;
	_press_output_started = 0;
	_pressure_ramp_started = 0;
}

bool CustomActionControl::motorTrimSampleValid(hrt_abstime now, bool loose, uint16_t &sample_mask) const
{
	sample_mask = 0;

	if (!sensorFresh(now)
	    || minimumTopDistance() <= _param_contact_distance.get()
	    || _vehicle_attitude.timestamp == 0 || now < _vehicle_attitude.timestamp
	    || now - _vehicle_attitude.timestamp > kAttitudeTimeout
	    || _vehicle_angular_velocity.timestamp == 0 || now < _vehicle_angular_velocity.timestamp
	    || now - _vehicle_angular_velocity.timestamp > kAttitudeTimeout
	    || _allocated_motors.timestamp == 0 || now < _allocated_motors.timestamp
	    || now - _allocated_motors.timestamp > kActuatorTimeout) {
		return false;
	}

	const matrix::Eulerf attitude{matrix::Quatf{_vehicle_attitude.q}};
	const float limit_multiplier = loose ? kTrimLooseLimitMultiplier : 1.f;
	const float angle_limit = math::radians(_param_trim_angle.get() * limit_multiplier);
	const float rate_limit = math::radians(_param_trim_rate.get() * limit_multiplier);

	if (!PX4_ISFINITE(attitude.phi()) || !PX4_ISFINITE(attitude.theta())
	    || fabsf(attitude.phi()) > angle_limit || fabsf(attitude.theta()) > angle_limit
	    || !PX4_ISFINITE(_vehicle_angular_velocity.xyz[0])
	    || !PX4_ISFINITE(_vehicle_angular_velocity.xyz[1])
	    || fabsf(_vehicle_angular_velocity.xyz[0]) > rate_limit
	    || fabsf(_vehicle_angular_velocity.xyz[1]) > rate_limit) {
		return false;
	}

	uint8_t motor_count = 0;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		const float control = _allocated_motors.control[i];

		if (PX4_ISFINITE(control)) {
			if (control < 0.f || control > _param_motor_limit.get()) {
				return false;
			}

			sample_mask |= static_cast<uint16_t>(1u << i);
			++motor_count;
		}
	}

	return motor_count >= kMinimumMotorCount;
}

void CustomActionControl::updateMotorTrimCandidate(MotorTrimCandidate &candidate, hrt_abstime now, bool loose,
		float required_time_s, const char *name)
{
	uint16_t sample_mask = 0;

	if (!motorTrimSampleValid(now, loose, sample_mask)) {
		if (!candidate.valid) {
			if (candidate.invalid_since == 0) {
				candidate.invalid_since = now;
			}

			candidate.previous_sample_valid = false;
			candidate.last_update = now;

			if (candidate.initialized && now >= candidate.invalid_since
			    && now - candidate.invalid_since >= kTrimInvalidResetDelay) {
				resetMotorTrimCandidate(candidate);
			}
		}

		return;
	}

	if (!candidate.initialized || sample_mask != candidate.mask) {
		resetMotorTrimCandidate(candidate);
		candidate.mask = sample_mask;

		for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
			if (sample_mask & (1u << i)) {
				candidate.filtered[i] = _allocated_motors.control[i];
			}
		}

		candidate.initialized = true;
		candidate.previous_sample_valid = true;
		candidate.last_update = now;
		return;
	}

	const float dt = now >= candidate.last_update
			 ? math::constrain(static_cast<float>(now - candidate.last_update) * 1e-6f, 0.001f, 0.1f)
			 : 0.001f;
	const float alpha = dt / (kMotorTrimFilterTimeConstant + dt);

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		if (sample_mask & (1u << i)) {
			candidate.filtered[i] += alpha * (_allocated_motors.control[i] - candidate.filtered[i]);
		}
	}

	if (candidate.previous_sample_valid) {
		candidate.valid_time_s += dt;
	}

	candidate.previous_sample_valid = true;
	candidate.last_update = now;
	candidate.invalid_since = 0;

	if (!candidate.valid && candidate.valid_time_s >= required_time_s) {
		candidate.valid = true;
		PX4_INFO("[MOTOR_TRIM] %s ready time=%.2f/%.2f mask=0x%03x first4=(%.3f %.3f %.3f %.3f)",
			 name, (double)candidate.valid_time_s, (double)required_time_s, (unsigned)candidate.mask,
			 (double)candidate.filtered[0], (double)candidate.filtered[1],
			 (double)candidate.filtered[2], (double)candidate.filtered[3]);
	}
}

const char *CustomActionControl::motorTrimSourceName(MotorTrimSource source) const
{
	switch (source) {
	case MotorTrimSource::ApproachStrict: return "approach-strict";
	case MotorTrimSource::SearchStrict: return "search-strict";
	case MotorTrimSource::SearchLoose: return "search-loose";
	default: return "none";
	}
}

void CustomActionControl::selectBestMotorTrim()
{
	const MotorTrimCandidate *candidate = nullptr;
	MotorTrimSource source = MotorTrimSource::None;

	if (_approach_strict_trim.valid) {
		candidate = &_approach_strict_trim;
		source = MotorTrimSource::ApproachStrict;

	} else if (_search_strict_trim.valid) {
		candidate = &_search_strict_trim;
		source = MotorTrimSource::SearchStrict;

	} else if (_search_loose_trim.valid) {
		candidate = &_search_loose_trim;
		source = MotorTrimSource::SearchLoose;
	}

	if (candidate == nullptr) {
		_motor_trim_source = MotorTrimSource::None;
		_motor_trim_mask = 0;
		_motor_trim_valid = false;

		for (float &output : _motor_trim) {
			output = 0.f;
		}

		return;
	}

	const bool source_changed = source != _motor_trim_source;
	_motor_trim_source = source;
	_motor_trim_mask = candidate->mask;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		_motor_trim[i] = candidate->filtered[i];
	}

	_motor_trim_valid = true;

	if (source_changed) {
		PX4_INFO("[MOTOR_TRIM] selected %s mask=0x%03x first4=(%.3f %.3f %.3f %.3f)",
			 motorTrimSourceName(source), (unsigned)_motor_trim_mask,
			 (double)_motor_trim[0], (double)_motor_trim[1],
			 (double)_motor_trim[2], (double)_motor_trim[3]);
	}
}

void CustomActionControl::updateMotorTrim(hrt_abstime now, bool actuator_updated)
{
	if (!actuator_updated) {
		return;
	}

	if (_state == custom_action_status_s::STATE_SEARCH_TOP) {
		updateMotorTrimCandidate(_search_loose_trim, now, true, _search_trim_required_s, "search-loose");
		updateMotorTrimCandidate(_search_strict_trim, now, false, _search_trim_required_s, "search-strict");

	} else if (_state == custom_action_status_s::STATE_TOP_APPROACH) {
		updateMotorTrimCandidate(_approach_strict_trim, now, false, _approach_trim_required_s, "approach-strict");
	}

	selectBestMotorTrim();
}

bool CustomActionControl::directActuatorReady(hrt_abstime now) const
{
	return _vehicle_control_mode.timestamp != 0
	       && now >= _vehicle_control_mode.timestamp
	       && now - _vehicle_control_mode.timestamp <= kControlModeTimeout
	       && _vehicle_control_mode.flag_armed
	       && _vehicle_control_mode.flag_control_offboard_enabled
	       && !_vehicle_control_mode.flag_control_allocation_enabled;
}

bool CustomActionControl::preparePressHandover(hrt_abstime now)
{
	if (_allocated_motors.timestamp == 0
	    || now < _allocated_motors.timestamp || now - _allocated_motors.timestamp > kActuatorTimeout) {
		return false;
	}

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		if (i < kMinimumMotorCount) {
			const float current_output = _allocated_motors.control[i];

			if (!PX4_ISFINITE(current_output) || current_output <= FLT_EPSILON
			    || current_output > _param_motor_limit.get()) {
				return false;
			}

			_press_entry_output[i] = current_output;
			_press_base_output[i] = current_output;

		} else {
			_press_entry_output[i] = 0.f;
			_press_base_output[i] = 0.f;
		}
	}

	_press_output_mask = (1u << kMinimumMotorCount) - 1u;
	PX4_INFO("[CONTACT_PRESS] current-output baseline captured entry=(%.3f %.3f %.3f %.3f)",
		 (double)_press_entry_output[0], (double)_press_entry_output[1],
		 (double)_press_entry_output[2], (double)_press_entry_output[3]);
	return true;
}

void CustomActionControl::publishDirectMotorSetpoint(hrt_abstime now)
{
	if (_press_output_mask == 0 || !directActuatorReady(now)) {
		return;
	}

	const bool takeover_start = _press_output_started == 0;

	if (takeover_start) {
		_press_output_started = now;
		PX4_INFO("[CONTACT_PRESS] direct actuator ready; starting 300 ms current-output handover");
	}

	const hrt_abstime blend_elapsed_us = now >= _press_output_started ? now - _press_output_started : 0;
	const float blend_progress = math::constrain(static_cast<float>(blend_elapsed_us)
				     / static_cast<float>(kPressMotorBlendTime), 0.f, 1.f);
	const bool pressure_ramp_start = blend_progress >= 1.f
					 && _state == custom_action_status_s::STATE_CONTACT_PRESS_WAIT;

	if (pressure_ramp_start) {
		_state = custom_action_status_s::STATE_CONTACT_PRESS;
		_reason = custom_action_status_s::REASON_NONE;
		_pressure_ramp_started = now;
		PX4_INFO("[CONTACT_PRESS] current-output handover complete; pressure ramp active");
	}

	const float pressure_elapsed = _pressure_ramp_started != 0 && now >= _pressure_ramp_started
				       ? static_cast<float>(now - _pressure_ramp_started) * 1e-6f
				       : 0.f;
	actuator_motors_s motors{};
	motors.timestamp = now;
	motors.timestamp_sample = now;
	motors.reversible_flags = 0;

	const float target_time_s = math::max(_param_press_time.get(), 0.2f);
	const float pressure_progress = math::constrain(pressure_elapsed / target_time_s, 0.f, 1.f);
	const float requested_gain = math::max(_param_press_gain.get(), 0.f) * pressure_progress;
	const float requested_scale = 1.f + requested_gain;
	float scale = requested_scale;
	const float motor_limit = _param_motor_limit.get();
	uint8_t limiting_motor = 0;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		if ((_press_output_mask & (1u << i)) && _press_base_output[i] > FLT_EPSILON) {
			const float motor_scale_limit = motor_limit / _press_base_output[i];

			if (motor_scale_limit < scale) {
				scale = motor_scale_limit;
				limiting_motor = static_cast<uint8_t>(i + 1);
			}
		}
	}

	// Never let any motor fall below its contact baseline. Keeping the scale at
	// least the previously applied value also makes the ramp monotonic if a
	// parameter changes while pressure mode is active.
	scale = math::max(scale, 1.f + _active_pressure_gain);
	_active_pressure_gain = math::max(scale - 1.f, 0.f);
	const bool output_limited = scale + FLT_EPSILON < requested_scale;
	_pressure_ramp_complete = pressure_progress >= 1.f;
	_limiting_motor = output_limited ? limiting_motor : 0;

	if (output_limited != _motor_output_limited) {
		if (output_limited) {
			PX4_WARN("[CONTACT_PRESS] motor %u limit active requested=%.3f applied=%.3f",
				 (unsigned)_limiting_motor, (double)requested_gain, (double)_active_pressure_gain);

		} else {
			PX4_INFO("[CONTACT_PRESS] motor limit cleared");
		}
	}

	_motor_output_limited = output_limited;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		if (_press_output_mask & (1u << i)) {
			const float pressure_target = _press_base_output[i] * scale;
			motors.control[i] = _press_entry_output[i]
					    + blend_progress * (pressure_target - _press_entry_output[i]);

		} else {
			motors.control[i] = NAN;
		}
	}

	_actuator_motors_pub.publish(motors);

	if (pressure_ramp_start) {
		publishStatus(true);
		publishAsyncAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
				static_cast<uint8_t>(Result::ContactPressEntered), _active_request_id,
				_active_source_system, _active_source_component);
	}
}

bool CustomActionControl::captureHoldPoint()
{
	if (!localStateValid()) {
		return false;
	}

	_hold_x = _local_position.x;
	_hold_y = _local_position.y;
	_hold_z = _local_position.z;
	_locked_yaw = _local_position.heading;
	return true;
}

void CustomActionControl::publishAck(const vehicle_command_s &command, uint8_t result, uint8_t project_result)
{
	publishAsyncAck(result, project_result, static_cast<uint16_t>(lroundf(command.param3)),
			command.source_system, command.source_component);
}

void CustomActionControl::publishAsyncAck(uint8_t result, uint8_t project_result, uint16_t request_id,
		uint8_t target_system, uint16_t target_component)
{
	vehicle_command_ack_s ack{};
	ack.timestamp = hrt_absolute_time();
	ack.command = custom_action_protocol::kMavCmdUser1;
	ack.result = result;
	ack.result_param1 = 0;
	ack.result_param2 = custom_action_protocol::encodeResult(request_id, static_cast<Result>(project_result));
	ack.target_system = target_system;
	ack.target_component = target_component;
	ack.from_external = false;
	_vehicle_command_ack_pub.publish(ack);
}

void CustomActionControl::startSearchTop(const vehicle_command_s &command, uint16_t request_id)
{
	const hrt_abstime now = hrt_absolute_time();
	Result rejection = Result::None;

	if (_owner != custom_action_status_s::OWNER_LEGACY || _state != custom_action_status_s::STATE_INACTIVE) {
		rejection = Result::StartBusy;

	} else if (!_param_enabled.get()) {
		rejection = Result::StartDisabled;

	} else if (_vehicle_status.timestamp == 0
		   || _vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		rejection = Result::StartNotArmed;

	} else if (_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		rejection = Result::StartNotOffboard;

	} else if (!localStateValid() || !captureHoldPoint()) {
		rejection = Result::StartEstimatorInvalid;

	} else if (!sensorFresh(now)) {
		rejection = Result::StartSensorInvalid;

	} else if (!_param_close_start_enabled.get()
		   && minimumTopDistance() <= (_param_top_gap.get() + _param_top_hysteresis.get())) {
		// Normal flight must start outside the slow-approach threshold so a covered
		// or contaminated sensor cannot make SEARCH_TOP begin next to contact.
		// CUST_CLOSE_EN bypasses only this initial gate for controlled bench tests.
		rejection = Result::StartDistanceTooClose;
	}

	if (rejection != Result::None) {
		PX4_WARN("[SEARCH_TOP] start rejected: result=%u", (unsigned)rejection);
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(rejection));
		return;
	}

	_state = custom_action_status_s::STATE_SEARCH_TOP;
	_owner = custom_action_status_s::OWNER_CUSTOM;
	_reason = custom_action_status_s::REASON_NONE;
	_handover_id = 0;
	_active_request_id = request_id;
	_active_source_system = command.source_system;
	_active_source_component = command.source_component;
	_start_z = _hold_z;
	_heading_reset_counter = _local_position.heading_reset_counter;
	_search_started = now;
	_last_top_distance_sequence = _top_distance.sequence;
	_filtered_top_distance = minimumTopDistance();
	_verify_min_distance = NAN;
	_verify_max_distance = NAN;
	_contact_threshold_frames = 0;
	_verify_started = 0;
	_press_started = 0;
	_handover_started = 0;
	resetMotorTrim();
	const float search_distance_time = _param_top_distance.get()
					   / math::max(_param_top_velocity.get(), 0.01f);
	const float search_available_time = math::min(_param_top_time.get(), search_distance_time);
	_search_trim_required_s = constrainedTrimTime(search_available_time);
	PX4_INFO("[SEARCH_TOP] entered xyz=(%.2f, %.2f, %.2f) yaw=%.2f",
		 (double)_hold_x, (double)_hold_y, (double)_hold_z, (double)_locked_yaw);
	PX4_INFO("[MOTOR_TRIM] search capture target=%.2f s", (double)_search_trim_required_s);
	publishStatus(true);
	publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
		   static_cast<uint8_t>(Result::CustomStarted));
}

void CustomActionControl::beginHandover(uint8_t reason, uint16_t handover_id,
		uint8_t target_system, uint16_t target_component, bool notify_pending)
{
	if (!captureHoldPoint()) {
		releaseToLegacy(reason);
		return;
	}

	_state = custom_action_status_s::STATE_INACTIVE;
	_owner = custom_action_status_s::OWNER_HANDOVER;
	_reason = reason;
	_handover_id = handover_id == 0 ? 1 : handover_id;
	_handover_started = hrt_absolute_time();
	_verify_started = 0;
	_press_started = 0;
	_contact_threshold_frames = 0;
	resetMotorTrim();
	PX4_WARN("[CUSTOM] handover id=%u reason=%s(%u) hold=(%.2f, %.2f, %.2f)",
		 _handover_id, reasonName(reason), reason,
		 (double)_hold_x, (double)_hold_y, (double)_hold_z);
	publishStatus(true);

	if (notify_pending) {
		publishAsyncAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
				static_cast<uint8_t>(Result::HandoverPending), _handover_id,
				target_system, target_component);
	}
}

void CustomActionControl::handleDirectionIntent(const vehicle_command_s &command, uint16_t request_id)
{
	if (_owner == custom_action_status_s::OWNER_CUSTOM) {
		PX4_INFO("[SEARCH_TOP] cancelled by direction=%d", (int)lroundf(command.param2));
		beginHandover(custom_action_status_s::REASON_DIRECTION, request_id,
			      command.source_system, command.source_component, false);
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			   static_cast<uint8_t>(Result::ButtonConsumed));

	} else if (_owner == custom_action_status_s::OWNER_HANDOVER) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(Result::HandoverPending));

	} else if (_owner == custom_action_status_s::OWNER_LEGACY) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			   static_cast<uint8_t>(Result::LegacyAllowed));

	} else {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(Result::None));
	}
}

void CustomActionControl::handleRebaseComplete(const vehicle_command_s &command, uint16_t request_id)
{
	const uint16_t supplied_handover_id = static_cast<uint16_t>(lroundf(command.param2));

	if (_owner != custom_action_status_s::OWNER_HANDOVER || supplied_handover_id != _handover_id) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(Result::HandoverPending));
		return;
	}

	_owner = custom_action_status_s::OWNER_LEGACY;
	_reason = custom_action_status_s::REASON_NONE;
	_handover_id = 0;
	_handover_started = 0;
	publishStatus(true);
	publishAsyncAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			static_cast<uint8_t>(Result::RebaseAccepted), request_id,
			command.source_system, command.source_component);
	PX4_INFO("[CUSTOM] V3 rebase accepted");
}

void CustomActionControl::handleCommand(const vehicle_command_s &command)
{
	if (command.command == vehicle_command_s::VEHICLE_CMD_NAV_LAND) {
		if (_owner == custom_action_status_s::OWNER_CUSTOM
		    || _owner == custom_action_status_s::OWNER_HANDOVER) {
			takeCommanderOwnership(custom_action_status_s::REASON_LAND);
		}

		return;
	}

	if (command.command != custom_action_protocol::kMavCmdUser1
	    || command.target_component != custom_action_protocol::kComponentId) {
		return;
	}

	const int action = lroundf(command.param1);
	const uint16_t request_id = static_cast<uint16_t>(lroundf(command.param3));

	switch (static_cast<Command>(action)) {
	case Command::SearchTop:
		startSearchTop(command, request_id);
		break;

	case Command::DirectionIntent:
		handleDirectionIntent(command, request_id);
		break;

	case Command::RebaseComplete:
		handleRebaseComplete(command, request_id);
		break;

	default:
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED,
			   static_cast<uint8_t>(Result::None));
		break;
	}
}

void CustomActionControl::enterTopApproach()
{
	_state = custom_action_status_s::STATE_TOP_APPROACH;
	_owner = custom_action_status_s::OWNER_CUSTOM;
	_verify_started = 0;
	_verify_min_distance = NAN;
	_verify_max_distance = NAN;
	_contact_threshold_frames = 0;
	resetMotorTrimCandidate(_approach_strict_trim);
	selectBestMotorTrim();
	const float available_distance = math::max(minimumTopDistance() - _param_contact_distance.get(), 0.f);
	const float available_time = available_distance / math::max(_param_approach_velocity.get(), 0.005f);
	_approach_trim_required_s = constrainedTrimTime(available_time);
	PX4_INFO("[TOP_FOUND] raw=%.3f filtered=%.3f m; slow approach",
		 (double)minimumTopDistance(), (double)_filtered_top_distance);
	PX4_INFO("[MOTOR_TRIM] diagnostic approach capture target=%.2f s",
		 (double)_approach_trim_required_s);
	publishStatus(true);
}

void CustomActionControl::enterContactVerify(hrt_abstime now)
{
	_state = custom_action_status_s::STATE_CONTACT_VERIFY;
	_verify_started = now;
	_verify_min_distance = _filtered_top_distance;
	_verify_max_distance = _filtered_top_distance;
	_contact_threshold_frames = 0;
	PX4_INFO("[CONTACT_VERIFY] raw=%.3f filtered=%.3f m after %u consecutive raw frames",
		 (double)minimumTopDistance(), (double)_filtered_top_distance,
		 (unsigned)kContactThresholdFramesRequired);
	publishStatus(true);
}

void CustomActionControl::enterContactPress(hrt_abstime now)
{
	if (_state != custom_action_status_s::STATE_CONTACT_VERIFY || !captureHoldPoint()
	    || !preparePressHandover(now)) {
		PX4_WARN("[CONTACT_PRESS] current motor output unavailable or above limit");
		beginHandover(custom_action_status_s::REASON_ACTUATOR_TIMEOUT, _active_request_id,
			      _active_source_system, _active_source_component, true);
		return;
	}

	_state = custom_action_status_s::STATE_CONTACT_PRESS_WAIT;
	_owner = custom_action_status_s::OWNER_CUSTOM;
	_reason = custom_action_status_s::REASON_NONE;
	_verify_started = 0;
	_press_started = now;
	_press_output_started = 0;
	_pressure_ramp_started = 0;
	_contact_threshold_frames = 0;
	PX4_INFO("[CONTACT_PRESS] contact confirmed at z=%.2f; waiting for direct actuator takeover",
		 (double)_hold_z);
	publishStatus(true);
}

void CustomActionControl::releaseToLegacy(uint8_t reason)
{
	_state = custom_action_status_s::STATE_INACTIVE;
	_owner = custom_action_status_s::OWNER_LEGACY;
	_reason = reason;
	_handover_id = 0;
	_search_started = 0;
	_handover_started = 0;
	_verify_started = 0;
	_press_started = 0;
	_contact_threshold_frames = 0;
	_filtered_top_distance = NAN;
	resetMotorTrim();
	publishStatus(true);
}

void CustomActionControl::takeCommanderOwnership(uint8_t reason)
{
	_state = custom_action_status_s::STATE_INACTIVE;
	_owner = custom_action_status_s::OWNER_COMMANDER;
	_reason = reason;
	_handover_id = 0;
	_search_started = 0;
	_handover_started = 0;
	_verify_started = 0;
	_press_started = 0;
	_contact_threshold_frames = 0;
	_filtered_top_distance = NAN;
	resetMotorTrim();
	PX4_INFO("[CUSTOM] control released to Commander: %s", reasonName(reason));
	publishStatus(true);
}

void CustomActionControl::publishControlSetpoint(hrt_abstime now)
{
	const bool precontact = isPrecontactState();
	const bool contact_press = _state == custom_action_status_s::STATE_CONTACT_PRESS
				   || _state == custom_action_status_s::STATE_CONTACT_PRESS_WAIT;
	offboard_control_mode_s control_mode{};
	control_mode.timestamp = now;

	if (contact_press) {
		// Direct actuator mode disables the position, attitude, rate and control
		// allocation pipeline. The active command blends monotonically into the
		// aligned filtered trim, then multiplies it by the common pressure gain.
		control_mode.direct_actuator = true;
		_offboard_control_mode_pub.publish(control_mode);
		publishDirectMotorSetpoint(now);
		return;
	}

	control_mode.position = true;
	control_mode.velocity = precontact;
	_offboard_control_mode_pub.publish(control_mode);

	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = now;
	setpoint.position[0] = _hold_x;
	setpoint.position[1] = _hold_y;
	setpoint.position[2] = precontact ? NAN : _hold_z;
	setpoint.velocity[0] = NAN;
	setpoint.velocity[1] = NAN;
	setpoint.velocity[2] = precontact ? verticalVelocityNed() : NAN;

	for (int i = 0; i < 3; ++i) {
		setpoint.acceleration[i] = NAN;
		setpoint.jerk[i] = NAN;
	}

	setpoint.yaw = _locked_yaw;
	setpoint.yawspeed = NAN;
	_trajectory_setpoint_pub.publish(setpoint);
}

void CustomActionControl::publishStatus(bool force)
{
	const hrt_abstime now = hrt_absolute_time();

	if (!force && now - _last_status_publish < kStatusInterval) {
		return;
	}

	custom_action_status_s status{};
	status.timestamp = now;
	status.state = _state;
	status.active = _state != custom_action_status_s::STATE_INACTIVE;
	status.action = status.active ? custom_action_status_s::ACTION_SEARCH_TOP : custom_action_status_s::ACTION_NONE;
	status.control_owner = _owner;
	status.handover_id = _handover_id;
	status.reason = _reason;
	status.pressure_target_gain = math::max(_param_press_gain.get(), 0.f);
	status.pressure_applied_gain = _active_pressure_gain;
	status.pressure_time_s = math::max(_param_press_time.get(), 0.2f);
	status.pressure_progress = 0.f;

	if (_state == custom_action_status_s::STATE_CONTACT_PRESS && _pressure_ramp_started != 0
	    && now >= _pressure_ramp_started) {
		const float elapsed_s = static_cast<float>(now - _pressure_ramp_started) * 1e-6f;
		status.pressure_progress = math::constrain(elapsed_s / status.pressure_time_s, 0.f, 1.f);
	}

	status.trim_source = static_cast<uint8_t>(_motor_trim_source);
	status.trim_candidate_mask = (_search_loose_trim.valid ? custom_action_status_s::TRIM_CANDIDATE_SEARCH_LOOSE : 0)
				     | (_search_strict_trim.valid ? custom_action_status_s::TRIM_CANDIDATE_SEARCH_STRICT : 0)
				     | (_approach_strict_trim.valid ? custom_action_status_s::TRIM_CANDIDATE_APPROACH_STRICT : 0);
	status.limiting_motor = _limiting_motor;
	status.pressure_limited = _motor_output_limited;
	status.pressure_ramp_complete = _pressure_ramp_complete;
	_status_pub.publish(status);
	_last_status_publish = now;
}

void CustomActionControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	updateParams();
	_vehicle_local_position_sub.update(&_local_position);
	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);
	const bool actuator_motors_updated = _actuator_motors_sub.update(&_allocated_motors);
	const bool top_distance_updated = _top_distance_sub.update(&_top_distance);

	vehicle_command_s command{};

	for (int i = 0; i < vehicle_command_s::ORB_QUEUE_LENGTH && _vehicle_command_sub.update(&command); ++i) {
		handleCommand(command);
	}

	// Commands can create state timestamps using hrt_absolute_time(). Capture the
	// cycle time afterwards so elapsed-time calculations can never start from an
	// older timestamp and underflow hrt_abstime.
	const hrt_abstime now = hrt_absolute_time();

	const bool armed = _vehicle_status.timestamp != 0
			   && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	if (_owner == custom_action_status_s::OWNER_COMMANDER) {
		if (!armed) {
			releaseToLegacy(custom_action_status_s::REASON_NONE);
		}

		publishStatus();
		return;
	}

	if ((_owner == custom_action_status_s::OWNER_CUSTOM
	     || _owner == custom_action_status_s::OWNER_HANDOVER)
	    && (!armed || _vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD)) {
		releaseToLegacy(custom_action_status_s::REASON_NONE);
	}

	bool new_distance_frame = false;

	if (top_distance_updated && _top_distance.sequence != _last_top_distance_sequence) {
		_last_top_distance_sequence = _top_distance.sequence;
		updateFilteredTopDistance();
		new_distance_frame = true;
	}

	if (_state == custom_action_status_s::STATE_SEARCH_TOP
	    || _state == custom_action_status_s::STATE_TOP_APPROACH) {
		updateMotorTrim(now, actuator_motors_updated);
	}

	if (isPrecontactState()) {
		if (!localStateValid()) {
			beginHandover(custom_action_status_s::REASON_ESTIMATOR, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (_local_position.heading_reset_counter != _heading_reset_counter) {
			beginHandover(custom_action_status_s::REASON_HEADING_RESET, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (!sensorFresh(now)) {
			beginHandover(custom_action_status_s::REASON_SENSOR_TIMEOUT, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (_start_z - _local_position.z >= _param_top_distance.get()) {
			beginHandover(custom_action_status_s::REASON_MAX_DISTANCE, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (_search_started != 0 && now >= _search_started
			   && now - _search_started >= static_cast<hrt_abstime>(_param_top_time.get() * 1_s)) {
			PX4_WARN("[TOP_TIMEOUT] raw=%.3f filtered=%.3f m",
				 (double)minimumTopDistance(), (double)_filtered_top_distance);
			beginHandover(custom_action_status_s::REASON_TIMEOUT, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (new_distance_frame && _state == custom_action_status_s::STATE_SEARCH_TOP
			   && _filtered_top_distance <= _param_top_gap.get()) {
			enterTopApproach();

		} else if (new_distance_frame && _state == custom_action_status_s::STATE_TOP_APPROACH) {
			if (_filtered_top_distance > _param_top_gap.get() + _param_top_hysteresis.get()) {
				_state = custom_action_status_s::STATE_SEARCH_TOP;
				_contact_threshold_frames = 0;
				resetMotorTrimCandidate(_approach_strict_trim);
				selectBestMotorTrim();
				PX4_INFO("[TOP_APPROACH] top lost; resume search");
				publishStatus(true);

			} else {
				const float raw_minimum_distance = minimumTopDistance();

				if (raw_minimum_distance <= _param_contact_distance.get()
				    && topDistanceCountAtOrBelow(_param_contact_distance.get()) >= kContactSensorCountRequired) {
					if (_contact_threshold_frames < kContactThresholdFramesRequired) {
						++_contact_threshold_frames;
					}

					if (_contact_threshold_frames >= kContactThresholdFramesRequired) {
						enterContactVerify(now);
					}

				} else {
					_contact_threshold_frames = 0;
				}
			}

		} else if (new_distance_frame && _state == custom_action_status_s::STATE_CONTACT_VERIFY) {
			const float raw_minimum_distance = minimumTopDistance();

			if (raw_minimum_distance > _param_contact_distance.get() + _param_top_hysteresis.get()
			    || topDistanceCountAtOrBelow(_param_contact_distance.get()) < kContactSensorCountRequired) {
				enterTopApproach();

			} else {
				_verify_min_distance = math::min(_verify_min_distance, _filtered_top_distance);
				_verify_max_distance = math::max(_verify_max_distance, _filtered_top_distance);

				if (_verify_max_distance - _verify_min_distance > _param_stability_band.get()) {
					_verify_started = now;
					_verify_min_distance = _filtered_top_distance;
					_verify_max_distance = _filtered_top_distance;

				} else if (_verify_started != 0 && now >= _verify_started
					   && now - _verify_started >= static_cast<hrt_abstime>(_param_contact_time.get() * 1_s)) {
					enterContactPress(now);
				}
			}
		}
	}

	if (_state == custom_action_status_s::STATE_CONTACT_PRESS_WAIT && _press_started != 0
	    && now >= _press_started && !directActuatorReady(now)
	    && now - _press_started >= kDirectActuatorTakeoverTimeout) {
		PX4_ERR("[CONTACT_PRESS] direct actuator takeover timed out");
		beginHandover(custom_action_status_s::REASON_ACTUATOR_TIMEOUT, _active_request_id,
			      _active_source_system, _active_source_component, true);
	}

	const hrt_abstime handover_elapsed = (_handover_started != 0 && now >= _handover_started)
					     ? now - _handover_started
					     : 0;
	const bool handover_output_allowed = _owner != custom_action_status_s::OWNER_HANDOVER
		|| handover_elapsed < static_cast<hrt_abstime>(_param_handover_timeout.get() * 1_s);

	if ((_owner == custom_action_status_s::OWNER_CUSTOM
	     || (_owner == custom_action_status_s::OWNER_HANDOVER && handover_output_allowed))
	    && armed && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		publishControlSetpoint(now);
	}

	publishStatus();
}

int CustomActionControl::publishTestCommand(int action, int value, int request_id)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = custom_action_protocol::kMavCmdUser1;
	command.param1 = static_cast<float>(action);
	command.param2 = static_cast<float>(value);
	command.param3 = static_cast<float>(request_id);
	command.target_system = 1;
	command.target_component = custom_action_protocol::kComponentId;
	command.source_system = 42;
	command.source_component = 191;
	command.from_external = true;
	uORB::Publication<vehicle_command_s> publication{ORB_ID(vehicle_command)};
	publication.publish(command);
	PX4_WARN("TEST ONLY command action=%d value=%d request=%d", action, value, request_id);
	return PX4_OK;
}

int CustomActionControl::task_spawn(int argc, char *argv[])
{
	CustomActionControl *instance = new CustomActionControl();

	if (instance != nullptr) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int CustomActionControl::custom_command(int argc, char *argv[])
{
	if (argc >= 2 && !strcmp(argv[0], "test_command")) {
		if (!strcmp(argv[1], "search") && argc >= 3) {
			return publishTestCommand(static_cast<int>(Command::SearchTop), 0, atoi(argv[2]));
		}

		if (!strcmp(argv[1], "direction") && argc >= 4) {
			return publishTestCommand(static_cast<int>(Command::DirectionIntent), atoi(argv[2]), atoi(argv[3]));
		}

		if (!strcmp(argv[1], "rebase") && argc >= 4) {
			return publishTestCommand(static_cast<int>(Command::RebaseComplete), atoi(argv[2]), atoi(argv[3]));
		}

		return print_usage("test_command: search <request>, direction <id> <request>, or rebase <handover> <request>");
	}

	return print_usage("unknown command");
}

int CustomActionControl::print_status()
{
	PX4_INFO("state=%u owner=%u reason=%u handover_id=%u top_seq=%u mask=0x%02x",
		 _state, _owner, _reason, _handover_id, _top_distance.sequence, _top_distance.valid_mask);
	PX4_INFO("hold xyz=(%.2f, %.2f, %.2f) yaw=%.2f",
		 (double)_hold_x, (double)_hold_y, (double)_hold_z, (double)_locked_yaw);
	PX4_INFO("motor trim valid=%s source=%s mask=0x%03x gain=%.3f limited=%s first4=(%.3f %.3f %.3f %.3f)",
		 _motor_trim_valid ? "yes" : "no", motorTrimSourceName(_motor_trim_source), (unsigned)_motor_trim_mask,
		 (double)_active_pressure_gain, _motor_output_limited ? "yes" : "no",
		 (double)_motor_trim[0], (double)_motor_trim[1], (double)_motor_trim[2], (double)_motor_trim[3]);
	const hrt_abstime now = hrt_absolute_time();
	const float pressure_time_s = math::max(_param_press_time.get(), 0.2f);
	const float pressure_elapsed_s = _press_started != 0 && now >= _press_started
				       ? static_cast<float>(now - _press_started) * 1e-6f : 0.f;
	PX4_INFO("pressure target=%.3f applied=%.3f time=%.2f/%.2fs complete=%s limiting_motor=%u",
		 (double)math::max(_param_press_gain.get(), 0.f), (double)_active_pressure_gain,
		 (double)math::min(pressure_elapsed_s, pressure_time_s), (double)pressure_time_s,
		 _pressure_ramp_complete ? "yes" : "no", (unsigned)_limiting_motor);
	PX4_INFO("diagnostic trim candidates loose=%s %.2fs strict=%s %.2fs approach=%s %.2fs targets=(%.2f %.2f)s",
		 _search_loose_trim.valid ? "yes" : "no", (double)_search_loose_trim.valid_time_s,
		 _search_strict_trim.valid ? "yes" : "no", (double)_search_strict_trim.valid_time_s,
		 _approach_strict_trim.valid ? "yes" : "no", (double)_approach_strict_trim.valid_time_s,
		 (double)_search_trim_required_s, (double)_approach_trim_required_s);
	return 0;
}

int CustomActionControl::print_usage(const char *reason)
{
	PRINT_MODULE_USAGE_NAME("custom_action_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_command", "TEST ONLY: inject project command on vehicle_command");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int custom_action_control_main(int argc, char *argv[])
{
	return ModuleBase::main(CustomActionControl::desc, argc, argv);
}
