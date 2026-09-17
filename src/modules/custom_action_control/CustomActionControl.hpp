/****************************************************************************
 * SEARCH_TOP controller. It owns trajectory output only while status reports
 * CUSTOM or HANDOVER; INACTIVE never represents an additional normal hold.
 ****************************************************************************/

#pragma once

#include <cmath>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/custom_action_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/top_distance.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

class CustomActionControl : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CustomActionControl();
	~CustomActionControl() override;

	static Descriptor desc;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;
	static int publishTestCommand(int action, int value, int request_id);

private:
	enum class MotorTrimSource : uint8_t {
		None = 0,
		SearchLoose,
		SearchStrict,
		ApproachStrict
	};

	struct MotorTrimCandidate {
		float filtered[actuator_motors_s::NUM_CONTROLS] {};
		uint16_t mask{0};
		float valid_time_s{0.f};
		hrt_abstime last_update{0};
		hrt_abstime invalid_since{0};
		bool initialized{false};
		bool previous_sample_valid{false};
		bool valid{false};
	};

	void Run() override;
	void handleCommand(const vehicle_command_s &command);
	void startSearchTop(const vehicle_command_s &command, uint16_t request_id);
	void handleDirectionIntent(const vehicle_command_s &command, uint16_t request_id);
	void handleRebaseComplete(const vehicle_command_s &command, uint16_t request_id);
	void enterTopApproach();
	void enterContactVerify(hrt_abstime now);
	void enterContactPress(hrt_abstime now);
	void beginHandover(uint8_t reason, uint16_t handover_id, uint8_t target_system,
			   uint16_t target_component, bool notify_pending);
	void releaseToLegacy(uint8_t reason);
	void takeCommanderOwnership(uint8_t reason);
	bool captureHoldPoint();
	bool flightStateAllowsCustom() const;
	bool localStateValid() const;
	bool sensorFresh(hrt_abstime now) const;
	float minimumTopDistance() const;
	uint8_t topDistanceCountAtOrBelow(float threshold) const;
	void updateFilteredTopDistance();
	bool isPrecontactState() const;
	float verticalVelocityNed() const;
	float constrainedTrimTime(float available_time_s) const;
	void resetMotorTrim();
	void resetMotorTrimCandidate(MotorTrimCandidate &candidate);
	void updateMotorTrim(hrt_abstime now, bool actuator_updated);
	void updateMotorTrimCandidate(MotorTrimCandidate &candidate, hrt_abstime now, bool loose,
				      float required_time_s, const char *name);
	bool motorTrimSampleValid(hrt_abstime now, bool loose, uint16_t &sample_mask) const;
	void selectBestMotorTrim();
	const char *motorTrimSourceName(MotorTrimSource source) const;
	bool directActuatorReady(hrt_abstime now) const;
	bool preparePressHandover(hrt_abstime now);
	void publishDirectMotorSetpoint(hrt_abstime now);
	void publishControlSetpoint(hrt_abstime now);
	void publishStatus(bool force = false);
	void publishAck(const vehicle_command_s &command, uint8_t result, uint8_t project_result);
	void publishAsyncAck(uint8_t result, uint8_t project_result, uint16_t request_id,
			     uint8_t target_system, uint16_t target_component);

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _top_distance_sub{ORB_ID(top_distance)};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<custom_action_status_s> _status_pub{ORB_ID(custom_action_status)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

	vehicle_local_position_s _local_position{};
	vehicle_status_s _vehicle_status{};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	actuator_motors_s _allocated_motors{};
	top_distance_s _top_distance{};

	uint8_t _state{custom_action_status_s::STATE_INACTIVE};
	uint8_t _owner{custom_action_status_s::OWNER_LEGACY};
	uint8_t _reason{custom_action_status_s::REASON_NONE};
	uint16_t _handover_id{0};
	uint16_t _active_request_id{0};
	uint8_t _active_source_system{0};
	uint16_t _active_source_component{0};

	float _hold_x{0.f};
	float _hold_y{0.f};
	float _hold_z{0.f};
	float _locked_yaw{0.f};
	float _start_z{0.f};
	float _filtered_top_distance{NAN};
	float _verify_min_distance{NAN};
	float _verify_max_distance{NAN};
	float _motor_trim[actuator_motors_s::NUM_CONTROLS] {};
	float _press_entry_output[actuator_motors_s::NUM_CONTROLS] {};
	float _press_base_output[actuator_motors_s::NUM_CONTROLS] {};
	MotorTrimCandidate _search_loose_trim{};
	MotorTrimCandidate _search_strict_trim{};
	MotorTrimCandidate _approach_strict_trim{};
	MotorTrimSource _motor_trim_source{MotorTrimSource::None};
	uint16_t _motor_trim_mask{0};
	uint16_t _press_output_mask{0};
	bool _motor_trim_valid{false};
	bool _motor_output_limited{false};
	bool _pressure_ramp_complete{false};
	float _active_pressure_gain{0.f};
	uint8_t _limiting_motor{0};
	float _search_trim_required_s{0.f};
	float _approach_trim_required_s{0.f};
	uint8_t _contact_threshold_frames{0};
	uint8_t _heading_reset_counter{0};
	hrt_abstime _search_started{0};
	uint16_t _last_top_distance_sequence{0};
	hrt_abstime _verify_started{0};
	hrt_abstime _press_started{0};
	hrt_abstime _press_output_started{0};
	hrt_abstime _pressure_ramp_started{0};
	hrt_abstime _handover_started{0};
	hrt_abstime _last_status_publish{0};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::CUST_TOP_EN>) _param_enabled,
		(ParamBool<px4::params::CUST_CLOSE_EN>) _param_close_start_enabled,
		(ParamFloat<px4::params::CUST_TOP_VEL>) _param_top_velocity,
		(ParamFloat<px4::params::CUST_TOP_DIST>) _param_top_distance,
		(ParamFloat<px4::params::CUST_TOP_TIME>) _param_top_time,
		(ParamFloat<px4::params::CUST_TOP_GAP>) _param_top_gap,
		(ParamFloat<px4::params::CUST_TOP_FILT>) _param_top_filter,
		(ParamFloat<px4::params::CUST_TOP_HYST>) _param_top_hysteresis,
		(ParamFloat<px4::params::CUST_APP_VEL>) _param_approach_velocity,
		(ParamFloat<px4::params::CUST_VER_VEL>) _param_verify_velocity,
		(ParamFloat<px4::params::CUST_CNT_DIST>) _param_contact_distance,
		(ParamFloat<px4::params::CUST_CNT_TIME>) _param_contact_time,
		(ParamFloat<px4::params::CUST_STAB_BND>) _param_stability_band,
		(ParamFloat<px4::params::CUST_TRIM_RATIO>) _param_trim_ratio,
		(ParamFloat<px4::params::CUST_TRIM_TMIN>) _param_trim_time_min,
		(ParamFloat<px4::params::CUST_TRIM_TMAX>) _param_trim_time_max,
		(ParamFloat<px4::params::CUST_TRIM_ANG>) _param_trim_angle,
		(ParamFloat<px4::params::CUST_TRIM_RATE>) _param_trim_rate,
		(ParamFloat<px4::params::CUST_PRS_GAIN>) _param_press_gain,
		(ParamFloat<px4::params::CUST_PRS_TIME>) _param_press_time,
		(ParamFloat<px4::params::CUST_MOT_LIM>) _param_motor_limit,
		(ParamFloat<px4::params::CUST_SENS_TO>) _param_sensor_timeout,
		(ParamFloat<px4::params::CUST_HO_TIME>) _param_handover_timeout
	)
};
