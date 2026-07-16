/****************************************************************************
 *
 * CCFC fork — cc_telemetry_publisher (FC<->CC architecture spec §4.2)
 *
 * ONE JOB: deterministic field selection + rate control from existing uORB
 * topics into the compact cc_telemetry_* uORB topics consumed by the
 * MAVLink CC_* streams (Phase 3). Nothing else.
 *
 * Rules enforced here (spec §4.2, invariants §0):
 *  - runs on the lp_default work queue; fixed base tick (50 Hz, or 200 Hz
 *    in the AI_ETH/DEBUG profiles); per-stream integer decimation
 *  - on each due tick, copy() the NEWEST sample of each input topic (uORB
 *    latest-value semantics); never queue history; never block
 *  - publishes whether or not anyone listens; never conditions behavior on
 *    companion/link state
 *  - no heap allocation after init, no file I/O, no string formatting in
 *    the tick path
 *  - missing/stale sources are marked (NaN floats, validity fields), never
 *    fabricated and never silently frozen (invariant 7)
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cc_telemetry_actuator.h>
#include <uORB/topics/cc_telemetry_estimator.h>
#include <uORB/topics/cc_telemetry_gps.h>
#include <uORB/topics/cc_telemetry_imu.h>
#include <uORB/topics/cc_telemetry_power.h>
#include <uORB/topics/cc_telemetry_state.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class CcTelemetryPublisher : public ModuleBase<CcTelemetryPublisher>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	CcTelemetryPublisher();
	~CcTelemetryPublisher() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	void Start();

private:
	void Run() override;

	void configure_rates();

	void publish_state(const hrt_abstime now);
	void publish_imu(const hrt_abstime now);
	void publish_power(const hrt_abstime now);
	void publish_gps(const hrt_abstime now);
	void publish_estimator(const hrt_abstime now);
	void publish_actuator(const hrt_abstime now);

	// schema version produced by this build (bumped on field-semantics
	// change of any CC_* payload — must match the companion's expectation)
	static constexpr uint8_t CC_SCHEMA_VERSION = 1;

	// base tick rates (spec §4.2): 50 Hz for MINIMAL/AI_UART, 200 Hz for
	// AI_ETH/DEBUG (the only profiles allowed to exceed 50 Hz streams)
	static constexpr int TICK_HZ_UART = 50;
	static constexpr int TICK_HZ_ETH  = 200;

	// source-freshness windows: a source older than this is reported as
	// missing (NaN/invalid markers), never re-stamped as fresh (invariant 7)
	static constexpr hrt_abstime FRESH_FAST_US   = 200_ms;  // imu, attitude, actuator
	static constexpr hrt_abstime FRESH_MEDIUM_US = 1_s;     // battery, estimator, local pos
	static constexpr hrt_abstime FRESH_SLOW_US   = 2_s;     // gps, status, control mode, failsafe

	// CC_TEL_PROFILE values (mirror of CC_LOG_PROFILE in cc_dialect.xml)
	enum class Profile : int32_t {
		MINIMAL = 0,
		AI_UART = 1,
		AI_ETH  = 2,
		DEBUG   = 3,
	};

	struct StreamState {
		uint32_t divider{0};        // 0 = stream disabled
		uint32_t sequence{0};       // per-stream monotonic counter (wraps)
		uint64_t publish_count{0};
		hrt_abstime last_publish{0};

		bool due(const uint64_t tick) const { return divider > 0 && (tick % divider) == 0; }
	};

	// ---- outputs -----------------------------------------------------
	uORB::Publication<cc_telemetry_state_s>     _state_pub{ORB_ID(cc_telemetry_state)};
	uORB::Publication<cc_telemetry_imu_s>       _imu_pub{ORB_ID(cc_telemetry_imu)};
	uORB::Publication<cc_telemetry_power_s>     _power_pub{ORB_ID(cc_telemetry_power)};
	uORB::Publication<cc_telemetry_gps_s>       _gps_pub{ORB_ID(cc_telemetry_gps)};
	uORB::Publication<cc_telemetry_estimator_s> _estimator_pub{ORB_ID(cc_telemetry_estimator)};
	uORB::Publication<cc_telemetry_actuator_s>  _actuator_pub{ORB_ID(cc_telemetry_actuator)};

	// ---- inputs ------------------------------------------------------
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};

	uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};
	// instance 0: SIH/SITL run a single IMU; multi-IMU primary selection is
	// a Phase 8 (bench) refinement — documented deviation D5
	uORB::Subscription _vehicle_imu_sub{ORB_ID(vehicle_imu), 0};
	uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status), 0};

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status), 0};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps), 0};

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status), 0};
	uORB::Subscription _estimator_status_flags_sub{ORB_ID(estimator_status_flags), 0};
	uint8_t _estimator_instance{0};

	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};

	// ---- state -------------------------------------------------------
	StreamState _state_stream{};
	StreamState _imu_stream{};
	StreamState _power_stream{};
	StreamState _gps_stream{};
	StreamState _estimator_stream{};
	StreamState _actuator_stream{};

	uint64_t _tick{0};
	int _tick_hz{TICK_HZ_UART};
	uint32_t _px4_boot_id{0};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CC_TEL_PROFILE>)  _param_cc_tel_profile,
		(ParamInt<px4::params::CC_TEL_IMU_RATE>) _param_cc_tel_imu_rate,
		(ParamInt<px4::params::CC_TEL_ACT_RATE>) _param_cc_tel_act_rate
	)
};
