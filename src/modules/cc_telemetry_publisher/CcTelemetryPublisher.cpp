/****************************************************************************
 *
 * CCFC fork — cc_telemetry_publisher implementation.
 * See CcTelemetryPublisher.hpp for the module contract, and
 * drone_companion/docs/phase2_px4_telemetry.md for mapping rationale.
 *
 ****************************************************************************/

#include "CcTelemetryPublisher.hpp"

#include <math.h>
#include <time.h>

CcTelemetryPublisher::CcTelemetryPublisher() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

CcTelemetryPublisher::~CcTelemetryPublisher()
{
	perf_free(_loop_perf);
	perf_free(_interval_perf);
}

void CcTelemetryPublisher::Start()
{
	// px4_boot_id: minted once per FC boot (spec §4.2). Wall clock (RTC on
	// the V6X, host clock in SITL) XOR the microsecond monotonic clock —
	// uniqueness across boots is the goal, not cryptographic strength.
	_px4_boot_id = static_cast<uint32_t>(time(nullptr)) ^ static_cast<uint32_t>(hrt_absolute_time());

	if (_px4_boot_id == 0) {
		_px4_boot_id = 1; // 0 is reserved as "invalid/unset"
	}

	updateParams();
	configure_rates();
}

void CcTelemetryPublisher::configure_rates()
{
	const Profile profile = static_cast<Profile>(_param_cc_tel_profile.get());

	// per-profile stream rates (Hz) — the table in spec §6 / phase2 doc A.3
	int state_hz{0}, imu_hz{0}, power_hz{0}, gps_hz{0}, est_hz{0}, act_hz{0};

	switch (profile) {
	case Profile::MINIMAL:
		// "heartbeat + safety only": all telemetry streams off
		_tick_hz = TICK_HZ_UART;
		break;

	case Profile::AI_UART:
	default:
		_tick_hz = TICK_HZ_UART;
		state_hz = 25;
		imu_hz   = math::constrain(_param_cc_tel_imu_rate.get(), 0, TICK_HZ_UART);
		power_hz = 10;
		gps_hz   = 5;
		est_hz   = 10;
		act_hz   = math::constrain(_param_cc_tel_act_rate.get(), 0, TICK_HZ_UART);
		break;

	case Profile::AI_ETH:
	case Profile::DEBUG:
		_tick_hz = TICK_HZ_ETH;
		state_hz = 50;
		imu_hz   = math::constrain(_param_cc_tel_imu_rate.get(), 0, 200);
		power_hz = 20;
		gps_hz   = 10;
		est_hz   = 20;
		act_hz   = math::constrain(_param_cc_tel_act_rate.get(), 0, 50);
		break;
	}

	// nearest-actual-rate divider: requested rates that do not divide the
	// tick (e.g. 20 Hz on a 50 Hz tick) pick the divider whose resulting
	// rate is closest (50/20 -> divider 3 -> 16.7 Hz, not 25 Hz)
	const auto divider = [this](int rate_hz) -> uint32_t {
		if (rate_hz <= 0) { return 0; }

		const int d_floor = math::max(1, _tick_hz / rate_hz);
		const int d_ceil = d_floor + 1;
		const float err_floor = fabsf(static_cast<float>(_tick_hz) / static_cast<float>(d_floor)
					      - static_cast<float>(rate_hz));
		const float err_ceil = fabsf(static_cast<float>(_tick_hz) / static_cast<float>(d_ceil)
					     - static_cast<float>(rate_hz));
		return static_cast<uint32_t>((err_ceil < err_floor) ? d_ceil : d_floor);
	};

	_state_stream.divider     = divider(state_hz);
	_imu_stream.divider       = divider(imu_hz);
	_power_stream.divider     = divider(power_hz);
	_gps_stream.divider       = divider(gps_hz);
	_estimator_stream.divider = divider(est_hz);
	_actuator_stream.divider  = divider(act_hz);

	ScheduleClear();
	ScheduleOnInterval(1_s / _tick_hz);
}

void CcTelemetryPublisher::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_interval_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
		configure_rates(); // live rate changes, no reboot (spec §12)
	}

	_tick++;
	const hrt_abstime now = hrt_absolute_time();

	if (_state_stream.due(_tick))     { publish_state(now); }

	if (_imu_stream.due(_tick))       { publish_imu(now); }

	if (_power_stream.due(_tick))     { publish_power(now); }

	if (_gps_stream.due(_tick))       { publish_gps(now); }

	if (_estimator_stream.due(_tick)) { publish_estimator(now); }

	if (_actuator_stream.due(_tick))  { publish_actuator(now); }

	perf_end(_loop_perf);
}

// fresh = source published within `window` of `now` (0 timestamp = never)
static inline bool source_fresh(const hrt_abstime now, const hrt_abstime stamp, const hrt_abstime window)
{
	return stamp != 0 && now - stamp < window;
}

void CcTelemetryPublisher::publish_state(const hrt_abstime now)
{
	vehicle_status_s status{};
	vehicle_attitude_s att{};
	vehicle_angular_velocity_s rates{};
	vehicle_local_position_s lpos{};
	vehicle_control_mode_s cmode{};
	failsafe_flags_s fsafe{};

	_vehicle_status_sub.copy(&status);
	_vehicle_control_mode_sub.copy(&cmode);
	_failsafe_flags_sub.copy(&fsafe);

	const bool att_ok   = _vehicle_attitude_sub.copy(&att)
			      && source_fresh(now, att.timestamp, FRESH_FAST_US);
	const bool rates_ok = _vehicle_angular_velocity_sub.copy(&rates)
			      && source_fresh(now, rates.timestamp, FRESH_FAST_US);
	const bool lpos_ok  = _vehicle_local_position_sub.copy(&lpos)
			      && source_fresh(now, lpos.timestamp, FRESH_MEDIUM_US);

	cc_telemetry_state_s out{};
	out.timestamp = now;
	out.sequence = _state_stream.sequence++;
	out.px4_boot_id = _px4_boot_id;
	out.mission_id = 0; // provided by CC_MISSION_CONTEXT from Phase 3 on

	// attitude / rates / local position: NaN marks a missing source
	// (invariant 7) while the stream keeps its rate
	for (int i = 0; i < 4; i++) { out.q[i] = att_ok ? att.q[i] : NAN; }

	for (int i = 0; i < 3; i++) { out.angular_velocity[i] = rates_ok ? rates.xyz[i] : NAN; }

	const bool xy_ok = lpos_ok && lpos.xy_valid;
	const bool z_ok  = lpos_ok && lpos.z_valid;
	const bool vxy_ok = lpos_ok && lpos.v_xy_valid;
	const bool vz_ok  = lpos_ok && lpos.v_z_valid;

	out.position_ned[0] = xy_ok ? lpos.x : NAN;
	out.position_ned[1] = xy_ok ? lpos.y : NAN;
	out.position_ned[2] = z_ok  ? lpos.z : NAN;
	out.velocity_ned[0] = vxy_ok ? lpos.vx : NAN;
	out.velocity_ned[1] = vxy_ok ? lpos.vy : NAN;
	out.velocity_ned[2] = vz_ok  ? lpos.vz : NAN;
	out.heading = lpos_ok ? lpos.heading : NAN;

	out.nav_state = status.nav_state;
	out.arming_state = status.arming_state;
	out.vehicle_type = status.system_type; // MAV_TYPE, same semantics as HEARTBEAT
	out.estimator_valid = (att_ok && xy_ok && z_ok) ? 1 : 0;

	// bit tables documented in CcTelemetryState.msg
	uint8_t cm = 0;

	if (source_fresh(now, cmode.timestamp, FRESH_SLOW_US)) {
		cm |= cmode.flag_armed ? 0x01 : 0;
		cm |= cmode.flag_control_manual_enabled ? 0x02 : 0;
		cm |= cmode.flag_control_auto_enabled ? 0x04 : 0;
		cm |= cmode.flag_control_offboard_enabled ? 0x08 : 0;
		cm |= cmode.flag_control_position_enabled ? 0x10 : 0;
		cm |= cmode.flag_control_velocity_enabled ? 0x20 : 0;
		cm |= cmode.flag_control_altitude_enabled ? 0x40 : 0;
		cm |= cmode.flag_control_termination_enabled ? 0x80 : 0;
	}

	out.control_mode_flags = cm;

	uint32_t ff = 0;

	if (source_fresh(now, fsafe.timestamp, FRESH_SLOW_US)) {
		ff |= fsafe.manual_control_signal_lost ? 0x0001 : 0;
		ff |= fsafe.gcs_connection_lost ? 0x0002 : 0;
		ff |= fsafe.battery_low_remaining_time ? 0x0004 : 0;
		ff |= fsafe.battery_unhealthy ? 0x0008 : 0;
		ff |= fsafe.geofence_breached ? 0x0010 : 0;
		ff |= fsafe.mission_failure ? 0x0020 : 0;
		ff |= fsafe.wind_limit_exceeded ? 0x0040 : 0;
		ff |= fsafe.flight_time_limit_exceeded ? 0x0080 : 0;
		ff |= fsafe.position_accuracy_low ? 0x0100 : 0;
		ff |= fsafe.navigator_failure ? 0x0200 : 0;
		ff |= fsafe.offboard_control_signal_lost ? 0x0400 : 0;
		ff |= fsafe.fd_critical_failure ? 0x0800 : 0;
		ff |= fsafe.fd_esc_arming_failure ? 0x1000 : 0;
		ff |= fsafe.fd_imbalanced_prop ? 0x2000 : 0;
		ff |= fsafe.fd_motor_failure ? 0x4000 : 0;
		ff |= fsafe.local_position_invalid ? 0x8000 : 0;
	}

	out.failsafe_flags = ff;
	out.schema_version = CC_SCHEMA_VERSION;

	_state_pub.publish(out);
	_state_stream.publish_count++;
	_state_stream.last_publish = now;
}

void CcTelemetryPublisher::publish_imu(const hrt_abstime now)
{
	sensor_combined_s sc{};
	vehicle_imu_s imu{};
	vehicle_imu_status_s imu_status{};

	const bool sc_ok  = _sensor_combined_sub.copy(&sc)
			    && source_fresh(now, sc.timestamp, FRESH_FAST_US);
	const bool imu_ok = _vehicle_imu_sub.copy(&imu)
			    && source_fresh(now, imu.timestamp, FRESH_FAST_US);
	const bool ist_ok = _vehicle_imu_status_sub.copy(&imu_status)
			    && source_fresh(now, imu_status.timestamp, FRESH_MEDIUM_US);

	cc_telemetry_imu_s out{};
	out.timestamp = now;
	out.sequence = _imu_stream.sequence++;

	for (int i = 0; i < 3; i++) {
		out.accel[i] = sc_ok ? sc.accelerometer_m_s2[i] : NAN;
		out.gyro[i]  = sc_ok ? sc.gyro_rad[i] : NAN;
		out.delta_angle[i]    = imu_ok ? imu.delta_angle[i] : NAN;
		out.delta_velocity[i] = imu_ok ? imu.delta_velocity[i] : NAN;
	}

	// slot mapping documented in CcTelemetryImu.msg (deviation D2):
	// PX4 v1.17 exposes scalar vibration metrics, not per-axis
	out.vibration_metric[0] = ist_ok ? imu_status.accel_vibration_metric : NAN;
	out.vibration_metric[1] = ist_ok ? imu_status.gyro_vibration_metric : NAN;
	out.vibration_metric[2] = ist_ok ? imu_status.delta_angle_coning_metric : NAN;

	out.clipping_count = ist_ok ? (imu_status.accel_clipping[0] + imu_status.accel_clipping[1]
				       + imu_status.accel_clipping[2]) : 0;
	out.temperature = ist_ok ? imu_status.temperature_accel : NAN;
	out.schema_version = CC_SCHEMA_VERSION;

	_imu_pub.publish(out);
	_imu_stream.publish_count++;
	_imu_stream.last_publish = now;
}

void CcTelemetryPublisher::publish_power(const hrt_abstime now)
{
	battery_status_s batt{};
	const bool ok = _battery_status_sub.copy(&batt)
			&& source_fresh(now, batt.timestamp, FRESH_MEDIUM_US)
			&& batt.connected;

	cc_telemetry_power_s out{};
	out.timestamp = now;
	out.sequence = _power_stream.sequence++;

	// battery_status marks invalid fields with 0 / -1 / NaN (see the msg
	// @invalid annotations); everything invalid maps to the dialect's NaN
	const float voltage = (ok && batt.voltage_v > 0.f) ? batt.voltage_v : NAN;
	const float current = (ok && batt.current_a >= 0.f) ? batt.current_a : NAN;

	out.voltage = voltage;
	out.current = current;
	out.power = (PX4_ISFINITE(voltage) && PX4_ISFINITE(current)) ? voltage * current : NAN;
	out.consumed_mah = (ok && batt.discharged_mah >= 0.f) ? batt.discharged_mah : NAN;
	out.remaining = (ok && batt.remaining >= 0.f) ? batt.remaining : NAN;
	out.temperature = ok ? batt.temperature : NAN; // already NaN when unknown
	out.cell_count = ok ? batt.cell_count : 0;
	out.warning = ok ? batt.warning : 0;
	out.connected = ok ? 1 : 0;
	out.schema_version = CC_SCHEMA_VERSION;

	_power_pub.publish(out);
	_power_stream.publish_count++;
	_power_stream.last_publish = now;
}

void CcTelemetryPublisher::publish_gps(const hrt_abstime now)
{
	sensor_gps_s gps{};
	const bool ok = _sensor_gps_sub.copy(&gps)
			&& source_fresh(now, gps.timestamp, FRESH_SLOW_US);

	cc_telemetry_gps_s out{};
	out.timestamp = now;
	out.sequence = _gps_stream.sequence++;

	if (ok) {
		// v1.17 sensor_gps carries double degrees; the dialect carries
		// int32 degE7 (and mm AMSL) — cannot NaN an int: fix_type == 0
		// is the invalidity marker for the integer fields
		out.lat = static_cast<int32_t>(round(gps.latitude_deg * 1e7));
		out.lon = static_cast<int32_t>(round(gps.longitude_deg * 1e7));
		out.alt = static_cast<int32_t>(round(gps.altitude_msl_m * 1e3));
		out.eph = gps.eph;
		out.epv = gps.epv;
		out.ground_speed = gps.vel_m_s;
		out.heading = gps.cog_rad; // NaN when unknown, per source contract
		out.noise_per_ms = static_cast<uint16_t>(math::constrain(gps.noise_per_ms, 0, 65535));
		out.jamming_indicator = static_cast<uint16_t>(math::constrain(gps.jamming_indicator, 0, 65535));
		out.fix_type = gps.fix_type;
		out.satellites_used = gps.satellites_used;

	} else {
		out.eph = NAN;
		out.epv = NAN;
		out.ground_speed = NAN;
		out.heading = NAN;
		// lat/lon/alt stay 0 with fix_type = 0 (no fix -> no position claim)
	}

	out.schema_version = CC_SCHEMA_VERSION;

	_gps_pub.publish(out);
	_gps_stream.publish_count++;
	_gps_stream.last_publish = now;
}

void CcTelemetryPublisher::publish_estimator(const hrt_abstime now)
{
	// follow the primary EKF instance (multi-EKF: V6X runs several)
	estimator_selector_status_s sel{};

	if (_estimator_selector_status_sub.copy(&sel) && sel.primary_instance != _estimator_instance) {
		if (_estimator_status_sub.ChangeInstance(sel.primary_instance)
		    && _estimator_status_flags_sub.ChangeInstance(sel.primary_instance)) {
			_estimator_instance = sel.primary_instance;

		} else {
			// keep the old instance rather than half-switching
			_estimator_status_sub.ChangeInstance(_estimator_instance);
			_estimator_status_flags_sub.ChangeInstance(_estimator_instance);
		}
	}

	estimator_status_s est{};
	estimator_status_flags_s flags{};

	const bool est_ok = _estimator_status_sub.copy(&est)
			    && source_fresh(now, est.timestamp, FRESH_MEDIUM_US);
	const bool flags_ok = _estimator_status_flags_sub.copy(&flags)
			      && source_fresh(now, flags.timestamp, FRESH_MEDIUM_US);

	cc_telemetry_estimator_s out{};
	out.timestamp = now;
	out.sequence = _estimator_stream.sequence++;

	// flags carry last-known values, ratios go NaN on staleness — the
	// consumer distinguishes "stale" via the ratios (deviation D6)
	out.status_flags = est.filter_fault_flags;
	out.solution_status_flags = est.solution_status_flags;

	uint16_t inno = 0;

	if (flags_ok) {
		inno |= flags.reject_hor_vel    ? 0x0001 : 0;
		inno |= flags.reject_ver_vel    ? 0x0002 : 0;
		inno |= flags.reject_hor_pos    ? 0x0004 : 0;
		inno |= flags.reject_ver_pos    ? 0x0008 : 0;
		inno |= flags.reject_yaw        ? 0x0010 : 0;
		inno |= flags.reject_airspeed   ? 0x0020 : 0;
		inno |= flags.reject_sideslip   ? 0x0040 : 0;
		inno |= flags.reject_hagl       ? 0x0080 : 0;
		inno |= flags.reject_optflow_x  ? 0x0100 : 0;
		inno |= flags.reject_optflow_y  ? 0x0200 : 0;
	}

	out.innovation_check_flags = inno;

	out.velocity_test_ratio = est_ok ? est.vel_test_ratio : NAN;
	out.position_test_ratio = est_ok ? est.pos_test_ratio : NAN;
	out.height_test_ratio   = est_ok ? est.hgt_test_ratio : NAN;
	out.mag_test_ratio      = est_ok ? est.hdg_test_ratio : NAN;
	out.airspeed_test_ratio = est_ok ? est.tas_test_ratio : NAN;
	out.schema_version = CC_SCHEMA_VERSION;

	_estimator_pub.publish(out);
	_estimator_stream.publish_count++;
	_estimator_stream.last_publish = now;
}

void CcTelemetryPublisher::publish_actuator(const hrt_abstime now)
{
	actuator_motors_s motors{};
	const bool ok = _actuator_motors_sub.copy(&motors)
			&& source_fresh(now, motors.timestamp, FRESH_FAST_US);

	cc_telemetry_actuator_s out{};
	out.timestamp = now;
	out.sequence = _actuator_stream.sequence++;

	uint8_t count = 0;

	for (int i = 0; i < 8; i++) {
		const float v = ok ? motors.control[i] : NAN;
		out.actuator_output[i] = v;

		if (count == i && PX4_ISFINITE(v)) {
			count++; // leading finite entries define motor_count
		}
	}

	out.motor_count = count;
	out.schema_version = CC_SCHEMA_VERSION;

	_actuator_pub.publish(out);
	_actuator_stream.publish_count++;
	_actuator_stream.last_publish = now;
}

int CcTelemetryPublisher::print_status()
{
	PX4_INFO("profile: %" PRId32 ", tick: %d Hz, px4_boot_id: %" PRIu32,
		 _param_cc_tel_profile.get(), _tick_hz, _px4_boot_id);

	const struct {
		const char *name;
		const StreamState *s;
	} rows[] = {
		{"state",     &_state_stream},
		{"imu",       &_imu_stream},
		{"power",     &_power_stream},
		{"gps",       &_gps_stream},
		{"estimator", &_estimator_stream},
		{"actuator",  &_actuator_stream},
	};

	for (const auto &row : rows) {
		const float rate = (row.s->divider > 0) ? static_cast<float>(_tick_hz) / static_cast<float>(row.s->divider) : 0.f;
		PX4_INFO("  %-9s rate: %5.1f Hz  divider: %2" PRIu32 "  seq: %10" PRIu32 "  published: %" PRIu64,
			 row.name, static_cast<double>(rate), row.s->divider, row.s->sequence, row.s->publish_count);
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_interval_perf);
	return 0;
}

int CcTelemetryPublisher::task_spawn(int argc, char *argv[])
{
	CcTelemetryPublisher *instance = new CcTelemetryPublisher();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	// the single permitted allocation: module construction at init
	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->Start();

	return PX4_OK;
}

int CcTelemetryPublisher::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CcTelemetryPublisher::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
CCFC companion-telemetry curator (FC<->CC architecture spec §4.2).

Deterministically samples existing uORB topics on a fixed work-queue tick
(50 Hz, or 200 Hz in the AI_ETH/DEBUG profiles) and publishes the compact
`cc_telemetry_*` topics consumed by the MAVLink CC_* streams toward the
companion computer. Rates follow `CC_TEL_PROFILE` / `CC_TEL_IMU_RATE` /
`CC_TEL_ACT_RATE` and change live on parameter update.

Missing or stale sources are marked (NaN fields, validity flags), never
fabricated; streams keep publishing at their configured rate regardless of
listeners or companion state.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cc_telemetry_publisher", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cc_telemetry_publisher_main(int argc, char *argv[])
{
	return CcTelemetryPublisher::main(argc, argv);
}
