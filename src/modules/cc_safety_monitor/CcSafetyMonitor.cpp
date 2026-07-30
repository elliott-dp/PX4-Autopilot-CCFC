/****************************************************************************
 * CCFC fork — cc_safety_monitor implementation (spec §4.5).
 ****************************************************************************/

#include "CcSafetyMonitor.hpp"

#include "../commander/px4_custom_mode.h"

using namespace ccfc;

// CC_REJECT_REASON values echoed in cc_safety_status.
static constexpr uint8_t CC_REJECT_NONE             = 0;
static constexpr uint8_t CC_REJECT_MONITOR_DISABLED = 6;

CcSafetyMonitor::CcSafetyMonitor() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool CcSafetyMonitor::init()
{
	ScheduleOnInterval(SCHEDULE_INTERVAL_US);
	return true;
}

void CcSafetyMonitor::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

FlightContext CcSafetyMonitor::flight_context()
{
	vehicle_status_s vs{};
	_vehicle_status_sub.copy(&vs);
	vehicle_control_mode_s cm{};
	_control_mode_sub.copy(&cm);
	vehicle_land_detected_s ld{};
	_land_detected_sub.copy(&ld);

	FlightContext c{};
	c.in_offboard = cm.flag_control_offboard_enabled;
	c.armed = (vs.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	// `landed` defaults false when unpublished — the SAFE default (a flying
	// vehicle is never mis-classified as grounded, so CRITICAL never downgrades
	// to a mere arming-block while airborne).
	c.airborne = !ld.landed;
	return c;
}

void CcSafetyMonitor::command_mode(MonitorAction action)
{
	uint8_t sub_mode;

	switch (action) {
	case MonitorAction::Land: sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;   break;

	case MonitorAction::Rtl:  sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;    break;

	case MonitorAction::Hold:
	default:                  sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER; break;
	}

	vehicle_status_s vs{};
	_vehicle_status_sub.copy(&vs);

	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1; // VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
	cmd.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
	cmd.param3 = sub_mode;
	cmd.target_system = vs.system_id != 0 ? vs.system_id : 1;
	cmd.target_component = 1;
	cmd.source_system = vs.system_id != 0 ? vs.system_id : 1;
	cmd.source_component = 191; // the monitor acts on the companion's behalf
	cmd.from_external = false;
	cmd.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(cmd);

	PX4_INFO("CCFC monitor: DO_SET_MODE AUTO sub %u (action %u)", sub_mode, (unsigned)action);
}

void CcSafetyMonitor::publish_status(MonitorAction action_taken, uint8_t reject_reason)
{
	const hrt_abstime now = hrt_absolute_time();

	cc_safety_status_s s{};
	s.timestamp = now;
	s.last_report_sequence = _last_seq;
	s.active_health_flags = _last_health_flags;
	s.report_age_ms = _have_report ? static_cast<uint32_t>((now - _last_report_time) / 1000) : 0;
	s.missed_reports = _missed_reports;
	s.companion_state = static_cast<uint8_t>(_sm.state());
	s.action_taken = static_cast<uint8_t>(action_taken);
	s.reject_reason = reject_reason;
	s.schema_version = 1;
	_safety_status_pub.publish(s);

	_last_status_pub = now;
}

void CcSafetyMonitor::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	const hrt_abstime now = hrt_absolute_time();

	// Monitor disabled: no state logic, no actions — just a 1 Hz echo saying so.
	if (_param_en.get() == 0) {
		if (now - _last_status_pub >= 1_s) {
			publish_status(MonitorAction::None, CC_REJECT_MONITOR_DISABLED);
		}

		return;
	}

	// --- ingest accepted reports (the receiver publishes only valid ones) ---
	const uint32_t ok_count = static_cast<uint32_t>(_param_ok_count.get());
	cc_health_report_s rep{};
	bool new_report = false;

	while (_health_report_sub.update(&rep)) {
		if (_have_report) {
			const int32_t diff = static_cast<int32_t>(rep.sequence - _last_seq);

			if (diff > 1) {
				_missed_reports += static_cast<uint32_t>(diff - 1);
			}
		}

		_sm.on_report(rep.severity, ok_count);
		_last_seq = rep.sequence;
		_last_health_flags = rep.health_flags;
		_last_recommended = rep.recommended_action;
		_last_report_companion_ts = rep.companion_timestamp_us;
		_last_report_time = now;
		_have_report = true;
		new_report = true;
	}

	// --- staleness (only meaningful once we have ever had a report) ---------
	const uint32_t timeout_us = static_cast<uint32_t>(_param_timeout_ms.get()) * 1000u;

	if (_have_report && _sm.state() != CompanionState::Stale
	    && (now - _last_report_time) > timeout_us) {
		_sm.on_timeout();
	}

	// --- decide ------------------------------------------------------------
	const CompanionState state = _sm.state();
	const FlightContext ctx = flight_context();
	const PolicyParams pp{ _param_req_offb.get() != 0, _param_crit_act.get(), _param_stale_act.get() };
	const MonitorAction action = decide_action(state, _last_recommended, ctx, pp);

	// --- edge-trigger: one action per state transition ---------------------
	const bool transition = (state != _edge_state);

	if (transition) {
		_edge_state = state;
		_commanded_this_transition = false;
	}

	if (transition && !_commanded_this_transition) {
		bool acted = true;

		if (is_flight_command(action)) {
			command_mode(action);                  // Hold / Land / RTL

		} else if (action == MonitorAction::BlockOffboard && ctx.in_offboard) {
			command_mode(MonitorAction::Hold);     // don't persist in Offboard: exit to Hold

		} else if (action == MonitorAction::Warn) {
			PX4_WARN("CCFC monitor: companion WARN (health_flags 0x%08x)", _last_health_flags);

		} else {
			acted = false; // None / passive arming-gate: nothing to command
		}

		if (acted) {
			_commanded_this_transition = true;
		}
	}

	// --- publish echo/ack: on transition, on a new report, or 1 Hz keepalive
	if (transition || new_report || (now - _last_status_pub >= 1_s)) {
		publish_status(action, CC_REJECT_NONE);
	}
}

// -------------------------------------------------------------------------
// ModuleBase boilerplate
// -------------------------------------------------------------------------

int CcSafetyMonitor::task_spawn(int argc, char *argv[])
{
	CcSafetyMonitor *instance = new CcSafetyMonitor();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

CcSafetyMonitor *CcSafetyMonitor::instantiate(int, char *[])
{
	return nullptr; // work-queue module: created in task_spawn
}

int CcSafetyMonitor::custom_command(int, char *[])
{
	return print_usage("unknown command");
}

int CcSafetyMonitor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
CCFC fork — cc_safety_monitor: the deterministic companion safety policy core.
Consumes validated cc_health_report, runs the companion state machine + policy
table, publishes cc_safety_status (state echo + report ACK), and issues at most
one conservative Hold/Land/RTL / exit-Offboard vehicle_command per state
transition. All behaviour is parameterized via CC_MON_*.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cc_safety_monitor", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int cc_safety_monitor_main(int argc, char *argv[])
{
	return CcSafetyMonitor::main(argc, argv);
}
