/****************************************************************************
 * CCFC fork — cc_safety_monitor (spec §4.5): the deterministic companion
 * safety policy core running on PX4.
 *
 * Consumes validated cc_health_report (published by the extended
 * mavlink_receiver after the Phase-3 gauntlet), runs the pure state machine
 * (cc_state_machine.hpp) + policy table (cc_policy_table.hpp), and:
 *   - publishes cc_safety_status (state echo + report ACK, streamed back to the
 *     companion as CC_SAFETY_STATUS — last_report_sequence stops the 5 Hz
 *     CRITICAL repeat),
 *   - on a state TRANSITION (edge-triggered, once per transition) issues at
 *     most a conservative Hold/Land/RTL / exit-Offboard vehicle_command,
 *   - honors pilot override (never re-commands until a new transition).
 *
 * All behaviour is parameterized (CC_MON_*) and re-read live. The module never
 * arms, takes off, or increases authority — see cc_policy_table.hpp.
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/cc_health_report.h>
#include <uORB/topics/cc_safety_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

#include "cc_policy_table.hpp"
#include "cc_state_machine.hpp"

using namespace time_literals;

class CcSafetyMonitor : public ModuleBase<CcSafetyMonitor>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	CcSafetyMonitor();
	~CcSafetyMonitor() override = default;

	static int task_spawn(int argc, char *argv[]);
	static CcSafetyMonitor *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void parameters_update();

	// Build the flight context the policy needs from the latest vehicle state.
	ccfc::FlightContext flight_context();

	// Issue the mode-change vehicle_command for a flight action.
	void command_mode(ccfc::MonitorAction action);

	// Publish the cc_safety_status echo/ack.
	void publish_status(ccfc::MonitorAction action_taken, uint8_t reject_reason);

	static constexpr uint32_t SCHEDULE_INTERVAL_US = 50_ms; // 20 Hz

	uORB::Subscription _health_report_sub{ORB_ID(cc_health_report)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<cc_safety_status_s> _safety_status_pub{ORB_ID(cc_safety_status)};
	uORB::Publication<vehicle_command_s>  _vehicle_command_pub{ORB_ID(vehicle_command)};

	ccfc::MonitorStateMachine _sm;
	ccfc::CompanionState _edge_state{ccfc::CompanionState::Unknown}; // last state we acted on
	bool _commanded_this_transition{false};

	// last accepted report bookkeeping (for the ACK + status echo)
	bool _have_report{false};
	uint32_t _last_seq{0};
	uint32_t _last_health_flags{0};
	uint32_t _missed_reports{0};
	uint64_t _last_report_companion_ts{0};
	uint8_t  _last_recommended{ccfc::ACTION_NONE};
	hrt_abstime _last_report_time{0};

	hrt_abstime _last_status_pub{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CC_MON_EN>)         _param_en,
		(ParamInt<px4::params::CC_MON_REQ_OFFB>)   _param_req_offb,
		(ParamInt<px4::params::CC_MON_TMOUT_MS>)   _param_timeout_ms,
		(ParamInt<px4::params::CC_MON_OK_COUNT>)   _param_ok_count,
		(ParamInt<px4::params::CC_MON_CRIT_ACT>)   _param_crit_act,
		(ParamInt<px4::params::CC_MON_STALE_ACT>)  _param_stale_act
	)
};
