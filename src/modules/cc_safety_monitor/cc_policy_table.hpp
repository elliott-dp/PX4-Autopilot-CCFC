/****************************************************************************
 * CCFC fork — cc_safety_monitor deterministic policy table (spec §4.5).
 *
 * A PURE function, free of PX4 dependencies, so it is exhaustively unit-tested
 * on the host (`cc_policy_table_test.cpp`, one case per table row — the phase
 * exit criterion). The module (`CcSafetyMonitor`) owns the state machine,
 * clocks, uORB and the edge-triggering; this header owns ONLY the mapping
 *
 *     (companion_state, recommended_action, flight_context, params) -> action
 *
 * and encodes the two hard invariants that make it safe:
 *   1. The monitor may only ever move the vehicle toward a MORE conservative
 *      state (Warn < BlockOffboard < Hold < Land/RTL). It has no output that
 *      arms, takes off, enters Offboard, or increases authority.
 *   2. A CRITICAL parameter action (CC_MON_CRIT_ACT) always wins over the
 *      companion's recommendation; the recommendation is advisory and logged.
 ****************************************************************************/

#pragma once

#include <cstdint>

namespace ccfc
{

// Mirrors CC_COMPANION_STATE in cc_dialect.xml.
enum class CompanionState : uint8_t {
	Unknown  = 0,
	Ok       = 1,
	Warn     = 2,
	Critical = 3,
	Stale    = 4,
};

// The action the monitor executes. Values are DELIBERATELY equal to
// CC_RECOMMENDED_ACTION so `action_taken` in CC_SAFETY_STATUS is a direct cast
// (spec §4.5: action_taken is a CC_RECOMMENDED_ACTION enum).
enum class MonitorAction : uint8_t {
	None          = 0, // CC_ACTION_NONE
	Warn          = 1, // CC_ACTION_WARN_ONLY   (events + STATUSTEXT only)
	BlockOffboard = 2, // CC_ACTION_BLOCK_OFFBOARD (arming gate, or exit if in Offboard)
	Hold          = 3, // CC_ACTION_HOLD
	Land          = 4, // CC_ACTION_LAND
	Rtl           = 5, // CC_ACTION_RTL
};

// CC_RECOMMENDED_ACTION values (companion advisory input).
static constexpr uint8_t ACTION_NONE           = 0;
static constexpr uint8_t ACTION_WARN_ONLY      = 1;
static constexpr uint8_t ACTION_BLOCK_OFFBOARD = 2;
static constexpr uint8_t ACTION_HOLD           = 3;
static constexpr uint8_t ACTION_LAND           = 4;
static constexpr uint8_t ACTION_RTL            = 5;

// The bit of vehicle state the policy needs. Supplied by the module from
// vehicle_status / vehicle_control_mode.
struct FlightContext {
	bool in_offboard; // vehicle_control_mode.flag_control_offboard_enabled
	bool armed;       // vehicle_status.arming_state == ARMED
	bool airborne;    // !vehicle_land_detected.landed (i.e. actually flying)

	constexpr bool armed_airborne() const { return armed && airborne; }
};

// The subset of §12 parameters the policy reads. `crit_act` / `stale_act` use
// the CC_MON_*_ACT encoding: 0 warn, 1 hold, 2 land, 3 RTL.
struct PolicyParams {
	bool    require_offboard; // CC_MON_REQ_OFFB
	int32_t crit_act;         // CC_MON_CRIT_ACT
	int32_t stale_act;        // CC_MON_STALE_ACT
};

// Map a CC_MON_*_ACT parameter value to a conservative MonitorAction.
// Any out-of-range value fails safe to Hold (never None) — a misconfigured
// parameter must not silently disable the protective action.
constexpr MonitorAction act_param_to_action(int32_t v)
{
	switch (v) {
	case 0:  return MonitorAction::Warn;

	case 1:  return MonitorAction::Hold;

	case 2:  return MonitorAction::Land;

	case 3:  return MonitorAction::Rtl;

	default: return MonitorAction::Hold;
	}
}

// The deterministic policy (spec §4.5 table). Pure: no clocks, no I/O, no
// hidden state. `state` is the ALREADY-hysteresed companion state the module's
// state machine produced; `recommended_action` is the latest valid report's
// advisory.
constexpr MonitorAction decide_action(CompanionState state,
				      uint8_t recommended_action,
				      FlightContext ctx,
				      PolicyParams params)
{
	switch (state) {
	case CompanionState::Critical:
		// Armed + airborne: execute the CRITICAL parameter action (param
		// wins over the recommendation). On the ground: never auto-act —
		// gate autonomy arming and warn (BlockOffboard carries both).
		return ctx.armed_airborne() ? act_param_to_action(params.crit_act)
		       : MonitorAction::BlockOffboard;

	case CompanionState::Warn:
		// Never auto-act on WARN (spec): warn only, whatever the
		// recommendation.
		return MonitorAction::Warn;

	case CompanionState::Stale:

		// In Offboard while flying: execute the STALE parameter action.
		// Otherwise block Offboard entry iff CC_MON_REQ_OFFB.
		if (ctx.armed_airborne() && ctx.in_offboard) {
			return act_param_to_action(params.stale_act);
		}

		return params.require_offboard ? MonitorAction::BlockOffboard
		       : MonitorAction::None;

	case CompanionState::Ok:
		// Honor an explicit companion BLOCK_OFFBOARD recommendation even
		// at OK severity; otherwise nothing.
		return (recommended_action == ACTION_BLOCK_OFFBOARD)
		       ? MonitorAction::BlockOffboard
		       : MonitorAction::None;

	case CompanionState::Unknown:
	default:
		// Boot state: block Offboard entry iff required.
		return params.require_offboard ? MonitorAction::BlockOffboard
		       : MonitorAction::None;
	}
}

// A MonitorAction that issues an active flight vehicle_command (vs a passive
// warn / arming-gate). Used by the module to decide whether to send a command.
constexpr bool is_flight_command(MonitorAction a)
{
	return a == MonitorAction::Hold || a == MonitorAction::Land || a == MonitorAction::Rtl;
}

} // namespace ccfc
