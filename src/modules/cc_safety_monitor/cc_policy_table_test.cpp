/****************************************************************************
 * CCFC fork — host unit tests for the cc_safety_monitor policy table.
 *
 * Exhaustive: one case per row of the spec §4.5 table, plus the hard
 * invariants (param-wins-over-recommendation, fail-safe on a bad parameter,
 * conservative-only output). Pure and PX4-free, so it builds and runs on the
 * host:
 *
 *     c++ -std=c++14 -I. cc_policy_table_test.cpp -o /tmp/t && /tmp/t
 *
 * Exit criterion (dev-plan Phase 6): the policy table has 100% host-side unit
 * coverage.
 ****************************************************************************/

#include "cc_policy_table.hpp"
#include "cc_state_machine.hpp"

#include <cstdio>

using namespace ccfc;

static int g_checks = 0;
static int g_fail = 0;

static const char *name(MonitorAction a)
{
	switch (a) {
	case MonitorAction::None:          return "None";

	case MonitorAction::Warn:          return "Warn";

	case MonitorAction::BlockOffboard: return "BlockOffboard";

	case MonitorAction::Hold:          return "Hold";

	case MonitorAction::Land:          return "Land";

	case MonitorAction::Rtl:           return "Rtl";
	}

	return "?";
}

#define EXPECT(desc, got, want) do {                                            \
		++g_checks;                                                    \
		MonitorAction _g = (got);                                      \
		MonitorAction _w = (want);                                     \
		if (_g != _w) {                                                \
			++g_fail;                                              \
			std::printf("FAIL  %-52s got %s want %s\n", desc,      \
				    name(_g), name(_w));                      \
		} else {                                                      \
			std::printf("ok    %-52s -> %s\n", desc, name(_g));   \
		}                                                             \
	} while (0)

// context shorthands
static constexpr FlightContext AIRBORNE_OFFB { /*offb*/true,  /*armed*/true,  /*air*/true  };
static constexpr FlightContext AIRBORNE_MAN  { /*offb*/false, /*armed*/true,  /*air*/true  };
static constexpr FlightContext GROUND_ARMED  { /*offb*/false, /*armed*/true,  /*air*/false };
static constexpr FlightContext GROUND_DISARM { /*offb*/false, /*armed*/false, /*air*/false };
static constexpr FlightContext GROUND_OFFB   { /*offb*/true,  /*armed*/true,  /*air*/false };

static constexpr PolicyParams P(bool req, int32_t crit, int32_t stale)
{
	return PolicyParams{ req, crit, stale };
}

int main()
{
	// --- CRITICAL, armed + airborne: execute CC_MON_CRIT_ACT ----------------
	EXPECT("CRITICAL air crit_act=land",
	       decide_action(CompanionState::Critical, ACTION_NONE, AIRBORNE_OFFB, P(true, 2, 1)),
	       MonitorAction::Land);
	EXPECT("CRITICAL air crit_act=hold",
	       decide_action(CompanionState::Critical, ACTION_NONE, AIRBORNE_OFFB, P(true, 1, 1)),
	       MonitorAction::Hold);
	EXPECT("CRITICAL air crit_act=rtl",
	       decide_action(CompanionState::Critical, ACTION_NONE, AIRBORNE_OFFB, P(true, 3, 1)),
	       MonitorAction::Rtl);
	EXPECT("CRITICAL air crit_act=warn",
	       decide_action(CompanionState::Critical, ACTION_NONE, AIRBORNE_OFFB, P(true, 0, 1)),
	       MonitorAction::Warn);
	// works in manual flight too (armed+airborne, not offboard)
	EXPECT("CRITICAL air(manual) crit_act=land",
	       decide_action(CompanionState::Critical, ACTION_NONE, AIRBORNE_MAN, P(true, 2, 1)),
	       MonitorAction::Land);

	// param WINS over the companion recommendation
	EXPECT("CRITICAL param(land) beats recommend(rtl)",
	       decide_action(CompanionState::Critical, ACTION_RTL, AIRBORNE_OFFB, P(true, 2, 1)),
	       MonitorAction::Land);

	// fail-safe: a bad CC_MON_CRIT_ACT falls to Hold, never None
	EXPECT("CRITICAL air crit_act=99 fails safe to Hold",
	       decide_action(CompanionState::Critical, ACTION_NONE, AIRBORNE_OFFB, P(true, 99, 1)),
	       MonitorAction::Hold);

	// --- CRITICAL on the ground: block autonomy arming + warn ---------------
	EXPECT("CRITICAL ground(disarmed) -> block",
	       decide_action(CompanionState::Critical, ACTION_LAND, GROUND_DISARM, P(true, 2, 1)),
	       MonitorAction::BlockOffboard);
	EXPECT("CRITICAL ground(armed, not airborne) -> block",
	       decide_action(CompanionState::Critical, ACTION_LAND, GROUND_ARMED, P(true, 2, 1)),
	       MonitorAction::BlockOffboard);

	// --- WARN: warn only, never auto-act ------------------------------------
	EXPECT("WARN airborne -> warn only",
	       decide_action(CompanionState::Warn, ACTION_LAND, AIRBORNE_OFFB, P(true, 2, 1)),
	       MonitorAction::Warn);
	EXPECT("WARN recommend BLOCK_OFFBOARD still warn only",
	       decide_action(CompanionState::Warn, ACTION_BLOCK_OFFBOARD, AIRBORNE_OFFB, P(true, 2, 1)),
	       MonitorAction::Warn);

	// --- STALE ---------------------------------------------------------------
	EXPECT("STALE airborne+offboard stale_act=hold",
	       decide_action(CompanionState::Stale, ACTION_NONE, AIRBORNE_OFFB, P(true, 2, 1)),
	       MonitorAction::Hold);
	EXPECT("STALE airborne+offboard stale_act=land",
	       decide_action(CompanionState::Stale, ACTION_NONE, AIRBORNE_OFFB, P(true, 2, 2)),
	       MonitorAction::Land);
	EXPECT("STALE not-offboard req_offb -> block",
	       decide_action(CompanionState::Stale, ACTION_NONE, AIRBORNE_MAN, P(true, 2, 1)),
	       MonitorAction::BlockOffboard);
	EXPECT("STALE not-offboard !req_offb -> none",
	       decide_action(CompanionState::Stale, ACTION_NONE, AIRBORNE_MAN, P(false, 2, 1)),
	       MonitorAction::None);
	EXPECT("STALE offboard-but-on-ground req_offb -> block",
	       decide_action(CompanionState::Stale, ACTION_NONE, GROUND_OFFB, P(true, 2, 1)),
	       MonitorAction::BlockOffboard);

	// --- OK ------------------------------------------------------------------
	EXPECT("OK recommend BLOCK_OFFBOARD -> honor",
	       decide_action(CompanionState::Ok, ACTION_BLOCK_OFFBOARD, AIRBORNE_OFFB, P(true, 2, 1)),
	       MonitorAction::BlockOffboard);
	EXPECT("OK recommend NONE -> none",
	       decide_action(CompanionState::Ok, ACTION_NONE, AIRBORNE_OFFB, P(true, 2, 1)),
	       MonitorAction::None);
	// OK never lands even if the companion (contradictorily) recommends LAND
	EXPECT("OK recommend LAND -> none (OK never auto-acts)",
	       decide_action(CompanionState::Ok, ACTION_LAND, AIRBORNE_OFFB, P(true, 2, 1)),
	       MonitorAction::None);

	// --- UNKNOWN -------------------------------------------------------------
	EXPECT("UNKNOWN req_offb -> block",
	       decide_action(CompanionState::Unknown, ACTION_NONE, GROUND_DISARM, P(true, 2, 1)),
	       MonitorAction::BlockOffboard);
	EXPECT("UNKNOWN !req_offb -> none",
	       decide_action(CompanionState::Unknown, ACTION_NONE, GROUND_DISARM, P(false, 2, 1)),
	       MonitorAction::None);

	// ======================================================================
	// State machine (hysteresis / staleness / reboot)
	// ======================================================================
	auto sname = [](CompanionState s) {
		switch (s) {
		case CompanionState::Unknown:  return "Unknown";

		case CompanionState::Ok:       return "Ok";

		case CompanionState::Warn:     return "Warn";

		case CompanionState::Critical: return "Critical";

		case CompanionState::Stale:    return "Stale";
		}

		return "?";
	};
#define EXPECT_STATE(desc, got, want) do {                                     \
		++g_checks;                                                    \
		CompanionState _g = (got), _w = (want);                        \
		if (_g != _w) { ++g_fail;                                      \
			std::printf("FAIL  %-52s got %s want %s\n", desc, sname(_g), sname(_w)); } \
		else { std::printf("ok    %-52s -> %s\n", desc, sname(_g)); }  \
	} while (0)

	const uint32_t OKN = 3; // CC_MON_OK_COUNT

	{
		MonitorStateMachine m;
		EXPECT_STATE("SM: first report OK -> Ok immediately", m.on_report(SEVERITY_OK, OKN), CompanionState::Ok);
	}
	{
		MonitorStateMachine m;
		EXPECT_STATE("SM: first report CRITICAL -> Critical", m.on_report(SEVERITY_CRITICAL, OKN), CompanionState::Critical);
	}
	{
		MonitorStateMachine m; m.on_report(SEVERITY_OK, OKN);
		EXPECT_STATE("SM: Ok -> WARN escalates immediately", m.on_report(SEVERITY_WARN, OKN), CompanionState::Warn);
	}
	{
		MonitorStateMachine m; m.on_report(SEVERITY_WARN, OKN);
		EXPECT_STATE("SM: Warn -> CRITICAL escalates", m.on_report(SEVERITY_CRITICAL, OKN), CompanionState::Critical);
	}
	{
		MonitorStateMachine m; m.on_report(SEVERITY_CRITICAL, OKN);
		m.on_report(SEVERITY_OK, OKN); m.on_report(SEVERITY_OK, OKN);
		EXPECT_STATE("SM: Critical + 2 OK (<count) stays Critical", m.state(), CompanionState::Critical);
		EXPECT_STATE("SM: Critical + 3rd OK -> Ok", m.on_report(SEVERITY_OK, OKN), CompanionState::Ok);
	}
	{
		MonitorStateMachine m; m.on_report(SEVERITY_CRITICAL, OKN);
		m.on_report(SEVERITY_OK, OKN); m.on_report(SEVERITY_OK, OKN);
		m.on_report(SEVERITY_WARN, OKN); // interrupts the OK streak, stays Critical
		EXPECT_STATE("SM: WARN interrupts recovery streak (stays Critical)", m.state(), CompanionState::Critical);
		m.on_report(SEVERITY_OK, OKN); m.on_report(SEVERITY_OK, OKN);
		EXPECT_STATE("SM: Critical + fresh 3 OK -> Ok", m.on_report(SEVERITY_OK, OKN), CompanionState::Ok);
	}
	{
		MonitorStateMachine m; m.on_report(SEVERITY_CRITICAL, OKN);
		EXPECT_STATE("SM: Critical + WARN never relaxes to Warn", m.on_report(SEVERITY_WARN, OKN), CompanionState::Critical);
	}
	{
		MonitorStateMachine m; m.on_report(SEVERITY_WARN, OKN);
		m.on_report(SEVERITY_OK, OKN); m.on_report(SEVERITY_OK, OKN);
		EXPECT_STATE("SM: Warn + 3 OK -> Ok", m.on_report(SEVERITY_OK, OKN), CompanionState::Ok);
	}
	{
		MonitorStateMachine m; m.on_report(SEVERITY_OK, OKN);
		EXPECT_STATE("SM: timeout -> Stale", m.on_timeout(), CompanionState::Stale);
	}
	{
		MonitorStateMachine m; m.on_timeout();
		m.on_report(SEVERITY_OK, OKN); m.on_report(SEVERITY_OK, OKN);
		EXPECT_STATE("SM: Stale + 2 OK stays Stale", m.state(), CompanionState::Stale);
		EXPECT_STATE("SM: Stale + 3rd OK -> Ok", m.on_report(SEVERITY_OK, OKN), CompanionState::Ok);
	}
	{
		MonitorStateMachine m; m.on_timeout();
		EXPECT_STATE("SM: Stale + WARN stays Stale (exit needs OKs)", m.on_report(SEVERITY_WARN, OKN), CompanionState::Stale);
	}
	{
		MonitorStateMachine m; m.on_timeout();
		EXPECT_STATE("SM: Stale + CRITICAL escalates to Critical", m.on_report(SEVERITY_CRITICAL, OKN), CompanionState::Critical);
	}
	{
		MonitorStateMachine m; m.on_report(SEVERITY_CRITICAL, OKN); m.reset();
		EXPECT_STATE("SM: reboot resets to Unknown", m.state(), CompanionState::Unknown);
	}
	{
		MonitorStateMachine m; m.on_report(SEVERITY_CRITICAL, OKN);
		EXPECT_STATE("SM: ok_count=1, Critical + single OK -> Ok", m.on_report(SEVERITY_OK, 1), CompanionState::Ok);
	}
	{
		MonitorStateMachine m; m.on_report(SEVERITY_OK, OKN);
		EXPECT_STATE("SM: self-declared STALE severity -> Stale", m.on_report(SEVERITY_STALE, OKN), CompanionState::Stale);
	}

	// --- Invariant: every reachable output is conservative ------------------
	// (Exhaustive sweep: no (state,recommend,ctx,param) combination can produce
	//  an action outside the conservative set — enforced by construction, but
	//  checked here so a future edit that added an escalating output fails.)
	{
		bool all_conservative = true;
		const CompanionState states[] = { CompanionState::Unknown, CompanionState::Ok,
						  CompanionState::Warn, CompanionState::Critical, CompanionState::Stale
						};

		for (CompanionState st : states) {
			for (uint8_t rec = 0; rec <= 5; ++rec) {
				for (int oi = 0; oi < 2; ++oi) {
					for (int ai = 0; ai < 2; ++ai) {
						for (int ri = 0; ri < 2; ++ri) {
							for (int32_t act = -1; act <= 4; ++act) {
								FlightContext c{ oi != 0, ai != 0, ai != 0 };
								MonitorAction a = decide_action(st, rec, c, P(ri != 0, act, act));

								// conservative set only (no arm/takeoff/offboard-enter exists)
								if (!(a == MonitorAction::None || a == MonitorAction::Warn ||
								      a == MonitorAction::BlockOffboard || a == MonitorAction::Hold ||
								      a == MonitorAction::Land || a == MonitorAction::Rtl)) {
									all_conservative = false;
								}
							}
						}
					}
				}
			}
		}

		++g_checks;

		if (!all_conservative) { ++g_fail; std::printf("FAIL  exhaustive sweep: non-conservative output\n"); }

		else { std::printf("ok    exhaustive sweep: all outputs conservative\n"); }
	}

	std::printf("\n%d/%d policy-table checks passed\n", g_checks - g_fail, g_checks);
	return g_fail == 0 ? 0 : 1;
}
