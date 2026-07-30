/****************************************************************************
 * CCFC fork — cc_safety_monitor companion state machine (spec §4.5).
 *
 * PURE (PX4-free) so it is host-unit-tested alongside the policy table. Owns
 * the hysteresis and staleness rules that decide the companion state; the
 * module drives it with report / timeout / reboot events and feeds the
 * resulting state into the policy table.
 *
 * Rules (spec §4.5):
 *   - UNKNOWN is the boot state; the first valid report sets the state per its
 *     severity.
 *   - Escalation is IMMEDIATE (OK->WARN, *->CRITICAL from any state).
 *   - De-escalation to OK requires CC_MON_OK_COUNT consecutive OK reports; a
 *     non-OK report resets that streak. CRITICAL therefore never relaxes to
 *     WARN — it stays CRITICAL until a clean run of OKs clears it.
 *   - STALE is entered on report-age timeout (or a companion self-declared
 *     STALE severity). Exiting STALE requires CC_MON_OK_COUNT fresh OKs; a
 *     CRITICAL report still escalates STALE->CRITICAL immediately (more
 *     conservative). A WARN report keeps STALE (spec: exit needs OKs).
 *   - FC reboot resets to UNKNOWN.
 ****************************************************************************/

#pragma once

#include <cstdint>

#include "cc_policy_table.hpp"

namespace ccfc
{

// CC_SEVERITY values (report input).
static constexpr uint8_t SEVERITY_OK       = 0;
static constexpr uint8_t SEVERITY_WARN     = 1;
static constexpr uint8_t SEVERITY_CRITICAL = 2;
static constexpr uint8_t SEVERITY_STALE    = 3;

class MonitorStateMachine
{
public:
	CompanionState state() const { return _state; }
	uint32_t ok_streak() const { return _ok_streak; }

	// Apply a valid, accepted report of the given severity. `ok_count` is
	// CC_MON_OK_COUNT (>= 1). Returns the new state.
	CompanionState on_report(uint8_t severity, uint32_t ok_count)
	{
		const uint32_t need = ok_count < 1 ? 1 : ok_count;

		switch (severity) {
		case SEVERITY_CRITICAL:
			_state = CompanionState::Critical;
			_ok_streak = 0;
			break;

		case SEVERITY_STALE: // companion self-declares its data untrustworthy
			_state = CompanionState::Stale;
			_ok_streak = 0;
			break;

		case SEVERITY_WARN:
			_ok_streak = 0;

			// escalate to WARN only from OK/UNKNOWN; never relax
			// CRITICAL or leave STALE on a WARN.
			if (_state != CompanionState::Critical && _state != CompanionState::Stale) {
				_state = CompanionState::Warn;
			}

			break;

		case SEVERITY_OK:
		default:
			if (_state == CompanionState::Unknown || _state == CompanionState::Ok) {
				// first report (or already OK): OK immediately.
				_state = CompanionState::Ok;
				_ok_streak = need;

			} else {
				// recovering from WARN/CRITICAL/STALE: need a run of OKs.
				if (_ok_streak < need) { _ok_streak++; }

				if (_ok_streak >= need) { _state = CompanionState::Ok; }
			}

			break;
		}

		return _state;
	}

	// Report age exceeded CC_MON_TIMEOUT_MS with no valid report.
	CompanionState on_timeout()
	{
		_state = CompanionState::Stale;
		_ok_streak = 0;
		return _state;
	}

	// FC reboot (or monitor (re)start): back to the boot state.
	void reset()
	{
		_state = CompanionState::Unknown;
		_ok_streak = 0;
	}

private:
	CompanionState _state{CompanionState::Unknown};
	uint32_t _ok_streak{0};
};

} // namespace ccfc
