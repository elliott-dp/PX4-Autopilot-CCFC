/****************************************************************************
 * CCFC fork — cc_safety_monitor parameters (spec §4.5, §12).
 * The safety policy is parameterized, never hardcoded; all are runtime-
 * updatable (the module re-reads them on parameter_update, no reboot).
 ****************************************************************************/

/**
 * Enable the companion safety monitor
 *
 * When 0 the monitor takes no action and echoes reject_reason
 * MONITOR_DISABLED in CC_SAFETY_STATUS; the companion link and telemetry are
 * unaffected.
 *
 * @group CC-FC Safety Monitor
 * @boolean
 */
PARAM_DEFINE_INT32(CC_MON_EN, 1);

/**
 * Require companion OK before Offboard
 *
 * When 1, the monitor blocks/exits Offboard whenever the companion state is
 * UNKNOWN, STALE, or CRITICAL-on-ground. Manual flight is never affected.
 *
 * @group CC-FC Safety Monitor
 * @boolean
 */
PARAM_DEFINE_INT32(CC_MON_REQ_OFFB, 1);

/**
 * Health-report stale timeout
 *
 * The companion state goes STALE if no valid CC_HEALTH_REPORT is accepted
 * within this window.
 *
 * (ICD name CC_MON_TIMEOUT_MS; shortened to fit PX4's 16-char param limit.)
 *
 * @group CC-FC Safety Monitor
 * @unit ms
 * @min 200
 * @max 20000
 */
PARAM_DEFINE_INT32(CC_MON_TMOUT_MS, 3000);

/**
 * Consecutive OK reports to recover
 *
 * Number of consecutive OK reports required to de-escalate from
 * WARN/CRITICAL/STALE back to OK (hysteresis).
 *
 * @group CC-FC Safety Monitor
 * @min 1
 * @max 20
 */
PARAM_DEFINE_INT32(CC_MON_OK_COUNT, 3);

/**
 * Action on CRITICAL (armed + airborne)
 *
 * Executed once, edge-triggered, on the transition into CRITICAL while armed
 * and airborne. The parameter wins over the companion's recommendation. On the
 * ground the monitor never auto-acts (it gates autonomy arming and warns).
 *
 * @group CC-FC Safety Monitor
 * @value 0 Warn only
 * @value 1 Hold / Loiter
 * @value 2 Land
 * @value 3 Return-To-Launch
 */
PARAM_DEFINE_INT32(CC_MON_CRIT_ACT, 1);

/**
 * Action on STALE (in Offboard, airborne)
 *
 * Executed on the transition into STALE while flying in Offboard. Default is
 * Hold (exit Offboard -> Loiter). Same encoding as CC_MON_CRIT_ACT.
 *
 * @group CC-FC Safety Monitor
 * @value 0 Warn only
 * @value 1 Hold / Loiter
 * @value 2 Land
 * @value 3 Return-To-Launch
 */
PARAM_DEFINE_INT32(CC_MON_STALE_ACT, 1);
