# CCFC: cc_safety_monitor state echo and report acknowledgement (spec §4.5).
# Mirrors CC_SAFETY_STATUS in cc_dialect.xml. Published by cc_safety_monitor
# (Phase 6) on every state change plus 1 Hz keepalive; streamed back to the
# companion (Phase 3) — last_report_sequence IS the ack that stops the
# companion's 5 Hz CRITICAL repeat.

uint64 timestamp             # time since system start (microseconds)

uint32 last_report_sequence  # highest CC_HEALTH_REPORT sequence accepted
uint32 active_health_flags   # CC_HEALTH_FLAGS of the last accepted report
uint32 report_age_ms         # age of the last accepted report (ms)
uint32 missed_reports        # cumulative sequence gaps detected since boot

uint8 companion_state        # CC_COMPANION_STATE: 0 UNKNOWN, 1 OK, 2 WARN, 3 CRITICAL, 4 STALE
uint8 action_taken           # CC_RECOMMENDED_ACTION the monitor actually executed
uint8 reject_reason          # CC_REJECT_REASON for the most recent rejected inbound (0 = none)
uint8 schema_version         # payload schema version
