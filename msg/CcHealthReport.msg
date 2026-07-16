# CCFC: validated companion health report (spec §4.4, §5.8).
# Mirrors CC_HEALTH_REPORT in cc_dialect.xml. Published by the extended
# mavlink_receiver (Phase 3) AFTER the full validation gauntlet; consumed by
# cc_safety_monitor (Phase 6) and the ULog blackbox. THE ONLY message class
# that can influence PX4 behavior, and only via the deterministic policy table.

uint64 timestamp               # time since system start (microseconds); uORB publish time
uint64 companion_timestamp_us  # CC monotonic time at report creation (companion clock)

uint32 sequence                # report sequence, monotonic per cc_boot_id
uint32 mission_id              # current mission identity
uint32 companion_boot_id       # companiond process boot identity
uint32 health_flags            # CC_HEALTH_FLAGS bitmask (see cc_dialect.xml)

uint16 detail_code             # dominant evidence code (joins CC_AI_DIAGNOSTIC / ai_health.parquet)
uint16 link_rtt_ms             # companion-measured link round-trip time (ms)
uint16 telemetry_age_ms        # age of newest FC telemetry at report time (ms)
uint16 companion_loop_ms       # health evaluation loop duration (ms, self-telemetry)
uint16 dropped_rx_count        # frames dropped by companion since boot (self-telemetry)

uint8 severity                 # CC_SEVERITY: 0 OK, 1 WARN, 2 CRITICAL, 3 STALE
uint8 recommended_action       # CC_RECOMMENDED_ACTION (advisory only)
uint8 confidence_percent       # confidence [0..100]
uint8 schema_version           # payload schema version
