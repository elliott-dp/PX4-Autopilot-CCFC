# CCFC: log-only companion diagnostic detail (spec §4.4, §13).
# Published by the extended mavlink_receiver after validation; NEVER drives
# policy — it exists so ULog can capture "why the AI said it" alongside the
# compact CC_HEALTH_REPORT ("what PX4 was told"), joined by detail_code.
# NOTE: uORB handoff topic added beyond the spec §4.1 list — deviation D10
# in docs/phase3/phase3_mavlink_link.md.

uint64 timestamp               # time since system start (microseconds); receive time
uint64 companion_timestamp_us  # CC monotonic time at finding creation

uint32 sequence                # diagnostic sequence, monotonic per cc_boot_id

float32 value                  # measured value that triggered the finding
float32 limit                  # threshold the value was compared against

uint16 detail_code             # evidence code (joins CC_HEALTH_REPORT.detail_code)

uint8 subsystem                # CC_SUBSYSTEM of the finding
uint8 severity                 # CC_SEVERITY of this individual finding
uint8 confidence_percent       # confidence [0..100]
uint8 schema_version           # payload schema version
