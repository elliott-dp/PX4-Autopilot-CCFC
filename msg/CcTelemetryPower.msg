# CCFC: Class C power/battery for the companion computer (spec §6).
# Mirrors CC_TELEMETRY_POWER in cc_dialect.xml. Nominal 10 Hz (UART).

uint64 timestamp       # time since system start (microseconds); sampling time

uint32 sequence        # per-stream monotonic counter

float32 voltage        # pack voltage (V); NaN if unknown
float32 current        # pack current (A), positive = discharge; NaN if unknown
float32 power          # instantaneous power (W) = voltage * current; NaN if unknown
float32 consumed_mah   # consumed charge since boot (mAh); NaN if unknown
float32 remaining      # remaining fraction [0..1]; NaN if unknown
float32 temperature    # battery temperature (degC); NaN if unknown

uint8 cell_count       # detected/configured cell count (0 if unknown)
uint8 warning          # PX4 battery warning level (battery_status.warning)
uint8 connected        # 1 if battery telemetry valid
uint8 schema_version   # payload schema version
