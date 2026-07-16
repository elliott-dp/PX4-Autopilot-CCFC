# CCFC: Class D GNSS/navigation quality for the companion computer (spec §6).
# Mirrors CC_TELEMETRY_GPS in cc_dialect.xml. Nominal 5 Hz (UART).

uint64 timestamp        # time since system start (microseconds); sampling time

uint32 sequence         # per-stream monotonic counter

int32 lat               # latitude,  WGS-84, degrees * 1e7 (from sensor_gps.latitude_deg)
int32 lon               # longitude, WGS-84, degrees * 1e7 (from sensor_gps.longitude_deg)
int32 alt               # altitude AMSL, millimeters (from sensor_gps.altitude_msl_m)

float32 eph             # horizontal position uncertainty (m)
float32 epv             # vertical position uncertainty (m)
float32 ground_speed    # ground speed (m/s)
float32 heading         # course over ground [-pi, pi] (rad); NaN if unknown

uint16 noise_per_ms     # receiver noise indicator, clamped from sensor_gps (0 if unavailable)
uint16 jamming_indicator # receiver jamming indicator, clamped from sensor_gps (0 if unavailable)

uint8 fix_type          # GNSS fix type (sensor_gps.fix_type)
uint8 satellites_used   # satellites used in solution
uint8 schema_version    # payload schema version
