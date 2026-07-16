# CCFC: Class A vehicle state for the companion computer (spec §6).
# Curated by cc_telemetry_publisher; mirrors CC_TELEMETRY_STATE in
# cc_dialect.xml (fc_timestamp_us <- timestamp at the MAVLink boundary).
# Nominal 25 Hz (AI_UART) / 50 Hz (AI_ETH).

uint64 timestamp          # time since system start (microseconds); sampling time

uint32 sequence           # per-stream monotonic counter, wraps at 2^32
uint32 px4_boot_id        # FC boot identity, constant per boot, nonzero
uint32 mission_id         # mission identity echoed from CC_MISSION_CONTEXT (0 if none)
uint32 failsafe_flags     # compacted failsafe_flags bitfield, bit table below

float32[4] q              # attitude quaternion (w, x, y, z), body to NED; NaN if unknown
float32[3] angular_velocity  # body rates (rad/s); NaN if unknown
float32[3] position_ned   # local position NED (m); NaN if invalid
float32[3] velocity_ned   # local velocity NED (m/s); NaN if invalid
float32 heading           # yaw NED [-pi, pi] (rad); NaN if unknown

uint8 nav_state           # vehicle_status.nav_state
uint8 arming_state        # vehicle_status.arming_state
uint8 vehicle_type        # vehicle_status.system_type (MAV_TYPE, as in HEARTBEAT)
uint8 estimator_valid     # 1 if attitude AND local position xy/z valid
uint8 control_mode_flags  # compacted vehicle_control_mode, bit table below
uint8 schema_version      # payload schema version (CC_SCHEMA_VERSION)

# control_mode_flags bit table (from vehicle_control_mode):
#   1 flag_armed                      2 flag_control_manual_enabled
#   4 flag_control_auto_enabled       8 flag_control_offboard_enabled
#  16 flag_control_position_enabled  32 flag_control_velocity_enabled
#  64 flag_control_altitude_enabled 128 flag_control_termination_enabled
#
# failsafe_flags bit table (from failsafe_flags topic):
#   0x0001 manual_control_signal_lost   0x0002 gcs_connection_lost
#   0x0004 battery_low_remaining_time   0x0008 battery_unhealthy
#   0x0010 geofence_breached            0x0020 mission_failure
#   0x0040 wind_limit_exceeded          0x0080 flight_time_limit_exceeded
#   0x0100 position_accuracy_low        0x0200 navigator_failure
#   0x0400 offboard_control_signal_lost 0x0800 fd_critical_failure
#   0x1000 fd_esc_arming_failure        0x2000 fd_imbalanced_prop
#   0x4000 fd_motor_failure             0x8000 local_position_invalid
