# CCFC: Class B IMU/vibration summary for the companion computer (spec §6).
# Mirrors CC_TELEMETRY_IMU in cc_dialect.xml. Nominal 50 Hz on UART.
# Raw full-rate IMU stays in PX4 ULog — this is a summary stream.

uint64 timestamp          # time since system start (microseconds); sampling time

uint32 sequence           # per-stream monotonic counter
uint32 clipping_count     # cumulative accel clipping events since boot
                          # (sum of vehicle_imu_status.accel_clipping[3])

float32[3] accel          # filtered specific force x,y,z (m/s^2) from sensor_combined
float32[3] gyro           # filtered angular rate x,y,z (rad/s) from sensor_combined
float32[3] delta_angle    # integrated delta angle over the IMU integration frame (rad)
float32[3] delta_velocity # integrated delta velocity over the IMU integration frame (m/s)
float32[3] vibration_metric # [accel_vibration_metric (m/s^2),
                            #  gyro_vibration_metric (rad/s),
                            #  delta_angle_coning_metric (rad^2)]
                            # NOTE: PX4 exposes scalar metrics, not per-axis;
                            # the three slots carry the three metrics (documented
                            # deviation D2 in docs/phase2_px4_telemetry.md)
float32 temperature       # IMU temperature (degC), NaN if unknown

uint8 schema_version      # payload schema version
