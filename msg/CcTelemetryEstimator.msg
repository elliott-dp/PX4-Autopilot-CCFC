# CCFC: Class E estimator health for the companion computer (spec §6).
# Mirrors CC_TELEMETRY_ESTIMATOR in cc_dialect.xml. Nominal 10 Hz (UART).
# Sampled from the PRIMARY estimator instance (estimator_selector_status).

uint64 timestamp              # time since system start (microseconds); sampling time

uint32 sequence               # per-stream monotonic counter
uint32 status_flags           # EKF fault flags (estimator_status.filter_fault_flags)

uint16 innovation_check_flags # compacted from estimator_status_flags.reject_* bools
                              # (v1.17 dropped the legacy bitmask), bit table:
                              #   0x0001 reject_hor_vel    0x0002 reject_ver_vel
                              #   0x0004 reject_hor_pos    0x0008 reject_ver_pos
                              #   0x0010 reject_yaw        0x0020 reject_airspeed
                              #   0x0040 reject_sideslip   0x0080 reject_hagl
                              #   0x0100 reject_optflow_x  0x0200 reject_optflow_y
uint16 solution_status_flags  # estimator_status.solution_status_flags

float32 velocity_test_ratio   # estimator_status.vel_test_ratio (<1 = passing)
float32 position_test_ratio   # estimator_status.pos_test_ratio
float32 height_test_ratio     # estimator_status.hgt_test_ratio
float32 mag_test_ratio        # estimator_status.hdg_test_ratio (heading innovation)
float32 airspeed_test_ratio   # estimator_status.tas_test_ratio (NaN if not applicable)

uint8 schema_version          # payload schema version
