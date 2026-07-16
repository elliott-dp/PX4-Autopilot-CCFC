# CCFC: Class F commanded actuator outputs for the companion computer (spec §6).
# Mirrors CC_TELEMETRY_ACTUATOR in cc_dialect.xml (the TRIMMED form: the
# fitted ESCs provide no telemetry, so this intentionally carries commanded
# outputs only; per-motor RPM/current/temp would be a future CC_TELEMETRY_ESC).
# Nominal 20 Hz (UART). Unused slots beyond motor_count are NaN.

uint64 timestamp           # time since system start (microseconds); sampling time

uint32 sequence            # per-stream monotonic counter

float32[8] actuator_output # commanded outputs from actuator_motors.control[0..7],
                           # normalized per PX4 output function; NaN beyond motor_count

uint8 motor_count          # number of valid (finite) entries in actuator_output
uint8 schema_version       # payload schema version
