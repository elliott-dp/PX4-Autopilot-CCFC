/****************************************************************************
 * CCFC fork — cc_telemetry_publisher parameters (spec §12).
 * Safety/telemetry behavior is parameterized, never hardcoded; all are
 * runtime-updatable (the module re-reads them on parameter_update).
 ****************************************************************************/

/**
 * Companion telemetry profile
 *
 * Selects the stream set and rates of the cc_telemetry_* topics feeding the
 * companion computer link (mirrors CC_LOG_PROFILE in cc_dialect.xml).
 * MINIMAL disables all companion telemetry streams (heartbeat + safety
 * traffic only). AI_UART is the mission profile sized for the 921600-baud
 * TELEM3 link. AI_ETH/DEBUG raise rates (200 Hz base tick) and are only for
 * Ethernet transport or bench work, after a measured soak test (spec §8).
 *
 * @group CC-FC Companion Link
 * @value 0 MINIMAL (companion telemetry off)
 * @value 1 AI_UART (mission profile, UART rates)
 * @value 2 AI_ETH (elevated rates, Ethernet only)
 * @value 3 DEBUG (development, full rates)
 */
PARAM_DEFINE_INT32(CC_TEL_PROFILE, 1);

/**
 * Companion IMU summary stream rate
 *
 * Rate of the CC_TELEMETRY_IMU (Class B) stream. Effective rate is the
 * nearest integer divisor of the profile base tick (50 Hz on AI_UART,
 * 200 Hz on AI_ETH/DEBUG) and is additionally capped at 50 Hz on AI_UART.
 * 0 disables the stream.
 *
 * @group CC-FC Companion Link
 * @unit Hz
 * @min 0
 * @max 200
 */
PARAM_DEFINE_INT32(CC_TEL_IMU_RATE, 50);

/**
 * Companion actuator stream rate
 *
 * Rate of the CC_TELEMETRY_ACTUATOR (Class F) stream. Effective rate is the
 * nearest integer divisor of the profile base tick, capped at 50 Hz on
 * AI_ETH/DEBUG and at the base tick on AI_UART. 0 disables the stream.
 *
 * @group CC-FC Companion Link
 * @unit Hz
 * @min 0
 * @max 50
 */
PARAM_DEFINE_INT32(CC_TEL_ACT_RATE, 20);

/**
 * Vehicle identity for the companion link
 *
 * Static identity shared by FC and companion (spec section 7): the
 * companion sends it in CC_MISSION_CONTEXT and the receiver refuses the
 * mission handshake on mismatch (configuration error). Must equal the
 * vehicle_id in the companion's cc-config.
 *
 * @group CC-FC Companion Link
 * @min 1
 * @max 2147483647
 */
PARAM_DEFINE_INT32(CC_VEHICLE_ID, 1);
