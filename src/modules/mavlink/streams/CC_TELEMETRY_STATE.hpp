/****************************************************************************
 * CCFC fork — CC_TELEMETRY_STATE stream (Class A, spec §6).
 * 1:1 copy of the cc_telemetry_state uORB topic curated by
 * cc_telemetry_publisher; uORB decimation is the rate authority, the
 * mavlink stream rate is only a ceiling (spec §4.3).
 ****************************************************************************/

#ifndef CC_TELEMETRY_STATE_HPP
#define CC_TELEMETRY_STATE_HPP

#include <uORB/topics/cc_telemetry_state.h>

class MavlinkStreamCcTelemetryState : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCcTelemetryState(mavlink); }

	static constexpr const char *get_name_static() { return "CC_TELEMETRY_STATE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CC_TELEMETRY_STATE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_CC_TELEMETRY_STATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCcTelemetryState(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(cc_telemetry_state)};

	bool send() override
	{
		cc_telemetry_state_s data;

		if (_sub.update(&data)) {
			mavlink_cc_telemetry_state_t msg{};

			msg.fc_timestamp_us = data.timestamp;
			msg.sequence = data.sequence;
			msg.px4_boot_id = data.px4_boot_id;
			msg.mission_id = data.mission_id;
			msg.failsafe_flags = data.failsafe_flags;
			memcpy(msg.q, data.q, sizeof(msg.q));
			memcpy(msg.angular_velocity, data.angular_velocity, sizeof(msg.angular_velocity));
			memcpy(msg.position_ned, data.position_ned, sizeof(msg.position_ned));
			memcpy(msg.velocity_ned, data.velocity_ned, sizeof(msg.velocity_ned));
			msg.heading = data.heading;
			msg.nav_state = data.nav_state;
			msg.arming_state = data.arming_state;
			msg.vehicle_type = data.vehicle_type;
			msg.estimator_valid = data.estimator_valid;
			msg.control_mode_flags = data.control_mode_flags;
			msg.schema_version = data.schema_version;

			mavlink_msg_cc_telemetry_state_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // CC_TELEMETRY_STATE_HPP
