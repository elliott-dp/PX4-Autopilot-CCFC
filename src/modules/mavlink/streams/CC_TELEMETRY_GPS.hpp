/****************************************************************************
 * CCFC fork — CC_TELEMETRY_GPS stream (Class D, spec §6). 1:1 copy of the
 * cc_telemetry_gps uORB topic.
 ****************************************************************************/

#ifndef CC_TELEMETRY_GPS_HPP
#define CC_TELEMETRY_GPS_HPP

#include <uORB/topics/cc_telemetry_gps.h>

class MavlinkStreamCcTelemetryGps : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCcTelemetryGps(mavlink); }

	static constexpr const char *get_name_static() { return "CC_TELEMETRY_GPS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CC_TELEMETRY_GPS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_CC_TELEMETRY_GPS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCcTelemetryGps(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(cc_telemetry_gps)};

	bool send() override
	{
		cc_telemetry_gps_s data;

		if (_sub.update(&data)) {
			mavlink_cc_telemetry_gps_t msg{};

			msg.fc_timestamp_us = data.timestamp;
			msg.sequence = data.sequence;
			msg.lat = data.lat;
			msg.lon = data.lon;
			msg.alt = data.alt;
			msg.eph = data.eph;
			msg.epv = data.epv;
			msg.ground_speed = data.ground_speed;
			msg.heading = data.heading;
			msg.noise_per_ms = data.noise_per_ms;
			msg.jamming_indicator = data.jamming_indicator;
			msg.fix_type = data.fix_type;
			msg.satellites_used = data.satellites_used;
			msg.schema_version = data.schema_version;

			mavlink_msg_cc_telemetry_gps_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // CC_TELEMETRY_GPS_HPP
