/****************************************************************************
 * CCFC fork — CC_TELEMETRY_POWER stream (Class C, spec §6). 1:1 copy of the
 * cc_telemetry_power uORB topic.
 ****************************************************************************/

#ifndef CC_TELEMETRY_POWER_HPP
#define CC_TELEMETRY_POWER_HPP

#include <uORB/topics/cc_telemetry_power.h>

class MavlinkStreamCcTelemetryPower : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCcTelemetryPower(mavlink); }

	static constexpr const char *get_name_static() { return "CC_TELEMETRY_POWER"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CC_TELEMETRY_POWER; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_CC_TELEMETRY_POWER_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCcTelemetryPower(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(cc_telemetry_power)};

	bool send() override
	{
		cc_telemetry_power_s data;

		if (_sub.update(&data)) {
			mavlink_cc_telemetry_power_t msg{};

			msg.fc_timestamp_us = data.timestamp;
			msg.sequence = data.sequence;
			msg.voltage = data.voltage;
			msg.current = data.current;
			msg.power = data.power;
			msg.consumed_mah = data.consumed_mah;
			msg.remaining = data.remaining;
			msg.temperature = data.temperature;
			msg.cell_count = data.cell_count;
			msg.warning = data.warning;
			msg.connected = data.connected;
			msg.schema_version = data.schema_version;

			mavlink_msg_cc_telemetry_power_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // CC_TELEMETRY_POWER_HPP
