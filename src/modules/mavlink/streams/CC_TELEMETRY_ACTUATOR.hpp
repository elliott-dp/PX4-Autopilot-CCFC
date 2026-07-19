/****************************************************************************
 * CCFC fork — CC_TELEMETRY_ACTUATOR stream (Class F, spec §6; trimmed form,
 * the fitted ESCs have no telemetry). 1:1 copy of the cc_telemetry_actuator
 * uORB topic.
 ****************************************************************************/

#ifndef CC_TELEMETRY_ACTUATOR_HPP
#define CC_TELEMETRY_ACTUATOR_HPP

#include <uORB/topics/cc_telemetry_actuator.h>

class MavlinkStreamCcTelemetryActuator : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCcTelemetryActuator(mavlink); }

	static constexpr const char *get_name_static() { return "CC_TELEMETRY_ACTUATOR"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CC_TELEMETRY_ACTUATOR; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_CC_TELEMETRY_ACTUATOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCcTelemetryActuator(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(cc_telemetry_actuator)};

	bool send() override
	{
		cc_telemetry_actuator_s data;

		if (_sub.update(&data)) {
			mavlink_cc_telemetry_actuator_t msg{};

			msg.fc_timestamp_us = data.timestamp;
			msg.sequence = data.sequence;
			memcpy(msg.actuator_output, data.actuator_output, sizeof(msg.actuator_output));
			msg.motor_count = data.motor_count;
			msg.schema_version = data.schema_version;

			mavlink_msg_cc_telemetry_actuator_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // CC_TELEMETRY_ACTUATOR_HPP
