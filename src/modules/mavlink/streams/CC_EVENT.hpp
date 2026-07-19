/****************************************************************************
 * CCFC fork — CC_EVENT stream (Class G, spec §6): event-driven notifications
 * sourced from PX4's events interface (uORB `event` topic; no dedicated
 * Cc*.msg exists by design). Mapping (deviation D12,
 * docs/phase3/phase3_mavlink_link.md):
 *   event_id   <- event.id
 *   argument0  <- event.arguments[0..3]  (little-endian)
 *   argument1  <- event.arguments[4..7]  (little-endian)
 *   sequence   <- event.event_sequence (events interface's own counter,
 *                 uint16 widened to uint32 — wraps earlier than 2^32; the
 *                 consumer's gap logic must tolerate that until a dedicated
 *                 counter is warranted)
 *   severity   <- internal log level (MSB nibble of log_levels):
 *                 emerg/alert/crit/err -> CRITICAL, warning/notice -> WARN,
 *                 info/debug -> OK
 *   subsystem  <- CC_SUBSYS_NONE (PX4 events carry no CC taxonomy)
 ****************************************************************************/

#ifndef CC_EVENT_HPP
#define CC_EVENT_HPP

#include <uORB/topics/event.h>

class MavlinkStreamCcEvent : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCcEvent(mavlink); }

	static constexpr const char *get_name_static() { return "CC_EVENT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CC_EVENT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_CC_EVENT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCcEvent(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(event)};

	static uint8_t severity_from_log_level(uint8_t log_levels)
	{
		const uint8_t internal = log_levels >> 4;

		if (internal <= 3) { return CC_SEVERITY_CRITICAL; }   // emerg/alert/crit/err

		if (internal <= 5) { return CC_SEVERITY_WARN; }       // warning/notice

		return CC_SEVERITY_OK;                                // info/debug
	}

	bool send() override
	{
		event_s data;

		if (_sub.update(&data)) {
			mavlink_cc_event_t msg{};

			msg.fc_timestamp_us = data.timestamp;
			msg.sequence = data.event_sequence;
			msg.event_id = data.id;
			memcpy(&msg.argument0, &data.arguments[0], sizeof(msg.argument0));
			memcpy(&msg.argument1, &data.arguments[4], sizeof(msg.argument1));
			msg.severity = severity_from_log_level(data.log_levels);
			msg.subsystem = CC_SUBSYS_NONE;
			msg.schema_version = 1;

			mavlink_msg_cc_event_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // CC_EVENT_HPP
