/****************************************************************************
 * CCFC fork — CC_SAFETY_STATUS stream (spec §4.3, §4.5): monitor state echo
 * and report acknowledgement. 1:1 copy of the cc_safety_status uORB topic.
 * The publisher (cc_safety_monitor) arrives in Phase 6 — until then the
 * topic is never advertised and this stream costs nothing (get_size() == 0).
 ****************************************************************************/

#ifndef CC_SAFETY_STATUS_HPP
#define CC_SAFETY_STATUS_HPP

#include <uORB/topics/cc_safety_status.h>

class MavlinkStreamCcSafetyStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCcSafetyStatus(mavlink); }

	static constexpr const char *get_name_static() { return "CC_SAFETY_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CC_SAFETY_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_CC_SAFETY_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCcSafetyStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(cc_safety_status)};

	bool send() override
	{
		cc_safety_status_s data;

		if (_sub.update(&data)) {
			mavlink_cc_safety_status_t msg{};

			msg.fc_timestamp_us = data.timestamp;
			msg.last_report_sequence = data.last_report_sequence;
			msg.active_health_flags = data.active_health_flags;
			msg.report_age_ms = data.report_age_ms;
			msg.missed_reports = data.missed_reports;
			msg.companion_state = data.companion_state;
			msg.action_taken = data.action_taken;
			msg.reject_reason = data.reject_reason;
			msg.schema_version = data.schema_version;

			mavlink_msg_cc_safety_status_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // CC_SAFETY_STATUS_HPP
