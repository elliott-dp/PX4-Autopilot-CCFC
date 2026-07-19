/****************************************************************************
 * CCFC fork — CC_TELEMETRY_ESTIMATOR stream (Class E, spec §6). 1:1 copy of
 * the cc_telemetry_estimator uORB topic.
 ****************************************************************************/

#ifndef CC_TELEMETRY_ESTIMATOR_HPP
#define CC_TELEMETRY_ESTIMATOR_HPP

#include <uORB/topics/cc_telemetry_estimator.h>

class MavlinkStreamCcTelemetryEstimator : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCcTelemetryEstimator(mavlink); }

	static constexpr const char *get_name_static() { return "CC_TELEMETRY_ESTIMATOR"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CC_TELEMETRY_ESTIMATOR; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_CC_TELEMETRY_ESTIMATOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCcTelemetryEstimator(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(cc_telemetry_estimator)};

	bool send() override
	{
		cc_telemetry_estimator_s data;

		if (_sub.update(&data)) {
			mavlink_cc_telemetry_estimator_t msg{};

			msg.fc_timestamp_us = data.timestamp;
			msg.sequence = data.sequence;
			msg.status_flags = data.status_flags;
			msg.innovation_check_flags = data.innovation_check_flags;
			msg.solution_status_flags = data.solution_status_flags;
			msg.velocity_test_ratio = data.velocity_test_ratio;
			msg.position_test_ratio = data.position_test_ratio;
			msg.height_test_ratio = data.height_test_ratio;
			msg.mag_test_ratio = data.mag_test_ratio;
			msg.airspeed_test_ratio = data.airspeed_test_ratio;
			msg.schema_version = data.schema_version;

			mavlink_msg_cc_telemetry_estimator_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // CC_TELEMETRY_ESTIMATOR_HPP
