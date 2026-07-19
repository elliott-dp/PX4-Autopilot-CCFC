/****************************************************************************
 * CCFC fork — CC_TELEMETRY_IMU stream (Class B, spec §6). 1:1 copy of the
 * cc_telemetry_imu uORB topic.
 ****************************************************************************/

#ifndef CC_TELEMETRY_IMU_HPP
#define CC_TELEMETRY_IMU_HPP

#include <uORB/topics/cc_telemetry_imu.h>

class MavlinkStreamCcTelemetryImu : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCcTelemetryImu(mavlink); }

	static constexpr const char *get_name_static() { return "CC_TELEMETRY_IMU"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CC_TELEMETRY_IMU; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_CC_TELEMETRY_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCcTelemetryImu(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(cc_telemetry_imu)};

	bool send() override
	{
		cc_telemetry_imu_s data;

		if (_sub.update(&data)) {
			mavlink_cc_telemetry_imu_t msg{};

			msg.fc_timestamp_us = data.timestamp;
			msg.sequence = data.sequence;
			msg.clipping_count = data.clipping_count;
			memcpy(msg.accel, data.accel, sizeof(msg.accel));
			memcpy(msg.gyro, data.gyro, sizeof(msg.gyro));
			memcpy(msg.delta_angle, data.delta_angle, sizeof(msg.delta_angle));
			memcpy(msg.delta_velocity, data.delta_velocity, sizeof(msg.delta_velocity));
			memcpy(msg.vibration_metric, data.vibration_metric, sizeof(msg.vibration_metric));
			msg.temperature = data.temperature;
			msg.schema_version = data.schema_version;

			mavlink_msg_cc_telemetry_imu_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // CC_TELEMETRY_IMU_HPP
