# CCFC: validated session handshake from the companion (spec §7).
# Published by the extended mavlink_receiver after the gauntlet accepts a
# CC_MISSION_CONTEXT (vehicle_id matches CC_VEHICLE_ID, dialect_hash matches
# the FC build's constant). Consumed by cc_telemetry_publisher (mission_id
# echo into CC_TELEMETRY_STATE) and by the logger as mission identity.
# NOTE: uORB handoff topic added beyond the spec §4.1 list — deviation D9
# in docs/phase3/phase3_mavlink_link.md (spec §4.4 requires the handoff but names
# no topic).

uint64 timestamp        # time since system start (microseconds); receive time

uint32 mission_id       # mission identity minted by the companion
uint32 cc_boot_id       # companiond process boot identity
uint32 vehicle_id       # validated static vehicle identity
uint32 dialect_hash     # validated dialect hash (== FC build's CC_DIALECT_HASH)

char[24] sw_version     # companion software version (NUL padded)

uint8 schema_version    # payload schema version
