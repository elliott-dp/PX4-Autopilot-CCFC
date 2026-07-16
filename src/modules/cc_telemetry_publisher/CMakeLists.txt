############################################################################
#
# CCFC fork — cc_telemetry_publisher (FC<->CC architecture spec §4.2)
#
############################################################################

px4_add_module(
	MODULE modules__cc_telemetry_publisher
	MAIN cc_telemetry_publisher
	COMPILE_FLAGS
	SRCS
		CcTelemetryPublisher.cpp
		CcTelemetryPublisher.hpp
	DEPENDS
		px4_work_queue
)
