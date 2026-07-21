############################################################################
#
# CCFC fork — cc_safety_monitor (FC<->CC architecture spec §4.5)
#
# The deterministic companion safety policy core. The decision logic lives in
# two PURE, PX4-free headers (cc_policy_table.hpp, cc_state_machine.hpp) that
# are exhaustively host-unit-tested in cc_policy_table_test.cpp (built and run
# outside the PX4 tree — see tools/phase6).
#
############################################################################

px4_add_module(
	MODULE modules__cc_safety_monitor
	MAIN cc_safety_monitor
	COMPILE_FLAGS
	SRCS
		CcSafetyMonitor.cpp
		CcSafetyMonitor.hpp
		cc_policy_table.hpp
		cc_state_machine.hpp
	DEPENDS
		px4_work_queue
)
