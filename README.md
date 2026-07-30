# PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![DOI](https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot)

[![Build Targets](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml/badge.svg?branch=main)](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml) [![SITL Tests](https://github.com/PX4/PX4-Autopilot/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22SITL+Tests%22)

[![Discord Shield](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

---

## 🔱 This is the CCFC fork

This fork adds the flight-controller half of a **companion-computer safety
architecture**: a CUAV V6X running this firmware talks to a Jetson Orin Nano
over a custom MAVLink 2 dialect, so the companion computer can observe,
record, and *recommend* — while this firmware remains the only thing that
ever commands the vehicle. The companion side (Rust) lives in
**[drone-companion](https://github.com/elliott-dp/drone-companion)** — that
repo's README has the full two-sided architecture and design rationale;
this section documents only what changed *here*.

**Base:** pinned to upstream PX4 release **`v1.17.0`** (commit
[`d6f12ad1c4`](https://github.com/elliott-dp/PX4-Autopilot-CCFC/commit/d6f12ad1c4)),
never `main` — a flight-safety-relevant fork tracks a released, stable tag on
purpose. Every fork change is additive and marked; no upstream flight-control
logic is modified.

### The one rule this fork exists to enforce

> The companion computer can *recommend* (hold position, land, return home).
> It can never command an actuator, and it can never hold state PX4 needs to
> fly. If the companion crashes, reboots, or sends nonsense, PX4 keeps flying
> exactly as it would with no companion attached at all.

### What was added, phase by phase

| Phase | Module / files | What it does |
|---|---|---|
| **2** | `src/modules/cc_telemetry_publisher/` + 8 new `msg/Cc*.msg` uORB topics | Curates six telemetry streams (state, IMU, power, GPS, estimator, actuator) off existing uORB topics at fixed, independently-configurable rates. Runs as a `ScheduledWorkItem` on the low-priority work queue — no heap allocation after init, missing sources marked NaN/invalid rather than fabricated. |
| **3** | `ccfc_dialect/` (vendored `cc_dialect.xml`) + 8 MAVLink stream classes in `src/modules/mavlink/streams/` + receiver validation in `mavlink_receiver` | The companion dialect crosses the link both ways. Every inbound `CC_*` message runs a normative gauntlet — source check, schema check, range check, sequence-gap accounting, flood cap — **before** anything is published to uORB, with per-check counters visible in `mavlink status`. |
| **4** | `px4-rc.mavlink` | The companion UDP/serial instance is switched to a dedicated *custom* MAVLink mode carrying only the `CC_*` contract + heartbeat — no default-dialect stream pollution, no PX4-side `TIMESYNC` requests (the companion drives time sync itself). |
| **6** | `src/modules/cc_safety_monitor/` | The actual safety decision core. Two **pure, PX4-free C++ headers** (`cc_policy_table.hpp`, `cc_state_machine.hpp`) decide the vehicle's response to a companion health report; the module wraps them, publishes `cc_safety_status`, and issues one conservative `Hold`/`Land`/`RTL`/exit-Offboard command per state transition. The module is a thin, untested-by-necessity shell around logic that *is* exhaustively tested — see below. |

### New parameters

| Param | Default | Controls |
|---|---|---|
| `CC_MON_EN` | `1` | Enable `cc_safety_monitor` |
| `CC_MON_REQ_OFFB` | `1` | Require a healthy companion link before Offboard is allowed |
| `CC_MON_TMOUT_MS` | `3000` | Health-report staleness timeout |
| `CC_MON_OK_COUNT` | `3` | Consecutive OK reports required before de-escalating |
| `CC_MON_CRIT_ACT` | `1` | Action on a CRITICAL report (warn-only / Hold / Land / RTL) |
| `CC_MON_STALE_ACT` | `1` | Action when the companion link goes stale |
| `CC_TEL_PROFILE` | `1` | Telemetry rate profile |
| `CC_TEL_IMU_RATE` | `50` Hz | IMU stream rate |
| `CC_TEL_ACT_RATE` | `20` Hz | Actuator stream rate |
| `CC_VEHICLE_ID` | `1` | Vehicle identity gate for `CC_MISSION_CONTEXT` |

### Test evidence

- **`cc_policy_table.hpp` / `cc_state_machine.hpp`: 40/40, host-run, no PX4
  build required** — the safety decision core is pure C++ with no PX4
  dependency, so it's tested with a one-line host compile:
  `c++ -std=c++14 -I. cc_policy_table_test.cpp -o t && ./t`. One case per
  policy-table row plus hysteresis, staleness, reboot, and an exhaustive
  sweep asserting every possible output is conservative (no path exists to
  arm, take off, or enter Offboard from this module).
- **Phase 2 SITL harness: 37/37** and **Phase 3 SITL harness: 50/50** —
  headless SIH-SITL checks against real MAVLink/uORB traffic, driven by the
  `drone-companion` repo's `tools/phase2` and `tools/phase3` harnesses.
- **CI**: curated to the 7 workflows that can actually run on a plain forked
  repo without PX4-org secrets or self-hosted runners — `checks.yml`
  (format, tests, SITL + fmu-v5 builds), `clang-tidy.yml`,
  the two EKF regression-change indicators, `failsafe_sim.yml`,
  `nuttx_env_config.yml`, `python_checks.yml`. Rationale for every kept and
  dropped workflow: [`.github/workflows/CCFC_README.md`](.github/workflows/CCFC_README.md).

### Dialect integrity

The custom dialect (`ccfc_dialect/cc_dialect.xml`) is vendored from
`drone-companion`'s `cc-dialect/` — the single source of truth stays there.
A build-time SHA-256 gate in `src/modules/mavlink/CMakeLists.txt` refuses to
build if the vendored XML doesn't match the compiled-in hash constant
(`src/include/ccfc/cc_dialect_hash.h`), so a stale copy fails loudly at
configure time instead of silently decoding traffic wrong.

### Board targets

- `px4_sitl_default` — the development and CI target; every phase above is
  proven here first.
- `cuav_fmu-v6x` — the real hardware target (CUAV V6X), for the upcoming
  bench hardware-in-the-loop phase.

---

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

PX4 is highly portable, OS-independent and supports Linux, NuttX and MacOS out of the box.

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE))
* [Supported airframes](https://docs.px4.io/main/en/airframes/airframe_reference.html) ([portfolio](https://px4.io/ecosystem/commercial-systems/)):
  * [Multicopters](https://docs.px4.io/main/en/frames_multicopter/)
  * [Fixed wing](https://docs.px4.io/main/en/frames_plane/)
  * [VTOL](https://docs.px4.io/main/en/frames_vtol/)
  * [Autogyro](https://docs.px4.io/main/en/frames_autogyro/)
  * [Rover](https://docs.px4.io/main/en/frames_rover/)
  * many more experimental types (Blimps, Boats, Submarines, High Altitude Balloons, Spacecraft, etc)
* Releases: [Downloads](https://github.com/PX4/PX4-Autopilot/releases)

## Releases

Release notes and supporting information for PX4 releases can be found on the [Developer Guide](https://docs.px4.io/main/en/releases/).

## Building a PX4 based drone, rover, boat or robot

The [PX4 User Guide](https://docs.px4.io/main/en/) explains how to assemble [supported vehicles](https://docs.px4.io/main/en/airframes/airframe_reference.html) and fly drones with PX4. See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Changing Code and Contributing

This [Developer Guide](https://docs.px4.io/main/en/development/development.html) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://docs.px4.io/main/en/contribute/).
See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://docs.px4.io/main/en/contribute/).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

See the latest list of maintainers on [MAINTAINERS](MAINTAINERS.md) file at the root of the project.

For the latest stats on contributors please see the latest stats for the Dronecode ecosystem in our project dashboard under [LFX Insights](https://insights.lfx.linuxfoundation.org/foundation/dronecode). For information on how to update your profile and affiliations please see the following support link on how to [Complete Your LFX Profile](https://docs.linuxfoundation.org/lfx/my-profile/complete-your-lfx-profile). Dronecode publishes a yearly snapshot of contributions and achievements on its [website under the Reports section](https://dronecode.org).

## Supported Hardware

For the most up to date information, please visit [PX4 User Guide > Autopilot Hardware](https://docs.px4.io/main/en/flight_controller/).

## Project Governance

The PX4 Autopilot project including all of its trademarks is hosted under [Dronecode](https://www.dronecode.org/), part of the Linux Foundation.

<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://dronecode.org/wp-content/uploads/sites/24/2020/08/dronecode_logo_default-1.png" alt="Dronecode Logo" width="110px"/></a>
<div style="padding:10px">&nbsp;</div>
