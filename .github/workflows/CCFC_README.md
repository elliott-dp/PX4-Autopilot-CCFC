# CCFC fork — workflow curation

This fork keeps only the upstream workflows that (a) can run on a plain
GitHub-hosted fork (no RunsOn self-hosted runners, no upstream secrets) and
(b) guard the CCFC contract:

| Kept | Why |
|---|---|
| `checks.yml` | format/newlines/tests/sitl+fmu-v5 builds (allyes removed — see in-file comment) |
| `clang-tidy.yml` | static analysis of the SITL build |
| `ekf_functional_change_indicator.yml`, `ekf_update_change_indicator.yml` | estimator regression guards |
| `failsafe_sim.yml` | failsafe simulator build |
| `nuttx_env_config.yml` | NuttX target build (flash guard for the Cc*.msg additions) |
| `python_checks.yml` | msg/tooling lint |

Everything else (20 workflows: RunsOn-runner jobs, docs/deploy pipelines,
container publishing, repo bots, ROS/mavros integration suites) was deleted
on 2026-07-19 after every fork push left them failed or queued forever —
they need PX4-org runners/secrets. Restore any of them from upstream if the
fork ever gains that infrastructure. Diagnosis notes:
drone-companion `docs/phase4/phase4_companiond.md` (CI section).
