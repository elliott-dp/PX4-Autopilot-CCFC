# ccfc_dialect — vendored wire contract (CCFC fork)

| File | Verbatim copy of | SHA-256 |
|---|---|---|
| `cc_dialect.xml` | `drone-companion/cc-dialect/cc_dialect.xml` | `dc5b8e9fd57504d18cd905a81282cfdcfccf3e6fd11069c4cc4b44539900e442` |
| `../src/include/ccfc/cc_dialect_hash.h` | `drone-companion/cc-dialect/generated/dialect_hash.h` | derived from the XML above (`CC_DIALECT_HASH = 0xdc5b8e9f`) |

The **single source of truth** is `drone-companion/cc-dialect/` (see its
README for the change workflow). This fork carries copies only so it can
build standalone: `src/modules/mavlink/CMakeLists.txt` copies the XML into
the mavlink submodule's `message_definitions/v1.0/` at **configure time**
(the submodule generates the C headers from it, selected via
`CONFIG_MAVLINK_DIALECT="cc_dialect"`) — so every build path works,
including CI, with no separate install step. (An earlier install.sh did
this externally; it broke CI, which never ran it.)

Guards against drift between the repos:
- the CMake step FATAL_ERRORs if the XML sha256 and `cc_dialect_hash.h`
  disagree with each other;
- the Phase 3 SITL harness asserts fork-copy sha == companion-copy sha;
- the wire itself: the harness decodes with bindings generated from the
  companion copy — CRC_EXTRA divergence fails every stream check.

Update procedure (only ever as part of a dialect change in
drone-companion): copy `cc_dialect.xml` + regenerated `dialect_hash.h`
here in the same commit, update this table's hashes, rebuild, re-run the
Phase 1 golden suite and the Phase 3 harness.
