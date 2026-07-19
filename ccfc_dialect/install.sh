#!/usr/bin/env bash
# ============================================================================
# CCFC fork — install the vendored cc_dialect.xml into the mavlink submodule
#
# PX4 generates its MAVLink C headers at build time from
#   src/modules/mavlink/mavlink/message_definitions/v1.0/${CONFIG_MAVLINK_DIALECT}.xml
# The submodule cannot carry our XML in git (that would mean forking the
# mavlink repo too), so this fork vendors the file here and this script
# copies it in before building. drone-companion/tools/phase2/build_px4.sh
# runs it automatically; run it manually after a fresh clone otherwise.
#
# It also verifies the two committed artifacts agree with each other:
# the XML's SHA-256 must match CC_DIALECT_SHA256 in
# src/include/ccfc/cc_dialect_hash.h (the constant the receiver compiles in
# and checks against CC_MISSION_CONTEXT.dialect_hash). If this fails, one
# of the copies is stale — re-vendor BOTH from drone-companion/cc-dialect
# in one commit (single source of truth lives there).
# ============================================================================
set -euo pipefail

FORK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
XML="$FORK_ROOT/ccfc_dialect/cc_dialect.xml"
HASH_H="$FORK_ROOT/src/include/ccfc/cc_dialect_hash.h"
DEST_DIR="$FORK_ROOT/src/modules/mavlink/mavlink/message_definitions/v1.0"

[[ -f "$XML" ]] || { echo "error: $XML missing" >&2; exit 1; }
[[ -d "$DEST_DIR" ]] || { echo "error: mavlink submodule not initialized ($DEST_DIR)" >&2; exit 1; }

if command -v shasum >/dev/null 2>&1; then
    SHA="$(shasum -a 256 "$XML" | awk '{print $1}')"
else
    SHA="$(sha256sum "$XML" | awk '{print $1}')"
fi

grep -q "$SHA" "$HASH_H" || {
    echo "error: ccfc_dialect/cc_dialect.xml (sha $SHA) does not match" >&2
    echo "       src/include/ccfc/cc_dialect_hash.h — stale vendored copy!" >&2
    echo "       Re-vendor both from drone-companion/cc-dialect." >&2
    exit 1
}

cp "$XML" "$DEST_DIR/cc_dialect.xml"
echo "[ccfc_dialect] installed cc_dialect.xml (sha ${SHA:0:8}) -> mavlink submodule"
