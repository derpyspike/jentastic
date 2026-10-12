#!/usr/bin/env bash
#
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Jennifer Sanchez
#
# Licensed under the GNU Affero General Public License v3.
# See LICENSE-AGPL for the full license text.
#
# =============================================================================
# Nexus Master Keys Library
# -----------------------------------------------------------------------------
# Sourceable bash library containing the canonical registry of Nexus Master
# Keys (mirrored from NEXUS_KEYS.md). Edit this file to add new keys; the
# builder will pick them up automatically by symbolic name.
#
# Usage:
#   source nexus-keys-lib.sh
#   echo "${NEXUS_KEYS[eevee]}"
# =============================================================================

# Prevent multiple sourcing
if [[ -n "${__NEXUS_KEYS_LIB_LOADED:-}" ]]; then
    return 0
fi
__NEXUS_KEYS_LIB_LOADED=1

# -----------------------------------------------------------------------------
# Master Key Registry
# -----------------------------------------------------------------------------
# Format: NEXUS_KEYS["<symbolic_name>"]="<C++ hex array>"
#
# IMPORTANT: Keep these in sync with NEXUS_KEYS.md. The current firmware
# (NexusModule.cpp) accepts up to 4 hardcoded master keys via the
# NEXUS_MASTER_KEY_1..4 defines.
# -----------------------------------------------------------------------------
declare -A NEXUS_KEYS

NEXUS_KEYS["eevee"]="{ 0x9a, 0xd2, 0x47, 0xb0, 0x2f, 0x4f, 0xa5, 0x08, 0x95, 0x3f, 0x5c, 0x1b, 0x6e, 0x13, 0x93, 0x98, 0x2c, 0xb3, 0xd6, 0xf3, 0x19, 0x3b, 0x87, 0xa5, 0xc8, 0x33, 0x2d, 0xce, 0x67, 0xae, 0x9d, 0x15 }"

NEXUS_KEYS["bloom"]="{ 0x6c, 0x5c, 0xa6, 0xe8, 0xfa, 0x3f, 0xe9, 0x46, 0x19, 0x6b, 0x82, 0x14, 0xa7, 0xe2, 0xd9, 0x96, 0xed, 0x6a, 0xe1, 0x10, 0xe4, 0xe5, 0xf3, 0xe8, 0xc6, 0x1b, 0xfb, 0xc4, 0x9e, 0x8b, 0x8f, 0x61 }"

NEXUS_KEYS["mew"]="{ 0x11, 0x0c, 0x24, 0x41, 0x08, 0x40, 0xd7, 0xca, 0xec, 0x9f, 0xb8, 0xf8, 0x71, 0x99, 0xcb, 0xcc, 0xb6, 0xac, 0xd9, 0x56, 0x77, 0x14, 0x9b, 0x3e, 0xee, 0xf2, 0x9c, 0x4c, 0xf5, 0x08, 0x12, 0x60 }"

NEXUS_KEYS["fennekin"]="{ 0xe7, 0x05, 0x7e, 0xe7, 0xeb, 0x80, 0xf9, 0x04, 0x0d, 0xf2, 0x30, 0xff, 0x0c, 0x4a, 0x09, 0x74, 0x94, 0xb9, 0x47, 0x92, 0xa0, 0x65, 0x78, 0xcb, 0xfd, 0xeb, 0xdc, 0xda, 0x57, 0x73, 0x26, 0x7e }"

NEXUS_KEYS["uxie"]="{ 0xd2, 0xea, 0x51, 0x13, 0x56, 0x7e, 0xd9, 0xdb, 0xdc, 0x1b, 0x0a, 0xb8, 0xa3, 0x3f, 0xf7, 0xfd, 0x35, 0x0b, 0xca, 0x8e, 0x2b, 0xb3, 0x68, 0x96, 0xe4, 0xd9, 0xd3, 0x3f, 0x45, 0x80, 0x85, 0x74 }"

# -----------------------------------------------------------------------------
# Default key set (matches the 4 keys previously hardcoded in platformio.ini)
# -----------------------------------------------------------------------------
NEXUS_DEFAULT_KEYS=("eevee" "bloom" "mew" "fennekin")

# Maximum number of master keys the firmware accepts (4 slots in
# NexusModule.cpp: NEXUS_MASTER_KEY_1..4)
NEXUS_MAX_KEYS=4

# -----------------------------------------------------------------------------
# Helper: resolve a list of symbolic names to hex arrays, in order
# -----------------------------------------------------------------------------
# Args: <output_array_name> <key1> [key2] [key3] [key4]
# Returns: populates the named array with resolved hex values, errors on
#          unknown names or excessive count.
# -----------------------------------------------------------------------------
nexus_resolve_keys() {
    local out_name=$1
    shift
    local -a resolved=()

    if [[ $# -gt $NEXUS_MAX_KEYS ]]; then
        echo "❌ Error: too many keys requested ($#), max is $NEXUS_MAX_KEYS" >&2
        return 1
    fi

    for key in "$@"; do
        if [[ -z "${NEXUS_KEYS[$key]:-}" ]]; then
            echo "❌ Error: unknown master key '$key'. Available:" >&2
            for k in "${!NEXUS_KEYS[@]}"; do
                echo "    - $k" >&2
            done
            return 1
        fi
        resolved+=("${NEXUS_KEYS[$key]}")
    done

    # Use nameref to populate caller's array
    local -n out_ref=$out_name
    out_ref=("${resolved[@]}")
    return 0
}

# -----------------------------------------------------------------------------
# Helper: build PLATFORMIO_BUILD_FLAGS fragments for the given keys
# -----------------------------------------------------------------------------
# Args: <output_var_name> <key1> [key2] [key3] [key4]
# Returns: sets the named variable to a string with -DNEXUS_MASTER_KEY_N="..."
#          entries ready to be concatenated into PLATFORMIO_BUILD_FLAGS.
# -----------------------------------------------------------------------------
nexus_build_key_flags() {
    local out_var=$1
    shift

    if [[ $# -gt $NEXUS_MAX_KEYS ]]; then
        echo "❌ Error: too many keys requested ($#), max is $NEXUS_MAX_KEYS" >&2
        return 1
    fi

    local flags=""
    local i=1
    for key in "$@"; do
        if [[ -z "${NEXUS_KEYS[$key]:-}" ]]; then
            echo "❌ Error: unknown master key '$key'. Available:" >&2
            for k in "${!NEXUS_KEYS[@]}"; do
                echo "    - $k" >&2
            done
            return 1
        fi
        local hex="${NEXUS_KEYS[$key]}"
        flags+=" -DNEXUS_MASTER_KEY_${i}=\"${hex}\""
        i=$((i + 1))
    done

    printf -v "$out_var" '%s' "$flags"
    return 0
}

# -----------------------------------------------------------------------------
# Helper: list all available key names
# -----------------------------------------------------------------------------
nexus_list_keys() {
    echo "Available master keys (edit nexus-keys-lib.sh to add more):"
    for k in "${!NEXUS_KEYS[@]}"; do
        local marker=""
        for d in "${NEXUS_DEFAULT_KEYS[@]}"; do
            if [[ "$k" == "$d" ]]; then
                marker=" (default)"
                break
            fi
        done
        echo "  - $k${marker}"
    done
}
