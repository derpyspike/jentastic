#!/usr/bin/env bash
#
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Jennifer Sanchez
#
# Licensed under the GNU Affero General Public License v3.
# See LICENSE-AGPL for the full license text.
#
# Exit immediately if a command fails unexpectedly
set -o pipefail
# Nexus-28006 (2026-09-16) — code review 4.2 fix.
# Previously: script had only `set -o pipefail` but the header comment
# claimed "Exit immediately if a command fails unexpectedly". pipefail
# only catches failures in pipelines (e.g. `cmd1 | cmd2`); a single failing
# command doesn't abort. Add `set -e` to actually match the header's claim.
# Real-world impact: a failing intermediate step (e.g. node-unreachable
# ping during pre-OTA reboot dance) would silently log + continue, leaving
# the OTA partially-applied. With set -e, the script aborts at the first
# failure, leaving state clean.
set -e

# Default mode: full safety (pre-OTA reboot dance, retry resets, post-OTA sanity).
# --fast skips everything but the OTA command itself (plus the auto-detect of the
# .bin file, which we still need to know what to flash).
FAST_MODE=false

# =============================================================================
# ⚙️  HARDCODED NODE CONFIGURATION
# =============================================================================

# 1. Map node name to its IP Address
declare -A NODE_IPS
NODE_IPS[MT_FENNEKIN]="192.168.8.113"
NODE_IPS[MT_FROAKIE]="192.168.8.195"
NODE_IPS[MT_TBEAM]="192.168.8.196"
NODE_IPS[MT_V4_PRUEBAS]="192.168.8.198"
NODE_IPS[MT_V3_BAT]="192.168.8.209"
NODE_IPS[MT_V4_BAT]="192.168.8.127"

# 2. Map node name to its PlatformIO environment
declare -A NODE_PLATFORMS
NODE_PLATFORMS[MT_FENNEKIN]="heltec-v4"
NODE_PLATFORMS[MT_FROAKIE]="heltec-v3"
NODE_PLATFORMS[MT_TBEAM]="t-beam-1w"
NODE_PLATFORMS[MT_V4_PRUEBAS]="heltec-v4"
NODE_PLATFORMS[MT_V3_BAT]="heltec-v3"
NODE_PLATFORMS[MT_V4_BAT]="heltec-v4"

# ==========================================

# Helper function to find the correct non-factory bin file
# Helper function to find the correct non-factory bin file
get_bin_file() {
    local platform=$1
    local search_dir=".pio/build/${platform}"
    
    if [ ! -d "$search_dir" ]; then
        echo "❌ Error: Directory $search_dir does not exist. Did you compile the firmware first?" >&2
        return 1
    fi
    
    # Target exactly 'firmware-<platform>-*.bin' and exclude 'factory' builds
    local bin_file
    bin_file=$(find "$search_dir" -maxdepth 1 -name "firmware-${platform}-*.bin" ! -name "*factory*" -print -quit)
    
    if [ -z "$bin_file" ]; then
        echo "❌ Error: No valid update .bin file found in $search_dir matching 'firmware-${platform}-*.bin'" >&2
        return 1
    fi
    
    echo "$bin_file"
}

# Helper function to perform the update on a single node
update_node() {
    local node_var=$1
    local ip=${NODE_IPS[$node_var]}
    local platform=${NODE_PLATFORMS[$node_var]}
    
    if [ -z "$ip" ] || [ -z "$platform" ]; then
        echo "❌ Error: Missing configuration for $node_var."
        return 1
    fi
    
    echo "=================================================="
    echo "🚀 Starting OTA Update process for: $node_var"
    echo "📡 IP: $ip"
    echo "⚙️  Platform: $platform"
    if [ "$FAST_MODE" = true ]; then
        echo "⚡ FAST MODE: skipping pre-OTA reboots and post-OTA sanity checks"
    fi
    echo "=================================================="
    
    # Auto-detect bin file
    local bin_file
    bin_file=$(get_bin_file "$platform") || return 1
    echo "📦 Target Firmware: $bin_file"
    
    # ------------------ THE REBOOT DANCE ------------------
    if [ "$FAST_MODE" = true ]; then
        echo "⚡ FAST MODE: skipping pre-OTA reboot dance."
    else
        echo "🔄 Initiating the double-reboot dance to stabilize the node..."
        
        echo "👉 Reboot 1/2..."
        meshtastic --host "$ip" --reboot
        echo "⏳ Waiting 45 seconds..."
        sleep 45
        
        echo "👉 Reboot 2/2..."
        meshtastic --host "$ip" --reboot
        echo "⏳ Waiting 45 seconds..."
        sleep 45
    fi
    
    # ------------------ THE OTA PROCESS (WITH RETRIES) ------------------
    local max_attempts=3
    local attempt=1
    local success=false
    
    while [ $attempt -le $max_attempts ]; do
        echo "⚡ OTA Attempt $attempt of $max_attempts..."
        
        # Run the meshtastic OTA command
        if meshtastic --host "$ip" --ota-update "$bin_file"; then
            echo "✅ meshtastic command reported success!"
            success=true
            break
        else
            echo "⚠️ OTA attempt $attempt failed."
            if [ $attempt -lt $max_attempts ]; then
                if [ "$FAST_MODE" = true ]; then
                    echo "⚡ FAST MODE: skipping inter-attempt reboot; retrying immediately..."
                else
                    echo "⏳ Waiting 30 seconds for node to settle, then performing an emergency reboot before retrying..."
                    sleep 30
                    meshtastic --host "$ip" --reboot
                    sleep 45
                fi
            fi
        fi
        attempt=$((attempt + 1))
    done
    
    if [ "$success" = false ]; then
        echo "❌ Major Failure: OTA update failed after $max_attempts attempts for $node_var."
        return 1
    fi
    
    # ------------------ SANITY CHECK ------------------
    if [ "$FAST_MODE" = true ]; then
        echo "⚡ FAST MODE: skipping post-OTA wait + sanity checks."
    else
        echo "⏳ Waiting 60 seconds for the node to reboot and establish network connection..."
        sleep 60
        
        echo "🔍 Performing sanity checks on $node_var..."
        if ping -c 1 -W 5 "$ip" > /dev/null 2>&1; then
            echo "📶 Ping check: SUCCESS (Node is online!)"
            
            # Try to query the node API to make sure the app layer is running
            if meshtastic --host "$ip" --info > /dev/null 2>&1; then
                echo "🎉 API check: SUCCESS! Node responded to query. Update completed flawlessly! 🥳"
            else
                echo "⚠️ API check: WARNING. Node pings but is not responding to Meshtastic API. It might still be booting up or in a weird state."
            fi
        else
            echo "❌ Ping check: FAILED! Node is not responding to ping. It might be bootlooping or disconnected."
            return 1
        fi
    fi
    
    echo ""
}

# --- Main CLI Argument Parsing ---
usage() {
    echo "Usage: $0 [--fast] <node_name_or_all>"
    echo "Examples:"
    echo "  $0 MT_FENNEKIN          -> Updates only Fennekin (full reboot dance + sanity checks)"
    echo "  $0 --fast MT_FENNEKIN   -> Updates only Fennekin (skip reboots + sanity)"
    echo "  $0 all                  -> Updates ALL configured nodes"
    echo "  $0 --fast all           -> Updates ALL configured nodes in fast mode"
    echo ""
    echo "--fast skips everything but the OTA command itself: no pre-OTA double reboot,"
    echo "no inter-attempt reboot, no post-OTA wait / ping / API check."
    echo ""
    echo "Available nodes:"
    for key in "${!NODE_IPS[@]}"; do
        echo "  - $key (IP: ${NODE_IPS[$key]}, Platform: ${NODE_PLATFORMS[$key]})"
    done
    exit 1
}

# Parse args: --fast may appear anywhere; first non-flag arg is the target.
TARGET=""
for arg in "$@"; do
    if [ "$arg" = "--fast" ]; then
        FAST_MODE=true
    elif [ "$arg" = "-h" ] || [ "$arg" = "--help" ]; then
        usage
    elif [ -z "$TARGET" ]; then
        TARGET="$arg"
    else
        echo "❌ Error: unexpected argument '$arg'"
        usage
    fi
done

if [ -z "$TARGET" ]; then
    usage
fi

if [ "$TARGET" = "all" ]; then
    echo "🌟 Running bulk OTA updates for ALL configured nodes! 🌟"
    # To run sequentially and safely:
    for node in "${!NODE_IPS[@]}"; do
        update_node "$node"
    done
else
    # Check if target exists in our map
    if [[ -z "${NODE_IPS[$TARGET]}" ]]; then
        echo "❌ Error: '$TARGET' is not a configured node."
        usage
    fi
    update_node "$TARGET"
fi
