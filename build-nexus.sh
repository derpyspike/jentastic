#!/usr/bin/env bash
#
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Jennifer Sanchez
#
# Licensed under the GNU Affero General Public License v3.
# See LICENSE-AGPL for the full license text.
#
# =============================================================================
# Nexus Combined Builder (v2.0)
# -----------------------------------------------------------------------------
# Independent builder that supersedes the legacy build-jenny.sh / build-javi.sh
# / build-admins.sh workflow. Supports parameterized master key injection,
# per-builder defaults, and organized output under outbuilds/.
#
# Usage:
#   ./build-nexus.sh jenny                           # 4 default keys + jenny-firmware
#   ./build-nexus.sh admins                          # NO keys + jt-firmware
#   ./build-nexus.sh javi                            # NO keys + nexus-firmware
#   ./build-nexus.sh default                         # NO keys + jt-firmware (same as admins for now)
#   ./build-nexus.sh all                             # jenny + admins + javi + default sequentially
#   ./build-nexus.sh jenny --no-default-keys         # jenny without keys
#   ./build-nexus.sh admins --default-keys           # admins WITH the 4 default keys
#   ./build-nexus.sh jenny --master-key-1 eevee \
#                      --master-key-2 mew            # custom key set
#   ./build-nexus.sh jenny --targets heltec-v4       # only build heltec-v4
#   ./build-nexus.sh jenny --dry-run                 # show what would be built
#   ./build-nexus.sh jenny --list-keys               # list available master keys
#
# Output:
#   outbuilds/<builder>-<app_repo>/firmware-<target>-*
#
# Notes:
#   - This script does NOT modify platformio.ini. Master keys are injected via
#     PLATFORMIO_BUILD_FLAGS at compile time.
#   - The legacy build-jenny.sh / build-javi.sh / build-admins.sh scripts are
#     left untouched. This is an independent alternative.
# =============================================================================

set -euo pipefail

# -----------------------------------------------------------------------------
# Paths
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

LIB_FILE="$SCRIPT_DIR/nexus-keys-lib.sh"
OUT_ROOT="$SCRIPT_DIR/outbuilds"

# -----------------------------------------------------------------------------
# Source the keys library
# -----------------------------------------------------------------------------
if [[ ! -f "$LIB_FILE" ]]; then
    echo "❌ Error: keys library not found at $LIB_FILE" >&2
    exit 1
fi
# shellcheck source=nexus-keys-lib.sh
source "$LIB_FILE"

# -----------------------------------------------------------------------------
# Per-builder defaults
# -----------------------------------------------------------------------------
# Format: builder_name|app_repo|default_key_mode|trusted_ch1|trusted_ch2|default_evil|default_private
#   key_mode = "default" | "none"
#   trusted_ch1/ch2 = channel name as string literal (will be quoted + escaped)
#   default_evil = "yes" | "no" (whether -DNEXUS_ENABLE_ADV is on by default; override with --evil/--no-evil)
#   default_private = "yes" | "no" (whether -DNEXUS_PRIVATE_BUILD is on by default; override with --private-build/--no-private-build)
# -----------------------------------------------------------------------------
BUILDER_JENNY="jenny|jenny-firmware|default|Jenny|Jenny2|yes|no"
BUILDER_ADMINS="admins|jt-firmware|none|Nexus|JenT|no|no"           # Nexus-28006 (2026-09-16): APP_REPO for admins is `jt-firmware` (short for jentastic-firmware), NOT `jentastic-firmware`.
BUILDER_JAVI="javi|nexus-firmware|none|Nexus|Nexus2|yes|no"
BUILDER_DEFAULT="default|jt-firmware|none|Nexus|JenT|no|no"        # Placeholder variant: mirrors admins until it gets its own defaults.

# -----------------------------------------------------------------------------
# Default target lists per builder (mirrors legacy scripts)
# -----------------------------------------------------------------------------
TARGETS_JENNY="heltec-v3 heltec-v4 station-g2 station-g3 tracker-t1000-e rak_wismeshtag seeed-xiao-s3 heltec-vision-master-e290-inkhud seeed_wio_tracker_L1 seeed_xiao_nrf52840_kit seeed_mesh_tracker_X1 rak3401-1watt nrf52_promicro_diy_tcxo nrf52_promicro_diy_sx1262"
TARGETS_ADMINS="nrf52_promicro_diy_tcxo nrf52_promicro_diy_sx1262-noscreen xiaowa-esp32c3 t-deck-tft heltec-v4 seeed_solar_node rak_wismeshtag tracker-t1000-e seeed_mesh_tracker_X1 rak3401-1watt"
TARGETS_JAVI="nrf52_promicro_diy_tcxo nrf52_promicro_diy_sx1262-noscreen station-g2 rak3401-1watt"
TARGETS_DEFAULT="$TARGETS_ADMINS"

# -----------------------------------------------------------------------------
# State
# -----------------------------------------------------------------------------
REQUESTED_BUILDER=""
EXTRA_FLAGS=""
DRY_RUN=false
TARGETS_OVERRIDE=""
# EVIL: 0 = explicit no, 1 = explicit yes, -1 = use builder default
EVIL_OVERRIDE=-1
# PRIVATE: 0 = explicit no, 1 = explicit yes, -1 = use builder default
PRIVATE_OVERRIDE=-1
# JENCORE / RNODE / STATSBOT / REPLYBOT master gates: 0 = explicit no, 1 = explicit yes, -1 = default (off,
# since 2026-10-24 they are disabled by default; no builder-table defaults)
JENCORE_OVERRIDE=-1
RNODE_OVERRIDE=-1
STATSBOT_OVERRIDE=-1
REPLYBOT_OVERRIDE=-1
# Extra PIO target (e.g. "upload", "monitor"); appended to "pio run -e $target"
PIO_EXTRA_TARGET=""

# Per-build key overrides (filled by CLI parsing)
USER_KEYS=()

# -----------------------------------------------------------------------------
# Usage
# -----------------------------------------------------------------------------
usage() {
    cat <<EOF
Usage: $0 <builder> [options]

Builders:
  jenny       jenny-firmware repo, 4 default keys (Eevee, Bloom, Mew, Fennekin)
  admins      jt-firmware repo, NO keys (unless --default-keys)
  javi        nexus-firmware repo, NO keys (unless --default-keys)
  default     jt-firmware repo, NO keys (unless --default-keys); same config as admins for now
  all         Build all four builders sequentially

Options:
  --default-keys              Force the 4 default keys for builders that default to none.
  --no-default-keys           Suppress default keys for builders that default to having them.
  --master-key-N <name>       Inject a specific master key into slot N (1..4).
                              Up to 4 --master-key-N flags accepted, in order.
                              N may be omitted or out of order; slots fill by call order.
                              Example: --master-key-1 eevee --master-key-3 mew
                              (slot 2 is left empty, the firmware will not see a key there)
  --targets <list>            Override the builder's target list (space-separated).
  --extra-flag "<pio-flag>"   Append extra PLATFORMIO_BUILD_FLAGS string.
  --evil                      Force -DNEXUS_ENABLE_ADV (overrides builder default).
  --no-evil                   Disable -DNEXUS_ENABLE_ADV (overrides builder default).
                              Builder defaults: jenny=ON, javi=ON, admins=OFF, default=OFF.
  --private-build             Force -DNEXUS_PRIVATE_BUILD (overrides builder default).
                              Required for private-side commands (see NexusModule_private.*).
  --no-private-build          Disable -DNEXUS_PRIVATE_BUILD (overrides builder default).
                              Builder defaults: jenny=OFF, admins=OFF, javi=OFF, default=OFF.
  --jencore                   Enable the JenCore bridge (-DNEXUS_ENABLE_JENCORE=1).
                              Disabled by default since 2026-10-24.
  --no-jencore                Explicitly keep JenCore disabled (default; symmetry flag).
  --rnode                     Enable the Reticulum RNode bridge (-DNEXUS_ENABLE_RNODE=1).
                              Disabled by default since 2026-10-24.
  --no-rnode                  Explicitly keep RNode disabled (default; symmetry flag).
  --statsbot                  Enable StatsCommandModule (-DNEXUS_ENABLE_STATSBOT=1).
                              Disabled by default; requires --jencore (or -DNEXUS_ENABLE_JENCORE_SNIFFER=1).
  --no-statsbot               Explicitly keep StatsCommandModule disabled (default).
  --replybot                  Enable ReplyBotModule (-DNEXUS_ENABLE_REPLYBOT=1).
                              Disabled by default.
  --no-replybot               Explicitly keep ReplyBotModule disabled (default).
  -t <pio-target>             Append an extra PlatformIO target to "pio run" (e.g. "upload", "monitor").
                              Example: -t upload -> pio run -e t-deck-tft -t upload
  --dry-run                   Print what would be built without invoking pio.
  --list-keys                 List available master keys and exit.
  -h, --help                  Show this help.

Output:
  Built firmware files are copied to:
    $OUT_ROOT/<builder>-<app_repo>/firmware-<target>-*

Examples:
  $0 jenny
  $0 admins --default-keys
  $0 jenny --master-key-1 eevee --master-key-2 mew
  $0 all --dry-run
  $0 default --targets heltec-v4
  $0 javi --targets station-g2 --no-default-keys
  $0 jenny --targets t-deck-tft -t upload   # build + upload to device
  $0 admins --no-evil                       # explicit safety build
  $0 jenny --private-build                 # enable private features
  $0 javi --targets tracker-t1000-e --jencore --rnode   # default-off bridges enabled
EOF
}

# -----------------------------------------------------------------------------
# Parse CLI
# -----------------------------------------------------------------------------
parse_args() {
    if [[ $# -eq 0 ]]; then
        usage
        exit 1
    fi

    # First positional is the builder name
    REQUESTED_BUILDER="$1"
    shift

    if [[ "$REQUESTED_BUILDER" == "-h" || "$REQUESTED_BUILDER" == "--help" ]]; then
        usage
        exit 0
    fi

    if [[ "$REQUESTED_BUILDER" == "--list-keys" ]]; then
        nexus_list_keys
        exit 0
    fi

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --default-keys)
                FORCE_DEFAULT_KEYS=1
                ;;
            --no-default-keys)
                FORCE_NO_KEYS=1
                ;;
            --master-key-1|--master-key-2|--master-key-3|--master-key-4)
                local slot="${1##*--master-key-}"
                if [[ -z "${2:-}" ]]; then
                    echo "❌ Error: $1 requires a key name argument" >&2
                    exit 1
                fi
                # Store as "slot:name"
                USER_KEYS+=("$slot:$2")
                shift
                ;;
            --targets)
                if [[ -z "${2:-}" ]]; then
                    echo "❌ Error: --targets requires a space-separated list" >&2
                    exit 1
                fi
                TARGETS_OVERRIDE="$2"
                shift
                ;;
            --extra-flag)
                if [[ -z "${2:-}" ]]; then
                    echo "❌ Error: --extra-flag requires an argument" >&2
                    exit 1
                fi
                EXTRA_FLAGS+=" $2"
                shift
                ;;
            --evil)
                EVIL_OVERRIDE=1
                ;;
            --no-evil)
                EVIL_OVERRIDE=0
                ;;
            --private-build)
                PRIVATE_OVERRIDE=1
                ;;
            --no-private-build)
                PRIVATE_OVERRIDE=0
                ;;
            --jencore)
                JENCORE_OVERRIDE=1
                ;;
            --no-jencore)
                JENCORE_OVERRIDE=0
                ;;
            --rnode)
                RNODE_OVERRIDE=1
                ;;
            --no-rnode)
                RNODE_OVERRIDE=0
                ;;
            --statsbot)
                STATSBOT_OVERRIDE=1
                ;;
            --no-statsbot)
                STATSBOT_OVERRIDE=0
                ;;
            --replybot)
                REPLYBOT_OVERRIDE=1
                ;;
            --no-replybot)
                REPLYBOT_OVERRIDE=0
                ;;
            -t)
                if [[ -z "${2:-}" ]]; then
                    echo "❌ Error: -t requires a PIO target argument (e.g. upload, monitor)" >&2
                    exit 1
                fi
                PIO_EXTRA_TARGET="$2"
                shift
                ;;
            --dry-run)
                DRY_RUN=true
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                echo "❌ Error: unknown option '$1'" >&2
                usage
                exit 1
                ;;
        esac
        shift
    done
}

# -----------------------------------------------------------------------------
# Resolve the key set for a given builder
# -----------------------------------------------------------------------------
# Sets the global KEYS_FLAGS variable to a string of -DNEXUS_MASTER_KEY_N=...
# entries, or empty string if no keys should be hardcoded.
# -----------------------------------------------------------------------------
resolve_keys_for_builder() {
    local key_mode="$1"
    KEYS_FLAGS=""

    local -a ordered_keys=()

    if [[ ${#USER_KEYS[@]} -gt 0 ]]; then
        # User explicitly specified keys. Sort by slot index, fill ordered_keys.
        # Slots not specified by the user remain empty (firmware won't see them).
        local i
        for i in 1 2 3 4; do
            for entry in "${USER_KEYS[@]}"; do
                local slot="${entry%%:*}"
                local name="${entry#*:}"
                if [[ "$slot" == "$i" ]]; then
                    ordered_keys+=("$name")
                    break
                fi
            done
        done
        # Override the displayed key_mode when custom keys are in use
        KEY_MODE_DISPLAY="custom"
    elif [[ "$key_mode" == "default" ]]; then
        ordered_keys=("${NEXUS_DEFAULT_KEYS[@]}")
        KEY_MODE_DISPLAY="default"
    elif [[ "$key_mode" == "none" ]]; then
        ordered_keys=()
        KEY_MODE_DISPLAY="none"
    fi

    if [[ ${#ordered_keys[@]} -gt 0 ]]; then
        if ! nexus_build_key_flags KEYS_FLAGS "${ordered_keys[@]}"; then
            echo "❌ Error resolving keys for builder" >&2
            exit 1
        fi
    fi
}

# -----------------------------------------------------------------------------
# Determine effective key mode (after applying --default-keys / --no-default-keys)
# -----------------------------------------------------------------------------
effective_key_mode() {
    local base_mode="$1"
    if [[ "${FORCE_DEFAULT_KEYS:-0}" == "1" ]]; then
        echo "default"
    elif [[ "${FORCE_NO_KEYS:-0}" == "1" ]]; then
        echo "none"
    else
        echo "$base_mode"
    fi
}

# -----------------------------------------------------------------------------
# Build a single (builder, target) pair
# -----------------------------------------------------------------------------
build_one() {
    local builder_name="$1"
    local app_repo="$2"
    local target="$3"
    local pio_flags="$4"
    local pio_extra="${5:-}"

    local out_dir="$OUT_ROOT/${builder_name}-${app_repo}"
    mkdir -p "$out_dir"

    # Compose the final pio command tail (with optional -t <extra>)
    local pio_run_tail=(-e "$target")
    if [[ -n "$pio_extra" ]]; then
        pio_run_tail+=(-t "$pio_extra")
    fi

    echo ""
    echo "=================================================================="
    echo "  Builder : $builder_name"
    echo "  Target  : $target"
    echo "  APP_REPO: $app_repo"
    echo "  Flags   :$pio_flags"
    if [[ -n "$pio_extra" ]]; then
        echo "  PIO -t  : $pio_extra"
    fi
    echo "  Output  : $out_dir"
    echo "=================================================================="

    if [[ "$DRY_RUN" == "true" ]]; then
        local dry_extra=""
        [[ -n "$pio_extra" ]] && dry_extra=" -t $pio_extra"
        echo "🔍 DRY-RUN: would run: pio run -e $target -t clean && pio run -e $target$dry_extra (with flags above)"
        return 0
    fi

    # Set env for pio. Note: PLATFORMIO_BUILD_FLAGS is read by platformio.ini.
    export APP_REPO="$app_repo"
    export PLATFORMIO_BUILD_FLAGS="$pio_flags"

    local t0
    t0=$(date +%s)
    if ! pio run -e "$target" -t clean; then
        echo "❌ Clean failed for $target" >&2
        return 1
    fi
    if ! pio run "${pio_run_tail[@]}"; then
        echo "❌ Build failed for $target" >&2
        return 1
    fi
    local t1
    t1=$(date +%s)
    echo "⏱  Build time: $((t1 - t0))s"

    # Copy artifacts to out_dir
    copy_artifacts "$target" "$out_dir"
}

# -----------------------------------------------------------------------------
# Copy build artifacts (.bin for ESP32, .zip + .uf2 for NRF52, plus .factory.bin)
# -----------------------------------------------------------------------------
copy_artifacts() {
    local target="$1"
    local out_dir="$2"
    local build_dir=".pio/build/$target"

    if [[ ! -d "$build_dir" ]]; then
        echo "⚠️  Warning: $build_dir not found, no artifacts to copy" >&2
        return 1
    fi

    local copied=0

    # ESP32-style: firmware-<target>-*.bin (non-factory and factory)
    while IFS= read -r -d '' f; do
        cp "$f" "$out_dir/"
        echo "  📦 copied: $(basename "$f")"
        copied=$((copied + 1))
    done < <(find "$build_dir" -maxdepth 1 -name "firmware-${target}-*.bin" -print0 2>/dev/null)

    # NRF52-style: .zip and .uf2
    while IFS= read -r -d '' f; do
        cp "$f" "$out_dir/"
        echo "  📦 copied: $(basename "$f")"
        copied=$((copied + 1))
    done < <(find "$build_dir" -maxdepth 1 \( -name "firmware-${target}-*.zip" -o -name "firmware-${target}-*.uf2" \) -print0 2>/dev/null)

    if [[ $copied -eq 0 ]]; then
        echo "⚠️  Warning: no artifacts found for $target" >&2
    else
        echo "✅ $copied artifact(s) copied to $out_dir"
    fi
}

# -----------------------------------------------------------------------------
# Build a single builder
# -----------------------------------------------------------------------------
run_builder() {
    local cfg="$1"
    # Parse pipe-delimited fields into an array. Format:
    #   builder_name|app_repo|default_key_mode|trusted_ch1|trusted_ch2|default_evil|default_private
    IFS='|' read -r -a _fields <<<"$cfg"
    local builder_name="${_fields[0]}"
    local app_repo="${_fields[1]}"
    local base_key_mode="${_fields[2]}"
    local trusted_ch1="${_fields[3]}"
    local trusted_ch2="${_fields[4]}"
    local default_evil="${_fields[5]:-no}"
    local default_private="${_fields[6]:-no}"
    unset _fields

    # Targets
    local targets_var="TARGETS_${builder_name^^}"
    local targets="${!targets_var:-$TARGETS_OVERRIDE}"
    if [[ -n "$TARGETS_OVERRIDE" ]]; then
        targets="$TARGETS_OVERRIDE"
    fi

    # Effective key mode
    local key_mode
    key_mode=$(effective_key_mode "$base_key_mode")

    # Resolve keys
    resolve_keys_for_builder "$key_mode"

    # EVIL: apply override, else use builder default
    local evil_active=0
    local evil_source=""
    if [[ "$EVIL_OVERRIDE" == "1" ]]; then
        evil_active=1; evil_source="CLI --evil"
    elif [[ "$EVIL_OVERRIDE" == "0" ]]; then
        evil_active=0; evil_source="CLI --no-evil"
    elif [[ "$default_evil" == "yes" ]]; then
        evil_active=1; evil_source="builder default"
    else
        evil_active=0; evil_source="builder default"
    fi

    # PRIVATE: apply override, else use builder default
    local private_active=0
    local private_source=""
    if [[ "$PRIVATE_OVERRIDE" == "1" ]]; then
        private_active=1; private_source="CLI --private-build"
    elif [[ "$PRIVATE_OVERRIDE" == "0" ]]; then
        private_active=0; private_source="CLI --no-private-build"
    elif [[ "$default_private" == "yes" ]]; then
        private_active=1; private_source="builder default"
    else
        private_active=0; private_source="builder default"
    fi

    # JENCORE / RNODE / STATSBOT / REPLYBOT: disabled by default; CLI-only enable (Jenny 2026-10-24/25)
    local jencore_active=0 jencore_source="default (disabled)"
    if [[ "$JENCORE_OVERRIDE" == "1" ]]; then
        jencore_active=1; jencore_source="CLI --jencore"
    elif [[ "$JENCORE_OVERRIDE" == "0" ]]; then
        jencore_source="CLI --no-jencore"
    fi
    local rnode_active=0 rnode_source="default (disabled)"
    if [[ "$RNODE_OVERRIDE" == "1" ]]; then
        rnode_active=1; rnode_source="CLI --rnode"
    elif [[ "$RNODE_OVERRIDE" == "0" ]]; then
        rnode_source="CLI --no-rnode"
    fi
    local statsbot_active=0 statsbot_source="default (disabled)"
    if [[ "$STATSBOT_OVERRIDE" == "1" ]]; then
        statsbot_active=1; statsbot_source="CLI --statsbot"
    elif [[ "$STATSBOT_OVERRIDE" == "0" ]]; then
        statsbot_source="CLI --no-statsbot"
    fi
    local replybot_active=0 replybot_source="default (disabled)"
    if [[ "$REPLYBOT_OVERRIDE" == "1" ]]; then
        replybot_active=1; replybot_source="CLI --replybot"
    elif [[ "$REPLYBOT_OVERRIDE" == "0" ]]; then
        replybot_source="CLI --no-replybot"
    fi

    # Compose full PIO flags: base + trusted channels + keys + extra + evil
    # Trusted channels: PLATFORMIO_BUILD_FLAGS is parsed by PlatformIO which strips outer
    # quotes; we need the inner quotes around the channel name preserved as if written
    # directly in platformio.ini, hence the explicit \" escaping.
    local base_flags="-DHAS_NEXUS=1 -DAPP_REPO=\$APP_REPO"
    if [[ -n "$trusted_ch1" ]]; then
        base_flags+=" -DNEXUS_TRUSTED_CHANNEL_1=\\\"$trusted_ch1\\\""
    fi
    if [[ -n "$trusted_ch2" ]]; then
        base_flags+=" -DNEXUS_TRUSTED_CHANNEL_2=\\\"$trusted_ch2\\\""
    fi
    if [[ "$evil_active" == "1" ]]; then
        base_flags+=" -DNEXUS_ENABLE_ADV"
    fi
    if [[ "$private_active" == "1" ]]; then
        # -DNEXUS_PRIVATE_BUILD=1 activates the private-side command handlers
        # in NexusModule_private.{h,cpp}. Without this define, those commands
        # fall through to "JT: Unknown command" at runtime.
        base_flags+=" -DNEXUS_PRIVATE_BUILD=1"
    fi
    if [[ "$jencore_active" == "1" ]]; then
        base_flags+=" -DNEXUS_ENABLE_JENCORE=1 -UCRYPTO_AES_NO_DECRYPT"
    fi
    if [[ "$rnode_active" == "1" ]]; then
        base_flags+=" -DNEXUS_ENABLE_RNODE=1"
    fi
    if [[ "$statsbot_active" == "1" ]]; then
        base_flags+=" -DNEXUS_ENABLE_STATSBOT=1"
    fi
    if [[ "$replybot_active" == "1" ]]; then
        base_flags+=" -DNEXUS_ENABLE_REPLYBOT=1"
    fi
    local full_flags="$base_flags$KEYS_FLAGS$EXTRA_FLAGS"

    echo ""
    echo "╔══════════════════════════════════════════════════════════════════"
    echo "║  Builder : $builder_name"
    echo "║  Repo    : $app_repo"
    echo "║  Targets : $targets"
    echo "║  Keys    : ${KEY_MODE_DISPLAY:-$key_mode} mode"
    echo "║  Channels: trusted_1=\"${trusted_ch1}\" trusted_2=\"${trusted_ch2}\""
    if [[ "$evil_active" == "1" ]]; then
        echo "║  ADV     : ON  (-DNEXUS_ENABLE_ADV)  source: $evil_source"
    else
        echo "║  ADV     : OFF                        source: $evil_source"
    fi
    if [[ "$private_active" == "1" ]]; then
        echo "║  PRIVATE : ON  (-DNEXUS_PRIVATE_BUILD=1)  source: $private_source"
    else
        echo "║  PRIVATE : OFF (public decaf build)      source: $private_source"
    fi
    if [[ "$jencore_active" == "1" ]]; then
        echo "║  JENCORE : ON  (-DNEXUS_ENABLE_JENCORE=1) source: $jencore_source"
    else
        echo "║  JENCORE : OFF (disabled by default)     source: $jencore_source"
    fi
    if [[ "$rnode_active" == "1" ]]; then
        echo "║  RNODE   : ON  (-DNEXUS_ENABLE_RNODE=1)   source: $rnode_source"
    else
        echo "║  RNODE   : OFF (disabled by default)     source: $rnode_source"
    fi
    if [[ "$statsbot_active" == "1" ]]; then
        echo "║  STATSBOT: ON  (-DNEXUS_ENABLE_STATSBOT=1) source: $statsbot_source"
    else
        echo "║  STATSBOT: OFF (disabled by default)      source: $statsbot_source"
    fi
    if [[ "$replybot_active" == "1" ]]; then
        echo "║  REPLYBOT: ON  (-DNEXUS_ENABLE_REPLYBOT=1) source: $replybot_source"
    else
        echo "║  REPLYBOT: OFF (disabled by default)      source: $replybot_source"
    fi
    if [[ -n "$PIO_EXTRA_TARGET" ]]; then
        echo "║  PIO -t  : $PIO_EXTRA_TARGET (appended to pio run per target)"
    fi
    if [[ -n "$KEYS_FLAGS" ]]; then
        echo "║  Key flags injected:"
        if [[ ${#USER_KEYS[@]} -gt 0 ]]; then
            # Sort by slot index for display
            for i in 1 2 3 4; do
                for entry in "${USER_KEYS[@]}"; do
                    local slot="${entry%%:*}"
                    local name="${entry#*:}"
                    if [[ "$slot" == "$i" ]]; then
                        echo "║    [slot $i] $name"
                        break
                    fi
                done
            done
        else
            for k in "${NEXUS_DEFAULT_KEYS[@]}"; do
                echo "║    - $k"
            done
        fi
    else
        echo "║  Key flags injected: (none)"
    fi
    echo "╚══════════════════════════════════════════════════════════════════"

    for target in $targets; do
        build_one "$builder_name" "$app_repo" "$target" "$full_flags" "$PIO_EXTRA_TARGET"
    done
}

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
FORCE_DEFAULT_KEYS=0
FORCE_NO_KEYS=0
parse_args "$@"

case "$REQUESTED_BUILDER" in
    jenny)   run_builder "$BUILDER_JENNY" ;;
    admins)  run_builder "$BUILDER_ADMINS" ;;
    javi)    run_builder "$BUILDER_JAVI" ;;
    default) run_builder "$BUILDER_DEFAULT" ;;
    all)
        run_builder "$BUILDER_JENNY"
        run_builder "$BUILDER_ADMINS"
        run_builder "$BUILDER_JAVI"
        run_builder "$BUILDER_DEFAULT"
        ;;
    *)
        echo "❌ Error: unknown builder '$REQUESTED_BUILDER'" >&2
        usage
        exit 1
        ;;
esac

echo ""
echo "🎉 All builds complete."
echo "   Output root: $OUT_ROOT/"
ls -la "$OUT_ROOT/" 2>/dev/null || echo "   (no output directories created — did you --dry-run?)"
