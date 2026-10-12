/*
 * NexusSensorBudget.h — Nexus sensor weight-reduction umbrella tiers.
 *
 * Two variant-facing knobs:
 *
 *   1. NEXUS_SENSOR_WEIGHTREDUCTION (broad module-level drops)
 *      Maps a single integer to a coordinated set of
 *      MESHTASTIC_EXCLUDE_* defines. Replaces the need to plumb 5+
 *      separate `MESHTASTIC_EXCLUDE_*` defines through every variant's
 *      `platformio.ini`.
 *
 *      Usage (in a variant's `platformio.ini` build_flags):
 *
 *          -DNEXUS_SENSOR_WEIGHTREDUCTION=3   # Tier 3: HEAVY (tracker / minimal)
 *          -DNEXUS_SENSOR_WEIGHTREDUCTION=2   # Tier 2: MEDIUM
 *          -DNEXUS_SENSOR_WEIGHTREDUCTION=1   # Tier 1: LIGHT
 *          # (omit or set to 0)               # Tier 0: BALANCED (default, no change)
 *
 *   2. NEXUS_SENSOR_PROMICRO_WHITELIST (fine-grained per-family drops)
 *      Maps a single integer to a coordinated set of
 *      JENTASTIC_EXCLUDE_<FAMILY> defines — the ProMicro "BME/BMP + INA
 *      only" sensor whitelist. Replaces 22 individual `-D` flags in
 *      the variant's platformio.ini.
 *
 *      Usage (in a variant's `platformio.ini` build_flags):
 *
 *          -DNEXUS_SENSOR_PROMICRO_WHITELIST=1   # LITE  (drop heavy single-purpose)
 *          -DNEXUS_SENSOR_PROMICRO_WHITELIST=2   # MEDIUM (drop env non-BME/BMP)
 *          -DNEXUS_SENSOR_PROMICRO_WHITELIST=3   # TIGHT  (BME/BMP + INA only)
 *          # (omit or set to 0)                  # OFF (default, no change)
 *
 *      The two umbrellas are orthogonal but compose cleanly. A variant
 *      can set both. NEXUS_SENSOR_WEIGHTREDUCTION drops the broad
 *      module; NEXUS_SENSOR_PROMICRO_WHITELIST prunes inside it.
 *
 * The mechanism is to FORCE-define the underlying `MESHTASTIC_EXCLUDE_*`
 * macros when the tier is high enough. This works because every sensor
 * source file (src/modules/Telemetry/Sensor/*.cpp) already gates itself
 * on those macros — so we don't need to touch any of them. The
 * `Modules.cpp` registration of the heavy sensor modules
 * (`EnvironmentTelemetryModule`, `AirQualityTelemetryModule`,
 * `HealthTelemetryModule`, `PowerTelemetryModule`) is also already gated
 * on the same macros, so the registration is skipped automatically.
 *
 * The Nexus-specific helpers in `NexusModule.cpp`
 * (`getNexusPowerTelemetry`, `getNexusEnvironmentTelemetry`) are likewise
 * gated, so the Shadow CLI's `/nexus TELEMETRY` quietly returns false
 * instead of dereferencing an unregistered module.
 *
 * Tier choice (see NexusModule.cpp for full rationale):
 *
 *   HEAVY  (3):  ~14 KB flash + ~1 KB RAM saved vs default.
 *                Drops: power, environment, air quality, health, detection.
 *                Keeps: device telemetry (battery, uptime via DeviceTelemetryModule).
 *                Recommended for: T1000-E ghost trackers, mesh-only nodes.
 *
 *   MEDIUM (2):  ~6 KB flash + ~0.4 KB RAM saved.
 *                Drops: air quality, health, detection.
 *                Keeps: environment (BME/BMP/SHT), power (INA).
 *                Recommended for: env+INA "weather station" nodes.
 *
 *   LIGHT  (1):  ~2 KB flash + ~0.1 KB RAM saved.
 *                Drops: air quality, health.
 *                Keeps: environment, power, detection.
 *                Recommended for: heltec-v4 with extra room to spare.
 *
 *   BALANCED (0): Default Meshtastic behavior. Nothing excluded.
 *
 * Implementation notes:
 * - We use `MESHTASTIC_EXCLUDE_*` (the existing Meshtastic name) rather
 *   than inventing a parallel `NEXUS_EXCLUDE_*` namespace, so the existing
 *   per-sensor `#if !MESHTASTIC_EXCLUDE_X` guards are reused unchanged.
 * - We force-define ONLY when the tier is high enough; lower tiers do
 *   not override user-set flags. This means a variant can still set
 *   `-DMESHTASTIC_EXCLUDE_AIR_QUALITY_SENSOR=1` explicitly and the tier
 *   won't clash.
 * - This header is intentionally tiny and idempotent: the `#define`s
 *   below are no-ops if the user has already provided the same value
 *   from the variant's build_flags.
 *
 * Optional source-level exclusion (rare; for Tier 3 on extremely tight
 * nRF52 builds):
 *
 * The `#if` guards inside each sensor `.cpp` produce an empty translation
 * unit when a macro is set, but the file is still tokenized. For a
 * variant that wants to skip the parsing entirely (saves ~3-5s compile
 * time on a clean build), drop the relevant subfolder from the variant's
 * `build_src_filter`:
 *
 *     build_src_filter = ${nrf52_base.build_src_filter}
 *       -<modules/Telemetry/Sensor/>              ; Tier 3: skip all sensors
 *       -<modules/Telemetry/EnvironmentTelemetry.*>  ; Tier 3: env module
 *       -<modules/Telemetry/AirQualityTelemetry.*>   ; Tier 1+: air module
 *       -<modules/Telemetry/HealthTelemetry.*>       ; Tier 1+: health module
 *       -<modules/Telemetry/PowerTelemetry.*>        ; Tier 3: power module
 *
 * Use the umbrella flag alone unless you have a specific reason to also
 * prune the source filter; the `#if` guards are usually sufficient.
 *
 * -----------------------------------------------------------------------
 * JENTASTIC_EXCLUDE_<X> per-family macros (Nexus-269, 2026-08-10)
 * -----------------------------------------------------------------------
 *
 * For variants that want fine-grained control over the env/power sensor
 * mix (e.g., keep BME/BMP/INA but drop BME680, SHT/LPS/AHT, NAU7802, etc.),
 * set individual `-DJENTASTIC_EXCLUDE_<FAMILY>=1` flags. Each macro:
 *
 *   1. Gates the per-sensor `.cpp` translation unit (forces it to no-op
 *      via the existing `#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR`
 *      umbrella is NOT enough; we need per-family control. The .cpp
 *      bodies are gated via an additional `#if !JENTASTIC_EXCLUDE_<X>`
 *      check inserted into each sensor header).
 *   2. Gates the `addSensor<Xxx>(...)` call site in
 *      `EnvironmentTelemetry::i2cScanFinished` (EnvironmentTelemetry.cpp)
 *      so the registration template is skipped.
 *   3. Gates the `power.h` include block for power-side sensors (INA
 *      family stays — they're whitelisted by default for nrf52 boards).
 *
 * Naming convention: JENTASTIC_EXCLUDE_<UPPER_SNAKE> matches the sensor
 * file prefix. E.g., `BME680Sensor.cpp` → `JENTASTIC_EXCLUDE_BME680`.
 *
 * Macro list (canonical source of truth for variants — DO NOT redefine
 * elsewhere; this header is included before any sensor source):
 *
 *   Air quality family (env-side detectors):
 *     JENTASTIC_EXCLUDE_RAK12035
 *
 *   NOTE: The other air-quality sensors (SCD30, SCD4X, SEN5X, SFA30,
 *   PMSA003I) are gated on MESHTASTIC_EXCLUDE_AIR_QUALITY_SENSOR at the
 *   module level via AirQualityTelemetry.cpp — there is no per-family
 *   JENTASTIC guard for them. Variants that want air quality gone
 *   should set -DMESHTASTIC_EXCLUDE_AIR_QUALITY_SENSOR=1 directly (or
 *   use NEXUS_SENSOR_WEIGHTREDUCTION Tier 1+). The JENTASTIC umbrella
 *   intentionally does NOT duplicate this toggle.
 *
 *   Health family (already covered by MESHTASTIC_EXCLUDE_HEALTH_TELEMETRY
 *   for the module, but per-family here is for the .cpp side):
 *     JENTASTIC_EXCLUDE_MAX30102, JENTASTIC_EXCLUDE_MLX90614,
 *     JENTASTIC_EXCLUDE_MLX90632
 *
 *   Environment sensors NOT in the BME/BMP whitelist:
 *     JENTASTIC_EXCLUDE_BME680 (explicit "not 680" requirement),
 *     JENTASTIC_EXCLUDE_DPS310, JENTASTIC_EXCLUDE_SHT31, JENTASTIC_EXCLUDE_SHT4X,
 *     JENTASTIC_EXCLUDE_LPS22HB, JENTASTIC_EXCLUDE_SHTC3, JENTASTIC_EXCLUDE_AHT10,
 *     JENTASTIC_EXCLUDE_MCP9808, JENTASTIC_EXCLUDE_PCT2075, JENTASTIC_EXCLUDE_NAU7802,
 *     JENTASTIC_EXCLUDE_RCWL9620, JENTASTIC_EXCLUDE_CGRADSENS,
 *     JENTASTIC_EXCLUDE_DFROBOT (covers Lark + Gravity),
 *     JENTASTIC_EXCLUDE_BH1750, JENTASTIC_EXCLUDE_LTR390UV, JENTASTIC_EXCLUDE_OPT3001,
 *     JENTASTIC_EXCLUDE_TSL2561, JENTASTIC_EXCLUDE_TSL2591, JENTASTIC_EXCLUDE_VEML7700
 *
 *   Power-side sensors (sibling to MAX17048 — not whitelisted by default):
 *     JENTASTIC_EXCLUDE_MAX17048, JENTASTIC_EXCLUDE_RAK9154
 *
 *   T1000-E / SenseCAP-specific (skip on non-t1000 variants):
 *     JENTASTIC_EXCLUDE_T1000X, JENTASTIC_EXCLUDE_INDICATOR
 *
 * (Nexus-269, 2026-08-10.)
 */

#pragma once

#ifdef NEXUS_SENSOR_WEIGHTREDUCTION

/* Tier 1+ (LIGHT): drop air quality + health telemetry. */
#if NEXUS_SENSOR_WEIGHTREDUCTION >= 1
#  ifndef MESHTASTIC_EXCLUDE_AIR_QUALITY_SENSOR
#    define MESHTASTIC_EXCLUDE_AIR_QUALITY_SENSOR 1
#  endif
#  ifndef MESHTASTIC_EXCLUDE_HEALTH_TELEMETRY
#    define MESHTASTIC_EXCLUDE_HEALTH_TELEMETRY 1
#  endif
#endif

/* Tier 2+ (MEDIUM): also drop detection sensors (motion sensors using detection). */
#if NEXUS_SENSOR_WEIGHTREDUCTION >= 2
#  ifndef MESHTASTIC_EXCLUDE_DETECTIONSENSOR
#    define MESHTASTIC_EXCLUDE_DETECTIONSENSOR 1
#  endif
#endif

/* Tier 3+ (HEAVY): also drop environment + power telemetry. */
#if NEXUS_SENSOR_WEIGHTREDUCTION >= 3
#  ifndef MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR
#    define MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR 1
#  endif
#  ifndef MESHTASTIC_EXCLUDE_POWER_TELEMETRY
#    define MESHTASTIC_EXCLUDE_POWER_TELEMETRY 1
#  endif
#endif

#endif /* NEXUS_SENSOR_WEIGHTREDUCTION */

/* ------------------------------------------------------------------- */
/* JENTASTIC_EXCLUDE_* per-family macros.                              */
/*                                                                    */
/* Default behavior: undefined. Set `-DJENTASTIC_EXCLUDE_<X>=1` in    */
/* the variant's build_flags to drop the corresponding sensor family. */
/* Call-site guards in EnvironmentTelemetry.cpp / power.h read these  */
/* directly. Sensor source-file bodies (Telemetry/Sensor/*.cpp) are   */
/* NOT re-gated here — they already short-circuit on the umbrella     */
/* MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR macro, which we keep       */
/* enabled for variants that want per-family control (i.e., variants  */
/* using JENTASTIC macros want to KEEP environment enabled so BME/BMP */
/* /INA still work).                                                  */
/*                                                                    */
/* So the JENTASTIC macros are call-site-only gates. The corresponding */
/* `.cpp` files will still be compiled (their existing                */
/* `#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR` guard passes), but  */
/* the resulting object file is unused at link time because nothing    */
/* instantiates the class. This is acceptable — the symbols don't end */
/* up in the final binary.                                            */
/*                                                                    */
/* This header is force-included at the top of:                       */
/*   - src/modules/Modules.cpp                                        */
/*   - src/modules/NexusModule.cpp                                    */
/*   - src/modules/Telemetry/EnvironmentTelemetry.cpp                 */
/*   - src/power.h                                                    */
/*   - src/Power.cpp                                                  */
/* so the umbrella force-defines below reach every translation unit   */
/* that reads JENTASTIC_EXCLUDE_<FAMILY>.                             */
/* ------------------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* JENTASTIC umbrella preset (Nexus-269, 2026-08-11).          */
/*                                                                    */
/* A single variant-facing knob that auto-defines the                 */
/* JENTASTIC_EXCLUDE_<FAMILY> flags for the ProMicro "BME/BMP + INA  */
/* only" sensor whitelist. Replaces 22 individual `-D` flags in the   */
/* variant's platformio.ini.                                          */
/*                                                                    */
/* Usage (in a variant's platformio.ini build_flags):                 */
/*                                                                    */
/*     -DNEXUS_SENSOR_PROMICRO_WHITELIST=1   # LITE                  */
/*     -DNEXUS_SENSOR_PROMICRO_WHITELIST=2   # MEDIUM                */
/*     -DNEXUS_SENSOR_PROMICRO_WHITELIST=3   # TIGHT (recommended)  */
/*                                                                    */
/* Higher number = more aggressive pruning. Level 3 (= TIGHT) is the  */
/* historical ProMicro whitelist (the 22 flags previously plumbed in   */
/* the variant's platformio.ini). Lower levels drop less.             */
/*                                                                    */
/* Keepers (whitelist, NOT excluded): BME280, BMP085, BMP280,        */
/* BMP3XX, INA219, INA226, INA260, INA3221. These are common         */
/* baseline sensors and stay unconditional.                           */
/*                                                                    */
/* The JENTASTIC umbrella is orthogonal to NEXUS_SENSOR_WEIGHTREDUCTION. */
/* Fine-grained intra-module pruning happens here; broad module-level  */
/* drops (MESHTASTIC_EXCLUDE_AIR_QUALITY_SENSOR, etc.) happen via the  */
/* NEXUS_SENSOR_WEIGHTREDUCTION tier. They compose: a variant can set  */
/* both.                                                              */
/*                                                                    */
/* Do NOT add new presets here without explicit approval.             */
/* If a new variant needs a different whitelist, add a new            */
/* umbrella macro (e.g. NEXUS_SENSOR_*) — do NOT modify this one.     */
/* ------------------------------------------------------------------- */

#ifdef NEXUS_SENSOR_PROMICRO_WHITELIST

/* Level 1+ (LITE): drop heavy single-purpose sensors. */
/* Air quality + thermal (MLX90632). Keeps env non-BME/BMP (DPS, SHT,
   AHT, LPS, MCP, PCT, BME680), light sensors, and power-side. */
#if NEXUS_SENSOR_PROMICRO_WHITELIST >= 1

/* Air quality */
/* RAK12035 is the only JENTASTIC air-quality macro that is actually
   read (EnvironmentTelemetry.cpp::i2cScanFinished, gated on
   __has_include + RAK_4631). The other air-quality sensors (SCD30,
   SCD4X, SEN5X, SFA30, PMSA003I) are gated on
   MESHTASTIC_EXCLUDE_AIR_QUALITY_SENSOR at the module level via
   AirQualityTelemetry.cpp — there is no per-family JENTASTIC guard
   for them. Variants that want air quality gone should set
   -DMESHTASTIC_EXCLUDE_AIR_QUALITY_SENSOR=1 directly (or via
   NEXUS_SENSOR_WEIGHTREDUCTION Tier 1+). Do NOT add JENTASTIC
   defines for them here — they would be dead code. */
#  ifndef JENTASTIC_EXCLUDE_RAK12035
#    define JENTASTIC_EXCLUDE_RAK12035 1
#  endif

/* Thermal (MLX90632 is the only one with a JENTASTIC gate; MLX90614 /
   MAX30102 are gated by MESHTASTIC_EXCLUDE_HEALTH_TELEMETRY) */
#  ifndef JENTASTIC_EXCLUDE_MLX90632
#    define JENTASTIC_EXCLUDE_MLX90632 1
#  endif

#endif /* Level 1+ */

/* Level 2+ (MEDIUM): also drop non-BME/BMP env sensors. Level 1 was
   "drop the heavy single-purpose ones"; Level 2 also drops the
   generic env sensors that BME/BMP cover (BME280, BMP085/280/3XX
   already measure pressure + temp + humidity). Keeps light sensors
   (BH1750/LTR390UV/OPT3001/TSL2561/TSL2591/VEML7700) and power-side
   (MAX17048/RAK9154). */
#if NEXUS_SENSOR_PROMICRO_WHITELIST >= 2

#  ifndef JENTASTIC_EXCLUDE_BME680
#    define JENTASTIC_EXCLUDE_BME680 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_DPS310
#    define JENTASTIC_EXCLUDE_DPS310 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_SHT31
#    define JENTASTIC_EXCLUDE_SHT31 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_SHT4X
#    define JENTASTIC_EXCLUDE_SHT4X 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_LPS22HB
#    define JENTASTIC_EXCLUDE_LPS22HB 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_SHTC3
#    define JENTASTIC_EXCLUDE_SHTC3 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_AHT10
#    define JENTASTIC_EXCLUDE_AHT10 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_MCP9808
#    define JENTASTIC_EXCLUDE_MCP9808 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_PCT2075
#    define JENTASTIC_EXCLUDE_PCT2075 1
#  endif

#endif /* Level 2+ */

/* Level 3+ (TIGHT): also drop light sensors, niche modules
   (RCWL9620 radar, NAU7802 load cell, CGRADSENS radiation, DFROBOT
   Lark/Gravity), and power-side sensors (MAX17048, RAK9154). Keeps
   only BME280, BMP085, BMP280, BMP3XX, INA219, INA226, INA260,
   INA3221. This is the historical ProMicro whitelist. */
#if NEXUS_SENSOR_PROMICRO_WHITELIST >= 3

#  ifndef JENTASTIC_EXCLUDE_NAU7802
#    define JENTASTIC_EXCLUDE_NAU7802 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_RCWL9620
#    define JENTASTIC_EXCLUDE_RCWL9620 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_CGRADSENS
#    define JENTASTIC_EXCLUDE_CGRADSENS 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_DFROBOT
#    define JENTASTIC_EXCLUDE_DFROBOT 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_BH1750
#    define JENTASTIC_EXCLUDE_BH1750 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_LTR390UV
#    define JENTASTIC_EXCLUDE_LTR390UV 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_OPT3001
#    define JENTASTIC_EXCLUDE_OPT3001 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_TSL2561
#    define JENTASTIC_EXCLUDE_TSL2561 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_TSL2591
#    define JENTASTIC_EXCLUDE_TSL2591 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_VEML7700
#    define JENTASTIC_EXCLUDE_VEML7700 1
#  endif

/* Power-side */
#  ifndef JENTASTIC_EXCLUDE_MAX17048
#    define JENTASTIC_EXCLUDE_MAX17048 1
#  endif
#  ifndef JENTASTIC_EXCLUDE_RAK9154
#    define JENTASTIC_EXCLUDE_RAK9154 1
#  endif

#endif /* Level 3+ */

#endif /* NEXUS_SENSOR_PROMICRO_WHITELIST */