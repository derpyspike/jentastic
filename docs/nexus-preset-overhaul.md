# Nexus Firmware Enhancement: Comprehensive PRESET Overhaul & Multi-Band Radio Guard

**Author:** Jirachi ⭐ on Noctali (`Noctali.local`)  
**Target:** Magnemite / Jenny / Pokéteam  
**Branch:** `feature/nexus-preset-overhaul`  
**Commit:** `7b3099f1d` (`feat(nexus): overhaul PRESET with LORA_24, Madrid24, Narrow, Lite, Mini-1..8 and band guards`)  
**Base Commit:** `9a696ee2f` (`fix/nexus-local-commands`)  
**Date:** 2026-10-20  

---

## 1. Executive Summary

The `PRESET` / `SETPRESET` command handler in `src/modules/NexusCommands.cpp` has been completely overhauled:

1. **2.4 GHz Preset Support (`LORA_24`):**
   - Added custom tactical preset **`Madrid24`** (`2482.0000 MHz`, BW 400 kHz, SF9, CR 4/5, region `LORA_24`).
   - Added full standard 2.4 GHz presets (`LF24`/`LongFast`, `MS24`/`MediumSlow`, `MF24`/`MediumFast`, `SS24`/`ShortSlow`, `SF24`/`ShortFast`, `LM24`/`LongModerate`, `LS24`/`LongSlow`, `ST24`/`ShortTurbo`, `LT24`/`LongTurbo`) with frequency override `2482.0000 MHz` and appropriate bandwidths (800 kHz, 400 kHz, 1600 kHz).
   - Standard aliases (`LF`, `MS`, `MF`, etc.) are context-aware: when the active region is `LORA_24`, they automatically configure the 2.4 GHz parameters; otherwise, they configure `EU_868`.

2. **Upstream Regional Profiles:**
   - **`NarrowFast` (`NF`) & `NarrowSlow` (`NS`):** Sets region to `EU_N_868`, frequency override `869.4417 MHz`, BW 62.5 kHz (code 62), CR 4/6 (code 6), SF7/SF8.
   - **`LiteFast` & `LiteSlow`:** Sets region to `EU_866`, frequency override `866.3000 MHz` (LiteFast) and `865.7000 MHz` (LiteSlow), BW 125 kHz (code 125), CR 4/5 (code 5), SF9/SF10.

3. **Sub-GHz Region Alignment:**
   - Standard EU presets and Spain/Tactical presets (`P1`, `P7`, `SP`, `SPM`, `NARROWPROMAX`) explicitly set region `EU_868`.
   - NAP (`NAP-1`..`5`) and NAPN (`NAPN-1`..`5`) explicitly set region `EU_866`.

4. **New High-Efficiency Mini Presets (`Mini-1` through `Mini-8`):**
   - The 4 NAP channels in `EU_866` use 125 kHz bandwidth in 200 kHz channel spacing, leaving 75 kHz of unallocated spectrum (37.5 kHz guardbands on each side).
   - Eight ultra-fast, low-spread presets (`Mini-1` through `Mini-8`) are placed directly in the centers of these 37.5 kHz spaces:
     - Bandwidth: **31.25 kHz** (`bandwidth = 31`)
     - Spreading Factor: **SF6**
     - Coding Rate: **4/5** (`coding_rate = 5`)
     - Region: **`EU_866`**

5. **Bidirectional Multi-Band Hardware Guard (with Dual-Band Exception):**
   - If a radio is single-band (e.g. SX1262, SX1280, LLCC68, RF95):
     - On `LORA_24`, setting any Sub-GHz preset (`EU_868`, `EU_866`, `EU_N_868`) is blocked with:  
       `JT: Cannot set Sub-GHz preset on LORA_24 (single-band radio).`
     - On Sub-GHz (`EU_868`, `EU_866`, `EU_N_868`, or unset), setting any 2.4 GHz preset (`Madrid24` or `LORA_24`) is blocked with:  
       `JT: Cannot set 2.4GHz preset on Sub-GHz region (single-band radio).`
   - **Hardware Exception:** If the transceiver is dual-band (**`radioType == LR1121_RADIO`** or **`LR2021_RADIO`**), the guard is bypassed, allowing seamless live transitions between 2.4 GHz and Sub-GHz!

---

## 2. Mathematical Frequency Derivations & Spectra

### A. NAP, Lite, and Mini Frequency Allocations (`EU_866`)

The 4 NAP channels are spaced at 600 kHz intervals starting at 865.700 MHz:
- Channel span per band: 200 kHz
- LoRaWAN / Lite channel width: 125 kHz centered on channel center frequency
- Unallocated guardband: $(200 - 125) / 2 = 37.5\text{ kHz}$ on the left and right
- Mini channels: $31.25\text{ kHz}$ wide, centered in each $37.5\text{ kHz}$ slice (leaving $3.125\text{ kHz}$ safety margins on outer and inner edges):
  $$\text{Center}_{\text{left}} = F_{\text{NAP}} - \frac{125 + 37.5}{2}\text{ kHz} = F_{\text{NAP}} - 81.25\text{ kHz}$$
  $$\text{Center}_{\text{right}} = F_{\text{NAP}} + \frac{125 + 37.5}{2}\text{ kHz} = F_{\text{NAP}} + 81.25\text{ kHz}$$

| Band | Channel Center | Lite / NAP (125 kHz) Span | Mini Preset (Left) | Mini Left Center | Mini Left Span (31.25 kHz) | Mini Preset (Right) | Mini Right Center | Mini Right Span (31.25 kHz) |
|---|---|---|---|---|---|---|---|---|
| **NAP 1** | `865.7000 MHz` | 865.6375 - 865.7625 | `Mini-1` | **`865.6188 MHz`** | 865.6031 - 865.6344 | `Mini-2` | **`865.7812 MHz`** | 865.7656 - 865.7969 |
| **NAP 2** | `866.3000 MHz` | 866.2375 - 866.3625 | `Mini-3` | **`866.2188 MHz`** | 866.2031 - 866.2344 | `Mini-4` | **`866.3812 MHz`** | 866.3656 - 866.3969 |
| **NAP 3** | `866.9000 MHz` | 866.8375 - 866.9625 | `Mini-5` | **`866.8188 MHz`** | 866.8031 - 866.8344 | `Mini-6` | **`866.9812 MHz`** | 866.9656 - 866.9969 |
| **NAP 4** | `867.5000 MHz` | 867.4375 - 867.5625 | `Mini-7` | **`867.4188 MHz`** | 867.4031 - 867.4344 | `Mini-8` | **`867.5812 MHz`** | 867.5656 - 867.5969 |

*(Note: `LiteSlow` shares `865.7000 MHz` with `NAP-1`; `LiteFast` shares `866.3000 MHz` with `NAP-2`).*

### B. Upstream Narrow Presets (`EU_N_868`)

Meshtastic's `EU_N_868` region defaults to `869.441666... MHz` (frequency slot: `869.4417 MHz`) with $62.5\text{ kHz}$ bandwidth and CR 4/6:
- `NarrowFast` (`NF`): SF7, BW 62, CR 6, `869.4417 MHz`, Region `EU_N_868`
- `NarrowSlow` (`NS`): SF8, BW 62, CR 6, `869.4417 MHz`, Region `EU_N_868`

---

## 3. Comprehensive Master Preset Reference Table

| Group | Preset Command | Aliases | SF | BW Code | CR | Override Freq | Target Region |
|---|---|---|---|---|---|---|---|
| **2.4 GHz Tactical** | `PRESET MADRID24` | `MADRID-24`, `MADRID_24` | 9 | 400 | 4/5 | `2482.0000 MHz` | `LORA_24` |
| **2.4 GHz Standard** | `PRESET LF24` | `LONGFAST24`, `LONGFAST_24`, `LF` (on 2.4G) | 11 | 800 | 4/5 | `2482.0000 MHz` | `LORA_24` |
| 2.4 GHz Standard | `PRESET MS24` | `MEDSLOW24`, `MEDSLOW_24`, `MS` (on 2.4G) | 10 | 800 | 4/5 | `2482.0000 MHz` | `LORA_24` |
| 2.4 GHz Standard | `PRESET MF24` | `MEDFAST24`, `MEDFAST_24`, `MF` (on 2.4G) | 9 | 800 | 4/5 | `2482.0000 MHz` | `LORA_24` |
| 2.4 GHz Standard | `PRESET SS24` | `SHORTSLOW24`, `SHORTSLOW_24`, `SS` (on 2.4G) | 8 | 800 | 4/5 | `2482.0000 MHz` | `LORA_24` |
| 2.4 GHz Standard | `PRESET SF24` | `SHORTFAST24`, `SHORTFAST_24`, `SF` (on 2.4G) | 7 | 800 | 4/5 | `2482.0000 MHz` | `LORA_24` |
| 2.4 GHz Standard | `PRESET LM24` | `LONGMODERATE24`, `LONGMODERATE_24`, `LM` (on 2.4G) | 11 | 400 | 4/8 | `2482.0000 MHz` | `LORA_24` |
| 2.4 GHz Standard | `PRESET LS24` | `LONGSLOW24`, `LONGSLOW_24`, `LS` (on 2.4G) | 12 | 400 | 4/8 | `2482.0000 MHz` | `LORA_24` |
| 2.4 GHz Standard | `PRESET ST24` | `SHORTTURBO24`, `SHORTTURBO_24`, `ST` (on 2.4G) | 7 | 1600 | 4/5 | `2482.0000 MHz` | `LORA_24` |
| 2.4 GHz Standard | `PRESET LT24` | `LONGTURBO24`, `LONGTURBO_24`, `LT` (on 2.4G) | 11 | 1600 | 4/8 | `2482.0000 MHz` | `LORA_24` |
| **EU_868 Standard** | `PRESET LF` | `LF868`, `LONGFAST`, `LONGFAST868` | 11 | 250 | 4/5 | `869.5250 MHz` | `EU_868` |
| EU_868 Standard | `PRESET MS` | `MS868`, `MEDSLOW`, `MEDSLOW868` | 10 | 250 | 4/5 | `869.5250 MHz` | `EU_868` |
| EU_868 Standard | `PRESET MF` | `MF868`, `MEDFAST`, `MEDFAST868` | 9 | 250 | 4/5 | `869.5250 MHz` | `EU_868` |
| EU_868 Standard | `PRESET SS` | `SS868`, `SHORTSLOW`, `SHORTSLOW868` | 8 | 250 | 4/5 | `869.5250 MHz` | `EU_868` |
| EU_868 Standard | `PRESET SF` | `SF868`, `SHORTFAST`, `SHORTFAST868` | 7 | 250 | 4/5 | `869.5250 MHz` | `EU_868` |
| EU_868 Standard | `PRESET LM` | `LM868`, `LONGMODERATE`, `LONGMODERATE868` | 11 | 125 | 4/8 | `869.5250 MHz` | `EU_868` |
| EU_868 Standard | `PRESET LS` | `LS868`, `LONGSLOW`, `LONGSLOW868` | 12 | 125 | 4/8 | `869.5250 MHz` | `EU_868` |
| EU_868 Standard | `PRESET ST` | `ST868`, `SHORTTURBO`, `SHORTTURBO868` | 7 | 500 | 4/5 | `869.5250 MHz` | `EU_868` |
| EU_868 Standard | `PRESET LT` | `LT868`, `LONGTURBO`, `LONGTURBO868` | 11 | 500 | 4/8 | `869.5250 MHz` | `EU_868` |
| **Tactical 868** | `PRESET P1` | `SFNARROW` | 7 | 62 | 4/5 | `869.6180 MHz` | `EU_868` |
| Tactical 868 | `PRESET P7` | `NARROWPLUS` | 6 | 62 | 4/5 | `869.6180 MHz` | `EU_868` |
| Tactical 868 | `PRESET NARROWPROMAX` | `NARROWPREMIUM` | 5 | 62 | 4/5 | `869.6180 MHz` | `EU_868` |
| Tactical 868 | `PRESET SP` | `SHORTPLUS` | 6 | 250 | 4/5 | `869.5250 MHz` | `EU_868` |
| Tactical 868 | `PRESET SPM` | `SHORTPROMAX` | 5 | 250 | 4/5 | `869.5250 MHz` | `EU_868` |
| **Narrow (EU_N_868)** | `PRESET NF` | `NARROWFAST`, `NARF`, `NARROW_FAST`, `NARROW-FAST` | 7 | 62 | 4/6 | `869.4417 MHz` | `EU_N_868` |
| Narrow (EU_N_868) | `PRESET NS` | `NARROWSLOW`, `NARS`, `NARROW_SLOW`, `NARROW-SLOW` | 8 | 62 | 4/6 | `869.4417 MHz` | `EU_N_868` |
| **Lite (EU_866)** | `PRESET LITEFAST` | `LITEF`, `LITE_FAST`, `LITE-FAST` | 9 | 125 | 4/5 | `866.3000 MHz` | `EU_866` |
| Lite (EU_866) | `PRESET LITESLOW` | `LITES`, `LITE_SLOW`, `LITE-SLOW` | 10 | 125 | 4/5 | `865.7000 MHz` | `EU_866` |
| **NAP (EU_866)** | `PRESET NAP-1` | `NAP1` | 7 | 125 | 4/5 | `865.7000 MHz` | `EU_866` |
| NAP (EU_866) | `PRESET NAP-2` | `NAP2` | 7 | 125 | 4/5 | `866.3000 MHz` | `EU_866` |
| NAP (EU_866) | `PRESET NAP-3` | `NAP3` | 7 | 125 | 4/5 | `866.9000 MHz` | `EU_866` |
| NAP (EU_866) | `PRESET NAP-4` | `NAP4` | 7 | 125 | 4/5 | 867.5000 MHz | `EU_866` |
| NAP (EU_866) | `PRESET NAP-5` | `NAP5` | 7 | 125 | 4/5 | 871.0000 MHz | `EU_866` |
| **NAPN (EU_866)** | `PRESET NAPN-1` | `NAPN1` | 7 | 62 | 4/5 | `865.7000 MHz` | `EU_866` |
| NAPN (EU_866) | `PRESET NAPN-2` | `NAPN2` | 7 | 62 | 4/5 | `866.3000 MHz` | `EU_866` |
| NAPN (EU_866) | `PRESET NAPN-3` | `NAPN3` | 7 | 62 | 4/5 | `866.9000 MHz` | `EU_866` |
| NAPN (EU_866) | `PRESET NAPN-4` | `NAPN4` | 7 | 62 | 4/5 | 867.5000 MHz | `EU_866` |
| NAPN (EU_866) | `PRESET NAPN-5` | `NAPN5` | 7 | 62 | 4/5 | 871.0000 MHz | `EU_866` |
| **Mini (EU_866)** | `PRESET MINI-1` | `MINI1`, `MICRO-1`, `MICRO1` | 6 | 31 | 4/5 | **`865.6188 MHz`** | `EU_866` |
| Mini (EU_866) | `PRESET MINI-2` | `MINI2`, `MICRO-2`, `MICRO2` | 6 | 31 | 4/5 | **`865.7812 MHz`** | `EU_866` |
| Mini (EU_866) | `PRESET MINI-3` | `MINI3`, `MICRO-3`, `MICRO3` | 6 | 31 | 4/5 | **`866.2188 MHz`** | `EU_866` |
| Mini (EU_866) | `PRESET MINI-4` | `MINI4`, `MICRO-4`, `MICRO4` | 6 | 31 | 4/5 | **`866.3812 MHz`** | `EU_866` |
| Mini (EU_866) | `PRESET MINI-5` | `MINI5`, `MICRO-5`, `MICRO5` | 6 | 31 | 4/5 | **`866.8188 MHz`** | `EU_866` |
| Mini (EU_866) | `PRESET MINI-6` | `MINI6`, `MICRO-6`, `MICRO6` | 6 | 31 | 4/5 | **`866.9812 MHz`** | `EU_866` |
| Mini (EU_866) | `PRESET MINI-7` | `MINI7`, `MICRO-7`, `MICRO7` | 6 | 31 | 4/5 | **`867.4188 MHz`** | `EU_866` |
| Mini (EU_866) | `PRESET MINI-8` | `MINI8`, `MICRO-8`, `MICRO8` | 6 | 31 | 4/5 | **`867.5812 MHz`** | `EU_866` |

---

## 4. Verification Receipts

Both target architectures were built clean using PlatformIO 6.1.19:

- **Target 1: Seeed T1000-E (`tracker-t1000-e`, nRF52840 + SX1262)**
  - Status: **`SUCCESS`** (00:02:48)
  - RAM: 39.8% (99,100 bytes / 248,832 bytes)
  - Flash: 76.9% (626,532 bytes / 815,104 bytes)
  - Output: `firmware-tracker-t1000-e-2.8.103.9a696ee.uf2` & `.hex`

- **Target 2: Heltec WiFi LoRa 32 V3 (`heltec-v3`, ESP32-S3 + SX1262)**
  - Status: **`SUCCESS`** (00:05:35)
  - RAM: 38.1% (124,912 bytes / 327,680 bytes)
  - Flash: 63.7% (2,127,835 bytes / 3,342,336 bytes)
  - Output: `firmware-heltec-v3-2.8.103.9a696ee.factory.bin` & `.bin`
