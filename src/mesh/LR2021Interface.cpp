#include "configuration.h"

#if (defined(USE_LR2021) || defined(ARCH_PORTDUINO)) && RADIOLIB_EXCLUDE_LR2021 != 1

#include "LR2021Interface.h"
#include "error.h"

LR2021Interface::LR2021Interface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                                 RADIOLIB_PIN_TYPE busy)
    : LR20x0Interface(hal, cs, irq, rst, busy)
{
}

bool LR2021Interface::wideLora()
{
    return true;
}

// ----------------------------------------------------------------------------
// LR2021 multi-SF side-detector configuration
// ----------------------------------------------------------------------------
//
// Thin wrapper around RadioLib::LR2021::setSideDetector (see LR2021.h:802,
// LR2021_config.cpp:1108). RadioLib already exposes the structured API:
// each LR2021LoRaSideDetector_t carries { sf, ldro, invertIQ, syncWord },
// and a single call sets up to 3 side detectors AND their syncwords.
//
// Our thin layer translates the operator's SF list (raw bytes 5..12) into
// the LR2021LoRaSideDetector_t array. We leave ldro=false, invertIQ=false,
// and syncWord = this->syncWord (the Meshtastic-side member from
// RadioLibInterface.h:100, default 0x2B, overridable via /nexus SETSYNCWORD
// which lands in vault v40) so the side detectors listen on the SAME byte
// as the main detector. This matches the Meshtastic protocol invariant of
// one shared syncword on the channel.
//
// IMPORTANT — this is the bug-fix that resolves the SF7-unheard issue
// caught on 2026-09-12 (commit pending). The pre-refactor hand-rolled
// adapter (7e1c908f9 / 6e6972faa) only sent opcode 0x0224 (SF config) and
// never sent 0x0225 (syncword), so side detectors inherited the chip's
// existing main syncword by chance. After the refactor to RadioLib's
// setSideDetector (7ef6a6837) the side detectors were getting their syncword
// field explicitly written — but to RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE
// (0x12), which doesn't match Meshtastic's 0x2B or any operator override.
// Result: side detectors never matched any incoming packet on the channel.
// Fix: read syncWord from the RadioLibInterface member so side detectors
// inherit whatever syncword the main detector was actually configured with.
//
// Chip-side validation (BW>500 kHz + num>2, main SF>=10 + num>2,
// sumDetFactors*bw>=32E6, main SF>=side[i].sf, main SF constraint against
// side[i].sf range, duplicate SF detection) is done by RadioLib inside
// setSideDetector and surfaces as RADIOLIB_ERR_INVALID_SIDE_DETECT or
// RADIOLIB_ERR_WRONG_MODEM. The CLI handler (NexusCommands.cpp) catches
// those and translates to a friendly 'JT LR2021_SF: rejected by chip' reply.
//
// Static (CLI-side) validators stay in the CLI handler — range [5..12],
// distinctness, spread<=4 — so operators see granular error messages
// ('SF13 out of range [5..12]', 'duplicate SF7', 'spread SF5..SF10
// exceeds 4') instead of a single 'reject'.
//
// Pre-refactor (commits 7e1c908f9 / 6cae862b7) we hand-rolled the SPI
// transaction against the Semtech opcode 0x0224 + a ZephCore-derived
// byte-encoder. The refactor (commit 7ef6a6837) drops ~120 lines of
// duplicated code; RadioLib already does the same wire transactions
// with the same opcodes and same byte layout, just bundled with the
// syncword side (0x0225) that we were missing anyway.

bool LR2021Interface::configureSideDetectors(const uint8_t *sfs, uint8_t num)
{
    // Disable path — RadioLib handles nullptr-or-zero cleanly: returns
    // RADIOLIB_ERR_NONE which we map to true here.
    if (num == 0) {
        return lora.setSideDetector(nullptr, 0) == RADIOLIB_ERR_NONE;
    }
    if (num > MAX_SF_TABLE) {
        return false;
    }
    LR2021LoRaSideDetector_t sideDet[MAX_SF_TABLE];
    for (uint8_t i = 0; i < num; i++) {
        sideDet[i].sf = sfs[i];
        sideDet[i].ldro = false;
        sideDet[i].invertIQ = false;
        // Use the main detector's syncword (default 0x2B, or whatever the
        // operator set via /nexus SETSYNCWORD, vault v40). Reading directly
        // from the RadioLibInterface member ensures the side detectors
        // always match the channel's syncword — the chip-actual byte.
        sideDet[i].syncWord = syncWord;
    }
    return lora.setSideDetector(sideDet, num) == RADIOLIB_ERR_NONE;
}

// Nexus-28005 (2026-09-13): single source of truth for the side-
// detector apply. Called from all three reconfiguration paths in
// LR20x0Interface (init, reconfigure, fullBegin) so /nexus SETLORA
// re-programs the side detector no matter which internal code path
// setlora triggers. Before this commit, the apply hook only lived in
// init() — but setlora goes through reconfigure()/fullBegin(), not
// init(), so the apply never fired when operators changed LoRa params
// after setting side detectors. Symptom: /nexus LR2021_SF 7 + SETLORA
// left the chip in single-SF RX.
//
// No-op when the table is empty (boot default — RAM-only persistence
// means storage starts at 0 after every reboot). If the chip rejects
// the table, clears the stored table so we don't try to re-apply a
// phantom entry on every subsequent reconfigure.
void LR2021Interface::applyStoredSideDetectors()
{
    if (sideSFsCount_ == 0) {
        return;
    }
    if (configureSideDetectors(sideSFs_, sideSFsCount_)) {
        LOG_INFO("LR2021 side detectors applied (n=%u)", (unsigned)sideSFsCount_);
    } else {
        LOG_WARN("LR2021 side detectors rejected by chip (n=%u) — falling back to single-SF",
                 (unsigned)sideSFsCount_);
        clearSideDetectors();
    }
}

// Nexus-28005 (2026-09-12): /nexus LR2021_SF <list> handler.
//
// Validates and stores the operator-configured side-detector SF table in
// RAM. The apply hook in LR20x0Interface::init() picks up the table on
// the next radio init (which happens on /nexus SETLORA, the role/region
// change, the LR2021_DIO_AS_RF_SWITCH toggle, or any other path that
// re-runs init()). Storage is RAM-only — operators re-set after reboot.
//
// Returns true on accepted storage; false on validation failure (caller
// uses this to format an error reply).
void LR2021Interface::setSideDetectors(const uint8_t *sfs, uint8_t num)
{
    // Defensive CLI-side validation. The CLI handler in NexusCommands.cpp
    // pre-validates with the same rules and emits per-constraint error
    // replies; this second pass keeps the apply hook safe if a future
    // caller bypasses those.
    if (num > MAX_SF_TABLE) return;
    if (num == 0) {
        sideSFsCount_ = 0;
        return;
    }
    uint8_t lo = 12, hi = 5;
    for (uint8_t i = 0; i < num; i++) {
        const uint8_t sf = sfs[i];
        if (sf < 5 || sf > 12) return;
        for (uint8_t j = i + 1; j < num; j++) {
            if (sfs[j] == sf) return;
        }
        if (sf < lo) lo = sf;
        if (sf > hi) hi = sf;
    }
    if ((hi - lo) > 4) return;
    for (uint8_t i = 0; i < num; i++) {
        sideSFs_[i] = sfs[i];
    }
    sideSFsCount_ = num;
}
#endif
