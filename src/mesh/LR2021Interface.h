#pragma once
#if RADIOLIB_EXCLUDE_LR2021 != 1
#include "LR20x0Interface.h"

/**
 * Our adapter for LR2021 radios
 */
class LR2021Interface : public LR20x0Interface<LR2021>
{
  public:
    LR2021Interface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                    RADIOLIB_PIN_TYPE busy);
    bool wideLora() override;

    // Nexus-28005 (2026-09-12): LR2021 multi-SF side-detector configuration.
    // Adapter for Semtech's LR20XX side-detector API (not exposed by upstream
    // RadioLib's LR2021 module yet). See implementation for license attribution.
    //
    // `sfs` points to an array of SF values 5..12 (one byte each, raw SF number,
    // not an enum). `num` is the count, max 3 (chip hardware limit). Pass num=0
    // to disable all side detectors. `syncWords` optionally provides an independent
    // syncword for each side detector (e.g. 0x12 for JenCore, 0x2B for Meshtastic,
    // or 0 to inherit the main detector's syncword).
    // Returns true on chip-level acceptance; false on validation failure
    // (num>3, out-of-range SF, duplicate SF, spread>4) or chip-side reject.
    bool configureSideDetectors(const uint8_t *sfs, uint8_t num, const uint8_t *syncWords = nullptr);

    // Storage + accessors used by /nexus LR2021_SF (NexusCommands.cpp) and
    // the apply hooks in LR20x0Interface::init(), ::reconfigure(), and
    // ::fullBegin(). RAM-only persistence (the table is lost on reboot;
    // operators re-set with /nexus LR2021_SF). A future vault bump could
    // persist this — Nexus-28005 keeps it RAM-only because the apply runs
    // before vault-load validation.
    static constexpr uint8_t MAX_SF_TABLE = 3;
    const uint8_t *sideDetectors() const { return sideSFs_; }
    const uint8_t *sideSyncWords() const { return sideSyncWords_; }
    uint8_t sideDetectorsCount() const { return sideSFsCount_; }
    void setSideDetectors(const uint8_t *sfs, uint8_t num, const uint8_t *syncWords = nullptr);
    void clearSideDetectors() { sideSFsCount_ = 0; }

    // Apply the stored side-detector table to the chip. Called from all
    // three reconfiguration paths in LR20x0Interface (init, reconfigure,
    // fullBegin) so /nexus SETLORA re-programs the side detector no matter
    // which internal code path setlora triggers (band-hop via fullBegin,
    // same-band via per-parameter setters, or recovery via fullBegin).
    // No-op when sideDetectorsCount() == 0 (boot default). If the chip
    // rejects the table, clears the stored table so we don't try to
    // re-apply a phantom entry on every subsequent reconfigure.
    void applyStoredSideDetectors();

    /// Apply the v70 multi-SF tier table (main syncword is handled by the
    /// driver; this programs the side detectors). Returns false on a hard
    /// validation/chip reject; a parked table (main SF temporarily above a
    /// side SF) returns true without touching the chip.
    bool applyMultiSFTiers();

    /// Disable the side detectors on the chip and clear the stored table.
    /// Used by TIER OFF.
    void clearMultiSFTiers();

  private:
    uint8_t sideSFs_[MAX_SF_TABLE] = {0};
    uint8_t sideSyncWords_[MAX_SF_TABLE] = {0};
    uint8_t sideSFsCount_ = 0;
};
#endif
