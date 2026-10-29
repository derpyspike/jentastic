# Code review: Nexus-28101 submodule state, and Nexus-28104 → Nexus-28105

Branch: `Nexus-28105-codereview-fixes`, based on `Nexus-28105`.

## Scope

Two review passes:

1. `Nexus-28101` working tree: the uncommitted `protobufs` submodule pointer.
2. `Nexus-28104`..`Nexus-28105` (31 commits, 79 files changed): detailed review of the diff.

## Pass 1: `protobufs` submodule

The checked-out `protobufs` submodule in this clone was at `6b1ded439`, an ancestor of the
commit (`8f97d66a6`) every branch's tree expects. Not a code change — the clone's submodule
checkout had simply never been advanced. Resolved locally with `git submodule update`; no
source change.

## Pass 2: `Nexus-28104`..`Nexus-28105` — findings and fixes

| # | File:line | Issue | Fix |
|---|-----------|-------|-----|
| 1 | `SX126xInterface.cpp:621`, `SX128xInterface.cpp:509` | JenCore's preamble-length override applied to any raw send (`isRawSending()`), so it also hit RNode raw frames when both features are enabled, corrupting on-air RNode/Reticulum frames. | Gated on `getRawSendingType() == RAW_SEND_JENCORE` instead of `isRawSending()`. |
| 2 | `SX126xInterface.cpp:856`, `LR11x0Interface.cpp:741`, `LR20x0Interface.cpp:1064` | `resetAGC()` gated on the stale `sendingPacket != NULL`, which misses raw (JenCore/RNode) sends — AGC reset could fire mid-transmission of a raw frame. | Switched to `isSending()` (`sendingPacket != NULL \|\| isRawSending()`), already defined on the base class. |
| 3 | `JenCoreRepeater.cpp:1255` (`queueOwnFrame`) | After `scopeOwnFrame()` prepended a 4-byte transport code, `FrameInfo` was re-parsed against the scoped buffer but the hash was still computed over the original unscoped `raw`/`len`, so the own-frame hash never matched the hash a neighbour computes on hearing the same (scoped) on-air frame — defeats own-frame loop prevention. | Hash now computed over `framed`/`framedLen` (whichever buffer `info` was actually parsed from). |
| 4 | `NRF52CryptoEngine.cpp` (`nrf52CryptoCellIdleCheck`) | CryptoCell active/idle state is shared between the main-loop idle sweep and crypto calls made under `cryptLock` (Router encode/decode, KeyVerificationModule, AdminModule — including from the BLE SoftDevice task). The idle sweep didn't take `cryptLock`, so it could call `nRFCrypto.end()` concurrently with an in-flight CryptoCell operation. | `nrf52CryptoCellIdleCheck()` now takes `cryptLock` (existing global, declared in `CryptoEngine.h`) for its check-and-end critical section. `cryptLock` is non-recursive, so it is **not** added inside `nrf52CryptoCellAcquire()` or `encryptAESCtr()` — those already run under a caller-held `cryptLock` on every path that can race with the idle sweep. |
| 5 | `RadioInterface.h:347` vs `RadioLibInterface.h:226` | Base `getSyncWord()` was declared non-`const`; `RadioLibInterface`'s override is `const`. The const mismatch meant it hid rather than overrode the base — callers through a `RadioInterface*` got the base's hardcoded `0` on every subclass except `LR20x0Interface` (which happened to also be non-`const`, and so *was* overriding the old base). | Base made `const`. `LR20x0Interface::getSyncWord()` (declaration + definition) made `const` to match — its body (`effectiveMainSyncword()`) was already `const`. |
| 6 | `JenCoreRepeater.cpp:685` (`handleTrace`) | Hop count is stored in the low 6 bits of the `path_len` byte (`MC_PATH_COUNT_MASK`); the top 2 bits encode hash size. The increment did `raw[pathLenAt] + 1` on the whole byte. The entry guard allowed `path_hash_count` up to 63, so a count of 63 incrementing to 64 overflowed into the hash-size bits, corrupting the route for every downstream receiver. | Guard tightened to reject `path_hash_count >= MC_PATH_COUNT_MASK` (63), and the increment now masks to the count subfield only, preserving the hash-size bits. |
| 7 | `RNodeEngine.cpp:265` | Promiscuous-mode TX accepted frames up to `MAX_RADIO_FRAME_LEN` (256), but `RadioLibInterface::queueRNodeFrame()` rejects anything over `MAX_LORA_PAYLOAD_LEN` (255, the hard LoRa PHY limit) and the return value was ignored — a 256-byte frame vanished silently. | Bound tightened to `MAX_LORA_PAYLOAD_LEN`, with a `LOG_WARN` on rejection (either from the length check or from `queueRNodeFrame()` itself) instead of a silent drop. |
| 8 | `NexusTxHook.cpp:29` (`setOverride`/`setOverrideFull`) | When all 8 override slots were in use, eviction always picked slot 0 regardless of actual age, despite a comment claiming LRU. A 9th concurrent override could evict a packet still queued for transmission. | Added `NexusTxOverride::setAt` (`millis()` at last claim). Extracted the shared slot-selection logic into `findOrEvictSlot()`, which now evicts the entry with the oldest `setAt` when no free/matching slot exists. |
| 9 | `JenCoreContacts.h:119` (`touch`) | `lastUsed` is stamped from a monotonically increasing 32-bit counter; `0` doubles as the free-slot sentinel used by every lookup and by `victim()`. On the 2^32nd `touch()`/`upsert()`, the touched contact's `lastUsed` becomes `0`, making it invisible to lookups and immediately re-claimable as a free slot. | `touch()` now skips `0` on counter wraparound (`if (++clock == 0) clock = 1;`), so a genuinely touched contact can never collide with the sentinel. |

### Not fixed (left as-is per explicit instruction)

`docs/nexus-split-jencore-rnode-msf-hybrid.md` and `docs/multi-protocol-user-guide.md`, added in
this range, conflict with this repo's `CLAUDE.md` house rule against in-tree design/user docs
(content belongs in the separate docs repo, per #11488). Left untouched on this branch; still
needs a decision on the actual `Nexus-28105` branch.

## Verification

- `git submodule update` brings `protobufs` to the commit every branch's tree already expects.
- `./bin/run-tests.sh` (both in the plain shell and under `nix develop`) fails before any suite runs:
  `coverage:test_utf8` errors on `#include "yaml-cpp/yaml.h"` (`PortduinoGlue.h`), and `pkg-config`/`pip`
  are missing from this machine's environment even inside this repo's own devshell. The failing compile
  units (`test/TestUtil.cpp`, `src/DebugConfiguration.cpp`) are not among the files touched by any of
  the 9 fixes below, so this is a local devshell gap rather than something introduced here - but it was
  not re-confirmed against a clean `Nexus-28105` checkout on this machine. Each fix was instead verified
  by manual trace against the existing call graph and established patterns in this codebase (e.g.
  `isSending()`, the `cryptLock` contract documented in `CryptoEngine.h`, `MC_PATH_COUNT_MASK`'s bit
  layout) - not by a green automated test run.
