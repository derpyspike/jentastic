# Nexus Manual — Channel, Keys & Build Security

This manual covers the parts of Nexus that are **not** command syntax:
how commands reach a node, what the trust keys are, and how to build a
firmware that carries your own keys. For the command reference itself see
[`nexus-commands.md`](nexus-commands.md).

---

## 1. The Nexus channel & how commands are dispatched

Nexus commands travel over a normal mesh channel. It must be one of the trusted channels. You send a text message that starts with a dispatch prefix:

| Prefix | Scope | Description |
|--------|-------|-------------|
| `/nexus <cmd>` | Broadcast | Sent over the primary/first configured Nexus channel; every node on that channel may process it. Broadcast commands carry a **5-second rate limit** (per node). |
| `/nexus-local <cmd>` | Local | Executed on the sending node only. |
| `/nexus-device !<hex8> <cmd>` | Targeted | Sent to one node by its hex node ID (e.g. `/nexus-device !defc895b INFO`). Bypasses the broadcast rate limit. |
| `/nexus-node <short4> <cmd>` | Targeted | Sent to one node by its 4-char short name. Bypasses the rate limit. |
| `/nexus-mac <hex2> <cmd>` | Targeted | Sent by the last 2 bytes of the MAC address. |
| `/nexus-group <grp> <cmd>` | Group | Sent to all nodes in group `<grp>`. |
| `/jt`, `/jent`, `/jentastic` | /nexus-* Aliases | Equivalent to `/nexus`. |

Replies are queued back over the same channel the command arrived on.
Broadcast commands are routed on the node's **trusted channel** (see §3)
so that responses land where operators listen.

> Full syntax and per-command arguments: [`nexus-commands.md`](nexus-commands.md).

---

## 2. Trust model — three key sources

A node decides whether an incoming packet is "an operator talking" by
checking the sender's public key against **three independent lists**
(`NexusModule::isAuthorizedAdmin`):

| # | Source | Where it lives | Mutable at runtime? |
|---|--------|----------------|---------------------|
| 1 | **Admin keys** — `config.security.admin_key[0..2]` | Node config, baked via `USERPREFS_USE_ADMIN_KEY_0/1/2` build flags or set from the app | Yes, via normal Meshtastic config |
| 2 | **Secondary keys** — `nexus_secondary_keys[0..3]` | Vault (on-flash, RAM-loaded) | **Yes** — `/nexus SKEY [1..4] <key\|OFF>` sets or wipes them |
| 3 | **Master keys** — `NEXUS_MASTER_KEY_1..4` | Compiled into the firmware binary | **No — irrevocable** (see §4) |

The same three lists are used by signature verification
(`verifyTrustedSignature`): an XEdDSA-signed packet is matched against
admin keys first (labels `A0–A2`), then secondary keys (`S0–S3`), then
master keys (`M1–M4`). Operators see these labels in trust indicators —
useful to know *which* key authorized an action.

### Why three layers?

- **Admin keys** = the normal upstream mechanism (per-device, set from
  the app).
- **Secondary keys** = the *revocable* Nexus layer. This is what
  community/shared builds use: grant trust over the air, revoke it with
  `SKEY n OFF`, no reflashing involved. Vault-persisted, so they survive
  reboot (`/nexus SAVE`).
- **Master keys** = the *irrevocable* floor — your own keys, you can bake them at
  compile time. Even if a node's config is wiped and its vault cleared,
  a firmware built with your master key still recognizes you.

---

## 3. Trusted channels

Two compile-time channel names (defaults **`Nexus`** / **`Nexus2`**,
override with `-DNEXUS_TRUSTED_CHANNEL_1/2=...`) plus two additional vault-managed
slots mark a channel as *trusted*:

- Broadcast command replies are routed there.
- Traffic on a trusted channel **bypasses the Nexus firewall drop
  rules** — your own management channel is never firewalled away.

Configure the channel name on the node (channel must exist and be
enabled), or bake the names in at compile time.

---

## 4. Master keys — the irrevocable layer

This feature is made for very specific usecases, if you want it you must compile your own binaries with these baked in.

**Important properties:**

- **No runtime wipe.** There is no command that clears them. `SKEY … OFF`
  only wipes *secondary* keys. As long as the firmware runs, the keys
  are trusted. They are burned into the firmware forever.
- **Rotation = reflash.** To change or remove a master key you must
  rebuild the firmware and reflash every node carrying it. This is
  deliberate: it is the one trust layer an attacker with mesh/vault
  access cannot alter.
- **Consequently: if you want master keys on your nodes, you must
  compile your own firmware.** Prebuilt/public binaries ship without
  *your* keys (and without anyone's — see §6).

**If a master key leaks:** you cannot un-leak it. Rebuild with the key
removed/rotated and reflash the affected nodes — then treat everything
signed by it up to that point as suspect.

This was a feature for my own personal nodes. If by any chance you wanna use, **consider really strongly if and how it fits into your security model.** Also bear in mind that baking in Admin Keys with the upstream userprefs is also possible and those can be changed at runtime unlike Master Keys

---

## 5. Compiling with master keys

### 5.1 Using `build-nexus.sh` (recommended)

Key names live in `nexus-keys-lib.sh`.
The builder resolves names → byte arrays → `-DNEXUS_MASTER_KEY_N` flags
and injects them via `PLATFORMIO_BUILD_FLAGS`.

The build-nexus.sh comes with a few examples from my own usecase.

```bash
# Default key set (4 slots):
./build-nexus.sh jenny

# Explicit slots, any known key name (max 4):
./build-nexus.sh jenny --master-key-1 eevee --master-key-2 mew

# Build with NO baked keys (community style):
./build-nexus.sh javi

# Restrict targets:
./build-nexus.sh jenny --master-key-1 eevee --targets heltec-v4
```

Builders and their key defaults:

| Builder | Repo | Keys |
|---------|------|------|
| `jenny` | jenny-firmware | default set (4 keys) unless overridden |
| `admins` | jt-firmware | none |
| `javi` | nexus-firmware | none |

Flags land in `PLATFORMIO_BUILD_FLAGS`, which `platformio.ini` picks up —
so any invocation route that exports those flags works.

### 5.2 By hand (plain PlatformIO)

The firmware builds **exactly like official Meshtastic firmware** (same
`platformio.ini`, same environments). To bake keys, export the flags
yourself before building:

```bash
export PLATFORMIO_BUILD_FLAGS='... -DNEXUS_MASTER_KEY_1="{ 0x00, 0x11, ... 0xff }"'
pio run -e heltec-v4 -t upload
```

(The byte array must be all 32 bytes, C brace syntax, quoted as shown —
`platformio.ini` parses the variable and PlatformIO strips the outer
quotes.)

### 5.3 Via `userPrefs.jsonc`

**Yes — master keys can be added through `userPrefs.jsonc`.**
`bin/platformio-custom.py` converts **every** key of the JSON file into
a compiler define; a value starting with `{` is passed through raw:

```jsonc
{
  // ...existing prefs...
  "NEXUS_MASTER_KEY_1": "{ 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff }"
}
```

becomes `-DNEXUS_MASTER_KEY_1={ 0x9a, ... }` on the compile line — the
`#ifdef NEXUS_MASTER_KEY_1` guards in `NexusModule.cpp` then fire and the
key joins `checkMasterKeys()`.

**Caveats — read before using this route:**

1. **The comment stripper removes `//`-to-end-of-line** from the whole
   file — fine for hex arrays, but never put URLs or `//` inside values.
2. Keys in `userPrefs.jsonc` apply to **every build from that tree** —
   there is no per-builder selection once they are in the file (use
   `--master-key-N` for per-build choice).
3. The file must remain valid JSONC after comment stripping (the build
   fails fast otherwise).

---

## 6. What prebuilt binaries contain

Public builds are compiled **without** master keys — slots 1–4
are simply `#ifdef`-empty, and `checkMasterKeys()` correctly falls
through to secondary keys (community path: `SKEY`). Node trust on such
builds comes from `admin_key[0..2]` (per-device config) and
`nexus_secondary_keys[0..3]` (runtime, configurable and revocable).

If you want your own irrevocable trust floor for some reason, build your firmware.
Also, this goes to show why trusting random binaries from the internet is generally a bad idea

---
