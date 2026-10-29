<div align=center>

# Jentastic/Nexus Commands Reference (Public Build)

</div>

## Table of Contents
1. [The Nexus Channel & Invocations](#1-the-nexus-channel--invocations)
2. [Help & Build Info](#2-help--build-info)
3. [Firewall & Traffic Filtering](#3-firewall--traffic-filtering)
4. [Zero-Hop Routing (ZH)](#4-zero-hop-routing-zh)
5. [Security & Features](#5-security--features)
6. [Hop-Aware Routing & Masks](#6-hop-aware-routing--masks)
7. [Radio Configuration & RF Control](#7-radio-configuration--rf-control)
8. [Identity, Naming & NodeDB](#8-identity-naming--nodedb)
9. [Network Observation, Scanning & SIGINT](#9-network-observation-scanning--sigint)
10. [Target Watchlist (In-Depth Guide)](#10-target-watchlist-in-depth-guide)
11. [Vault & Storage Persistence](#11-vault--storage-persistence)
12. [Channels & Configuration Management](#12-channels--configuration-management)
13. [Hardware, Sensors & Telemetry](#13-hardware-sensors--telemetry)
14. [Messaging, Routing & Alerts](#14-messaging-routing--alerts)
15. [Aliases, Macros, Groups & Automation](#15-aliases-macros-groups--automation)
16. [Filesystem & System Utilities](#16-filesystem--system-utilities)
17. [Complete Alphabetical Command Index](#17-complete-alphabetical-command-index)

---

## 1. The Nexus Channel & Invocations

The `/nexus` command engine processes commands received over Meshtastic mesh packets or via the local phone interface.

### Dispatch Prefixes

| Prefix | Target Scope | Description & Routing |
|--------|--------------|-----------------------|
| `/nexus <cmd>` | Broadcast | Broadcast over the primary or first configured trusted channel. |
| `/nexus-device !<hex8> <cmd>` | Targeted Node | Directly targeted to a node by hex ID (e.g. `/nexus-device !defc895b INFO`). Bypasses broadcast cooldown. |
| `/nexus-node <short> <cmd>` | Targeted Node | Directly targeted to a node by 4-char short name (e.g. `/nexus-node JNT1 RSSI`). Bypasses broadcast cooldown. |
| `/nexus-mac <mac2> <cmd>` | Targeted MAC | Targeted by the **last 2 bytes** of the MAC as a hex suffix (e.g. `/nexus-mac 1a2b INFO`). A full 6-byte MAC will not match. |
| `/nexus-group <grp> <cmd>` | Group Target | Targeted to all nodes assigned to `<grp>`. |
| `/nexus-local <cmd>` | Local Interface | Executes locally without retransmitting over the mesh. |
| `/jt`, `/jent`, `/jentastic` | Alias Roots | Fully equivalent aliases for `/nexus`. |

### Command Redirection (`/nexus R`)
Any `/nexus` command can be redirected to output its response to a specific channel or direct message (DM):

| Syntax | Example | Description |
|--------|---------|-------------|
| `/nexus R CH <id\|name> <cmd>` | `/nexus R CH 1 NODES` | Executes command and routes reply to channel `<id>` or channel `<name>`. |
| `/nexus R DM <!id\|name> <cmd>` | `/nexus R DM !defc895b SYS` | Executes command and routes reply as a private DM to the target node. |

*Safety rule:* Nested redirect depth is capped at `NEXUS_MAX_REDIRECT_DEPTH` (3). Redirection (`R` / `REDIRECT`, subcommands `CH` / `CHAN` and `DM`) is only recognized at the top level — a nested `R ...` inside a redirected command is not processed (known limitation).

---

## 2. Help & Build Info

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `HELP` | `HELP` | Opens paged general help index listing categories. | `JT: Paging help categories...` |
| | `HELP <topic>` | Detailed help for: `DROPS`, `FW`, `DROP`/`FIREWALL`/`OUTDROP`, `FEATURES`, `SIGINT`/`SCAN`, `ADMIN`/`SYSTEM`/`BD`, `VOLT`/`VOLTAGE`, `SPLIT`, `APC`, `FSIG`, `PPING`, `SKEY`, `LORA`/`PRESET`/`SETLORA`, `GPIO`, `TEXT`/`FTEXT`/`INJECT`, `IDENTITY`/`NAME`, `TM`, `RL`/`RATELIMIT`, `RLS`/`RATELIMITSTAT`, `PL`/`RSSILOG`, `CRON`, `NIGN`/`IGNORE`, `NODE`/`TARGET`. (No `?` alias.) | `JT: Paging <topic> help...`<br>(variants: `JT: Paging DROPS list help...`, `JT: Paging FIREWALL help...` for DROP/FIREWALL/OUTDROP, `JT: Paging ADMIN/SYS help...` for BD) |
| `INFO` | `INFO` | Displays device ID, short name, firmware version, active role, and MAC address. | `JT INFO:`<br>`!<id> [<short>]`<br>`Ver: <ver>`<br>`Role: <role>`<br>`MAC: <aa:bb:cc:dd:ee:ff>` |
| `COMPILATION` / `COMP` | `COMPILATION` | Displays target platform, architecture, source repository, and enabled compile-time feature flags. | `JT COMPILATION:`<br>`Ver: <ver>`<br>`Tgt: <target> (<arch>)`<br>`Repo: <repo>`<br>`Built: <date>`<br>`Flags: <flags>` |
| `VERSION` / `V` | `VERSION` | Displays raw firmware version string from metadata. | `JT VERSION: <firmware_version>` |

---

## 3. Firewall & Traffic Filtering

The Nexus Firewall provides granular inbound and outbound drop rules, rate limiting, channel filtering, signature matching, and favorite bypasses.

### 3.1 Drop Toggles & Bulk Hex Configuration

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `DROP` | `DROP <TAG>` | Enable dropping of packets matching `<TAG>`. Example: `DROP TR`. | `JT: <TAG> filter ACTIVE` |
| `UNDROP` | `UNDROP <TAG>` | Disable dropping of packets matching `<TAG>`. Example: `UNDROP TR`. | `JT: <TAG> filter DISABLED` |
| `OUTDROP` | `OUTDROP <TAG>` | Enable outbound (TX) dropping for `<TAG>` (`NI`, `PO`, `TL`, `TR`, `AD`, `RO`). Example: `OUTDROP PO`. | `JT Output: <TAG> filter ACTIVE` |
| `UNOUTDROP`| `UNOUTDROP <TAG>` | Disable outbound dropping for `<TAG>`. Example: `UNOUTDROP PO`. | `JT Output: <TAG> filter DISABLED` |
| **`DROPS`** | `DROPS` | Lists all currently active inbound drop tags (including `TO`, `NM` and the `RL_*` flags), whitelists, and blacklists. (200-byte reply buffer — a huge active-tag list truncates; `HEXDROPS` shows the full mask.) | `JT ACTIVE DROPS:`<br>`<tag1> <tag2>...`<br>`WL: 0xXX...`<br>`BL: 0xYY...` |
| `OUTDROPS`| `OUTDROPS` | Lists all currently active outbound drop tags. | `JT ACTIVE OUTDROPS:`<br>`<tag1> <tag2>...` |
| `HEXDROPS`| `HEXDROPS` | Displays active inbound drops as a 64-bit hexadecimal bitmask. | `JT HEXDROPS: 0x<HEX16>` |
| `HEXOUTDROPS`| `HEXOUTDROPS` | Displays active outbound drops as a 64-bit hexadecimal bitmask. | `JT HEXOUTDROPS: 0x<HEX16>` |
| `SETHEX` | `SETHEX <hex>` | Set all inbound drop rules directly from a 64-bit hex mask. Example: `SETHEX 0x0000000000000001`. | `JT: Drops updated to 0x<HEX16>` |
| `SETHEXOUT`| `SETHEXOUT <hex>` | Set all outbound drop rules directly from a 64-bit hex mask. | `JT: Output Drops updated to 0x<HEX16>` |
| `FIREWALL` / `FW` | `FIREWALL` / `FW` | Paged aggregated status of inbound drops, outbound drops, hex masks, invalid filters, and PR lists. | `JT: Paging firewall status...` |
| | `FW <sec>` | Query a single firewall section (`DROPS`, `OUTDROPS`, `INVALIDS` — all five delegate to the section handler). | Delegates to the section handler. |

### 3.2 Recognized `DROP` / `UNDROP` Tags

| Tag | Alias | Target Category | Description |
|-----|-------|-----------------|-------------|
| `NODEINFO` | `NI` | Port `NODEINFO_APP` | Drops all incoming NodeInfo packets. |
| `POSITION` | `PO` | Port `POSITION_APP` | Drops all incoming Position packets. |
| `TELEMETRY` | `TL` | Port `TELEMETRY_APP` | Drops all incoming Telemetry packets. |
| `NEIGHBOR` | `NB` | Port `NEIGHBORINFO_APP` | Drops NeighborInfo packets. |
| `TRACEROUTE` | `TR` | Port `TRACEROUTE_APP` | Drops all TraceRoute packets. |
| `TRACEROUTE_REMOTE` | `TRR` | Port `TRACEROUTE_APP` | Drops TraceRoute packets where our node is a relay (protects path tracing). |
| `TRACEROUTE_LOCAL` | `TRL` | Port `TRACEROUTE_APP` | Drops TraceRoute packets addressed to or from our node (invisibility). |
| `TRACEROUTE_DIRECT` | `TRD` / `TR_DIRECT` | Port `TRACEROUTE_APP` | Drops zero-hop direct TraceRoutes. |
| `TRACEROUTE_DIRECT_REMOTE` | `TRDR` / `TR_DIRECT_REMOTE`| Port `TRACEROUTE_APP` | Drops direct TraceRoutes where we are a relay. |
| `TRACEROUTE_DIRECT_LOCAL` | `TRDL` / `TR_DIRECT_LOCAL`| Port `TRACEROUTE_APP` | Drops direct TraceRoutes to/from our node. |
| `PRIVATE` | `PR` | Encrypted Payload | Drops encrypted packets from unknown channels. |
| `HOP6` / `HOP7` | `H6` / `H7` | Packet Header | Drops packets arriving with `hop_start` (configured max hops) equal to 6 or 7 respectively. |
| `REQ_NODEINFO` | `RNI` | Protocol Requests | Drops NodeInfo requests (`want_response = true`). |
| `REQ_TELEMETRY`| `RTL` | Protocol Requests | Drops Telemetry requests (`want_response = true`). |
| `REQ_POSITION` | `RPO` | Protocol Requests | Drops Position requests (`want_response = true`). |
| **`BR_NODEINFO`** | **`BRNI`** | Broadcast Requests | Drops **broadcast** NodeInfo requests **(Important for the Spanish mesh)**. |
| `BR_TELEMETRY`| `BRTL` | Broadcast Requests | Drops broadcast Telemetry requests. |
| `BR_POSITION` | `BRPO` | Broadcast Requests | Drops broadcast Position requests. |
| `ACKS` | `AK` | Port `ROUTING_APP` | Drops routing acknowledgments. |
| `PKI` | `PK` | Direct Messages | Drops all PKI encrypted direct messages. |
| `PKI_REMOTE` | `PKR` | Direct Messages | Drops remote PKI packets not addressed to/from this node. |
| `PKI_LOCAL` | `PKL` | Direct Messages | Drops local PKI packets addressed to or from this node. |
| `PREHOP` | `PH` | Packet Header | Drops packets with corrupted or impossible `hop_start` metrics. (Not really needed in 2.8) |
| `STORE_FORWARD` | `SF` | Port `STORE_FORWARD_APP` | Drops store-and-forward repetitions. |
| `ADMIN` | `AD` | Port `ADMIN_APP` | Drops Admin messages. |
| `TEXT` | `TX` | Port `TEXT_MESSAGE_APP` | Drops plain text messages. |
| `KEYVER` | `KV` | Port `KEY_VERIFICATION_APP`| Drops Key Verification packets. |
| `ALERT` | `AL` | Port `ALERT_APP` | Drops emergency / alert packets. |
| `WAYPOINT` | `WP` | Port `WAYPOINT_APP` | Drops Waypoint broadcast packets. |
| `NO_MQTT` | `NM` | Bitfield | Drops packets without `OK_TO_MQTT` bit. |
| `FAV_BYPASS` | `FB` | Global Bypass | When ON, favorite nodes bypass all firewall drops. |
| `SKIP_SKIP` | `SS` | Opaque Relay | Honors `skip_skip_handle` toggle for opaque relays. |
| `RL_TEXT` | `RLTX` | Rate-Limit | Drops text packets over the `TEXT` rate-limit bucket. |
| `RL_TRACE` | `RLTR` | Rate-Limit | Drops traceroute packets over the `TRACE` bucket. |
| `RL_OTHER` | `RLOT` | Rate-Limit | Drops other packets over the `OTHER` bucket. |
| `RL_NODEINFO` | `RLNI` | Rate-Limit | Drops NodeInfo over the `NODEINFO` bucket. |
| `RL_POSITION` | `RLPO` | Rate-Limit | Drops Position over the `POSITION` bucket. |
| `RL_TELEMETRY` | `RLTL` | Rate-Limit | Drops Telemetry over the `TELEMETRY` bucket. |
| `RL_PRIVATE` | `RLPR` | Rate-Limit | Drops Private over the `PRIVATE` bucket. |
| `TELE_DEVICE` | `TD` | Telemetry Sub-Filter | Drops device telemetry subset. |
| `TELE_POWER` | `TP` | Telemetry Sub-Filter | Drops power telemetry subset. |
| `TELE_ENV` | `TE` | Telemetry Sub-Filter | Drops environment telemetry subset. |
| `TELE_OTHER` | `TO` | Telemetry Sub-Filter | Drops remaining telemetry subsets. |
| `POS_MANUAL` | `PM` | Position Sub-Filter | Drops manual positions. |
| `POS_GPS` | `PG` | Position Sub-Filter | Drops GPS positions. |
| `POS_OTHER` | `POO` | Position Sub-Filter | Drops other position sources. |

### 3.3 Channel Whitelists, Blacklists & Invalids
This allows you to filter specific private channels. Only works for opaque channels, in other words, channels the node doesn't have configured, such as foreign private channels.
Whitelist becomes active when it isn't empty. So adding any channels to the whitelist makes the Opaque firewall whitelist only.

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `PRWHITELIST` / `PRALLOW` | `PRWHITELIST ADD <ch_hash>` | Allow only private channels matching hash (channels 8+). | `JT: Ch <hash> (0x<hex>) added to Whitelist` |
| | `PRWHITELIST REM <ch_hash>` | Remove channel from whitelist. | `JT: Ch <hash> (0x<hex>) removed from Whitelist` |
| | `PRWHITELIST LIST` | List all active whitelisted channel hashes. | `JT Whitelist: ` + `<dec>(0x<hex>) ` per entry |
| `PRBLACKLIST` / `PRBLOCK` | `PRBLACKLIST ADD <ch_hash>` | Block private channels matching hash. Example: `PRBLOCK ADD 0xC3`. | `JT: Ch <hash> (0x<hex>) added to Blacklist` |
| | `PRBLACKLIST REM <ch_hash>` | Remove channel from blacklist. | `JT: Ch <hash> (0x<hex>) removed from Blacklist` |
| | `PRBLACKLIST LIST` | List all active blacklisted channel hashes. | `JT Blacklist: ` + `<dec>(0x<hex>) ` per entry |
| `INVALID` / `INVALIDS` / `INV` | bare | Display the five packet validation rules: ON/OFF state plus each rule's drop counter (in parentheses). | `JT INVALID Rules:`<br>`NULL:<ON\|OFF> (<n>) SPAIN:<ON\|OFF> (<n>)`<br>`GERM:<ON\|OFF> (<n>) HW:<ON\|OFF> (<n>)`<br>`ID:<ON\|OFF> (<n>)` |
| `INVALID <RULE>` / `UNINVALID <RULE>` (aliases `INV <RULE>` / `UNINV <RULE>`) | Enable / disable one rule — there is **no** `[ON\|OFF]` argument: `INVALID <RULE>` turns it on, `UNINVALID <RULE>` turns it off. | Rule words: `NULL`/`NULL_ISLAND`, `SPAIN`, `GERMANY` (status line abbreviates it `GERM`), `HW_UNSET`/`HW`, `ID_MISMATCH`/`ID`. Exact drop behavior for each rule is described below. | `JT: INVALID <RULE> <ACTIVE\|DISABLED>` |

#### Invalid validation rules — what each rule drops

All five rules are **OFF by default**. They live inside `shouldDropPacket()` as step **3c** and evaluate **decoded (unencrypted) packets only** — they run after the favorite bypass, Nexus ignore list, relay-drop mask, pre-hop validation, PKI filter, hop-congestion/ACK/request filters and rate limiting (3b), and fire before the port-based `DROP` rules (step 4). A packet matching any enabled rule is dropped like any other firewall drop: not relayed, not fed to apps, not inserted in the NodeDB. Favorite nodes have already bypassed the gate earlier (step 0), so they are exempt. The rule mask persists in the vault as `invalid_mask` (schema v29) via `/nexus SAVE`; the `(<n>)` counters are RAM-only session statistics.
Bear in mind the Spain and Germany filters are very rough boxes so they are only approximate.

| Rule | Aliases | Applies to | Drops the packet when | Exact test |
|------|---------|------------|-----------------------|------------|
| `NULL` | `NULL_ISLAND` | `POSITION` | Coordinates are Null Island: exactly `0,0` **or** within ±0.01° of it (≈1.1 km box around 0,0). This rule is the origin of the whole filter — a bug injector was spamming nodes at (0,0). ⚠️ While ON, legitimate "no fix" position reports (`0,0`) are dropped too. | `latitude_i == 0 && longitude_i == 0`, or `fabs(lat) < 0.01 && fabs(lon) < 0.01` |
| `SPAIN` | — | `POSITION` | Reported position falls outside **both** Spain boxes: Peninsula `35.0–44.0°N, 10.0°W–5.0°E` and Canaries `27.0–30.0°N, 19.0–13.0°W`. Anything outside (Portugal, France, Morocco, spoofed coordinates…) is dropped. | drop unless the position is inside the Peninsula box **or** the Canaries box |
| `GERMANY` (status: `GERM`) | — | `POSITION` | Reported position falls outside the Germany box `47.0–55.5°N, 5.5–15.5°E`. | drop unless `47.0 <= lat <= 55.5` **and** `5.5 <= lon <= 15.5` |
| `HW_UNSET` | `HW` | `NODEINFO` | The NodeInfo's `hw_model` is `HardwareModel_UNSET` — the sender never declared hardware (uninitialized, corrupt, or synthetic NodeInfo). | `user.hw_model == HardwareModel_UNSET` |
| `ID_MISMATCH` | `ID` | `NODEINFO` | The `!hex` ID inside the NodeInfo payload decodes to a **non-zero** value that differs from the packet header's `from` address — a forged NodeInfo claiming to be another node. A payload ID of `0` is skipped (treated as unset). | `payloadId != 0 && payloadId != mp.from` |

*Position rules decode the `meshtastic.Position` protobuf and compute `lat/lon = latitude_i/longitude_i ÷ 1e7`; NodeInfo rules decode `meshtastic.User`. The status line abbreviates Germany as `GERM`, but the parser only accepts the word `GERMANY`. `INVALIDS` also appears as a section of the aggregated `FIREWALL` report.*


### 3.4 Relay Dropping & Duplicate Relay Suppression
RDROP drops packets relayed by a specific relay_node.
IDR is used on Clients and Client Bases to ignore the dupes from a speciifc relay_node, for example when a wronfully configured router node that isn't community coordinated breaks the managed flooding, with IDR you can nullify that effect.

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `RDROP` | `RDROP` | Displays list of dropped relay node IDs (last byte mask). | `JT RDROP: ` + `*<hex> ` per entry |
| | `RDROP <hex>` | Add relay ID (byte) to drop mask. Accepts `*`-prefixed or bare hex (`RDROP A4`, `RDROP *A4`). | `JT: Relay *<hex> DROPPED` |
| `RUNDROP` | `RUNDROP <hex>` | Remove relay ID from drop mask. | `JT: Relay *<hex> ALLOWED` |
| | `RDROP CLEAR` | Clear all dropped relay node IDs. | `JT: Relay Drop Mask CLEARED.` |
| `IDR` / `IGNORE-DUPE-RELAY` | `IDR` | Displays the per-relay duplicate-suppression mask (list of relay IDs). | `JT IDR: ` + `*<hex> ` per entry |
| | `IDR <hex>` | Add relay ID to the ignore-dupe mask. | `JT: Relay *<hex> added to IDR` |
| | `IDR CLEAR` | Clear the whole IDR mask. | `JT: IDR Mask CLEARED.` |
| `UNIDR` | `UNIDR <hex>` | Remove one relay ID from the IDR mask (takes a hex argument). | `JT: Relay *<hex> removed from IDR` |

### 3.5 Target Favorites & Ignore Lists

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `FAV` | `FAV <!id\|name>` | Mark node as favorite in NodeDB. | `JT: Updated node !<id> (<short>)` |
| `UNFAV` | `UNFAV <!id\|name>` | Unmark node as favorite. | `JT: Updated node !<id> (<short>)` |
| `FFAV` | `FFAV <!id\|name>` | Force favorite node even if not yet in NodeDB. | `JT: Forced node !<id> (<short>)` |
| `FUNFAV` | `FUNFAV <!id\|name>` | Force remove favorite node. Creates the node if missing. | `JT: Forced node !<id> (<short>)` |
| `FAVS` | `FAVS` | Lists all active favorite nodes with short names (paged). | Paging `Favorites`:<br>`!<id>:<short> ` per entry<br>Reply: `JT: Paging favorites to mesh...` |
| `IGNORE` | `IGNORE <!id\|name>` | Add node to NodeDB ignore list. | `JT: Updated node !<id> (<short>)` |
| `UNIGNORE` | `UNIGNORE <!id\|name>` | Remove node from NodeDB ignore list. | `JT: Updated node !<id> (<short>)` |
| `FIGNORE` | `FIGNORE <!id\|name>` | Force ignore node ID. | `JT: Forced node !<id> (<short>)` |
| `FUNIGNORE`| `FUNIGNORE <!id\|name>` | Force unignore node ID. | `JT: Forced node !<id> (<short>)` |
| `IGNORED` | `IGNORED` | Lists all currently ignored nodes (paged). | Paging `Ignored`:<br>`!<id>:<short> ` per entry<br>Reply: `JT: Paging ignored to mesh...` |
| **`NIGN`** | `NIGN` | Shows Nexus Ignore list (Extra Ignore List in addition to the upstream NodeDB one) — paged. | Paging `JT Ignores`:<br>`[<n>] !<id> (<short>)` per entry<br>Reply: `JT: Paging ignore list (<n> nodes)...` |
| | `NIGN ADD <!id>` | Add node ID to Nexus Ignore table. | `JT: !<id> ADDED to ignores` / `JT: NIGN list full (64)` |
| | `NIGN REM <!id>` | Remove node ID from Nexus Ignore table (the subcommand is `REM`, **not** `DEL`; there is no `CLEAR`). | `JT: !<id> REMOVED` |
| `DELNODE` | `DELNODE <!id>` | Delete a node entry entirely from NodeDB. | `JT: Node !<id> removed from DB` |

### 3.6 Signature Firewall (`FSIG`)
Nexus includes an 8-slot signature firewall that scans incoming packet payloads for specific byte patterns and drops matches. Can be used to drop any arbitrary binary signature in packets.

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `FSIG` / `FSIG LIST` | `FSIG` | Pages all 8 signature slots showing active patterns, encoding format (hex, b64, text), and length in bytes. | `JT: Paging signatures...` |
| `FSIG SET` | `FSIG SET <1-8> <pattern>` | Sets a signature slot (auto-detects hex, base64, or text; max 16 bytes). Examples:<br>• Hex: `FSIG SET 1 DEADBEEF`<br>• Text: `FSIG SET 2 test_alert`<br>• Base64: `FSIG SET 3 3q2+7w==` | `JT: S<slot> set (<type>, <len>B).` |
| `FSIG OFF` | `FSIG OFF <1-8>` | Clears signature slot `<1-8>`. Example: `FSIG OFF 1`. | `JT: S<slot> cleared.` |

---

## 4. Zero-Hop Routing (ZH)
In JT you can do the Zero Hops features in Routers in two different ways, either the upstream way where you favorite routers in the NodeDB or the JT way where we have a 256 bit bitmask with every possible zero_hop relay_node
When you turn ZH on with this command the JT Zero Hop is active and overrides the upstream one, when it is OFF the upstream one works instead.
Lookup order when ZH is ON: the manual mask first, then the auto-router mask (if `ROUTER ON`), then the auto-client_base mask (if `CLIENT ON`) is checked. `ZH LIST`/`SHORT`/`PAGED` take an optional target — `MANUAL` (default), `ROUTER`, `CLIENT`, or `ALL` (= effective set: manual + enabled auto masks) — so all three lists are viewable independently, and `ZH LISTS` pages all three in one run with per-list headers (`[OFF]` tags a disabled auto list). Both auto masks are RAM-only and populated from mesh traffic — each packet heard **directly** (zero hops away, LoRa transport) looks its sender up in NodeDB (single lookup, no sweep) and sets the role bit: ROUTER/ROUTER_LATE into the router mask, CLIENT_BASE into the client mask — so clearing one just re-fills as traffic flows. Multi-hop, unknown-hop, and MQTT/UDP/API-forwarded senders are never added.

| Subcommand | Syntax | Description & Examples | Exact Output / Template |
|------------|--------|------------------------|-------------------------|
| `ON` | `ZH ON` | Enable Zero-Hop routing globally (sets `zero_hop_config & 0x01`). | `JT: ZH ON (POS:... TELE:... RTR:... CLI:...)` |
| `OFF` | `ZH OFF` | Disable Zero-Hop routing globally (clears `zero_hop_config & 0x01`). | `JT: ZH OFF (POS:... TELE:... RTR:... CLI:...)` |
| `POS ON` / `POS OFF` | `ZH POS ON`<br>`ZH POS OFF` | Toggle Position-app preservation for Zero-Hop nodes (sets/clears `zero_hop_config & 0x04`). | `JT: ZH ... (POS:ON TELE:...)` |
| `TELE ON` / `TELE OFF` | `ZH TELE ON`<br>`ZH TELE OFF` | Toggle Telemetry-app preservation for Zero-Hop nodes (sets/clears `zero_hop_config & 0x02`). | `JT: ZH ... (POS:... TELE:ON)` |
| `ROUTER ON` / `ROUTER OFF` (alias `RT`) | `ZH ROUTER ON`<br>`ZH ROUTER OFF` | Consult the auto-router mask (ROUTER / ROUTER_LATE roles, RAM-only, auto-populated from directly-heard zero-hop packets via a single NodeDB lookup) after a manual-mask miss (sets/clears `zero_hop_config & 0x08`). | `JT: ZH ... (... RTR:ON[<n>] CLI:...)` |
| `CLIENT ON` / `CLIENT OFF` (aliases `CB` / `CLIENT_BASE`) | `ZH CLIENT ON`<br>`ZH CLIENT OFF` | Consult the auto-client mask (CLIENT_BASE role, RAM-only, auto-populated from directly-heard zero-hop packets via a single NodeDB lookup) after the router mask (sets/clears `zero_hop_config & 0x10`). | `JT: ZH ... (... RTR:... CLI:ON[<n>])` |
| `ADD <id>` | `ZH ADD <hex>` | Add a node's last-byte ID (0x00-0xFF) to the Zero-Hop mask. Example: `ZH ADD C3`. | `JT: ZH ID 0xC3 ADDED` |
| `DEL <id>` | `ZH DEL <hex>` | Remove a node from the Zero-Hop mask. Example: `ZH DEL C3`. | `JT: ZH ID 0xC3 REMOVED` |
| `SYNC` | `ZH SYNC` | Auto-populate the mask from current NodeDB: every favorite + user-bearing node whose role is ROUTER, ROUTER_LATE, or CLIENT_BASE. | `JT: ZH SYNC OK (<count> nodes)` |
| `CLEAR` | `ZH CLEAR` | Wipe the entire Zero-Hop mask (32 bytes / 256 relays, all bits zeroed). | `JT: ZH MASK CLEARED` |
| `CLEAR ROUTER` (alias `RT`) | `ZH CLEAR ROUTER` | Wipe the auto-router mask. | `JT: ZH ROUTER MASK CLEARED` |
| `CLEAR CLIENT` (aliases `CB` / `CLIENT_BASE`) | `ZH CLEAR CLIENT` | Wipe the auto-client mask. | `JT: ZH CLIENT MASK CLEARED` |
| `EXPORT` (alias `OUTPUT` / `OUT` / `B64`) | `ZH EXPORT` | Emit the 32-byte `zero_hop_mask` as a 44-char base64 string in a single reply. Suitable for copy/paste or committing to a notebook. Companion to `ZH IMPORT`. | `JT ZH B64: <base64>` |
| `IMPORT <b64>` (alias `INPUT` / `IN`) | `ZH IMPORT <base64>` | Parse a 44-char base64 blob (whitespace tolerated, padding optional) and overwrite `zero_hop_mask`. Decoded length must equal exactly 32 bytes (256 relays). | `JT: ZH IMPORT OK (32 bytes / 256 relays)`<br>`JT: ZH IMPORT failed (got <n> bytes, need 32)` |
| `LIST [target]` | `ZH LIST`<br>`ZH LIST ROUTER`<br>`ZH LIST CLIENT`<br>`ZH LIST ALL` | Single-reply listing of the selected zero-hop list with friendly names. Target: `MANUAL` (default — hand-built mask), `ROUTER` (auto router/router_late), `CLIENT` (auto client_base), `ALL` (effective set = manual + enabled autos). Best for small lists: entries beyond the 250-byte buffer are **silently dropped** (use `PAGED` for large lists — there is no auto-fallback). Format: `[<hex>] <name>` per line. | `ZH List (manual):`<br>`[C3] HQ01`<br>`[07] Relay7`<br>...<br>`JT: ZH List (manual) is EMPTY` |
| `SHORT [target]` (alias `S` / `HEX`) | `ZH SHORT`<br>`ZH SHORT ROUTER`<br>`ZH SHORT CLIENT`<br>`ZH SHORT ALL` | Compact listing of the selected list's relay IDs as space-separated hex bytes (no names); same target words as `LIST`. Practical limit ≈83 entries (250-byte buffer; reply truncates at 200 chars). | `JT ZH [manual] (<count>): 07 C3 1A 2B ...`<br>`JT: ZH [manual] (short) empty` |
| `PAGED [target]` (alias `P` / `PG`) | `ZH PAGED`<br>`ZH PAGED ROUTER`<br>`ZH PAGED CLIENT`<br>`ZH PAGED ALL` | Same content as `LIST` (same target words) but routed through the paging mechanism so each entry is its own mesh reply. Use for lists with >~25 entries. | `JT ZH List (manual):`<br>`JT: Paging ZH [manual] list (<count> entries)...`<br>`[<hex>] <name>`<br>`...`<br>`-- <count> entries --` |
| `LISTS` | `ZH LISTS` | Paged view of **all three** lists in one run — manual, then auto router/router_late, then auto client_base — each with its own `-- <list> (<n>) --` header (disabled auto lists tagged `[OFF]`) and a `-- all: <n> entries --` footer, so an operator can tell where every zero-hop relay came from. | `JT ZH Lists:`<br>`-- manual (3) --`<br>`[C3] HQ01`<br>`-- router [OFF] (12) --`<br>`...`<br>`-- all: 20 entries --`<br>Reply: `JT: Paging ZH lists (manual/router/client, 20 total)...` |
| _bare_ `ZH` | `ZH` | Show current global state: Zero-Hop enabled, Position preserved, Telemetry preserved. Used as default reply when no subcommand matches. | `JT: ZH <ON\|OFF> (POS:<ON\|OFF> TELE:<ON\|OFF>)` |



---

## 5. Security & Features

Two independent control groups live here. `SECURITY` (alias `SEC`) governs who may issue `/nexus` commands and how rejected packets are handled (vault v65). `FEATURES` toggles the 64-bit runtime feature bitmask controlling phone forwarding and radio behavior (vault v66). Both masks are RAM-resident until you run `/nexus SAVE`.

### 5.1 `SECURITY` (alias `SEC`)

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `SECURITY` | `SECURITY` | Displays security overview: signature requirement, DM allowance, silent logging, auto-fav bits (incl. extra-channel), replay-ring fill. | `JT SECURITY:`<br>` REQ_SIG:<st>`<br>` ALLOW_DM:<st>`<br>` SILENT:<st>`<br>` FAV_NX:<st>`<br>` FAV_TR:<st>`<br>` FAV_XT:<st> ch<n>`<br>` BYPASS_RP:<st> (<n>/256)` |
| `SECURITY <RULE>` | `SECURITY <RULE> [ON\|OFF]` (or `SECURITY ON\|OFF` for all bits) | Toggle security rule: `REQ_SIG` (`RSIG`/`RS`), `ALLOW_DM` (`AD`/`ADM`), `SILENT_LOG` (`SL`/`SIL`), `AUTO_FAV_NEXUS` (`AFN`/`AN`), `AUTO_FAV_TRUSTED` (`AFT`/`AT`), `AUTO_FAV_EXTRA` (`AFX`), `BYPASS_RP` (`BRP`). | `JT SECURITY: <RULE> -> 0x<mask> (run /nexus SAVE to persist)` |
| `SECURITY AFX` | `SECURITY AFX <0-7>` | Select the AUTO_FAV_EXTRA channel (bitmask bits 7-9) **and** enable bit 6 in one round trip. When enabled, any decoded packet (non-zero portnum) from a non-self node on that channel is auto-favorited — same concept as AUTO_FAV_NEXUS but keyed by channel index instead of the compile-time/hardcoded Nexus trusted channels. `AFX on`/`AFX off` toggles without changing the stored channel; the channel persists with `/nexus SAVE`. | `JT SECURITY: AUTO_FAV_EXTRA ON ch<n> -> 0x<mask> (run /nexus SAVE to persist)` |

#### What each option does

All seven bits live in one `uint32_t` (`nexus_security_bitmask`, vault v65) and default **OFF**. Command authorization runs in two steps: the replay check first, then the auth gate. Internal packets (`nexusIsInternal()` = `isFromUs()` **and** not received over LoRa — so a packet looping back over the air with `from==0`/`from==us` is *not* internal) bypass every rule in this table. A rejected command is never executed; the packet itself still returns `CONTINUE` for upstream visibility.

| Option | Aliases | Bit | What it does when ON |
|--------|---------|-----|----------------------|
| `REQ_SIG` | `RSIG`, `RS` | 0 | Require a valid XEdDSA signature from a trusted key (admin / secondary / master) on every `/nexus` command arriving on a Nexus channel. OFF = anyone on the channel may issue commands. Reject reason: `signature required (REQ_SIG)`. |
| `ALLOW_DM` | `AD`, `ADM` | 1 | Accept `/nexus` commands received as DMs. DMs are never signed, so authorization falls back to `isAuthorizedAdmin()` — the sender's public key must be in the Nexus trust list, otherwise the reject reason is `DM not authorized (ALLOW_DM, pubkey not in trust list)`. OFF = every DM command is rejected with `DM not authorized (ALLOW_DM=OFF)`. |
| `SILENT_LOG` | `SL`, `SIL` | 2 | On a security reject (replay **or** auth), log and drop **without replying**. OFF = the sender gets `JT SECURITY: rejected — <reason>`, which helps debugging but also lets an attacker probe your bitmask state. Commands targeted at a *different* node are always silent regardless of this bit, so the node never leaks its security state to eavesdroppers. |
| `AUTO_FAV_NEXUS` | `AFN`, `AN` | 3 | Auto-favorite senders: any decoded valid packet from a node **on a Nexus channel** sets the NodeDB favorite bit. Already-favorited nodes and self-packets are skipped. |
| `AUTO_FAV_TRUSTED` | `AFT`, `AT` | 4 | Same trigger as `AUTO_FAV_NEXUS` (decoded, valid, non-self, Nexus channel), but favorites the sender only if `verifyTrustedSignature()` matches a Nexus trusted key — i.e. only signers you already trust, not everyone on the channel. |
| `BYPASS_RP` | `BRP` | 5 | **Inverted: ON = replay protection OFF.** With the bit OFF (default), a duplicate `MeshPacket.id` from a non-internal sender is rejected before dispatch — 256-entry ring, no TTL, and the ID is recorded on accept *and* reject so a malicious duplicate can't unlock the original's slot. ON skips both the check **and** the ring fill: the ring stays frozen exactly as it was when you flipped it, and picking back up resumes from there. |
| `AUTO_FAV_EXTRA` | `AFX` | 6 (bits 7-9 = channel 0-7) | Like `AUTO_FAV_NEXUS` but keyed to **one** operator-chosen channel — deliberately *not* gated on the compile-time Nexus trusted channels, so you can auto-favorite activity on any ordinary channel. `SECURITY AFX <0-7>` sets the channel **and** enables bit 6 in one round trip; `AFX on` / `AFX off` toggles without changing the stored channel. |

*Auto-favorites never touch flash by themselves — `/nexus SAVE` persists both the bitmask and any favorites it set. The `SECURITY` overview line `BYPASS_RP:<st> (<n>/256)` shows the live replay-ring fill.*

### 5.2 `FEATURES`

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `FEATURES` | `FEATURES` or `FEATURES LIST` | Show the 64-bit feature mask plus each named bit `ON`/`off`. Unset bits print lowercase `off`. | `JT FEATURES: 0x<HEX16>`<br>`FORWARD_OPAQUE_TO_PHONE:      <ON\|off>`<br>`FORWARD_FOREIGN_PKI_TO_PHONE: <ON\|off>`<br>`DISABLE_AGC_RESET:            <ON\|off>`<br>`LATEBASE:                     <ON\|off>` |
| `FEATURES <NAME> [ON\|OFF]` | `FEATURES OPAQUE ON`<br>`FEATURES LATEBASE OFF` | Toggle one named bit (case-insensitive, whitespace-tolerant; verb defaults to `ON` when omitted). Names and aliases in the bit table below. | `JT FEATURES: <NAME> <ON\|off> (mask=0x<HEX16>)`<br>`JT FEATURES: unknown name '<name>' (try FORWARD_OPAQUE_TO_PHONE / FORWARD_FOREIGN_PKI_TO_PHONE / DISABLE_AGC_RESET / LATEBASE)`<br>`JT FEATURES: bad verb '<v>' (ON\|OFF)` |
| `FEATURES 0x<hex>` / bare hex | `FEATURES 0x9`<br>`FEATURES 9` | Set the whole mask directly — `0x`-prefixed, or bare hex of 1-16 digits with no spaces. | `JT FEATURES: 0x<HEX16>`<br>`JT FEATURES: bad hex '<v>'` |
| `FEATURES OFF` | `FEATURES OFF` | Clear every feature bit (all defaults restored). | `JT FEATURES: off (mask=0x0)` |

**Feature bits:**

| Bit | Name & Aliases | Effect |
|-----|----------------|--------|
| `1 << 0` | `FORWARD_OPAQUE_TO_PHONE` (`OPAQUE`, `OPAQ`) | Forward opaque-channel packets to the phone app instead of skipping them. |
| `1 << 1` | `FORWARD_FOREIGN_PKI_TO_PHONE` (`PKI`, `FOREIGN_PKI`) | Additionally forward packets signed by foreign (non-trusted) PKI keys to the phone app. |
| `1 << 2` | `DISABLE_AGC_RESET` (`AGCRESET`, `AGC`, `DISABLEAGCRESET`) | No-ops the 60 s periodic radio AGC reset; RX recovery still runs. |
| `1 << 3` | `LATEBASE` (`LB`, `LBASE`) | Restores upstream v2.8.1's late-rebroadcast-window clamp for favorited `CLIENT_BASE` so it repeats with `ROUTER_LATE` timing; off = JT's client/router window behavior. |

---

## 6. Hop-Aware Routing & Masks

This is useful to control what the hops set for specific packets are, this is way less useful in 2.8 as it was in 2.7 back, has been ported regardless from 2.7 based Nexus. You can control both the start_hops and hop_limit in the packet for either all packets, by channel and by ports. So for example to send a text message with 7 hops but telemetry packets with 1 for example. With such a setting for example you could talk to the entire country in Spain but don't flood regions 500km away with your telemetry...
Again: The 2.8 already does this automatically, but if you wanna override it manually you can.



| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `HOPMASK` | `HOPMASK` | Shows global hop mask status. | `JT Hop Masking: <ACTIVE\|OFF> (S:<start>, L:<limit>)` |
| | `HOPMASK <start> <limit>`| Enforce fixed `hop_start` and `hop_limit` on all rebroadcast packets (0-7). Example: `HOPMASK 3 3`. | `JT: Hop Masking ACTIVE (Start:<start>, Limit:<limit>)` |
| | `HOPMASK OFF` | Disable global hop masking. | `JT: Hop Masking DISABLED.` |
| `CH-HOPMASK` / `CHM` | `CH-HOPMASK <ch> <s > <l>`| Apply hop masking exclusively to channel `<ch>`. Example: `CH-HOPMASK 0 2 2`. | `JT: Ch <ch> (<name>) Hopmask ACTIVE (S:<s>, L:<l>)` |
| | `CH-HOPMASK <ch> OFF` | Disable hop masking for channel `<ch>`. | `JT: Ch <ch> (<name>) Hopmask DISABLED` |
| | `CH-HOPMASK` | Lists hop masks configured for all channels. | `JT Ch Hopmasks:`<br>`Ch <id> (<name>): S:<s> L:<l>...` |
| `PORT-HOPMASK` / `PHM`| `PORT-HOPMASK <port> <s> <l>`| Apply hop masking to specific port (`NODEINFO`, `TELEMETRY`, `POSITION`, `TRACEROUTE`, `TEXT`). | `JT: Port <port> Hopmask ACTIVE (S:<s>, L:<l>)` |
| | `PORT-HOPMASK <port> OFF`| Disable hop masking for `<port>`. | `JT: Port <port> Hopmask DISABLED` |
| | `PORT-HOPMASK` | Lists all port-specific hop masks. | `JT Port Hopmasks:`<br>`<port>: S:<s> L:<l>...` |
| `HOPSCALE` / `HSC` | `HOPSCALE` | Paged dynamic hop-scaling status (read-only — there is **no** setter). | `JT HOPSCALE: status=<st>  entries=<n>/128 (<pct>%)  samp=1/<n>  filt=1/<n>`<br>`lastSuggestedHop: <n>/7  lastRequiredHop: <n>  politeness: <f> (<label>)`<br>`perHop [h0..h7]: ...` / `trend: ...` / `perHour [h0..h12] ...`<br>`hopscale wants: PO:<n> TL:<n> NI:<n> NBI:<n>` |
| | `HOPSCALE WANTS` (alias `HSC WANTS`) | Show user-configured hop wants vs caps per port. | `JT HSC WANTS: user=<n> want=<n> cap=<n>` + `POSITION:/TELEMETRY:/NODEINFO:/NEIGHBORINFO: cap` lines |
| `HOPSTATS` / `HS` | `HOPSTATS` | Displays distribution histogram of hops used across received packets (zero buckets above h3 are omitted). | `JT HOPSTATS (T:<n>):`<br>`H0:<n> H1:<n> H2:<n> [H3..]` |
| `HOPMATRIX` / `HM` | `HOPMATRIX` | 2D matrix comparing hop_start vs hops_used for network density analysis. | `JT: Paging hop matrix...` |

---

## 7. Radio Configuration & RF Control

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `LORA` / `LR` | `LORA` | Modem summary: SF, BW, CR, preset number/state, frequency, power, MQTT-ignore and rebroadcast mode. | `JT LORA:`<br>`SF:<sf> BW:<bw> CR:<cr>`<br>`Preset:<n> (ACTIVE\|OFF)`<br>`Freq:<freq>MHz`<br>`Power:<pwr>dBm`<br>`IgnoreMQTT:<0/1>`<br>`Rebroadcast:<mode>` |
| `LORA-STATUS` / `LRS` | `LORA-STATUS` | Real-time hardware status: frequency, bandwidth, SF, CR, preamble, slot/CW timing, syncword, chip power, air rate. **With `SETFSK`/`SETFLRC` active it reports the modulation actually on air instead**: a `Mode:` line plus BR/dev/RxBw (FSK) or CR/BT (FLRC), preamble in bits, speed = BR; SF/BW/CR/Slot/CW lines are omitted (they'd show the parked LoRa config). | `JT LORA Status:`<br>`Freq: <freq>MHz`<br>`BW:<bw>kHz SF:<sf> CR:<cr>`<br>`Pre:<n> (<us>ms)`<br>`Slot:<ms> CW:<lo>-<hi>`<br>`SW:0x<HEX>`<br>`ChipPwr:<dBm>dBm`<br>`Speed:<bps> b/s` |
| `SETLORA` | `SETLORA <sf> <bw> <cr> <freq>` | Reconfigure modem parameters directly in RAM (applies after deferred timer). Example: `SETLORA 7 62 5 869.618`. | `JT: LoRa accepted. SF:<sf> BW:<bw> CR:<cr> Freq:<freq>MHz. Applying LIVE in 5s.` |
| `SETFSK` | `SETFSK <br_kbps> [freqDev_kHz] [rxBw_kHz] [preamble_bits]` · bare = status · `SETFSK OFF\|RESET\|DEFAULT` | Switch the radio from LoRa to **FSK/GFSK** modulation (whole fleet: SX126x, LR11x0, SX128x, LR2021 — not SX127x/RF95). Params & theory: see **SETFSK/SETFLRC reference** below. RAM-only; deferred apply like `SETLORA`. Example: `SETFSK 50 25 117.3 16`. | `JT: FSK accepted. BR:<br>k DEV:<dev>k BW:<bw>k PRE:<pre>. Applying LIVE in 5s.`<br>`JT: Invalid FSK params (BR:0.6-300k, DEV:1.5-200k, BW:4.8-500k, PRE:4-1024)`<br>`JT: SETFSK not supported on this radio` |
| `SETFLRC` | `SETFLRC <br_kbps> <cr> [bt] [preamble_bits]` · bare = status · `SETFLRC OFF\|RESET\|DEFAULT` | Switch the radio to **FLRC** modulation — SX128x (2.4 GHz) and LR2021 (X1, sub-GHz & 2.4G) only; `cr` is required. Params & theory: see **SETFSK/SETFLRC reference** below. Example: `SETFLRC 650 3`. | `JT: FLRC accepted. BR:<br>k CR:<cr> BT:<bt> PRE:<pre>. Applying LIVE in 5s.`<br>`JT: Invalid FLRC params (BR:1-2600k, CR:2\|3\|4 = 1/2\|3/4\|uncoded, BT:0\|1\|2, PRE:4-1024)`<br>`JT: SETFLRC not supported on this radio` |
| `PRESET` / `SETPRESET` | `PRESET <preset>` | Load standard, tactical, narrow, lite, mini, or 2.4 GHz LoRa presets. Automatically sets the proper region (`EU_868`, `EU_N_868`, `EU_866`, or `LORA_24`) with bidirectional hardware band protection (single-band radios cannot jump between Sub-GHz and 2.4 GHz; dual-band **LR1121** and **LR2021** can freely switch). Example: `PRESET Madrid24` or `PRESET Mini-1`. | `JT: Preset '<preset>' accepted (<region>). SF:<sf> BW:<bw> CR:<cr> Freq:<freq>MHz. Applying LIVE in 5s.`<br>`JT: Cannot set Sub-GHz preset on LORA_24 (single-band radio).`<br>`JT: Cannot set 2.4GHz preset on Sub-GHz region (single-band radio).` |
| `SETPREAMBLE` / `SETPRE` | `SETPREAMBLE <len>` | Set custom preamble length (6-1024 symbols). Example: `SETPREAMBLE 16`. | `JT: Preamble Length set to <len>. Applying in 5s...` |
| | `SETPREAMBLE` | Show active preamble override. | `JT Preamble Override: <len>` |
| `SETSYNCWORD` / `SETSW` | `SETSYNCWORD <hex\|dec\|RESET>`| Set custom radio sync word (0x00-0xFF; `DEFAULT`/`DEF` are aliases of `RESET`, default 0x2B). Bare form queries. Example: `SETSW 0x12`. | `JT: Sync Word set to 0x<HEX>. Applying in 5s...`<br>`JT: Sync Word reset to default 0x2B. Applying in 5s...`<br>Query: `JT Sync Word: 0x<HEX> (Meshtastic default\|override)` |
| `SETPOWER` / `SETPWR` / `SETTXPOWER` | `SETPOWER <dBm>` | Set radio transmit power in dBm (-9..31). Example: `SETPOWER 22`. | `JT: TX Power set to <dBm> dBm.` |
| `GETTXPOWER` | `GETTXPOWER` | Read active and chip TX power settings. | `JT TX Power:`<br>`Active: <dBm> dBm`<br>`Chip: <dBm> dBm` |
| `LNA` | `LNA` | Shows low-noise amplifier gain mode. (Heltec V4.3 specific) | `JT LNA: <status>` |
| `SETLNA` | `SETLNA <mode>` | Configure LNA gain settings. | `JT: LNA set to <mode>` |
| `TX ON` / `TX OFF` | `TX ON` / `TX OFF` | Emergency radio transmitter killswitch (enables or disables RF emissions). | `JT: LoRa TX ENABLED` / `JT: LoRa TX DISABLED (Silent)` |
| `SETREBROADCAST` | `SETREBROADCAST <mode>` | Set rebroadcast mode: `ALL`, `ALL_SKIP`, `LOCAL_ONLY`/`LOCAL`, `KNOWN_ONLY`/`KNOWN`, `NONE`, `CORE`/`CORE_ONLY`, or digit 0-5. Bare `REBROADCAST` queries. | `JT: Rebroadcast Mode set to <mode> (<n>)`<br>Query: `JT: Rebroadcast Mode is <mode> (<n>)` |
| `IGNORMQTT` / `IGNOREMQTT` | `IGNORMQTT [ON\|OFF]` | Ignore incoming MQTT packets entirely. | `JT: LoRa Ignore MQTT is <ON\|OFF>`<br>`JT: LoRa Ignore MQTT set to <ON\|OFF>` |
| **`CLIENTALWAYS`** | `CLIENTALWAYS [ON\|OFF]` | Disables relay cancelation. Works both on CLIENTs and CLIENT_BASEs **USE ONLY WHEN A NODE MUST REPEAT LIKE A ROUTER BUT CANNOT BE A ROUTER.** (No `CA` alias.) Bare form queries. | `JT: Client Always set to <ON\|OFF>`<br>Query: `JT: Client Always is <ON\|OFF>` |
| `BASETEXTS` | `BASETEXTS [ON\|OFF]` | Useful for roof top base nodes, does the same thing a Client Always but only with text messages, if CA is on it changes nothing. (No `BT` alias.) Bare form queries. | `JT: Base Texts set to <ON\|OFF>`<br>Query: `JT: Base Texts is <ON\|OFF>` |
| `ROLE` | `ROLE` | Query active device role. | `JT ROLE: <role> (<n>)` |
| `SETROLE` | `SETROLE <role> [RAM]` | Change device role. Append `RAM` for live in-memory change with role defaults applied without saving to flash and without rebooting (anti-flash-wear for solar threshold crossing). Accepted: `CLIENT`, `SENS`/`SENSOR`, `RTR`/`ROUTER`, `RTR_CLIENT`, `BASE`/`CLIENT_BASE`, `MUTE`/`CLIENT_MUTE`, `HIDDEN`/`CLIENT_HIDDEN`, `LOST`/`LOST_AND_FOUND`, `TAK`, `TAK_TRACKER`, or digit 0-13. | RAM: `JT: Role set to <role> (<n>) in RAM (no reboot).`<br>Flash: `JT: Role updated to <role> (<n>).` |
| **`VOLT`** / `VOLTAGE` | `VOLT [LIST\|STATUS]`<br>`VOLT <1..4> OFF`<br>`VOLT WAKE`<br>`VOLT <1..4> <V\|mV> <HIGH\|LOW> <CMD\|HIB\|WAKE> [CH1..3\|BAT] [cmd...]` | 4-threshold voltage power automation engine (v68). Monitors battery or multi-channel INA voltage (3-reading debounce, 50 mV hysteresis). Actions:<br>• `CMD`: Execute local command (up to 31 chars)<br>• `HIB`: Enter timed 60s microamp deep sleep sniff loop<br>• `WAKE`: Wake from hibernation + run optional wake command<br>Channels: `BAT`/`CH1` (default), `CH2`, `CH3` (multi-channel INA3221). Persistent via `/nexus SAVE`. | `JT: Paging VOLT rules...`<br>`JT VOLT: [<slot>] <V> <DIR> <ACT> [<CH>] configured (RAM). Run /nexus SAVE to persist.` |
| `LR2021_SF` / **`MULTISF`** / `MSF`| `LR2021_SF [SFs...]` | Configure LR2021 hardware multi-SF concurrent receiver channels with independent syncwords (compiled only with `USE_LR2021` / `ARCH_PORTDUINO`). Supports optional `:0xSW`, `:jc`, or `:mc` per channel. Example: `LR2021_SF 6 7 8:0x12` or `LR2021_SF 6 7 8:jc`. | `JT LR2021_SF: side=[SF6,SF7,SF8:0x12] stored and applied (n=<n>)`<br>`JT LR2021_SF: side=(empty) cleared (single-SF RX, n=0)` |
| **`JENCORE`** / `JC` / `MC` / `JEN_CORE` / `MESHCORE` / `MCORE` | `JENCORE [STATUS]`<br>`JENCORE ON\|OFF`<br>`JENCORE IDENTITY` (alias `ID`)<br>`JENCORE RELAY <ON\|OFF>`<br>`JENCORE PATHSIZE [1-3]`<br>`JENCORE FLOODMAX [<gen> <unscoped> <advert>]`<br>`JENCORE DISCOVER`<br>`JENCORE NEIGHBORS`<br>`JENCORE TRACE [<hex>]`<br>`JENCORE REGION [STATUS\|ADD <name>\|DEL <name>\|HOME <name>\|UNSCOPED ON/OFF\|OFF]`<br>`JENCORE ADVERT [LOCAL\|INTERVAL <local-min> <flood-min>]`<br>`JENCORE CONTACTS`<br>`JENCORE CHANNELS [ADD <name\|#tag> [base64-psk]]`<br>`JENCORE HYBRID <ON\|OFF>`<br>`JENCORE STATS` | JenCore interop, repeater and companion subsystem. The master enable is **OFF** by default; while off, all sniff/relay/companion/advert paths are idle. Decodes DMs and group text on `JC:` (legacy `MC:`) channels and relays flood packets. `PATHSIZE` sets the trace path-hash width in bytes (default 3). `FLOODMAX` sets MeshCore flood hop limits (defaults 64/64/8). `REGION` manages up to 4 transport-code regions; the first/`HOME` region scopes outgoing floods. `ADVERT` sends a manual advert (`LOCAL` = zero-hop) or sets automatic intervals (both **0 = off** by default). `DISCOVER`/`NEIGHBORS` probe and list zero-hop JenCore repeaters. `TRACE` is one-way: the endpoint reports it, not the sender. `CHANNELS` seeds `JC:Public`, `JC:#test`, `JC:#bot` or adds `JC:<name>`; a `#tag` key derives as `SHA256("#tag")[:16]`. `HYBRID` enables the SX126x/SX128x hybrid syncword (`0x14B4`) RX path where supported. | Status:<br>`JT JENCORE:`<br>` Enabled: <ON\|OFF>`<br>` Relay: <ON\|OFF>`<br>` PathHash: <n> byte(s)`<br>` Regions: <n>`<br>` Advert: local=<n>min flood=<n>min`<br>` HybridSync: <ON (0x14B4)\|OFF\|N/A>`<br>` RxSeen: <n> (dup=<n>)`<br>` TxRelay: <n>`<br>` Contacts: <n>`<br>Others: `JT JENCORE: ON` / `JT JENCORE: OFF`<br>`JT JENCORE: PathHash = <n> byte(s)`<br>`JT JENCORE: flood_max=<g> unscoped=<u> advert=<a>`<br>`JT JENCORE: regions=<n> home=<name> unscoped=<ON\|OFF> \| <*home>, <others>`<br>`JT JENCORE: advert local=<n>min flood=<n>min (0=off)`<br>`JT JENCORE: <local\|flood> ADVERT queued`<br>`JT JENCORE: HybridSync <ON (0x14B4)\|OFF (standard 0x2B)>`<br>`JT JC STATS: ...` |
| **`JRNODE`** / `RNODE` / `RN` / `JRN` | `JRNODE [STATUS]`<br>`JRNODE ON\|OFF`<br>`JRNODE MODE <BRIDGE\|STANDALONE\|OFF>`<br>`JRNODE TCP <ON\|OFF>`<br>`JRNODE BLE <ON\|OFF>`<br>`JRNODE TIER <5-12>` (alias `SF`) | Reticulum RNode engine. Starts **OFF** by default; `ON` selects BRIDGE. Exposes an RNode over TCP port 7633 (WiFi) and Nordic UART Service (BLE NUS, advertised as `RNode _XXXX`). In BRIDGE mode it runs concurrently with Meshtastic and JenCore, ACKing Reticulum parameter commands while keeping the configured tier. In STANDALONE mode the radio hardware is dedicated exclusively to Reticulum. | `JT JRNODE:`<br>` Enabled: <ON\|OFF>`<br>` Mode: <BRIDGE\|STANDALONE\|OFF>`<br>` Tier SF: <SF>`<br>` TCP: <ON\|OFF> (<conn\|idle>)`<br>` BLE: <ON\|OFF> (<conn\|idle>)`<br>` RxPkts: <n> (bytes=<n>)`<br>` TxPkts: <n> (bytes=<n>)`<br>` LastRSSI: <n>, SNR: <f>`<br>`JT JRNODE: ON (BRIDGE)`<br>`JT JRNODE: Mode <BRIDGE\|STANDALONE\|OFF>`<br>`JT JRNODE: TCP server <ON\|OFF>`<br>`JT JRNODE: BLE NUS <ON\|OFF>`<br>`JT JRNODE: Reticulum tier SF set to <SF>` |
| `G3` / `FRONTEND` | `G3 [STATUS]` | Station G3 PA / LNA RF front-end control status (compiled only on `STATION_G3` builds). | `JT G3 STATUS`<br>`  PA  = <state>`<br>`  LNA = <state>`<br>`  vault: PA=<st> LNA=<st> (run /nexus SAVE to persist)` |
| | `G3 LNA <ON\|OFF\|BYPASS>` | Set G3 frontend LNA state (RAM-only until SAVE). | `JT G3: LNA=<ON\|OFF> (GPIO10 OUTPUT <LOW\|HIGH> / bypassed). RAM-only — run /nexus SAVE to persist.` |
| | `G3 PA <ON\|OFF>` | Set G3 frontend PA state (RAM-only until SAVE). | `JT G3: PA=<ON\|OFF> (GPIO9 OUTPUT HIGH / INPUT Hi-Z). RAM-only — run /nexus SAVE to persist.` |
| | `G3 REINIT` | Re-probe both jumper pads and re-apply front-end state. | `JT G3 REINIT`<br>`  PA  = <state>`<br>`  LNA = <state>` |
| **`APC`** | `APC [STATUS]`<br>`APC ON [MIN=<dBm>]`<br>`APC OFF` | Automatic Power Control engine. Dynamically scales down TX power for direct 0-hop unicast packets (DMs, routing ACKs) based on the target node's last recorded SNR. Prevents receiver saturation, saves battery, and reduces unnecessary RF pollution. Example: `APC ON MIN=-9`. Persistent via `/nexus SAVE`. | `JT APC: <ON\|OFF> (Min: <dBm> dBm)`<br>`JT APC: ON (Min: <dBm> dBm, RAM). Run /nexus SAVE to persist.`<br>`JT APC: OFF (RAM). Run /nexus SAVE to persist.` |
| **`SPLIT`** | `SPLIT [STATUS]`<br>`SPLIT OFF`<br>`SPLIT MSF <PRESET\|SF> [PWR=<dBm>] [ECHO] [MESHRELAY]`<br>`SPLIT ROOF <PRESET\|SF> [PWR=<dBm>] [ECHO] [MESHRELAY]`<br>`SPLIT CLIENT <PRESET\|SF> [PWR=<dBm>]` | Split-Tier Dual-Preset Engine (v69). Bridges outdoor community mesh and high-speed low-power indoor networks.<br>• `MSF`: Concurrent multi-SF bridge on LR2021 hardware (zero deafness, stock clients supported). E.g. Main=SF6 (`NarrowPlus`), Side=SF7 (`SFNarrow`).<br>• `ROOF`: Single-radio rooftop split (SX1262/SX1280/LR1121). Listens outside, repeats indoor packets at low power, repeats indoor client uplinks out to mesh.<br>• `CLIENT`: Asymmetric indoor client. Listens on indoor tier, transmits uplinks directly on outdoor mesh tier.<br>Options: `PWR=<dBm>` (default -9), `ECHO` (repeat indoor traffic indoors), `MESHRELAY` (repeat mesh traffic out to mesh), `NOZH` (disable zero-hop mask check). Persistent via `/nexus SAVE`. | `JT SPLIT: Mode=<mode> Alt=SF<sf>/<bw>k/<freq>MHz Pwr=<dBm> dBm Flags=0x<HEX>`<br>`JT SPLIT: <mode> configured (Alt=SF<sf>/<bw>k/<freq>MHz Pwr=<dBm> dBm, RAM). Run /nexus SAVE to persist.`<br>`JT SPLIT: OFF (RAM). Run /nexus SAVE to persist.` |
| **`TIER`** | `TIER [STATUS]`<br>`TIER MADRID`<br>`TIER MAIN <MESHTASTIC\|RNODE\|MESHCORE> <syncword>`<br>`TIER SIDE <1-3> <SF5-12> <MESHTASTIC\|RNODE\|MESHCORE> <syncword>`<br>`TIER SIDE <1-3> OFF`<br>`TIER CLEAR`<br>`TIER OFF`<br>`TIER APPLY`<br>`TIER SAVE`<br>`TIER TX <1-3\|SF5-12>`<br>`TIER TXCHANNEL [<ch> <1-3\|SF5-12\|OFF>]` | Manual multi-SF tier table (**LR2021 targets only**). The main carrier comes from the current `config.lora`: set it with `SETLORA` first (wait ~5 s), declare `MAIN`, add sides in order (each SF higher than the main SF), then `APPLY`. `MADRID` stages the SF5 / 62.5 kHz / 869.618 MHz preset and enables JRNODE BRIDGE + JENCORE. `TX` picks the default Meshtastic TX side; `TXCHANNEL` overrides it per channel (stored by channel index, so renames survive). `CLEAR` zeroes the table, TX overrides and enable flag but leaves the main LoRa config untouched; `OFF` only disables the side detectors. `SAVE` writes the vault. | `JT TIER: <ON\|OFF> main=SF<sf>/<proto>:0x<SW> sides=<n> tx=<slot> txch=<n> \| <i>:SF<sf>/<proto>:0x<SW> ... [OK\|<reason>]`<br>`JT TIER: Madrid staged (SF5 JenRNode main, SF6/7 Meshtastic, SF8 MeshCore). Applying in 5s. Run /nexus SAVE.`<br>`JT TIER: main = SF<sf>/<proto>:0x<SW> (main SF comes from SETLORA; run TIER APPLY)`<br>`JT TIER: side <n> = SF<sf>/<proto>:0x<SW>` / `JT TIER: side <n> removed`<br>`JT TIER: table cleared (main LoRa config unchanged)`<br>`JT TIER: OFF (side detectors cleared; main LoRa config unchanged).`<br>`JT TIER: applied` / `JT TIER: rejected: <reason>`<br>`JT TIER: vault saved`<br>`JT TIER: Meshtastic TX tier = <n> / SF<sf>`<br>`JT TIER: TXCHANNEL <n> \| ch<idx>(<name>)=SF<sf>, ...`<br>`JT TIER: ch <idx> '<name>' -> tier <n> / SF<sf>` |

**PRESET settings reference** — presets accepted across Sub-GHz (`EU_868`, `EU_N_868`, `EU_866`) and 2.4 GHz (`LORA_24`):
**Beware that the legality of each varies per country. Make sure to always verify and follow your local regulations.**

| Group | Preset | Long name / alias | SF | BW (kHz) | CR | Frequency (MHz) | Region Set |
|---|---|---|---|---|---|---|---|
| **EU_868 Standard** | `LF` / `LF868` | `LONGFAST` | 11 | 250 | 4/5 | 869.5250 | `EU_868` |
| EU_868 Standard | `MS` / `MS868` | `MEDSLOW` | 10 | 250 | 4/5 | 869.5250 | `EU_868` |
| EU_868 Standard | `MF` / `MF868` | `MEDFAST` | 9 | 250 | 4/5 | 869.5250 | `EU_868` |
| EU_868 Standard | `SS` / `SS868` | `SHORTSLOW` | 8 | 250 | 4/5 | 869.5250 | `EU_868` |
| EU_868 Standard | `SF` / `SF868` | `SHORTFAST` | 7 | 250 | 4/5 | 869.5250 | `EU_868` |
| EU_868 Standard | `LM` / `LM868` | `LONGMODERATE` | 11 | 125 | 4/8 | 869.5250 | `EU_868` |
| EU_868 Standard | `LS` / `LS868` | `LONGSLOW` | 12 | 125 | 4/8 | 869.5250 | `EU_868` |
| EU_868 Standard | `ST` / `ST868` | `SHORTTURBO` | 7 | 500 | 4/5 | 869.5250 | `EU_868` |
| EU_868 Standard | `LT` / `LT868` | `LONGTURBO` | 11 | 500 | 4/8 | 869.5250 | `EU_868` |
| **Spain / Tactical** | `P1` | `SFNARROW` | 7 | 62 | 4/5 | 869.6180 | `EU_868` |
| Spain / Tactical | `P7` | `NARROWPLUS` | 6 | 62 | 4/5 | 869.6180 | `EU_868` |
| Spain / Tactical | `NARROWPROMAX` | `NARROWPREMIUM` | 5 | 62 | 4/5 | 869.6180 | `EU_868` |
| Spain / Tactical | `SP` | `SHORTPLUS` | 6 | 250 | 4/5 | 869.5250 | `EU_868` |
| Spain / Tactical | `SPM` | `SHORTPROMAX` | 5 | 250 | 4/5 | 869.5250 | `EU_868` |
| **Upstream Narrow** | `NF` | `NARROWFAST` / `NARF` | 7 | 62 | 4/6 | 869.4417 | `EU_N_868` |
| Upstream Narrow | `NS` | `NARROWSLOW` / `NARS` | 8 | 62 | 4/6 | 869.4417 | `EU_N_868` |
| **Upstream Lite** | `LITEFAST` | `LITEF` | 9 | 125 | 4/5 | 866.3000 | `EU_866` |
| Upstream Lite | `LITESLOW` | `LITES` | 10 | 125 | 4/5 | 865.7000 | `EU_866` |
| **NAP (BW 125)** | `NAP-1` / `NAP1` | — | 7 | 125 | 4/5 | 865.7000 | `EU_866` |
| NAP (BW 125) | `NAP-2` / `NAP2` | — | 7 | 125 | 4/5 | 866.3000 | `EU_866` |
| NAP (BW 125) | `NAP-3` / `NAP3` | — | 7 | 125 | 4/5 | 866.9000 | `EU_866` |
| NAP (BW 125) | `NAP-4` / `NAP4` | — | 7 | 125 | 4/5 | 867.5000 | `EU_866` |
| NAP (BW 125) | `NAP-5` / `NAP5` | — | 7 | 125 | 4/5 | 871.0000 | `EU_866` |
| **NAPN (BW 62)** | `NAPN-1` / `NAPN1` | — | 7 | 62 | 4/5 | 865.7000 | `EU_866` |
| NAPN (BW 62) | `NAPN-2` / `NAPN2` | — | 7 | 62 | 4/5 | 866.3000 | `EU_866` |
| NAPN (BW 62) | `NAPN-3` / `NAPN3` | — | 7 | 62 | 4/5 | 866.9000 | `EU_866` |
| NAPN (BW 62) | `NAPN-4` / `NAPN4` | — | 7 | 62 | 4/5 | 867.5000 | `EU_866` |
| NAPN (BW 62) | `NAPN-5` / `NAPN5` | — | 7 | 62 | 4/5 | 871.0000 | `EU_866` |
| **Mini Series** | `MINI-1` / `MINI1` | Left of NAP1 (span: 865.6031 - 865.6344) | 6 | 31 | 4/5 | 865.6188 | `EU_866` |
| Mini Series | `MINI-2` / `MINI2` | Right of NAP1 (span: 865.7656 - 865.7969) | 6 | 31 | 4/5 | 865.7812 | `EU_866` |
| Mini Series | `MINI-3` / `MINI3` | Left of NAP2 (span: 866.2031 - 866.2344) | 6 | 31 | 4/5 | 866.2188 | `EU_866` |
| Mini Series | `MINI-4` / `MINI4` | Right of NAP2 (span: 866.3656 - 866.3969) | 6 | 31 | 4/5 | 866.3812 | `EU_866` |
| Mini Series | `MINI-5` / `MINI5` | Left of NAP3 (span: 866.8031 - 866.8344) | 6 | 31 | 4/5 | 866.8188 | `EU_866` |
| Mini Series | `MINI-6` / `MINI6` | Right of NAP3 (span: 866.9656 - 866.9969) | 6 | 31 | 4/5 | 866.9812 | `EU_866` |
| Mini Series | `MINI-7` / `MINI7` | Left of NAP4 (span: 867.4031 - 867.4344) | 6 | 31 | 4/5 | 867.4188 | `EU_866` |
| Mini Series | `MINI-8` / `MINI8` | Right of NAP4 (span: 867.5656 - 867.5969) | 6 | 31 | 4/5 | 867.5812 | `EU_866` |
| **Madrid24 (2.4 GHz)** | `MADRID24` | `MADRID_24` / `MADRID-24` | 9 | 400 | 4/5 | 2482.0000 | `LORA_24` |
| **LORA_24 Standard** | `LF` / `LF24` | `LONGFAST24` (or `LF` when on 2.4G) | 11 | 800 | 4/5 | 2482.0000 | `LORA_24` |
| LORA_24 Standard | `MS` / `MS24` | `MEDSLOW24` (or `MS` when on 2.4G) | 10 | 800 | 4/5 | 2482.0000 | `LORA_24` |
| LORA_24 Standard | `MF` / `MF24` | `MEDFAST24` (or `MF` when on 2.4G) | 9 | 800 | 4/5 | 2482.0000 | `LORA_24` |
| LORA_24 Standard | `SS` / `SS24` | `SHORTSLOW24` (or `SS` when on 2.4G) | 8 | 800 | 4/5 | 2482.0000 | `LORA_24` |
| LORA_24 Standard | `SF` / `SF24` | `SHORTFAST24` (or `SF` when on 2.4G) | 7 | 800 | 4/5 | 2482.0000 | `LORA_24` |
| LORA_24 Standard | `LM` / `LM24` | `LONGMODERATE24` (or `LM` on 2.4G) | 11 | 400 | 4/8 | 2482.0000 | `LORA_24` |
| LORA_24 Standard | `LS` / `LS24` | `LONGSLOW24` (or `LS` on 2.4G) | 12 | 400 | 4/8 | 2482.0000 | `LORA_24` |
| LORA_24 Standard | `ST` / `ST24` | `SHORTTURBO24` (or `ST` on 2.4G) | 7 | 1600 | 4/5 | 2482.0000 | `LORA_24` |
| LORA_24 Standard | `LT` / `LT24` | `LONGTURBO24` (or `LT` on 2.4G) | 11 | 1600 | 4/8 | 2482.0000 | `LORA_24` |

*`PRESET` writes a custom modem config (`use_preset = false`): `spread_factor`, `bandwidth`, `coding_rate`, `override_frequency`, and target `region`, applied LIVE 5 s after the reply. Single-band radios are guarded against setting out-of-band presets (Sub-GHz vs 2.4 GHz); dual-band chips (**LR1121** and **LR2021**) can switch bands freely.*

**SETFSK / SETFLRC reference — switching away from LoRa (FSK / GFSK / FLRC):**
**Status: experimental.** Both commands are compile-verified across all target chips but not yet bench-proven on air — test with a known pair before relying on a link.

If you know LoRa, you already know the trade you're making: LoRa is *chirp spread spectrum* — the radio sweeps the carrier continuously (the "chirp"), and each chirp's spreading factor buys processing gain, so it demodulates below the noise floor. That's why LoRa is slow but reaches kilometers.

- **FSK** (frequency-shift keying) is the classical alternative every walkie-talkie, BLE and old packet-radio link used: the carrier simply shifts between two frequencies to mean 0 and 1. No chirp, no processing gain → it needs a cleaner signal, but it's fast and simple.
- **GFSK** = FSK with a *Gaussian* filter shaping the transmit pulses so the signal doesn't splatter outside its channel (that's the `BT` knob on the FLRC side). For reception it is equivalent to FSK — a mismatched shaper costs a little SNR, not the link. (On SX126x, RadioLib's "FSK" is literally GFSK with shaping off.)
- **FLRC** (Fast Long Range Coding) is Semtech's high-speed mode built into LoRa-class silicon (SX128x, LR2021): near-raw bit rates with forward-error correction and a fixed prefix/sync word — much faster than FSK, more robust than plain FSK at short range, but without LoRa's chirp gain, so nothing like LoRa's range.

**Knob mapping** — what replaces what when you leave LoRa:

| LoRa concept | FSK equivalent | FLRC equivalent |
|---|---|---|
| `SF` (rate ↔ range trade) | `br_kbps` | `br_kbps` |
| `BW` (signal width / filter) | `freqDev_kHz` + `rxBw_kHz` | set by rate family |
| `CR` (redundancy) | none — FSK payload is uncoded | `cr` |
| preamble (symbols) | `preamble_bits` (raw bits — far cheaper per packet) | `preamble_bits` |
| — (shaping) | implicit (NRZ) | `bt` |

**`SETFSK <br_kbps> [freqDev_kHz] [rxBw_kHz] [preamble_bits]`** — defaults in brackets. Frequency and TX power are NOT part of this command: set them with `SETLORA`/`PRESET` and `SETPOWER` as usual; FSK just rides the configured frequency.

| Param | Range | Default | What it actually does |
|---|---|---|---|
| `br_kbps` | 0.6 – 300 (SX126x/LR11x0)<br>0.5 – 2000 (LR2021)<br>125 – 2000 (SX128x discrete) | *required* | Raw bit rate in kbps. Same direction as trading SF down: higher = shorter airtime and more bits per joule, but each bit carries less energy, so range and weak-signal tolerance drop fast. Sub-GHz radios accept continuous rates down to 0.6 kbps (SX126x/LR11x0) or 0.5–2000 kbps (LR2021). SX128x (2.4 GHz) **strictly requires discrete hardware rates**: `125, 250, 400, 500, 800, 1000, 1600, 2000` kbps. |
| `freqDev_kHz` | 1.5 – 200 (SX126x/LR11x0)<br>0.6 – 500 (LR2021)<br>62.5 – 1000 & `br/8..2×br` (SX128x) | `0.5 × br_kbps` (`0.25 × br` on SX128x, min `62.5`) | How far the carrier shifts for a 1 vs a 0 (FSK deviation). Wider = easier, more drift-tolerant demodulation and a wider spectrum footprint; narrower = tighter channel but the PLL must track cleanly. **Both ends must use the same value** — a deviation mismatch is a silent no-link. Sane starting points: `SETFSK 50 25 117.3 16` for Sub-GHz, or `SETFSK 250 125` for SX128x. |
| `rxBw_kHz` | 4.8 – 500 (SX126x/LR11x0)<br>4.8 – 3076 (LR2021)<br>Auto on SX128x | Auto Carson's rule (`>= br + 2×dev`, e.g. `117.3` at 50/25) | The receiver's double-sideband (DSB) IF filter. By **Carson's rule** (Semtech SX1261-2 §6.2.1 / LR1110 §8.2.1), `rxBw` must be at least `br_kbps + 2 × freqDev_kHz` (plus crystal/TCXO offset margin); any omitted or undersized `rxBw` is automatically promoted to the smallest valid hardware filter step satisfying `rxBw >= br + 2×dev` (e.g. `100 kHz -> 117.3 kHz`) so the receiver IF filter never clips the FSK carrier tones. |
| `preamble_bits` | 4 – 1024 (SX126x/LR11x0/LR2021)<br>4 – 32 step 4 (SX128x) | 16 (32-bit physical floor on Sub-GHz GFSK) | Leading alternating pattern so the receiver's AGC settles and bit clock locks before the payload. Raw **bits**, not LoRa symbols — 16 bits costs a rounding error of airtime vs 16 LoRa symbols. Note: on SX126x, LR11x0, and LR2021 GFSK, the driver enforces a 32-bit physical preamble floor for reliable AGC/CDR lock. |

**`SETFLRC <br_kbps> <cr> [bt] [preamble_bits]`** — `cr` is mandatory; `bt` and `preamble` optional. **Family-local by physics:** 2.4 GHz FLRC pairs talk to SX128x peers only, sub-GHz FLRC pairs talk to LR2021 (X1) peers only — they cannot cross bands or chip families.

| Param | Range | Default | What it actually does |
|---|---|---|---|
| `br_kbps` | SX128x: `{260, 325, 520, 650, 1000, 1300}`<br>LR2021: `{260, 325, 520, 650, 1040, 1300, 2080, 2600}` | *required* | Raw bit rate in kbps. The chip **only accepts these specific hardware combinations** (paired with matching internal bandwidths from 300 kHz to 2.6 MHz). Other rates return `RADIOLIB_ERR_INVALID_BIT_RATE` (-101). |
| `cr` | `2`\|`3`\|`4` | *required* | Forward-error-correction ratio: `2` = 1/2 (half the transmission is redundancy — max robustness), `3` = 3/4 (balanced), `4` = 1/1 uncoded (max throughput, no safety net). Same *idea* as LoRa's CR, different code — do not mix values between peers either. |
| `bt` | SX128x: `0`\|`2`<br>LR2021: `0`\|`1`\|`2` | 2 | Gaussian TX pulse shaping: `0` = unshaped NRZ (cleanest eyes, widest spectral shoulders), `1` = BT 0.3 (tightest spectrum; LR2021 only — SX128x FLRC rejects BT 0.3), `2` = BT 0.5 (default — good containment without much ISI). Prefer matching on both ends; a mismatch costs SNR, not the link. |
| `preamble_bits` | 4 – 32 step 4 (SX128x & LR2021) | 16 | Same role as FSK's preamble (must be a multiple of 4 bits between 4 and 32 on SX128x and LR2021 FLRC). |

**Which chips accept what** (bare `SETFSK`/`SETFLRC` reports your radio's capability as `caps:`):

| Chip family | Typical boards | Band Support | `SETFSK` | `SETFLRC` |
|---|---|---|---|---|
| SX126x (1261/1262/1268, LLCC68) | heltec-v3, heltec-v4, station-g3, t-beam-1w, T-Deck, RAK4631, T-Echo, ProMicro DIY | Sub-GHz (150–960 MHz) | ✅ GFSK (0.6–300 kbps) | ❌ no FLRC hardware |
| SX127x / RF95 | heltec v1/v2, older TLora | Sub-GHz | ❌ (`caps: none`) | ❌ |
| LR1110 | T1000-E, dreamcatcher | Sub-GHz (+ GNSS/Wi-Fi) | ✅ GFSK (0.6–300 kbps) | ❌ (driver exposes none) |
| LR1121 | T1000-E Plus, custom trackers | Sub-GHz, 2.4 GHz, Satellite L/S | ✅ GFSK (Sub-GHz & 2.4G) | ❌ (driver exposes none) |
| SX128x (1280/1281) | betafpv 2400, ProMicro DIY E28/24G, MakePython 2.4G | 2.4 GHz ISM only | ✅ GFSK (`125–2000k` discrete) | ✅ FLRC (260–1300 kbps) |
| LR2021 | X1 | Dual-band: Sub-GHz & 2.4 GHz/Satellite | ✅ GFSK (0.5k–2000 kbps) | ✅ FLRC (260–2600 kbps, Sub-GHz & 2.4G) |

**How the mode behaves:**

- **Reply first, apply in 5 s** — identical deferred pattern to `SETLORA`/`SETSYNCWORD`, so your confirmation always reaches the mesh before the radio re-initializes.
- **Persists until disabled**: channel changes and other reconfigures re-apply the alt modem instead of silently reverting to LoRa. `SETFSK OFF` (aliases `RESET`, `DEFAULT`) returns to LoRa; a **reboot also returns to LoRa** — the state is RAM-only.
- **Status**: bare `SETFSK` (or `SETFLRC`) → `JT Alt Modem: <LORA|FSK|FLRC> BR:<x>k dev:<x>k bw:<x>k pre:<n> | caps:<FSK FLRC|none>`.
- **No SNR in FSK/FLRC**: the chip has no LoRa-style SNR, so `rx_snr` is an RSSI-derived pseudo-SNR (−120…−50 dBm mapped to −10…+10 dB) — near/far timing math still works.
- **No LoRa CAD**: channel-activity uses a dynamic RSSI energy sniff (`max(altSensitivityDbm(), noiseFloor) + 10 dB` ⇒ busy) instead of LoRa's preamble detector — collision behaviour differs from LoRa networks.

**Interoperability rules:**

- Every chip gets the same **Nexus wire profile** pinned explicitly (4-byte DC-balanced sync word `{0x2D, 0x4B, 0xD4, 0x93}`, whitening **off**, Gaussian shaping `BT=0.5`, 2-byte CCITT CRC, variable-length packets, 32-bit minimum preamble floor on Sub-GHz GFSK) instead of per-chip defaults — so any FSK-capable Nexus pair on the same band matches **across families** (e.g. sub-GHz SX126x ↔ LR1110 ↔ sub-GHz LR2021). The dangerous default (SX128x whitening on) is deliberately overridden.
- **Band boundaries and chip constraints still apply:**
  - SX128x operates exclusively at **2.4 GHz** and cannot physically communicate with sub-GHz radios (SX126x, LR1110, or sub-GHz LR2021).
  - On SX128x, GFSK modulation accepts only discrete bit rates (`125, 250, 400, 500, 800, 1000, 1600, 2000` kbps) and requires `freqDev >= 62.5 kHz` (e.g. `SETFSK 250 125`). The sub-GHz reference profile `SETFSK 50 25 117.3 16` is rejected by SX128x hardware.
- **Both ends must use byte-identical parameters.** There is no negotiation.
- **Stock Meshtastic nodes hear nothing** of FSK/FLRC traffic (and vice versa) — that isolation is the point: a private test channel on public spectrum. You need two Nexus nodes.
- FLRC never crosses families (SX128x↔SX128x 2.4 GHz, LR2021↔LR2021 sub-GHz). **T1000-E has no FLRC** — for the standard T1000-E + X1 pair, GFSK is the shared link modulation.
- Observe band/duty rules as always — `SETFSK`/`SETFLRC` change the modulation, not your legal limits.

**Examples:**

```
SETFSK                 → status + caps of this radio
SETFSK 50 25 117.3 16  → reference profile (Sub-GHz): 50 kbps, dev 25 kHz, RXBW 117.3 kHz, 16-bit preamble
SETFSK 50              → same as above (auto dev 25 kHz, Carson RXBW 117.3 kHz, 16-bit preamble)
SETFSK 200             → 200 kbps with auto dev 100 kHz, Carson RXBW 467 kHz, 16-bit preamble (Sub-GHz)
SETFSK 250 125         → reference profile (SX128x 2.4 GHz): 250 kbps, dev 125 kHz
SETFLRC 650 3          → 650 kbps, CR 3/4, BT 0.5, 16-bit preamble (SX128x / LR2021)
SETFLRC 650 4 1 32     → uncoded max throughput, BT 0.3, 32-bit preamble
SETFLRC 1300 2         → 1.3 Mbps, CR 1/2 (high throughput with full FEC)
SETFSK OFF             → back to LoRa (applies in 5 s)
```

---

## 8. Identity, Naming & NodeDB

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `NAME` | `NAME <long_name>` | Change device Long Name (DM/Targeted only). Assigns a **new node number** as a side effect. | `JT: Name set. New NodeNum assigned.` |
| `OWNER` | `OWNER <short> <long>` | Change Short Name and Long Name simultaneously (**short first**, max 4 chars; rest is the long name). Example: `OWNER BASE BaseStation`. Also assigns a new node number. | `JT: Owner updated to <short> / <long>. New NodeNum assigned.` |
| `HW` | `HW` | Reports board, MCU family and radio chip (with XTAL/TCXO clock type). | `JT HW:`<br>`Board: <board>`<br>`MCU: <mcu>`<br>`Radio: <radio> (XTAL\|TCXO)` |
| `ACC` | `ACC` | Displays the last **accelerometer** motion sample (X/Y/Z in g) — *not* a crypto accelerator report. | `JT Motion (cached <ms>):`<br>`X: <x>g`<br>`Y: <y>g`<br>`Z: <z>g`<br>(or `JT Motion: No recent sample (ageMs=<n>, sensor present=<0/1>)`) |
| `NODES` | `NODES [search]` | Paged full NodeDB listing (optional search filter). | Paging `NodeDB`:<br>`!<id>:<name>:<hops>:<snr>:<last>` per row<br>Reply: `JT: NodeDB results paged...` |
| `FNODE` | `FNODE <search\|!id>` | Full per-node detail listing (names, HW, SNR/hops, keys, etc., paged). | Paging `Node Detail`:<br>`!<id> (<short>)` + multi-line detail per node |
| `NODEDB` / `DB` | `NODEDB <!id\|short>` | Look up comprehensive record for node from database (user, hardware, SNR, last seen). | `JT NODEDB <id>:`<br>`Name: <name> [<short>]`<br>`Seen: <sec>s ago, Role:<role>`<br>`SNR: <snr>, Hop: <h>, Ch: <ch>`<br>`Fav:<0/1> Ign:<0/1> Adm:<0/1>` |
| `NODES28` / `N28` | `NODES28` | Lists nodes running Meshtastic v2.8+ structural / signed protocols (P = structural 2.8, V = XEdDSA-signed). | `JT NODES28: <n>/<total> matched (P=structural2.8, V=XEdDSA-signed)` + paged rows `!<id>  <short> <P\|-> <V\|-> [ (self)]`<br>`JT NODES28: empty nodeDB` |
| `DBINFO` | `DBINFO` | NodeDB health stats: capacity, online count, direct count, favorites. | `JT DBINFO:`<br>`Nodes: <n>/<max> (<pct>%)`<br>`Online: <n>`<br>`Direct: <n>`<br>`Favs: <n>` |
| `FIXMYINFO` / `FMI` | `FIXMYINFO` | Fixes the node's own NodeDB entry in place (long name, short name, HW, role) and saves to disk. Does **not** broadcast. | `JT FMI: Fixed fields: <fields>`<br>`JT FMI: Info already in sync.`<br>`JT FMI Error: Local node not found in NodeDB!` |
| `ADMINS` | `ADMINS` | Lists admin-flagged nodes (paged). | Paging `Admins`:<br>`!<id>:<short>` per row<br>Reply: `JT: Paging admins to mesh...` |
| `NEIGHBORS` / `DIRECT` | `NEIGHBORS` | Lists nodes heard directly (0 hops away) with SNR (paged). | Paging `Neighbors (0-hop)`:<br>`!<id>:<short>(<snr>)` per row<br>Reply: `JT: Paging neighbors to mesh...` |
| `PHONE` | `PHONE` | Displays local Bluetooth / Serial / WiFi phone connection status and pairing PIN. | `JT PHONE:`<br>`BT:<state> RSSI:<r>`<br>`Serial:<state>`<br>`WiFi:<state>...`<br>`PIN:<6-digit>` |
| `POS` | `POS` | Displays current position fix: source type, coordinates, altitude, GPS model/fix state, satellites, and precision (PDOP). | `JT POS [<MANUAL\|INT\|EXT>]:`<br>`Lat:<lat> Lon:<lon>`<br>`Alt:<alt>m`<br>`GPS:<model> (<2D\|3D\|NoFix>)`<br>`Sats:<n> PDOP:<dop>`<br>(no GPS: `Lat/Lon/Alt` only, or `JT POS: No fix.`) |
| `TRUSTED` | `TRUSTED [0\|1] [name\|OFF]` | Manage the 2 named trusted-channel slots. Bare lists both; `<slot> <name>` sets a trusted channel name (15 chars); `<slot> OFF` disables. | `JT TRUSTED Slots:`<br>`[0] <name> (<state>)`<br>`[1] <name> (<state>)`<br>`JT: Trusted Slot <n> -> '<name>'` / `JT: Trusted Slot <n> DISABLED` |

---

## 9. Network Observation, Scanning & SIGINT

Tools for passive spectrum analysis, traffic metrics, neighbor surveys, and packet inspection.

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `NOISE` | `NOISE` | Starts a high-resolution RF noise floor measurement (async: ack first, report follows). | Ack: `JT: Noise snapshot started...`<br>Report: `Noise (<state>,R:<n>)`<br>`<freq>M/<bw>k`<br>`<25 raw samples>`<br>`Avg:<v> Min:<v> Max:<v>` |
| `FREQSCAN` | `FREQSCAN <freq_MHz> <bw_kHz>` | Same as NOISE but on a specific frequency: hops there with TX disabled, measures, returns. Freq range 100-2500 MHz; BW must be 31, 62, 125, 250 or 500 kHz. Example: `FREQSCAN 868.0 250`. | Ack: `JT: FreqScan START (<freq>M/<bw>k). Deaf for 15s.`<br>Report: `JT FREQSCAN (<freq>M):` + noise body (Avg/Min/Max) |
| `NETSCAN` | `NETSCAN` | Performs 60-second background passive mapping of surrounding mesh transmitters. It does it by sending a zero-hop traceroute and then listening for any and all packets heard in the mesh, including unrelated ones.| Ack: `JT: NETSCAN started. Collecting for 60s...`<br>Report (paged): `JT NETSCAN (<n>):`<br>`ID,RSSI,SNR,FE,RY` + CSV rows |
| `CLIENTSCAN` | `CLIENTSCAN [time]` | Identifies active repeating client devices. It uses the command confirmation as a test packet, and reports all the repeats it hears of that packet. Optionally you can specify the amount of time in seconds (1-600, default 100) you want it to wait for repeats.  | Ack: `JT: CLIENTSCAN Start (<n>s)`<br>Report: `JT CLIENTSCAN (!<id>):`<br>`RY,Delay,RSSI,H/S,SNR` + CSV row per heard repeat |
| `RSSI` / `RS` | `RSSI` | Reports signal metrics (RSSI, SNR, Frequency Error, Coding Rate) plus hops/relay and OM/MQTT/UDP/PKI flags of last received packet. | `JT RSSI:`<br>`📡 Hops:<h>/<start> (Direct)\|Relay:<r>`<br>`📶 RSSI:<rssi> SNR:<snr>`<br>`🔬 Δf: <fe>Hz CR:<cr>`<br>`🌐 OM:<0/1> MQTT:<0/1> UDP:<0/1> PKI:<0/1> S:<sig-emoji>` |
| `SHORTRSSI` / `SR` | `SHORTRSSI` | Minimal 2-line RF signal report of last received packet (Hops/Relay, RSSI/SNR/FE). | `JT SR:`<br>`H:<h>/<start> relay:<id>`<br>`R:<rssi> S:<snr> F:<fe>` |
| `RSSITEL` / `RT` | `RSSITEL` | Ultra-compact RF and telemetry report of last received packet (Hops/Relay, RSSI/SNR/FE, ChUtil/AirTime, Batt %/V, live Power/Env sensors, CPU temp, uptime days). | `JT RT:`<br>`H:<h>/<start> relay:<id>`<br>`R:<rssi> S:<snr> F:<fe>`<br>`CU:<cu>% AT:<at>%`<br>`B%:<bpct>% BV:<volt>V`<br>`V1:<v>V I1:<i>mA T:<t>C H:<h>% P:<p>` (real readings; omitted when no sensors)<br>`CPUT:<temp>C Up:<days>` |
| `RSSILOG` / `PL` | `RSSILOG [n]` | Displays recent received packets from a **32-entry** circular ring buffer (default: last 5, max 32). | `JT: Paging RSSILOG (<n> entries) to mesh...`<br>Rows (CSV): `<id16>,<hops>/<start>,<rssi>,<snr>,<relay>,<age>s` |
| `SIGSTATS` / `SS` | `SIGSTATS` | Packet privacy classification (wire total, private-encrypted, known-decrypted) plus noise floor. *Not* signature verification stats — see `VERIFYSTATS`. | `JT SIG-STATS:`<br>`Total Wire: <n>`<br>`Private: <n> (<pct>%)`<br>`Known/Dec: <n> (<pct>%)`<br>`Noise Floor: <dbm> dBm` |
| `VERIFYSTATS` / `VS` | `VERIFYSTATS` | XEdDSA signature verification breakdown (NoSig / invalid / OK buckets with packets, bytes, percent). | `JT VERIFY-STATS:`<br>`NoSig:  <pkts> pkts, <bytes> B (<pct>%)`<br>`SigInv: <pkts> pkts, <bytes> B (<pct>%)`<br>`SigOK:  <pkts> pkts, <bytes> B (<pct>%)`<br>`Total:  <pkts> pkts, <bytes> B` |
| `CRSTATS` / `CRS` | `CRSTATS` | Coding Rate distribution stats across received packets (display-domain CR 5-8, each with direct/relay/total). | `JT CR Stats:`<br>`CR:<n>  D:<n>  R:<n>  T:<n>  <pct>%` per row<br>`Total: <n> packets`<br>Empty: `JT CR Stats:` / `No packets recorded.` / `Reset with /nexus CS (CLRSTATS).` |
| `CLRSTATS` / `CS` | `CLRSTATS` | Resets all volatile runtime diagnostic and packet statistics counters. | `JT: All tactical stats cleared.` |
| `TRANSSTATS` / `TS` | `TRANSSTATS` | Transport-interface packet distribution (INTL, LORA, ALT1-3, MQTT, UDP, API) — *not* airtime/duty-cycle (paged). | Paging `Transport Stats`:<br>`Total Packets: <n>`<br>`<iface>: <n> (<pct>%)` per row<br>Reply: `JT: Paging transport stats...` |
| `LORACHIPSTATS` / `LCS` | `LCS [RESET]` | Percentage-of-time breakdown of LoRa chip states (TX/RX/STBY/FS/SLP/BUSY/ERR/UNK), sampled at 1 Hz since boot (RAM-only; `LCS RESET` zeroes, `CLRSTATS` clears too) — one state per line, with the raw sample count in parentheses. Family coverage: SX126x/SX1280 = busy pin + GetStatus mode; RF95 = REG_OP_MODE; LR11x0/LR20x0 = busy pin + software RX view (chip mode not datasheet-confirmed there). | `JT LCS: n=<samples> (~<days>d)`<br>`TX <p>% (<n>)`<br>`RX <p>% (<n>)`<br>`STBY <p>% (<n>)`<br>`FS <p>% (<n>)`<br>`SLP <p>% (<n>)`<br>`BUSY <p>% (<n>)`<br>`ERR <p>% (<n>)`<br>`UNK <p>% (<n>)` |
| `FIREWALLSTATS` / `FWS` | `FIREWALLSTATS` | Comprehensive counter report for all drop rules, favorite bypasses, and rate limits (paged). | `JT: Paging firewall stats...` |
| `MQTTSTATS` / `MQS` | `MQTTSTATS` | Classifies received packets by ok-to-MQTT bit / via-MQTT flag. *Not* gateway Rx/Tx counters. | `JT MQTT:`<br>`ON:<n>(<pct>%) MQTT:<n>(<pct>%)`<br>`OFF:<n>(<pct>%) Tot:<n>` |
| `STATS` | `STATS` | Mesh router throughput metrics (Tx, Rx, bad, dup, relayed, cancelled, dropped, noise). | `JT STATS:`<br>`TX:<n> RX:<n>`<br>`Bad:<n>(<pct>%) Dp:<n>(<pct>%)`<br>`Relay:<n> Can:<n>`<br>`Drop:<n> Noise:<dbm>` |
| `SEARCHKEY` | `SEARCHKEY <b64_pubkey>`| Search NodeDB for the node with this **exact 32-byte** public key (prefix search is not supported). | Paging rows `!<id>: <long> (<short>)`<br>Reply: `JT: Found <n> matching nodes` / `No matches found.` |
| `SEARCHRELAY` / `SRELAY`| `SEARCHRELAY <hex\|0xhex\|dec>` | Find nodes in database that match a given 8-bit relay ID byte. Example: `SEARCHRELAY *A4`. | `JT Relay *<hex> Matches:`<br>`<short1>, <short2>...`<br>`JT Relay *<hex>: No matches in NodeDB` |
| `RELAYS` / `RY` | `RELAYS [<hex>]` | All relay IDs heard on the mesh with nonzero stats (direct/relayed/total), paged — no top-10 truncation. `RELAYS <hex>` shows one relay. | Paging rows `*<hex> (<name>): D:<n> R:<n> T:<n>`<br>Reply: `JT: Paging RELAYS (<n> active) to mesh...`<br>Single: `Relay *<hex> (<name>) Stats:` / `Direct/Relayed/Total: <n>` |
| `SORTEDRELAYSTATS` / **`SRY`** | `SORTEDRELAYSTATS [top-N]` | Complete sorted list of all active relays by packet count (optional top-N). | Paging rows `*<hex> (<name>): D:<n> R:<n> T:<n> (<pct>%)`<br>Reply: `JT: Paging sorted relay stats (<n> of <total> active)...` |
| `PACKETSTATS` / `PS` | `PACKETSTATS` | Breakout of packets received by PortNum application type (paged; includes subtype breakdowns). | Paging `Packet Stats`:<br>`Total: <n> pkts (<bytes>)`<br>`<PORT>: <n> pkts (<pct>%) <bytes> (<pct>%)` per row<br>Reply: `JT: Paging packet stats...` |
| `SORTEDPACKETSTATS` / **`SPS`**  | `SORTEDPACKETSTATS [top-N]` | Sorted **PortNum application** entries by packet volume (ports, not nodes). Optional top-N. | Paging rows as above, sorted<br>Reply: `JT: Paging sorted packet stats (<n> of <total> active)...` |
| `CHANNELSTATS` / `CHS` | `CHANNELSTATS` | Traffic volume distribution across all 256 channel indexes seen (paged, with channel names). | Paging `Channel Stats`:<br>`Total: <n> pkts (<bytes>)`<br>`Ch<idx>(<name>): <n> pkts (<pct>%) <bytes> (<pct>%)` per row<br>Reply: `JT: Paging channel stats...` |
| **`SORTEDCHANNELSTATS`** / **`SCHS`** | `SORTEDCHANNELSTATS [top-N]`| Sorted channel usage breakdown (optional top-N). | Reply: `JT: Paging sorted channel stats (<n> of <total> active)...` |
| `TRACE` / `TR` | `TRACE <!id\|short>` | Run a real traceroute to the target node (async result). | `JT: Starting trace to !<id>...`<br>`JT: Trace busy. Try again.` / `JT: Trace failed to start` / `JT: Invalid trace target '<t>'` |
| `SENDSTATUS` | `SENDSTATUS <text>` | Send freeform text (max 79 chars) as a Node Status message (`NODE_STATUS_APP`) on Ch 0. | `JT: Status sent (<n> chars)` / `JT: Empty status text` |
| `STATUSPS` | `STATUSPS [N]` | Build a top-N per-PortNum packet-volume summary (default 5) and send it as a Node Status message. | `JT: Status sent (<n> of <total> active ports, PS: <port>:<count>(<pct>%), ...)` / `JT: Status sent (no traffic)` |
---

## 10. Target Watchlist (In-Depth Guide)

The **Target Watchlist** (`WATCH`) is an advanced continuous-monitoring system built into Nexus. It tracks up to 8 specific nodes simultaneously, computing running signal averages, variance, RF standard deviations, hop distributions, relay path dependencies, packet intervals, and port distributions.
**These are RAM-only, they are never stored in the Vault, so after a reboot you lose them.**

### 10.1 Command Syntax & Management

| Command | Subcommands / Syntax | Description | Exact Output / Template |
|---------|----------------------|-------------|-------------------------|
| `WATCH` | `WATCH ADD <!id\|short>` | Add node to watchlist (max 8). Initializes min/max tracking and reset counters. | `JT: Added <hex8> to watchlist.` |
| | `WATCH DEL <!id\|short>` | Remove node from watchlist. | `JT: Removed <hex8>.` |
| | `WATCH LIST` | List all monitored nodes with short names. | `Watchlist: <id1>[<short1>] <id2>...` / `JT: Watchlist empty.` |
| | `WATCH CLEAR` | Clear all 8 slots from the watchlist. | `JT: Watchlist cleared.` |
| | `WATCH RESET` | Reset statistical counters for all slots while keeping the node IDs in the list. | `JT: Watchlist stats reset.` |
| | `WATCH STATS <!id\|short>` | Output paged statistical dossier for one target node. | `JT: Sending paged stats...` |
| | `WATCH ALL` | Output paged dossiers for all monitored targets with recorded packets. | `JT: Sending paged stats for all...` |

### 10.2 Understanding the `WATCH STATS` Output

When `WATCH STATS <target>` or `WATCH ALL` is executed, Nexus produces a multi-line report structured as follows:

```text
Target: defc895b
Pkts: 42 Bytes: 9820
RF (Direct Only):
Avg:-88dB SNR:9.2 FE:120
SD: 2.4 SNR:1.1 FE:35
Min:-94dB SNR:7.0 FE:80
Max:-82dB SNR:11.5 FE:160
Hops: h0:18 h1:14 h2:10 h3:0 h4:0 h5:0 h6:0 h7:0
Relay: Last:3a Primary:3a (14)
Timing: Min:15s Max:312s Age:4s
Ports: NI:4 POS:20 TEL:12 TXT:4 ADM:2 RT:0 SF:0 OTH:0
```

#### Field-by-Field Breakdown:
1. **Target & Traffic Totals:**
   - `Target`: 32-bit hex ID of the monitored node.
   - `Pkts` / `Bytes`: Total number of packets and payload bytes received from this node since tracking began.
2. **RF (Direct Only):**
   *Note: Signal variance and deviation are calculated ONLY on direct (0-hop) packets to ensure physical link accuracy.*
   - `Avg`: Average RSSI (dBm), average SNR (dB), and average carrier Frequency Error (Hz).
   - `SD`: **Standard Deviation** for RSSI, SNR, and Frequency Error. High SNR standard deviation indicates multi-path interference or a moving node; stable Frequency Error confirms frequency crystal stability.
   - `Min` / `Max`: Worst-case and best-case values recorded over the observation window.
3. **Hop Distribution (`Hops`):**
   - Counts how many packets arrived at each hop count (`h0`..`h7`).
4. **Relay Tracking (`Relay`):**
   - `Last`: 8-bit hex ID seen in the `relay_node` byte of the most recent packet (including 00 for direct packets).
   - `Primary`: the relay currently believed dominant via a Boyer-Moore-style majority vote — the `(count)` counter **decrements** when a different relay is seen, so it is an indicator, not a true packet count (known quirk).
5. **Timing Behavior (`Timing`):**
   - `Min` / `Max`: Minimum and maximum observed intervals (in seconds) between consecutive transmissions from this node. Useful for characterizing telemetry or beacon intervals.
   - `Age`: Seconds elapsed since the last packet was received from this target.
6. **Port Distribution (`Ports`):**
   - Frequency breakdown by Meshtastic application type: `NI` (NodeInfo), `POS` (Position), `TEL` (Telemetry), `TXT` (Text), `ADM` (Admin), `RT` (Routing/Ack), `SF` (Store & Forward), `OTH` (Other).

---

## 11. Vault & Storage Persistence

Nexus stores its persistent configuration in an encrypted / versioned flash segment (`/nexus.bin`).

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `SAVE` | `SAVE` | Commits current RAM state (drop rules, presets, aliases, macros, signatures, security settings) to flash. | `JT: Vault saved to flash (JenT).` |
| `LOAD` | `LOAD` | Discards current RAM state and reloads configuration from flash vault. | `JT: Vault loaded from flash.` |
| `VAULT` / `VST` | `VAULT` | Displays vault version, size, drop/invalid masks and slot usage. | `JT VAULT:`<br>`Ver:v<ver> Size:<n>B`<br>`Drop:0x<HEX16>`<br>`OutDr:0x<HEX16>`<br>`Inv:0x<HEX8>`<br>`Groups:<n>/4`<br>`Aliases:<n>/16`<br>`Macros:<n>/4`<br>`Cron:<n>/8`<br>`Volt:<n>/4`<br>`Split:<mode> APC:<ON\|OFF>` |
| `VAULT DIFF` / `VDF` | `VAULT DIFF` | Compares current RAM settings against stored flash vault to identify unsaved changes (paged per-field diff). | Paging `JT VAULT DIFF:` + per-field lines, footer `✓ no diffs (vault mirrors RAM)` or `<n> field(s) differ`<br>`JT VAULT DIFF:`<br>`No valid vault (<reason>) - any RAM state will be saved on next /nexus SAVE` |
| **`FREEZE`** | `FREEZE` | Shows filesystem write-freeze status across all subsystems (one per line). | `JT FREEZE:`<br>`  NodeDB:  <frozen\|unfrozen>`<br>`  Warm:    <frozen\|unfrozen>`<br>`  Device:  <frozen\|unfrozen>`<br>`  Config:  <frozen\|unfrozen>`<br>`(DBFLUSH bypasses all four)` |
| | `FREEZE ON` | Freezes all filesystem writes (prevents flash wear or writes during testing). | `JT: All freezes ACTIVE. saveToDisk() blocked on every segment.`<br>`Use 'FREEZE OFF' to resume or 'DBFLUSH' to force-flush.` |
| | `FREEZE OFF` | Unfreezes all subsystems, restoring normal flash persistence. | `JT: All freezes cleared. Normal save resumes.` |
| | `FREEZE NODEFREEZE\|WARMFREEZE\|DEVICEFREEZE\|CONFIGFREEZE [ON\|OFF]` | Per-subsystem write freeze (note the subcommand names — `FREEZE NODEDB ON` does not work). Bare form shows current state. Also: `FREEZE STATUS`, legacy `DISK`, `DB`/`DBONLY`. | `JT: NodeDB freeze <ON\|OFF>. (Persist via /nexus SAVE.)` (analogous for `Warm`, `Device`, `Config`) |
| `DBFLUSH` | `DBFLUSH` | Forces an immediate write of NodeDB to disk, bypassing active freeze flags. | `JT: DBFLUSH <OK\|FAILED> (freeze flags unchanged: nodedb=<0/1> warm=<0/1> device=<0/1> config=<0/1>).` |
| `BD` | `BD` / `BD GET` | Query maximum broadcast reply / paging delay in seconds. | `JT BD: max delay = <u>s (range 100..<max>ms)` |
| | `BD SET <1..300>` | Set broadcast reply / paging jitter delay (prevents packet collisions on dense meshes). Persisted via `SAVE`. | `JT BD: max delay set to <u>s (jitter 100..<max>ms). (RAM-only; /nexus SAVE to persist.)` |

---

## 12. Channels & Configuration Management

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `CHASH` / `CH` / `CHANNELS` | `CHASH` (aliases `CHANNELS`, `CHHASHES`) | Shows channels with 8-bit PSK hashes (name + hash only, no role). | `JT CHASH:`<br>`<n>:<name>(0x<hash>/<dec>) ...` |
| | bare `CH` | Shows channels with role numbers (no hashes). | `JT CHANNELS:`<br>`<n>:<name>(<role>) ...` |
| `SETCHNAME` | `SETCHNAME <idx\|name> <name>`| Rename channel at index `<idx>` (or by name). `USETCHNAME` skips trusted-channel protection. Example: `SETCHNAME 1 Ops`. | `JT: Ch <n> name set to '<name>'`<br>`JT: Error - Trusted channel protected.` |
| `SETCHPSK` | `SETCHPSK <idx\|name> <psk>` | Set encryption key for channel `<idx>` — **base64 only** (no default/none/hex forms). `USETCHPSK` skips trusted-channel protection. | `JT: Ch <n> PSK updated (<len> bytes)`<br>`JT: Invalid Base64 PSK` |
| `ADDCH` | `ADDCH <name> [psk]` | Add a new channel in the first available slot. | `JT: Added Ch <n> '<name>'` |
| `DELCH` | `DELCH <idx\|name>` | Delete channel at index `<idx>` (Primary channel 0 cannot be deleted). Use `UDELCH` to skip trusted-channel protection. | `JT: Deleted Ch <n>`<br>`JT: Error - Cannot delete Primary (Index 0)`<br>`JT: Error - Trusted channel protected.` |
| `CONFIG` | `CONFIG` | Return active operational configuration summary (role, broadcast intervals, flags, GPS/Fixed state, location). | `JT CONFIG:`<br>`Role: <role>`<br>`NI:<secs>s`<br>`TEL: D:<secs>s E:<secs>s P:<secs>s`<br>`POS:<secs>s(<flags>) Smart:<0/1>`<br>`GPS:<0/1> Fixed:<0/1> HM:<0/1> RM:<0/1>`<br>`Loc: <lat,lon/None>` |
| `SETCONFIG` | `SETCONFIG` (bare) | Show accepted syntax and target options. | `JT SETCONFIG: NI\|TEL_D\|TEL_E\|TEL_P\|POS\|SMART\|FIXED\|GPS <val> \| LOC lat,lon` |
| | `SETCONFIG NI <secs>` | Set NodeInfo broadcast interval in seconds. Example: `SETCONFIG NI 10800`. | `JT: Config updated.` |
| | `SETCONFIG TEL_D <secs>` | Set Device Telemetry (battery, voltage, CPU, uptime) interval. Example: `SETCONFIG TEL_D 900`. | `JT: Config updated.` |
| | `SETCONFIG TEL_E <secs>` | Set Environment Telemetry (temp, humidity, pressure) interval. Example: `SETCONFIG TEL_E 1800`. | `JT: Config updated.` |
| | `SETCONFIG TEL_P <secs>` | Set Power Telemetry (INA219/INA226 current, bus voltage, power) interval. Example: `SETCONFIG TEL_P 900`. | `JT: Config updated.` |
| | `SETCONFIG POS <secs>` | Set Position broadcast interval in seconds. Example: `SETCONFIG POS 900`. | `JT: Config updated.` |
| | `SETCONFIG SMART <ON/OFF>` | Enable/disable Smart Position broadcast (motion/distance triggered). Example: `SETCONFIG SMART ON`. | `JT: Config updated.` |
| | `SETCONFIG FIXED <ON/OFF>` | Enable/disable Fixed Position mode (disables live GPS queries). Example: `SETCONFIG FIXED ON`. | `JT: Config updated.` |
| | `SETCONFIG GPS <ON/OFF>` | Enable or power down the onboard GPS/GNSS receiver hardware. Example: `SETCONFIG GPS OFF`. | `JT: Config updated.` |
| | `SETCONFIG LOC <lat>,<lon>` | Set manual stationary fixed position coordinates. Example: `SETCONFIG LOC 49.15163,9.25764`. | `JT: Manual location set to <lat>,<lon>` |
| `SHARE` | `SHARE <node>` | Generates a Meshtastic **node-share contact** URL to share a specific Node from the NodeDB. | `JT SHARE (<short>): https://meshtastic.org/v/#<b64>` |
| `INFRA` | `INFRA` | Lists all infrastructure nodes detected on the mesh (paged). | Paging `JT Infrastructure`:<br>`!<id>:<short>:<role>:H<hops>` per line<br>Reply: `JT: Paging infrastructure (<n> nodes)...` |
| `MODULES` / `MOD` | `MODULES` | Lists all compiled and active Meshtastic firmware modules (paged). | Paging `JT Loaded Modules:`<br>`[<n>/<total>] <module>` per line<br>Reply: `JT: Paging <n> loaded modules...` |
| `FULLCONFIGDUMP` | `FULLCONFIGDUMP` | Paged comprehensive dump of all device configuration and module settings. | `JT: Paging full config dump...` |
| `RATELIMIT` / `RL` | `RATELIMIT` | Shows current rate limiter **limits** per bucket. | `JT RL Limits: TX:<n> TR:<n> OT:<n> NI:<n> PO:<n> TL:<n> PR:<n>` |
| | `RATELIMIT STATUS\|STAT\|BUCKETS` | Paged per-bucket status view. | `JT: Paging RL buckets...` |
| | `RATELIMIT <bucket> <max>` | Set bucket ceiling in **packets per hour** (single numeric arg). Buckets: `TEXT`, `TRACE`, `OTHER`, `NODEINFO`/`NI`, `POSITION`/`PO`, `TELEMETRY`/`TL`, `PRIVATE`/`PR`. | `JT: RL <bucket> set to <max> pkts/hr` |
| `RATELIMITSTAT` / `RLS` | `RATELIMITSTAT` | Displays hit counters and drop statistics for rate limiter buckets (paged, title `RL Buckets`). | `JT: Paging RL buckets...` |
| `SKEY` | `SKEY <1\|2\|3\|4> <Key\|OFF\|WIPE>` | Manage 4 secondary dynamic admin public keys for remote cryptographic auth. Supports Base64 (128/64 chars) or Hex (64 chars). Bare `SKEY` does not dispatch. | `JT: SKEY <slot> updated.` / `JT: SKEY <slot> wiped.`<br>`JT: Invalid SKEY index (1-4)` |
| | `TM CONFIG` / `TM RESET` / `TM DEDUP_INT\|CACHE_HOPS\|RATELIMIT_WIN\|RATELIMIT_MAX\|UNKNOWN_TH <val>` | Show or tune TrafficManagementModule parameters. | `JT TM Config:`<br>`IN:<n> CR:<n> RLW:<n> RLM:<n>`<br>`UK:<n>` / `JT: TM Config Updated.` |
| `TM` | `TM` | TrafficManagementModule stats (dedup / rate-limit / cache-hops counters). *Not* a module on/off switch. | `JT TM Stats:`<br>`DP:<n> RL:<n> CH:<n>`<br>`RP:<n>` |

---

## 13. Hardware, Sensors & Telemetry

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `TELEMETRY` | `TELEMETRY` | Consolidated telemetry report: channel/TX utilization, battery, power channels, environment. | `JT TELEMETRY:`<br>`ChUtil:<cu>% TxUtil:<tu>%`<br>`DB:<v>V(<pct>%)`<br>`[V1:<v>V I1:<i>mA ...]`<br>`T:<t>C H:<h>% P:<p>`<br>`[Lux:<l> UV:<u>]` |
| `ENV` | `ENV` | Real-time environment sensor read (BME280 / SHT31 / BMP280). | `Temp:<t>C Hum:<h>% Pres:<p>hPa`<br>`Lux:<l> UV Index:<u>`<br>`JT ENV: No sensor data available` |
| `POWER` | `POWER` | Real-time I2C power monitor polling (INA219 / INA226 / AXP192 / AXP2101). | `JT POWER:`<br>`V1:<v>V I1:<i>mA`<br>`[V2/V3 per channel]`<br>`ChU:<cu>% TxU:<tu>%`<br>(or `No Sensor Data`) |
| `PMGT` | `PMGT` | Dumps power-management **config** (power-saving flag, shutdown / BT-wait / SDS / light-sleep / min-wake seconds). | `JT PMGT:`<br>`PS:<0/1> SD:<s> BT:<s>`<br>`SDS:<s> LS:<s> MW:<s>` |
| | `PMGT SAVING\|SHUTDOWN\|BT_WAIT\|SDS\|LS\|MIN_WAKE\|ADC <val>` | Set individual power config values. | `JT: Power Config Updated.` / `JT PMGT: Unknown sub-command.` |
| `WIFI` | `WIFI` | Shows WiFi connection status: SSID, RSSI, BSSID, IP. | `JT WIFI:`<br>`SSID:<ssid>`<br>`RSSI:<r> BSSID:<b>`<br>`IP:<ip>`<br>`JT WIFI: Not Connected (Status:<n>)` |
| | `WIFI STATUS\|ON\|OFF\|SSID\|PSK [val]` | Configure WiFi. `STATUS` reports the stored PSK redacted as `[redacted]`. | `JT WIFI STATUS:`<br>`SSID:<ssid>`<br>`PSK:[redacted]`<br>`RSSI:<r> BSSID:<b>`<br>`IP:<ip>`<br>`JT: WiFi Config Updated.` |
| `WIFIOTA` / `OTA` | `WIFIOTA` | ESP32 OTA firmware metadata (paged). Non-ESP32 replies `JT WIFIOTA:` / `Only available on ESP32`. | Paging `JT WiFiOTA Info:`:<br>`  Partition/Available/Kind/Project/`<br>`  Version/Built/IDF/SHA256: ...`<br>Reply: `JT: Paging WiFiOTA info...` |
| `MCUINFO` / `MCU` | `MCUINFO` | Chip/core/flash/heap/die-temp/boot/MAC/RNG/uptime (paged). | Paging `JT MCU Info:`:<br>`  Platform/Features/Flash/IDF/CPU/`<br>`  Heap/DieTemp/Boot/MAC/RNG/Up-time: ...`<br>Reply: `JT: Paging MCU info...` |
| `SENSORS` | `SENSORS` | Enumerates all active I2C sensors detected on the bus and their live readings (paged). | Paging `JT Sensors:`:<br>`  I2C (<n> detected):`<br>`  - <name> @0x<addr> (<ok>)`<br>`  Power (live) / Env (live) / GPS: ...`<br>Reply: `JT: Paging sensors...` |
| `GPIO` | `GPIO` | Shows accepted syntax (bare form does **not** show pin states). | `JT: GPIO <W\|R\|M\|WATCH> <pin> [val]` |
| | `GPIO W <pin> <H\|L>` | Set GPIO pin state. Example: `GPIO W 12 H`. | `JT: GPIO <pin> set to <HIGH\|LOW>` |
| | `GPIO R <pin>` | Read GPIO pin state. | `JT: GPIO <pin> is <HIGH\|LOW>` |
| | `GPIO M <pin> <IN\|OUT\|PU>` | Set pin mode (input / output / pull-up). | — |
| | `GPIO WATCH <pin\|OFF>` | Enable or disable interrupt-driven change notification on a GPIO pin (pins 0-63; re-issuing on a watched pin unwatches it). | `JT: Watching GPIO <pin>` / `JT: Unwatched GPIO <pin>` / `JT: GPIO Watching DISABLED` |
| `PSRAM` | `PSRAM` | PSRAM memory utilization: used, total, and free kilobytes. | `JT PSRAM: <used>/<total> KB (<pct>% used, <free> KB free)` |
| `HEAP` | `HEAP` | Heap usage, largest free block, fragmentation. | `JT HEAP:`<br>`U:<used>(<pct>%) F:<free>`<br>`MaxBlk:<n>`<br>`Frag:<pct>% (Contig)` |
| `RAM` / `MEM` | `RAM` | Full memory diagnostic: heap, PSRAM, Nexus bss size, FreeRTOS stack watermark. | `HEAP U:<u> F:<f> T:<t> MB:<maxblk>`<br>`PSRAM U:<u> F:<f> T:<t>`<br>`NEXUS bss:<size>`<br>`STACK <task> free:<words> words` |
| `TA` (alias `TL`) | `TA [ON\|OFF] [mode-0-2]`| Trace Alert: notifies when another node targets this device with TraceRoute. Mode is a positional integer (no `MODE` keyword): 0=Direct only, 1=Relayed only, 2=All. | `JT: Trace Alert <ON\|OFF> (Mode: <m>)` |
| `META_ALERT` / `MA` | `META_ALERT [ON\|OFF] [mode-0-2]`| Metadata Alert: notifies when another node requests this device's metadata (same positional mode syntax). | `JT: Meta Alert <ON\|OFF> (Mode: <m>)` |
| `INALM` | `INALM [ON\|OFF\|1\|0]` | Toggle 'INA Legacy Math' power computation (not an alarm threshold despite the name). **Was made specifically for version 2.7.23, doesn't make much sense for other versions, but is there regardless.** | `JT INA Legacy Math: <ON\|OFF>`<br>`JT: INA Legacy Math set to <ON\|OFF>.` |
| `METRICS` | `METRICS <node>` | Reports another node's last device metrics (battery voltage, channel utilization) from NodeDB. Requires a node argument. | `JT METRICS <id>:`<br>`Bat:<v>V ChU:<cu>% ...`<br>`JT: No metrics for <id>` |
| `UTIL` | `UTIL` | Channel airtime utilization and transmit duty cycle percentages. | `JT UTIL:`<br>`ChUtil: <pct>%`<br>`TxUtil: <pct>%` |
| `SYS` | `SYS` | System health summary: CPU die temperature, boot reason, heap usage. | `JT SYS:`<br>`DieTemp: <t>C`<br>`Boot: <reason>`<br>`Heap: <used>/<total>` |
| `UPTIME` / `UP` | `UPTIME` | Displays device uptime since boot in days, hours, minutes, and seconds. | `JT UPTIME: <d>d <h>h <m>m <s>s` |

---

## 14. Messaging, Routing & Alerts

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `TEXT` | `TEXT <msg>` | Broadcast a plain text message on the primary channel. | `JT: Text sent to Ch 0` |
| `TEXTC` | `TEXTC <ch> <msg>` | Broadcast a plain text message on channel `<ch>`. Example: `TEXTC 1 Hello Team`. | `JT: Text sent to Ch <ch> (<name>)` |
| `DM` | `DM <!id\|short> <msg>` | Send an encrypted direct message to target node. Example: `DM !defc895b Rendezvous at checkpoint`. | `JT: DM sent to !<id>`<br>`JT: Usage DM !id\|name message` |
| `DM-FORWARD` / `DMF` | `DM-FORWARD [ON\|OFF]` | Forward all received DMs onward automatically (no target-node argument — anything other than `ON` means `OFF`). | bare: `JT DM Forwarding: <ENABLED\|DISABLED>`<br>with arg: `JT: DM Forwarding set to <ON\|OFF>` |
| `DMTEST` | `DMTEST` | Send test payload to the commanding sender to measure DM round-trip latency (no argument). | `JT: DM Test sent to !<id>` |
| `PPING` | `PPING` | Show public ping configuration as a numeric flag dump. | `JT PPING:`<br>`Global:<0/1> RSSI:<0/1> SNR:<0/1> FE:<0/1> Flags:<0/1> DI:<0/1> RI:<0/1> DM:<0/1> EM:<0/1>`<br>`Chans:<list> Custom:<0/1>` |
| | `PPING <ON\|OFF>` | Enable or disable automatic response to public pings. | `JT: Public Ping ENABLED` / `JT: Public Ping DISABLED` |
| | `PPING EMOJI <OFF\|INDIRECT\|ALWAYS>`| Set emoji ping acknowledgment mode (aliases `INDR`, `ALW`). | `JT: Ping response in TEXT mode.` / `JT: Ping response in EMOJI mode for INDIRECT pings.` / `JT: Ping response in ALWAYS-EMOJI mode.` |
| | `PPING HEADER <text\|OFF>`| Configure custom prefix header for public ping responses. | `JT: Custom PPING Header set to '<text>'` / `JT: Custom PPING Header DISABLED.` |
| | `PPING RSSI\|SNR\|FREQERR\|FLAGS\|OM\|ALLOWDM <ON\|OFF>`, `PPING DIRECT IGNORE`, `PPING IGNORE REMOTE`, `PPING CHAN [ON\|OFF] <ch>`, `PPING SETHEX <hex>`, `PPING GETHEX` | Additional per-metric and routing toggles (see `PPING` usage line). | Usage: `JT PPING: ON\|OFF\|RSSI\|SNR\|FREQERR\|FLAGS\|OM\|DIRECT IGNORE\|IGNORE REMOTE\|ALLOWDM\|SETHEX\|GETHEX` |
| `BELL` | `BELL` | Triggers single hardware buzzer alert bell tone. | `JT: Local BELL triggered` |
| `PLAY` | `PLAY <rtttl_melody>` | Plays an RTTTL ringtone melody on hardware buzzer. | `JT: Playing custom RTTTL...` |
| `STARWARS` | `STARWARS` | Plays the Imperial March on hardware buzzer. | `JT: May the Force be with you...` |
| `OKTOMQTT` | `OKTOMQTT [ON\|OFF]` (alias `OKMQTT`/`OMQ`) | Toggle whether outgoing text messages set the `ok_to_mqtt` flag (LoRa config bit 105). Value aliases: `TRUE/ENABLE/1`, `FALSE/DISABLE/0`; `STATUS`/`STAT` query. | `JT OKTOMQTT: <ON\|OFF> (LoRa config bit 105, ok_to_mqtt on outgoing packets)`<br>`JT: OKTOMQTT set to <ON\|OFF> (saved; next outgoing packet will carry ok_to_mqtt=<0\|1>)` |
| **`GOODPRACTICES`** / **`BUENASPRACTICAS`** / `GP` / `BP` | `GOODPRACTICES` | Apply the Spanish Meshtastic Community airtime best practices: NodeInfo 72h, Position 72h, SmartPos OFF, active telemetry < 12h bumped to minimum of 24h. | `JT: Good Practices applied: NI=72h, POS=72h, SmartPos=OFF, TelemBumped=<count> (to 24h)` |
| `DU` / `DECODEURL` | `DU <url>` | Decode and parse a Meshtastic **node-share** URL (SharedContact payload: id, names, HW, role, public key). | `JT DU !<id>:`<br>`LName: <long>`<br>`SName: <short>`<br>`HW: <hw> Role: <role>`<br>`PK: <first2>..<last2>` |
| `MQTT` | `MQTT` | Displays MQTT client status and current override mode. | `JT MQTT [<state>]:`<br>`Status: <status>`<br>`Mode: <mode>`<br>`Srv: <srv>`<br>`Map: <map>` |
| | `MQTT <DEFAULT\|ALWAYS\|STRICT>` | Set MQTT override mode (`ALWAYS`=`ALWAYS-UPLOAD`/`1`, `STRICT`=`STRICT-PRIVACY`/`2`, `DEFAULT`=`0`). Note: `STATUS`/`CONNECT` are **not** valid subcommands. | `JT: MQTT Override set to <mode>` / `JT: Invalid MQTT Mode (DEFAULT\|ALWAYS\|STRICT)` |
| `SENDPOS` | `SENDPOS` | Inject current GPS position to DM or channel. | `JT: Position injected to <DM\|Channel>` |
| `SENDMYNODE` / `SENDNODE` | `SENDMYNODE` (aliases `SMN`, `SMI`, `SENDMYINFO`) | Broadcast **this node's** NodeInfo packet immediately (options `C <ch>`, `H <hl> <hs>`, `M`). | `JT: NodeInfo injected for !<id> on Ch:<n> (<name>)<opts>` |
| | `SENDNODE <!id\|short>` (alias `SN`) | Inject **another node's** NodeInfo from NodeDB (not a self-broadcast). | `JT: NodeInfo injected for !<id> on Ch:<n> (<name>)<opts>`<br>`JT: Node <id> not in DB or invalid` |
| `ZH` | `ZH [ADD <id>\|DEL <id>\|SYNC\|LIST\|...]` | Zero-Hop routing group management — full subcommand table in [§4](#4-zero-hop-routing-zh). Note: removal is `DEL`, not `REM`. | `JT: ZH ID 0x<hex> <ADDED\|REMOVED>` |

---

## 15. Aliases, Macros, Groups & Automation

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `ALIAS` | `ALIAS LIST` | List all 16 custom short node aliases stored in vault (paged). | Paging `Aliases`:<br>`<alias> -> !<id>` per line<br>Reply: `JT: Paging aliases to mesh...` |
| | `ALIAS SET <!id> <alias>` | Assign a custom 4-character alias to a node ID. Example: `ALIAS SET !defc895b HQ01`. | `JT: Alias '<alias>' set for !<id>` |
| | `ALIAS DEL <alias>` | Delete an assigned alias **by alias name** (not node ID). | `JT: Alias '<alias>' deleted` / `JT: Alias not found` |
| `MACRO` / `MC` | `MACRO LIST` | List all 4 stored command macros (paged). | Paging `Macros`:<br>`<name>: <cmds>` per line<br>Reply: `JT: Paging macros to mesh...` |
| | `MACRO SET <name> <cmds>` | Store command sequence (separated by `;`; steps run with 10s delay). Slot is chosen by name match or first free. Example: `MACRO SET PATROL RSSI; POS; UTIL`. | `JT: Macro '<name>' stored` |
| | `MACRO RUN <name>` / **`MR <name>`** | Execute stored macro sequence **by name**. | `JT: Running macro sequence '<name>'...` (multi-step)<br>`JT: Executing macro '<name>'...` (single) |
| | `MACRO DEL <name>` | Delete macro by name. | `JT: Macro deleted` / `JT: Macro not found` |
| `GROUP` | `GROUP LIST` | List the 4 group name slots (these are just names; groups do not store node membership). | `JT Groups: [g1] [g2] [g3] [g4]` |
| | `GROUP ADD <name>` | Store a group name (max 8 chars) in the first free slot. | `JT: Group '<name>' ADDED to slot <n>` |
| | `GROUP REM <name>` | Remove a group name from its slot. | `JT: Group slot <n> REMOVED` |
| | `GROUP CLEAR` | Clear all 4 group slots. | `JT: All groups CLEARED` |
| `CRON` | `CRON LIST` | List active automated recurring task slots (8 slots max, paged). | Paging rows `[<n>] <PER\|SCH\|AT>:<val> -> <cmd>`<br>Reply: `JT: Paging CRON list...` |
| | `CRON ADD PER <secs> <cmd>` | Run `<cmd>` every `<secs>` **seconds** (minimum 60). Example (every 30 min): `CRON ADD PER 1800 NOISE`. Slot auto-allocated. | `JT: CRON Job <n> ADDED.` |
| | `CRON ADD SCH HH:MM <cmd>` | Run `<cmd>` daily at `HH:MM`. | `JT: CRON Job <n> ADDED.` |
| | `CRON ADD AT <val> <cmd>` | One-shot scheduled run. | `JT: CRON Job <n> ADDED.` |
| | `CRON REM <id>` | Remove cron job `<id>`. | `JT: CRON Job <n> REMOVED.` |

---

## 16. Filesystem & System Utilities

| Command | Subcommands / Syntax | Description & Examples | Exact Output / Template |
|---------|----------------------|------------------------|-------------------------|
| `TIME` | `TIME` | Displays real-time clock (RTC) current UTC epoch, local time, and synchronization source. | `JT TIME (<source>):`<br>`UTC: <epoch>`<br>`LOC: <YYYY-MM-DD HH:MM:SS>` |
| `SETTIME` | `SETTIME <epoch_sec>` | Manually set system RTC clock time from Unix epoch timestamp (values below 1700000000 are rejected). | `JT: Time set to <epoch_sec>` |
| `REBOOT` | `REBOOT` | Reboots the device (initiates safe shutdown with a mandatory 20-second delay). | `JT: Device rebooting in 20 seconds...` |
| `LS` | `LS` | List all files stored on internal LittleFS / SPIFFS filesystem. | Paging `File List (LS)`: <br>`<file>:<size>B` per line<br>Reply: `JT: Paging file list to mesh...` |
| `CAT` | `CAT <filename>` | Read file and return its content **base64-encoded** (files over 700B are refused). Example: `CAT /prefs/db.proto`. | Paging `File Download (B64)`:<br>`JT: Paging file '<filename>' (<size>B) to mesh...`<br>`JT: File too large (<size>B > 700B)` |
| `RM` | `RM <filename>` | Delete file from filesystem. | `JT: File '<filename>' deleted`<br>`JT: Failed to delete '<filename>'`<br>`JT: File '<filename>' not found` |
| `FS` | `FS` | Displays filesystem storage capacity: total, used, and free bytes (platform specific). | ESP32: `JT FS:`<br>`Used: <u>B`<br>`Total: <t>B`<br>`Free: <f>B`<br>nRF52: `JT FS (nRF52):`<br>`  LittleFS: <u>B / <t>B (<f>B free)`<br>`  Warm ring: 12288B reserved (raw flash, 0xEA000-0xED000)` |

---

## 17. Complete Alphabetical Command Index

| Command | Category | Primary Function |
|---------|----------|------------------|
| `ACC` | §8 Identity & NodeDB | Accelerometer motion sample (X/Y/Z g) |
| `ADDCH` | §12 Channels & Config | Add a new channel slot |
| `ADMINS` | §8 Identity & NodeDB | List authorized admin public keys |
| `ALIAS` | §15 Aliases, Macros & Automation | Custom 4-char node alias management |
| `APC` | §7 Radio & RF Control | Automatic Power Control (dynamic SNR step-down) |
| `BASETEXTS` | §7 Radio & RF Control | Client-base text-only rebroadcast toggle |
| `BD` | §11 Vault & Persistence | Broadcast delay / paging jitter settings |
| `BELL` | §14 Messaging & Routing | Hardware buzzer single alert bell |
| `CAT` | §16 Filesystem & Utilities | Display file content from LittleFS |
| `CHANNELSTATS` / `CHS` | §9 Observation & SIGINT | Traffic volume breakdown per channel |
| `CHASH` / `CH` / `CHANNELS` | §12 Channels & Config | Channel list with roles (CH) or hashes (CHASH) |
| `CH-HOPMASK` / `CHM` | §6 Hop-Aware Routing | Channel-specific hop masking |
| `CLIENTALWAYS` | §7 Radio & RF Control | Enforce client-timing priority |
| `CLIENTSCAN` | §9 Observation & SIGINT | Survey repeating clients via test-packet echoes |
| `CLRSTATS` / `CS` | §9 Observation & SIGINT | Clear volatile runtime statistics |
| `COMPILATION` / `COMP`| §2 Help & Build Info | Firmware compilation and target info |
| `CONFIG` | §12 Channels & Config | Device protobuf configuration summary |
| `CRON` | §15 Aliases, Macros & Automation | 8-slot periodic command scheduler |
| `CRSTATS` / `CRS` | §9 Observation & SIGINT | Coding Rate distribution stats |
| `DBFLUSH` | §11 Vault & Persistence | Force immediate NodeDB flash sync |
| `DBINFO` | §8 Identity & NodeDB | NodeDB memory and cache health |
| `DELCH` | §12 Channels & Config | Delete channel slot |
| `DELNODE` | §3 Firewall & Filtering | Delete node record from database |
| `DM` | §14 Messaging & Routing | Send encrypted Direct Message |
| `DM-FORWARD` / `DMF` | §14 Messaging & Routing | Automatically forward incoming DMs |
| `DMTEST` | §14 Messaging & Routing | Probe DM round-trip latency |
| `DROP` / `UNDROP` | §3 Firewall & Filtering | Enable/disable inbound packet drop rules |
| `DROPS` | §3 Firewall & Filtering | List active inbound drop filters |
| `DU` / `DECODEURL` | §14 Messaging & Routing | Decode node-share contact URL |
| `ENV` | §13 Hardware & Sensors | Real-time I2C environment sensor data |
| `FAV` / `UNFAV` | §3 Firewall & Filtering | Favorite node tagging |
| `FAVS` | §3 Firewall & Filtering | List all favorite nodes |
| `FEATURES` | §5 Security & Features | 64-bit runtime feature bitmask toggles |
| `FFAV` / `FUNFAV` | §3 Firewall & Filtering | Force favorite / force unfavorite node |
| `FIGNORE` / `FUNIGNORE`| §3 Firewall & Filtering | Force ignore node ID |
| `FIREWALL` / `FW` | §3 Firewall & Filtering | Aggregated firewall status report |
| `FIREWALLSTATS` / `FWS`| §9 Observation & SIGINT | Packet drop counters and stats |
| `FIXMYINFO` / `FMI` | §8 Identity & NodeDB | Repair own NodeDB entry fields in place |
| `FNODE` | §8 Identity & NodeDB | Full per-node detail listing (search or !id) |
| `FREEZE` | §11 Vault & Persistence | Subsystem filesystem write freeze |
| `FREQSCAN` | §9 Observation & SIGINT | Spectrum slice peak RSSI sweep |
| `FS` | §16 Filesystem & Utilities | Filesystem capacity and free space |
| `FSIG` | §3 Firewall & Filtering | 8-slot payload signature firewall |
| `FULLCONFIGDUMP` | §12 Channels & Config | Full device protobuf config dump |
| `G3` / `FRONTEND` | §7 Radio & RF Control | Station G3 PA / LNA frontend control |
| `GETTXPOWER` | §7 Radio & RF Control | Read active transmit power |
| `GPIO` | §13 Hardware & Sensors | GPIO pin read, set, and change watch |
| `GROUP` | §15 Aliases, Macros & Automation | Group name slots (4 names, no membership) |
| `HEAP` | §13 Hardware & Sensors | Free and used heap memory bytes |
| `HELP` | §2 Help & Build Info | Interactive paged help system |
| `HEXDROPS` | §3 Firewall & Filtering | Inbound drop rules as 64-bit hex |
| `HEXOUTDROPS` | §3 Firewall & Filtering | Outbound drop rules as 64-bit hex |
| `HOPMASK` | §6 Hop-Aware Routing | Global hop_start / hop_limit override |
| `HOPMATRIX` / `HM` | §6 Hop-Aware Routing | 2D Hop Start vs Hop Limit matrix |
| `HOPSCALE` / `HSC` | §6 Hop-Aware Routing | Dynamic hop scaling status (read-only) |
| `HOPSTATS` / `HS` | §6 Hop-Aware Routing | Hop count distribution histogram |
| `HW` | §8 Identity & NodeDB | Hardware chip, CPU frequency, and MAC |
| `IDR` / `UNIDR` / `IGNORE-DUPE-RELAY` | §3 Firewall & Filtering | Per-relay duplicate suppression mask |
| `IGNORE` / `UNIGNORE` | §3 Firewall & Filtering | NodeDB ignore list management |
| `IGNORED` | §3 Firewall & Filtering | List ignored nodes |
| `INALM` | §13 Hardware & Sensors | INA power sensor alarm thresholds |
| `INFO` | §2 Help & Build Info | Node identity, version, and role |
| `INFRA` | §12 Channels & Config | List mesh infrastructure router nodes |
| `INVALIDS` / `INV` | §3 Firewall & Filtering | Packet sanity validation drop rules |
| `JENCORE` / `JC` / `MC` | §7 Radio & RF Control | JenCore interop, repeater & companion bridge |
| `JRNODE` / `RNODE` / `RN` | §7 Radio & RF Control | Reticulum RNode engine & KISS bridge |
| `LCS` / `LORACHIPSTATS` | §9 Observation & SIGINT | LoRa chip state time percentages |
| `LNA` | §7 Radio & RF Control | Show LNA gain mode (Heltec V4.3) |
| `LOAD` | §11 Vault & Persistence | Reload settings from flash vault |
| `LORA` / `LR` | §7 Radio & RF Control | Compact radio modem settings |
| `LORA-STATUS` / `LRS` | §7 Radio & RF Control | Real-time hardware radio status |
| `LR2021_SF` / `MULTISF` / `MSF`| §7 Radio & RF Control | LR2021 multi-SF concurrent channels |
| `LS` | §16 Filesystem & Utilities | List LittleFS files |
| `MACRO` / `MC` | §15 Aliases, Macros & Automation | Stored command macro execution |
| `MCUINFO` / `MCU` | §13 Hardware & Sensors | Detailed MCU core and flash info |
| `META_ALERT` / `MA` | §13 Hardware & Sensors | Alert on metadata request receipt |
| `METRICS` | §13 Hardware & Sensors | Another node's battery / channel-utilization metrics |
| `MODULES` / `MOD` | §12 Channels & Config | List active firmware modules |
| `MQTT` | §14 Messaging & Routing | MQTT client status and connection |
| `MQTTSTATS` / `MQS` | §9 Observation & SIGINT | MQTT uplink and downlink metrics |
| `NAME` | §8 Identity & NodeDB | Change device Long Name |
| `NEIGHBORS` / `DIRECT`| §8 Identity & NodeDB | List 0-hop direct neighbors |
| `NETSCAN` | §9 Observation & SIGINT | 60-second neighborhood scan |
| `NIGN` | §3 Firewall & Filtering | Nexus memory-resident ignore table |
| `NODEDB` / `DB` | §8 Identity & NodeDB | Query node entry details from DB |
| `NODES` | §8 Identity & NodeDB | Total and active node counts |
| `NODES28` / `N28` | §8 Identity & NodeDB | List nodes running v2.8+ protocols |
| `NOISE` | §9 Observation & SIGINT | Immediate RF noise floor measurement |
| `OKTOMQTT` | §14 Messaging & Routing | Toggle MQTT export flag on packets |
| `GOODPRACTICES` / `BUENASPRACTICAS` / `GP` / `BP` | §14 Messaging & Routing | Apply stealth and airtime good practices (NI/POS 72h, SmartPos OFF, telem 24h) |
| `OUTDROP` / `UNOUTDROP`| §3 Firewall & Filtering | Outbound (TX) drop rule toggles |
| `OUTDROPS` | §3 Firewall & Filtering | List active outbound drop rules |
| `OWNER` | §8 Identity & NodeDB | Set Long Name and Short Name |
| `PACKETSTATS` / `PS` | §9 Observation & SIGINT | Packet counts categorized by PortNum |
| `PHONE` | §8 Identity & NodeDB | Local phone connection status |
| `PLAY` | §14 Messaging & Routing | Play custom RTTTL ringtone melody |
| `PMGT` | §13 Hardware & Sensors | Power management IC diagnostic state |
| `PORT-HOPMASK` / `PHM`| §6 Hop-Aware Routing | Port-specific hop limit override |
| `POS` | §8 Identity & NodeDB | Live GPS coordinates, altitude, PDOP |
| `POWER` | §13 Hardware & Sensors | I2C power monitor voltage & current |
| `PPING` | §14 Messaging & Routing | Public ping response configuration |
| `PRESET` / `SETPRESET` | §7 Radio & RF Control | Load standard or custom LoRa preset |
| `PRWHITELIST` / `PRALLOW`| §3 Firewall & Filtering | Whitelist allowed private channel hashes |
| `PRBLACKLIST` / `PRBLOCK`| §3 Firewall & Filtering | Blacklist blocked private channel hashes |
| `PSRAM` | §13 Hardware & Sensors | PSRAM memory usage statistics |
| `RAM` / `MEM` | §13 Hardware & Sensors | Heap, PSRAM and stack diagnostic |
| `RATELIMIT` / `RL` | §12 Channels & Config | Traffic rate limit bucket thresholds |
| `RATELIMITSTAT` | §12 Channels & Config | Rate limiter drop counters |
| `RDROP` / `RUNDROP` | §3 Firewall & Filtering | Relay ID byte drop mask filter |
| `REBOOT` | §16 Filesystem & Utilities | Safe reboot with 20s delay |
| `RELAYS` / `RY` | §9 Observation & SIGINT | All active relay IDs with D/R/T counts |
| `RM` | §16 Filesystem & Utilities | Delete file from filesystem |
| `ROLE` | §7 Radio & RF Control | Show active device role |
| `RSSI` / `RS` | §9 Observation & SIGINT | Last received packet RF metrics |
| `SHORTRSSI` / `SR` | §9 Observation & SIGINT | Minimal 2-line packet RF report |
| `RSSITEL` / `RT` | §9 Observation & SIGINT | Compact RF & telemetry packet report |
| `RSSILOG` / `PL` | §9 Observation & SIGINT | 32-entry packet history ring buffer (shows last 5) |
| `SAVE` | §11 Vault & Persistence | Commit RAM configuration to vault |
| `SEARCHKEY` | §9 Observation & SIGINT | Find node by exact 32-byte public key |
| `SEARCHRELAY` / `SRELAY`| §9 Observation & SIGINT | Find nodes matching relay byte ID |
| `SECURITY` | §5 Security & Features | Signature auth, DM gate, replay protection |
| `SENDMYNODE` / `SENDNODE` / `SMN` / `SN`| §14 Messaging & Routing | Broadcast NodeInfo (own node, or any target node) |
| `SENDPOS` | §14 Messaging & Routing | Broadcast local position packet |
| `SENDSTATUS` | §9 Observation & SIGINT | Send freeform text as a Node Status message |
| `SETCHNAME` | §12 Channels & Config | Rename channel slot |
| `SETCHPSK` | §12 Channels & Config | Set channel encryption key |
| `SETCONFIG` | §12 Channels & Config | Direct protobuf configuration write |
| `SETHEX` | §3 Firewall & Filtering | Set inbound drops from 64-bit hex |
| `SETHEXOUT` | §3 Firewall & Filtering | Set outbound drops from 64-bit hex |
| `SETFLRC` | §7 Radio & RF Control | Switch modem to FLRC (SX128x/LR2021 only) |
| `SETFSK` | §7 Radio & RF Control | Switch modem to FSK/GFSK modulation |
| `SETLNA` | §7 Radio & RF Control | Set LNA gain mode |
| `SETLORA` | §7 Radio & RF Control | Reconfigure SF, BW, CR, Frequency |
| `SETPOWER` / `SETPWR` / `SETTXPOWER` | §7 Radio & RF Control | Set radio transmit power (dBm) |
| `SETPREAMBLE` / `SETPRE`| §7 Radio & RF Control | Configure custom preamble length |
| `SETREBROADCAST` | §7 Radio & RF Control | Configure rebroadcast mode |
| `SETROLE` | §7 Radio & RF Control | Update device operating role |
| `SETSYNCWORD` / `SETSW`| §7 Radio & RF Control | Set custom radio sync word |
| `SETTIME` | §16 Filesystem & Utilities | Set system RTC clock time |
| `SETTINGS` / `ST` | §12 Channels & Config | Operational switches and status |
| `SHARE` | §12 Channels & Config | Generate node-share contact URL / QR link |
| `SIGSTATS` / `SS` | §9 Observation & SIGINT | Packet privacy classification & noise floor |
| `SKEY` | §12 Channels & Config | Secondary dynamic admin master keys |
| `SORTEDCHANNELSTATS` | §9 Observation & SIGINT | Sorted channel traffic distribution |
| `SORTEDPACKETSTATS` | §9 Observation & SIGINT | Sorted PortNum traffic activity list |
| `SORTEDRELAYSTATS` | §9 Observation & SIGINT | Sorted relay ID activity list |
| `SPLIT` | §7 Radio & RF Control | Split-Tier Dual-Preset Engine (MSF/Roof/Client) |
| `STARWARS` | §14 Messaging & Routing | Play Star Wars melody on buzzer |
| `STATUSPS` | §9 Observation & SIGINT | Send packet-volume summary as Node Status |
| `SYS` | §13 Hardware & Sensors | CPU temperature, boot reason, heap |
| `TA` | §13 Hardware & Sensors | TraceRoute receipt alert toggle |
| `TELEMETRY` | §13 Hardware & Sensors | Battery, temperature, pressure telemetry |
| `TEXT` | §14 Messaging & Routing | Broadcast text message on channel 0 |
| `TEXTC` | §14 Messaging & Routing | Broadcast text message on channel `<ch>` |
| `TIER` | §7 Radio & RF Control | Multi-SF tier table & per-channel TX mapping (LR2021) |
| `TIME` | §16 Filesystem & Utilities | Current date, time, and sync source |
| `TM` | §12 Channels & Config | TrafficManagementModule tuning & stats |
| `TRACE` / `TR` | §9 Observation & SIGINT | Trace route to destination node |
| `TRANSSTATS` / `TS` | §9 Observation & SIGINT | Transport interface packet distribution |
| `TRUSTED` | §8 Identity & NodeDB | Named trusted-channel slots (2) |
| `TX ON` / `TX OFF` | §7 Radio & RF Control | Transmitter RF emission killswitch |
| `UPTIME` / `UP` | §13 Hardware & Sensors | Device uptime since boot |
| `UTIL` | §13 Hardware & Sensors | Channel utilization and airtime % |
| `VAULT` / `VST` | §11 Vault & Persistence | Vault schema version, size, window |
| `VAULT DIFF` / `VDF` | §11 Vault & Persistence | Diff RAM settings against saved vault |
| `VERIFYSTATS` / `VS` | §9 Observation & SIGINT | XEdDSA signature verification stats |
| `VERSION` / `V` | §2 Help & Build Info | Firmware version string |
| `VOLT` / `VOLTAGE` | §7 Radio & RF Control / §13 Hardware & Sensors | 4-threshold voltage automation engine |
| `WATCH` | §10 Target Watchlist | In-depth continuous node RF & hop tracker |
| `WIFI` | §13 Hardware & Sensors | WiFi connection status, SSID, IP |
| `WIFIOTA` / `OTA` | §13 Hardware & Sensors | ESP32 OTA firmware metadata |
| `ZH` | §4 Zero-Hop Routing (ZH) | Zero-Hop relay mask & routing state |

