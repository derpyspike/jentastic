# Nexus Firmware Fix: `/nexus-local` and `/jt-local` Ingress, Routing & Paging

**Author:** Jirachi ⭐ on Noctali (`Noctali.local`)  
**Target:** Magnemite / Pokéteam  
**Branch:** `fix/nexus-local-commands`  
**Commit:** `9a696ee2f` (`fix(nexus): resolve /nexus-local and /jt-local command failures, routing & paging`)  
**Base Commit:** `806522535`  
**Date:** 2026-10-19  

---

## 1. Overview & Context

Users reported that `/nexus-local` and `/jt-local` commands appeared dead, failed to execute, or produced complete silence in the client apps (Android, iOS, Web), whereas targeting the node explicitly via `/nexus-node <shortname> <cmd>` (or `/jt-node ...`) worked reliably.

This document details the architectural root causes that caused `-local` to fail, how `/nexus-node` behaved differently, and the comprehensive fix implemented and verified across both nRF52 and ESP32 platforms.

---

## 2. Root Cause Analysis: Why `/nexus-node` Worked and `-local` Failed

### A. Missing Ingress Confirmation in `MeshService::handleToRadio`
- **Location:** `src/mesh/MeshService.cpp:349-357`
- **Mechanism:**
  When a local client (phone or serial/web) sends a command, `handleToRadio()` invokes `nexusModule->handleLocal(p)`. For `-local` commands, `handleLocal` returned `ProcessMessage::STOP`.
  Because `STOP` caused `handleToRadio()` to return immediately without calling `sendToMesh()`, the routine that sends `sendQueueStatusToPhone(qs, ERRNO_OK, mp.id)` was **completely bypassed**.
- **User Impact:**
  The phone client was waiting for an acknowledgment that its outgoing message was accepted. Without this QueueStatus confirmation, the Meshtastic client app marked the outgoing `/nexus-local` command as stuck in "Sending..." or "Failed" (spinner or cloud with slash).

### B. Destination Corruption in `handleLocal` Broke Channel vs DM & Paging
- **Location:** `src/modules/NexusModule.cpp:2576` (prior code)
- **Mechanism:**
  `handleLocal` previously executed:
  ```cpp
  meshtastic_MeshPacket local = mp;
  local.to = myNodeInfo.my_node_num; // <-- Forced destination to our own node!
  handleNexusCommand(local, tail, false, mp.id, true);
  ```
  Even when an operator typed `/nexus-local HELP` in their public channel (where `mp.to == 0xFFFFFFFF`), `local.to` was overwritten to `myNodeInfo.my_node_num`.
  In `handleNexusCommand`:
  1. `bool isDM = (mp.to == myNodeInfo.my_node_num);` evaluated to `true`.
  2. For paged commands (e.g. `HELP`, `SYS`, `NODES`, `VAULT`, `FIREWALL`), commands called:
     ```cpp
     startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, ...);
     ```
     Because `mp.to == myNodeInfo.my_node_num` was true, it chose `mp.from`.
     However, in `MeshService::handleToRadio` (line 323), `p.from = 0;`!
     As a result, `startPaging` was invoked with **`to = 0`** (an invalid node ID).

### C. Client Thread Disconnect (`toPhoneOnly` hardcoded `0xFFFFFFFF`)
- **Location:** `src/modules/NexusCommands.cpp:8963` and `src/modules/NexusModule.cpp:5159`
- **Mechanism:**
  When `toPhoneOnly == true`, `handleNexusCommand` bypassed `queueMeshReply` and invoked:
  ```cpp
  sendToPhone(0xFFFFFFFF, mp.channel, replyId, replyMsg, rLen);
  ```
  And `NexusModule::sendReply()` also hardcoded `sendToPhone(0xFFFFFFFF, ...)`, ignoring `context.to`.
- **User Impact:**
  If an operator ran `/nexus-local` in a Direct Message (DM) to their radio:
  The phone client strictly filters DM conversations to packets matching the conversation peer. A packet addressed to `0xFFFFFFFF` is categorized as a channel broadcast. The DM chat screen displayed **absolute silence**.

### D. Silent RF Ingress Rejection
- **Location:** `src/modules/NexusModule.cpp:2451-2457`
- **Mechanism:**
  `-local` is intended solely for local interfaces (BLE, USB serial, Web, TCP). If received over LoRa RF, it was logged as a Security Violation and silently dropped without informing the sender why it didn't work.

---

## 3. Summary of Fixes Applied

1. **Immediate QueueStatus Acknowledgment (`src/modules/NexusModule.cpp`):**
   In `NexusModule::handleLocal()`, before executing the command and returning `ProcessMessage::STOP`, we now immediately call:
   ```cpp
   if (router && service) {
       meshtastic_QueueStatus qs = router->getQueueStatus();
       service->sendQueueStatusToPhone(qs, ERRNO_OK, mp.id);
   }
   ```
   This clears the client app's pending transmit state, turning the spinner/cloud into a confirmed sent message.

2. **Accurate Origin & Destination Preservation (`src/modules/NexusModule.cpp`):**
   In `NexusModule::handleLocal()`:
   - If `local.from == 0`, set it to `nodeDB->getNodeNum()` so downstream reply builders have a valid sender.
   - If `local.to == 0`, normalize it to `NODENUM_BROADCAST`.
   - Preserve `local.to` (do not clobber it to `myNodeInfo.my_node_num` if it was broadcast).
   - Compute `bool isBcast = (local.to == NODENUM_BROADCAST || isBroadcast(local.to));` and pass it to `handleNexusCommand()`.

3. **Dynamic Destination in `sendToPhone` and `sendReply` (`src/modules/NexusCommands.cpp` & `NexusModule.cpp`):**
   - In `NexusCommands::handleNexusCommand` (both private dispatcher tail and public dispatcher tail):
     ```cpp
     uint32_t to = isDM ? (mp.from ? mp.from : myNodeInfo.my_node_num) : 0xFFFFFFFF;
     sendToPhone(to, mp.channel, replyId, replyMsg, rLen);
     ```
   - In `NexusModule::sendReply()`: replaced hardcoded `0xFFFFFFFF` with `context.to`:
     ```cpp
     if (context.phoneOnly)
         return sendToPhone(context.to, context.channel, context.replyId, message, length);
     ```
   - In `NexusModule::startPaging()`: added safety fallback `if (to == 0) to = 0xFFFFFFFF;` to prevent invalid destination 0.

4. **Empty Command Fallback (`src/modules/NexusCommands.cpp`):**
   If an operator sends `/nexus` or `/nexus-local` without arguments (`*cmd == '\0'`), the firmware now responds with:
   `JT: Ready. Send HELP for command list.`
   instead of the confusing `JT: Unknown command ''`.

5. **Helpful Mesh Rejection Notice (`src/modules/NexusModule.cpp`):**
   If a remote node sends `/nexus-local` over RF, unless `SILENT_LOG` is active, the node replies back over the mesh with:
   `JT SECURITY: -local prefix is for local interface only. Use -node over mesh.`

---

## 4. Verification Receipts

Both primary targets were compiled cleanly using PlatformIO 6.1.19:

1. **`tracker-t1000-e` (nRF52840 + LR1110):**
   - **Result:** `SUCCESS in 00:02:56.800`
   - **RAM:** 39.8% (used 99,100 B of 248,832 B)
   - **Flash:** 76.1% (used 620,004 B of 815,104 B)
   - **Artifacts:** `firmware-tracker-t1000-e-2.8.103.8065225.hex`, `.uf2`, `.zip`

2. **`heltec-v3` (ESP32-S3 + SX1262):**
   - **Result:** `SUCCESS in 00:05:59.067`
   - **DIRAM:** 55.28% (used 188,919 B of 341,760 B)
   - **Flash:** 63.5% (used 2,121,103 B of 3,342,336 B)
   - **Artifacts:** `firmware-heltec-v3-2.8.103.8065225.bin`, `.factory.bin`

---
*Generated by Jirachi ⭐ for Jenny and Magnemite.*
