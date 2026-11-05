# Third-party licenses

This directory collects the license texts and attribution for the upstream
projects that Nexus / Jentastic derives from or references. The repository's
own terms remain in the root: `LICENSE` (GPL-3.0) and `LICENSE-AGPL`
(AGPL-3.0).

## Components

| Component | Upstream | Maintainer / organisation | License | Local text |
| --- | --- | --- | --- | --- |
| Meshtastic firmware | <https://github.com/meshtastic/firmware> | Meshtastic project and contributors | GPL-3.0 | [`GPL-3.0.txt`](GPL-3.0.txt) |
| Meshtacore firmware | <https://github.com/xejarque/meshtacore-firmware> | Xavi (`xejarque`) and contributors | GPL-3.0 + AGPL-3.0 | [`GPL-3.0.txt`](GPL-3.0.txt), [`AGPL-3.0.txt`](AGPL-3.0.txt) |
| MeshCore | <https://github.com/meshcore-dev/MeshCore> | Scott Powell / rippleradios.com; MeshCore project | MIT | [`MIT-MeshCore.txt`](MIT-MeshCore.txt) |
| Dutch-MeshCore MeshCore | <https://github.com/Dutch-MeshCore/MeshCore> | Scott Powell / rippleradios.com; Dutch-MeshCore | MIT | [`MIT-Dutch-MeshCore.txt`](MIT-Dutch-MeshCore.txt) |
| ZephCore | <https://github.com/liquidraver/ZephCore> | `liquidraver` and ZephCore contributors | MIT | [`MIT-ZephCore.txt`](MIT-ZephCore.txt) |
| microReticulum | <https://github.com/attermann/microReticulum> | Chad Attermann (`attermann`) and contributors | Apache-2.0 | [`Apache-2.0.txt`](Apache-2.0.txt) |
| RNode_Firmware | <https://github.com/markqvist/RNode_Firmware> | Mark Qvist (`markqvist`) and contributors | GPL-3.0 | [`GPL-3.0-RNode_Firmware.txt`](GPL-3.0-RNode_Firmware.txt) |
| microReticulum_Firmware | <https://github.com/attermann/microReticulum_Firmware> | Chad Attermann (`attermann`) and contributors | GPL-3.0 | [`GPL-3.0-RNode_Firmware.txt`](GPL-3.0-RNode_Firmware.txt) |
| Columba | <https://github.com/torlando-tech/columba> | `torlando-tech` and Columba contributors | MPL-2.0 | [`MPL-2.0.txt`](MPL-2.0.txt) |

## Attribution notes

- `GPL-3.0.txt` is the verbatim GNU GPL v3 text shipped with this repository.
  It is identical to the copies in Meshtastic firmware and Meshtacore firmware.
- `GPL-3.0-RNode_Firmware.txt` is the GPL v3 text shipped by RNode_Firmware and
  microReticulum_Firmware. It uses the updated FSF URLs; the license terms are
  identical to `GPL-3.0.txt`.
- `AGPL-3.0.txt` is the verbatim GNU AGPL v3 text. It covers the Meshtacore
  firmware and the Nexus / Jentastic additions in this repository.
- The MIT texts include the upstream copyright lines verbatim:
  MeshCore and Dutch-MeshCore MeshCore (`Copyright (c) 2025 Scott Powell /
  rippleradios.com`; the Dutch-MeshCore didn't change it from upstream but I included it anyway,
  and ZephCore (`Copyright (c) 2025-2026 ZephCore contributors`).
- `Apache-2.0.txt` and `MPL-2.0.txt` are the standard license texts. The
  upstream repositories did not contain a separate NOTICE file when these
  copies were taken; the copyright holders are listed in the table above.
- The license texts were retrieved from the upstream repositories on
  2026-11-05.
