<div align="center" markdown="1">

# Jentastic/Nexus firmware
</div>

This is a friendly custom version (soft-fork if you wanna call it that) of the Meshtastic® firmware
Made for use in the Spanish mesh as well as for personal use. Made with human-reviewed, human-curated agentic engineering, aka AI agents make PRs and I review, fix them up and merge them.

Based on upstream Meshtastic firmware **v2.8.1**. Versions append two digits to the official version, starting with `00` — so **2.8.103** is JT/Nexus version **03**, the fourth Nexus release built on 2.8.1 (`00`, `01`, `02`, `03`).

Meshtastic® is a registered trademark of Meshtastic LLC. This project is an independent community initiative and is not affiliated with, sponsored by, or endorsed by Meshtastic LLC.
<div align=center><img src="splash-screenshot.png" width=512 height=256></img></div>

⚠️ The documentation is still Work in progress.

## The name
This firmware was originally the Nexus firmware, in honor of one of the nodes it was planned for. The community came up with the quite fitting nickname of Jentastic so it stuck. That's why it has both names.

## License
The Nexus module as well as the accompanying scripts are licensed under the GNU **Affero** General Public License v3; upstream code and any changes I made to it are licensed under the GNU General Public License v3.
AGPLv3-licensed files are tagged as such at the beginning of the files.
If you wanna read the reasoning for this, read [docs/why-the-license.md](docs/why-the-license.md).

## Features
A summary of the highlights — see [docs/nexus-commands.md](docs/nexus-commands.md) for the full command reference:
* **Remote CLI administration** — configure, diagnose and manage nodes over LoRa, including commands that upstream remote admin can't do (send messages/traceroutes from a node, read RSSI/freq error remotely).
* **Stateless firewall** — drop misbehaving traffic and manage ignores/favourites across large meshes.
* **Multicast commands** — manage many nodes at once, plus cronjobs and macros for automation.
* **Signal intelligence** — signal/packet statistics, ping bot, mesh diagnostics.
* **LR2021 multi Spreading Factor listen.**
* **Client Base role repeats on CLIENT window instead of the LATE window.**

## Documentation
* [docs/nexus-commands.md](docs/nexus-commands.md) — full command reference for the public build.
* [docs/manual.md](docs/manual.md) — the Nexus channel, master keys and how to compile your own firmware with them.

## Building
Builds the same way as the official Meshtastic firmware (PlatformIO — `pio run -e <your_env>` and flash as usual). For a keyed/personal build there is also `build-nexus.sh`, a wrapper that injects builder defaults and if you wanna use them, master keys — see [docs/manual.md](docs/manual.md).

## Why does this exist?

This firmware was born around a year ago during the deployments and testing of infrastructure nodes in the Spanish Meshtastic™ mesh, currently one of the largest meshes in the world. This involved many preset tests, including the now custom SFNarrow preset we use. And during these tests and deployments we noticed a few shortcomings of the upstream firmware, notably:
* Remote admin in 2.7 doesn't let you send messages with a node (for understandable reasons), or send a traceroute from a node.
  * Also notably you can't see RSSI or Frequency Error on traceroutes. And Freq Error is normally only available on serial; on mountain nodes, both are impossible
* When trying different LoRa settings you have to change a lot of nodes, basically every node you administer, and in a chain, over the normal remote management, this notably works really badly over multiple hops, especially if you are currently in a preset that doesn't work very well, while a CLI command can and does reach.
* If for any reason you disable the TX enabled on a node, you can't enable it over LoRa anymore.
But more importantly when the network got larger we noticed something else too. A mesh network doesn't get to grow large without misconfigured nodes spamming and bringing the network down, only solution is usually to add them to ignores, which is a throw the kid out with the bath water approach usually, but necessary. And with the upstream firmware, you don't get any confirmation of if it worked or not (and with 2.7 firmwares, if the node wasn't in the Node DB it would fail, which on a 2000+ node network is almost guaranteed to be the case) and more importantly, you don't have a way to see if the nodes are in ignores or not.
A similar thing happens with the favourites on nodes, important for zero hop routers, as well as for the case of remote admin, for whichever portable node you use to admin these nodes.

For these reasons, the CLI has become insanely useful: you can track signals, send messages with a router so other people can see what signal they receive from the router when the router doesn't hear them, throttle misbehaving traffic, or manage multiple deployed nodes in a more convenient way.

## What this isn't
This isn't a hard fork, or meant to be one. It is designed to be completely interchangeable with either the upstream official firmware or other custom firmwares, and not break compatibility in any way with either the mesh or client apps.
