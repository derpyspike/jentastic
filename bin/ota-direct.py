#!/usr/bin/env python3
#
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Jennifer Sanchez
#
# Licensed under the GNU Affero General Public License v3.
# See LICENSE-AGPL for the full license text.
#
"""Complete an ESP32 WiFi OTA update on a node already in OTA mode.

`meshtastic --ota-update` first asks the running app to reboot into OTA mode
through the admin API on port 4403. If an update was interrupted, the node is
already sitting in OTA mode (port 3232 open, 4403 closed), so the CLI can no
longer send that request. This helper speaks the port-3232 protocol directly
and streams the image, so an interrupted update can be finished.

Protocol (matches meshtastic.ota.ESP32WiFiOTA):
    OTA <size> <sha256-hex>\\n
    <OK | ERASING>
    <raw firmware bytes>
    <OK>

Exit codes: 0 success, 1 OTA failure, 2 not in OTA mode / unreachable.

Usage:
    ota-direct.py --check <host>
    ota-direct.py <host> <firmware.bin>
"""

import argparse
import hashlib
import socket
import sys

OTA_PORT = 3232
CHUNK_SIZE = 1024


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as firmware:
        for block in iter(lambda: firmware.read(65536), b""):
            digest.update(block)
    return digest


def read_line(sock):
    line = bytearray()
    while not line.endswith(b"\n"):
        char = sock.recv(1)
        if not char:
            raise ConnectionError("connection closed while waiting for a response")
        line += char
    return line.decode("utf-8", "replace").strip()


def is_in_ota_mode(host, port=OTA_PORT, timeout=2.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def update(host, firmware_path, port=OTA_PORT, timeout=10.0, verify_timeout=180.0):
    with open(firmware_path, "rb") as firmware:
        data = firmware.read()
    size = len(data)
    digest = file_sha256(firmware_path).hexdigest()

    print(f"Direct OTA: {firmware_path} ({size} bytes, sha256 {digest})", flush=True)
    sock = socket.create_connection((host, port), timeout=timeout)
    sock.settimeout(verify_timeout)
    try:
        sock.sendall(f"OTA {size} {digest}\n".encode("ascii"))

        while True:
            response = read_line(sock)
            if response == "OK":
                break
            if response == "ERASING":
                print("Device is erasing flash...", flush=True)
            elif response.startswith("ERR "):
                print(f"Device reported error: {response}", file=sys.stderr, flush=True)
                return 1
            elif response:
                print(f"Device: {response}", flush=True)

        sent = 0
        next_percent = 0
        while sent < size:
            chunk = data[sent:sent + CHUNK_SIZE]
            sock.sendall(chunk)
            sent += len(chunk)
            percent = sent * 100 // size
            if percent >= next_percent:
                end = "\n" if percent >= 100 else "\r"
                print(f"[{percent:3d}%] {sent}/{size} bytes", end=end, flush=True)
                next_percent = percent + 1

        while True:
            response = read_line(sock)
            if response == "OK":
                print("Direct OTA completed successfully.", flush=True)
                return 0
            if response.startswith("ERR "):
                print(f"OTA failed: {response}", file=sys.stderr, flush=True)
                return 1
            if response == "ACK":
                continue
            if response:
                print(f"Device: {response}", flush=True)
    except (OSError, ConnectionError) as error:
        print(f"Direct OTA error: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(description="Complete an ESP32 WiFi OTA already in OTA mode.")
    parser.add_argument("host", help="node IP address")
    parser.add_argument("firmware", nargs="?", help="firmware .bin to send")
    parser.add_argument("--check", action="store_true", help="only test whether port 3232 is open")
    parser.add_argument("--port", type=int, default=OTA_PORT, help=f"OTA port (default {OTA_PORT})")
    parser.add_argument("--timeout", type=float, default=10.0, help="connect timeout in seconds")
    parser.add_argument("--verify-timeout", type=float, default=180.0, help="read/verify timeout in seconds")
    args = parser.parse_args()

    if args.check:
        if is_in_ota_mode(args.host, args.port):
            print("ota-mode")
            return 0
        print("not-ota")
        return 2

    if not args.firmware:
        parser.error("firmware is required unless --check is used")
    if not is_in_ota_mode(args.host, args.port):
        print(f"{args.host} is not listening on port {args.port} (not in OTA mode)", file=sys.stderr)
        return 2
    return update(args.host, args.firmware, args.port, args.timeout, args.verify_timeout)


if __name__ == "__main__":
    sys.exit(main())
