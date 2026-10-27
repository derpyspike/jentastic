// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "RNodeTcpServer.h"

#if NEXUS_ENABLE_RNODE_TCP && HAS_WIFI

#include "RNodeEngine.h"
#include <Arduino.h>

RNodeTcpServer *rnodeTcpServer = nullptr;
static WiFiClient activeTcpClient;

RNodeTcpServer::RNodeTcpServer() : concurrency::OSThread("RNodeTcp") {}

void RNodeTcpServer::init()
{
    rnodeEngine.registerTcpWriteCallback(&RNodeTcpServer::sendData);
    serverStarted = false;
}

void RNodeTcpServer::sendData(const uint8_t *data, size_t len)
{
    if (activeTcpClient && activeTcpClient.connected() && len > 0) {
        activeTcpClient.write(data, len);
        activeTcpClient.flush();
    }
}

int32_t RNodeTcpServer::runOnce()
{
    if (!rnodeEngine.isTcpEnabled() || rnodeEngine.getMode() == RNODE_MODE_OFF) {
        if (client && client.connected()) {
            client.stop();
            activeTcpClient.stop();
            rnodeEngine.setTcpConnected(false);
        }
        if (serverStarted) {
            server.end();
            serverStarted = false;
        }
        return 1000;
    }

    bool wifiActive = (WiFi.status() == WL_CONNECTED) || ((WiFi.getMode() & WIFI_MODE_AP) != 0);
    if (!wifiActive) {
        if (client && client.connected()) {
            client.stop();
            activeTcpClient.stop();
            rnodeEngine.setTcpConnected(false);
        }
        return 1000;
    }

    if (!serverStarted) {
        server.begin(RNODE_DEFAULT_TCP_PORT);
        server.setNoDelay(true);
        serverStarted = true;
        LOG_INFO("RNode TCP server listening on port %d", RNODE_DEFAULT_TCP_PORT);
    }

    if (server.hasClient()) {
        WiFiClient newClient = server.available();
        if (client && client.connected()) {
            client.stop();
        }
        client = newClient;
        client.setNoDelay(true);
        activeTcpClient = client;
        rnodeEngine.setTcpConnected(true);
        LOG_INFO("RNode TCP: client connected");
    }

    if (client) {
        if (client.connected()) {
            while (client.available()) {
                uint8_t buf[128];
                int n = client.read(buf, sizeof(buf));
                if (n > 0) {
                    rnodeEngine.feedBytes(buf, n, RNODE_CLIENT_TCP);
                }
            }
        } else {
            client.stop();
            activeTcpClient.stop();
            rnodeEngine.setTcpConnected(false);
            LOG_INFO("RNode TCP: client disconnected");
        }
    }

    return 20; // Service every 20ms
}

#endif // NEXUS_ENABLE_RNODE_TCP && HAS_WIFI
