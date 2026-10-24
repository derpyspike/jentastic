// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#pragma once

#include "configuration.h"

#if MESHTASTIC_ENABLE_RNODE_TCP && HAS_WIFI

#include "concurrency/OSThread.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

class RNodeTcpServer : public concurrency::OSThread
{
  public:
    RNodeTcpServer();
    virtual ~RNodeTcpServer() = default;

    void init();
    static void sendData(const uint8_t *data, size_t len);

  protected:
    virtual int32_t runOnce() override;

  private:
    WiFiServer server{RNODE_DEFAULT_TCP_PORT};
    WiFiClient client;
    bool serverStarted = false;
};

extern RNodeTcpServer *rnodeTcpServer;

#endif // MESHTASTIC_ENABLE_RNODE_TCP && HAS_WIFI
