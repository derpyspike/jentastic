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
