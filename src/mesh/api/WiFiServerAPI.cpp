#include "configuration.h"
#include <Arduino.h>

#if HAS_WIFI
#include "WiFiServerAPI.h"

static std::unique_ptr<WiFiServerPort> apiPort;

void initApiServer(int port)
{
    // Start API server on port 4403
    if (!apiPort) {
        apiPort = std::make_unique<WiFiServerPort>(port);
        LOG_INFO("API server listen on TCP port %d", port);
        apiPort->init();
    }
}
void deInitApiServer()
{
    apiPort.reset();
}

WiFiServerAPI::WiFiServerAPI(WiFiClient &_client) : ServerAPI(_client)
{
    api_type = TYPE_WIFI;
    LOG_INFO("Incoming wifi connection");
}

WiFiServerPort::WiFiServerPort(int port) : APIServerPort(port) {}

bool WiFiServerPort::hasActiveClient() {
    return openAPI != nullptr;
}

String WiFiServerPort::getClientInfo() {
    if (openAPI) {
        return openAPI->getClient().remoteIP().toString() + ":" + String(openAPI->getClient().remotePort());
    }
    return "N/A";
}

bool hasWiFiClient() {
    return apiPort && apiPort->hasActiveClient();
}

String getWiFiClientInfo() {
    if (apiPort) return apiPort->getClientInfo();
    return "N/A";
}
#endif