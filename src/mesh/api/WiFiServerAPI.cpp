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
#ifdef ARCH_PORTDUINO
        auto ip = openAPI->getClient().remoteIP();
        char endpoint[24];
        snprintf(endpoint, sizeof(endpoint), "%u.%u.%u.%u:%u", static_cast<unsigned>(ip[0]), static_cast<unsigned>(ip[1]),
                 static_cast<unsigned>(ip[2]), static_cast<unsigned>(ip[3]),
                 static_cast<unsigned>(openAPI->getClient().remotePort()));
        return String(endpoint);
#else
        return openAPI->getClient().remoteIP().toString() + ":" + String(openAPI->getClient().remotePort());
#endif
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