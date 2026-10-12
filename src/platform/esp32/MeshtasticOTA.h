#ifndef MESHTASTICOTA_H
#define MESHTASTICOTA_H

#include "mesh-pb-constants.h"
#include <Arduino.h>
#ifdef ESP_PLATFORM
#include <esp_ota_ops.h>
#endif

#define METHOD_OTA_BLE 1
#define METHOD_OTA_WIFI 2

namespace MeshtasticOTA
{
void initialize();
bool isUpdated();
const esp_partition_t *getAppPartition();
bool getAppDesc(const esp_partition_t *part, esp_app_desc_t *app_desc);
bool checkOTACapability(const esp_app_desc_t *app_desc, uint8_t method);
void recoverConfig(meshtastic_Config_NetworkConfig *network);
void saveConfig(meshtastic_Config_NetworkConfig *network, meshtastic_OTAMode method, uint8_t *ota_hash);
bool trySwitchToOTA();
const char *getVersion();
// v267: formatted info dump for /nexus WIFIOTA. Writes a multi-line string
// describing the OTA loader installed at the ota_1 partition (or its absence).
// Recommended bufSize >= 384 bytes to fit a full dump.
void getInfo(char *buf, size_t bufSize);
} // namespace MeshtasticOTA

#endif // MESHTASTICOTA_H
