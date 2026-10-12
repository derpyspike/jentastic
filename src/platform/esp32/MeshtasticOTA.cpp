// First, in its own block so the include sorter keeps it there: configuration.h supplies the
// variant defines mesh-pb-constants.h needs (portduino resolves MAX_NUM_NODES at runtime).
#include "configuration.h"

#include "MeshtasticOTA.h"
#ifdef ESP_PLATFORM
#include <Preferences.h>
#include <esp_ota_ops.h>
#endif

namespace MeshtasticOTA
{

static const char *nvsNamespace = "MeshtasticOTA";
static const char *combinedAppProjectName = "MeshtasticOTA";
static const char *bleOnlyAppProjectName = "MeshtasticOTA-BLE";
static const char *wifiOnlyAppProjectName = "MeshtasticOTA-WiFi";

static bool updated = false;

bool isUpdated()
{
    return updated;
}

void initialize()
{
    Preferences prefs;
    prefs.begin(nvsNamespace);
    if (prefs.getBool("updated")) {
        LOG_INFO("First boot after OTA update");
        updated = true;
        prefs.putBool("updated", false);
    }
    prefs.end();
}

void recoverConfig(meshtastic_Config_NetworkConfig *network)
{
    LOG_INFO("Recovering WiFi settings after OTA update");

    Preferences prefs;
    prefs.begin(nvsNamespace, true);
    String ssid = prefs.getString("ssid");
    String psk = prefs.getString("psk");
    prefs.end();

    network->wifi_enabled = true;
    strncpy(network->wifi_ssid, ssid.c_str(), sizeof(network->wifi_ssid));
    strncpy(network->wifi_psk, psk.c_str(), sizeof(network->wifi_psk));
}

void saveConfig(meshtastic_Config_NetworkConfig *network, meshtastic_OTAMode method, uint8_t *ota_hash)
{
    LOG_INFO("Saving WiFi settings for upcoming OTA update");

    Preferences prefs;
    prefs.begin(nvsNamespace);
    prefs.putUChar("method", method);
    prefs.putBytes("ota_hash", ota_hash, 32);
    prefs.putString("ssid", network->wifi_ssid);
    prefs.putString("psk", network->wifi_psk);
    prefs.putBool("updated", false);
    prefs.end();
}

const esp_partition_t *getAppPartition()
{
    return esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
}

bool getAppDesc(const esp_partition_t *part, esp_app_desc_t *app_desc)
{
    if (esp_ota_get_partition_description(part, app_desc) != ESP_OK) {
        LOG_INFO("esp_ota_get_partition_description failed");
        return false;
    }
    return true;
}

bool checkOTACapability(const esp_app_desc_t *app_desc, uint8_t method)
{
    // Combined loader supports all (both) transports, BLE and WiFi
    if (strcmp(app_desc->project_name, combinedAppProjectName) == 0) {
        LOG_INFO("OTA partition contains combined BLE/WiFi OTA Loader");
        return true;
    }
    if (method == METHOD_OTA_BLE && strcmp(app_desc->project_name, bleOnlyAppProjectName) == 0) {
        LOG_INFO("OTA partition contains BLE-only OTA Loader");
        return true;
    }
    if (method == METHOD_OTA_WIFI && strcmp(app_desc->project_name, wifiOnlyAppProjectName) == 0) {
        LOG_INFO("OTA partition contains WiFi-only OTA Loader");
        return true;
    }
    LOG_INFO("OTA partition does not contain a known OTA loader");
    return false;
}

bool trySwitchToOTA()
{
    const esp_partition_t *part = getAppPartition();

    if (part == NULL) {
        LOG_WARN("Can't get app partition in preparation of OTA reboot");
        return false;
    }

    uint8_t result = esp_ota_set_boot_partition(part);
    // Partition and app checks should now be done in the AdminModule before this is called
    if (result != ESP_OK) {
        LOG_WARN("Can't switch to OTA partition (reason %d)", result);
        return false;
    }

    return true;
}

const char *getVersion()
{
    const esp_partition_t *part = getAppPartition();
    static esp_app_desc_t app_desc;
    if (!getAppDesc(part, &app_desc))
        return "";
    return app_desc.version;
}

void getInfo(char *buf, size_t bufSize)
{
    if (!buf || bufSize == 0) return;

    const esp_partition_t *part = getAppPartition();
    if (part == NULL) {
        // Body only: the "JT WiFiOTA Info:" title is supplied by startPaging()
        // at the call site in NexusModule.cpp. Don't repeat it here. See
        // Commit 42.
        snprintf(buf, bufSize,
            "  Available: no\n"
            "  No OTA partition found at ota_1.");
        return;
    }

    esp_app_desc_t app_desc;
    if (!getAppDesc(part, &app_desc)) {
        snprintf(buf, bufSize,
            "  Partition: 0x%lx (size %lu)\n"
            "  Available: error\n"
            "  OTA partition present but app descriptor unreadable\n"
            "  (corrupt? wrong firmware type?)",
            (unsigned long)part->address, (unsigned long)part->size);
        return;
    }

    // Sanitize printable fields (project_name/time/idf_ver are char arrays; null may not be set).
    char proj[33]; strncpy(proj, app_desc.project_name, sizeof(proj) - 1); proj[sizeof(proj) - 1] = '\0';
    char ver[33];  strncpy(ver,  app_desc.version,     sizeof(ver)  - 1); ver[sizeof(ver)  - 1] = '\0';
    char when[17]; strncpy(when, app_desc.time,        sizeof(when) - 1); when[sizeof(when) - 1] = '\0';
    char idf[33];  strncpy(idf,  app_desc.idf_ver,     sizeof(idf)  - 1); idf[sizeof(idf)   - 1] = '\0';

    // First 16 bytes of SHA256 as hex (32 chars). Enough to identify a unique build.
    char sha[33];
    for (int i = 0; i < 16; i++) {
        snprintf(sha + i * 2, sizeof(sha) - i * 2, "%02x", app_desc.app_elf_sha256[i]);
    }

    // Identify the loader variant for the operator's benefit.
    const char *kind = "Unknown";
    if (strcmp(proj, combinedAppProjectName) == 0)      kind = "Combined BLE+WiFi (since fw 2.7.18)";
    else if (strcmp(proj, bleOnlyAppProjectName) == 0)  kind = "BLE-only";
    else if (strcmp(proj, wifiOnlyAppProjectName) == 0) kind = "WiFi-only";

    snprintf(buf, bufSize,
        "  Partition: 0x%lx (size %lu)\n"
        "  Available: yes\n"
        "  Kind:      %s\n"
        "  Project:   %s\n"
        "  Version:   %s\n"
        "  Built:     %s\n"
        "  IDF:       %s\n"
        "  SHA256:    %s... (first 16/32 bytes)",
        (unsigned long)part->address, (unsigned long)part->size,
        kind, proj, ver, when, idf, sha);
}

} // namespace MeshtasticOTA
