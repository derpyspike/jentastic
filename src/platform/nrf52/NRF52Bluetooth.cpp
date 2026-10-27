#include "NRF52Bluetooth.h"
#include "BLEDfuSecure.h"
#include "BluetoothCommon.h"
#include "HardwareRNG.h"
#include "PowerFSM.h"
#include "SPILock.h"
#include "configuration.h"
#include "error.h"
#include "main.h"
#include "mesh/PhoneAPI.h"
#include "mesh/Throttle.h"
#include "mesh/mesh-pb-constants.h"
#include <InternalFileSystem.h>
#include <bluefruit.h>
#include <utility/bonding.h>

#ifdef NEXUS_ENABLE_AIRTAG
extern bool nexus_airtag_enabled;
extern uint8_t nexus_airtag_key[28];
static bool inAirtagMode = false;
static uint32_t lastModeSwitch = 0;
#endif

static BLEService meshBleService = BLEService(BLEUuid(MESH_SERVICE_UUID_16));
static BLECharacteristic fromNum = BLECharacteristic(BLEUuid(FROMNUM_UUID_16));
static BLECharacteristic fromRadio = BLECharacteristic(BLEUuid(FROMRADIO_UUID_16));
static BLECharacteristic toRadio = BLECharacteristic(BLEUuid(TORADIO_UUID_16));
static BLECharacteristic logRadio = BLECharacteristic(BLEUuid(LOGRADIO_UUID_16));

static BLEDis bledis;             // DIS (Device Information Service) helper class instance
static BLEBas blebas;             // BAS (Battery Service) helper class instance
static int lastBatteryLevel = -1; // last value written to BAS, to skip redundant writes/notifies

#if NEXUS_ENABLE_RNODE_BLE
#include "mesh/RNodeEngine.h"
#include <services/BLEUart.h>
static BLEUart rnodeBleUart;
static uint16_t rnodeConnHandle = BLE_CONN_HANDLE_INVALID;

/// RNode clients (Columba) only list scan results whose name starts "RNode "; the Meshtastic app only
/// lists bonded devices whose name ends "_XXXX". "RNode _XXXX" satisfies both, and at 11 chars still fits
/// next to the 128-bit NUS UUID in the 31-byte scan response.
static void makeRNodeBleName(char *out, size_t cap)
{
    uint8_t dmac[6] = {0};
    Bluefruit.getAddr(dmac);
    snprintf(out, cap, "RNode _%02X%02X", dmac[4], dmac[5]);
}

static void rnode_nrf52_ble_write(const uint8_t *data, size_t len)
{
    const uint16_t target = rnodeConnHandle != BLE_CONN_HANDLE_INVALID ? rnodeConnHandle : Bluefruit.connHandle();
    if (!data || len == 0 || !rnodeBleUart.notifyEnabled(target))
        return;
    BLEConnection *conn = Bluefruit.Connection(target);
    if (!conn)
        return;
    const uint16_t mtu = conn->getMtu();
    const size_t chunk = mtu > 3 ? (size_t)(mtu - 3) : 20;
    for (size_t offset = 0; offset < len;) {
        const size_t n = (len - offset > chunk) ? chunk : (len - offset);
        // Address the connection explicitly: with the Meshtastic phone and Columba connected at the
        // same time, BLEUart's default uses Bluefruit.connHandle() (whichever connected last) and
        // would notify the wrong link.
        const size_t written = rnodeBleUart.write(target, data + offset, n);
        if (written == 0)
            break;
        offset += written;
    }
}
#endif
#ifndef BLE_DFU_SECURE
namespace
{
// The library's DFU handler jumps to the bootloader from the callback task with no flash quiesce, so
// wrap it: wait out any write in flight first, and unlock again if it comes back without jumping.
class QuiescingBLEDfu : public BLEDfu
{
    struct Peek : BLECharacteristic {
        using BLECharacteristic::_wr_authorize_cb;
    };
    static BLECharacteristic::write_authorize_cb_t libraryCb;

    static void onControlWrite(uint16_t conn_hdl, BLECharacteristic *chr, ble_gatts_evt_write_t *request)
    {
        // Only START_DFU resets, and the library reads this byte without checking len, so match it exactly.
        if (request->data[0] != 1) {
            libraryCb(conn_hdl, chr, request);
            return;
        }
        nrf52FlashQuiesce();
        // The handler reloads the bond keys through LittleFS, so it cannot run under the FS mutex; spiLock still
        // fences every other writer until the jump.
        InternalFS._unlockFS();
        libraryCb(conn_hdl, chr, request);
        spiLock->unlock();
    }

  public:
    err_t begin() override
    {
        err_t err = BLEDfu::begin();
        libraryCb = _chr_control.*(&Peek::_wr_authorize_cb);
        _chr_control.setWriteAuthorizeCallback(onControlWrite);
        return err;
    }
};
BLECharacteristic::write_authorize_cb_t QuiescingBLEDfu::libraryCb;
} // namespace
static QuiescingBLEDfu bledfu; // DFU software update helper service
#else
static BLEDfuSecure bledfusecure; // DFU software update helper service
#endif

// This scratch buffer is used for various bluetooth reads/writes - but it is safe because only one bt operation can be in
// process at once
// static uint8_t trBytes[_max(_max(_max(_max(ToRadio_size, RadioConfig_size), User_size), MyNodeInfo_size), FromRadio_size)];
static uint8_t fromRadioBytes[meshtastic_FromRadio_size];
static uint8_t toRadioBytes[meshtastic_ToRadio_size];

// Last ToRadio value received from the phone
static uint8_t lastToRadio[MAX_TO_FROM_RADIO_SIZE];

static uint16_t connectionHandle = BLE_CONN_HANDLE_INVALID;

static bool passkeyShowing;

class BluetoothPhoneAPI : public PhoneAPI
{
    /**
     * Subclasses can use this as a hook to provide custom notifications for their transport (i.e. bluetooth notifies)
     */
    virtual void onNowHasData(uint32_t fromRadioNum) override
    {
        PhoneAPI::onNowHasData(fromRadioNum);

        LOG_INFO("BLE notify fromNum");
        fromNum.notify32(fromRadioNum);
    }

    /// Check the current underlying physical link to see if the client is currently connected
    virtual bool checkIsConnected() override { return Bluefruit.connected(connectionHandle); }

  public:
    BluetoothPhoneAPI() { api_type = TYPE_BLE; }
};

static BluetoothPhoneAPI *bluetoothPhoneAPI;

/// A connection only becomes "the phone" when it actually speaks the Meshtastic API; the RNode/NUS
/// client shares the same peripheral and must not make the phone API think it is connected.
static void markPhoneConnection(uint16_t conn_hdl)
{
    if (connectionHandle == conn_hdl)
        return;
    connectionHandle = conn_hdl;
    // A new physical link must start unauthenticated (single admin auth slot).
#ifdef MESHTASTIC_PHONEAPI_ACCESS_CONTROL
    if (bluetoothPhoneAPI) {
        bluetoothPhoneAPI->setAdminAuthorized(false);
    }
#endif
    meshtastic::BluetoothStatus newStatus(meshtastic::BluetoothStatus::ConnectionState::CONNECTED);
    bluetoothStatus->updateStatus(&newStatus);
}

void onConnect(uint16_t conn_handle)
{
    // Get the reference to current connection
    BLEConnection *connection = Bluefruit.Connection(conn_handle);
    char central_name[32] = {0};
    connection->getPeerName(central_name, sizeof(central_name));
    LOG_INFO("BLE Connected to %s (handle=%u)", central_name, (unsigned)conn_handle);

    // A connection stops advertising by default. Keep it running while a second slot is free so the
    // Meshtastic phone and the RNode/NUS client can be connected at the same time.
    if (Bluefruit.Periph.connected() < 2)
        Bluefruit.Advertising.start(0);

}
/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void onDisconnect(uint16_t conn_handle, uint8_t reason)
{
    LOG_INFO("BLE Disconnected, reason = 0x%x (handle=%u)", reason, (unsigned)conn_handle);
#if NEXUS_ENABLE_RNODE_BLE
    if (conn_handle == rnodeConnHandle) {
        rnodeConnHandle = BLE_CONN_HANDLE_INVALID;
        rnodeEngine.setBleConnected(false);
    }
#endif
    if (conn_handle == connectionHandle) {
        connectionHandle = BLE_CONN_HANDLE_INVALID;
        if (bluetoothPhoneAPI) {
            bluetoothPhoneAPI->close();
        }

        // Clear the last ToRadio packet buffer to avoid rejecting first packet from new connection
        memset(lastToRadio, 0, sizeof(lastToRadio));

        // Notify UI (or any other interested firmware components)
        meshtastic::BluetoothStatus newStatus(meshtastic::BluetoothStatus::ConnectionState::DISCONNECTED);
        bluetoothStatus->updateStatus(&newStatus);
    }

    // Free a slot again: let the other persona (re)connect without waiting for a full advertising cycle.
    if (Bluefruit.Periph.connected() < 2)
        Bluefruit.Advertising.start(0);

#if HAS_SCREEN
    // If a pairing prompt is active, make sure we dismiss it on disconnect/cancel/failure paths.
    if (passkeyShowing) {
        passkeyShowing = false;
        if (screen) {
            screen->endAlert();
        }
    }
#endif
}
void onCccd(uint16_t conn_hdl, BLECharacteristic *chr, uint16_t cccd_value)
{
    // Display the raw request packet
    LOG_INFO("CCCD Updated: %u", cccd_value);
    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.

    // According to the GATT spec: cccd value = 0x0001 means notifications are enabled
    // and cccd value = 0x0002 means indications are enabled

    if (chr->uuid == fromNum.uuid || chr->uuid == logRadio.uuid) {
        markPhoneConnection(conn_hdl);
        auto result = cccd_value == 2 ? chr->indicateEnabled(conn_hdl) : chr->notifyEnabled(conn_hdl);
        if (result) {
            LOG_INFO("Notify/Indicate enabled");
        } else {
            LOG_INFO("Notify/Indicate disabled");
        }
    }
}
void startAdv(void)
{
#ifdef NEXUS_ENABLE_AIRTAG
    if (nexus_airtag_enabled && inAirtagMode) {
        LOG_INFO("Nexus: Starting AirTag Advertisement");
        Bluefruit.Advertising.stop();
        Bluefruit.Advertising.clearData();
        Bluefruit.ScanResponse.clearData();
        
        // Apple AirTag Advertisement Payload (Type 0xFF Manufacturer Data)
        // [0..1] Company ID: 0x004C (Apple)
        // [2] Offline Finding Type: 0x12
        // [3] Offline Finding Length: 0x19 (25 bytes)
        // [4] Status: 0x00
        // [5..26] Public Key (22 bytes)
        // [27] Status Flag: 0x00
        // [28] Hint: 0x00
        
        uint8_t adData[31];
        adData[0] = 0x4C; adData[1] = 0x00; // Apple
        adData[2] = 0x12;                  // Type
        adData[3] = 0x19;                  // Len
        adData[4] = 0x00;                  // Status
        memcpy(&adData[5], nexus_airtag_key, 22); // Payload
        adData[27] = 0x00;                 // Flag
        adData[28] = 0x00;                 // Hint
        
        Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, adData, 29);
        Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
        
        Bluefruit.Advertising.setInterval(160, 160); // Fast interval for tracker reliability (100ms)
        Bluefruit.Advertising.restartOnDisconnect(true);
        Bluefruit.Advertising.start(0);
        return;
    }
#endif

    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    // IncludeService UUID
    // Bluefruit.ScanResponse.addService(meshBleService);
#if NEXUS_ENABLE_RNODE_BLE
    const bool rnodeBle = rnodeEngine.isEnabled() && rnodeEngine.isBleEnabled();
#else
    const bool rnodeBle = false;
#endif
    // RNode NUS is 128-bit. Advertising it next to the 128-bit Meshtastic service exceeds the
    // 31-byte legacy limit, so with RNode BLE on we split the payload: adv = flags + mesh UUID +
    // TxPower, scan response = "RNode xx" name + NUS UUID. That is 24 and 30 bytes respectively.
    if (rnodeBle)
        Bluefruit.Advertising.addTxPower();
    else
        Bluefruit.ScanResponse.addTxPower();
    Bluefruit.ScanResponse.addName();
    // Include Name
    // Bluefruit.Advertising.addName();
    Bluefruit.Advertising.addService(meshBleService);
#if NEXUS_ENABLE_RNODE_BLE
    if (rnodeBle)
        Bluefruit.ScanResponse.addService(rnodeBleUart);
#endif
    /* Start Advertising
     * - Enable auto advertising if disconnected
     * - Interval:  fast mode = 20 ms, slow mode = 417,5 ms
     * - Timeout for fast mode is 30 seconds
     * - Start(timeout) with timeout = 0 will advertise forever (until connected)
     *
     * For recommended advertising interval
     * https://developer.apple.com/library/content/qa/qa1931/_index.html
     */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 668); // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
    Bluefruit.Advertising.start(0); // 0 = Don't stop advertising after n seconds.  FIXME, we should stop advertising after X
}
// Just ack that the caller is allowed to read
static void authorizeRead(uint16_t conn_hdl)
{
    ble_gatts_rw_authorize_reply_params_t reply = {.type = BLE_GATTS_AUTHORIZE_TYPE_READ};
    reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
    sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);
}
/**
 * client is starting read, pull the bytes from our API class
 */
void onFromRadioAuthorize(uint16_t conn_hdl, BLECharacteristic *chr, ble_gatts_evt_read_t *request)
{
    markPhoneConnection(conn_hdl);
    if (request->offset == 0) {
        // If the read is long, we will get multiple authorize invocations - we only populate data on the first
        size_t numBytes = bluetoothPhoneAPI->getFromRadio(fromRadioBytes);
        // Someone is going to read our value as soon as this callback returns.  So fill it with the next message in the queue
        // or make empty if the queue is empty
        fromRadio.write(fromRadioBytes, numBytes);
    } else {
        // LOG_INFO("Ignore successor read");
    }
    authorizeRead(conn_hdl);
}

void onToRadioWrite(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
    markPhoneConnection(conn_hdl);
    LOG_INFO("toRadioWriteCb data %p, len %u", data, len);
    if (memcmp(lastToRadio, data, len) != 0) {
        LOG_DEBUG("New ToRadio packet");
        memcpy(lastToRadio, data, len);
        bluetoothPhoneAPI->handleToRadio(data, len);
    } else {
        LOG_DEBUG("Drop dup ToRadio packet we just saw");
    }
}

void setupMeshService(void)
{
    bluetoothPhoneAPI = new BluetoothPhoneAPI();
    meshBleService.begin();
    // Note: You must call .begin() on the BLEService before calling .begin() on
    // any characteristic(s) within that service definition.. Calling .begin() on
    // a BLECharacteristic will cause it to be added to the last BLEService that
    // was 'begin()'ed!
    auto secMode =
        config.bluetooth.mode == meshtastic_Config_BluetoothConfig_PairingMode_NO_PIN ? SECMODE_OPEN : SECMODE_ENC_NO_MITM;
    fromNum.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    fromNum.setPermission(secMode, SECMODE_NO_ACCESS); // FIXME, secure this!!!
    fromNum.setFixedLen(
        0); // Variable len (either 0 or 4)  FIXME consider changing protocol so it is fixed 4 byte len, where 0 means empty
    fromNum.setMaxLen(4);
    fromNum.setCccdWriteCallback(onCccd); // Optionally capture CCCD updates
    // We don't yet need to hook the fromNum auth callback
    // fromNum.setReadAuthorizeCallback(fromNumAuthorizeCb);
    fromNum.write32(0); // Provide default fromNum of 0
    fromNum.begin();

    fromRadio.setProperties(CHR_PROPS_READ);
    fromRadio.setPermission(secMode, SECMODE_NO_ACCESS);
    fromRadio.setMaxLen(sizeof(fromRadioBytes));
    fromRadio.setReadAuthorizeCallback(
        onFromRadioAuthorize,
        false); // We don't call this callback via the adafruit queue, because we can safely run in the BLE context
    fromRadio.setBuffer(fromRadioBytes, sizeof(fromRadioBytes)); // we preallocate our fromradio buffer so we won't waste space
    // for two copies
    fromRadio.begin();

    toRadio.setProperties(CHR_PROPS_WRITE);
    toRadio.setPermission(secMode, secMode); // FIXME secure this!
    toRadio.setFixedLen(0);
    toRadio.setMaxLen(512);
    toRadio.setBuffer(toRadioBytes, sizeof(toRadioBytes));
    // We don't call this callback via the adafruit queue, because we can safely run in the BLE context
    toRadio.setWriteCallback(onToRadioWrite, false);
    toRadio.begin();

    logRadio.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    logRadio.setPermission(secMode, SECMODE_NO_ACCESS);
    logRadio.setMaxLen(512);
    logRadio.setCccdWriteCallback(onCccd);
    logRadio.write32(0);
    logRadio.begin();

#if NEXUS_ENABLE_RNODE_BLE
    if (rnodeEngine.isBleEnabled()) {
        rnodeBleUart.begin();
        rnodeBleUart.setNotifyCallback([](uint16_t conn_hdl, bool enabled) {
            if (enabled) {
                rnodeConnHandle = conn_hdl;
                rnodeEngine.setBleConnected(true);
                LOG_INFO("RNode BLE client subscribed (handle=%u)", (unsigned)conn_hdl);
            } else if (conn_hdl == rnodeConnHandle) {
                rnodeConnHandle = BLE_CONN_HANDLE_INVALID;
                rnodeEngine.setBleConnected(false);
                LOG_INFO("RNode BLE client unsubscribed (handle=%u)", (unsigned)conn_hdl);
            }
        });
        rnodeBleUart.setRxCallback([](uint16_t conn_hdl) {
            rnodeConnHandle = conn_hdl;
            rnodeEngine.setBleConnected(true);
            while (rnodeBleUart.available()) {
                uint8_t buf[64];
                int n = rnodeBleUart.read(buf, sizeof(buf));
                if (n > 0) {
                    rnodeEngine.feedBytes(buf, n, RNODE_CLIENT_BLE);
                }
            }
        });
        rnodeEngine.registerBleWriteCallback(rnode_nrf52_ble_write);
    }
#endif
}
static uint32_t configuredPasskey;
void NRF52Bluetooth::shutdown()
{
    // Shutdown bluetooth for minimum power draw
    LOG_INFO("Disable NRF52 bluetooth");
    Bluefruit.Security.setPairPasskeyCallback(NRF52Bluetooth::onUnwantedPairing); // Actively refuse (during factory reset)

    // Clear the auto-restart flag before dropping the link: our DISCONNECTED event is only processed
    // after this callback returns and would re-start advertising. startAdv()/resumeAdvertising() re-set it.
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.stop();
    disconnect();
}
void NRF52Bluetooth::startDisabled()
{
    // Setup Bluetooth
    nrf52Bluetooth->setup();
    // Shutdown bluetooth for minimum power draw
    Bluefruit.Advertising.stop();
    Bluefruit.setTxPower(-40); // Minimum power
    LOG_INFO("Disable NRF52 BT (tx power min, advertise stopped)");
}
bool NRF52Bluetooth::isConnected()
{
    return Bluefruit.connected(connectionHandle);
}
int NRF52Bluetooth::getRssi()
{
    return 0; // FIXME figure out where to source this
}

// Valid BLE TX power levels as per nRF52840 Product Specification are: "-20 to +8 dBm TX power, configurable in 4 dB steps".
// See https://docs.nordicsemi.com/bundle/ps_nrf52840/page/keyfeatures_html5.html
#define VALID_BLE_TX_POWER(x)                                                                                                    \
    ((x) == -20 || (x) == -16 || (x) == -12 || (x) == -8 || (x) == -4 || (x) == 0 || (x) == 4 || (x) == 8)

void NRF52Bluetooth::setup()
{
    // Initialise the Bluefruit module
    LOG_INFO("Init the Bluefruit nRF52 module");
    Bluefruit.autoConnLed(false);
    // Two peripheral slots so the Meshtastic app and Columba can be connected at once. HIGH (MTU 128,
    // short event) rather than MAX, because the MAX event length/queues do not fit two links in the
    // SoftDevice's RAM budget. Never call begin() twice: the SoftDevice is already enabled after the
    // first attempt, and a second call can wedge the stack into a reset loop.
    Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);
    if (!Bluefruit.begin(2, 0)) {
        // sd_ble_enable() rejected our RAM base: the linker RAM ORIGIN
        // (src/platform/nrf52/nrf52840_s140_v*.ld) is below what the SoftDevice needs for the
        // current Bluefruit config. Rebuild with -DCFG_DEBUG=1 to get "SoftDevice's RAM requires:
        // 0x..." in the log, then raise the ORIGIN accordingly.
        LOG_ERROR("Bluefruit.begin(2) failed: SoftDevice RAM too small");
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_UNSPECIFIED);
        return;
    }
    // Clear existing data.
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();
#if defined(NRF52_BLE_TX_POWER) && VALID_BLE_TX_POWER(NRF52_BLE_TX_POWER)
    Bluefruit.setTxPower(NRF52_BLE_TX_POWER);
#endif
    if (config.bluetooth.mode != meshtastic_Config_BluetoothConfig_PairingMode_NO_PIN) {
        if (config.bluetooth.mode == meshtastic_Config_BluetoothConfig_PairingMode_FIXED_PIN) {
            configuredPasskey = config.bluetooth.fixed_pin;
        } else {
            uint32_t hwrand = 0;
            HardwareRNG::fill(reinterpret_cast<uint8_t *>(&hwrand), sizeof(hwrand));
            configuredPasskey = hwrand % 900000u + 100000u;
        }
        auto pinString = std::to_string(configuredPasskey);
        LOG_INFO("Bluetooth pin set to '%i'", configuredPasskey);
        Bluefruit.Security.setPIN(pinString.c_str());
        Bluefruit.Security.setIOCaps(true, false, false);
        Bluefruit.Security.setPairPasskeyCallback(NRF52Bluetooth::onPairingPasskey);
        Bluefruit.Security.setPairCompleteCallback(NRF52Bluetooth::onPairingCompleted);
        Bluefruit.Security.setSecuredCallback(NRF52Bluetooth::onConnectionSecured);
        meshBleService.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
    } else {
        Bluefruit.Security.setIOCaps(false, false, false);
        meshBleService.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    }
    // Set the advertised device name (keep it short!). RNode clients (Columba) filter scans on the
    // "RNode " prefix, so the NUS persona gets that name while Meshtastic apps keep finding us by
    // the mesh service UUID.
#if NEXUS_ENABLE_RNODE_BLE
    if (rnodeEngine.isEnabled() && rnodeEngine.isBleEnabled()) {
        char rnodeName[12];
        makeRNodeBleName(rnodeName, sizeof(rnodeName));
        Bluefruit.setName(rnodeName);
    } else
#endif
        Bluefruit.setName(getDeviceName());
    // Set the connect/disconnect callback handlers
    Bluefruit.Periph.setConnectCallback(onConnect);
    Bluefruit.Periph.setDisconnectCallback(onDisconnect);

    // Do not change Slave Latency to value other than 0 !!!
    // There is probably a bug in SoftDevice + certain Apple iOS versions being
    // brain damaged causing connectivity problems.

    // On one side it seems SoftDevice is using SlaveLatency value even
    // if connection parameter negotation failed and phone sees it as connectivity errors.

    // On the other hand Apple can randomly refuse any parameter negotiation and shutdown connection
    // even if you meet Apple Developer Guidelines for BLE devices. Because f* you, that's why.

    // While this API call sets preferred connection parameters (PPCP) - many phones ignore it (yeah) and it seems SoftDevice
    // will try to renegotiate connection parameters based on those values after phone connection.
    // So those are relatively safe values so Apple braindead firmware won't get angry and at least we may try
    // to negotiate some longer connection interval to save battery.

    // See https://github.com/meshtastic/firmware/pull/8858 for measurements.  We are dealing with microamp savings anyway so not
    // worth dying on a hill here.

    Bluefruit.Periph.setConnSlaveLatency(0);
    // 1.25 ms units - so min, max is 15, 100 ms range.
    Bluefruit.Periph.setConnInterval(12, 80);

#ifndef BLE_DFU_SECURE
    bledfu.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
    bledfu.begin(); // Install the DFU helper
#else
    bledfusecure.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM); // add by WayenWeng
    bledfusecure.begin();                                                     // Install the DFU helper
#endif
    // Configure and Start the Device Information Service
    LOG_INFO("Init the Device Information Service");
    bledis.setModel(optstr(HW_VERSION));
    bledis.setFirmwareRev(optstr(APP_VERSION));
    bledis.begin();
    // Start the BLE Battery Service and set it to 100%
    LOG_INFO("Init the Battery Service");
    blebas.begin();
    blebas.write(0); // Unknown battery level for now
    lastBatteryLevel = 0;
    // Setup the Heart Rate Monitor service using
    // BLEService and BLECharacteristic classes
    LOG_INFO("Init the Mesh bluetooth service");
    setupMeshService();
    // Setup the advertising packet(s)
    LOG_INFO("Set up the advertising payload(s)");
    startAdv();
    LOG_INFO("Advertise");
}
void NRF52Bluetooth::resumeAdvertising()
{
    // shutdown() latches onUnwantedPairing to actively refuse pairing (used on the factory-reset /
    // BT-disable teardown path). The real pairing passkey callback is installed only in setup(), but a
    // shutdown()->resumeAdvertising() re-enable cycle skips setup() entirely (see setBluetoothEnable() in
    // main-nrf52.cpp), so restore the correct callback here - otherwise the device silently refuses all
    // pairing until the next reboot. Mirror the mode check in setup(): only PIN modes drive a passkey-
    // display callback, so NO_PIN (Just Works) needs no restore.
    if (config.bluetooth.mode != meshtastic_Config_BluetoothConfig_PairingMode_NO_PIN)
        Bluefruit.Security.setPairPasskeyCallback(NRF52Bluetooth::onPairingPasskey);

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 668); // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
    Bluefruit.Advertising.start(0);
}
/// Given a level between 0-100, update the BLE attribute
void updateBatteryLevel(uint8_t level)
{
    if (!nrf52Bluetooth) // skip until the Battery Service has been begun in setup()
        return;

    if (level > 100) // BAS battery level must stay within 0-100
        level = 100;
    if (level == lastBatteryLevel)
        return;
    lastBatteryLevel = level;
    blebas.write(level);
}
void NRF52Bluetooth::clearBonds()
{
    LOG_INFO("Clear bluetooth bonds");
    bond_print_list(BLE_GAP_ROLE_PERIPH);
    bond_print_list(BLE_GAP_ROLE_CENTRAL);
    Bluefruit.Periph.clearBonds();
    Bluefruit.Central.clearBonds();
}
void NRF52Bluetooth::onConnectionSecured(uint16_t conn_handle)
{
    LOG_INFO("BLE connection secured");
}
bool NRF52Bluetooth::onPairingPasskey(uint16_t conn_handle, uint8_t const passkey[6], bool match_request)
{
    char passkey1[4] = {passkey[0], passkey[1], passkey[2], '\0'};
    char passkey2[4] = {passkey[3], passkey[4], passkey[5], '\0'};
    LOG_INFO("BLE pair process started with passkey %s %s", passkey1, passkey2);
    powerFSM.trigger(EVENT_BLUETOOTH_PAIR);

    // Get passkey as string
    // Note: possible leading zeros
    std::string textkey;
    for (uint8_t i = 0; i < 6; i++)
        textkey += (char)passkey[i];

    // Notify UI (or other components) of pairing event and passkey
    meshtastic::BluetoothStatus newStatus(textkey);
    bluetoothStatus->updateStatus(&newStatus);

#if HAS_SCREEN && !defined(MESHTASTIC_EXCLUDE_SCREEN)
    if (screen) {
        std::string configuredPasskeyText = std::to_string(configuredPasskey);
        std::string ble_message =
            "Bluetooth\nPIN\n[M]" + configuredPasskeyText.substr(0, 3) + " " + configuredPasskeyText.substr(3, 6);
        // Use the pairing_pin notification type so the lockdown UI short-
        // circuit (Screen.cpp updateUiFrame) allows the overlay through
        // even on a locked device - see H13 audit fix. The banner content
        // is the per-attempt ephemeral pair PIN, not operator content.
        graphics::BannerOverlayOptions opts;
        opts.message = ble_message.c_str();
        opts.durationMs = 30000;
        opts.notificationType = graphics::notificationTypeEnum::pairing_pin;
        screen->showOverlayBanner(opts);
    }
#endif
    passkeyShowing = true;

    if (match_request) {
        uint32_t start_time = millis();
        while (Throttle::isWithinTimespanMs(start_time, 30000)) {
            if (!Bluefruit.connected(conn_handle))
                break;
        }
    }
    LOG_INFO("BLE passkey pair: match_request=%i", match_request);
    return true;
}

// Actively refuse new BLE pairings
// After clearing bonds (at factory reset), clients seem initially able to attempt to re-pair, even with advertising disabled.
// On NRF52Bluetooth::shutdown, we change the pairing callback to this method, to aggressively refuse any connection attempts.
bool NRF52Bluetooth::onUnwantedPairing(uint16_t conn_handle, uint8_t const passkey[6], bool match_request)
{
    NRF52Bluetooth::disconnect();
    return false;
}

// Disconnect any BLE connections
void NRF52Bluetooth::disconnect()
{
    static constexpr uint32_t DISCONNECT_TIMEOUT_MSEC = 1000;
    uint8_t connection_num = Bluefruit.connected();
    if (connection_num) {
        // Close all connections. We're only expecting one.
        for (uint8_t i = 0; i < connection_num; i++)
            Bluefruit.disconnect(i);

        // Best-effort wait: on Bluefruit's BLE event task the DISCONNECTED event can't be processed
        // until this callback returns, so an unbounded wait would deadlock until the watchdog fires.
        uint32_t start = millis();
        while (Bluefruit.connected() && Throttle::isWithinTimespanMs(start, DISCONNECT_TIMEOUT_MSEC))
            delay(1);

        if (Bluefruit.connected())
            LOG_WARN("BLE disconnect unconfirmed after %ums, shutdown anyway", millis() - start);
        else
            LOG_INFO("Ended BLE connection");
    }
}

void NRF52Bluetooth::onPairingCompleted(uint16_t conn_handle, uint8_t auth_status)
{
    if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS) {
        LOG_INFO("BLE pair success");
        meshtastic::BluetoothStatus newConnectedStatus(meshtastic::BluetoothStatus::ConnectionState::CONNECTED);
        bluetoothStatus->updateStatus(&newConnectedStatus);
    } else {
        LOG_INFO("BLE pair failed, status 0x%02x", auth_status);
        // Notify UI (or any other interested firmware components)
        meshtastic::BluetoothStatus newDisconnectedStatus(meshtastic::BluetoothStatus::ConnectionState::DISCONNECTED);
        bluetoothStatus->updateStatus(&newDisconnectedStatus);
    }

    // Todo: migrate this display code back into Screen class, and observe bluetoothStatus
    passkeyShowing = false;
    if (screen) {
        screen->endAlert();
    }
}

void NRF52Bluetooth::sendLog(const uint8_t *logMessage, size_t length)
{
    if (!isConnected() || length > 512)
        return;
    if (logRadio.indicateEnabled())
        logRadio.indicate(logMessage, (uint16_t)length);
    else
        logRadio.notify(logMessage, (uint16_t)length);
}

void NRF52Bluetooth::tick()
{
#if NEXUS_ENABLE_RNODE_BLE
    // A connection stops advertising. Restarting inside onConnect() can be rejected while the
    // SoftDevice is still processing the connect event, so re-arm here once the event has settled.
    // That is what lets a second central (the Meshtastic phone and Columba) connect at the same time.
    if (rnodeEngine.isEnabled() && rnodeEngine.isBleEnabled() && Bluefruit.Periph.connected() < 2 &&
        !Bluefruit.Advertising.isRunning()) {
        static uint32_t lastAdvKick = 0;
        if (Throttle::hasElapsed(lastAdvKick, 1000)) {
            lastAdvKick = millis();
            Bluefruit.Advertising.start(0);
            if (Bluefruit.Advertising.isRunning())
                LOG_INFO("BLE: advertising re-armed for a second connection");
            else
                LOG_WARN("BLE: advertising re-arm failed");
        }
    }
#endif
#ifdef NEXUS_ENABLE_AIRTAG
    if (!nexus_airtag_enabled) {
        if (inAirtagMode) {
            inAirtagMode = false;
            Bluefruit.Advertising.stop();
            startAdv();
        }
        return;
    }

    // Do not switch modes if a phone is currently connected
    if (isConnected()) {
        if (inAirtagMode) {
            inAirtagMode = false;
            Bluefruit.Advertising.stop();
            startAdv();
        }
        return;
    }

    uint32_t now = millis();
    // Timing: 30 seconds Meshtastic, 10 seconds AirTag
    uint32_t interval = inAirtagMode ? 10000 : 30000;

    if (now - lastModeSwitch > interval) {
        inAirtagMode = !inAirtagMode;
        lastModeSwitch = now;
        
        LOG_INFO("Nexus: Switching BLE to %s mode", inAirtagMode ? "AirTag" : "Meshtastic");
        
        Bluefruit.Advertising.stop();
        Bluefruit.Advertising.clearData();
        startAdv();
    }
#endif
}
