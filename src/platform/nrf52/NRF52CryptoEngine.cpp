#include "CryptoEngine.h"
#include "configuration.h"
#include <Adafruit_nRFCrypto.h>

// SaSi_LibInit() (nRFCrypto.begin) costs ~95 ms of the main loop. Starting and stopping CryptoCell around every
// AES block or RNG fill put that on every packet - a DM's nonce fill stopped it and decrypting the ack restarted it.
// Keep it up while in use and power it down after a short idle period instead.
static const uint32_t CRYPTOCELL_IDLE_MSEC = 2000;
static uint32_t cryptoCellLastUse;

void nrf52CryptoCellAcquire()
{
    nRFCrypto.begin(); // no-op while already up
    cryptoCellLastUse = millis() | 1;
}

void nrf52CryptoCellIdleCheck()
{
    if (cryptoCellLastUse && millis() - cryptoCellLastUse > CRYPTOCELL_IDLE_MSEC) {
        nRFCrypto.end();
        cryptoCellLastUse = 0;
    }
}

class NRF52CryptoEngine : public CryptoEngine
{
  public:
    NRF52CryptoEngine() {}

    ~NRF52CryptoEngine() {}

    virtual void encryptAESCtr(CryptoKey _key, uint8_t *_nonce, size_t numBytes, uint8_t *bytes) override
    {
        if (_key.length > 16) {
            // CryptoCell only accelerates AES-128; AES-256 uses the shared software CTR.
            CryptoEngine::encryptAESCtr(_key, _nonce, numBytes, bytes);
        } else if (_key.length > 0) {
            nrf52CryptoCellAcquire();
            nRFCrypto_AES ctx;
            // Must not be uint8_t: blockLen() rounds up to the AES block size, so numBytes in
            // 241..256 yields 256, which truncates to 0 and leaves Process() writing numBytes
            // attacker-controlled bytes onto a zero-length stack buffer.
            size_t myLen = ctx.blockLen(numBytes);
            char encBuf[myLen] = {0};
            ctx.begin();
            ctx.Process((char *)bytes, numBytes, _nonce, _key.bytes, _key.length, encBuf, ctx.encryptFlag, ctx.ctrMode);
            ctx.end();
            memcpy(bytes, encBuf, numBytes);
        }
    }
};

CryptoEngine *crypto = new NRF52CryptoEngine();