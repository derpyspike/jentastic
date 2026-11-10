// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez

#include "ExternalFlash.h"

#if NEXUS_ENABLE_EXTERNAL_FLASH && defined(ARCH_NRF52)

#include "DebugConfiguration.h"
#include <Arduino.h>
#include <nrfx_qspi.h>
#include <cstdint>
#include <cstring>

namespace nexus::extflash
{
namespace
{
constexpr uint32_t PROG_SIZE = 256;

bool tried = false;
bool qspiInited = false;
bool probedReady = false;
uint32_t jedecId = 0;
uint32_t flashBytes = 0;
uint8_t progBounce[PROG_SIZE] __attribute__((aligned(4)));

bool cinstr(uint8_t opcode, uint8_t *rx, uint8_t rxLen)
{
    nrf_qspi_cinstr_conf_t c = {};
    c.opcode = opcode;
    c.length = (nrf_qspi_cinstr_len_t)(rxLen + 1);
    // WP#/HOLD# sit on IO2/IO3 and must stay deasserted.
    c.io2_level = true;
    c.io3_level = true;
    c.wipwait = false;
    c.wren = false;
    return nrfx_qspi_cinstr_xfer(&c, nullptr, rx) == NRFX_SUCCESS;
}

bool probe()
{
    (void)cinstr(0xAB, nullptr, 0); // Release from Deep Power-Down
    delayMicroseconds(50);

    uint8_t id[3] = {};
    if (!cinstr(0x9F, id, 3)) {
        LOG_WARN("ExtFlash: JEDEC read failed");
        probedReady = false;
        return false;
    }
    jedecId = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
    flashBytes = (id[2] < 32) ? (1UL << id[2]) : 0;
    probedReady = (id[0] != 0x00 && id[0] != 0xFF);
    LOG_INFO("ExtFlash: JEDEC=%06lX size=%luKB", (unsigned long)jedecId, (unsigned long)(flashBytes / 1024));
    return probedReady;
}

bool initQspi()
{
    if (qspiInited)
        return true;

#ifndef PIN_QSPI_SCK
#error "NEXUS_ENABLE_EXTERNAL_FLASH requires PIN_QSPI_* in the variant"
#endif

#ifdef PIN_QSPI_POWER
    // Switched 3V3 rail on the X1; the part answers JEDEC only after it settles.
    pinMode(PIN_QSPI_POWER, OUTPUT);
    digitalWrite(PIN_QSPI_POWER, HIGH);
    delay(100);
#endif

    nrfx_qspi_config_t cfg = {};
    cfg.pins.sck_pin = (uint8_t)g_ADigitalPinMap[PIN_QSPI_SCK];
    cfg.pins.csn_pin = (uint8_t)g_ADigitalPinMap[PIN_QSPI_CS];
    cfg.pins.io0_pin = (uint8_t)g_ADigitalPinMap[PIN_QSPI_IO0];
    cfg.pins.io1_pin = (uint8_t)g_ADigitalPinMap[PIN_QSPI_IO1];
    cfg.pins.io2_pin = (uint8_t)g_ADigitalPinMap[PIN_QSPI_IO2];
    cfg.pins.io3_pin = (uint8_t)g_ADigitalPinMap[PIN_QSPI_IO3];
    cfg.prot_if.readoc = NRF_QSPI_READOC_FASTREAD; // single-line 0x0B
    cfg.prot_if.writeoc = NRF_QSPI_WRITEOC_PP;     // single-line 0x02
    cfg.prot_if.addrmode = NRF_QSPI_ADDRMODE_24BIT;
    cfg.prot_if.dpmconfig = false;
    cfg.phy_if.sck_delay = 5;
    cfg.phy_if.dpmen = false;
    cfg.phy_if.spi_mode = NRF_QSPI_MODE_0;
    cfg.phy_if.sck_freq = NRF_QSPI_FREQ_32MDIV16; // 2 MHz, safe on all parts
    cfg.irq_priority = 7;

    if (nrfx_qspi_init(&cfg, nullptr, nullptr) != NRFX_SUCCESS) {
        LOG_WARN("ExtFlash: QSPI init failed");
        return false;
    }
    qspiInited = true;
    return true;
}
} // namespace

bool begin()
{
    if (tried)
        return probedReady;
    tried = true;

    if (!initQspi()) {
        probedReady = false;
        return false;
    }
    return probe();
}

bool retry()
{
    if (!qspiInited)
        return begin();
    return probe();
}

bool ready() { return probedReady; }
uint32_t jedec() { return jedecId; }
uint32_t capacity() { return flashBytes; }

bool read(uint32_t addr, void *data, size_t len)
{
    uint8_t *p = static_cast<uint8_t *>(data);
    if (!probedReady)
        return false;

    if ((reinterpret_cast<uintptr_t>(p) & 3) != 0) {
        uint8_t buf4[4] __attribute__((aligned(4)));
        size_t count = 4 - (reinterpret_cast<uintptr_t>(p) & 3);
        if (count > len)
            count = len;
        if (nrfx_qspi_read(buf4, 4, addr) != NRFX_SUCCESS)
            return false;
        memcpy(p, buf4, count);
        p += count;
        addr += (uint32_t)count;
        len -= count;
    }
    if (len > 3) {
        size_t len4 = len & ~(size_t)3;
        if (nrfx_qspi_read(p, (uint32_t)len4, addr) != NRFX_SUCCESS)
            return false;
        p += len4;
        addr += (uint32_t)len4;
        len -= len4;
    }
    if (len) {
        uint8_t buf4[4] __attribute__((aligned(4)));
        if (nrfx_qspi_read(buf4, 4, addr) != NRFX_SUCCESS)
            return false;
        memcpy(p, buf4, len);
    }
    return true;
}

bool program(uint32_t addr, const void *data, size_t len)
{
    if (!probedReady)
        return false;

    const uint8_t *src = static_cast<const uint8_t *>(data);
    while (len) {
        size_t n = len < PROG_SIZE ? len : PROG_SIZE;
        size_t n4 = (n + 3) & ~(size_t)3;
        memcpy(progBounce, src, n);
        if (n4 > n)
            memset(progBounce + n, 0xFF, n4 - n);
        if (nrfx_qspi_write(progBounce, (uint32_t)n4, addr) != NRFX_SUCCESS)
            return false;
        src += n;
        addr += (uint32_t)n;
        len -= n;
    }
    return true;
}

bool erase4k(uint32_t addr) { return probedReady && nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, addr) == NRFX_SUCCESS; }
} // namespace nexus::extflash

#endif // NEXUS_ENABLE_EXTERNAL_FLASH && ARCH_NRF52
