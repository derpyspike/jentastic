#include "RadioLibInterface.h"
#include "ContentionStats.h"
#include "PacketProfiler.h"
#include "modules/NexusModule.h"
#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
#include "FSCommon.h"
#include "HardwareRNG.h"
#include "JenCoreRepeater.h"
#if MESHTASTIC_ENABLE_JENCORE_COMPANION
#include "JenCoreCompanion.h"
#endif
#include "NodeDB.h"
#include "Router.h"
#include "SafeFile.h"
#include "gps/RTC.h"
#endif
#if MESHTASTIC_ENABLE_RNODE
#include "RNodeEngine.h"
#if MESHTASTIC_ENABLE_RNODE_TCP && HAS_WIFI
#include "RNodeTcpServer.h"
#endif
#endif
#include "MeshTypes.h"
#include "NodeDB.h"
#include "PowerMon.h"
#include "RadioTxHook.h"
#include "SPILock.h"
#include "Throttle.h"
#include "UptimeClock.h"
#include "configuration.h"
#include "error.h"
#include "main.h"
#include "mesh-pb-constants.h"
#include <pb_decode.h>
#include <pb_encode.h>

#if ARCH_PORTDUINO
#include "PortduinoGlue.h"
#include "meshUtils.h"
#endif

void LockingArduinoHal::spiBeginTransaction()
{
    spiLock->lock();

    ArduinoHal::spiBeginTransaction();
}

void LockingArduinoHal::spiEndTransaction()
{
    ArduinoHal::spiEndTransaction();

    spiLock->unlock();
}

#if ARCH_PORTDUINO
void LockingArduinoHal::spiTransfer(uint8_t *out, size_t len, uint8_t *in)
{
    spi->transfer(out, in, len);
}
#endif

RadioLibInterface::RadioLibInterface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                                     RADIOLIB_PIN_TYPE busy, PhysicalLayer *_iface)
    : NotifiedWorkerThread("RadioIf"), module(hal, cs, irq, rst, busy), iface(_iface)
{
    instance = this;

    // Initialize unused sample slots to a sane default; sample count controls averaging.
    for (uint8_t i = 0; i < NOISE_FLOOR_SAMPLES; i++) {
        noiseFloorSamples[i] = NOISE_FLOOR_DEFAULT;
    }

#if defined(ARCH_STM32WL) && defined(USE_SX1262)
    module.setCb_digitalWrite(stm32wl_emulate_digitalWrite);
    module.setCb_digitalRead(stm32wl_emulate_digitalRead);
#endif
}

#ifdef ARCH_ESP32
// ESP32 doesn't use that flag
#define YIELD_FROM_ISR(x) portYIELD_FROM_ISR()
#else
#define YIELD_FROM_ISR(x) portYIELD_FROM_ISR(x)
#endif

void INTERRUPT_ATTR RadioLibInterface::isrLevel0Common(PendingISR cause)
{
    PROF_DO(prof::isrAt = micros());
    instance->disableInterrupt();

    BaseType_t xHigherPriorityTaskWoken;
    instance->notifyFromISR(&xHigherPriorityTaskWoken, cause, true);

    /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
    The macro used to do this is dependent on the port and may be called
    portEND_SWITCHING_ISR. */
    YIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void INTERRUPT_ATTR RadioLibInterface::isrRxLevel0()
{
    isrLevel0Common(ISR_RX);
}

void INTERRUPT_ATTR RadioLibInterface::isrTxLevel0()
{
    isrLevel0Common(ISR_TX);
}

/** Our ISR code currently needs this to find our active instance
 */
RadioLibInterface *RadioLibInterface::instance;

/** Could we send right now (i.e. either not actively receiving or transmitting)? */
bool RadioLibInterface::canSendImmediately()
{
    // We wait _if_ we are partially though receiving a packet (rather than just merely waiting for one).
    // To do otherwise would be doubly bad because not only would we drop the packet that was on the way in,
    // we almost certainly guarantee no one outside will like the packet we are sending.
    bool busyTx = isSending();
    bool busyRx = isReceiving && isActivelyReceiving();

    if (busyTx || busyRx) {
        if (busyTx) {
            LOG_WARN("Can not send yet, busyTx");
        }
        // If we've been trying to send the same packet more than one minute and we haven't gotten a
        // TX IRQ from the radio, the radio is probably broken.
        if (busyTx && !Throttle::isWithinTimespanMs(lastTxStart, 60000)) {
            LOG_ERROR("Hardware Failure! busyTx >60s");
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_TRANSMIT_FAILED);
            // reboot in 5 seconds when this condition occurs.
            rebootAtMsec = Time::skipZero(lastTxStart + 65000);
        }
        if (busyRx) {
            LOG_WARN("Can not send yet, busyRx");
        }
        return false;
    } else
        return true;
}

bool RadioLibInterface::receiveDetected(uint16_t irq, unsigned long syncWordHeaderValidFlag, unsigned long preambleDetectedFlag)
{
    bool detected = (irq & (syncWordHeaderValidFlag | preambleDetectedFlag));
    // Handle false detections
    if (detected) {
        if (!activeReceiveStart) {
            activeReceiveStart = Time::skipZero(Time::getMillis());
        } else if (!Throttle::isWithinTimespanMs(activeReceiveStart, 2 * preambleTimeMsec)) {
            if (!(irq & syncWordHeaderValidFlag)) {
                // The HEADER_VALID flag should be set by now if it was really a packet, so ignore PREAMBLE_DETECTED flag
                activeReceiveStart = 0;
                LOG_TRACE("Ignore false preamble detection");
                return false;
            } else {
                uint32_t maxPacketTimeMsec = getPacketTime(meshtastic_Constants_DATA_PAYLOAD_LEN + sizeof(PacketHeader));
                if (!Throttle::isWithinTimespanMs(activeReceiveStart, maxPacketTimeMsec)) {
                    // We should have gotten an RX_DONE IRQ by now if it was really a packet, so ignore HEADER_VALID flag
                    activeReceiveStart = 0;
                    LOG_TRACE("Ignore false header detection");
                    return false;
                }
            }
        }
    }
    return detected;
}

/// Send a packet (possibly by enquing in a private fifo).  This routine will
/// later free() the packet to pool.  This routine is not allowed to stall because it is called from
/// bluetooth comms code.  If the txmit queue is empty it might return an error
ErrorCode RadioLibInterface::send(meshtastic_MeshPacket *p)
{

#ifndef DISABLE_WELCOME_UNSET

    if (config.lora.region != meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
        if (disabled || !config.lora.tx_enabled) {
            LOG_WARN("send - !config.lora.tx_enabled");
            packetPool.release(p);
            return ERRNO_DISABLED;
        }

    } else {
        LOG_WARN("send - lora tx disabled: Region unset");
        packetPool.release(p);
        return ERRNO_DISABLED;
    }

#else

    if (disabled || !config.lora.tx_enabled) {
        LOG_WARN("send - !config.lora.tx_enabled");
        packetPool.release(p);
        return ERRNO_DISABLED;
    }

#endif

    if (p->to == NODENUM_BROADCAST_NO_LORA) {
        LOG_DEBUG("Drop no-LoRa pkt");
        return ERRNO_SHOULD_RELEASE;
    }

    // Sometimes when testing it is useful to be able to never turn on the xmitter
#ifndef LORA_DISABLE_SENDING
    printPacket("enqueue for send", p);

    LOG_TRACE("txGood=%d,txRelay=%d,rxGood=%d,rxBad=%d", txGood, txRelay, rxGood, rxBad);
    bool dropped = false;
    ErrorCode res = txQueue.enqueue(p, &dropped) ? ERRNO_OK : ERRNO_UNKNOWN;

    if (dropped) {
        txDrop++;
    }

    if (res != ERRNO_OK) { // we weren't able to queue it, so we must drop it to prevent leaks
        packetPool.release(p);
        return res;
    }
    PROF_DO(prof::mark(p->id, prof::SLOT_TXQ));

    // set (random) transmit delay to let others reconfigure their radio,
    // to avoid collisions and implement timing-based flooding
    setTransmitDelay();

    return res;
#else
    packetPool.release(p);
    return ERRNO_DISABLED;
#endif
}

meshtastic_QueueStatus RadioLibInterface::getQueueStatus()
{
    meshtastic_QueueStatus qs;

    qs.res = qs.mesh_packet_id = 0;
    qs.free = txQueue.getFree();
    qs.maxlen = txQueue.getMaxLen();

    return qs;
}

bool RadioLibInterface::canSleep(bool deepSleep)
{
    // A packet being actively transmitted has already left the TX queue (sendingPacket), so
    // check it separately. It only vetoes deep sleep: light sleep keeps the radio powered and
    // the TX finishes on its own, but deep sleep powers the radio down and would truncate the
    // packet on air.
    bool res = txQueue.empty() && !(deepSleep && isSending());
    if (!res) { // only print debug messages if we are vetoing sleep
        LOG_DEBUG("Radio wait to sleep, txEmpty=%d, txInFlight=%d", txQueue.empty(), isSending());
    }
    return res;
}

/** Allow other firmware components to ask whether we are currently sending a packet
Initially implemented to protect T-Echo's capacitive touch button from spurious presses during tx
*/
bool RadioLibInterface::isSending()
{
    return sendingPacket != NULL || isRawSending();
}

/** Attempt to cancel a previously sent packet.  Returns true if a packet was found we could cancel */
bool RadioLibInterface::cancelSending(NodeNum from, PacketId id)
{
    auto p = txQueue.remove(from, id);
    if (p) {
        RadioTxHooks::packetReleased(this, p);
        packetPool.release(p); // free the packet we just removed
    }

    bool result = (p != NULL);
    LOG_DEBUG("cancelSending id=0x%08x, removed=%d", id, result);
    return result;
}

/** Attempt to find a packet in the TxQueue. Returns true if the packet was found. */
bool RadioLibInterface::findInTxQueue(NodeNum from, PacketId id)
{
    return txQueue.find(from, id);
}

void RadioLibInterface::updateNoiseFloor()
{
    // Only sample from idle receive mode. TX/RX-critical paths must return to radio work quickly.
    if (!isReceiving || isSending() || isActivelyReceiving() || isIRQPending()) {
        return;
    }

    if (Throttle::isWithinTimespanMs(lastNoiseFloorUpdate, NOISE_FLOOR_UPDATE_INTERVAL_MS)) {
        return;
    }
    lastNoiseFloorUpdate = Time::getMillis();

    int16_t rssi = getCurrentRSSI();
    if (rssi == NOISE_FLOOR_INVALID || rssi >= 0 || rssi < NOISE_FLOOR_VALID_MIN) {
        LOG_DEBUG("Skipping invalid RSSI reading: %d", rssi);
        return;
    }

    noiseFloorSamples[currentSampleIndex] = (int32_t)rssi;
    currentSampleIndex++;

    if (currentSampleIndex >= NOISE_FLOOR_SAMPLES) {
        currentSampleIndex = 0;
        isNoiseFloorBufferFull = true;
    }

    currentNoiseFloor = getAverageNoiseFloorInternal();

    LOG_TRACE("Noise floor: %d dBm (samples: %d, latest: %d dBm)", currentNoiseFloor, getNoiseFloorSampleCountInternal(), rssi);
}

uint8_t RadioLibInterface::getNoiseFloorSampleCountInternal() const
{
    return isNoiseFloorBufferFull ? NOISE_FLOOR_SAMPLES : currentSampleIndex;
}

int32_t RadioLibInterface::getAverageNoiseFloorInternal() const
{
    uint8_t sampleCount = getNoiseFloorSampleCountInternal();

    if (sampleCount == 0) {
        return NOISE_FLOOR_DEFAULT;
    }

    int32_t sum = 0;
    for (uint8_t i = 0; i < sampleCount; i++) {
        sum += noiseFloorSamples[i];
    }

    return sum / sampleCount;
}

int32_t RadioLibInterface::getAverageNoiseFloor()
{
    return getAverageNoiseFloorInternal();
}

int32_t RadioLibInterface::getNoiseFloor()
{
    return currentNoiseFloor;
}

bool RadioLibInterface::hasNoiseFloorSamples()
{
    return getNoiseFloorSampleCountInternal() > 0;
}

uint8_t RadioLibInterface::getNoiseFloorSampleCount()
{
    return getNoiseFloorSampleCountInternal();
}

void RadioLibInterface::resetNoiseFloor()
{
    currentSampleIndex = 0;
    isNoiseFloorBufferFull = false;
    currentNoiseFloor = NOISE_FLOOR_DEFAULT;
    LOG_INFO("Noise floor reset - rolling window will restart");
}

bool RadioLibInterface::randomBytes(uint8_t *buffer, size_t length)
{
    if (!buffer || length == 0 || !iface) {
        return false;
    }

    // Older RadioLib versions only expose random(min, max), so fill the buffer byte-by-byte.
    for (size_t i = 0; i < length; ++i) {
        int32_t value = iface->random(0, 255);
        if (value < 0) {
            return false;
        }
        buffer[i] = static_cast<uint8_t>(value & 0xFF);
    }

    return true;
}

/** radio helper thread callback.
We never immediately transmit after any operation (either Rx or Tx). Instead we should wait a random multiple of
'slotTimes' (see definition in RadioInterface.h) taken from a contention window (CW) to lower the chance of collision.
The CW size is determined by setTransmitDelay() and depends either on the current channel utilization or SNR in case
of a flooding message. After this, we perform channel activity detection (CAD) and reset the transmit delay if it is
currently active.
*/
// In software-IRQ-poll mode (LORA_DIO1_SOFTWARE_POLL) a 1ms poll tick is almost always pending, so
// TX timers must be allowed to overwrite the pending notification or TX scheduling starves. On all
// other targets keep the historical non-overwriting behavior.
#ifdef LORA_DIO1_SOFTWARE_POLL
static constexpr bool txTimerOverwrite = true;
#else
static constexpr bool txTimerOverwrite = false;
#endif

// cppcheck-suppress constParameterPointer ; a function pointer can't meaningfully point to const
bool RadioLibInterface::isIsrTxCallback(void (*callback)())
{
    return callback == isrTxLevel0;
}

void RadioLibInterface::scheduleIrqPollTick()
{
    // Never overwrite a pending notification (especially TRANSMIT_DELAY_COMPLETED),
    // otherwise poll ticks would starve TX scheduling.
    //
    // There is a single notification slot, so while a TX is queued and the radio is busy receiving,
    // the self-rescheduling TRANSMIT_DELAY_COMPLETED timer (which does overwrite, see txTimerOverwrite)
    // can keep the slot and prevent a poll tick from being scheduled. In that window a completing
    // RX/TX is not seen by the poll; RadioInterface's pollMissedIrqs() (~1s) is the backup that
    // recovers it, so the effect is bounded added latency under heavy contention, not a lost event.
    notifyLater(1, ISR_POLL_TICK, false);
}

void RadioLibInterface::deliverPendingIrqFromPoll(PendingISR cause)
{
    disableInterrupt(); // stop polling; this is the poll-path equivalent of isrLevel0Common()
    notify(cause, true);
}

void RadioLibInterface::onNotify(uint32_t notification)
{
#ifdef PROFILE_PACKETS
    if (prof::isrAt && (notification == ISR_TX || notification == ISR_RX)) {
        prof::add(prof::ISR_LATENCY, micros() - prof::isrAt);
        if (notification == ISR_TX)
            prof::add(prof::TX_AIR, prof::isrAt - prof::txStartAt);
        prof::isrAt = 0;
    }
#endif

    switch (notification) {
    case ISR_TX:
        if (sendingPacket) {
            if (uint32_t wait = nextFrameDelayMsec()) {
                notifyLater(wait, NEXT_FRAME, true); // sendingPacket stays set, so nothing else is sent meanwhile
                break;
            }
        }
        onTransmitDone();
        break;
    case NEXT_FRAME:
        PROF_DO(prof::txStartAt = micros());
        if (sendingPacket && sendNextFrame()) {
            enableInterrupt(isrTxLevel0);
            break;
        }
        onTransmitDone();
        break;
    case ISR_RX:
        // PREAMBLE_DETECTED/HEADER_VALID/SYNC are routed to the edge-triggered DIO1
        // (MESHTASTIC_RADIOLIB_IRQ_RX_FLAGS), so every packet's own preamble fires this
        // notification BEFORE RX_DONE. Handled as a completed receive it reads an empty
        // buffer (silent rxBad) and the startReceive() below puts the chip in standby —
        // aborting the packet in flight. LoRa only survives because its 32 ms preamble
        // outlasts the ~1 ms abort+re-arm; FSK's whole preamble+sync is ~0.3 ms (16 bits
        // @ 50 kbps), so the chip never re-syncs and the receiver is deaf to every
        // packet — bench 2026-10-18: SX1262 AND SX1280 pairs both deaf, clean logs.
        // Guard: terminal events only. A preamble/header-only edge: clear the latch so
        // DIO1 drops (RX_DONE then produces a fresh edge instead of waiting for the
        // ~1 s pollMissedIrqs backup), re-arm the interrupt, no abort.
        if (!iface->checkIrq(RADIOLIB_IRQ_RX_DONE) && !iface->checkIrq(RADIOLIB_IRQ_TIMEOUT) &&
            !iface->checkIrq(RADIOLIB_IRQ_CRC_ERR) && !iface->checkIrq(RADIOLIB_IRQ_HEADER_ERR)) {
            iface->clearIrq((1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED) | (1UL << RADIOLIB_IRQ_HEADER_VALID) |
                            (1UL << RADIOLIB_IRQ_SYNC_WORD_VALID));
            if (!sendingPacket)
                enableInterrupt(isrRxLevel0);
            return;
        }
        handleReceiveInterrupt();
        startReceive();
        setTransmitDelay();
        break;
    case ISR_POLL_TICK:
        handleSoftwareLoraIrqPoll();
        break;
    case TRANSMIT_DELAY_COMPLETED:

        // If we are not currently in receive mode, then restart the random delay (this can happen if the main thread
        // has placed the unit into standby)  FIXME, how will this work if the chipset is in sleep mode?
        if (!txQueue.empty()) {
            if (!canSendImmediately()) {
                setTransmitDelay(); // currently Rx/Tx-ing: reset random delay
            } else {
                meshtastic_MeshPacket *txp = txQueue.getFront();
                assert(txp);
                const uint32_t now = Time::getMillis();
                // Not `long remaining = tx_after - Time::getMillis()`: that uint32_t subtraction widens to
                // ~4.29e9 where long is 64-bit (portduino), rescheduling a due packet ~49.7 days out.
                if (txp->tx_after && !Throttle::deadlinePassedAt(now, txp->tx_after)) {
                    // There's still some delay pending on this packet, so resume waiting for it to elapse
                    notifyLater(txp->tx_after - now, TRANSMIT_DELAY_COMPLETED, txTimerOverwrite);
                } else if (const RadioTxHook::PreTxAction action = RadioTxHooks::beforeTransmit(this, txp);
                           action == RadioTxHook::PRETX_DROP) {
                    // A module refuses this packet on the radio config we are holding: drop it rather
                    // than transmit it, and move on to the next queued packet.
                    meshtastic_MeshPacket *bad = txQueue.dequeue();
                    LOG_DEBUG("Drop Tx packet 0x%08x, refused before transmit", bad->id);
                    RadioTxHooks::packetReleased(this, bad);
                    packetPool.release(bad);
                    setTransmitDelay();
                } else if (action == RadioTxHook::PRETX_DEFER) {
                    setTransmitDelay(); // the radio config moved, so re-run the delay and scan on it
                } else {
                    if (isChannelActive()) { // check if there is currently a LoRa packet on the channel
#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
                        meshtasticCadBusy++;
#endif
                        if (!RadioTxHooks::holdsRadio(txp)) {
                            startReceive(); // try receiving this packet, afterwards we'll be trying to transmit again
                        }
                        setTransmitDelay();
                    } else {
                        // Send any outgoing packets we have ready as fast as possible to keep the time between channel scan and
                        // actual transmission as short as possible
                        txp = txQueue.dequeue();
                        assert(txp);
                        startSend(txp);
                        LOG_TRACE("%d packets in TX queue", txQueue.getMaxLen() - txQueue.getFree());
                    }
                }
            }
        } else {
            // Do nothing, because the queue is empty
        }
        break;
    default:
        assert(0); // We expected to receive a valid notification from the ISR
    }
}

void RadioLibInterface::onTransmitDone()
{
    handleTransmitInterrupt(); // completeSending() already restored the radio to the home config
    // Let the hooks pre-stage the radio for the NEXT queued packet. Not required for correctness -
    // TRANSMIT_DELAY_COMPLETED asks again before the scan, which is where the answer is acted on -
    // but it keeps the post-TX listen window on the channel we are about to transmit on.
    (void)RadioTxHooks::beforeTransmit(this, txQueue.getFront());
    startReceive();
    setTransmitDelay();
#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
    // A JenCore frame never follows a transmission straight away: hold it back a few random slots.
    jencoreRepeater.afterTransmit(slotTimeMsec);
    wakeJenCoreRelay();
#endif
}

void RadioLibInterface::setTransmitDelay()
{
    meshtastic_MeshPacket *p = txQueue.getFront();
    if (!p) {
        return; // noop if there's nothing in the queue
    }

    // We want all sending/receiving to be done by our daemon thread.
    // We use a delay here because this packet might have been sent in response to a packet we just received.
    // So we want to make sure the other side has had a chance to reconfigure its radio.

    if (p->tx_after) {
        unsigned long add_delay = p->rx_rssi ? getTxDelayMsecWeighted(p) : getTxDelayMsec();
        unsigned long now = Time::getMillis();
        // skipZero, not timerEndsAtMillis: this is a clamp of three candidates rather than a plain
        // now + delay, and `if (p->tx_after)` above is the read that takes 0 as "no delay wanted" -
        // so a recomputation landing on 0 drops the CSMA backoff and the packet goes out at once.
        //
        // Narrow to uint32_t BEFORE skipZero, not after: add_delay is unsigned long, 64-bit on the
        // portduino host, so the clamp can exceed UINT32_MAX there. skipZero on the wide value would
        // pass 0x100000000 through as non-zero and the store to this uint32_t field would truncate it
        // back to the 0 being avoided.
        p->tx_after = Time::skipZero(
            (uint32_t)min(max(p->tx_after + add_delay, now + add_delay), now + 2 * getTxDelayMsecWeightedWorst(p->rx_snr)));
        notifyLater(p->tx_after - now, TRANSMIT_DELAY_COMPLETED, txTimerOverwrite);
    } else if (p->rx_snr == 0 && p->rx_rssi == 0) {
        /* We assume if rx_snr = 0 and rx_rssi = 0, the packet was generated locally.
         *   This assumption is valid because of the offset generated by the radio to account for the noise
         *   floor.
         */
        startTransmitTimer(true);
    } else {
        // If there is a SNR, start a timer scaled based on that SNR.
        LOG_TRACE("rx_snr found. hop_limit:%d rx_snr:%f", p->hop_limit, p->rx_snr);
        startTransmitTimerRebroadcast(p);
    }
}

void RadioLibInterface::startTransmitTimer(bool withDelay)
{
    // If we have work to do and the timer wasn't already scheduled, schedule it now
    if (!txQueue.empty()) {
        uint32_t delay = !withDelay ? 1 : getTxDelayMsec();
        notifyLater(delay, TRANSMIT_DELAY_COMPLETED, txTimerOverwrite); // This will implicitly enable
    }
}

void RadioLibInterface::startTransmitTimerRebroadcast(meshtastic_MeshPacket *p)
{
    // If we have work to do and the timer wasn't already scheduled, schedule it now
    if (!txQueue.empty()) {
        uint32_t delay = getTxDelayMsecWeighted(p);
        notifyLater(delay, TRANSMIT_DELAY_COMPLETED, txTimerOverwrite); // This will implicitly enable
    }
}

/**
 * If the packet is not already in the late rebroadcast window, move it there
 */
void RadioLibInterface::clampToLateRebroadcastWindow(NodeNum from, PacketId id)
{
    // Look for non-late packets only, so we don't do this twice!
    meshtastic_MeshPacket *p = txQueue.remove(from, id, true, false);
    if (p) {
        p->tx_after = Time::timerEndsAtMillis(getTxDelayMsecWeightedWorst(p->rx_snr));
        bool dropped = false;
        if (txQueue.enqueue(p, &dropped)) {
            LOG_TRACE("Move queued packet to late rebroadcast window %ums from now", (uint32_t)(p->tx_after - millis()));
        } else {
            packetPool.release(p);
        }
        if (dropped) {
            txDrop++;
        }
    }
}

/**
 * If there is a packet pending TX in the queue with a worse hop limit, remove it pending replacement with a better version
 * @return Whether a pending packet was removed
 */
bool RadioLibInterface::removePendingTXPacket(NodeNum from, PacketId id, uint32_t hop_limit_lt)
{
    meshtastic_MeshPacket *p = txQueue.remove(from, id, true, true, hop_limit_lt);
    if (p) {
        LOG_DEBUG("Drop pending-TX packet 0x%08x, hop limit %d", p->id, p->hop_limit);
        RadioTxHooks::packetReleased(this, p);
        packetPool.release(p);
        return true;
    }
    return false;
}

bool RadioLibInterface::sendRawFrame(const uint8_t *bytes, size_t len)
{
    // Must not collide with the normal packet TX path or an in-flight raw frame. Listen-before-talk
    // is the caller's job (see maybeSendJenCoreFrame), which leaves the radio in standby.
    if (len == 0 || len > sizeof(rawTxBuf) || isSending()) {
        LOG_WARN("sendRawFrame rejected (len=%u sending=%d)", (unsigned)len, isSending());
        return false;
    }
    if (disabled || !config.lora.tx_enabled) {
        LOG_WARN("sendRawFrame - tx disabled");
        return false;
    }

    memcpy(rawTxBuf, bytes, len);
    rawTxLen = len;

    // Drive the radio into TX. startTransmit() arms the TX-done IRQ which
    // handleTransmitInterrupt() will route to completeRawSend(). rawSending is set before
    // configHardwareForSend() so drivers can pick the JenCore sync word for this frame.
    rawSending = true;
    enableInterrupt(isrTxLevel0);
    configHardwareForSend();
    int res = iface->startTransmit(rawTxBuf, rawTxLen);
    if (res != RADIOLIB_ERR_NONE) {
        LOG_ERROR("sendRawFrame startTransmit failed, error=%d", res);
        rawSending = false;
        rawTxLen = 0;
        startReceive();
        return false;
    }
    lastTxStart = millis();
    return true;
}

bool RadioLibInterface::jencoreTxAllowed()
{
    if (disabled || !config.lora.tx_enabled)
        return false;
    // Same duty-cycle guard as Router::send: a foreign-protocol frame must not push us over the regional limit.
    const float effectiveDutyCycle = getEffectiveDutyCycle();
    return config.lora.override_duty_cycle || effectiveDutyCycle >= 100 || airTime->utilizationTXPercent() <= effectiveDutyCycle;
}

bool RadioLibInterface::queueJenCoreFrame(const uint8_t *bytes, size_t len)
{
#if MESHTASTIC_ENABLE_JENCORE_COMPANION
    if (!bytes || len == 0 || len > JenCoreRepeater::MAX_FRAME_LEN)
        return false;

    if (!jencoreTxAllowed()) {
        LOG_WARN("JenCore TX refused: tx disabled or duty cycle exceeded");
        return false;
    }

    // Our own text outranks every queued flood (it evicts the worst one when the queue is full) but never evicts
    // another frame of ours (advert, discover reply) or routed traffic.
    if (!jencoreRepeater.queueOwnFrame(bytes, len)) {
        LOG_WARN("JenCore TX refused: the relay queue holds only own and routed frames");
        return false;
    }

    jencoreCompanion.noteTransmitted(bytes, len);
    jencoreCompanion.txSent++;
    wakeJenCoreRelay();
    return true;
#else
    (void)bytes;
    (void)len;
    return false;
#endif
}

void RadioLibInterface::completeRawSend()
{
    rawSending = false;
    uint32_t xmitMsec = getPacketTime(rawTxLen, false);
    airTime->logAirtime(TX_LOG, xmitMsec);
#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
    jencoreRepeater.txAirtimeMs += xmitMsec;
#endif
#if MESHTASTIC_ENABLE_RNODE
    if (rawSendingType == RAW_SEND_RNODE) {
        rnodeEngine.txPackets++;
        rnodeEngine.txBytes += rawTxLen;
    }
#endif
    rawSendingType = RAW_SEND_NONE;
    // txGood feeds the Meshtastic num_packets_tx telemetry: JenCore frames are counted by jencoreRepeater/Companion.
    rawTxLen = 0;
}

#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
static constexpr size_t JENCORE_SEED_LEN = 32;
static constexpr uint32_t JENCORE_IDENTITY_LOAD_MS = 5 * 1000; // identity ready early: discovery needs it
static constexpr uint32_t JENCORE_ADVERT_FIRST_MS = 60 * 1000; // first zero-hop ADVERT a minute after boot
// A zero-hop advert every 2 minutes for direct neighbours (JenCore simple_repeater default); a flood advert for the whole
// mesh a few minutes after boot, so the node shows up across repeaters without waiting, then every 12 hours. A flood is
// re-sent by every node, so it must stay rare.
static constexpr uint32_t JENCORE_LOCAL_ADVERT_INTERVAL_MS = 2 * 60 * 1000;
static constexpr uint32_t JENCORE_FLOOD_ADVERT_FIRST_MS = 3 * 60 * 1000;
static constexpr uint32_t JENCORE_FLOOD_ADVERT_INTERVAL_MS = 12UL * 60 * 60 * 1000;
static constexpr uint32_t JENCORE_ADVERT_RETRY_MS = 60 * 1000;           // no identity / no valid time
static constexpr uint32_t JENCORE_ADVERT_NUDGE_MS = 1000;                // radio busy: ask again
static constexpr uint32_t JENCORE_SERVICE_MS = 5 * 1000;                 // longest wait between housekeeping steps
static constexpr uint32_t JENCORE_CONTACTS_SAVE_MIN_MS = 10 * 60 * 1000; // flash wear: at most one save per 10 minutes
static constexpr uint32_t JENCORE_TX_POLL_MS = 100;                      // a raw frame is on air: look again for a lost TX-done

/** Runs RadioLibInterface::jencoreAdvertTick() on its own schedule. */
class JenCoreAdvertThread : public concurrency::OSThread
{
  public:
    explicit JenCoreAdvertThread(RadioLibInterface *radio) : OSThread("JenCoreAdvert", JENCORE_IDENTITY_LOAD_MS), radio(radio)
    {
    }

  protected:
    int32_t runOnce() override { return radio->jencoreServiceTick(); }

  private:
    RadioLibInterface *radio;
};

/**
 * The repeater's JenCore identity seed, kept in flash so the node keeps the same JenCore key across reboots.
 * Created from the hardware RNG on first use; never created from a weak source.
 */
static bool loadOrCreateJenCoreSeed(uint8_t seed[JENCORE_SEED_LEN], bool forceNew)
{
#ifdef FSCom
    static const char *path = "/prefs/meshcore_identity.bin";
    if (!forceNew) {
        concurrency::LockGuard g(spiLock);
        auto f = FSCom.open(path, FILE_O_READ);
        if (f) {
            bool ok = f.size() == JENCORE_SEED_LEN && f.read(seed, JENCORE_SEED_LEN) == JENCORE_SEED_LEN;
            f.close();
            if (ok)
                return true;
            LOG_WARN("JenCore identity: %s is corrupt, creating a new key", path);
        }
    }
#endif
    if (!HardwareRNG::fill(seed, JENCORE_SEED_LEN, true)) {
        LOG_ERROR("JenCore identity: no hardware entropy, not creating a key");
        return false;
    }
#ifdef FSCom
    // SafeFile: atomic replace under spiLock. A bare FILE_O_WRITE appends on nRF52 LittleFS.
    SafeFile out(path, true);
    out.write(seed, JENCORE_SEED_LEN);
    if (!out.close())
        LOG_ERROR("JenCore identity: could not save %s, the key will change on reboot", path);
#endif
    LOG_INFO("JenCore identity created");
    return true;
}

#if MESHTASTIC_ENABLE_JENCORE_COMPANION
static const char *jencoreContactsPath = "/prefs/meshcore_contacts.bin";

static void loadJenCoreContacts()
{
#ifdef FSCom
    uint8_t image[JenCoreCompanion::SAVED_IMAGE_MAX];
    size_t n = 0;
    {
        concurrency::LockGuard g(spiLock);
        auto f = FSCom.open(jencoreContactsPath, FILE_O_READ);
        if (!f)
            return;
        n = f.size() <= sizeof(image) ? f.read(image, f.size()) : 0;
        f.close();
    }
    const size_t loaded = jencoreCompanion.importContacts(image, n);
    LOG_INFO("JenCore contacts: %u restored from flash", (unsigned)loaded);
#endif
}

static void saveJenCoreContacts()
{
#ifdef FSCom
    uint8_t image[JenCoreCompanion::SAVED_IMAGE_MAX];
    const size_t n = jencoreCompanion.exportContacts(image, sizeof(image));
    if (n == 0)
        return;
    SafeFile out(jencoreContactsPath, true); // atomic replace
    out.write(image, n);
    if (!out.close())
        LOG_WARN("JenCore contacts: could not save %s", jencoreContactsPath);
#endif
}
#endif

size_t RadioLibInterface::formatContentionStats(char *buf, size_t len)
{
    ContentionStats s;
    s.mtRxGood = rxGood;
    s.mtRxBad = rxBad;
    s.mtTxGood = txGood;
    s.mtCadBusy = meshtasticCadBusy;
    s.mtTxAirtimeMs = meshtasticTxAirtimeMs;
    s.mtRxAirtimeMs = meshtasticRxAirtimeMs;
    s.mcSeen = jencoreRepeater.rxSeen;
    s.mcDup = jencoreRepeater.rxDup;
    s.mcRelayed = jencoreRepeater.txRelayed;
    s.mcDropped = jencoreRepeater.txDropped;
    s.mcDeferChannel = jencoreRepeater.deferChannel;
    s.mcDeferRadio = jencoreRepeater.deferRadio;
    s.mcTxAirtimeMs = jencoreRepeater.txAirtimeMs;
    s.mcRxAirtimeMs = jencoreRepeater.rxAirtimeMs;
    s.channelUtilPercent = airTime->channelUtilizationPercent();
    s.txUtilPercent = airTime->utilizationTXPercent();
    return ::formatContentionStats(buf, len, s);
}

uint32_t RadioLibInterface::jencoreServiceTick()
{
    uint32_t next = jencoreAdvertTick();
#if MESHTASTIC_ENABLE_JENCORE_COMPANION
    if (jencoreRepeater.hasIdentity() && !jencoreCompanion.hasIdentity()) {
        jencoreCompanion.setIdentity(jencoreRepeater.getIdentitySeed(), jencoreRepeater.getIdentityPub());
    }
    if (jencoreCompanion.hasIdentity() && !jencoreContactsLoaded) {
        jencoreContactsLoaded = true;
        loadJenCoreContacts();
        jencoreContactsFingerprint = jencoreCompanion.contactsFingerprint();
        lastJenCoreContactsSaveMs = millis();
    }

    jencoreCompanion.service();

    if (jencoreContactsLoaded && Throttle::hasElapsed(lastJenCoreContactsSaveMs, JENCORE_CONTACTS_SAVE_MIN_MS)) {
        const uint32_t fingerprint = jencoreCompanion.contactsFingerprint();
        if (fingerprint != jencoreContactsFingerprint) {
            saveJenCoreContacts();
            jencoreContactsFingerprint = fingerprint;
            lastJenCoreContactsSaveMs = millis();
        }
    }
#endif
    return next < JENCORE_SERVICE_MS ? next : JENCORE_SERVICE_MS;
}

uint32_t RadioLibInterface::jencoreAdvertTick()
{
    if (!jencoreRepeater.hasIdentity()) {
        uint8_t seed[JENCORE_SEED_LEN];
        if (!loadOrCreateJenCoreSeed(seed, false))
            return JENCORE_ADVERT_RETRY_MS;
        // A key JenCore rejects (public key starting 0x00/0xFF, ~1 in 128) is replaced by a fresh one.
        for (int attempt = 0; !jencoreRepeater.setIdentitySeed(seed); attempt++) {
            if (attempt >= 8 || !loadOrCreateJenCoreSeed(seed, true))
                return JENCORE_ADVERT_RETRY_MS;
        }
        const uint8_t *pub = jencoreRepeater.getIdentityPub();
        LOG_INFO("JenCore identity ready, pub=%02x%02x%02x%02x...", pub[0], pub[1], pub[2], pub[3]);
    }

    // One of our frames is queued but not on air yet: nudge the relay thread until it goes. A nudge that finds
    // the relay already scheduled changes nothing, so the next tick can repeat it.
    if (jencoreRepeater.hasPendingOwnFrame()) {
        wakeJenCoreRelay();
        return JENCORE_ADVERT_NUDGE_MS;
    }

    const bool floodDue = Throttle::deadlinePassed(nextJenCoreFloodAdvertMs);
    const bool localDue = Throttle::deadlinePassed(nextJenCoreLocalAdvertMs);
    if (!floodDue && !localDue) {
        const uint32_t now = millis();
        const uint32_t toFlood = nextJenCoreFloodAdvertMs - now, toLocal = nextJenCoreLocalAdvertMs - now;
        return toFlood < toLocal ? toFlood : toLocal;
    }

    // JenCore clients order adverts by timestamp, so it must be a real time - but any source will do.
    const uint32_t now = getValidTime(RTCQualityDevice);
    if (now == 0) {
        LOG_INFO("JenCore ADVERT postponed: no valid time yet");
        return JENCORE_ADVERT_RETRY_MS;
    }

    // A flood also reaches the direct neighbours, so it stands in for the local advert that is due with it.
    // JenCore clients ignore an advert without a name (BaseChatMesh::onAdvertRecv).
    char fallbackName[20];
    const char *name = owner.long_name;
    if (name[0] == 0) {
        snprintf(fallbackName, sizeof(fallbackName), "Meshtastic-%04x", (unsigned)(nodeDB->getNodeNum() & 0xFFFF));
        name = fallbackName;
    }
    uint8_t raw[JenCoreRepeater::MAX_FRAME_LEN];
    // The Meshtastic role decides how JenCore sees us: a router is a repeater, anything else a companion to message.
    jencoreRepeater.setNodeType(JenCoreRepeater::nodeTypeForRole(config.device.role));
    size_t len = jencoreRepeater.buildAdvert(raw, sizeof(raw), name, now, floodDue);
    if (len == 0 || !jencoreRepeater.queueOwnFrame(raw, len)) {
        LOG_WARN("JenCore ADVERT could not be built or queued");
        return JENCORE_ADVERT_RETRY_MS;
    }
    LOG_INFO("JenCore %s ADVERT queued (%u bytes, name=%s, ts=%u)", floodDue ? "flood" : "zero-hop", (unsigned)len, name,
             (unsigned)now);
    const uint32_t queuedAt = millis();
    if (floodDue)
        nextJenCoreFloodAdvertMs = queuedAt + JENCORE_FLOOD_ADVERT_INTERVAL_MS;
    nextJenCoreLocalAdvertMs = queuedAt + JENCORE_LOCAL_ADVERT_INTERVAL_MS;
    wakeJenCoreRelay();
    return JENCORE_ADVERT_NUDGE_MS;
}

bool RadioLibInterface::isMeshtasticOnOurChannels(size_t length)
{
    // Meshtastic first: a frame that parses as JenCore may still be a Meshtastic packet between two
    // other nodes. Only the trial decrypt can tell, so it runs on these ambiguous frames alone.
    if (length <= sizeof(PacketHeader))
        return false;
    const PacketHeader &h = radioBuffer.header;
    return decodesOnOurChannels(h.from, h.id, h.channel, radioBuffer.payload, length - sizeof(PacketHeader));
}

JenCoreRelayThread::JenCoreRelayThread(RadioLibInterface *radio) : OSThread("JenCoreRelay"), radio(radio)
{
    disable();
}

void JenCoreRelayThread::wakeIn(uint32_t ms)
{
    enabled = true;
    setIntervalFromNow(ms);
}

int32_t JenCoreRelayThread::runOnce()
{
    return radio->jencoreRelayTick();
}

void RadioLibInterface::wakeJenCoreRelay()
{
    if (!jencoreRelayThread)
        return;
#if MESHTASTIC_ENABLE_RNODE
    if (rnodeTxCount > 0) {
        jencoreRelayThread->wakeIn(1);
        return;
    }
#endif
    if (!jencoreRepeater.hasPendingFrame())
        return;
    const uint32_t dueIn = jencoreRepeater.relayDueInMs();
    // A frame that is already due is being retried on its own backoff: pulling that forward would defeat it.
    if (dueIn == 0 && jencoreRelayThread->isArmed())
        return;
    jencoreRelayThread->wakeIn(dueIn + 1);
}

uint32_t RadioLibInterface::jencoreRelayTick()
{
    maybeSendJenCoreFrame();
#if MESHTASTIC_ENABLE_RNODE
    maybeSendRNodeFrame();
#endif

    if (!jencoreRepeater.hasPendingFrame()
#if MESHTASTIC_ENABLE_RNODE
        && rnodeTxCount == 0
#endif
    )
        return jencoreRelayThread->disable();
    // On air now: the TX-done event wakes us again; this only covers a TX-done that never comes.
    if (isSending())
        return JENCORE_TX_POLL_MS;
    // Due but deferred (radio or channel busy): come back after a random backoff. Otherwise at its deadline.
    return jencoreRepeater.isRelayDue() ? jencoreRepeater.busyBackoffMs(slotTimeMsec) : jencoreRepeater.relayDueInMs() + 1;
}

void RadioLibInterface::maybeSendJenCoreFrame()
{
    if (isSending() || !jencoreRepeater.hasPendingFrame() || !jencoreRepeater.isRelayDue()) {
        return;
    }
    // A deferred frame is retried on a backoff; log the reason at most once a second.
    static uint32_t lastDeferLogMs = 0;
    const bool logDefer = !Throttle::isWithinTimespanMs(lastDeferLogMs, 1000);
    // MeshBeacon moves the radio to another preset for its packet: a JenCore frame sent now would go out on it.
    if (txQueue.getFront() && RadioTxHooks::holdsRadio(txQueue.getFront())) {
        if (logDefer) {
            LOG_DEBUG("JenCore TX deferred: radio held for a beacon");
            lastDeferLogMs = millis();
        }
        return;
    }
    // A permanent refusal would otherwise retry the same frame forever.
    if (!jencoreTxAllowed()) {
        jencoreRepeater.clearPendingFrame();
        jencoreRepeater.txDropped++;
        return;
    }
    uint8_t frame[JenCoreRepeater::MAX_FRAME_LEN];
    size_t len = jencoreRepeater.takePendingFrame(frame, sizeof(frame)); // checks an ADVERT's signature, radio listening
    if (len == 0) {
        return;
    }
    // Same listen-before-talk as the Meshtastic TX path. A busy radio or channel leaves the frame
    // queued (takePendingFrame copies, not clears) for the next jencoreRelayTick().
    const bool own = jencoreRepeater.pendingIsOwn();
    if (!isReceiving || !canSendImmediately()) {
        jencoreRepeater.deferRadio++;
        if (logDefer) {
            LOG_DEBUG("JenCore TX deferred: radio busy");
            lastDeferLogMs = millis();
        }
        return;
    }
    if (isChannelActive()) {
        jencoreRepeater.deferChannel++;
        if (logDefer) {
            LOG_DEBUG("JenCore TX deferred: channel busy");
            lastDeferLogMs = millis();
        }
        startReceive();
        return;
    }
    // The scan can dispatch a pending radio event (setStandby) that starts a Meshtastic TX: leave that one alone.
    if (isSending())
        return;
    rawSendingType = RAW_SEND_JENCORE;
    if (!sendRawFrame(frame, len)) {
        rawSendingType = RAW_SEND_NONE;
        startReceive(); // the channel scan left the radio in standby
        return;
    }
    LOG_INFO("JenCore TX %s: len=%u header=0x%02x path_len=0x%02x", own ? "own" : "relay", (unsigned)len, frame[0], frame[1]);
    if (!own)
        jencoreRepeater.markRelayed();
    jencoreRepeater.clearPendingFrame();
}

#if MESHTASTIC_ENABLE_RNODE
bool RadioLibInterface::queueRNodeFrame(const uint8_t *bytes, size_t len)
{
    if (!bytes || len == 0 || len > MAX_LORA_PAYLOAD_LEN)
        return false;
    if (disabled || !config.lora.tx_enabled)
        return false;
    if (rnodeTxCount >= RNODE_TX_QUEUE_SIZE)
        return false;

    memcpy(rnodeTxQueue[rnodeTxHead].buf, bytes, len);
    rnodeTxQueue[rnodeTxHead].len = len;
    rnodeTxHead = (rnodeTxHead + 1) % RNODE_TX_QUEUE_SIZE;
    rnodeTxCount++;

    wakeJenCoreRelay();
    return true;
}

void RadioLibInterface::maybeSendRNodeFrame()
{
    if (rnodeTxCount == 0 || isSending() || !isReceiving || !canSendImmediately())
        return;
    if (isChannelActive()) {
        startReceive();
        return;
    }

    RNodeTxEntry &entry = rnodeTxQueue[rnodeTxTail];
    rawSendingType = RAW_SEND_RNODE;
    if (sendRawFrame(entry.buf, entry.len)) {
        LOG_INFO("RNode TX: len=%u", (unsigned)entry.len);
        rnodeTxTail = (rnodeTxTail + 1) % RNODE_TX_QUEUE_SIZE;
        rnodeTxCount--;
    } else {
        rawSendingType = RAW_SEND_NONE;
        startReceive();
    }
}
#endif
#else
void RadioLibInterface::maybeSendJenCoreFrame() {}
size_t RadioLibInterface::formatContentionStats(char *, size_t)
{
    return 0;
}
#endif

void RadioLibInterface::handleTransmitInterrupt()
{
    // This can be null if we forced the device to enter standby mode.  In that case
    // ignore the transmit interrupt
    if (sendingPacket)
        completeSending();
    else if (isRawSending())
        completeRawSend();
    powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn); // But our transmitter is definitely off now
}

void RadioLibInterface::completeSending()
{
    // We are careful to clear sending packet before calling printPacket because
    // that can take a long time
    auto p = sendingPacket;
    sendingPacket = NULL;
#ifdef LED_LORA
    digitalWrite(LED_LORA, LED_STATE_OFF);
#endif

    if (p) {
        // Packet has been sent, count it toward our TX airtime utilization.
        uint32_t xmitMsec = getPacketTime(p);
        airTime->logAirtime(TX_LOG, xmitMsec);

        txGood++;
        if (!isFromUs(p))
            txRelay++;
        printPacket("Completed sending", p);
        // Keep this inside `if (p)`: completeSending() also runs on every setStandby(), where a hook
        // undoing its own pre-TX switch would recurse back through reconfigure().
        RadioTxHooks::packetReleased(this, p);

        // We are done sending that packet, release it
        packetPool.release(p);
    }
}

void RadioLibInterface::handleReceiveInterrupt()
{
    PROF_SCOPE(RX_HANDLE);
    // when this is called, we should be in receive mode - if we are not, just jump out instead of bombing. Possible Race
    // Condition?
    if (!isReceiving) {
        LOG_ERROR("handleReceiveInterrupt called while not in rx mode");
        return;
    }

    isReceiving = false;

    // read the number of actually received bytes
    size_t length = iface->getPacketLength();

    // Some drivers report this as a 16 bit value, so a bad readback can overrun radioBuffer in readData()
    if (length > sizeof(radioBuffer)) {
        LOG_ERROR("Ignore rx packet, bad length %u", (unsigned int)length);
        rxBad++;
        return;
    }

    uint32_t rxMsec = getPacketTime(length, true);

#ifndef DISABLE_WELCOME_UNSET
    if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
        LOG_WARN("lora rx disabled: Region unset");
        airTime->logAirtime(RX_ALL_LOG, rxMsec);
        return;
    }
#endif

    int state = readFrame(length);
    if (state == RX_FRAME_PARTIAL)
        return;
#if ARCH_PORTDUINO
    if (portduino_config.logoutputlevel == level_trace) {
        printBytes("Raw incoming packet: ", (uint8_t *)&radioBuffer, length);
    }
#endif
    if (state != RADIOLIB_ERR_NONE) {
        // Log PacketHeader similar to RadioInterface::printPacket so we can try to match RX errors to other packets in the logs.
        LOG_ERROR("Ignore rx packet, error=%d (maybe id=0x%08x fr=0x%08x to=0x%08x flags=0x%02x rxSNR=%g rxRSSI=%i "
                  "nextHop=0x%x relay=0x%x)",
                  state, radioBuffer.header.id, radioBuffer.header.from, radioBuffer.header.to, radioBuffer.header.flags,
                  iface->getSNR(), lround(iface->getRSSI()), radioBuffer.header.next_hop, radioBuffer.header.relay_node);
        rxBad++;

        airTime->logAirtime(RX_ALL_LOG, rxMsec);

    } else {
#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
        // Experimental JenCore interop: a JenCore frame has no Meshtastic PacketHeader, so it
        // must be intercepted before the PacketHeader interpretation below. When enabled, such
        // frames are logged and (optionally) flood-relayed, and never enter the Meshtastic router.
        JenCoreRepeater::FrameInfo mcInfo;
        if (jencoreRepeater.looksLikeJenCoreFrame((const uint8_t *)&radioBuffer, length, nodeDB->getNodeNum(), &mcInfo) &&
            !isMeshtasticOnOurChannels(length)) {
            LOG_INFO("JenCore frame: len=%u route=%u type=0x%02x path=%u/%u payload=%u snr=%g rssi=%d", (unsigned)length,
                     mcInfo.route_type, mcInfo.payload_type, mcInfo.path_hash_count, mcInfo.path_hash_size, mcInfo.payload_len,
                     iface->getSNR(), (int)lround(iface->getRSSI()));
#if MESHTASTIC_ENABLE_JENCORE_COMPANION
            jencoreCompanion.handleFrame((const uint8_t *)&radioBuffer, length, iface->getSNR(),
                                          (int32_t)lround(iface->getRSSI()));
#endif
            jencoreRepeater.handleReceivedFrame((const uint8_t *)&radioBuffer, length, iface->getSNR(),
                                                 getPacketTime(length, false));
            wakeJenCoreRelay(); // when the adaptive relay delay elapses
            airTime->logAirtime(RX_LOG, rxMsec);
            jencoreRepeater.rxAirtimeMs += rxMsec;
            return; // do not deliver to the Meshtastic router
        }
        meshtasticRxAirtimeMs += rxMsec;
#endif
#if MESHTASTIC_ENABLE_RNODE
        if (rnodeEngine.getMode() != RNODE_MODE_OFF) {
            if (rnodeEngine.handleReceivedRadioPacket((const uint8_t *)&radioBuffer, length, iface->getSNR(),
                                                      (int32_t)lround(iface->getRSSI()))) {
                airTime->logAirtime(RX_LOG, rxMsec);
                return; // Handled by Reticulum RNode engine, do not deliver to Meshtastic router
            }
        }
#endif
        // Skip the 4 headers that are at the beginning of the rxBuf
        int32_t payloadLen = length - sizeof(PacketHeader);

        // check for short packets
        if (payloadLen < 0) {
            LOG_WARN("Ignore received packet too short");
            rxBad++;
            airTime->logAirtime(RX_ALL_LOG, rxMsec);
        } else {
            rxGood++;
            // altered packet with "from == 0" can do Remote Node Administration without permission
            if (radioBuffer.header.from == 0) {
                LOG_WARN("Ignore received packet without sender");
                return;
            }

            // Note: we deliver _all_ packets to our router (i.e. our interface is intentionally promiscuous).
            // This allows the router and other apps on our node to sniff packets (usually routing) between other
            // nodes.
            meshtastic_MeshPacket *mp = packetPool.allocZeroed();
            if (!mp) {
                airTime->logAirtime(RX_LOG, rxMsec);
                return;
            }

            // Keep the assigned fields in sync with src/mqtt/MQTT.cpp:onReceiveProto
            mp->from = radioBuffer.header.from;
            mp->to = radioBuffer.header.to;
            mp->id = radioBuffer.header.id;
            mp->channel = radioBuffer.header.channel;
            assert(HOP_MAX <= PACKET_FLAGS_HOP_LIMIT_MASK); // If hopmax changes, carefully check this code
            mp->hop_limit = radioBuffer.header.flags & PACKET_FLAGS_HOP_LIMIT_MASK;
            mp->hop_start = (radioBuffer.header.flags & PACKET_FLAGS_HOP_START_MASK) >> PACKET_FLAGS_HOP_START_SHIFT;
            mp->want_ack = !!(radioBuffer.header.flags & PACKET_FLAGS_WANT_ACK_MASK);
            mp->via_mqtt = !!(radioBuffer.header.flags & PACKET_FLAGS_VIA_MQTT_MASK);
            // If hop_start is not set, next_hop and relay_node are invalid (firmware <2.3)
            mp->next_hop = mp->hop_start == 0 ? NO_NEXT_HOP_PREFERENCE : radioBuffer.header.next_hop;
            mp->relay_node = mp->hop_start == 0 ? NO_RELAY_NODE : radioBuffer.header.relay_node;

            addReceiveMetadata(mp);

            mp->which_payload_variant =
                meshtastic_MeshPacket_encrypted_tag; // Mark that the payload is still encrypted at this point
            assert(((uint32_t)payloadLen) <= sizeof(mp->encrypted.bytes));
            memcpy(mp->encrypted.bytes, radioBuffer.payload, payloadLen);
            mp->encrypted.size = payloadLen;

            printPacket("Lora RX", mp);

#ifdef LED_LORA
            loraRxPacketObservable.notifyObservers(mp->from);
#endif

            airTime->logAirtime(RX_LOG, rxMsec);

            deliverToReceiver(mp);
        }
    }
}

void RadioLibInterface::startReceive()
{
    isReceiving = true;
    // Drivers only reach here once the chip actually accepted the RX start, so the radio is alive again.
    // This is the sole place the recovery ladder is cleared - nothing short of an armed RX counts as fixed.
    rxOffline = false;
    chipRecoveryFailures = 0;
    powerMon->setState(meshtastic_PowerMon_State_Lora_RXOn);
#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
    if (!jencoreAdvertThread) {
        // The zero-hop advert for direct neighbours goes out shortly after boot, the first flood a few minutes later.
        nextJenCoreFloodAdvertMs = millis() + JENCORE_FLOOD_ADVERT_FIRST_MS;
        nextJenCoreLocalAdvertMs = millis() + JENCORE_ADVERT_FIRST_MS;
        jencoreAdvertThread = new JenCoreAdvertThread(this);
        jencoreRelayThread = new JenCoreRelayThread(this);
    }
#endif
#if MESHTASTIC_ENABLE_RNODE_TCP && HAS_WIFI
    if (!rnodeTcpServer) {
        rnodeTcpServer = new RNodeTcpServer();
        rnodeTcpServer->init();
    }
#endif
}

void RadioLibInterface::pollMissedIrqs()
{
    // RadioLibInterface::enableInterrupt uses EDGE-TRIGGERED interrupts. Poll as a backup to catch missed edges.
    if (isReceiving) {
        checkRxDoneIrqFlag();
    }
    if (isSending()) { // raw (JenCore) frames too, or a missed TX_DONE latches rawSending forever
        checkTxDoneIrqFlag();
    }
}

void RadioLibInterface::resetAGC()
{
    // Base implementation: no-op. Override in chip-specific subclasses.
}

void RadioLibInterface::sampleChipState()
{
    // Internal throttle: NexusModule::runOnce() also fires early on activity
    // (setIntervalFromNow), so keep the effective sample rate at ~1 Hz regardless.
    if (Throttle::isWithinTimespanMs(lastChipStateMs, CHIP_STATE_SAMPLE_INTERVAL_MS))
        return;
    lastChipStateMs = Time::getMillis();

    uint8_t state;
    if (sendingPacket) {
        state = CHIP_TX; // software knows TX exactly; don't touch SPI mid-transmit
    } else if (rxOffline) {
        state = CHIP_RXOFF; // RX re-arm failing - chip state can't be trusted, count separately
    } else {
        state = readChipState();
        if (state >= CHIP_STATE_COUNT)
            state = CHIP_UNK;
    }

    chipStateCount[state]++;
    chipStateTotal++;
}

uint8_t RadioLibInterface::readChipState()
{
    return CHIP_UNK; // families without a status read override this
}

void RadioLibInterface::periodicRadioMaintenance()
{
    // Every startReceive() call site is event-driven (RX/TX ISR, the CAD-busy branch, reconfigure), and a
    // radio left with RX off can no longer raise an RX interrupt - on a node with nothing to transmit
    // nothing would ever re-arm it. This periodic tick is that retry; maybeRecoverChipStateLoss() throttles.
    if (rxOffline) {
        LOG_WARN("Radio RX offline, retrying");
        if (maybeRecoverChipStateLoss())
            startReceive();
        return; // a chip just re-inited (or still dead) has no use for an AGC reset this tick
    }

    // Nexus feature DISABLE_AGC_RESET (FEATURES bit2): skip the periodic AGC reset
    // when the operator has it ON. Only gates this call - the RX re-arm ladder above
    // is untouched (a deaf chip must still recover regardless of feature state).
    if (meshtastic::nexusModule &&
        (meshtastic::nexusModule->getNexusFeaturesHex() & meshtastic::FEATURE_DISABLE_AGC_RESET)) {
        return;
    }

    resetAGC();
}

bool RadioLibInterface::maybeRecoverChipStateLoss()
{
    // One attempt per window: the transient resets this recovers from need a single re-init, and a
    // chip that stays dead must not stall the TX/RX paths with a begin() attempt on every call
    if (lastChipRecoveryMs && Throttle::isWithinTimespanMs(lastChipRecoveryMs, 30 * 1000UL)) {
        LOG_DEBUG("Radio recovery suppressed, %us since the last attempt", (Time::getMillis() - lastChipRecoveryMs) / 1000);
        return false;
    }

    // The ladder counts re-arms, not re-inits: only RadioLibInterface::startReceive() clears the count, and
    // only once the chip really accepted RX. Judging the previous attempt here - a throttle window later,
    // after its retry - is what stops a begin() that succeeded while leaving RX dead from crediting itself.
    if (chipRecoveryFailures >= MAX_CHIP_RECOVERY_FAILURES && rebootAtMsec == 0) {
        // Attempts are a throttle window apart, so this is minutes of a provably deaf chip. begin() alone
        // clearly isn't reviving it; reboot to re-run init(), which redoes the power-on sequence it skips.
        LOG_ERROR("Radio still deaf after %u re-inits, rebooting", chipRecoveryFailures);
        rebootAtMsec = Time::timerEndsAtMillis(DEFAULT_REBOOT_SECONDS * 1000);
    }
    chipRecoveryFailures++;

    lastChipRecoveryMs = Time::skipZero(Time::getMillis());
    RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
    LOG_ERROR("Radio chip state lost mid-operation, re-init");
    bool recovered = recoverChipStateLoss();
    LOG_INFO("Radio re-init %s", recovered ? "succeeded" : "failed");
    return recovered;
}

void RadioLibInterface::checkRxDoneIrqFlag()
{
    if (iface->checkIrq(RADIOLIB_IRQ_RX_DONE)) {
        LOG_WARN("caught missed RX_DONE");
        notify(ISR_RX, true);
    }
}

void RadioLibInterface::checkTxDoneIrqFlag()
{
    if (iface->checkIrq(RADIOLIB_IRQ_TX_DONE)) {
        LOG_WARN("caught missed TX_DONE");
        notify(ISR_TX, true);
    }
}

void RadioLibInterface::configHardwareForSend()
{
    powerMon->setState(meshtastic_PowerMon_State_Lora_TXOn);
}

void RadioLibInterface::setStandby()
{
    // neither sending nor receiving
    powerMon->clearState(meshtastic_PowerMon_State_Lora_RXOn);
    powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn);
}

// Nexus port (Nexus-28001, 2026-08-16): read NexusModule::syncword_override (vault v40) and
// apply to the subclass syncWord member. Called from RadioInterface::applyModemConfig() at
// boot AND on every LoRa re-config (including the deferred apply triggered by /nexus
// SETSYNCWORD). syncword_override == 0 leaves the default 0x2b in place.
void RadioLibInterface::applyNexusSyncwordOverride()
{
    if (meshtastic::nexusModule && meshtastic::nexusModule->syncword_override != 0) {
        syncWord = meshtastic::nexusModule->syncword_override;
    }
}

/** start an immediate transmit */
bool RadioLibInterface::startSend(meshtastic_MeshPacket *txp)
{
    /* NOTE: Minimize the actions before startTransmit() to keep the time between
             channel scan and actual transmit as low as possible to avoid collisions. */
    if (disabled || !config.lora.tx_enabled) {
        LOG_WARN("Drop Tx packet: LoRa Tx disabled");
        // Never reaches completeSending(), so any per-packet radio state has to be released here.
        RadioTxHooks::packetReleased(this, txp);
        packetPool.release(txp);
        return false;
    } else {
        configHardwareForSend(); // must be after setStandby

        size_t numbytes = beginSending(txp);

        PROF_DO(prof::markEnd(txp->id, prof::SLOT_TXQ, prof::TX_QUEUE));
        PROF_DO(prof::txStartAt = micros());
        int res = startTransmitFrame(numbytes);
        if (res != RADIOLIB_ERR_NONE) {
            LOG_ERROR("startTransmit failed, error=%d", res);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_RADIO_SPI_BUG);

            // This send failed, but make sure to 'complete' it properly
            completeSending();
            powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn); // Transmitter off now
            startReceive(); // Restart receive mode (because startTransmit failed to put us in xmit mode)
        } else {
            // Must be done AFTER, starting transmit, because startTransmit clears (possibly stale) interrupt pending register
            // bits
            enableInterrupt(isrTxLevel0);
            // unset-sentinel-ok: busyTx/sendingPacket is the armed flag, so 0 is a legal stamp
            lastTxStart = Time::getMillis();
            printPacket("Started Tx", txp);
#ifdef LED_LORA
            digitalWrite(LED_LORA, LED_STATE_ON);
#endif
        }

        return res == RADIOLIB_ERR_NONE;
    }
}

float RadioLibInterface::getInstantaneousRSSI()
{
    return iface->getRSSI();
}
