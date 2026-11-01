#include "RidBleBroadcaster.h"

#include <string.h>

static_assert(RidLocationEncoder::MESSAGE_SIZE == 25, "Location payload must be 25 bytes");
static_assert(RidBasicIdEncoder::MESSAGE_SIZE == 25, "Basic ID payload must be 25 bytes");

bool RidBleLegacyFrame::encode(const uint8_t* message, size_t messageLength,
                               uint8_t counter, uint8_t* out, size_t outSize)
{
    if (out == nullptr || outSize < ADVERTISING_SIZE) return false;
    memset(out, 0, ADVERTISING_SIZE);
    if (message == nullptr || messageLength != MESSAGE_SIZE ||
        (message[0] & 0x0f) != 2 || (message[0] >> 4) > 1) return false;
    const uint8_t prefix[] = {0x1e, 0x16, 0xfa, 0xff, 0x0d};
    memcpy(out, prefix, sizeof(prefix));
    out[5] = counter;
    memcpy(out + 6, message, MESSAGE_SIZE);
    return true;
}

RidBleBroadcaster::RidBleBroadcaster(RidBleTransport& transport,
                                   const RidBasicIdV1& identity, RidBleReporter reporter)
    : transport(transport), identity(identity), reporter(reporter)
{
}

void RidBleBroadcaster::report(RidBleDiagnosticKind kind, RidBleOperation operation,
                              int32_t status, uint32_t nowMs, uint32_t gapMs)
{
    if (reporter != nullptr)
    {
        RidBleDiagnostic d = diagnostics(nowMs);
        d.kind = kind;
        d.operation = operation;
        d.status = status;
        if (kind == RidBleDiagnosticKind::INITIALIZATION || kind == RidBleDiagnosticKind::BASIC_ID_REJECTED)
            d.payloadLength = 0;
        if (kind == RidBleDiagnosticKind::DEADLINE_MISSED) d.gapMs = gapMs;
        if (kind == RidBleDiagnosticKind::LOCATION_AGE_CHANGED) d.locationAge = latestLocationAge;
        reporter(d);
    }
}

RidBleDiagnostic RidBleBroadcaster::diagnostics(uint32_t nowMs) const
{
    RidBleDiagnostic d{};
    d.kind = RidBleDiagnosticKind::ADVERTISING;
    d.operation = RidBleOperation::START;
    d.messageType = currentType;
    d.counter = advertisingData[5];
    d.payloadLength = framePrepared ? sizeof(advertisingData) : 0;
    d.nowMs = nowMs;
    d.gapMs = lastPublicationGapMs;
    d.restartGapAvailable = restartGapAvailable;
    d.restartGapMs = restartGapMs;
    d.maxRestartGapMs = maxRestartGapMs;
    d.switchLatencyMs = switchLatencyMs;
    d.callbackDelayMs = callbackDelayMs;
    d.preparedAgeMs = framePrepared ? static_cast<uint32_t>(nowMs - preparedMs) : 0;
    d.advertisingAgeMs = state == State::ADVERTISING || state == State::STOPPING
        ? static_cast<uint32_t>(nowMs - advertisingStartedMs) : 0;
    d.successfulUpdates = successfulUpdates;
    d.failedUpdates = failedUpdates;
    d.advertisingActive = state == State::ADVERTISING || state == State::STOPPING;
    d.locationAge = currentType == RidBleMessageType::LOCATION && framePrepared
        ? currentLocationAge(nowMs) : latestLocationAge;
    return d;
}

RidLocationAgeInfo RidBleBroadcaster::currentLocationAge(uint32_t nowMs) const
{
    RidLocationAgeInfo age = preparedLocationAge;
    const uint32_t elapsed = static_cast<uint32_t>(nowMs - preparedMs);
    if (age.receiveAgeAvailable) age.receiveAgeMs += elapsed;
    if (age.timestampAgeAvailable)
    {
        // Saturate diagnostic addition rather than hiding an old timestamp.
        age.timestampAgeEstimateMs = UINT32_MAX - age.timestampAgeEstimateMs < elapsed
            ? UINT32_MAX : age.timestampAgeEstimateMs + elapsed;
        age.timestampStale = age.timestampStale ||
            age.timestampAgeEstimateMs > RidLocationAgeMonitor::MAX_DYNAMIC_AGE_MS;
    }
    age.gpiSuppressed = age.gpiSuppressed || age.timestampStale || age.clockAmbiguous;
    return age;
}

bool RidBleBroadcaster::begin(uint32_t nowMs)
{
    if (state != State::NOT_STARTED) return false;
    const auto result = RidBasicIdEncoder().encode(identity, basicPacket, sizeof(basicPacket));
    basicIdValid = result == RidBasicIdEncodeResult::SUCCESS;
    if (!basicIdValid)
        report(RidBleDiagnosticKind::BASIC_ID_REJECTED, RidBleOperation::BASIC_VALIDATE,
               static_cast<int32_t>(result), nowMs);
    const int32_t status = transport.begin();
    report(RidBleDiagnosticKind::INITIALIZATION, RidBleOperation::INITIALIZE, status, nowMs);
    state = status == 0 ? State::READY : State::FAULT;
    return status == 0;
}

bool RidBleBroadcaster::healthy() const
{
    return state != State::NOT_STARTED && state != State::FAULT;
}

RidBleOperation RidBleBroadcaster::pendingOperation() const
{
    if (state == State::STOPPING) return RidBleOperation::STOP;
    if (state == State::CONFIGURING) return RidBleOperation::CONFIGURE;
    return RidBleOperation::START;
}

void RidBleBroadcaster::fail(RidBleOperation operation, int32_t status, uint32_t nowMs)
{
    ++failedUpdates;
    state = State::FAULT;
    report(RidBleDiagnosticKind::ERROR, operation, status, nowMs);
    // Stop is best effort if controller state is uncertain. No new frames are
    // submitted after a fault; reset is required. Never wait inside the loop.
    const int32_t stopStatus = transport.stop();
    if (stopStatus != 0 && operation != RidBleOperation::STOP)
        report(RidBleDiagnosticKind::ERROR, RidBleOperation::STOP, stopStatus, nowMs);
}

void RidBleBroadcaster::update(uint32_t nowMs, const DroneStateV1& freshnessCheckedState,
                               const RidLocationAgeInfo& locationAge)
{
    const bool suppressionChanged = latestLocationAge.gpiSuppressed != locationAge.gpiSuppressed;
    latestLocationAge = locationAge;
    if (!healthy()) return;
    if (suppressionChanged)
        report(RidBleDiagnosticKind::LOCATION_AGE_CHANGED, RidBleOperation::FRAME, 0, nowMs);
    const DroneStateV1 outgoingState = RidLocationAgeMonitor::forAdvertising(freshnessCheckedState, locationAge);

    // Configuration (including refreshes) and start share a bounded deadline.
    // Do not accept a late callback and start an old Location after a loop stall.
    if ((state == State::CONFIGURING || state == State::STARTING || state == State::STOPPING) &&
        static_cast<uint32_t>(nowMs - stageStartedMs) >= GAP_OPERATION_TIMEOUT_MS)
    {
        fail(pendingOperation(), OPERATION_TIMEOUT, nowMs);
        return;
    }

    // At most one completion per call; callbacks only enqueue these small events.
    RidBleTransportEvent event;
    if (transport.poll(event))
    {
        if (event.type == RidBleEventType::QUEUE_OVERFLOW)
        {
            fail(RidBleOperation::EVENT_QUEUE, UNEXPECTED_EVENT, nowMs);
            return;
        }
        const bool expected =
            (state == State::CONFIGURING && event.type == RidBleEventType::CONFIGURED) ||
            (state == State::STARTING && event.type == RidBleEventType::STARTED) ||
            (state == State::STOPPING && event.type == RidBleEventType::STOPPED);
        if (!expected || event.status != 0)
        {
            fail(pendingOperation(), expected ? event.status : UNEXPECTED_EVENT, nowMs);
            return;
        }
        const uint32_t completedMs = event.timeAvailable ? event.completedMs : nowMs;
        callbackDelayMs = static_cast<uint32_t>(nowMs - completedMs);
        if (state == State::STOPPING)
        {
            stoppedMs = completedMs;
            stoppedObserved = true;
            state = State::READY;
        }
        else if (state == State::CONFIGURING) state = State::READY_TO_START;
        else
        {
            const size_t index = static_cast<size_t>(currentType);
            const uint32_t gap = static_cast<uint32_t>(completedMs - lastPublishedMs[index]);
            lastPublicationGapMs = published[index] ? gap : 0;
            if (published[index] && gap > (currentType == RidBleMessageType::LOCATION
                                          ? LOCATION_MAX_GAP_MS : BASIC_ID_MAX_GAP_MS))
                report(RidBleDiagnosticKind::DEADLINE_MISSED, RidBleOperation::START, 0, nowMs, gap);
            lastPublishedMs[index] = completedMs;
            published[index] = true;
            nextCounter[index] = static_cast<uint8_t>(nextCounter[index] + 1U);
            ++successfulUpdates;
            advertisingStartedMs = completedMs;
            if (stoppedObserved)
            {
                restartGapAvailable = true;
                restartGapMs = static_cast<uint32_t>(completedMs - stoppedMs);
                if (restartGapMs > maxRestartGapMs) maxRestartGapMs = restartGapMs;
                switchLatencyMs = static_cast<uint32_t>(completedMs - stopRequestedMs);
                stoppedObserved = false;
            }
            nextType = currentType == RidBleMessageType::LOCATION && basicIdValid
                ? RidBleMessageType::BASIC_ID : RidBleMessageType::LOCATION;
            state = State::ADVERTISING;
            report(RidBleDiagnosticKind::ADVERTISING, RidBleOperation::START, 0, nowMs);
        }
    }

    if (state == State::READY)
    {
        // Prefer fresh Location after a delayed loop instead of catching up
        // missed slots with stale cached packets. Healthy operation alternates.
        currentType = nextType;
        if (!basicIdValid || (published[1] &&
            static_cast<uint32_t>(nowMs - lastPublishedMs[1]) >= 750))
            currentType = RidBleMessageType::LOCATION;
        uint8_t packet[25];
        if (currentType == RidBleMessageType::BASIC_ID) memcpy(packet, basicPacket, sizeof(packet));
        else RidLocationEncoder().encode(RidLocationMapper().map(outgoingState), packet);
        if (!RidBleLegacyFrame::encode(packet, sizeof(packet),
                                      nextCounter[static_cast<size_t>(currentType)],
                                      advertisingData, sizeof(advertisingData)))
        {
            fail(RidBleOperation::FRAME, INVALID_FRAME, nowMs);
            return;
        }
        state = State::CONFIGURING;
        preparedMs = nowMs;
        framePrepared = true;
        currentLocationSuppressed = locationAge.gpiSuppressed;
        currentLocationSuppressed = currentLocationSuppressed || !outgoingState.positionValid;
        preparedLocationAge = locationAge;
        stageStartedMs = nowMs;
        const int32_t status = transport.configure(advertisingData, sizeof(advertisingData));
        if (status != 0) fail(RidBleOperation::CONFIGURE, status, nowMs);
    }
    else if (state == State::READY_TO_START)
    {
        if (currentType == RidBleMessageType::LOCATION)
        {
            uint8_t latest[25];
            RidLocationEncoder().encode(RidLocationMapper().map(outgoingState), latest);
            if (memcmp(latest, advertisingData + 6, sizeof(latest)) != 0)
            {
                // Telemetry/freshness changed while GAP configured the packet.
                // Refresh without consuming a counter or extending the deadline.
                memcpy(advertisingData + 6, latest, sizeof(latest));
                preparedMs = nowMs;
                currentLocationSuppressed = locationAge.gpiSuppressed || !outgoingState.positionValid;
                preparedLocationAge = locationAge;
                state = State::CONFIGURING;
                const int32_t status = transport.configure(advertisingData, sizeof(advertisingData));
                if (status != 0) fail(RidBleOperation::CONFIGURE, status, nowMs);
                return;
            }
            preparedMs = nowMs;
            preparedLocationAge = locationAge;
        }
        state = State::STARTING;
        const int32_t status = transport.start();
        if (status != 0) fail(RidBleOperation::START, status, nowMs);
    }
    else if (state == State::ADVERTISING &&
             (static_cast<uint32_t>(nowMs - advertisingStartedMs) >= DWELL_MS ||
              (currentType == RidBleMessageType::LOCATION && !currentLocationSuppressed &&
               (locationAge.gpiSuppressed || !outgoingState.positionValid ||
                currentLocationAge(nowMs).timestampStale ||
                (preparedLocationAge.receiveAgeAvailable && currentLocationAge(nowMs).receiveAgeMs >= 1000)))))
    {
        state = State::STOPPING;
        stageStartedMs = nowMs;
        stopRequestedMs = nowMs;
        const int32_t status = transport.stop();
        if (status != 0) fail(RidBleOperation::STOP, status, nowMs);
    }
}

const char* RidBleBroadcaster::operationName(RidBleOperation operation)
{
    switch (operation)
    {
        case RidBleOperation::INITIALIZE: return "INITIALIZE";
        case RidBleOperation::CONFIGURE: return "CONFIGURE_RAW";
        case RidBleOperation::START: return "START";
        case RidBleOperation::STOP: return "STOP";
        case RidBleOperation::EVENT_QUEUE: return "EVENT_QUEUE";
        case RidBleOperation::BASIC_VALIDATE: return "BASIC_VALIDATE";
        case RidBleOperation::FRAME: return "FRAME";
        default: return "UNKNOWN";
    }
}

const char* RidBleBroadcaster::messageName(RidBleMessageType type)
{
    return type == RidBleMessageType::LOCATION ? "LOCATION" : "BASIC_ID";
}
