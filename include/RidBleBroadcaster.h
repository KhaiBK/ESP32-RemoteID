#pragma once

#include "DroneState.h"
#include "RidBasicIdEncoder.h"
#include "RidLocationEncoder.h"
#include "RidLocationMapper.h"
#include "RidLocationAgeMonitor.h"

enum class RidBleMessageType : uint8_t { BASIC_ID = 0, LOCATION = 1, COUNT = 2 };
enum class RidBleOperation : uint8_t { INITIALIZE, CONFIGURE, START, STOP, EVENT_QUEUE, BASIC_VALIDATE, FRAME };
enum class RidBleEventType : uint8_t { CONFIGURED, STARTED, STOPPED, QUEUE_OVERFLOW };

struct RidBleTransportEvent
{
    RidBleEventType type;
    int32_t status;
    uint32_t completedMs;
    bool timeAvailable;
    RidBleTransportEvent(RidBleEventType type = RidBleEventType::CONFIGURED,
                         int32_t status = 0, uint32_t completedMs = 0, bool timeAvailable = false)
        : type(type), status(status), completedMs(completedMs), timeAvailable(timeAvailable) {}
};

// Accepted requests return zero; completion is delivered separately by poll().
// Transport callbacks must not access DroneState or perform encoding/printing.
class RidBleTransport
{
public:
    virtual ~RidBleTransport() = default;
    virtual int32_t begin() = 0;
    virtual int32_t configure(const uint8_t* data, size_t length) = 0;
    virtual int32_t start() = 0;
    virtual int32_t stop() = 0;
    virtual bool poll(RidBleTransportEvent& event) = 0;
};

class RidBleLegacyFrame
{
public:
    static constexpr size_t MESSAGE_SIZE = 25;
    static constexpr size_t ADVERTISING_SIZE = 31;
    static bool encode(const uint8_t* message, size_t messageLength, uint8_t counter,
                       uint8_t* out, size_t outSize);
};

enum class RidBleDiagnosticKind : uint8_t { INITIALIZATION, ADVERTISING, BASIC_ID_REJECTED, ERROR, DEADLINE_MISSED, LOCATION_AGE_CHANGED };

struct RidBleDiagnostic
{
    RidBleDiagnosticKind kind;
    RidBleOperation operation;
    RidBleMessageType messageType;
    uint8_t counter;
    size_t payloadLength;
    int32_t status;
    uint32_t nowMs;
    uint32_t gapMs;
    bool restartGapAvailable;
    uint32_t restartGapMs;
    uint32_t maxRestartGapMs;
    uint32_t switchLatencyMs;
    uint32_t callbackDelayMs;
    uint32_t preparedAgeMs;
    uint32_t advertisingAgeMs;
    uint32_t successfulUpdates;
    uint32_t failedUpdates;
    bool advertisingActive;
    RidLocationAgeInfo locationAge;
};

using RidBleReporter = void (*)(const RidBleDiagnostic& diagnostic);

// Platform-independent asynchronous broadcaster, driven from the Arduino loop.
class RidBleBroadcaster
{
public:
    static constexpr uint32_t DWELL_MS = 250;
    static constexpr uint32_t GAP_OPERATION_TIMEOUT_MS = 1000;
    static constexpr uint32_t LOCATION_MAX_GAP_MS = 1000;
    static constexpr uint32_t BASIC_ID_MAX_GAP_MS = 3000;
    static constexpr int32_t OPERATION_TIMEOUT = -1;
    static constexpr int32_t UNEXPECTED_EVENT = -2;
    static constexpr int32_t INVALID_FRAME = -3;

    RidBleBroadcaster(RidBleTransport& transport, const RidBasicIdV1& identity,
                      RidBleReporter reporter = nullptr);
    bool begin(uint32_t nowMs);
    // Caller must apply freshness before every invocation. No delay or waits.
    void update(uint32_t nowMs, const DroneStateV1& freshnessCheckedState,
                const RidLocationAgeInfo& locationAge = RidLocationAgeInfo());
    bool healthy() const;
    bool basicIdAvailable() const { return basicIdValid; }
    RidBleDiagnostic diagnostics(uint32_t nowMs) const;
    const uint8_t* currentAdvertisingData() const { return advertisingData; }
    static const char* operationName(RidBleOperation operation);
    static const char* messageName(RidBleMessageType type);

private:
    enum class State : uint8_t { NOT_STARTED, READY, CONFIGURING, READY_TO_START, STARTING, ADVERTISING, STOPPING, FAULT };
    RidBleTransport& transport;
    RidBasicIdV1 identity;
    RidBleReporter reporter;
    State state = State::NOT_STARTED;
    bool basicIdValid = false;
    uint8_t basicPacket[25] = {};
    uint8_t advertisingData[31] = {};
    static constexpr size_t MESSAGE_TYPE_COUNT = static_cast<size_t>(RidBleMessageType::COUNT);
    uint8_t nextCounter[MESSAGE_TYPE_COUNT] = {};
    bool published[MESSAGE_TYPE_COUNT] = {};
    uint32_t lastPublishedMs[MESSAGE_TYPE_COUNT] = {};
    uint32_t stageStartedMs = 0;
    uint32_t advertisingStartedMs = 0;
    uint32_t preparedMs = 0;
    bool framePrepared = false;
    bool currentLocationSuppressed = false;
    RidLocationAgeInfo latestLocationAge;
    RidLocationAgeInfo preparedLocationAge;
    bool stoppedObserved = false;
    uint32_t stoppedMs = 0;
    uint32_t stopRequestedMs = 0;
    bool restartGapAvailable = false;
    uint32_t restartGapMs = 0, maxRestartGapMs = 0, switchLatencyMs = 0;
    uint32_t callbackDelayMs = 0, lastPublicationGapMs = 0;
    uint32_t successfulUpdates = 0, failedUpdates = 0;
    RidBleMessageType nextType = RidBleMessageType::LOCATION;
    RidBleMessageType currentType = RidBleMessageType::LOCATION;

    void report(RidBleDiagnosticKind kind, RidBleOperation operation, int32_t status,
                uint32_t nowMs, uint32_t gapMs = 0);
    void fail(RidBleOperation operation, int32_t status, uint32_t nowMs);
    RidBleOperation pendingOperation() const;
    RidLocationAgeInfo currentLocationAge(uint32_t nowMs) const;
};
