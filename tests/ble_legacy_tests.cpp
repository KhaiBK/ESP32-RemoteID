#include "RidBleBroadcaster.h"

#include <array>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>

namespace
{
unsigned assertions = 0, failures = 0;
#define CHECK(condition) do { ++assertions; if (!(condition)) { ++failures; \
    std::printf("FAIL line %d: %s\n", __LINE__, #condition); } } while (0)

using Frame = std::array<uint8_t, 31>;
std::vector<RidBleDiagnostic> diagnostics;
void report(const RidBleDiagnostic& d) { diagnostics.push_back(d); }

// A DEVELOPMENT-ONLY syntax fixture: no manufacturer assignment is claimed.
RidBasicIdV1 devIdentity()
{
    RidBasicIdV1 id;
    id.idType = RidBasicIdType::SERIAL_NUMBER;
    id.uaType = RidUaType::HELICOPTER_OR_MULTIROTOR;
    std::strcpy(id.uasId, "TEST4DEV1");
    return id;
}

DroneStateV1 exampleState()
{
    DroneStateV1 s;
    s.operationalStatus = RidOperationalStatus::AIRBORNE;
    s.operationalStatusValid = true;
    s.latitudeDeg = -35.3633513;
    s.longitudeDeg = 149.1652409;
    s.positionValid = true;
    s.heightM = 10;
    s.heightValid = true;
    return s;
}

class FakeGap : public RidBleTransport
{
public:
    bool complete = true, active = false;
    int32_t initError = 0, configError = 0, startError = 0, stopError = 0;
    int32_t configCompletion = 0, startCompletion = 0, stopCompletion = 0;
    unsigned initCalls = 0, stopCalls = 0;
    Frame configured{};
    std::vector<Frame> configurations, starts;
    std::deque<RidBleTransportEvent> events;
    std::vector<char> calls;

    int32_t begin() override { ++initCalls; return initError; }
    int32_t configure(const uint8_t* data, size_t length) override
    {
        CHECK(!active); // Never update a running controller.
        CHECK(length == 31);
        calls.push_back('C');
        if (configError) return configError;
        std::memcpy(configured.data(), data, length);
        configurations.push_back(configured);
        if (complete) events.push_back({RidBleEventType::CONFIGURED, configCompletion});
        return 0;
    }
    int32_t start() override
    {
        CHECK(!active);
        calls.push_back('A');
        if (startError) return startError;
        if (complete) events.push_back({RidBleEventType::STARTED, startCompletion});
        return 0;
    }
    int32_t stop() override
    {
        ++stopCalls;
        calls.push_back('S');
        if (stopError) return stopError;
        if (complete) events.push_back({RidBleEventType::STOPPED, stopCompletion});
        return 0;
    }
    bool poll(RidBleTransportEvent& event) override
    {
        if (events.empty()) return false;
        event = events.front();
        events.pop_front();
        if (event.status == 0 && event.type == RidBleEventType::STARTED)
        {
            active = true;
            starts.push_back(configured);
        }
        if (event.status == 0 && event.type == RidBleEventType::STOPPED) active = false;
        return true;
    }
};

void run(RidBleBroadcaster& b, const DroneStateV1& state, uint32_t origin,
         uint32_t duration, uint32_t step = 1)
{
    for (uint32_t dt = 0; dt < duration; dt += step) b.update(origin + dt, state);
}

bool sawError(RidBleOperation op, int32_t code)
{
    for (const auto& d : diagnostics)
        if (d.kind == RidBleDiagnosticKind::ERROR && d.operation == op && d.status == code) return true;
    return false;
}

void framing()
{
    // Independent golden layout matching OpenDroneID transmitter-linux's
    // legacy Service Data AD structure (not its additional HCI length byte).
    const uint8_t packet[25] = {
        0x02, 0x12, 'T', 'E', 'S', 'T', '4', 'D', 'E', 'V', '1', 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    const uint8_t expected[31] = {
        0x1e, 0x16, 0xfa, 0xff, 0x0d, 0xa5,
        0x02, 0x12, 'T', 'E', 'S', 'T', '4', 'D', 'E', 'V', '1', 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    uint8_t guarded[33];
    std::memset(guarded, 0xcc, sizeof(guarded));
    CHECK(RidBleLegacyFrame::encode(packet, 25, 0xa5, guarded + 1, 31));
    CHECK(std::memcmp(guarded + 1, expected, 31) == 0);
    CHECK(guarded[0] == 0xcc && guarded[32] == 0xcc);
    CHECK(guarded[1] + 1 == 31); // Exactly one AD structure, no Flags/name.
    CHECK(guarded[3] == 0xfa && guarded[4] == 0xff); // UUID FFFA little-endian.
    CHECK(guarded[5] == 0x0d);
    CHECK(!RidBleLegacyFrame::encode(packet, 25, 1, nullptr, 31));
    for (size_t size = 0; size < 31; ++size)
    {
        std::memset(guarded, 0xcc, sizeof(guarded));
        CHECK(!RidBleLegacyFrame::encode(packet, 25, 1, guarded, size));
        for (uint8_t value : guarded) CHECK(value == 0xcc);
    }
    CHECK(!RidBleLegacyFrame::encode(nullptr, 25, 0, guarded, 31));
    for (unsigned size : {0U, 24U, 26U})
        CHECK(!RidBleLegacyFrame::encode(packet, size, 0, guarded, 31));
    uint8_t modified[25];
    std::memcpy(modified, packet, 25);
    modified[0] = 0x12;
    CHECK(RidBleLegacyFrame::encode(modified, 25, 255, guarded, 31));
    CHECK(std::memcmp(guarded + 6, modified, 25) == 0);
    for (uint8_t header : {0x01, 0x13, 0x22, 0xf2})
    {
        modified[0] = header;
        CHECK(!RidBleLegacyFrame::encode(modified, 25, 0, guarded, 31));
        for (unsigned i = 0; i < 31; ++i) CHECK(guarded[i] == 0);
    }
    std::puts("PASS framing: full golden vector, UUID/application, exact length, payload and buffer guards");
}

void schedulingAndCounters()
{
    diagnostics.clear();
    FakeGap gap;
    RidBleBroadcaster b(gap, devIdentity(), report);
    CHECK(!b.healthy());
    CHECK(b.begin(0));
    CHECK(!b.begin(0));
    CHECK(gap.initCalls == 1);
    CHECK(b.basicIdAvailable());
    run(b, exampleState(), 0, 290000, 10);
    CHECK(b.healthy());
    CHECK(gap.starts.size() > 1024); // Both counters wrap at least twice.
    unsigned perType[2] = {};
    for (size_t i = 0; i < gap.starts.size(); ++i)
    {
        const auto& frame = gap.starts[i];
        const unsigned type = frame[6] >> 4;
        CHECK(type == (i % 2 == 0 ? 1U : 0U));
        CHECK(frame[5] == static_cast<uint8_t>(perType[type]++));
        CHECK(frame[0] == 30 && frame[1] == 0x16 && frame[4] == 0x0d);
    }
    CHECK(perType[0] > 512 && perType[1] > 512);
    for (size_t i = 0; i < gap.calls.size(); ++i)
        CHECK(gap.calls[i] == "CAS"[i % 3]);
    bool seen[2] = {};
    uint32_t previous[2] = {};
    unsigned advertised = 0;
    for (const auto& d : diagnostics)
    {
        CHECK(d.kind != RidBleDiagnosticKind::ERROR);
        CHECK(d.kind != RidBleDiagnosticKind::DEADLINE_MISSED);
        if (d.kind != RidBleDiagnosticKind::ADVERTISING) continue;
        const unsigned type = static_cast<unsigned>(d.messageType);
        CHECK(d.payloadLength == 31);
        CHECK(d.counter == gap.starts[advertised++][5]);
        if (seen[type]) CHECK(d.nowMs - previous[type] <= (type == 1 ? 1000U : 3000U));
        seen[type] = true;
        previous[type] = d.nowMs;
    }
    CHECK(advertised == gap.starts.size());
    std::puts("PASS scheduling: alternation, callback ordering, publication deadlines, per-type counter wraparound");
}

void dwellAndLatestState()
{
    diagnostics.clear();
    FakeGap gap;
    RidBasicIdV1 invalid; // Unconfigured: Location must still be sent.
    RidBleBroadcaster b(gap, invalid, report);
    CHECK(b.begin(0));
    CHECK(!b.basicIdAvailable());
    auto s = exampleState();
    b.update(0, s); // Configure requested, not yet started.
    CHECK(gap.starts.empty());
    b.update(100, s); // Configure acknowledgement -> start requested.
    CHECK(gap.starts.empty());
    b.update(200, s); // Start acknowledgement: dwell starts now.
    CHECK(gap.starts.size() == 1);
    b.update(449, s);
    CHECK(gap.stopCalls == 0);
    b.update(450, s);
    CHECK(gap.stopCalls == 1 && gap.configurations.size() == 1);
    // Freshness-invalidated state must replace the previously valid packet.
    s.positionValid = false;
    s.heightValid = false;
    s.operationalStatusValid = false;
    b.update(451, s); // STOP acknowledgement, generate from current state.
    CHECK(gap.configurations.size() == 2);
    uint8_t expected[25];
    RidLocationEncoder().encode(RidLocationMapper().map(s), expected);
    CHECK(std::memcmp(gap.configurations.back().data() + 6, expected, 25) == 0);
    CHECK(std::memcmp(gap.configurations[0].data() + 6, expected, 25) != 0);
    run(b, s, 452, 10000);
    CHECK(b.healthy());
    for (const auto& frame : gap.starts) CHECK((frame[6] >> 4) == 1);
    bool rejected = false;
    for (const auto& d : diagnostics)
        if (d.kind == RidBleDiagnosticKind::BASIC_ID_REJECTED) rejected = true;
    CHECK(rejected);
    // A nonempty malformed identity must be suppressed as well.
    FakeGap malformedGap;
    auto malformed = devIdentity();
    malformed.uasId[4] = '0';
    RidBleBroadcaster malformedB(malformedGap, malformed);
    CHECK(malformedB.begin(0));
    CHECK(!malformedB.basicIdAvailable());
    run(malformedB, s, 0, 2000);
    for (const auto& frame : malformedGap.starts) CHECK(frame[6] == 0x12);
    std::puts("PASS freshness handoff, acknowledged dwell, unconfigured/malformed Basic ID suppression");
}

void rolloverAndDelayedLoop()
{
    diagnostics.clear();
    FakeGap gap;
    RidBleBroadcaster b(gap, devIdentity(), report);
    const uint32_t origin = UINT32_MAX - 100;
    CHECK(b.begin(origin));
    run(b, exampleState(), origin, 5000);
    CHECK(b.healthy() && gap.starts.size() > 10);
    for (const auto& d : diagnostics) CHECK(d.kind != RidBleDiagnosticKind::ERROR);

    diagnostics.clear();
    FakeGap delayed;
    RidBleBroadcaster late(delayed, devIdentity(), report);
    CHECK(late.begin(0));
    run(late, exampleState(), 0, 3); // Location started at t=2.
    late.update(2000, exampleState()); // Request stop after a long main-loop stall.
    late.update(2001, exampleState()); // Prioritize a current Location, no burst catch-up.
    late.update(2002, exampleState());
    late.update(2003, exampleState());
    CHECK(delayed.starts.size() == 2);
    CHECK(delayed.starts.back()[6] == 0x12);
    bool deadline = false;
    for (const auto& d : diagnostics)
        if (d.kind == RidBleDiagnosticKind::DEADLINE_MISSED && d.gapMs == 2001) deadline = true;
    CHECK(deadline);
    run(late, exampleState(), 2004, 1000);
    CHECK(delayed.starts[2][6] == 0x02); // Alternation resumes after priority refresh.
    std::puts("PASS millis rollover and delayed-loop Location priority/deadline diagnostic");
}

void delayedConfiguration()
{
    diagnostics.clear();
    FakeGap gap;
    RidBleBroadcaster b(gap, devIdentity(), report);
    auto s = exampleState();
    CHECK(b.begin(0));
    b.update(0, s);
    s.positionValid = false;
    s.operationalStatusValid = false;
    b.update(800, s); // Old configure acknowledgement: refresh, do not start.
    CHECK(gap.starts.empty());
    CHECK(gap.configurations.size() == 2);
    CHECK(gap.calls.back() == 'C');
    CHECK(gap.configurations[0][5] == gap.configurations[1][5]);
    uint8_t expected[25];
    RidLocationEncoder().encode(RidLocationMapper().map(s), expected);
    CHECK(std::memcmp(gap.configurations.back().data() + 6, expected, 25) == 0);
    b.update(801, s);
    b.update(802, s);
    CHECK(gap.starts.size() == 1 && b.healthy());
    CHECK(std::memcmp(gap.starts.back().data() + 6, expected, 25) == 0);

    // A queued callback observed after the deadline cannot start an old frame.
    diagnostics.clear();
    FakeGap lateGap;
    RidBleBroadcaster late(lateGap, devIdentity(), report);
    CHECK(late.begin(0));
    late.update(0, s);
    late.update(1000, s);
    CHECK(!late.healthy() && lateGap.starts.empty());
    CHECK(sawError(RidBleOperation::CONFIGURE, RidBleBroadcaster::OPERATION_TIMEOUT));

    // Repeated telemetry changes cannot extend configuration indefinitely.
    diagnostics.clear();
    FakeGap changingGap;
    RidBleBroadcaster changing(changingGap, devIdentity(), report);
    CHECK(changing.begin(0));
    changing.update(0, s);
    for (uint32_t now = 100; now < 1000; now += 100)
    {
        s.heightM += 1;
        changing.update(now, s);
        CHECK(changing.healthy() && changingGap.starts.empty());
    }
    changing.update(1000, s);
    CHECK(!changing.healthy());
    CHECK(sawError(RidBleOperation::CONFIGURE, RidBleBroadcaster::OPERATION_TIMEOUT));
    std::puts("PASS delayed configure freshness refresh, late acknowledgement rejection and bounded refreshes");
}

void failuresAreBounded()
{
    auto s = exampleState();
    for (unsigned operation = 0; operation < 3; ++operation)
    {
        diagnostics.clear();
        FakeGap gap;
        RidBleBroadcaster b(gap, devIdentity(), report);
        CHECK(b.begin(0));
        if (operation == 0) gap.configError = 73;
        if (operation == 1) gap.startError = 73;
        if (operation == 2) gap.stopError = 73;
        run(b, s, 0, 1000);
        CHECK(!b.healthy());
        const RidBleOperation op = operation == 0 ? RidBleOperation::CONFIGURE
            : operation == 1 ? RidBleOperation::START : RidBleOperation::STOP;
        CHECK(sawError(op, 73));
        const auto count = gap.configurations.size();
        run(b, s, 1000, 5000);
        CHECK(gap.configurations.size() == count);
    }
    for (unsigned operation = 0; operation < 3; ++operation)
    {
        diagnostics.clear();
        FakeGap gap;
        if (operation == 0) gap.configCompletion = 91;
        if (operation == 1) gap.startCompletion = 91;
        if (operation == 2) gap.stopCompletion = 91;
        RidBleBroadcaster b(gap, devIdentity(), report);
        CHECK(b.begin(0));
        run(b, s, 0, 1000);
        const RidBleOperation op = operation == 0 ? RidBleOperation::CONFIGURE
            : operation == 1 ? RidBleOperation::START : RidBleOperation::STOP;
        CHECK(!b.healthy() && sawError(op, 91));
    }
    for (unsigned operation = 0; operation < 3; ++operation)
    {
        diagnostics.clear();
        FakeGap gap;
        RidBleBroadcaster b(gap, devIdentity(), report);
        CHECK(b.begin(0));
        uint32_t requested = 0;
        if (operation == 0)
        {
            gap.complete = false;
            b.update(0, s);
        }
        else if (operation == 1)
        {
            b.update(0, s);
            gap.complete = false;
            b.update(1, s); // Configure + start share the deadline from t=0.
        }
        else
        {
            run(b, s, 0, 3);
            gap.complete = false;
            requested = 252;
            b.update(requested, s);
        }
        b.update(requested + 999, s);
        CHECK(b.healthy());
        b.update(requested + 1000, s);
        const RidBleOperation op = operation == 0 ? RidBleOperation::CONFIGURE
            : operation == 1 ? RidBleOperation::START : RidBleOperation::STOP;
        CHECK(!b.healthy() && sawError(op, RidBleBroadcaster::OPERATION_TIMEOUT));
    }
    for (const auto type : {RidBleEventType::STARTED, RidBleEventType::QUEUE_OVERFLOW})
    {
        diagnostics.clear();
        FakeGap gap;
        gap.complete = false;
        RidBleBroadcaster b(gap, devIdentity(), report);
        CHECK(b.begin(0));
        b.update(0, s);
        gap.events.push_back({type, 0});
        b.update(1, s);
        CHECK(!b.healthy());
        CHECK(sawError(type == RidBleEventType::QUEUE_OVERFLOW
              ? RidBleOperation::EVENT_QUEUE : RidBleOperation::CONFIGURE,
              RidBleBroadcaster::UNEXPECTED_EVENT));
    }
    diagnostics.clear();
    FakeGap failedInit;
    failedInit.initError = 42;
    RidBleBroadcaster noStack(failedInit, devIdentity(), report);
    CHECK(!noStack.begin(0));
    run(noStack, s, 0, 2000);
    CHECK(failedInit.configurations.empty());
    CHECK(!noStack.healthy());
    CHECK(diagnostics.size() == 1 && diagnostics[0].status == 42);
    std::puts("PASS immediate/callback errors, bounded missing callbacks, queue faults and failed initialization");
}

void payloadPreservationAndUnchangedCounters()
{
    FakeGap gap;
    RidBleBroadcaster b(gap, devIdentity());
    CHECK(b.begin(0));
    const auto s = exampleState();
    uint8_t location[25], basic[25];
    RidLocationEncoder().encode(RidLocationMapper().map(s), location);
    CHECK(RidBasicIdEncoder().encode(devIdentity(), basic, sizeof(basic)) == RidBasicIdEncodeResult::SUCCESS);
    uint8_t locationCopy[25], basicCopy[25];
    std::memcpy(locationCopy, location, 25);
    std::memcpy(basicCopy, basic, 25);
    uint8_t out[31];
    CHECK(RidBleLegacyFrame::encode(location, 25, 19, out, sizeof(out)));
    CHECK(std::memcmp(out + 6, locationCopy, 25) == 0);
    CHECK(std::memcmp(location, locationCopy, 25) == 0);
    CHECK(RidBleLegacyFrame::encode(basic, 25, 71, out, sizeof(out)));
    CHECK(std::memcmp(out + 6, basicCopy, 25) == 0);
    CHECK(std::memcmp(basic, basicCopy, 25) == 0);
    run(b, s, 0, 5000);
    unsigned counts[2] = {};
    for (const auto& frame : gap.starts)
    {
        const unsigned type = frame[6] >> 4;
        CHECK(std::memcmp(frame.data() + 6, type == 1 ? location : basic, 25) == 0);
        // Chosen permitted policy: increment this type on each successful
        // publication, including unchanged payloads. Switching does not bump
        // the other type. Counter is stable during each radio dwell.
        CHECK(frame[5] == static_cast<uint8_t>(counts[type]++));
    }
    CHECK(counts[0] > 2 && counts[1] > 2);
    CHECK(b.diagnostics(5000).successfulUpdates == gap.starts.size());
    // Fail a subsequent raw update: no publication or successful update count
    // may be invented, and the attempted counter is still that type's next one.
    gap.configError = 67;
    const size_t successful = gap.starts.size();
    run(b, s, 5000, 1000);
    CHECK(!b.healthy());
    CHECK(gap.starts.size() == successful);
    const auto failed = b.diagnostics(6000);
    CHECK(failed.successfulUpdates == successful && failed.failedUpdates == 1);
    CHECK(failed.counter == static_cast<uint8_t>(counts[static_cast<unsigned>(failed.messageType)]));
    std::puts("PASS exact Location/Basic insertion, immutable encoder bytes, unchanged-payload policy and failure counters");
}

void locationAgeMonitoring()
{
    RidLocationAgeMonitor monitor;
    auto age = monitor.snapshot(0);
    CHECK(!age.receiveAgeAvailable && !age.timestampAgeAvailable && !age.gpiSuppressed);
    monitor.observePosition(60000, 1000);
    age = monitor.snapshot(1000);
    CHECK(age.receiveAgeAvailable && age.receiveAgeMs == 0);
    CHECK(!age.timestampAgeAvailable && !age.gpiSuppressed); // Explicitly UNKNOWN, not zero age.
    monitor.observeSystemTime(120000, 1000);
    age = monitor.snapshot(1000);
    CHECK(age.timestampAgeAvailable && age.timestampAgeEstimateMs == 60000);
    CHECK(age.receiveAgeMs == 0 && age.timestampStale && age.gpiSuppressed);

    auto s = exampleState();
    s.horizontalSpeedValid = s.trackDirectionValid = s.verticalSpeedValid = true;
    s.locationTimestampValid = s.timestampAccuracyValid = true;
    s.pressureAltitudeValid = s.geodeticAltitudeValid = s.horizontalAccuracyValid = true;
    s.pressureAltitudeM = 120;
    s.geodeticAltitudeM = 150;
    const auto outgoing = RidLocationAgeMonitor::forAdvertising(s, age);
    CHECK(s.positionValid && s.heightValid && s.locationTimestampValid); // Source unchanged.
    CHECK(!outgoing.positionValid && !outgoing.heightValid && !outgoing.locationTimestampValid);
    CHECK(!outgoing.horizontalSpeedValid && !outgoing.trackDirectionValid && !outgoing.verticalSpeedValid);
    CHECK(!outgoing.timestampAccuracyValid);
    CHECK(outgoing.operationalStatusValid && outgoing.operationalStatus == s.operationalStatus);
    CHECK(outgoing.pressureAltitudeValid && outgoing.pressureAltitudeM == 120);
    CHECK(outgoing.geodeticAltitudeValid && outgoing.geodeticAltitudeM == 150);
    CHECK(outgoing.horizontalAccuracyValid);

    diagnostics.clear();
    FakeGap gap;
    RidBleBroadcaster b(gap, devIdentity(), report);
    CHECK(b.begin(1000));
    for (uint32_t now = 1000; now < 1003; ++now) b.update(now, s, monitor.snapshot(now));
    CHECK(gap.starts.size() == 1);
    const auto& frame = gap.starts[0];
    for (unsigned offset = 11; offset <= 18; ++offset) CHECK(frame[offset] == 0); // LAT/LON unknown.
    CHECK(frame[27] == 255 && frame[28] == 255); // Timestamp unknown.
    CHECK((frame[7] >> 4) == 2); // Operational Status preserved.
    uint8_t expected[25];
    RidLocationEncoder().encode(RidLocationMapper().map(outgoing), expected);
    CHECK(std::memcmp(frame.data() + 6, expected, 25) == 0);

    // Losing the anchor or re-receiving the same stale timestamp cannot clear
    // previously observed evidence of staleness.
    age = monitor.snapshot(6000);
    CHECK(!age.timestampAgeAvailable && age.timestampStale && age.gpiSuppressed);
    monitor.observePosition(60000, 6001);
    CHECK(monitor.snapshot(6001).gpiSuppressed);
    monitor.observePosition(60001, 6001);
    CHECK(monitor.snapshot(6001).gpiSuppressed);
    monitor.observePosition(59999, 6001);
    CHECK(monitor.snapshot(6001).gpiSuppressed);
    monitor.observeSystemTime(125001, 6001);
    monitor.observePosition(125001, 6001);
    age = monitor.snapshot(6001);
    CHECK(age.timestampAgeAvailable && age.timestampAgeEstimateMs == 0 && !age.gpiSuppressed);
    CHECK(!monitor.snapshot(7001).gpiSuppressed); // Exactly 1000 ms tag age is allowed.
    CHECK(monitor.snapshot(7002).gpiSuppressed); // Beyond 1000 ms is blocked.

    // Clock discontinuity, post-reset recovery, future tags and both rollovers.
    monitor.observeSystemTime(100, 7003);
    age = monitor.snapshot(7003);
    CHECK(age.clockAmbiguous && age.gpiSuppressed && !age.timestampAgeAvailable);
    monitor.observePosition(100, 7003);
    CHECK(!monitor.snapshot(7003).gpiSuppressed);
    monitor.observePosition(10000, 7004);
    CHECK(monitor.snapshot(7004).clockAmbiguous);
    RidLocationAgeMonitor rollover;
    rollover.observeSystemTime(UINT32_MAX - 100, UINT32_MAX - 100);
    rollover.observePosition(UINT32_MAX - 300, UINT32_MAX - 100);
    age = rollover.snapshot(200);
    CHECK(age.receiveAgeMs == 301 && age.timestampAgeEstimateMs == 501 && !age.gpiSuppressed);
    rollover.observeSystemTime(300, 300);
    rollover.observePosition(300, 300);
    CHECK(!rollover.snapshot(301).clockAmbiguous);
    std::puts("PASS 60-second stale GPI isolation, unknown ages, stale latch, clock ambiguity/recovery and rollover");
}

void activePacketAgeAndControllerGaps()
{
    diagnostics.clear();
    FakeGap gap;
    RidBleBroadcaster b(gap, devIdentity(), report);
    CHECK(b.begin(0));
    auto s = exampleState();
    RidLocationAgeInfo age;
    age.receiveAgeAvailable = age.timestampAgeAvailable = true;
    for (uint32_t now = 0; now < 3; ++now)
    {
        age.receiveAgeMs = now;
        age.timestampAgeEstimateMs = 900 + now;
        b.update(now, s, age);
    }
    CHECK(gap.starts.size() == 1 && gap.stopCalls == 0);
    // A NEW fresh GPI must not hide the age of the OLD packet still on air.
    s.latitudeDeg += 0.01;
    age.receiveAgeMs = age.timestampAgeEstimateMs = 0;
    b.update(101, s, age);
    CHECK(gap.stopCalls == 1); // Early stop instead of finishing the 250 ms dwell.
    const auto aged = b.diagnostics(101);
    CHECK(aged.locationAge.timestampAgeEstimateMs == 1001 && aged.locationAge.timestampStale);
    CHECK(aged.preparedAgeMs == 100 && aged.advertisingAgeMs == 99);

    FakeGap timedGap;
    timedGap.complete = false;
    RidBleBroadcaster timed(timedGap, devIdentity());
    CHECK(timed.begin(0));
    timed.update(0, s);
    timedGap.events.push_back({RidBleEventType::CONFIGURED, 0, 5, true});
    timed.update(8, s);
    timedGap.events.push_back({RidBleEventType::STARTED, 0, 10, true});
    timed.update(12, s);
    CHECK(timed.diagnostics(12).advertisingAgeMs == 2);
    timed.update(260, s);
    timedGap.events.push_back({RidBleEventType::STOPPED, 0, 263, true});
    timed.update(270, s);
    timedGap.events.push_back({RidBleEventType::CONFIGURED, 0, 273, true});
    timed.update(280, s);
    timedGap.events.push_back({RidBleEventType::STARTED, 0, 284, true});
    timed.update(290, s);
    const auto measured = timed.diagnostics(290);
    CHECK(measured.restartGapAvailable && measured.restartGapMs == 21);
    CHECK(measured.maxRestartGapMs == 21 && measured.switchLatencyMs == 24);
    CHECK(measured.callbackDelayMs == 6 && measured.preparedAgeMs == 20 && measured.advertisingAgeMs == 6);
    CHECK(measured.successfulUpdates == 2 && measured.failedUpdates == 0);
    CHECK(timed.healthy());
    std::puts("PASS active-packet age isolation/early stop and timestamped controller-gap measurements (not RF proof)");
}
} // namespace

int main()
{
    framing();
    schedulingAndCounters();
    dwellAndLatestState();
    rolloverAndDelayedLoop();
    delayedConfiguration();
    failuresAreBounded();
    payloadPreservationAndUnchangedCounters();
    locationAgeMonitoring();
    activePacketAgeAndControllerGaps();
    std::printf("BLE legacy: %u assertions, %u failures\n", assertions, failures);
    return failures ? 1 : 0;
}
