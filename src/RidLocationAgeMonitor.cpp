#include "RidLocationAgeMonitor.h"

void RidLocationAgeMonitor::observeSystemTime(uint32_t fcBootMs, uint32_t receivedMs)
{
    // A backward FC clock (except normal uint32 rollover) invalidates the
    // relationship to a position received before that clock change.
    if (systemSeen && static_cast<uint32_t>(fcBootMs - systemBootMs) > INT32_MAX)
    {
        positionClockInvalid = positionSeen;
        staleObserved = false;
        staleReferenceAvailable = false;
    }
    systemSeen = true;
    systemBootMs = fcBootMs;
    systemRxMs = receivedMs;
}

void RidLocationAgeMonitor::observePosition(uint32_t fcBootMs, uint32_t receivedMs)
{
    // Re-receiving an already-known stale timestamp must not make it fresh.
    if (!positionSeen) staleObserved = false;
    else if (fcBootMs != positionBootMs)
    {
        const uint32_t behindKnownClock = static_cast<uint32_t>(staleReferenceBootMs - fcBootMs);
        // Even without a new anchor, a replay still behind a previously
        // observed FC clock cannot erase evidence that it is already old.
        staleObserved = staleReferenceAvailable && behindKnownClock <= INT32_MAX &&
            behindKnownClock > MAX_DYNAMIC_AGE_MS;
    }
    positionSeen = true;
    positionBootMs = fcBootMs;
    positionRxMs = receivedMs;
    positionClockInvalid = false;
}

RidLocationAgeInfo RidLocationAgeMonitor::snapshot(uint32_t nowMs)
{
    RidLocationAgeInfo age;
    age.receiveAgeAvailable = positionSeen;
    age.receiveAgeMs = static_cast<uint32_t>(nowMs - positionRxMs);
    age.positionBootMs = positionBootMs;
    age.anchorAvailable = systemSeen;
    age.anchorAgeMs = static_cast<uint32_t>(nowMs - systemRxMs);
    age.clockAmbiguous = positionSeen && positionClockInvalid;
    if (systemSeen && positionSeen && !positionClockInvalid &&
        age.anchorAgeMs < BOOT_ESTIMATE_HORIZON_MS)
    {
        const uint32_t difference = static_cast<uint32_t>(systemBootMs - positionBootMs);
        // Signed modular subtraction without an implementation-defined cast.
        const int64_t bootDifference = difference <= INT32_MAX
            ? static_cast<int64_t>(difference) : static_cast<int64_t>(difference) - (INT64_C(1) << 32);
        const int64_t estimate = bootDifference + age.anchorAgeMs;
        if (difference == UINT32_C(0x80000000) || estimate < 0 || estimate > UINT32_MAX)
            age.clockAmbiguous = true;
        else
        {
            age.timestampAgeAvailable = true;
            age.timestampAgeEstimateMs = static_cast<uint32_t>(estimate);
            if (age.timestampAgeEstimateMs > MAX_DYNAMIC_AGE_MS)
            {
                staleObserved = true;
                staleReferenceAvailable = true;
                staleReferenceBootMs = systemBootMs + age.anchorAgeMs;
            }
        }
    }
    age.timestampStale = positionSeen && staleObserved;
    age.gpiSuppressed = age.timestampStale || age.clockAmbiguous;
    return age;
}

DroneStateV1 RidLocationAgeMonitor::forAdvertising(const DroneStateV1& source, const RidLocationAgeInfo& age)
{
    DroneStateV1 copy = source;
    if (age.gpiSuppressed)
    {
        // Only this outgoing snapshot changes. Other telemetry and the stored
        // takeoff reference remain untouched. Existing encoders emit unknowns.
        copy.positionValid = false;
        copy.horizontalSpeedValid = false;
        copy.trackDirectionValid = false;
        copy.verticalSpeedValid = false;
        copy.heightValid = false;
        copy.locationTimestampValid = false;
        copy.timestampAccuracyValid = false;
    }
    return copy;
}
