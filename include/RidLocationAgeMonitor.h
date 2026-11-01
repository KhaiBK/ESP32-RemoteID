#pragma once

#include "DroneState.h"

// Observed transport ages, not a sensor time-of-applicability guarantee.
struct RidLocationAgeInfo
{
    bool receiveAgeAvailable = false;
    uint32_t receiveAgeMs = 0;
    bool timestampAgeAvailable = false;
    uint32_t timestampAgeEstimateMs = 0;
    bool clockAmbiguous = false;
    bool timestampStale = false;
    bool gpiSuppressed = false;
    uint32_t positionBootMs = 0;
    bool anchorAvailable = false;
    uint32_t anchorAgeMs = 0;
};

// Side-channel observer of the SAME selected FC's SYSTEM_TIME/GPI messages.
// Does not change MavlinkAdapter, DroneState, synchronization, or ARM state.
class RidLocationAgeMonitor
{
public:
    static constexpr uint32_t MAX_DYNAMIC_AGE_MS = 1000;
    // Provisional age-estimate horizon, independent of production UTC holdover.
    static constexpr uint32_t BOOT_ESTIMATE_HORIZON_MS = 5000;
    void observeSystemTime(uint32_t fcBootMs, uint32_t receivedMs);
    void observePosition(uint32_t fcBootMs, uint32_t receivedMs);
    RidLocationAgeInfo snapshot(uint32_t nowMs);
    static DroneStateV1 forAdvertising(const DroneStateV1& source, const RidLocationAgeInfo& age);

private:
    bool systemSeen = false, positionSeen = false;
    uint32_t systemBootMs = 0, systemRxMs = 0;
    uint32_t positionBootMs = 0, positionRxMs = 0;
    bool positionClockInvalid = false;
    bool staleObserved = false;
    bool staleReferenceAvailable = false;
    uint32_t staleReferenceBootMs = 0;
};
