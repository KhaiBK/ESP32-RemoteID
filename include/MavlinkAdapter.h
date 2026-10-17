#pragma once

#include <stdint.h>
#include <ardupilotmega/mavlink.h>

#include "DroneState.h"


class MavlinkAdapter
{
public:

    // Adapter receives MAVLink data and updates DroneState.
    explicit MavlinkAdapter(DroneStateV1& state);

    // Entry point for every successfully parsed MAVLink message.
    void handleMessage(
        const mavlink_message_t& msg,
        uint32_t nowMs
    );


private:

    DroneStateV1& state;
    
    // --------------------------------------------------
// ARM / Takeoff reference state
// --------------------------------------------------

bool armedStateInitialized = false;

bool armed = false;

bool takeoffReferencePending = false;

bool takeoffReferenceValid = false;

float takeoffAltitudeMslM = 0.0f;

bool fcEmergency = false;


uint8_t landedState = MAV_LANDED_STATE_UNDEFINED;

// --------------------------------------------------
// UTC / boot-time synchronization
// --------------------------------------------------

bool timeSyncValid = false;

int64_t unixMinusBootUs = 0;

uint32_t timeSyncUpdatedMs = 0;

    // --------------------------------------------------
    // MAVLink message handlers
    // --------------------------------------------------

    void handleHeartbeat(
        const mavlink_message_t& msg,
        uint32_t nowMs
    );

    void handleExtendedSysState(
        const mavlink_message_t& msg,
        uint32_t nowMs
    );

    void handleGlobalPositionInt(
        const mavlink_message_t& msg,
        uint32_t nowMs
    );

    void handleGpsRawInt(
        const mavlink_message_t& msg,
        uint32_t nowMs
    );

    void handleScaledPressure(
        const mavlink_message_t& msg,
        uint32_t nowMs
    );

    void handleSystemTime(
        const mavlink_message_t& msg,
        uint32_t nowMs
    );

    void updateOperationalStatus(
          uint32_t nowMs
    );
};