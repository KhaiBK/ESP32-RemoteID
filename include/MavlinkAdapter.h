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
};