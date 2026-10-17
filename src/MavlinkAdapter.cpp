#include "MavlinkAdapter.h"
#include <math.h>

// ======================================================
// Constructor
// ======================================================

MavlinkAdapter::MavlinkAdapter(DroneStateV1& state)
    : state(state)
{
}


// ======================================================
// Main MAVLink message dispatcher
// ======================================================

void MavlinkAdapter::handleMessage(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handleHeartbeat(msg, nowMs);
            break;

        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
            handleExtendedSysState(msg, nowMs);
            break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            handleGlobalPositionInt(msg, nowMs);
            break;

        case MAVLINK_MSG_ID_GPS_RAW_INT:
            handleGpsRawInt(msg, nowMs);
            break;

        case MAVLINK_MSG_ID_SCALED_PRESSURE:
            handleScaledPressure(msg, nowMs);
            break;

        case MAVLINK_MSG_ID_SYSTEM_TIME:
            handleSystemTime(msg, nowMs);
            break;

        default:
            // Message not used by Remote ID Location/Vector V1.
            break;
    }
}


// ======================================================
// Message handlers
//
// These are intentionally empty for now.
// We will implement them one by one.
// ======================================================

void MavlinkAdapter::handleHeartbeat(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    (void)msg;
    (void)nowMs;
}


void MavlinkAdapter::handleExtendedSysState(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    (void)msg;
    (void)nowMs;
}


void MavlinkAdapter::handleGlobalPositionInt(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    mavlink_global_position_int_t pos;

    mavlink_msg_global_position_int_decode(
        &msg,
        &pos
    );

    // ==================================================
    // 1. Latitude / Longitude
    // MAVLink:
    //   lat, lon = degE7
    // DroneState:
    //   degree
    // ==================================================

    state.latitudeDeg =
        static_cast<double>(pos.lat) / 1e7;

    state.longitudeDeg =
        static_cast<double>(pos.lon) / 1e7;


    bool latitudeValid =
        state.latitudeDeg >= -90.0 &&
        state.latitudeDeg <= 90.0;

    bool longitudeValid =
        state.longitudeDeg >= -180.0 &&
        state.longitudeDeg <= 180.0;

    state.positionValid =
        latitudeValid && longitudeValid;

    state.positionUpdatedMs = nowMs;


    // ==================================================
    // 2. Horizontal velocity components
    //
    // MAVLink GLOBAL_POSITION_INT:
    //   vx = North velocity, cm/s
    //   vy = East velocity,  cm/s
    //
    // Convert to m/s
    // ==================================================

    float velocityNorthMps =
        static_cast<float>(pos.vx) * 0.01f;

    float velocityEastMps =
        static_cast<float>(pos.vy) * 0.01f;


    // ==================================================
    // 3. Horizontal Speed
    // ==================================================

    state.horizontalSpeedMps =
        sqrtf(
            velocityNorthMps * velocityNorthMps +
            velocityEastMps  * velocityEastMps
        );

    state.horizontalSpeedValid = true;


    // ==================================================
    // 4. Track Direction
    //
    // atan2(East, North)
    //
    // Result:
    //   0 deg   = North
    //   90 deg  = East
    //   180 deg = South
    //   270 deg = West
    // ==================================================

    constexpr float RAD_TO_DEG =
        57.29577951308232f;

    float trackDirection =
        atan2f(
            velocityEastMps,
            velocityNorthMps
        ) * RAD_TO_DEG;

    if (trackDirection < 0.0f)
    {
        trackDirection += 360.0f;
    }

    state.trackDirectionDeg =
        trackDirection;


    // Temporary mathematical validity rule:
    // if both horizontal components are exactly zero,
    // direction is undefined.
    //
    // Final low-speed validity threshold is still OPEN.
    state.trackDirectionValid =
        (pos.vx != 0 || pos.vy != 0);


    // ==================================================
    // 5. Vertical Speed
    //
    // MAVLink:
    //   vz positive = DOWN
    //
    // Remote ID semantic:
    //   positive = UP
    // ==================================================

    state.verticalSpeedMps =
        -(static_cast<float>(pos.vz) * 0.01f);

    state.verticalSpeedValid = true;


    // Position and motion came from this same message.
    state.motionUpdatedMs = nowMs;
}

void MavlinkAdapter::handleGpsRawInt(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    (void)msg;
    (void)nowMs;
}


void MavlinkAdapter::handleScaledPressure(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    (void)msg;
    (void)nowMs;
}


void MavlinkAdapter::handleSystemTime(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    (void)msg;
    (void)nowMs;
}