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
    mavlink_heartbeat_t heartbeat;

    mavlink_msg_heartbeat_decode(
        &msg,
        &heartbeat
    );


    bool newArmedState =
        (heartbeat.base_mode &
         MAV_MODE_FLAG_SAFETY_ARMED) != 0;

    
    fcEmergency =
        (heartbeat.system_status ==
         MAV_STATE_EMERGENCY);
    // --------------------------------------------------
    // Detect DISARMED -> ARMED transition
    // --------------------------------------------------

    if (!armed && newArmedState)
    {
        takeoffReferencePending = true;
        takeoffReferenceValid = false;

        state.heightValid = false;
    }


    // --------------------------------------------------
    // Detect ARMED -> DISARMED transition
    // --------------------------------------------------

    if (armed && !newArmedState)
    {
        takeoffReferencePending = false;
        takeoffReferenceValid = false;

        state.heightValid = false;
    }


    armed = newArmedState;

   updateOperationalStatus(nowMs);
}


void MavlinkAdapter::handleExtendedSysState(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    mavlink_extended_sys_state_t ext;

    mavlink_msg_extended_sys_state_decode(
        &msg,
        &ext
    );

    // Store source state.
    // Final Remote ID operational status is decided
    // by updateOperationalStatus().
    landedState = ext.landed_state;

    updateOperationalStatus(nowMs);
}


void MavlinkAdapter::updateOperationalStatus(
    uint32_t nowMs
)
{
    // Highest priority:
    // failure of the Remote ID module itself
    if (ridSystemFailure)
    {
        state.operationalStatus =
            RidOperationalStatus::RID_SYSTEM_FAILURE;

        state.operationalStatusValid = true;
    }

    // Flight controller emergency
    else if (fcEmergency)
    {
        state.operationalStatus =
            RidOperationalStatus::EMERGENCY;

        state.operationalStatusValid = true;
    }

    // Normal flight state
    else
    {
        switch (landedState)
        {
            case MAV_LANDED_STATE_ON_GROUND:

                state.operationalStatus =
                    RidOperationalStatus::GROUND;

                state.operationalStatusValid = true;
                break;


            case MAV_LANDED_STATE_IN_AIR:

            case MAV_LANDED_STATE_TAKEOFF:

            case MAV_LANDED_STATE_LANDING:

                state.operationalStatus =
                    RidOperationalStatus::AIRBORNE;

                state.operationalStatusValid = true;
                break;


            case MAV_LANDED_STATE_UNDEFINED:

            default:

                state.operationalStatus =
                    RidOperationalStatus::UNDECLARED;

                state.operationalStatusValid = false;
                break;
        }
    }

    state.statusUpdatedMs = nowMs;
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



    // ============================================
    // Height Above Takeoff  
    // ============================================

    float currentAltitudeMslM =
        static_cast<float>(pos.alt) / 1000.0f;

    if (takeoffReferencePending)
    {
        takeoffAltitudeMslM =
            currentAltitudeMslM;

        takeoffReferenceValid = true;
        takeoffReferencePending = false;
    }

    if (takeoffReferenceValid)
    {
        state.heightM =
            currentAltitudeMslM -
            takeoffAltitudeMslM;

        state.heightReference =
            RidHeightReference::ABOVE_TAKEOFF;

        state.heightValid = true;

        state.heightUpdatedMs = nowMs;
    }
    else
    {
        state.heightValid = false;
    }

    // ==================================================
// 7. Location Timestamp
//
// GLOBAL_POSITION_INT.time_boot_ms:
//   time of this position sample since FC boot
//
// Convert that sample time to UTC using SYSTEM_TIME.
// ==================================================

 if (timeSyncValid)
{
    int64_t sampleUtcUs =
        unixMinusBootUs +
        static_cast<int64_t>(
            pos.time_boot_ms
        ) * 1000LL;

    if (sampleUtcUs > 0)
    {
        state.locationTimestampUtcUs =
            static_cast<uint64_t>(
                sampleUtcUs
            );

        state.locationTimestampValid = true;

        state.timeUpdatedMs = nowMs;
    }
    else
    {
        state.locationTimestampValid = false;
    }
}
else
{
    state.locationTimestampValid = false;
}

}



void MavlinkAdapter::handleGpsRawInt(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    mavlink_gps_raw_int_t gps;

    mavlink_msg_gps_raw_int_decode(
        &msg,
        &gps
    );


    // ==================================================
    // Geodetic Altitude
    //
    // GPS_RAW_INT.alt_ellipsoid:
    //   altitude above WGS84 ellipsoid
    //   unit: mm
    //
    // DroneState:
    //   geodeticAltitudeM
    //   unit: m
    // ==================================================

    bool has3DFix =
        gps.fix_type >= GPS_FIX_TYPE_3D_FIX;


    // alt_ellipsoid is a MAVLink 2 extension field.
    //
    // GPS_RAW_INT minimum payload = original MAVLink 1 fields.
    // alt_ellipsoid occupies the first 4 extension bytes.
    //
    // Conservative V1 check:
    // only accept it when those extension bytes are present.
    bool altEllipsoidPresent =
        msg.len >=
        (MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN + 4);


    if (has3DFix && altEllipsoidPresent)
    {
        state.geodeticAltitudeM =
            static_cast<float>(
                gps.alt_ellipsoid
            ) / 1000.0f;

        state.geodeticAltitudeValid = true;

        state.geodeticAltitudeUpdatedMs =
            nowMs;
    }
    else
    {
        state.geodeticAltitudeValid = false;
    }
    // ==================================================
// GPS Accuracy / Uncertainty
//
// GPS_RAW_INT:
//   h_acc   = horizontal position uncertainty, mm
//   v_acc   = vertical altitude uncertainty, mm
//   vel_acc = speed uncertainty, mm/s
//
// ArduPilot currently sends these as one-sigma
// standard deviation estimates.
// ==================================================


// --------------------------------------------------
// Horizontal Accuracy
// --------------------------------------------------

if (has3DFix && gps.h_acc > 0)
{
    state.horizontalAccuracyM =
        static_cast<float>(gps.h_acc) / 1000.0f;

    state.horizontalAccuracyValid = true;
}
else
{
    state.horizontalAccuracyValid = false;
}


// --------------------------------------------------
// Vertical Accuracy
// --------------------------------------------------

if (has3DFix && gps.v_acc > 0)
{
    state.verticalAccuracyM =
        static_cast<float>(gps.v_acc) / 1000.0f;

    state.verticalAccuracyValid = true;
}
else
{
    state.verticalAccuracyValid = false;
}


// --------------------------------------------------
// Speed Accuracy
// --------------------------------------------------

if (has3DFix && gps.vel_acc > 0)
{
    state.speedAccuracyMps =
        static_cast<float>(gps.vel_acc) / 1000.0f;

    state.speedAccuracyValid = true;
}
else
{
    state.speedAccuracyValid = false;
}


// Update accuracy freshness timestamp if at least
// one accuracy source is available.
if (state.horizontalAccuracyValid ||
    state.verticalAccuracyValid ||
    state.speedAccuracyValid)
{
    state.accuracyUpdatedMs = nowMs;
}
}


void MavlinkAdapter::handleScaledPressure(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    mavlink_scaled_pressure_t pressure;

    mavlink_msg_scaled_pressure_decode(
        &msg,
        &pressure
    );


    // ==================================================
    // Pressure Altitude
    //
    // SCALED_PRESSURE.press_abs:
    //   absolute pressure, hPa
    //
    // ASTM / Remote ID pressure altitude:
    //   referenced to standard pressure 1013.2 hPa
    // ==================================================

    constexpr float STANDARD_PRESSURE_HPA = 1013.2f;

    if (isfinite(pressure.press_abs) &&
        pressure.press_abs > 0.0f)
    {
        float ratio =
            pressure.press_abs /
            STANDARD_PRESSURE_HPA;

        state.pressureAltitudeM =
            44330.0f *
            (1.0f - powf(ratio, 0.1903f));

        state.pressureAltitudeValid = true;

        state.pressureAltitudeUpdatedMs =
            nowMs;
    }
    else
    {
        state.pressureAltitudeValid = false;
    }
}


void MavlinkAdapter::handleSystemTime(
    const mavlink_message_t& msg,
    uint32_t nowMs
)
{
    mavlink_system_time_t sys;

    mavlink_msg_system_time_decode(
        &msg,
        &sys
    );


    // ==================================================
    // Synchronize FC boot time with UTC
    //
    // SYSTEM_TIME:
    //   time_unix_usec = UTC Unix time [us]
    //   time_boot_ms   = time since FC boot [ms]
    // ==================================================

    if (sys.time_unix_usec != 0)
    {
        int64_t bootTimeUs =
            static_cast<int64_t>(
                sys.time_boot_ms
            ) * 1000LL;

        unixMinusBootUs =
            static_cast<int64_t>(
                sys.time_unix_usec
            ) - bootTimeUs;

        timeSyncValid = true;
        timeSyncUpdatedMs = nowMs;
    }
    else
    {
        timeSyncValid = false;
    }

    state.timestampAccuracyS = 0.0f;
    state.timestampAccuracyValid = false;
}