#pragma once

#include <stdint.h>

// ======================================================
// Remote ID semantic enums
// ======================================================

enum class RidOperationalStatus : uint8_t
{
    UNDECLARED = 0,
    GROUND,
    AIRBORNE,
    EMERGENCY,
    RID_SYSTEM_FAILURE
};

enum class RidHeightReference : uint8_t
{
    ABOVE_TAKEOFF = 0,
    AGL
};


// ======================================================
// Normalized drone state for Remote ID Location/Vector
// ======================================================

struct DroneStateV1
{
    // --------------------------------------------------
    // Operational Status
    // --------------------------------------------------

    RidOperationalStatus operationalStatus =
        RidOperationalStatus::UNDECLARED;

    bool operationalStatusValid = false;


    // --------------------------------------------------
    // Horizontal Motion
    // --------------------------------------------------

    // Direction of movement relative to True North
    // Unit: degree
    // Range: 0 <= trackDirectionDeg < 360
    float trackDirectionDeg = 0.0f;

    bool trackDirectionValid = false;


    // Horizontal ground speed
    // Unit: m/s
    float horizontalSpeedMps = 0.0f;

    bool horizontalSpeedValid = false;


    // --------------------------------------------------
    // Vertical Motion
    // --------------------------------------------------

    // Positive = climbing
    // Negative = descending
    // Unit: m/s
    float verticalSpeedMps = 0.0f;

    bool verticalSpeedValid = false;


    // --------------------------------------------------
    // Position
    // --------------------------------------------------

    // Unit: degree
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;

    bool positionValid = false;


    // --------------------------------------------------
    // Altitudes
    // --------------------------------------------------

    // Pressure altitude
    // Reference pressure: 1013.2 hPa
    // Unit: m
    float pressureAltitudeM = 0.0f;

    bool pressureAltitudeValid = false;


    // WGS84 Height Above Ellipsoid
    // Unit: m
    float geodeticAltitudeM = 0.0f;

    bool geodeticAltitudeValid = false;


    // --------------------------------------------------
    // Height
    // --------------------------------------------------

    // Height relative to the selected reference
    // Unit: m
    float heightM = 0.0f;

    RidHeightReference heightReference =
        RidHeightReference::ABOVE_TAKEOFF;

    bool heightValid = false;


    // --------------------------------------------------
    // Accuracy
    // --------------------------------------------------

    // Unit: m
    float horizontalAccuracyM = 0.0f;
    bool horizontalAccuracyValid = false;

    // Unit: m
    float verticalAccuracyM = 0.0f;
    bool verticalAccuracyValid = false;

    // Unit: m
    float barometricAltitudeAccuracyM = 0.0f;
    bool barometricAltitudeAccuracyValid = false;

    // Unit: m/s
    float speedAccuracyMps = 0.0f;
    bool speedAccuracyValid = false;


    // --------------------------------------------------
    // Location Timestamp
    // --------------------------------------------------

    // UTC time of applicability of the position sample
    // Unit: microseconds since Unix epoch
    uint64_t locationTimestampUtcUs = 0;

    bool locationTimestampValid = false;


    // Unit: seconds
    float timestampAccuracyS = 0.0f;

    bool timestampAccuracyValid = false;


    // --------------------------------------------------
    // Freshness metadata
    //
    // These are NOT ASTM fields.
    // They are used internally for stale-data detection.
    // Unit: millis() time
    // --------------------------------------------------

    uint32_t statusUpdatedMs = 0;

    uint32_t motionUpdatedMs = 0;

    uint32_t positionUpdatedMs = 0;

    uint32_t pressureAltitudeUpdatedMs = 0;

    uint32_t geodeticAltitudeUpdatedMs = 0;

    uint32_t heightUpdatedMs = 0;

    uint32_t accuracyUpdatedMs = 0;

    uint32_t timeUpdatedMs = 0;
};