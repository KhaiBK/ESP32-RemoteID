#pragma once

#include <stdint.h>
#include <stddef.h>


// ============================================================
// Operator Location Type
//
// ASTM F3411 System Message Byte 1 bits [1..0]
//
// 0 = Take Off
// 1 = Dynamic
// 2 = Fixed
// ============================================================

enum class RidOperatorLocationType : uint8_t
{
    TAKEOFF = 0,
    DYNAMIC = 1,
    FIXED   = 2
};


// ============================================================
// Classification Type
//
// ASTM F3411 System Message Byte 1 bits [4..2]
//
// 0 = Undeclared
// 1 = European Union
// ============================================================

enum class RidClassificationType : uint8_t
{
    UNDECLARED = 0,
    EUROPEAN_UNION = 1
};


// ============================================================
// RidSystemSourceV1
//
// Normalized source representation.
//
// Values here are physical / semantic values.
// They have NOT yet been converted into ASTM encoded codes.
// ============================================================

struct RidSystemSource
{
    // ========================================================
    // Operator location source
    // ========================================================

    RidOperatorLocationType operatorLocationType =
        RidOperatorLocationType::TAKEOFF;


    RidClassificationType classificationType =
        RidClassificationType::UNDECLARED;


    // ========================================================
    // Operator position
    //
    // TAKEOFF:
    //   lat/lon captured at takeoff
    //
    // DYNAMIC:
    //   actual Remote Pilot / GCS GNSS
    //
    // FIXED:
    //   configured fixed operator position
    // ========================================================

    bool operatorPositionValid = false;

    double operatorLatitudeDeg = 0.0;
    double operatorLongitudeDeg = 0.0;


    // ========================================================
    // Area / group
    // ========================================================

    uint16_t areaCount = 1;

    float areaRadiusM = 0.0f;


    bool areaCeilingValid = false;
    float areaCeilingM = 0.0f;


    bool areaFloorValid = false;
    float areaFloorM = 0.0f;


    // ========================================================
    // EU Classification
    //
    // Used only when:
    // classificationType == EUROPEAN_UNION
    //
    // Category:
    // 0 Undefined
    // 1 Open
    // 2 Specific
    // 3 Certified
    //
    // Class:
    // 0 Undefined
    // 1 Class 0
    // 2 Class 1
    // ...
    // 7 Class 6
    // ========================================================

    uint8_t categoryEU = 0;
    uint8_t classEU = 0;


    // ========================================================
    // Operator Altitude
    //
    // WGS84 HAE
    // ========================================================

    bool operatorAltitudeValid = false;
    float operatorAltitudeM = 0.0f;


    // ========================================================
    // Timestamp
    //
    // Full UTC timestamp corresponding to the Location
    // Message applicability time.
    //
    // Unit: microseconds since Unix epoch 1970.
    // ========================================================

    bool timestampValid = false;
    uint64_t timestampUtcUs = 0;
};

