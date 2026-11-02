#include "RidLocationMapper.h"
#include <math.h>
#include <limits.h>
#include <string.h>

namespace
{

uint8_t mapHorizontalAccuracy(float accuracyM)
{
    // ASTM Horizontal Accuracy / NACp

    if (!isfinite(accuracyM) || accuracyM < 0.0f)
    {
        return 0;
    }

    if (accuracyM < 1.0f)
    {
        return 12;
    }
    else if (accuracyM < 3.0f)
    {
        return 11;
    }
    else if (accuracyM < 10.0f)
    {
        return 10;
    }
    else if (accuracyM < 30.0f)
    {
        return 9;
    }
    else if (accuracyM < 92.6f)
    {
        return 8;
    }
    else if (accuracyM < 185.2f)
    {
        return 7;
    }
    else if (accuracyM < 555.6f)
    {
        return 6;
    }
    else if (accuracyM < 926.0f)
    {
        return 5;
    }
    else if (accuracyM < 1852.0f)
    {
        return 4;
    }
    else if (accuracyM < 3704.0f)
    {
        return 3;
    }
    else if (accuracyM < 7408.0f)
    {
        return 2;
    }
    else if (accuracyM < 18520.0f)
    {
        return 1;
    }

    return 0;
}


uint8_t mapVerticalAccuracy(float accuracyM)
{
    // ASTM Vertical Accuracy / GVA

    if (!isfinite(accuracyM) || accuracyM < 0.0f)
    {
        return 0;
    }

    if (accuracyM < 1.0f)
    {
        return 6;
    }
    else if (accuracyM < 3.0f)
    {
        return 5;
    }
    else if (accuracyM < 10.0f)
    {
        return 4;
    }
    else if (accuracyM < 25.0f)
    {
        return 3;
    }
    else if (accuracyM < 45.0f)
    {
        return 2;
    }
    else if (accuracyM < 150.0f)
    {
        return 1;
    }

    return 0;
}


uint8_t mapSpeedAccuracy(float accuracyMps)
{
    if (!isfinite(accuracyMps) ||
        accuracyMps < 0.0f)
    {
        return 0;
    }

    if (accuracyMps < 0.3f)
    {
        return 4;
    }
    else if (accuracyMps < 1.0f)
    {
        return 3;
    }
    else if (accuracyMps < 3.0f)
    {
        return 2;
    }
    else if (accuracyMps < 10.0f)
    {
        return 1;
    }

    return 0;
}

uint8_t mapTimestampAccuracy(float accuracyS)
{
    if (!isfinite(accuracyS) ||
        accuracyS < 0.0f ||
        accuracyS > 1.5f)
    {
        return 0;
    }

    for (uint8_t code = 1; code <= 15; ++code)
    {
        float threshold =
            static_cast<float>(code) / 10.0f;

        if (accuracyS <= threshold)
        {
            return code;
        }
    }

    return 0;
}

uint16_t mapAltitude(
    float altitudeM,
    bool valid
)
{
    if (!valid ||
        !isfinite(altitudeM) ||
        altitudeM <= -1000.0f)
    {
        return 0;
    }

    float encoded =
        (altitudeM + 1000.0f) / 0.5f;

    // Saturate before rounding to avoid overflow
    if (encoded >= 65535.0f)
    {
        return 65535;
    }

    long code = lroundf(encoded);

    // Code 0 is reserved for Unknown
    if (code < 1)
    {
        code = 1;
    }

    return static_cast<uint16_t>(code);
}

} // namespace




RidLocationMapped RidLocationMapper::map(
    const DroneStateV1& state
) const
{
    RidLocationMapped out;


    // ==================================================
    // BYTE 1
    // ==================================================


    // --------------------------------------------------
    // bits 7..4
    // Operational Status
    // --------------------------------------------------

    if (!state.operationalStatusValid)
    {
        // ASTM: Undeclared
        out.operationalStatusCode = 0;
    }
    else
    {
        switch (state.operationalStatus)
        {
            case RidOperationalStatus::GROUND:
                out.operationalStatusCode = 1;
                break;

            case RidOperationalStatus::AIRBORNE:
                out.operationalStatusCode = 2;
                break;

            case RidOperationalStatus::EMERGENCY:
                out.operationalStatusCode = 3;
                break;
            case RidOperationalStatus::UNDECLARED:
            default:
                out.operationalStatusCode = 0;
                break;
        }
    }


    // --------------------------------------------------
    // bit 2
    // Height Type
    // --------------------------------------------------

    switch (state.heightReference)
    {
        case RidHeightReference::AGL:
            out.heightTypeBit = 1;
            break;

        case RidHeightReference::ABOVE_TAKEOFF:
        default:
            out.heightTypeBit = 0;
            break;
    }


    // --------------------------------------------------
    // bit 1
    // EW Direction Segment
    //
    // ASTM represents the complete 0..360 degree
    // direction using:
    //
    //   EW segment bit
    //        +
    //   Byte 2 direction value
    //
    // Byte 2 will be implemented next.
    // --------------------------------------------------

   // --------------------------------------------------
// Byte 1 bit 1 + Byte 2
// Track Direction
//
// These two fields are coupled and therefore must
// be mapped together.
// --------------------------------------------------

if (state.trackDirectionValid &&
    isfinite(state.trackDirectionDeg))
{
    float direction =
        fmodf(state.trackDirectionDeg, 360.0f);

    // Normalize in bounded time, including very large finite inputs.
    if (direction < 0.0f)
    {
        direction += 360.0f;
    }


    // ASTM / OpenDroneID direction resolution:
    // 1 degree
    unsigned int directionInt =
        static_cast<unsigned int>(
            roundf(direction)
        );


    // Example: 359.6 deg rounds to 360.
    // 360 deg is equivalent to 0 deg.
    if (directionInt == 360)
    {
        directionInt = 0;
    }


    if (directionInt < 180)
    {
        out.ewDirectionSegmentBit = 0;

        out.trackDirectionCode =
            static_cast<uint8_t>(
                directionInt
            );
    }
    else
    {
        out.ewDirectionSegmentBit = 1;

        out.trackDirectionCode =
            static_cast<uint8_t>(
                directionInt - 180
            );
    }
}
else
{
    // Unknown Direction semantic = 361 deg.
    //
    // Encoded representation:
    // EW = 1
    // Direction = 181
    //
    // Decoder:
    // 181 + 180 = 361 deg -> Unknown

    out.ewDirectionSegmentBit = 1;
    out.trackDirectionCode = 181;
}
    


    // --------------------------------------------------
    // bit 0
    // Speed Multiplier
    //
    // ASTM:
    //
    // <= 63.75 m/s       -> multiplier 0
    // >  63.75 m/s       -> multiplier 1
    //
    // Unknown speed will later use:
    // multiplier = 1 and Byte 3 = 255.
    // --------------------------------------------------
// --------------------------------------------------
// Byte 1 bit 0 + Byte 3
// Horizontal Speed
//
// Speed Multiplier and Speed Code
// must be mapped together.
// --------------------------------------------------

if (!state.horizontalSpeedValid ||
    !isfinite(state.horizontalSpeedMps) ||
    state.horizontalSpeedMps < 0.0f)
{
    // Unknown speed
    out.speedMultiplierBit = 1;
    out.horizontalSpeedCode = 255;
}
else
{
    float speed = state.horizontalSpeedMps;

    if (speed <= 63.75f)
    {
        // Range 1:
        // resolution = 0.25 m/s

        out.speedMultiplierBit = 0;

        unsigned int code =
            static_cast<unsigned int>(
                roundf(speed / 0.25f)
            );

        if (code > 255)
        {
            code = 255;
        }

        out.horizontalSpeedCode =
            static_cast<uint8_t>(code);
    }
    else if (speed < 254.25f)
    {
        // Range 2:
        // resolution = 0.75 m/s

        out.speedMultiplierBit = 1;

        unsigned int code =
            static_cast<unsigned int>(
                roundf(
                    (speed - 63.75f) / 0.75f
                )
            );

        // 255 is reserved for Unknown
        if (code > 254)
        {
            code = 254;
        }

        out.horizontalSpeedCode =
            static_cast<uint8_t>(code);
    }
    else
    {
        // Saturation
        out.speedMultiplierBit = 1;
        out.horizontalSpeedCode = 254;
    }
}


// --------------------------------------------------
// Byte 4
// Vertical Speed
// --------------------------------------------------

if (!state.verticalSpeedValid ||
    !isfinite(state.verticalSpeedMps))
{
    // ASTM/OpenDroneID Invalid / Unknown
    // semantic value = 63 m/s
    // encoded = 63 / 0.5 = 126
    out.verticalSpeedCode = 126;
}
else
{
    float speed = state.verticalSpeedMps;

    // Representable Remote ID range
    if (speed > 62.0f)
    {
        speed = 62.0f;
    }
    else if (speed < -62.0f)
    {
        speed = -62.0f;
    }

    int code =
        static_cast<int>(
            roundf(speed / 0.5f)
        );

    out.verticalSpeedCode =
        static_cast<int8_t>(code);
}
  // --------------------------------------------------
// Position validity
//
// Latitude and longitude form one position.
// Validate them together.
// --------------------------------------------------

bool positionUsable =
    state.positionValid &&
    isfinite(state.latitudeDeg) &&
    isfinite(state.longitudeDeg) &&
    state.latitudeDeg >= -90.0 &&
    state.latitudeDeg <= 90.0 &&
    state.longitudeDeg >= -180.0 &&
    state.longitudeDeg <= 180.0;

out.positionValid = positionUsable;


// --------------------------------------------------
// Bytes 5-8
// Latitude
//
// Resolution: 1e-7 degree
// --------------------------------------------------

if (positionUsable)
{
    out.latitudeCode =
        static_cast<int32_t>(
            llround(
                state.latitudeDeg *
                10000000.0
            )
        );
}
else
{
    out.latitudeCode = 0;
}


// --------------------------------------------------
// Bytes 9-12
// Longitude
//
// Resolution: 1e-7 degree
// --------------------------------------------------

if (positionUsable)
{
    out.longitudeCode =
        static_cast<int32_t>(
            llround(
                state.longitudeDeg *
                10000000.0
            )
        );
}
else
{
    out.longitudeCode = 0;
}

// --------------------------------------------------
// Bytes 13-14
// Pressure Altitude
// --------------------------------------------------

out.pressureAltitudeCode =
    mapAltitude(
        state.pressureAltitudeM,
        state.pressureAltitudeValid
    );
// --------------------------------------------------
// Bytes 15-16
// Geodetic Altitude (HAE)
// --------------------------------------------------
out.geodeticAltitudeCode =
    mapAltitude(
        state.geodeticAltitudeM,
        state.geodeticAltitudeValid
    );

// --------------------------------------------------
// Bytes 17-18
// Height
//
// Meaning is selected by heightTypeBit in Byte 1.
// --------------------------------------------------
 out.heightCode =
    mapAltitude(
        state.heightM,
        state.heightValid
    );

// --------------------------------------------------
// Byte 19
// Horizontal + Vertical Accuracy
//
// Source values are physical accuracy estimates
// stored in DroneState.
//
// V1 policy:
// classify directly using ASTM thresholds,
// following the ArduPilot-compatible approach.
// --------------------------------------------------

if (state.horizontalAccuracyValid)
{
    out.horizontalAccuracyCode =
        mapHorizontalAccuracy(
            state.horizontalAccuracyM
        );
}
else
{
    out.horizontalAccuracyCode = 0;
}


if (state.verticalAccuracyValid)
{
    out.verticalAccuracyCode =
        mapVerticalAccuracy(
            state.verticalAccuracyM
        );
}
else
{
    out.verticalAccuracyCode = 0;
}

// --------------------------------------------------
// Byte 20
// Barometric Altitude Accuracy + Speed Accuracy
// --------------------------------------------------

if (state.barometricAltitudeAccuracyValid)
{
    out.barometricAltitudeAccuracyCode =
    mapVerticalAccuracy(
        state.barometricAltitudeAccuracyM
    );
}
else
{
    out.barometricAltitudeAccuracyCode = 0;
}


if (state.speedAccuracyValid)
{
    out.speedAccuracyCode =
        mapSpeedAccuracy(
            state.speedAccuracyMps
        );
}
else
{
    out.speedAccuracyCode = 0;
}

// --------------------------------------------------
// Bytes 21-22
// Location Timestamp
//
// DroneState contains full UTC time of the
// GLOBAL_POSITION_INT sample.
//
// Convert to tenths of seconds since UTC hour.
// --------------------------------------------------

if (state.locationTimestampValid)
{
    constexpr uint64_t US_PER_HOUR =
        3600ULL * 1000000ULL;

    uint64_t usSinceHour =
        state.locationTimestampUtcUs %
        US_PER_HOUR;

    double secondsSinceHour =
        static_cast<double>(usSinceHour) /
        1000000.0;

    long code =
        lround(
            secondsSinceHour * 10.0
        );

    // OpenDroneID valid range is up to 3600 s.
    if (code < 0)
    {
        code = 0;
    }
    else if (code > 36000)
    {
        code = 36000;
    }

    out.timestampCode =
        static_cast<uint16_t>(code);

    out.timestampValid = true;
}
else
{
    out.timestampCode = 0xFFFF;
    out.timestampValid = false;
}

// --------------------------------------------------
// Byte 23
// Timestamp Accuracy
// --------------------------------------------------

if (state.timestampAccuracyValid)
{
    out.timestampAccuracyCode =
        mapTimestampAccuracy(
            state.timestampAccuracyS
        );
}
else
{
    out.timestampAccuracyCode = 0;
}

    return out;
}

///////////////////////////// SYSTEM /////////////////////////////////////////////////////////////////////////////////////

namespace
{

// ============================================================
// ASTM System Timestamp Epoch
//
// 2019-01-01 00:00:00 UTC
//
// Unix:
// 1546300800 seconds
// ============================================================

constexpr uint64_t ASTM_EPOCH_2019_S =
    1546300800ULL;


// ============================================================
// End of ASTM timestamp range.
//
// uint32_t seconds from 2019 epoch.
// ============================================================

constexpr uint64_t ASTM_TIMESTAMP_MAX =
    0xFFFFFFFFULL;


// ============================================================
// Latitude mapping
//
// physical degree
//      ↓
// signed int32 degree * 1e7
// ============================================================

int32_t mapLatitude(
    double latitudeDeg
)
{
    return static_cast<int32_t>(
        llround(
            latitudeDeg *
            10000000.0
        )
    );
}


// ============================================================
// Longitude mapping
// ============================================================

int32_t mapLongitude(
    double longitudeDeg
)
{
    return static_cast<int32_t>(
        llround(
            longitudeDeg *
            10000000.0
        )
    );
}


// ============================================================
// Altitude mapping
//
// ASTM:
// range:
//     -1000 ... 31767 m
//
// encoded:
//     (altitude + 1000) / 0.5
//
// 0 is used for Unknown.
//
// 31767 m:
//     (31767 + 1000) / 0.5
//     = 65534
//
// Therefore 65534 is the highest valid code.
// ============================================================

uint16_t mapSystemAltitude(
    float altitudeM,
    bool valid
)
{
    if (
        !valid ||
        !isfinite(altitudeM)
    )
    {
        return 0;
    }


    // ASTM lower bound / Unknown
    if (altitudeM <= -1000.0f)
    {
        return 0;
    }


    // ASTM upper bound
    if (altitudeM > 31767.0f)
    {
        altitudeM = 31767.0f;
    }


    float encoded =
        (altitudeM + 1000.0f) /
        0.5f;


    long code =
        lroundf(encoded);


    // 0 is reserved for Unknown.
    if (code < 1)
    {
        code = 1;
    }


    if (code > 65534)
    {
        code = 65534;
    }


    return static_cast<uint16_t>(
        code
    );
}


// ============================================================
// Area Radius
//
// ASTM:
// 1 code = 10 m
//
// maximum = 2.5 km
//         = 2500 m
//         = code 250
// ============================================================

uint8_t mapAreaRadius(
    float radiusM
)
{
    if (
        !isfinite(radiusM) ||
        radiusM <= 0.0f
    )
    {
        return 0;
    }


    if (radiusM > 2500.0f)
    {
        radiusM = 2500.0f;
    }


    long code =
        lroundf(
            radiusM / 10.0f
        );


    if (code < 0)
    {
        code = 0;
    }
    else if (code > 250)
    {
        code = 250;
    }


    return static_cast<uint8_t>(
        code
    );
}


// ============================================================
// System Timestamp
//
// Input:
// Unix timestamp in microseconds.
//
// Output:
// uint32 seconds since ASTM epoch
// 2019-01-01 00:00:00 UTC.
//
// IMPORTANT:
// System timestamp is different from Location timestamp.
//
// Location:
//     0.1 s since current UTC hour.
//
// System:
//     seconds since 2019 epoch.
// ============================================================

bool mapSystemTimestamp(
    uint64_t utcUs,
    uint32_t& outCode
)
{
    if (utcUs == 0)
    {
        outCode = 0;
        return false;
    }


    uint64_t unixSeconds =
        utcUs / 1000000ULL;


    if (
        unixSeconds <
        ASTM_EPOCH_2019_S
    )
    {
        outCode = 0;
        return false;
    }


    uint64_t secondsSince2019 =
        unixSeconds -
        ASTM_EPOCH_2019_S;


    if (
        secondsSince2019 >
        ASTM_TIMESTAMP_MAX
    )
    {
        outCode = 0;
        return false;
    }


    outCode =
        static_cast<uint32_t>(
            secondsSince2019
        );


    return true;
}

} // namespace



RidSystemMapped RidSystemMapper::map(
    const RidSystemSourceV1& source
) const
{
    RidSystemMapped out;


    // ========================================================
    // BYTE 1
    // ========================================================


    // --------------------------------------------------------
    // Bits 4..2
    // Classification Type
    // --------------------------------------------------------

    switch (source.classificationType)
    {
        case RidClassificationType::EUROPEAN_UNION:
        {
            out.classificationTypeCode = 1;
            break;
        }


        case RidClassificationType::UNDECLARED:
        default:
        {
            out.classificationTypeCode = 0;
            break;
        }
    }


    // --------------------------------------------------------
    // Bits 1..0
    // Operator Location / Altitude Source Type
    // --------------------------------------------------------

    switch (source.operatorLocationType)
    {
        case RidOperatorLocationType::DYNAMIC:
        {
            out.operatorLocationTypeCode = 1;
            break;
        }


        case RidOperatorLocationType::FIXED:
        {
            out.operatorLocationTypeCode = 2;
            break;
        }


        case RidOperatorLocationType::TAKEOFF:
        default:
        {
            out.operatorLocationTypeCode = 0;
            break;
        }
    }


    // ========================================================
    // Operator position validation
    // ========================================================

    bool positionUsable =
        source.operatorPositionValid &&
        isfinite(source.operatorLatitudeDeg) &&
        isfinite(source.operatorLongitudeDeg) &&

        source.operatorLatitudeDeg >= -90.0 &&
        source.operatorLatitudeDeg <= 90.0 &&

        source.operatorLongitudeDeg >= -180.0 &&
        source.operatorLongitudeDeg <= 180.0;


    out.operatorPositionValid =
        positionUsable;


    // ========================================================
    // Bytes 2-5
    // Operator Latitude
    // ========================================================

    if (positionUsable)
    {
        out.operatorLatitudeCode =
            mapLatitude(
                source.operatorLatitudeDeg
            );
    }
    else
    {
        out.operatorLatitudeCode = 0;
    }


    // ========================================================
    // Bytes 6-9
    // Operator Longitude
    // ========================================================

    if (positionUsable)
    {
        out.operatorLongitudeCode =
            mapLongitude(
                source.operatorLongitudeDeg
            );
    }
    else
    {
        out.operatorLongitudeCode = 0;
    }


    // ========================================================
    // Bytes 10-11
    // Area Count
    //
    // ASTM:
    // default = 1
    // maximum = 65000
    // ========================================================

    if (
        source.areaCount == 0
    )
    {
        out.areaCountCode = 1;
    }
    else if (
        source.areaCount > 65000
    )
    {
        out.areaCountCode = 65000;
    }
    else
    {
        out.areaCountCode =
            source.areaCount;
    }


    // ========================================================
    // Byte 12
    // Area Radius
    // ========================================================

    out.areaRadiusCode =
        mapAreaRadius(
            source.areaRadiusM
        );


    // ========================================================
    // Bytes 13-14
    // Area Ceiling
    // ========================================================

    out.areaCeilingCode =
        mapSystemAltitude(
            source.areaCeilingM,
            source.areaCeilingValid
        );


    // ========================================================
    // Bytes 15-16
    // Area Floor
    // ========================================================

    out.areaFloorCode =
        mapSystemAltitude(
            source.areaFloorM,
            source.areaFloorValid
        );


    // ========================================================
    // Byte 17
    // UA Classification
    //
    // Only meaningful when:
    //
    // Classification Type == European Union
    // ========================================================

    if (
        source.classificationType ==
        RidClassificationType::EUROPEAN_UNION
    )
    {
        // Category
        //
        // 0 Undefined
        // 1 Open
        // 2 Specific
        // 3 Certified
        //
        // 4..15 Reserved

        if (source.categoryEU <= 3)
        {
            out.categoryCode =
                source.categoryEU;
        }
        else
        {
            out.categoryCode = 0;
        }


        // Class
        //
        // 0 Undefined
        // 1 Class 0
        // ...
        // 7 Class 6
        //
        // 8..15 Reserved

        if (source.classEU <= 7)
        {
            out.classCode =
                source.classEU;
        }
        else
        {
            out.classCode = 0;
        }
    }
    else
    {
        // Classification Type Undeclared:
        // Byte 17 should carry no EU classification.

        out.categoryCode = 0;
        out.classCode = 0;
    }


    // ========================================================
    // Bytes 18-19
    // Operator Altitude
    //
    // IMPORTANT:
    // WGS84 HAE, NOT MSL altitude.
    // ========================================================

    out.operatorAltitudeValid =
        source.operatorAltitudeValid &&
        isfinite(source.operatorAltitudeM);


    out.operatorAltitudeCode =
        mapSystemAltitude(
            source.operatorAltitudeM,
            out.operatorAltitudeValid
        );


    // ========================================================
    // Bytes 20-23
    // Timestamp
    // ========================================================

    if (source.timestampValid)
    {
        out.timestampValid =
            mapSystemTimestamp(
                source.timestampUtcUs,
                out.timestampCode
            );
    }
    else
    {
        out.timestampCode = 0;
        out.timestampValid = false;
    }


    return out;
}

/////////////////// Self Id /////////////////////////////////////////////////////////////////////////////////////////

RidSelfIdMapped RidSelfIdMapper::map(
    const RidSelfIdSource& source
) const
{
    RidSelfIdMapped out;


    if (!source.valid)
    {
        return out;
    }


    // ==================================================
    // Description Type
    // ==================================================

    uint8_t type =
        static_cast<uint8_t>(
            source.descriptionType
        );


    // Current standard values used by this project.
    if (type > 2)
    {
        return out;
    }


    out.descriptionTypeCode =
        type;


    // ==================================================
    // Description
    // Maximum wire length = 23 bytes
    // ==================================================

    size_t length =
        strnlen(
            source.description,
            23
        );


    if (length == 0)
    {
        return out;
    }


    memcpy(
        out.description,
        source.description,
        length
    );


    // Remaining bytes stay 0x00.

    out.valid = true;

    return out;
}