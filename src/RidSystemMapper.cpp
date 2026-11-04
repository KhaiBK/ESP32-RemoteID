#include "RidSystemMapper.h"
#include <math.h>
#include <limits.h>
#include <string.h>


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
    const RidSystemSource& source
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
