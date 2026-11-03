#pragma once

#include <stdint.h>
#include <stddef.h>


// ============================================================
// ASTM F3411 System mapped representation
//
// Values here have already been converted into ASTM codes,
// but have NOT yet been packed into the final 25-byte packet.
// ============================================================

struct RidSystemMapped
{
    // ========================================================
    // Byte 1
    //
    // bits 7..5 : Reserved = 0
    // bits 4..2 : Classification Type
    // bits 1..0 : Operator Location / Altitude Source Type
    // ========================================================

    uint8_t classificationTypeCode = 0;

    uint8_t operatorLocationTypeCode = 0;


    // ========================================================
    // Operator position validity
    //
    // NOT a wire field.
    // ========================================================

    bool operatorPositionValid = false;


    // ========================================================
    // Bytes 2-5
    // Operator Latitude
    //
    // signed int32
    // resolution = 1e-7 degree
    // ========================================================

    int32_t operatorLatitudeCode = 0;


    // ========================================================
    // Bytes 6-9
    // Operator Longitude
    //
    // signed int32
    // resolution = 1e-7 degree
    // ========================================================

    int32_t operatorLongitudeCode = 0;


    // ========================================================
    // Bytes 10-11
    // Area Count
    // ========================================================

    uint16_t areaCountCode = 1;


    // ========================================================
    // Byte 12
    // Area Radius
    //
    // resolution = 10 m
    //
    // 0   -> 0 m
    // 1   -> 10 m
    // ...
    // 250 -> 2500 m
    // ========================================================

    uint8_t areaRadiusCode = 0;


    // ========================================================
    // Bytes 13-14
    // Area Ceiling
    //
    // WGS84 HAE
    //
    // code = round((altitude + 1000) / 0.5)
    //
    // code = 0 -> Unknown
    // ========================================================

    uint16_t areaCeilingCode = 0;


    // ========================================================
    // Bytes 15-16
    // Area Floor
    //
    // WGS84 HAE
    // ========================================================

    uint16_t areaFloorCode = 0;


    // ========================================================
    // Byte 17
    //
    // bits 7..4 : Category
    // bits 3..0 : Class
    // ========================================================

    uint8_t categoryCode = 0;
    uint8_t classCode = 0;


    // ========================================================
    // Bytes 18-19
    // Operator Altitude
    //
    // WGS84 HAE
    // ========================================================

    bool operatorAltitudeValid = false;

    uint16_t operatorAltitudeCode = 0;


    // ========================================================
    // Bytes 20-23
    // Timestamp
    //
    // Seconds since:
    // 2019-01-01 00:00:00 UTC
    //
    // NOT Unix epoch directly.
    // ========================================================

    bool timestampValid = false;

    uint32_t timestampCode = 0;
};
