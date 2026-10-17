#pragma once

#include <stdint.h>


// ======================================================
// ASTM F3411 Location/Vector mapped representation
//
// Values in this structure have already been mapped
// according to ASTM rules, but have NOT yet been packed
// into the final 25-byte message.
// ======================================================

struct RidLocationMapped
{
    // ==================================================
    // Byte 1
    // ==================================================

    // bits 7..4
    uint8_t operationalStatusCode = 0;


    // bit 2
    // 0 = Above Takeoff
    // 1 = AGL
    uint8_t heightTypeBit = 0;


    // bit 1
    // 0 = [0, 180)
    // 1 = [180, 360)
    uint8_t ewDirectionSegmentBit = 1;


    // bit 0
    // 0 = normal speed range
    // 1 = extended speed range
    uint8_t speedMultiplierBit = 1;

    // ==================================================

// Byte 2
// Track Direction
//
// Used together with EW Direction Segment from Byte 1.
//
// EW = 0:
//     direction = 0 ... 179 deg
//
// EW = 1:
//     direction = encoded value + 180 deg
//
// Unknown:
//     EW = 1
//     Direction code = 181
//     -> decoded semantic value = 361 deg
// ==================================================

uint8_t trackDirectionCode = 181;

// ==================================================
// Byte 3
// Horizontal Speed
//
// Used together with Speed Multiplier bit in Byte 1.
// ==================================================

uint8_t horizontalSpeedCode = 255;

// ==================================================
// Byte 4
// Vertical Speed
//
// Resolution: 0.5 m/s
// Positive = climb
// Negative = descend
//
// Unknown semantic value = 63 m/s
// Encoded value = 63 / 0.5 = 126
// ==================================================

int8_t verticalSpeedCode = 126;

// ==================================================
// Location validity metadata
//
// This is NOT a wire field.
// It tells later stages whether the mapped position
// is usable.
// ==================================================

bool positionValid = false;


// ==================================================
// Bytes 5-8
// Latitude
//
// Signed 32-bit integer
// Resolution: 1e-7 degree
// ==================================================

int32_t latitudeCode = 0;

// ==================================================
// Bytes 9-12
// Longitude
//
// Signed 32-bit integer
// Resolution: 1e-7 degree
// ==================================================

int32_t longitudeCode = 0;

// ==================================================
// Bytes 13-14
// Pressure / Barometric Altitude
//
// Resolution: 0.5 m
// Offset: -1000 m
//
// code = round((altitude + 1000) / 0.5)
//
// code = 0 -> Unknown
// ==================================================

uint16_t pressureAltitudeCode = 0;

// ==================================================
// Bytes 15-16
// Geodetic Altitude (HAE)
//
// Resolution: 0.5 m
// Offset: -1000 m
//
// code = round((altitude + 1000) / 0.5)
//
// code = 0 -> Unknown
// ==================================================

uint16_t geodeticAltitudeCode = 0;
// ==================================================
// Bytes 17-18
// Height
//
// Interpretation depends on Height Type bit:
//   0 = Above Takeoff
//   1 = AGL
//
// Resolution: 0.5 m
// Offset: -1000 m
//
// code = round((height + 1000) / 0.5)
//
// code = 0 -> Unknown
// ==================================================

uint16_t heightCode = 0;

// ==================================================
// ==================================================
// Byte 19
//
// bits 7..4 : Vertical Accuracy
// bits 3..0 : Horizontal Accuracy
//
// These are ASTM category codes.
// They have NOT yet been packed into Byte 19.
// ==================================================

uint8_t horizontalAccuracyCode = 0;
uint8_t verticalAccuracyCode = 0;

// ==================================================
// Byte 20
//
// bits 7..4 : Barometric Altitude Accuracy
// bits 3..0 : Speed Accuracy
// ==================================================

uint8_t barometricAltitudeAccuracyCode = 0;
uint8_t speedAccuracyCode = 0;

// ==================================================
// Bytes 21-22
// Timestamp
//
// Unit after mapping: 0.1 second since UTC hour.
//
// Valid:
//   code = round(secondsSinceHour * 10)
//
// Unknown:
//   0xFFFF
// ==================================================

bool timestampValid = false;
uint16_t timestampCode = 0xFFFF;
// ==================================================
// Byte 23
//
// bits 7..4 : Reserved
// bits 3..0 : Timestamp Accuracy
//
// 0     = Unknown
// 1..15 = <= 0.1 ... <= 1.5 s
// ==================================================

uint8_t timestampAccuracyCode = 0;
};