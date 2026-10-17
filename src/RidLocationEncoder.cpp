#include "RidLocationEncoder.h"

#include <string.h>


namespace
{

constexpr uint8_t LOCATION_MESSAGE_TYPE = 1;
constexpr uint8_t PROTOCOL_VERSION = 2;


// ======================================================
// Write signed 32-bit value in little-endian order
// ======================================================

void writeInt32LE(
    uint8_t* out,
    int32_t value
)
{
    uint32_t raw =
        static_cast<uint32_t>(value);

    out[0] =
        static_cast<uint8_t>(
            raw & 0xFF
        );

    out[1] =
        static_cast<uint8_t>(
            (raw >> 8) & 0xFF
        );

    out[2] =
        static_cast<uint8_t>(
            (raw >> 16) & 0xFF
        );

    out[3] =
        static_cast<uint8_t>(
            (raw >> 24) & 0xFF
        );
}


// ======================================================
// Write unsigned 16-bit value in little-endian order
// ======================================================

void writeUInt16LE(
    uint8_t* out,
    uint16_t value
)
{
    out[0] =
        static_cast<uint8_t>(
            value & 0xFF
        );

    out[1] =
        static_cast<uint8_t>(
            (value >> 8) & 0xFF
        );
}

} // namespace


static_assert(
    RidLocationEncoder::MESSAGE_SIZE == 25,
    "Remote ID Location/Vector message must be 25 bytes"
);


void RidLocationEncoder::encode(
    const RidLocationMapped& mapped,
    uint8_t out[MESSAGE_SIZE]
) const
{
    // ==================================================
    // Initialize complete 25-byte Location/Vector packet
    // ==================================================

    memset(out, 0, MESSAGE_SIZE);


    // ==================================================
    // Byte 0
    //
    // bits 7..4 : Message Type
    // bits 3..0 : Protocol Version
    // ==================================================

    out[0] =
        static_cast<uint8_t>(
            ((LOCATION_MESSAGE_TYPE & 0x0F) << 4) |
            (PROTOCOL_VERSION & 0x0F)
        );


    // ==================================================
    // Byte 1
    //
    // bits 7..4 : Operational Status
    // bit 3     : Reserved = 0
    // bit 2     : Height Type
    // bit 1     : EW Direction Segment
    // bit 0     : Speed Multiplier
    // ==================================================

    out[1] =
        static_cast<uint8_t>(
            ((mapped.operationalStatusCode & 0x0F) << 4) |
            ((mapped.heightTypeBit         & 0x01) << 2) |
            ((mapped.ewDirectionSegmentBit & 0x01) << 1) |
            (mapped.speedMultiplierBit     & 0x01)
        );


    // ==================================================
    // Byte 2
    // Track Direction
    // ==================================================

    out[2] =
        mapped.trackDirectionCode;


    // ==================================================
    // Byte 3
    // Horizontal Speed
    // ==================================================

    out[3] =
        mapped.horizontalSpeedCode;


    // ==================================================
    // Byte 4
    // Vertical Speed
    //
    // int8_t -> uint8_t preserves the raw
    // two's-complement byte representation.
    // ==================================================

    out[4] =
        static_cast<uint8_t>(
            mapped.verticalSpeedCode
        );


    // ==================================================
    // Bytes 5-8
    // Latitude
    //
    // Signed int32, little-endian
    // ==================================================

    writeInt32LE(
        &out[5],
        mapped.latitudeCode
    );


    // ==================================================
    // Bytes 9-12
    // Longitude
    //
    // Signed int32, little-endian
    // ==================================================

    writeInt32LE(
        &out[9],
        mapped.longitudeCode
    );


    // ==================================================
    // Bytes 13-14
    // Pressure / Barometric Altitude
    //
    // uint16_t, little-endian
    // ==================================================

    writeUInt16LE(
        &out[13],
        mapped.pressureAltitudeCode
    );


    // ==================================================
    // Bytes 15-16
    // Geodetic Altitude
    //
    // uint16_t, little-endian
    // ==================================================

    writeUInt16LE(
        &out[15],
        mapped.geodeticAltitudeCode
    );


    // ==================================================
    // Bytes 17-18
    // Height
    //
    // uint16_t, little-endian
    // ==================================================

    writeUInt16LE(
        &out[17],
        mapped.heightCode
    );


    // ==================================================
    // Byte 19
    //
    // bits 7..4 : Vertical Accuracy
    // bits 3..0 : Horizontal Accuracy
    // ==================================================

    out[19] =
        static_cast<uint8_t>(
            ((mapped.verticalAccuracyCode & 0x0F) << 4) |
            (mapped.horizontalAccuracyCode & 0x0F)
        );


    // ==================================================
    // Byte 20
    //
    // bits 7..4 : Barometric Altitude Accuracy
    // bits 3..0 : Speed Accuracy
    // ==================================================

    out[20] =
        static_cast<uint8_t>(
            ((mapped.barometricAltitudeAccuracyCode & 0x0F) << 4) |
            (mapped.speedAccuracyCode & 0x0F)
        );


    // ==================================================
    // Bytes 21-22
    // Location Timestamp
    //
    // uint16_t, little-endian
    // Unit: 0.1 s since UTC hour
    // ==================================================

    writeUInt16LE(
        &out[21],
        mapped.timestampCode
    );


    // ==================================================
    // Byte 23
    //
    // bits 7..4 : Reserved = 0
    // bits 3..0 : Timestamp Accuracy
    // ==================================================

    out[23] =
        static_cast<uint8_t>(
            mapped.timestampAccuracyCode & 0x0F
        );


    // ==================================================
    // Byte 24
    // Reserved
    // ==================================================

    out[24] = 0x00;
}