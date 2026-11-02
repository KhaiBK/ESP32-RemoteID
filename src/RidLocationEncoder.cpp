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

////////////////////////// SYSTEM /////////////////////////////////////////////////////////////////////////////////////

namespace
{

constexpr uint8_t SYSTEM_MESSAGE_TYPE = 4;
constexpr uint8_t SYSTEM_PROTOCOL_VERSION = 2;


// ======================================================
// Write signed int32 little-endian
// ======================================================

void Sys_writeInt32LE(
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
// Write uint16 little-endian
// ======================================================

void Sys_writeUInt16LE(
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


// ======================================================
// Write uint32 little-endian
// ======================================================

void Sys_writeUInt32LE(
    uint8_t* out,
    uint32_t value
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


    out[2] =
        static_cast<uint8_t>(
            (value >> 16) & 0xFF
        );


    out[3] =
        static_cast<uint8_t>(
            (value >> 24) & 0xFF
        );
}

} // namespace



static_assert(
    RidSystemEncoder::MESSAGE_SIZE == 25,
    "Remote ID System message must be 25 bytes"
);



void RidSystemEncoder::encode(
    const RidSystemMapped& mapped,
    uint8_t out[MESSAGE_SIZE]
) const
{
    // ==================================================
    // Clear complete packet
    // ==================================================

    memset(
        out,
        0,
        MESSAGE_SIZE
    );


    // ==================================================
    // Byte 0
    //
    // bits 7..4 : Message Type = System = 4
    // bits 3..0 : Protocol Version = 2
    //
    // => 0x42
    // ==================================================

    out[0] =
        static_cast<uint8_t>(
            ((SYSTEM_MESSAGE_TYPE & 0x0F) << 4)
            |
            (SYSTEM_PROTOCOL_VERSION & 0x0F)
        );


    // ==================================================
    // Byte 1
    //
    // bits 7..5 : Reserved = 0
    // bits 4..2 : Classification Type
    // bits 1..0 : Operator Location / Altitude Source
    // ==================================================

    out[1] =
        static_cast<uint8_t>(
            ((mapped.classificationTypeCode & 0x07) << 2)
            |
            (mapped.operatorLocationTypeCode & 0x03)
        );


    // ==================================================
    // Bytes 2-5
    // Operator Latitude
    // ==================================================

    Sys_writeInt32LE(
        &out[2],
        mapped.operatorLatitudeCode
    );


    // ==================================================
    // Bytes 6-9
    // Operator Longitude
    // ==================================================

    Sys_writeInt32LE(
        &out[6],
        mapped.operatorLongitudeCode
    );


    // ==================================================
    // Bytes 10-11
    // Area Count
    // ==================================================

    Sys_writeUInt16LE(
        &out[10],
        mapped.areaCountCode
    );


    // ==================================================
    // Byte 12
    // Area Radius
    // ==================================================

    out[12] =
        mapped.areaRadiusCode;


    // ==================================================
    // Bytes 13-14
    // Area Ceiling
    // ==================================================

    Sys_writeUInt16LE(
        &out[13],
        mapped.areaCeilingCode
    );


    // ==================================================
    // Bytes 15-16
    // Area Floor
    // ==================================================

    Sys_writeUInt16LE(
        &out[15],
        mapped.areaFloorCode
    );


    // ==================================================
    // Byte 17
    //
    // bits 7..4 : Category
    // bits 3..0 : Class
    // ==================================================

    out[17] =
        static_cast<uint8_t>(
            ((mapped.categoryCode & 0x0F) << 4)
            |
            (mapped.classCode & 0x0F)
        );


    // ==================================================
    // Bytes 18-19
    // Operator Altitude
    // ==================================================

    Sys_writeUInt16LE(
        &out[18],
        mapped.operatorAltitudeCode
    );


    // ==================================================
    // Bytes 20-23
    // Timestamp
    //
    // uint32 little-endian
    // seconds since 2019-01-01 UTC
    // ==================================================

    Sys_writeUInt32LE(
        &out[20],
        mapped.timestampCode
    );


    // ==================================================
    // Byte 24
    // Reserved
    // ==================================================

    out[24] = 0x00;
}

///////////////////////////// Self Id ////////////////////////////////////////////////////////////////////////////////////

namespace
{

constexpr uint8_t SELF_ID_MESSAGE_TYPE = 3;
constexpr uint8_t SELF_PROTOCOL_VERSION = 2;

}


static_assert(
    RidSelfIdEncoder::MESSAGE_SIZE == 25,
    "Remote ID Self ID message must be 25 bytes"
);


bool RidSelfIdEncoder::encode(
    const RidSelfIdMapped& mapped,
    uint8_t out[MESSAGE_SIZE]
) const
{
    if (out == nullptr)
    {
        return false;
    }


    memset(
        out,
        0,
        MESSAGE_SIZE
    );


    if (!mapped.valid)
    {
        return false;
    }


    // ==================================================
    // Byte 0
    //
    // Message Type = 3
    // Protocol     = 2
    //
    // 0011 0010 = 0x32
    // ==================================================

    out[0] =
        static_cast<uint8_t>(
            ((SELF_ID_MESSAGE_TYPE & 0x0F) << 4)
            |
            (SELF_PROTOCOL_VERSION & 0x0F)
        );


    // ==================================================
    // Byte 1
    // Description Type
    // ==================================================

    out[1] =
        mapped.descriptionTypeCode;


    // ==================================================
    // Bytes 2..24
    // Description
    // ==================================================

    memcpy(
        &out[2],
        mapped.description,
        23
    );


    return true;
}