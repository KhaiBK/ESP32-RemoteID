#include "RidSystemEncoder.h"
#include <string.h>


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

