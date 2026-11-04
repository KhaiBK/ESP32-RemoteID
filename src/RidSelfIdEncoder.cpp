#include "RidSelfIdEncoder.h"

#include <string.h>


namespace
{

constexpr uint8_t SELF_ID_MESSAGE_TYPE = 3;
constexpr uint8_t PROTOCOL_VERSION = 2;

}


static_assert(
    RidSelfIdEncoder::MESSAGE_SIZE == 25,
    "Remote ID Self ID message must be 25 bytes"
);


bool RidSelfIdEncoder::encode(
    const RidSelfId& selfId,
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


    // ==================================================
    // Validate source
    // ==================================================

    if (!selfId.valid)
    {
        return false;
    }


    uint8_t descriptionType =
        static_cast<uint8_t>(
            selfId.descriptionType
        );


    // Current supported ASTM values:
    // 0 = Text
    // 1 = Emergency
    // 2 = Extended Status

    if (descriptionType > 2)
    {
        return false;
    }


    size_t length =
        strnlen(
            selfId.description,
            23
        );


    if (length == 0)
    {
        return false;
    }


    // ==================================================
    // Byte 0
    //
    // bits 7..4 = Message Type = 3
    // bits 3..0 = Protocol Version = 2
    //
    // => 0x32
    // ==================================================

    out[0] =
        static_cast<uint8_t>(
            ((SELF_ID_MESSAGE_TYPE & 0x0F) << 4)
            |
            (PROTOCOL_VERSION & 0x0F)
        );


    // ==================================================
    // Byte 1
    // Description Type
    // ==================================================

    out[1] =
        descriptionType;


    // ==================================================
    // Bytes 2..24
    // Description
    //
    // Maximum 23 bytes.
    //
    // Because packet was memset to zero,
    // unused bytes automatically remain 0x00.
    // ==================================================

    memcpy(
        &out[2],
        selfId.description,
        length
    );


    return true;
}