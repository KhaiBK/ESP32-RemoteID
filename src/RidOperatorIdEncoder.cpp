#include "RidOperatorIdEncoder.h"

#include <string.h>

namespace
{
constexpr uint8_t OPERATOR_ID_MESSAGE_TYPE = 5;
constexpr uint8_t PROTOCOL_VERSION = 2;

} // namespace


static_assert(
    RidOperatorIdEncoder::MESSAGE_SIZE == 25,
    "Remote ID Operator ID message must be 25 bytes"
);


bool RidOperatorIdEncoder::encode(
    RidOperatorIdType type,
    const char* operatorId,
    uint8_t out[MESSAGE_SIZE]
) const
{
    // 1. Validate output pointer
    if (out == nullptr)
    {
        return false;
    }

    // 2. Initialize the entire 25-byte buffer
    memset(out, 0, MESSAGE_SIZE);

    // 3. Validate input pointer
    if (operatorId == nullptr)
    {
        return false;
    }

    // 4. Validate Operator ID Type
    // ASTM F3411-22a Table 12:
    // 0       : Operator ID
    // 1-200   : Reserved
    // 201-255 : Private use

    if (type != RidOperatorIdType::OPERATOR_ID)
    {
        return false;
    }

    // 5. Validate Operator ID length and ASCII characters
    size_t length = 0;

    while (length <= OPERATOR_ID_SIZE &&
           operatorId[length] != '\0')
    {
        const uint8_t character =
            static_cast<uint8_t>(operatorId[length]);

        if (character < 0x20 || character > 0x7E)
        {
            return false;
        }

        ++length;
    }

    // 6. Validate length (1-20 characters)
    if (length == 0 || length > OPERATOR_ID_SIZE)
    {
        return false;
    }

    // 7. Encode Message Header
    // Message Type = 5
    // Protocol Version = 2
    // Byte 0 = 0x52

    out[0] = static_cast<uint8_t>(
        ((OPERATOR_ID_MESSAGE_TYPE & 0x0F) << 4) |
        (PROTOCOL_VERSION & 0x0F)
    );

    // 8. Encode Operator ID Type
    // Byte 1 = 0x00

    out[1] = static_cast<uint8_t>(type);

    // 9. Encode Operator ID
    // Bytes 2-21: ASCII characters, zero-padded

    memcpy(&out[2], operatorId, length);

    // 10. Bytes 22-24 remain zero (Reserved)

    return true;
}