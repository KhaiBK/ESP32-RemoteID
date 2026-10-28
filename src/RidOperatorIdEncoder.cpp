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
    memset(out, 0, MESSAGE_SIZE);

    if (operatorId == nullptr ||
        (type != RidOperatorIdType::CAA_ASSIGNED &&
         type != RidOperatorIdType::EU_UAS_OPERATOR_ID))
    {
        return false;
    }

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

    if (length == 0 || length > OPERATOR_ID_SIZE)
    {
        return false;
    }

    out[0] =
        static_cast<uint8_t>(
            ((OPERATOR_ID_MESSAGE_TYPE & 0x0F) << 4) |
            (PROTOCOL_VERSION & 0x0F)
        );
    out[1] = static_cast<uint8_t>(type);
    memcpy(&out[2], operatorId, length);

    return true;
}
