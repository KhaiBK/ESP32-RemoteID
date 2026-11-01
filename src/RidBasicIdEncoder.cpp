#include "RidBasicIdEncoder.h"

#include <string.h>

namespace
{
constexpr uint8_t BASIC_ID_MESSAGE_TYPE = 0;
constexpr uint8_t PROTOCOL_VERSION = 2;

size_t identifierLength(const RidBasicIdV1& identity)
{
    size_t length = 0;
    while (length < sizeof(identity.uasId) && identity.uasId[length] != '\0')
    {
        ++length;
    }
    return length;
}

bool isUpperAlphanumeric(char c)
{
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z');
}

bool isSerialCharacter(char c)
{
    return isUpperAlphanumeric(c) && c != 'I' && c != 'O';
}
} // namespace

RidBasicIdEncodeResult RidBasicIdEncoder::validate(const RidBasicIdV1& identity) const
{
    if (identity.idType != RidBasicIdType::SERIAL_NUMBER &&
        identity.idType != RidBasicIdType::CAA_REGISTRATION)
    {
        return RidBasicIdEncodeResult::UNSUPPORTED_ID_TYPE;
    }
    if (static_cast<uint8_t>(identity.uaType) > 15)
    {
        return RidBasicIdEncodeResult::INVALID_UA_TYPE;
    }

    const size_t length = identifierLength(identity);
    if (length == 0)
    {
        return RidBasicIdEncodeResult::EMPTY_IDENTIFIER;
    }
    if (length > RidBasicIdV1::UAS_ID_SIZE)
    {
        return RidBasicIdEncodeResult::IDENTIFIER_TOO_LONG;
    }
    for (size_t i = length + 1; i < sizeof(identity.uasId); ++i)
    {
        if (identity.uasId[i] != '\0')
        {
            return RidBasicIdEncodeResult::INVALID_IDENTIFIER;
        }
    }

    if (identity.idType == RidBasicIdType::SERIAL_NUMBER)
    {
        // CTA-2063-A: four manufacturer characters, one length code (1..F),
        // and 1..15 serial characters. Manufacturer/serial exclude I and O.
        if (length < 6)
        {
            return RidBasicIdEncodeResult::SERIAL_LENGTH_MISMATCH;
        }
        const char code = identity.uasId[4];
        size_t serialLength;
        if (code >= '1' && code <= '9')
        {
            serialLength = static_cast<size_t>(code - '0');
        }
        else if (code >= 'A' && code <= 'F')
        {
            serialLength = static_cast<size_t>(code - 'A') + 10;
        }
        else
        {
            return RidBasicIdEncodeResult::INVALID_SERIAL_LENGTH_CODE;
        }
        if (length != 5 + serialLength)
        {
            return RidBasicIdEncodeResult::SERIAL_LENGTH_MISMATCH;
        }
        for (size_t i = 0; i < length; ++i)
        {
            if (i != 4 && !isSerialCharacter(identity.uasId[i]))
            {
                return RidBasicIdEncodeResult::INVALID_IDENTIFIER;
            }
        }
    }
    else
    {
        // Structural CAA format: nonempty nationality mark, '.', nonempty ID.
        // Checking actual national assignment/registration requires external data.
        size_t separator = length;
        for (size_t i = 0; i < length; ++i)
        {
            if (identity.uasId[i] == '.')
            {
                if (separator != length)
                {
                    return RidBasicIdEncodeResult::INVALID_REGISTRATION_FORMAT;
                }
                separator = i;
            }
            else if (!isUpperAlphanumeric(identity.uasId[i]))
            {
                return RidBasicIdEncodeResult::INVALID_IDENTIFIER;
            }
        }
        if (separator == length || separator == 0 || separator + 1 == length)
        {
            return RidBasicIdEncodeResult::INVALID_REGISTRATION_FORMAT;
        }
    }
    return RidBasicIdEncodeResult::SUCCESS;
}

RidBasicIdEncodeResult RidBasicIdEncoder::encode(
    const RidBasicIdV1& identity,
    uint8_t* out,
    size_t outSize
) const
{
    if (out == nullptr || outSize < MESSAGE_SIZE)
    {
        return RidBasicIdEncodeResult::INVALID_OUTPUT_BUFFER;
    }
    memset(out, 0, MESSAGE_SIZE);
    const RidBasicIdEncodeResult result = validate(identity);
    if (result != RidBasicIdEncodeResult::SUCCESS)
    {
        return result;
    }

    out[0] = (BASIC_ID_MESSAGE_TYPE << 4) | PROTOCOL_VERSION;
    out[1] = (static_cast<uint8_t>(identity.idType) << 4) |
             static_cast<uint8_t>(identity.uaType);
    memcpy(out + 2, identity.uasId, identifierLength(identity));
    return RidBasicIdEncodeResult::SUCCESS;
}

const char* RidBasicIdEncoder::resultName(RidBasicIdEncodeResult result)
{
    switch (result)
    {
        case RidBasicIdEncodeResult::SUCCESS: return "SUCCESS";
        case RidBasicIdEncodeResult::INVALID_OUTPUT_BUFFER: return "INVALID_OUTPUT_BUFFER";
        case RidBasicIdEncodeResult::UNSUPPORTED_ID_TYPE: return "UNSUPPORTED_ID_TYPE";
        case RidBasicIdEncodeResult::INVALID_UA_TYPE: return "INVALID_UA_TYPE";
        case RidBasicIdEncodeResult::EMPTY_IDENTIFIER: return "EMPTY_IDENTIFIER";
        case RidBasicIdEncodeResult::IDENTIFIER_TOO_LONG: return "IDENTIFIER_TOO_LONG";
        case RidBasicIdEncodeResult::INVALID_IDENTIFIER: return "INVALID_IDENTIFIER";
        case RidBasicIdEncodeResult::INVALID_SERIAL_LENGTH_CODE: return "INVALID_SERIAL_LENGTH_CODE";
        case RidBasicIdEncodeResult::SERIAL_LENGTH_MISMATCH: return "SERIAL_LENGTH_MISMATCH";
        case RidBasicIdEncodeResult::INVALID_REGISTRATION_FORMAT: return "INVALID_REGISTRATION_FORMAT";
        default: return "UNKNOWN_RESULT";
    }
}
