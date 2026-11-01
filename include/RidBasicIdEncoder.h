#pragma once

#include "RidBasicId.h"

enum class RidBasicIdEncodeResult : uint8_t
{
    SUCCESS = 0,
    INVALID_OUTPUT_BUFFER,
    UNSUPPORTED_ID_TYPE,
    INVALID_UA_TYPE,
    EMPTY_IDENTIFIER,
    IDENTIFIER_TOO_LONG,
    INVALID_IDENTIFIER,
    INVALID_SERIAL_LENGTH_CODE,
    SERIAL_LENGTH_MISMATCH,
    INVALID_REGISTRATION_FORMAT
};

class RidBasicIdEncoder
{
public:
    static constexpr size_t MESSAGE_SIZE = 25;

    // Syntax validation only. Binary UUID/session types are not supported in V1.
    RidBasicIdEncodeResult validate(const RidBasicIdV1& identity) const;

    // On success, writes exactly 25 bytes. On invalid identity, clears those
    // 25 bytes and returns an error; the caller must not use them as a packet.
    // Null/short buffers are rejected without writes. Input/output must not alias.
    RidBasicIdEncodeResult encode(
        const RidBasicIdV1& identity,
        uint8_t* out,
        size_t outSize
    ) const;

    static const char* resultName(RidBasicIdEncodeResult result);
};
