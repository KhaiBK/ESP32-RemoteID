#include "RidAuthEncoder.h"

#include <string.h>

namespace
{

constexpr uint8_t AUTH_MESSAGE_TYPE = 2;
constexpr uint8_t PROTOCOL_VERSION = 2;

} // namespace

static_assert(
    RidAuthEncoder::MESSAGE_SIZE == 25,
    "Remote ID Authentication message must be 25 bytes"
);

static_assert(
    RidAuthEncoder::PAGE_0_DATA_SIZE == 17,
    "Page 0 data size must be 17 bytes"
);

static_assert(
    RidAuthEncoder::PAGE_N_DATA_SIZE == 23,
    "Page N data size must be 23 bytes"
);

size_t RidAuthEncoder::calculateTotalPages(size_t authDataLength) const
{
    if (authDataLength > MAX_AUTH_DATA_SIZE)
    {
        return 0;
    }

    if (authDataLength <= PAGE_0_DATA_SIZE)
    {
        return 1;
    }

    const size_t remaining = authDataLength - PAGE_0_DATA_SIZE;
    const size_t extraPages = (remaining + PAGE_N_DATA_SIZE - 1) / PAGE_N_DATA_SIZE;

    return 1 + extraPages;
}

uint32_t RidAuthEncoder::toAstmTimestamp(uint64_t unixTimestampSeconds)
{
    if (unixTimestampSeconds >= ASTM_EPOCH_OFFSET)
    {
        return static_cast<uint32_t>(unixTimestampSeconds - ASTM_EPOCH_OFFSET);
    }

    return 0;
}

bool RidAuthEncoder::encodePage(
    RidAuthType authType,
    uint8_t pageIndex,
    uint32_t timestamp,
    const uint8_t* authData,
    size_t authDataLength,
    uint8_t out[MESSAGE_SIZE]
) const
{
    if (out == nullptr)
    {
        return false;
    }

    if (authDataLength > MAX_AUTH_DATA_SIZE)
    {
        return false;
    }

    // Standard BMG0120 & BMG0180: None and Network RID require empty auth data
    if ((authType == RidAuthType::NONE || authType == RidAuthType::NETWORK_REMOTE_ID) &&
        authDataLength != 0)
    {
        return false;
    }

    if (authDataLength > 0 && authData == nullptr)
    {
        return false;
    }

    const size_t totalPages = calculateTotalPages(authDataLength);
    if (totalPages == 0 || totalPages > MAX_PAGES)
    {
        return false;
    }

    const uint8_t lastPageIndex = static_cast<uint8_t>(totalPages - 1);
    if (pageIndex > lastPageIndex)
    {
        return false;
    }

    memset(out, 0, MESSAGE_SIZE);

    // Byte 0: Header (Message Type 0x2, Protocol Version 0x2 -> 0x22)
    out[0] = static_cast<uint8_t>(
        ((AUTH_MESSAGE_TYPE & 0x0F) << 4) |
        (PROTOCOL_VERSION & 0x0F)
    );

    if (pageIndex == 0)
    {
        // Table 8: Page 0 Details
        // Byte 1: AuthType [7..4] | PageNumber=0 [3..0]
        out[1] = static_cast<uint8_t>(
            ((static_cast<uint8_t>(authType) & 0x0F) << 4) | (0x00 & 0x0F)
        );

        // Byte 2: Reserved [7..4] | Last Page Index [3..0]
        out[2] = static_cast<uint8_t>(lastPageIndex & 0x0F);

        // Byte 3: Length (bytes)
        out[3] = static_cast<uint8_t>(authDataLength);

        // Bytes 4..7: Timestamp (32-bit uint LE, seconds since 2019-01-01)
        out[4] = static_cast<uint8_t>(timestamp & 0xFF);
        out[5] = static_cast<uint8_t>((timestamp >> 8) & 0xFF);
        out[6] = static_cast<uint8_t>((timestamp >> 16) & 0xFF);
        out[7] = static_cast<uint8_t>((timestamp >> 24) & 0xFF);

        // Bytes 8..24: Authentication Data / Signature (up to 17 bytes)
        const size_t bytesToCopy = (authDataLength < PAGE_0_DATA_SIZE)
            ? authDataLength
            : PAGE_0_DATA_SIZE;

        if (bytesToCopy > 0 && authData != nullptr)
        {
            memcpy(&out[8], authData, bytesToCopy);
        }
    }
    else
    {
        // Table 9: Pages 1 through 15 Details
        // Byte 1: AuthType [7..4] | PageNumber [3..0]
        out[1] = static_cast<uint8_t>(
            ((static_cast<uint8_t>(authType) & 0x0F) << 4) | (pageIndex & 0x0F)
        );

        // Bytes 2..24: Authentication Data / Signature (23 bytes per page)
        const size_t offset = PAGE_0_DATA_SIZE +
            (static_cast<size_t>(pageIndex) - 1) * PAGE_N_DATA_SIZE;

        if (offset < authDataLength && authData != nullptr)
        {
            const size_t remaining = authDataLength - offset;
            const size_t bytesToCopy = (remaining < PAGE_N_DATA_SIZE)
                ? remaining
                : PAGE_N_DATA_SIZE;

            memcpy(&out[2], &authData[offset], bytesToCopy);
        }
    }

    return true;
}

bool RidAuthEncoder::encodeAllPages(
    RidAuthType authType,
    uint32_t timestamp,
    const uint8_t* authData,
    size_t authDataLength,
    uint8_t out[][MESSAGE_SIZE],
    size_t maxPages,
    size_t* pagesWritten
) const
{
    if (out == nullptr)
    {
        return false;
    }

    const size_t totalPages = calculateTotalPages(authDataLength);
    if (totalPages == 0 || totalPages > maxPages || totalPages > MAX_PAGES)
    {
        return false;
    }

    for (size_t p = 0; p < totalPages; ++p)
    {
        if (!encodePage(
                authType,
                static_cast<uint8_t>(p),
                timestamp,
                authData,
                authDataLength,
                out[p]))
        {
            return false;
        }
    }

    if (pagesWritten != nullptr)
    {
        *pagesWritten = totalPages;
    }

    return true;
}
