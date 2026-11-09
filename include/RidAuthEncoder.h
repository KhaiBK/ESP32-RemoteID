#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Authentication Type defined in ASTM F3411-22a (Table 8 & Table 9, Section 5.4.5)
 */
enum class RidAuthType : uint8_t
{
    NONE = 0,                       // 0: None (Signature empty)
    UAS_ID_SIGNATURE = 1,           // 1: UAS ID Signature
    OPERATOR_ID_SIGNATURE = 2,      // 2: Operator ID Signature
    MESSAGE_SET_SIGNATURE = 3,      // 3: Message Set Signature
    NETWORK_REMOTE_ID = 4,          // 4: Authentication provided by Network Remote ID
    SPECIFIC_AUTH_METHOD = 5,       // 5: Specific Authentication Method (SAM)
    // 6-9: Reserved for Spec
    // 0xA - 0xF: Available for Private Use
};

/**
 * @brief Remote ID Authentication Message Encoder (ASTM F3411-22a Table 8 & Table 9)
 *
 * Message Type: 0x2
 * Protocol Version: 0x2
 * Message Size: 25 bytes
 *
 * Page 0 (Table 8):
 *   Byte 0     : Header (0x22: MsgType=2, ProtoVer=2)
 *   Byte 1     : AuthType [7..4] | PageNumber=0 [3..0]
 *   Byte 2     : Reserved [7..4] | LastPageIndex [3..0] (0..15)
 *   Byte 3     : Length (0..255 bytes of total concatenated auth data)
 *   Bytes 4..7 : Timestamp (32-bit uint LE, seconds since 2019-01-01 00:00:00 UTC)
 *   Bytes 8..24: Auth Data / Signature (17 bytes)
 *
 * Pages 1..15 (Table 9):
 *   Byte 0     : Header (0x22)
 *   Byte 1     : AuthType [7..4] | PageNumber [3..0] (1..15)
 *   Bytes 2..24: Auth Data / Signature (23 bytes)
 */
class RidAuthEncoder
{
public:
    static constexpr size_t MESSAGE_SIZE = 25;
    static constexpr size_t PAGE_0_DATA_SIZE = 17;
    static constexpr size_t PAGE_N_DATA_SIZE = 23;
    static constexpr size_t MAX_PAGES = 16;
    static constexpr size_t MAX_AUTH_DATA_SIZE = 255;
    static constexpr uint32_t ASTM_EPOCH_OFFSET = 1546300800UL; // 2019-01-01 00:00:00 UTC

    RidAuthEncoder() = default;

    /**
     * @brief Calculate the total number of pages needed for a given auth data length.
     * @return Number of pages (1 to 16), or 0 if length exceeds MAX_AUTH_DATA_SIZE.
     */
    size_t calculateTotalPages(size_t authDataLength) const;

    /**
     * @brief Convert standard Unix timestamp (seconds since 1970-01-01) to ASTM F3411 epoch timestamp.
     */
    static uint32_t toAstmTimestamp(uint64_t unixTimestampSeconds);

    /**
     * @brief Encode a specific page of an Authentication Message.
     *
     * @param authType Authentication type (Table 8)
     * @param pageIndex Current page index (0 to lastPageIndex)
     * @param timestamp ASTM F3411 epoch timestamp (seconds since 2019-01-01 00:00:00 UTC)
     * @param authData Full authentication data / signature buffer (can be nullptr if authDataLength == 0)
     * @param authDataLength Total length in bytes of authentication data across all pages (0..255)
     * @param out Output buffer of at least 25 bytes
     * @return true if successfully encoded, false on error
     */
    bool encodePage(
        RidAuthType authType,
        uint8_t pageIndex,
        uint32_t timestamp,
        const uint8_t* authData,
        size_t authDataLength,
        uint8_t out[MESSAGE_SIZE]
    ) const;

    /**
     * @brief Encode all pages of an Authentication Message into an array of packets.
     *
     * @param authType Authentication type
     * @param timestamp ASTM F3411 epoch timestamp
     * @param authData Full authentication data buffer
     * @param authDataLength Total length of authentication data
     * @param out Array of output buffers (each 25 bytes)
     * @param maxPages Capacity of out array
     * @param pagesWritten Pointer to store the number of pages written
     * @return true if successfully encoded, false on error
     */
    bool encodeAllPages(
        RidAuthType authType,
        uint32_t timestamp,
        const uint8_t* authData,
        size_t authDataLength,
        uint8_t out[][MESSAGE_SIZE],
        size_t maxPages,
        size_t* pagesWritten
    ) const;
};
