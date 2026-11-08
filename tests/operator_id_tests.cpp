#include "RidOperatorIdEncoder.h"
#include "RidOperatorIdConfig.h"
#include "opendroneid.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <string>

namespace
{
using Packet = std::array<uint8_t, 25>;
unsigned assertions = 0, failures = 0, comparisons = 0;
#define CHECK(value) do { ++assertions; if (!(value)) { ++failures; \
    if (failures < 20) std::printf("FAIL line %d: %s\n", __LINE__, #value); } } while (0)

Packet compare(const std::string& text)
{
    RidOperatorIdEncoder encoder;
    uint8_t guarded[27];
    std::memset(guarded, 0xa5, sizeof(guarded));
    CHECK(encoder.encode(RidOperatorIdType::OPERATOR_ID, text.c_str(), guarded + 1));
    CHECK(guarded[0] == 0xa5 && guarded[26] == 0xa5);
    Packet packet;
    std::copy(guarded + 1, guarded + 26, packet.begin());
    CHECK(packet[0] == 0x52 && packet[1] == 0);
    CHECK(std::memcmp(packet.data() + 2, text.data(), text.size()) == 0);
    CHECK(std::all_of(packet.begin() + 2 + text.size(), packet.end(), [](uint8_t v) { return v == 0; }));
    CHECK(packet[22] == 0 && packet[23] == 0 && packet[24] == 0);

    ODID_OperatorID_data data;
    odid_initOperatorIDData(&data);
    data.OperatorIdType = ODID_OPERATOR_ID;
    std::memcpy(data.OperatorId, text.data(), text.size());
    ODID_OperatorID_encoded reference;
    std::memset(&reference, 0xa5, sizeof(reference));
    CHECK(encodeOperatorIDMessage(&reference, &data) == ODID_SUCCESS);
    CHECK(sizeof(reference) == packet.size());
    CHECK(std::memcmp(packet.data(), &reference, packet.size()) == 0);
    ODID_OperatorID_data decoded;
    CHECK(decodeOperatorIDMessage(&decoded, &reference) == ODID_SUCCESS);
    CHECK(decoded.OperatorIdType == ODID_OPERATOR_ID && std::strcmp(decoded.OperatorId, text.c_str()) == 0);
    ++comparisons;
    return packet;
}

void reject(RidOperatorIdType type, const char* text)
{
    uint8_t guarded[27];
    std::memset(guarded, 0xa5, sizeof(guarded));
    CHECK(!RidOperatorIdEncoder().encode(type, text, guarded + 1));
    CHECK(guarded[0] == 0xa5 && guarded[26] == 0xa5);
    CHECK(std::all_of(guarded + 1, guarded + 26, [](uint8_t v) { return v == 0; }));
}
} // namespace

int main()
{
    std::puts("DEVELOPMENT-ONLY Operator ID fixture; no official assignment is claimed.");
    static_assert(RidOperatorIdEncoder::MESSAGE_SIZE == 25, "25-byte wire payload");
    const Packet golden = {{
        0x52, 0x00, 0x56, 0x4e, 0x2e, 0x4f, 0x50, 0x32, 0x30, 0x32,
        0x36, 0x38, 0x34, 0x30, 0x30, 0x30, 0x31, 0, 0, 0, 0, 0, 0, 0, 0
    }};
    CHECK(compare("VN.OP2026840001") == golden);
    CHECK(RidOperatorIdConfig::TYPE == RidOperatorIdType::OPERATOR_ID);
    CHECK(std::strcmp(RidOperatorIdConfig::VALUE, "VN.OP2026840001") == 0);
    for (size_t length = 1; length <= 20; ++length)
        compare(std::string(length, static_cast<char>('A' + length % 26)));

    // Each input byte at each of 20 positions: independently compare every
    // accepted printable string with Core C; controls/high bytes must reject.
    for (size_t position = 0; position < 20; ++position)
        for (unsigned value = 1; value <= 255; ++value)
        {
            std::string text(20, 'A');
            text[position] = static_cast<char>(value);
            if (value >= 0x20 && value <= 0x7e) compare(text);
            else reject(RidOperatorIdType::OPERATOR_ID, text.c_str());
        }
    reject(RidOperatorIdType::OPERATOR_ID, nullptr);
    reject(RidOperatorIdType::OPERATOR_ID, "");
    reject(RidOperatorIdType::OPERATOR_ID, "123456789012345678901");
    char noTerminator[21];
    std::memset(noTerminator, 'A', sizeof(noTerminator));
    reject(RidOperatorIdType::OPERATOR_ID, noTerminator);
    for (unsigned type = 1; type <= 255; ++type)
        reject(static_cast<RidOperatorIdType>(type), "VN.OP2026840001");
    CHECK(!RidOperatorIdEncoder().encode(RidOperatorIdType::OPERATOR_ID, "VN.OP2026840001", nullptr));
    std::printf("Operator ID: %u assertions, %u complete 25-byte OpenDroneID comparisons, %u failures\n",
                assertions, comparisons, failures);
    return failures ? 1 : 0;
}
