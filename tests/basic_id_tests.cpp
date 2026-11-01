#include "RidBasicIdEncoder.h"
#include "opendroneid.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <string>

using Packet = std::array<uint8_t, 25>;
static unsigned assertions = 0, failures = 0, comparisons = 0;

static void check(bool condition, const std::string& label)
{
    ++assertions;
    if (!condition) {
        ++failures;
        if (failures <= 20) std::fprintf(stderr, "FAIL: %s\n", label.c_str());
    }
}

// DEVELOPMENT-ONLY fixtures. TEST is not asserted to be an assigned/reserved
// manufacturer code. No identifier here is claimed to identify a real aircraft.
static RidBasicIdV1 fixture(const std::string& text = "TEST4DEV1",
                            RidBasicIdType type = RidBasicIdType::SERIAL_NUMBER)
{
    RidBasicIdV1 identity;
    identity.idType = type;
    check(text.size() <= RidBasicIdV1::UAS_ID_SIZE, "fixture fits owned input buffer");
    std::memcpy(identity.uasId, text.data(), std::min(text.size(), RidBasicIdV1::UAS_ID_SIZE));
    return identity;
}

static Packet compareReference(const RidBasicIdV1& identity, const std::string& label)
{
    RidBasicIdEncoder encoder;
    uint8_t guarded[27];
    std::memset(guarded, 0xa5, sizeof(guarded));
    check(encoder.encode(identity, guarded + 1, 25) == RidBasicIdEncodeResult::SUCCESS,
          label + " project accepts input");
    check(guarded[0] == 0xa5 && guarded[26] == 0xa5, label + " exactly 25 output bytes");
    Packet actual;
    std::copy(guarded + 1, guarded + 26, actual.begin());

    ODID_BasicID_data data;
    odid_initBasicIDData(&data);
    data.IDType = static_cast<ODID_idtype_t>(identity.idType);
    data.UAType = static_cast<ODID_uatype_t>(identity.uaType);
    std::memcpy(data.UASID, identity.uasId, sizeof(identity.uasId));
    ODID_BasicID_encoded reference;
    std::memset(&reference, 0xa5, sizeof(reference));
    check(encodeBasicIDMessage(&reference, &data) == ODID_SUCCESS, label + " upstream accepts input");
    check(std::memcmp(actual.data(), &reference, actual.size()) == 0, label + " all 25 bytes match upstream");
    check(actual[0] == 0x02, label + " type zero and version two");
    check(actual[1] >> 4 == static_cast<uint8_t>(identity.idType) &&
          (actual[1] & 0x0f) == static_cast<uint8_t>(identity.uaType), label + " type nibble order");
    const size_t length = std::strlen(identity.uasId);
    check(std::memcmp(actual.data() + 2, identity.uasId, length) == 0, label + " exact identifier bytes");
    check(std::all_of(actual.begin() + 2 + length, actual.begin() + 22,
                      [](uint8_t value) { return value == 0; }), label + " zero padding");
    check(actual[22] == 0 && actual[23] == 0 && actual[24] == 0, label + " reserved zero");
    ODID_BasicID_data decoded;
    odid_initBasicIDData(&decoded);
    check(decodeBasicIDMessage(&decoded, &reference) == ODID_SUCCESS &&
          std::memcmp(decoded.UASID, identity.uasId, sizeof(identity.uasId)) == 0,
          label + " upstream decoded identity");
    ++comparisons;
    return actual;
}

static void reject(const RidBasicIdV1& identity, RidBasicIdEncodeResult expected,
                   const std::string& label)
{
    uint8_t guarded[27];
    std::memset(guarded, 0xa5, sizeof(guarded));
    RidBasicIdEncoder encoder;
    check(encoder.validate(identity) == expected, label + " validation error");
    check(encoder.encode(identity, guarded + 1, 25) == expected, label + " encoding error");
    check(guarded[0] == 0xa5 && guarded[26] == 0xa5, label + " rejection buffer guards");
    check(std::all_of(guarded + 1, guarded + 26, [](uint8_t value) { return value == 0; }),
          label + " invalid identity cannot leave a previous valid packet");
}

static void acceptedPackets()
{
    const auto identity = fixture();
    const Packet actual = compareReference(identity, "development serial golden");
    const Packet golden = {{0x02, 0x12, 0x54, 0x45, 0x53, 0x54, 0x34, 0x44, 0x45, 0x56, 0x31}};
    check(actual == golden, "independent full golden literal, including padding and reserved bytes");
    compareReference(fixture("TEST11"), "shortest serial");
    const auto full = compareReference(fixture("TESTF123456789ABCDEF"), "20-character serial");
    check(full[21] == 'F', "full identifier uses the twentieth wire byte without truncation");
    compareReference(fixture("TEST40000"), "numeric manufacturer serial component");

    const std::string alphabet = "0123456789ABCDEFGHJKLMNPQRSTUVWXYZ";
    for (unsigned length = 1; length <= 15; ++length) {
        std::string text = "TEST";
        text += length <= 9 ? char('0' + length) : char('A' + length - 10);
        for (unsigned i = 0; i < length; ++i) text += alphabet[(i * 7 + length) % alphabet.size()];
        for (unsigned ua = 0; ua <= 15; ++ua) {
            auto value = fixture(text);
            value.uaType = static_cast<RidUaType>(ua);
            compareReference(value, "serial length code and UA type sweep");
        }
    }
    // CAA strings are also DEVELOPMENT-ONLY syntax fixtures; assignment is not asserted.
    for (const char* text : {"N.123456", "ZZ.IO123", "ZZ.123456789ABCDEFGH"})
        compareReference(fixture(text, RidBasicIdType::CAA_REGISTRATION), "CAA text fixture");
    for (unsigned ua = 0; ua <= 15; ++ua) {
        auto value = fixture("ZZ.DEV123", RidBasicIdType::CAA_REGISTRATION);
        value.uaType = static_cast<RidUaType>(ua);
        compareReference(value, "CAA UA type sweep");
    }
}

static void rejectedPackets()
{
    RidBasicIdV1 empty;
    reject(empty, RidBasicIdEncodeResult::EMPTY_IDENTIFIER, "unconfigured identity");
    auto value = fixture();
    std::memset(value.uasId, 'A', sizeof(value.uasId));
    reject(value, RidBasicIdEncodeResult::IDENTIFIER_TOO_LONG, "unterminated 21-character identifier");
    value = fixture(); value.uasId[10] = 'X';
    reject(value, RidBasicIdEncodeResult::INVALID_IDENTIFIER, "hidden data after terminator");
    reject(fixture("TEST"), RidBasicIdEncodeResult::SERIAL_LENGTH_MISMATCH, "missing serial component");
    reject(fixture("TEST0DEV1"), RidBasicIdEncodeResult::INVALID_SERIAL_LENGTH_CODE, "zero length code");
    reject(fixture("TESTGDEV1"), RidBasicIdEncodeResult::INVALID_SERIAL_LENGTH_CODE, "length code over F");
    reject(fixture("TEST3DEV1"), RidBasicIdEncodeResult::SERIAL_LENGTH_MISMATCH, "length code mismatch");
    for (const char* text : {"TEIT4DEV1", "TEOT4DEV1", "TEST4DEI1", "TEST4DEO1",
                                    "TEST4dev1", "TEST4DE-1", "TEST4DE 1"})
        reject(fixture(text), RidBasicIdEncodeResult::INVALID_IDENTIFIER, "invalid serial alphabet");
    for (const char* text : {"ZZDEV123", ".DEV123", "ZZ.", "ZZ.DEV.123"})
        reject(fixture(text, RidBasicIdType::CAA_REGISTRATION),
               RidBasicIdEncodeResult::INVALID_REGISTRATION_FORMAT, "invalid CAA structure");
    for (const char* text : {"ZZ.dev123", "ZZ.DEV-123", "ZZ.DEV 123", "ZZ.DEV\n"})
        reject(fixture(text, RidBasicIdType::CAA_REGISTRATION),
               RidBasicIdEncodeResult::INVALID_IDENTIFIER, "invalid CAA alphabet");
    value = fixture(); value.uasId[6] = static_cast<char>(0x80);
    reject(value, RidBasicIdEncodeResult::INVALID_IDENTIFIER, "non-ASCII identifier");

    for (unsigned type = 0; type <= 255; ++type) {
        if (type == 1 || type == 2) continue;
        value = fixture(); value.idType = static_cast<RidBasicIdType>(type);
        reject(value, RidBasicIdEncodeResult::UNSUPPORTED_ID_TYPE, "unsupported/reserved ID type");
    }
    for (unsigned ua = 16; ua <= 255; ++ua) {
        value = fixture(); value.uaType = static_cast<RidUaType>(ua);
        reject(value, RidBasicIdEncodeResult::INVALID_UA_TYPE, "UA type cannot overflow its nibble");
    }
}

static void characterSweep()
{
    const std::string base = "TEST4DEV1";
    RidBasicIdEncoder encoder;
    for (size_t position = 0; position < base.size(); ++position) {
        for (unsigned c = 0; c <= 255; ++c) {
            auto value = fixture(base);
            value.uasId[position] = static_cast<char>(c);
            const bool allowed = position == 4 ? c == '4' :
                (((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z')) && c != 'I' && c != 'O');
            check((encoder.validate(value) == RidBasicIdEncodeResult::SUCCESS) == allowed,
                  "exhaustive byte values at each serial character position");
            if (allowed) compareReference(value, "serial alphabet reference comparison");
        }
    }
}

static void bufferHandling()
{
    RidBasicIdEncoder encoder;
    const auto identity = fixture();
    check(encoder.encode(identity, nullptr, 25) == RidBasicIdEncodeResult::INVALID_OUTPUT_BUFFER,
          "null output rejected");
    for (size_t size = 0; size < 25; ++size) {
        uint8_t output[30]; std::memset(output, 0xa5, sizeof(output));
        check(encoder.encode(identity, output, size) == RidBasicIdEncodeResult::INVALID_OUTPUT_BUFFER,
              "undersized buffer rejected");
        check(std::all_of(output, output + 30, [](uint8_t v) { return v == 0xa5; }),
              "undersized buffer untouched");
    }
    uint8_t output[30]; std::memset(output, 0xa5, sizeof(output));
    check(encoder.encode(identity, output, sizeof(output)) == RidBasicIdEncodeResult::SUCCESS,
          "oversized buffer accepted");
    check(std::all_of(output + 25, output + 30, [](uint8_t v) { return v == 0xa5; }),
          "oversized buffer tail untouched");
    RidBasicIdV1 empty;
    check(encoder.encode(empty, output, sizeof(output)) == RidBasicIdEncodeResult::EMPTY_IDENTIFIER,
          "valid packet replaced by invalid input reports failure");
    check(std::all_of(output, output + 25, [](uint8_t v) { return v == 0; }),
          "failure removes old packet bytes");
    check(std::all_of(output + 25, output + 30, [](uint8_t v) { return v == 0xa5; }),
          "rejection does not clear unrelated output storage");
    for (unsigned result = 0; result <= unsigned(RidBasicIdEncodeResult::INVALID_REGISTRATION_FORMAT); ++result)
        check(std::strcmp(RidBasicIdEncoder::resultName(static_cast<RidBasicIdEncodeResult>(result)),
                          "UNKNOWN_RESULT") != 0, "every result has a diagnostic name");
}

int main()
{
    static_assert(RidBasicIdEncoder::MESSAGE_SIZE == 25, "project Basic ID length");
    static_assert(sizeof(ODID_BasicID_encoded) == 25, "reference Basic ID length");
    std::puts("DEVELOPMENT-ONLY fixtures: syntax tests do not establish manufacturer/CAA assignment or uniqueness.");
    acceptedPackets(); rejectedPackets(); characterSweep(); bufferHandling();
    std::printf("%s: %u assertions, %u complete 25-byte OpenDroneID comparisons, %u failures\n",
                failures ? "FAIL" : "PASS", assertions, comparisons, failures);
    return failures ? 1 : 0;
}
