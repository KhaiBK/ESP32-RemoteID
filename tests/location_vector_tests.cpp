#include "MavlinkAdapter.h"
#include "RidLocationMapper.h"
#include "RidLocationEncoder.h"
#include "opendroneid.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>

using Packet = std::array<uint8_t, 25>;
static unsigned assertions = 0, failures = 0, packets = 0, comparisons = 0;
static const uint64_t HOUR_US = 3600000000ULL;

static void check(bool condition, const std::string& name)
{
    ++assertions;
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", name.c_str());
    }
}

static Packet encode(const DroneStateV1& state)
{
    uint8_t guarded[27];
    std::memset(guarded, 0xa5, sizeof(guarded));
    RidLocationEncoder().encode(RidLocationMapper().map(state), guarded + 1);
    check(guarded[0] == 0xa5 && guarded[26] == 0xa5, "encoder writes exactly 25 bytes");
    Packet packet;
    std::copy(guarded + 1, guarded + 26, packet.begin());
    check((packet[1] & 0x08) == 0 && (packet[23] & 0xf0) == 0 && packet[24] == 0,
          "reserved bits/byte zero");
    ++packets;
    return packet;
}

static void expectPacket(const Packet& actual, const Packet& expected, const std::string& name)
{
    for (size_t i = 0; i < actual.size(); ++i) {
        if (actual[i] != expected[i]) {
            char detail[120];
            std::snprintf(detail, sizeof(detail), " byte %u actual=%02X expected=%02X",
                          unsigned(i), unsigned(actual[i]), unsigned(expected[i]));
            check(false, name + detail);
            return;
        }
    }
    check(true, name);
}

static Packet literal(const char* hex)
{
    Packet out = {};
    std::istringstream input(hex);
    unsigned value = 0;
    for (auto& byte : out) {
        check(bool(input >> std::hex >> value) && value <= 255, "25-byte literal syntax");
        byte = static_cast<uint8_t>(value);
    }
    check(!(input >> value), "literal contains exactly 25 bytes");
    return out;
}

static DroneStateV1 sample()
{
    DroneStateV1 s;
    s.operationalStatus = RidOperationalStatus::GROUND;
    s.operationalStatusValid = true;
    s.trackDirectionDeg = 90; s.trackDirectionValid = true;
    s.horizontalSpeedMps = 5; s.horizontalSpeedValid = true;
    s.verticalSpeedMps = -1.5f; s.verticalSpeedValid = true;
    s.latitudeDeg = 1; s.longitudeDeg = -2; s.positionValid = true;
    s.pressureAltitudeM = 100; s.pressureAltitudeValid = true;
    s.geodeticAltitudeM = 200; s.geodeticAltitudeValid = true;
    s.heightM = 10; s.heightValid = true;
    s.horizontalAccuracyM = 2; s.horizontalAccuracyValid = true;
    s.verticalAccuracyM = 5; s.verticalAccuracyValid = true;
    s.barometricAltitudeAccuracyM = 2; s.barometricAltitudeAccuracyValid = true;
    s.speedAccuracyMps = 0.5f; s.speedAccuracyValid = true;
    s.locationTimestampUtcUs = HOUR_US * 500000ULL + 123400000ULL;
    s.locationTimestampValid = true;
    s.timestampAccuracyS = 0.2f; s.timestampAccuracyValid = true;
    return s;
}

// Adapt physical units/validity to the independent upstream API. Do not use
// RidLocationMapped here: a mapper error must remain visible in comparisons.
static ODID_Location_data referenceData(const DroneStateV1& s)
{
    ODID_Location_data r;
    odid_initLocationData(&r);
    switch (s.operationalStatusValid ? s.operationalStatus : RidOperationalStatus::UNDECLARED) {
    case RidOperationalStatus::GROUND: r.Status = ODID_STATUS_GROUND; break;
    case RidOperationalStatus::AIRBORNE: r.Status = ODID_STATUS_AIRBORNE; break;
    case RidOperationalStatus::EMERGENCY: r.Status = ODID_STATUS_EMERGENCY; break;
    default: r.Status = ODID_STATUS_UNDECLARED;
    }
    r.HeightType = s.heightReference == RidHeightReference::AGL
        ? ODID_HEIGHT_REF_OVER_GROUND : ODID_HEIGHT_REF_OVER_TAKEOFF;
    if (s.trackDirectionValid && std::isfinite(s.trackDirectionDeg)) {
        double direction = std::fmod(double(s.trackDirectionDeg), 360.0);
        r.Direction = float(direction < 0 ? direction + 360.0 : direction);
    }
    if (s.horizontalSpeedValid && std::isfinite(s.horizontalSpeedMps) && s.horizontalSpeedMps >= 0)
        r.SpeedHorizontal = std::min(s.horizontalSpeedMps, MAX_SPEED_H);
    if (s.verticalSpeedValid && std::isfinite(s.verticalSpeedMps))
        r.SpeedVertical = std::max(float(MIN_SPEED_V), std::min(s.verticalSpeedMps, float(MAX_SPEED_V)));
    if (s.positionValid && std::isfinite(s.latitudeDeg) && std::isfinite(s.longitudeDeg) &&
        std::abs(s.latitudeDeg) <= 90 && std::abs(s.longitudeDeg) <= 180) {
        r.Latitude = s.latitudeDeg; r.Longitude = s.longitudeDeg;
    }
    auto altitude = [](float value, bool valid) {
        return valid && std::isfinite(value) && value > MIN_ALT ? std::min(value, MAX_ALT) : float(INV_ALT);
    };
    r.AltitudeBaro = altitude(s.pressureAltitudeM, s.pressureAltitudeValid);
    r.AltitudeGeo = altitude(s.geodeticAltitudeM, s.geodeticAltitudeValid);
    r.Height = altitude(s.heightM, s.heightValid);
    auto usable = [](float value, bool valid) { return valid && std::isfinite(value) && value > 0; };
    r.HorizAccuracy = usable(s.horizontalAccuracyM, s.horizontalAccuracyValid)
        ? createEnumHorizontalAccuracy(s.horizontalAccuracyM) : ODID_HOR_ACC_UNKNOWN;
    r.VertAccuracy = usable(s.verticalAccuracyM, s.verticalAccuracyValid)
        ? createEnumVerticalAccuracy(s.verticalAccuracyM) : ODID_VER_ACC_UNKNOWN;
    r.BaroAccuracy = usable(s.barometricAltitudeAccuracyM, s.barometricAltitudeAccuracyValid)
        ? createEnumVerticalAccuracy(s.barometricAltitudeAccuracyM) : ODID_VER_ACC_UNKNOWN;
    r.SpeedAccuracy = usable(s.speedAccuracyMps, s.speedAccuracyValid)
        ? createEnumSpeedAccuracy(s.speedAccuracyMps) : ODID_SPEED_ACC_UNKNOWN;
    r.TSAccuracy = usable(s.timestampAccuracyS, s.timestampAccuracyValid)
        ? createEnumTimestampAccuracy(s.timestampAccuracyS) : ODID_TIME_ACC_UNKNOWN;
    r.TimeStamp = s.locationTimestampValid
        ? float(double(s.locationTimestampUtcUs % HOUR_US) / 1000000.0) : float(INV_TIMESTAMP);
    return r;
}

static void compareReference(const DroneStateV1& s, const std::string& name)
{
    const ODID_Location_data input = referenceData(s);
    ODID_Location_encoded reference;
    std::memset(&reference, 0xa5, sizeof(reference));
    check(encodeLocationMessage(&reference, &input) == ODID_SUCCESS, name + " upstream accepts input");
    Packet expected;
    std::memcpy(expected.data(), &reference, expected.size());
    expectPacket(encode(s), expected, name);
    ++comparisons;
}

static unsigned u16(const Packet& p, size_t offset)
{
    return unsigned(p[offset]) | (unsigned(p[offset + 1]) << 8);
}

struct Fixture {
    DroneStateV1 state;
    MavlinkAdapter adapter;
    Fixture() : adapter(state) {}
    void heartbeat(uint32_t now, bool armed = false, uint8_t status = MAV_STATE_ACTIVE) {
        mavlink_heartbeat_t value = {};
        value.base_mode = armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0;
        value.system_status = status;
        mavlink_message_t msg;
        mavlink_msg_heartbeat_encode(1, 1, &msg, &value); adapter.handleMessage(msg, now);
    }
    void extended(uint32_t now, uint8_t landed = MAV_LANDED_STATE_IN_AIR) {
        mavlink_extended_sys_state_t value = {}; value.landed_state = landed;
        mavlink_message_t msg;
        mavlink_msg_extended_sys_state_encode(1, 1, &msg, &value); adapter.handleMessage(msg, now);
    }
    void position(uint32_t now, int32_t altitude = 100000) {
        mavlink_global_position_int_t value = {};
        value.time_boot_ms = now; value.lat = -353633513; value.lon = 1491652409;
        value.alt = altitude; value.vx = 300; value.vy = 400; value.vz = -150;
        mavlink_message_t msg;
        mavlink_msg_global_position_int_encode(1, 1, &msg, &value); adapter.handleMessage(msg, now);
    }
    void gps(uint32_t now) {
        mavlink_gps_raw_int_t value = {};
        value.fix_type = GPS_FIX_TYPE_3D_FIX; value.alt_ellipsoid = 120000;
        value.h_acc = 1500; value.v_acc = 2500; value.vel_acc = 500;
        mavlink_message_t msg;
        mavlink_msg_gps_raw_int_encode(1, 1, &msg, &value); adapter.handleMessage(msg, now);
    }
    void pressure(uint32_t now) {
        mavlink_scaled_pressure_t value = {}; value.press_abs = 1013.25f;
        mavlink_message_t msg;
        mavlink_msg_scaled_pressure_encode(1, 1, &msg, &value); adapter.handleMessage(msg, now);
    }
    void time(uint32_t now) {
        mavlink_system_time_t value = {};
        value.time_unix_usec = HOUR_US * 500000ULL + uint64_t(now) * 1000;
        value.time_boot_ms = now;
        mavlink_message_t msg;
        mavlink_msg_system_time_encode(1, 1, &msg, &value); adapter.handleMessage(msg, now);
    }
    void all(uint32_t now) {
        heartbeat(now); extended(now); gps(now); pressure(now); time(now); position(now);
        adapter.updateFreshness(now);
    }
};

static void goldenAndInvalid()
{
    const Packet ground = literal("12 10 5A 14 FD 80 96 98 00 00 D3 CE FE 98 08 60 09 E4 07 4B 53 D2 04 02 00");
    auto s = sample();
    expectPacket(encode(s), ground, "independent Python struct.pack ground golden");
    compareReference(s, "ground");
    s.operationalStatus = RidOperationalStatus::AIRBORNE;
    Packet airborne = ground; airborne[1] = 0x20;
    expectPacket(encode(s), airborne, "airborne golden"); compareReference(s, "airborne");
    s.heightReference = RidHeightReference::AGL;
    Packet agl = airborne; agl[1] = 0x24;
    expectPacket(encode(s), agl, "AGL flag golden"); compareReference(s, "AGL");
    s.operationalStatus = RidOperationalStatus::EMERGENCY; compareReference(s, "emergency");
    s.operationalStatusValid = false; compareReference(s, "invalid status");
    s = sample(); s.latitudeDeg = -35.3633513; s.longitudeDeg = 149.1652409;
    compareReference(s, "negative latitude and positive longitude");
    s = DroneStateV1();
    expectPacket(encode(s), literal("12 03 B5 FF 7E 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 FF FF 00 00"),
          "all unavailable golden");
    compareReference(s, "all unavailable");
    for (double bad : {91.0, -91.0, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
        s = sample(); s.latitudeDeg = bad;
        auto p = encode(s);
        check(std::all_of(p.begin() + 5, p.begin() + 13, [](uint8_t b) { return b == 0; }), "invalid coordinate clears pair");
        compareReference(s, "invalid latitude");
    }
    s = sample(); s.positionValid = false; compareReference(s, "invalid position flag");
    s = sample(); s.longitudeDeg = 181; compareReference(s, "invalid longitude");
    for (double lat : {-90.0, 0.0, 90.0}) for (double lon : {-180.0, 0.0, 180.0}) {
        s = sample(); s.latitudeDeg = lat; s.longitudeDeg = lon; compareReference(s, "coordinate boundary");
    }
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity()}) {
        s = sample(); s.trackDirectionDeg = s.horizontalSpeedMps = s.verticalSpeedMps = bad;
        s.pressureAltitudeM = s.geodeticAltitudeM = s.heightM = bad;
        s.horizontalAccuracyM = s.verticalAccuracyM = s.barometricAltitudeAccuracyM = bad;
        s.speedAccuracyMps = s.timestampAccuracyS = bad;
        compareReference(s, "nonfinite fields become unknown");
    }
    s = sample(); s.trackDirectionValid = s.horizontalSpeedValid = s.verticalSpeedValid = false;
    compareReference(s, "unknown motion flags");
    s = sample(); s.horizontalSpeedMps = -1; compareReference(s, "negative horizontal speed");
    s = sample();
    s.horizontalAccuracyM = s.verticalAccuracyM = s.barometricAltitudeAccuracyM = -1;
    s.speedAccuracyMps = s.timestampAccuracyS = -1;
    compareReference(s, "negative accuracy values become unknown");
    // DroneState has explicit validity bits; a known zero differs from the
    // upstream convenience helpers' zero-means-unavailable input convention.
    s = sample();
    s.horizontalAccuracyM = s.verticalAccuracyM = s.barometricAltitudeAccuracyM = 0;
    s.speedAccuracyMps = s.timestampAccuracyS = 0;
    auto p = encode(s);
    check(p[19] == 0x6c && p[20] == 0x64 && p[23] == 1, "explicitly valid zero accuracy policy");
    s.horizontalAccuracyValid = s.verticalAccuracyValid = s.barometricAltitudeAccuracyValid = false;
    s.speedAccuracyValid = s.timestampAccuracyValid = false;
    compareReference(s, "unavailable zero accuracy fields");
}

static void boundaries()
{
    for (float direction : {-720.f, -360.f, -0.5f, 0.f, 0.49f, 0.5f, 179.49f, 179.5f,
                            180.f, 180.5f, 359.49f, 359.5f, 359.99f, 360.f, 720.f}) {
        auto s = sample(); s.trackDirectionDeg = direction; compareReference(s, "direction boundary");
    }
    for (float speed : {0.f, 0.124f, 0.125f, 63.624f, 63.625f, 63.75f, 63.751f,
                        64.124f, 64.125f, 253.874f, 253.875f, 254.25f, 255.f, 1000.f}) {
        auto s = sample(); s.horizontalSpeedMps = speed; compareReference(s, "horizontal speed boundary");
    }
    for (float speed : {-100.f, -62.f, -0.25f, -0.249f, 0.f, 0.249f, 0.25f, 62.f, 63.f, 100.f}) {
        auto s = sample(); s.verticalSpeedMps = speed; compareReference(s, "signed vertical speed boundary");
    }
    struct AltCase { float meters; unsigned code; };
    for (const auto& c : {AltCase{-1001, 0}, {-1000, 0}, {-999.99f, 1}, {-999.75f, 1},
                          {-999.5f, 1}, {-999.25f, 2}, {0, 2000}, {10, 2020},
                          {31767.5f, 65535}, {40000, 65535}}) {
        auto s = sample(); s.pressureAltitudeM = s.geodeticAltitudeM = s.heightM = c.meters;
        auto p = encode(s);
        check(u16(p, 13) == c.code && u16(p, 15) == c.code && u16(p, 17) == c.code, "altitude boundary integer oracle");
        if (c.meters != -999.99f) compareReference(s, "altitude boundary");
    }
    // Preserve finer integer-UTC precision than the reference API's float seconds.
    for (uint64_t us : {0ULL, 49999ULL, 50000ULL, 99999ULL, 100000ULL,
                        3599949999ULL, 3599950000ULL, 3599999999ULL, 3600000000ULL}) {
        auto s = sample(); s.locationTimestampUtcUs = HOUR_US * 500000ULL + us;
        auto p = encode(s);
        check(u16(p, 21) == ((us % HOUR_US) + 50000) / 100000, "timestamp integer rounding/hour oracle");
    }
    auto s = sample(); s.locationTimestampValid = false;
    check(u16(encode(s), 21) == 65535, "invalid timestamp FFFF");
    compareReference(s, "invalid timestamp");
    s = sample();
    s.horizontalSpeedMps = s.pressureAltitudeM = s.geodeticAltitudeM = s.heightM = std::numeric_limits<float>::max();
    s.verticalSpeedMps = -std::numeric_limits<float>::max();
    compareReference(s, "finite overflow-scale values saturate safely");
    for (float threshold : {1.f, 3.f, 10.f, 30.f, 92.6f, 185.2f, 555.6f, 926.f,
                            1852.f, 3704.f, 7408.f, 18520.f}) {
        for (float value : {std::nextafter(threshold, 0.f), threshold,
                            std::nextafter(threshold, std::numeric_limits<float>::infinity())}) {
            s = sample(); s.horizontalAccuracyM = value; compareReference(s, "horizontal accuracy threshold");
        }
    }
    for (float threshold : {1.f, 3.f, 10.f, 25.f, 45.f, 150.f}) {
        for (float value : {std::nextafter(threshold, 0.f), threshold,
                            std::nextafter(threshold, std::numeric_limits<float>::infinity())}) {
            s = sample(); s.verticalAccuracyM = s.barometricAltitudeAccuracyM = value;
            compareReference(s, "vertical/barometric accuracy threshold");
        }
    }
    for (float threshold : {0.3f, 1.f, 3.f, 10.f}) {
        for (float value : {std::nextafter(threshold, 0.f), threshold,
                            std::nextafter(threshold, std::numeric_limits<float>::infinity())}) {
            s = sample(); s.speedAccuracyMps = value; compareReference(s, "speed accuracy threshold");
        }
    }
    for (unsigned code = 1; code <= 15; ++code) {
        float threshold = code / 10.f;
        for (float value : {std::nextafter(threshold, 0.f), threshold,
                            std::nextafter(threshold, std::numeric_limits<float>::infinity())}) {
            s = sample(); s.timestampAccuracyS = value; compareReference(s, "timestamp accuracy threshold");
        }
    }
}

static void randomPackets()
{
    uint32_t seed = 0x341122a;
    auto next = [&seed]() { seed = seed * 1664525U + 1013904223U; return seed; };
    for (unsigned i = 0; i < 1000; ++i) {
        auto s = sample();
        s.operationalStatus = static_cast<RidOperationalStatus>(next() % 4);
        s.heightReference = next() % 2 ? RidHeightReference::AGL : RidHeightReference::ABOVE_TAKEOFF;
        s.trackDirectionDeg = int(next() % 5761) * 0.25f - 720;
        s.horizontalSpeedMps = (next() % 1201) * 0.25f;
        s.verticalSpeedMps = int(next() % 281) * 0.5f - 70;
        s.latitudeDeg = (int(next() % 180000001) - 90000000) / 1000000.0;
        s.longitudeDeg = (int(next() % 360000001) - 180000000) / 1000000.0;
        s.pressureAltitudeM = s.geodeticAltitudeM = s.heightM = (next() % 82000) * 0.5f - 900;
        s.horizontalAccuracyM = 0.25f + (next() % 80000) * 0.25f;
        s.verticalAccuracyM = s.barometricAltitudeAccuracyM = 0.25f + (next() % 800) * 0.25f;
        s.speedAccuracyMps = 0.025f + (next() % 400) * 0.025f;
        s.timestampAccuracyS = (1 + next() % 15) / 10.f;
        s.locationTimestampUtcUs = HOUR_US * 500000ULL + uint64_t(next() % 36000) * 100000ULL;
        if (i % 7 == 0) s.positionValid = false;
        if (i % 11 == 0) s.locationTimestampValid = false;
        if (i % 13 == 0) s.trackDirectionValid = s.horizontalSpeedValid = s.verticalSpeedValid = false;
        compareReference(s, "random full packet " + std::to_string(i));
    }
}

static void freshnessAndAdapter()
{
    {
        Fixture f; f.all(0); compareReference(f.state, "adapter complete packet");
        check(std::abs(f.state.horizontalSpeedMps - 5) < 0.001f && f.state.verticalSpeedMps == 1.5f,
              "MAVLink units, horizontal magnitude and vertical sign");
        check(std::abs(f.state.pressureAltitudeM) < 0.01f, "1013.25 hPa yields zero standard pressure altitude");
        f.heartbeat(1001); f.extended(1001); f.gps(1001); f.pressure(1001); f.time(1001);
        f.adapter.updateFreshness(1001);
        const auto p = encode(f.state);
        check(!f.state.positionValid && u16(p, 21) == 65535 && p[2] == 181 && p[3] == 255 && p[4] == 126,
              "GPI timeout propagates unknown position/motion/timestamp");
        check(f.state.geodeticAltitudeValid && f.state.pressureAltitudeValid, "GPI timeout isolation");
        compareReference(f.state, "GPI timeout packet");
    }
    {
        Fixture f; f.all(0);
        f.heartbeat(1000); f.extended(1000); f.pressure(1000); f.time(1000); f.position(1000);
        f.adapter.updateFreshness(1000); const auto p = encode(f.state);
        check(u16(p, 15) == 0 && p[19] == 0 && (p[20] & 0x0f) == 0, "GPS timeout altitude and accuracy codes");
        check(f.state.positionValid && f.state.pressureAltitudeValid, "GPS timeout isolation");
        compareReference(f.state, "GPS timeout packet");
    }
    {
        Fixture f; f.all(0);
        f.heartbeat(1000); f.extended(1000); f.gps(1000); f.time(1000); f.position(1000);
        f.adapter.updateFreshness(1000);
        check(u16(encode(f.state), 13) == 0 && f.state.geodeticAltitudeValid, "pressure timeout isolation");
        compareReference(f.state, "pressure timeout packet");
    }
    {
        Fixture f; f.all(0);
        for (uint32_t t : {4999U, 5000U}) {
            f.heartbeat(t); f.extended(t); f.gps(t); f.pressure(t); f.position(t);
            f.adapter.updateFreshness(t);
            check(f.state.locationTimestampValid == (t == 4999), "SYSTEM_TIME exact holdover boundary");
            check(f.state.positionValid, "SYSTEM_TIME timeout preserves position");
            compareReference(f.state, "SYSTEM_TIME boundary packet");
        }
        f.time(5001); f.position(5002); f.adapter.updateFreshness(5002);
        check(f.state.locationTimestampValid, "SYSTEM_TIME recovery");
        compareReference(f.state, "SYSTEM_TIME recovery packet");
    }
    {
        Fixture f; f.all(0); f.extended(3000); f.adapter.updateFreshness(3000);
        check((encode(f.state)[1] >> 4) == 0 && !f.state.operationalStatusValid, "heartbeat stale maps undeclared");
        f.heartbeat(3001); f.adapter.updateFreshness(3001);
        check((encode(f.state)[1] >> 4) == 2, "heartbeat recovery maps airborne");
        f.heartbeat(6000); f.adapter.updateFreshness(6000);
        check((encode(f.state)[1] >> 4) == 0, "extended state stale maps undeclared");
        f.heartbeat(6001, false, MAV_STATE_EMERGENCY);
        check((encode(f.state)[1] >> 4) == 3, "fresh emergency overrides missing extended state");
        compareReference(f.state, "status freshness packet");
    }
    {
        Fixture f; f.heartbeat(0, true); f.position(1);
        check(!f.state.heightValid && u16(encode(f.state), 17) == 0, "startup armed keeps unknown height");
        f.heartbeat(2, false); f.heartbeat(3, true); f.position(4, 100000); f.position(5, 110000);
        check(f.state.heightValid && u16(encode(f.state), 17) == 2020, "ten meter height after observed arming");
        f.heartbeat(3003, true); f.position(3004, 120000);
        check(f.state.heightValid && f.state.heightM == 20, "safe heartbeat recovery retains original reference");
    }
    {
        Fixture f; f.heartbeat(0, false); f.heartbeat(3000, true); f.position(3001, 200000);
        check(!f.state.heightValid, "ambiguous arming recovery never captures mid-flight reference");
    }
    {
        Fixture f; const uint32_t base = 0xffffff00U;
        f.heartbeat(base); f.extended(base + 10U);
        f.adapter.updateFreshness(base + 2999U);
        check(f.state.operationalStatusValid, "status freshness across millis rollover");
        f.adapter.updateFreshness(base + 3000U);
        check(!f.state.operationalStatusValid, "status expiry across millis rollover");
    }
}

static void auditObservations()
{
    // Characterize open adapter limitations without claiming they pass a
    // standards compliance test or freezing the undesired behavior in asserts.
    Fixture trimmed;
    mavlink_gps_raw_int_t gps = {};
    gps.fix_type = GPS_FIX_TYPE_3D_FIX; gps.alt_ellipsoid = 100000;
    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_encode(1, 1, &msg, &gps);
    trimmed.adapter.handleMessage(msg, 0);
    if (!trimmed.state.geodeticAltitudeValid)
        std::printf("OPEN OBSERVATION: MAVLink-2 GPS len=%u with HAE=100m is rejected by extension-length guard.\n", unsigned(msg.len));

    Fixture repeated; repeated.all(0);
    repeated.heartbeat(60000); repeated.extended(60000); repeated.gps(60000);
    repeated.pressure(60000); repeated.time(60000);
    mavlink_global_position_int_t position = {};
    position.lat = 10000000; position.lon = 20000000; position.time_boot_ms = 0;
    mavlink_msg_global_position_int_encode(1, 1, &msg, &position);
    repeated.adapter.handleMessage(msg, 60000); repeated.adapter.updateFreshness(60000);
    if (repeated.state.positionValid && repeated.state.locationTimestampValid)
        std::printf("OPEN OBSERVATION: freshly received position with a 60-second-old sample time remains valid.\n");
}

int main(int argc, char** argv)
{
    static_assert(RidLocationEncoder::MESSAGE_SIZE == 25, "project packet size");
    static_assert(sizeof(ODID_Location_encoded) == 25, "upstream packed ABI size");
    const uint16_t one = 1;
    if (*reinterpret_cast<const uint8_t*>(&one) != 1) {
        std::fprintf(stderr, "Unsupported host: upstream packed reference requires little-endian ABI.\n");
        return 2;
    }
    if (argc == 2 && std::strcmp(argv[1], "--extreme-direction") == 0) {
        for (float value : {std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()}) {
            auto s = sample(); s.trackDirectionDeg = value; compareReference(s, "extreme finite direction");
        }
    } else {
        goldenAndInvalid(); boundaries(); randomPackets(); freshnessAndAdapter(); auditObservations();
    }
    std::printf("%s: %u assertions, %u complete packets, %u OpenDroneID comparisons, %u failures\n",
                failures ? "FAIL" : "PASS", assertions, packets, comparisons, failures);
    return failures ? 1 : 0;
}
