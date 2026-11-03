#include <Arduino.h>
#include <ardupilotmega/mavlink.h>
#include "DroneState.h"
#include "MavlinkAdapter.h"
#include "RidLocationMapper.h"
#include "RidLocationEncoder.h"
#include "RidSystemMapper.h"
#include "RidSystemEncoder.h"
#include "RidBasicIdConfig.h"
#include "RidBasicIdEncoder.h"
#include "RidSelfId.h"
#include "RidSelfIdEncoder.h"

// BLE 4 legacy advertising; independent of TIMING_ONLY and packet diagnostics.
#ifndef RID_BLE_LEGACY_ENABLED
#define RID_BLE_LEGACY_ENABLED 1
#endif
#if RID_BLE_LEGACY_ENABLED
#include "Esp32RidBleTransport.h"
#endif

// Verbose capture is opt-in so the UART/parser loop stays responsive.
#ifndef RID_BLE_VERBOSE_DIAGNOSTIC
#define RID_BLE_VERBOSE_DIAGNOSTIC 0
#endif
#ifndef RID_BLE_PACKET_DIAGNOSTIC
#define RID_BLE_PACKET_DIAGNOSTIC 0
#endif
#ifndef RID_MAVLINK_VERBOSE_DIAGNOSTIC
#define RID_MAVLINK_VERBOSE_DIAGNOSTIC 0
#endif

// Temporary packet capture diagnostic; independent of TIMING_ONLY.
#ifndef RID_LOCATION_PACKET_DIAGNOSTIC
#define RID_LOCATION_PACKET_DIAGNOSTIC 0
#endif

// Independent Basic ID diagnostic; does not enable any broadcast transport.
#ifndef RID_BASIC_ID_PACKET_DIAGNOSTIC
#define RID_BASIC_ID_PACKET_DIAGNOSTIC 0
#endif


HardwareSerial FCSerial(2);

// UART tu USB-TTL
static const int FC_RX = 33;
static const int FC_TX = 32;

mavlink_message_t msg;
mavlink_status_t status;

DroneStateV1 droneState;
MavlinkAdapter mavlinkAdapter(droneState);
RidLocationMapper ridLocationMapper;
RidLocationEncoder ridLocationEncoder;

////////////////////////////////////////
RidSystemMapper ridSystemMapper;
RidSystemEncoder ridSystemEncoder;

RidSelfId selfId;
RidSelfIdEncoder ridSelfIdEncoder;
//////////////////////////////////////////

uint32_t lastDroneStatePrint = 0;

uint32_t byteCount = 0;
uint32_t messageCount = 0;

// Gioi han toc do in Serial Monitor
uint32_t lastGpsPrint = 0;
uint32_t lastPosPrint = 0;
uint32_t lastAttPrint = 0;
uint32_t lastVfrPrint = 0;
uint32_t lastSysPrint = 0;
uint32_t lastReport = 0;


namespace
{

#if RID_BLE_LEGACY_ENABLED
void reportRidBle(const RidBleDiagnostic& diagnostic);
Esp32RidBleTransport ridBleTransport;
RidLocationAgeMonitor ridLocationAgeMonitor;
RidBleBroadcaster ridBleBroadcaster(ridBleTransport, RidBasicIdConfig::identity(), reportRidBle);
uint32_t lastRidBleSummaryMs = 0;
bool ridBleInitReported = false;
int32_t ridBleInitStatus = 0;

void printRidBleSummary(const RidBleDiagnostic& d)
{
    char receiveAge[16] = "UNKNOWN", timestampAge[16] = "UNKNOWN";
    if (d.locationAge.receiveAgeAvailable)
        snprintf(receiveAge, sizeof(receiveAge), "%lu", static_cast<unsigned long>(d.locationAge.receiveAgeMs));
    if (d.locationAge.timestampAgeAvailable)
        snprintf(timestampAge, sizeof(timestampAge), "%lu", static_cast<unsigned long>(d.locationAge.timestampAgeEstimateMs));
    Serial.printf("[RID BLE] init=%s step=%s status=0x%08lX healthy=%s active=%s msg=%s counter=%u len=%u t=%lu updateGap_ms=%lu restartGap_ms=%lu(%s) maxGap_ms=%lu switch_ms=%lu ok=%lu failed=%lu\n",
                  ridBleInitReported ? (ridBleInitStatus == 0 ? "OK" : "FAILED") : "NOT_CALLED",
                  ridBleTransport.initializationStep(), static_cast<unsigned long>(ridBleInitStatus),
                  ridBleBroadcaster.healthy() ? "YES" : "NO",
                  d.advertisingActive ? "YES" : "NO", RidBleBroadcaster::messageName(d.messageType),
                  static_cast<unsigned int>(d.counter), static_cast<unsigned int>(d.payloadLength),
                  static_cast<unsigned long>(d.nowMs), static_cast<unsigned long>(d.gapMs),
                  static_cast<unsigned long>(d.restartGapMs), d.restartGapAvailable ? "GAP callbacks" : "unavailable",
                  static_cast<unsigned long>(d.maxRestartGapMs), static_cast<unsigned long>(d.switchLatencyMs),
                  static_cast<unsigned long>(d.successfulUpdates), static_cast<unsigned long>(d.failedUpdates));
    Serial.printf("[RID AGE] rx_ms=%s timestampEstimate_ms=%s sampleAge=UNVERIFIED prepared_ms=%lu advertised_ms=%lu callbackDelay_ms=%lu guard=%s clock=%s BUR0030=OPEN\n",
                  receiveAge, timestampAge, static_cast<unsigned long>(d.preparedAgeMs),
                  static_cast<unsigned long>(d.advertisingAgeMs), static_cast<unsigned long>(d.callbackDelayMs),
                  d.locationAge.gpiSuppressed ? "BLOCKED" : "PASS/UNVERIFIED",
                  d.locationAge.clockAmbiguous ? "AMBIGUOUS" : "no detected ambiguity");
}

void reportRidBle(const RidBleDiagnostic& d)
{
    switch (d.kind)
    {
        case RidBleDiagnosticKind::INITIALIZATION:
        {
            ridBleInitReported = true;
            ridBleInitStatus = d.status;
            Serial.printf("[RID BLE] init=%s step=%s status=0x%08lX (%s)\n",
                          d.status == 0 ? "OK" : "FAILED", ridBleTransport.initializationStep(),
                          static_cast<unsigned long>(d.status), esp_err_to_name(d.status));
            if (d.status == 0)
            {
                const uint8_t* address = ridBleTransport.publicAddress();
                Serial.printf("[RID BLE] public address=%02X:%02X:%02X:%02X:%02X:%02X legacy ADV_NONCONN_IND LE1M channels=37,38,39 interval=100ms\n",
                              address[0], address[1], address[2], address[3], address[4], address[5]);
            }
            break;
        }
        case RidBleDiagnosticKind::ADVERTISING:
#if RID_BLE_VERBOSE_DIAGNOSTIC
            printRidBleSummary(d);
#endif
#if RID_BLE_PACKET_DIAGNOSTIC
        {
            const uint8_t* bytes = ridBleBroadcaster.currentAdvertisingData();
            constexpr char HEX[] = "0123456789ABCDEF";
            char line[3 * RidBleLegacyFrame::ADVERTISING_SIZE];
            for (size_t i = 0; i < RidBleLegacyFrame::ADVERTISING_SIZE; ++i)
            {
                line[3 * i] = HEX[bytes[i] >> 4];
                line[3 * i + 1] = HEX[bytes[i] & 15];
                line[3 * i + 2] = i + 1 == RidBleLegacyFrame::ADVERTISING_SIZE ? '\0' : ' ';
            }
            Serial.printf("[RID BLE AD31] msg=%s counter=%u\n", RidBleBroadcaster::messageName(d.messageType),
                          static_cast<unsigned int>(d.counter));
            Serial.println(line);
        }
#endif
            break;
        case RidBleDiagnosticKind::BASIC_ID_REJECTED:
            Serial.printf("[RID BLE] Basic ID suppressed: %s; Location only\n",
                          RidBasicIdEncoder::resultName(static_cast<RidBasicIdEncodeResult>(d.status)));
            break;
        case RidBleDiagnosticKind::ERROR:
            Serial.printf("[RID BLE] ERROR operation=%s status=%ld (0x%08lX); updates halted, stop requested, reset required\n",
                          RidBleBroadcaster::operationName(d.operation), static_cast<long>(d.status),
                          static_cast<unsigned long>(d.status));
            break;
        case RidBleDiagnosticKind::DEADLINE_MISSED:
            Serial.printf("[RID BLE] WARNING %s update gap=%lums\n",
                          RidBleBroadcaster::messageName(d.messageType), static_cast<unsigned long>(d.gapMs));
            break;
        case RidBleDiagnosticKind::LOCATION_AGE_CHANGED:
            Serial.printf("[RID BLE] Location timestamp guard=%s stale=%s clockAmbiguous=%s; BUR0030=OPEN\n",
                          d.locationAge.gpiSuppressed ? "BLOCKED" : "RELEASED",
                          d.locationAge.timestampStale ? "YES" : "NO", d.locationAge.clockAmbiguous ? "YES" : "NO");
            break;
    }
}
#endif

// Source selected for this measurement.
// These IDs match the FC shown in your current logs.
constexpr uint8_t TIMING_FC_SYSID = 1;
constexpr uint8_t TIMING_FC_COMPID = 1;

// Print one summary every 5 seconds.
// This is a diagnostic reporting period, NOT a freshness timeout.
constexpr uint32_t TIMING_REPORT_PERIOD_MS = 5000;

// Temporarily disable verbose telemetry prints during measurement.
constexpr bool TIMING_ONLY = true;
constexpr uint32_t GPS_RAW_DEBUG_PERIOD_MS = 1000;
uint32_t lastGpsRawFreshnessPrintMs = 0;

struct MavlinkRxStats
{
    const char* name;
    uint32_t messageId;

    bool seen = false;

    uint32_t count = 0;

    // Local M5 time when parsing of the message completed.
    uint32_t lastRxMs = 0;

    uint32_t lastIntervalMs = 0;
    uint32_t maxIntervalMs = 0;

    uint64_t intervalSumMs = 0;

    MavlinkRxStats(
        const char* messageName,
        uint32_t id
    )
        : name(messageName),
          messageId(id)
    {
    }
};


MavlinkRxStats mavlinkRxStats[] =
{
    {"HEARTBEAT",           MAVLINK_MSG_ID_HEARTBEAT},
    {"EXTENDED_SYS_STATE",  MAVLINK_MSG_ID_EXTENDED_SYS_STATE},
    {"GLOBAL_POSITION_INT", MAVLINK_MSG_ID_GLOBAL_POSITION_INT},
    {"GPS_RAW_INT",         MAVLINK_MSG_ID_GPS_RAW_INT},
    {"SCALED_PRESSURE",     MAVLINK_MSG_ID_SCALED_PRESSURE},
    {"SYSTEM_TIME",         MAVLINK_MSG_ID_SYSTEM_TIME}
};


uint32_t lastTimingReportMs = 0;
// Diagnostic printing period, NOT the receive timeout.
constexpr uint32_t GPI_DEBUG_PERIOD_MS = 1000;

uint32_t lastGpiFreshnessPrintMs = 0;

#if RID_LOCATION_PACKET_DIAGNOSTIC
constexpr uint32_t RID_LOCATION_PACKET_PERIOD_MS = 1000;
uint32_t lastRidLocationPacketPrintMs = 0;
#endif

#if RID_BASIC_ID_PACKET_DIAGNOSTIC
const RidBasicIdV1 basicIdIdentity = RidBasicIdConfig::identity();
RidBasicIdEncoder ridBasicIdEncoder;
constexpr uint32_t RID_BASIC_ID_PACKET_PERIOD_MS = 1000;
uint32_t lastRidBasicIdPacketPrintMs = 0;
#endif

} // namespace

// ==========================================================================================================
// 
// ==========================================================================================================

struct TakeoffSnapshot
{
    bool positionValid = false;

    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;

    bool altitudeValid = false;
    float altitudeHAEM = 0.0f;

    bool locked = false;
};

TakeoffSnapshot takeoffSnapshot;


// ==========================================================================================================
// update
// ==========================================================================================================

void updateTakeoffSnapshot()
{
    if (takeoffSnapshot.locked)
    {
        return;
    }


    if (!droneState.positionValid)
    {
        return;
    }


    bool airborne =
        droneState.operationalStatusValid &&
        droneState.operationalStatus ==
            RidOperationalStatus::AIRBORNE;


    if (!airborne)
    {
        // While still on ground, continuously keep
        // the newest valid candidate takeoff position.

        takeoffSnapshot.positionValid = true;

        takeoffSnapshot.latitudeDeg =
            droneState.latitudeDeg;

        takeoffSnapshot.longitudeDeg =
            droneState.longitudeDeg;


        takeoffSnapshot.altitudeValid =
            droneState.geodeticAltitudeValid;

        takeoffSnapshot.altitudeHAEM =
            droneState.geodeticAltitudeM;

        return;
    }


    // First AIRBORNE state:
    // freeze last ground position.

    if (!takeoffSnapshot.positionValid)
    {
        // Fallback if no previous ground sample existed.

        takeoffSnapshot.positionValid = true;

        takeoffSnapshot.latitudeDeg =
            droneState.latitudeDeg;

        takeoffSnapshot.longitudeDeg =
            droneState.longitudeDeg;


        takeoffSnapshot.altitudeValid =
            droneState.geodeticAltitudeValid;

        takeoffSnapshot.altitudeHAEM =
            droneState.geodeticAltitudeM;
    }


    takeoffSnapshot.locked = true;


    Serial.println(
        "[RID SYSTEM] Takeoff position locked"
    );
}

// ===========================================================================================================
// Build Remote ID System Source
// ===========================================================================================================

RidSystemSourceV1 buildRidSystemSource()
{
    RidSystemSourceV1 source;


    // ==================================================
    // Operator source
    // ==================================================

    source.operatorLocationType =
        RidOperatorLocationType::TAKEOFF;


    // ==================================================
    // Classification
    //
    // Prototype hiện tại không áp EU classification.
    // ==================================================

    source.classificationType =
        RidClassificationType::UNDECLARED;


    // ==================================================
    // Takeoff position
    // ==================================================

    source.operatorPositionValid =
        takeoffSnapshot.positionValid;


    source.operatorLatitudeDeg =
        takeoffSnapshot.latitudeDeg;


    source.operatorLongitudeDeg =
        takeoffSnapshot.longitudeDeg;


    // ==================================================
    // Single aircraft
    // ==================================================

    source.areaCount = 1;

    source.areaRadiusM = 0.0f;


    // No group / formation ceiling/floor.

    source.areaCeilingValid = false;
    source.areaFloorValid = false;


    // ==================================================
    // Classification
    // ==================================================

    source.categoryEU = 0;
    source.classEU = 0;


    // ==================================================
    // Operator / Takeoff altitude
    //
    // Must be WGS84 HAE.
    // ==================================================

    source.operatorAltitudeValid =
        takeoffSnapshot.altitudeValid;


    source.operatorAltitudeM =
        takeoffSnapshot.altitudeHAEM;


    // ==================================================
    // System Timestamp
    //
    // ASTM System timestamp is the applicability time
    // of the Location message.
    //
    // Use the same absolute UTC time already maintained
    // for Location.
    // ==================================================

    source.timestampValid =
        droneState.locationTimestampValid;


    source.timestampUtcUs =
        droneState.locationTimestampUtcUs;


    return source;
}

// ===========================================================================================================
// Print Remote ID System Mapped
// ===========================================================================================================

void printRidSystemMapped()
{
    RidSystemSourceV1 source =
        buildRidSystemSource();


    RidSystemMapped rid =
        ridSystemMapper.map(
            source
        );


    Serial.println();
    Serial.println(
        "======= ASTM SYSTEM MAPPED ======="
    );


    Serial.print(
        "Classification code   : "
    );
    Serial.println(
        rid.classificationTypeCode
    );


    Serial.print(
        "Operator type code     : "
    );
    Serial.println(
        rid.operatorLocationTypeCode
    );


    Serial.print(
        "Operator Pos valid     : "
    );
    Serial.println(
        rid.operatorPositionValid
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Operator Lat code      : "
    );
    Serial.println(
        rid.operatorLatitudeCode
    );


    Serial.print(
        "Operator Lon code      : "
    );
    Serial.println(
        rid.operatorLongitudeCode
    );


    Serial.print(
        "Area Count code        : "
    );
    Serial.println(
        rid.areaCountCode
    );


    Serial.print(
        "Area Radius code       : "
    );
    Serial.println(
        rid.areaRadiusCode
    );


    Serial.print(
        "Area Ceiling code      : "
    );
    Serial.println(
        rid.areaCeilingCode
    );


    Serial.print(
        "Area Floor code        : "
    );
    Serial.println(
        rid.areaFloorCode
    );


    Serial.print(
        "Category code          : "
    );
    Serial.println(
        rid.categoryCode
    );


    Serial.print(
        "Class code             : "
    );
    Serial.println(
        rid.classCode
    );


    Serial.print(
        "Operator Alt valid     : "
    );
    Serial.println(
        rid.operatorAltitudeValid
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Operator Alt code      : "
    );
    Serial.println(
        rid.operatorAltitudeCode
    );


    Serial.print(
        "Timestamp valid        : "
    );
    Serial.println(
        rid.timestampValid
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Timestamp code         : "
    );
    Serial.println(
        rid.timestampCode
    );


    Serial.println(
        "=================================="
    );
}

// ============================================================================================================
// Print Remote ID System Encoded
// ============================================================================================================

void printRidSystemEncoded()
{
    RidSystemSourceV1 source =
        buildRidSystemSource();


    RidSystemMapped mapped =
        ridSystemMapper.map(
            source
        );


    uint8_t packet[
        RidSystemEncoder::MESSAGE_SIZE
    ];


    ridSystemEncoder.encode(
        mapped,
        packet
    );


    Serial.println();
    Serial.println(
        "======= RID SYSTEM ENCODED ======="
    );


    for (
        size_t i = 0;
        i < RidSystemEncoder::MESSAGE_SIZE;
        ++i
    )
    {
        Serial.print("Byte ");

        if (i < 10)
        {
            Serial.print(" ");
        }


        Serial.print(i);

        Serial.print(" : 0x");


        if (packet[i] < 0x10)
        {
            Serial.print("0");
        }


        Serial.println(
            packet[i],
            HEX
        );
    }


    Serial.println(
        "=================================="
    );
}

// ============================================================================================================
// Print Remote ID Self ID
// ============================================================================================================

void printRidSelfId()
{
    Serial.println();
    Serial.println(
        "======= SELF ID SOURCE ======="
    );


    Serial.print(
        "Valid            : "
    );

    Serial.println(
        selfId.valid
            ? "YES"
            : "NO"
    );


    Serial.print(
        "Description Type : "
    );

    Serial.println(
        static_cast<uint8_t>(
            selfId.descriptionType
        )
    );


    Serial.print(
        "Description      : "
    );

    Serial.println(
        selfId.description
    );


    Serial.println(
        "=============================="
    );
}

// ============================================================================================================
// Print Remote ID Self ID Encoded
// ============================================================================================================

void printRidSelfIdEncoded()
{
    uint8_t packet[
        RidSelfIdEncoder::MESSAGE_SIZE
    ] = {};


    bool encoded =
        ridSelfIdEncoder.encode(
            selfId,
            packet
        );


    if (!encoded)
    {
        Serial.println(
            "[SELF ID] ENCODE FAILED"
        );

        return;
    }


    Serial.println();
    Serial.println(
        "======= SELF ID ENCODED ======="
    );


    for (
        size_t i = 0;
        i < RidSelfIdEncoder::MESSAGE_SIZE;
        ++i
    )
    {
        Serial.print("Byte ");

        if (i < 10)
        {
            Serial.print(" ");
        }


        Serial.print(i);

        Serial.print(" : 0x");


        if (packet[i] < 0x10)
        {
            Serial.print("0");
        }


        Serial.println(
            packet[i],
            HEX
        );
    }


    Serial.println(
        "==============================="
    );
}

// ======================================================
// HEARTBEAT
// ======================================================

void handleHeartbeat(const mavlink_message_t &msg)
{
    mavlink_heartbeat_t hb;

    mavlink_msg_heartbeat_decode(&msg, &hb);

    bool armed =
        (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);

    Serial.println();
    Serial.println("===== HEARTBEAT =====");

    Serial.print("System ID     : ");
    Serial.println(msg.sysid);

    Serial.print("Component ID  : ");
    Serial.println(msg.compid);

    Serial.print("Vehicle type  : ");
    Serial.println(hb.type);

    Serial.print("Autopilot     : ");
    Serial.println(hb.autopilot);

    Serial.print("Base mode     : ");
    Serial.println(hb.base_mode);

    Serial.print("Custom mode   : ");
    Serial.println(hb.custom_mode);

    Serial.print("System status : ");
    Serial.println(hb.system_status);

    Serial.print("ARM state     : ");

    if (armed)
        Serial.println("ARMED");
    else
        Serial.println("DISARMED");

    Serial.println("=====================");
}


// ======================================================
// GPS_RAW_INT
// ======================================================

void handleGpsRaw(const mavlink_message_t &msg)
{
    if (millis() - lastGpsPrint < 1000)
        return;

    lastGpsPrint = millis();

    mavlink_gps_raw_int_t gps;

    mavlink_msg_gps_raw_int_decode(&msg, &gps);

    Serial.println();
    Serial.println("===== GPS_RAW_INT =====");

    Serial.print("Fix type      : ");
    Serial.println(gps.fix_type);

    Serial.print("Latitude      : ");
    Serial.println(gps.lat / 1e7, 7);

    Serial.print("Longitude     : ");
    Serial.println(gps.lon / 1e7, 7);

    Serial.print("Altitude MSL  : ");
    Serial.print(gps.alt / 1000.0);
    Serial.println(" m");

    Serial.print("Altitude HAE  : ");
    Serial.print( gps.alt_ellipsoid / 1000.0,2);
    Serial.println(" m");

    Serial.print("Message len   : ");
    Serial.println(msg.len);

    Serial.print("H Accuracy raw: ");
    Serial.print(gps.h_acc);
    Serial.println(" mm");

    Serial.print("V Accuracy raw: ");
    Serial.print(gps.v_acc);
    Serial.println(" mm");

    Serial.print("Speed Acc raw : ");
    Serial.print(gps.vel_acc);
    Serial.println(" mm/s");

    Serial.print("Satellites    : ");
    Serial.println(gps.satellites_visible);

    if (gps.eph != UINT16_MAX)
    {
        Serial.print("HDOP          : ");
        Serial.println(gps.eph / 100.0);
    }

    if (gps.vel != UINT16_MAX)
    {
        Serial.print("Ground speed  : ");
        Serial.print(gps.vel / 100.0);
        Serial.println(" m/s");
    }

    if (gps.cog != UINT16_MAX)
    {
        Serial.print("Course        : ");
        Serial.print(gps.cog / 100.0);
        Serial.println(" deg");
    }

    Serial.println("=======================");
}


// ======================================================
// GLOBAL_POSITION_INT
// ======================================================

void handleGlobalPosition(const mavlink_message_t &msg)
{
    if (millis() - lastPosPrint < 1000)
        return;

    lastPosPrint = millis();

    mavlink_global_position_int_t pos;

    mavlink_msg_global_position_int_decode(
        &msg,
        &pos
    );

    Serial.println();
    Serial.println("===== GLOBAL_POSITION_INT =====");

    Serial.print("Latitude       : ");
    Serial.println(pos.lat / 1e7, 7);

    Serial.print("Longitude      : ");
    Serial.println(pos.lon / 1e7, 7);

    Serial.print("Altitude MSL   : ");
    Serial.print(pos.alt / 1000.0);
    Serial.println(" m");

    Serial.print("Relative alt   : ");
    Serial.print(pos.relative_alt / 1000.0);
    Serial.println(" m");

    Serial.print("Velocity North : ");
    Serial.print(pos.vx / 100.0);
    Serial.println(" m/s");

    Serial.print("Velocity East  : ");
    Serial.print(pos.vy / 100.0);
    Serial.println(" m/s");

    Serial.print("Velocity Down  : ");
    Serial.print(pos.vz / 100.0);
    Serial.println(" m/s");

    if (pos.hdg != UINT16_MAX)
    {
        Serial.print("Heading        : ");
        Serial.print(pos.hdg / 100.0);
        Serial.println(" deg");
    }

    Serial.println("===============================");
}


// ======================================================
// ATTITUDE
// ======================================================

void handleAttitude(const mavlink_message_t &msg)
{
    if (millis() - lastAttPrint < 1000)
        return;

    lastAttPrint = millis();

    mavlink_attitude_t att;

    mavlink_msg_attitude_decode(&msg, &att);

    Serial.println();
    Serial.println("===== ATTITUDE =====");

    Serial.print("Roll  : ");
    Serial.print(att.roll * RAD_TO_DEG);
    Serial.println(" deg");

    Serial.print("Pitch : ");
    Serial.print(att.pitch * RAD_TO_DEG);
    Serial.println(" deg");

    Serial.print("Yaw   : ");
    Serial.print(att.yaw * RAD_TO_DEG);
    Serial.println(" deg");

    Serial.println("====================");
}


// ======================================================
// VFR_HUD
// ======================================================

void handleVfrHud(const mavlink_message_t &msg)
{
    if (millis() - lastVfrPrint < 1000)
        return;

    lastVfrPrint = millis();

    mavlink_vfr_hud_t hud;

    mavlink_msg_vfr_hud_decode(&msg, &hud);

    Serial.println();
    Serial.println("===== VFR_HUD =====");

    Serial.print("Ground speed : ");
    Serial.print(hud.groundspeed);
    Serial.println(" m/s");

    Serial.print("Airspeed     : ");
    Serial.print(hud.airspeed);
    Serial.println(" m/s");

    Serial.print("Heading      : ");
    Serial.print(hud.heading);
    Serial.println(" deg");

    Serial.print("Throttle     : ");
    Serial.print(hud.throttle);
    Serial.println(" %");

    Serial.print("Altitude     : ");
    Serial.print(hud.alt);
    Serial.println(" m");

    Serial.print("Climb rate   : ");
    Serial.print(hud.climb);
    Serial.println(" m/s");

    Serial.println("===================");
}


// ======================================================
// SYS_STATUS
// ======================================================

void handleSysStatus(const mavlink_message_t &msg)
{
    if (millis() - lastSysPrint < 2000)
        return;

    lastSysPrint = millis();

    mavlink_sys_status_t sys;

    mavlink_msg_sys_status_decode(&msg, &sys);

    Serial.println();
    Serial.println("===== SYS_STATUS =====");

    Serial.print("CPU load       : ");
    Serial.print(sys.load / 10.0);
    Serial.println(" %");

    if (sys.voltage_battery != UINT16_MAX)
    {
        Serial.print("Battery voltage: ");
        Serial.print(sys.voltage_battery / 1000.0);
        Serial.println(" V");
    }

    if (sys.current_battery != -1)
    {
        Serial.print("Battery current: ");
        Serial.print(sys.current_battery / 100.0);
        Serial.println(" A");
    }

    if (sys.battery_remaining != -1)
    {
        Serial.print("Battery remain : ");
        Serial.print(sys.battery_remaining);
        Serial.println(" %");
    }

    Serial.println("======================");
}


// ======================================================
// SCALED
// ======================================================
void handleScaledPressureDebug(
    const mavlink_message_t& msg
)
{
    mavlink_scaled_pressure_t pressure;

    mavlink_msg_scaled_pressure_decode(
        &msg,
        &pressure
    );

    Serial.println();
    Serial.println("===== SCALED_PRESSURE =====");

    Serial.print("Absolute pressure : ");
    Serial.print(pressure.press_abs, 2);
    Serial.println(" hPa");

    Serial.print("Differential press: ");
    Serial.print(pressure.press_diff, 2);
    Serial.println(" hPa");

    Serial.print("Temperature       : ");
    Serial.print(pressure.temperature / 100.0f, 2);
    Serial.println(" C");

    Serial.println("===========================");
}


// ======================================================
// EXTENDED_SYS_STATE
// ======================================================
void handleExtendedSysStateDebug(
    const mavlink_message_t& msg
)
{
    mavlink_extended_sys_state_t ext;

    mavlink_msg_extended_sys_state_decode(
        &msg,
        &ext
    );

    Serial.println();
    Serial.println("===== EXTENDED_SYS_STATE =====");

    Serial.print("Landed state : ");
    Serial.println(ext.landed_state);

    Serial.print("VTOL state   : ");
    Serial.println(ext.vtol_state);

    Serial.println("==============================");
}

// ======================================================
// SYSTEM_TIME
// ======================================================

void handleSystemTimeDebug(
    const mavlink_message_t& msg
)
{
    mavlink_system_time_t sys;

    mavlink_msg_system_time_decode(
        &msg,
        &sys
    );

    Serial.println();
    Serial.println("===== SYSTEM_TIME =====");

    Serial.printf(
        "Unix time us : %llu\n",
        (unsigned long long)sys.time_unix_usec
    );

    Serial.print("Boot time ms : ");
    Serial.println(sys.time_boot_ms);

    Serial.println("=======================");
}


void printDroneState()
{
    Serial.println();
    Serial.println("========== DRONE STATE V1 ==========");


    // ==================================================
    // Position
    // ==================================================

    Serial.print("Position valid      : ");
    Serial.println(
        droneState.positionValid
            ? "YES"
            : "NO"
    );

    Serial.print("Latitude            : ");
    Serial.println(
        droneState.latitudeDeg,
        7
    );

    Serial.print("Longitude           : ");
    Serial.println(
        droneState.longitudeDeg,
        7
    );


    // ==================================================
    // Horizontal Motion
    // ==================================================

    Serial.print("H Speed valid       : ");
    Serial.println(
        droneState.horizontalSpeedValid
            ? "YES"
            : "NO"
    );

    Serial.print("H Speed             : ");
    Serial.print(
        droneState.horizontalSpeedMps,
        2
    );
    Serial.println(" m/s");


    Serial.print("Track valid         : ");
    Serial.println(
        droneState.trackDirectionValid
            ? "YES"
            : "NO"
    );

    Serial.print("Track Direction     : ");
    Serial.print(
        droneState.trackDirectionDeg,
        2
    );
    Serial.println(" deg");


    // ==================================================
    // Vertical Motion
    // ==================================================

    Serial.print("V Speed valid       : ");
    Serial.println(
        droneState.verticalSpeedValid
            ? "YES"
            : "NO"
    );

    Serial.print("V Speed             : ");
    Serial.print(
        droneState.verticalSpeedMps,
        2
    );
    Serial.println(" m/s");


    // ==================================================
    // Height
    // ==================================================

    Serial.print("Height valid        : ");
    Serial.println(
        droneState.heightValid
            ? "YES"
            : "NO"
    );

    Serial.print("Height              : ");
    Serial.print(
        droneState.heightM,
        2
    );
    Serial.println(" m");

    Serial.print("Height reference    : ");

    if (droneState.heightReference ==
        RidHeightReference::ABOVE_TAKEOFF)
    {
        Serial.println("ABOVE_TAKEOFF");
    }
    else
    {
        Serial.println("AGL");
    }


    // ==================================================
    // Altitudes
    // ==================================================

    Serial.print("Pressure Alt valid  : ");
    Serial.println(
        droneState.pressureAltitudeValid
            ? "YES"
            : "NO"
    );

    Serial.print("Pressure Alt        : ");
    Serial.print(
        droneState.pressureAltitudeM,
        2
    );
    Serial.println(" m");


    Serial.print("Geodetic valid      : ");
    Serial.println(
        droneState.geodeticAltitudeValid
            ? "YES"
            : "NO"
    );

    Serial.print("Geodetic Alt        : ");
    Serial.print(
        droneState.geodeticAltitudeM,
        2
    );
    Serial.println(" m HAE");


    // ==================================================
    // Accuracy
    // ==================================================

    Serial.print("H Accuracy valid    : ");
    Serial.println(
        droneState.horizontalAccuracyValid
            ? "YES"
            : "NO"
    );

    Serial.print("H Accuracy          : ");
    Serial.print(
        droneState.horizontalAccuracyM,
        3
    );
    Serial.println(" m");


    Serial.print("V Accuracy valid    : ");
    Serial.println(
        droneState.verticalAccuracyValid
            ? "YES"
            : "NO"
    );

    Serial.print("V Accuracy          : ");
    Serial.print(
        droneState.verticalAccuracyM,
        3
    );
    Serial.println(" m");


    Serial.print("Baro Acc valid      : ");
    Serial.println(
        droneState.barometricAltitudeAccuracyValid
            ? "YES"
            : "NO"
    );

    Serial.print("Baro Accuracy       : ");
    Serial.print(
        droneState.barometricAltitudeAccuracyM,
        3
    );
    Serial.println(" m");


    Serial.print("Speed Acc valid     : ");
    Serial.println(
        droneState.speedAccuracyValid
            ? "YES"
            : "NO"
    );

    Serial.print("Speed Accuracy      : ");
    Serial.print(
        droneState.speedAccuracyMps,
        3
    );
    Serial.println(" m/s");


    // ==================================================
    // Operational Status
    // ==================================================

    Serial.print("Operational valid   : ");
    Serial.println(
        droneState.operationalStatusValid
            ? "YES"
            : "NO"
    );

    Serial.print("Operational status  : ");

    switch (droneState.operationalStatus)
    {
        case RidOperationalStatus::GROUND:
            Serial.println("GROUND");
            break;

        case RidOperationalStatus::AIRBORNE:
            Serial.println("AIRBORNE");
            break;

        case RidOperationalStatus::EMERGENCY:
            Serial.println("EMERGENCY");
            break;

        case RidOperationalStatus::UNDECLARED:
        default:
            Serial.println("UNDECLARED");
            break;
    }


    // ==================================================
    // Location Timestamp
    // ==================================================

    Serial.print("Timestamp valid     : ");
    Serial.println(
        droneState.locationTimestampValid
            ? "YES"
            : "NO"
    );

    if (droneState.locationTimestampValid)
    {
        uint64_t milliseconds =
            droneState.locationTimestampUtcUs /
            1000ULL;

        uint32_t msSinceHour =
            milliseconds %
            3600000ULL;

        float secondsSinceHour =
            msSinceHour /
            1000.0f;

        Serial.print("Time since hour     : ");
        Serial.print(
            secondsSinceHour,
            1
        );
        Serial.println(" s");
    }
    else
    {
        Serial.println(
            "Time since hour     : N/A"
        );
    }


    // ==================================================
    // Timestamp Accuracy
    // ==================================================

    Serial.print("Timestamp Acc valid : ");
    Serial.println(
        droneState.timestampAccuracyValid
            ? "YES"
            : "NO"
    );

    Serial.print("Timestamp Accuracy  : ");
    Serial.print(
        droneState.timestampAccuracyS,
        1
    );
    Serial.println(" s");


    Serial.println("====================================");
}
//=====================================
// RID_MAPPED
//====================================

void printRidLocationMapped()
{
    RidLocationMapped rid =
        ridLocationMapper.map(droneState);

    Serial.println();
    Serial.println("======= ASTM MAPPED FIELDS =======");


    // ==================================================
    // Byte 1
    // ==================================================

    Serial.print("Operational code     : ");
    Serial.println(
        rid.operationalStatusCode
    );

    Serial.print("Height Type bit       : ");
    Serial.println(
        rid.heightTypeBit
    );

    Serial.print("EW Segment bit        : ");
    Serial.println(
        rid.ewDirectionSegmentBit
    );

    Serial.print("Speed Mult bit        : ");
    Serial.println(
        rid.speedMultiplierBit
    );


    // ==================================================
    // Byte 2
    // ==================================================

    Serial.print("Track Direction code  : ");
    Serial.println(
        rid.trackDirectionCode
    );


    // ==================================================
    // Byte 3
    // ==================================================

    Serial.print("Horizontal Speed code : ");
    Serial.println(
        rid.horizontalSpeedCode
    );


    // ==================================================
    // Byte 4
    // ==================================================

    Serial.print("Vertical Speed code   : ");
    Serial.println(
        static_cast<int>(
            rid.verticalSpeedCode
        )
    );


    // ==================================================
    // Position metadata
    // ==================================================

    Serial.print("Position valid        : ");
    Serial.println(
        rid.positionValid
            ? "YES"
            : "NO"
    );


    // ==================================================
    // Bytes 5-8
    // ==================================================

    Serial.print("Latitude code         : ");
    Serial.println(
        rid.latitudeCode
    );


    // ==================================================
    // Bytes 9-12
    // ==================================================

    Serial.print("Longitude code        : ");
    Serial.println(
        rid.longitudeCode
    );


    // ==================================================
    // Bytes 13-14
    // ==================================================

    Serial.print("Pressure Alt code     : ");
    Serial.println(
        rid.pressureAltitudeCode
    );


    // ==================================================
    // Bytes 15-16
    // ==================================================

    Serial.print("Geodetic Alt code     : ");
    Serial.println(
        rid.geodeticAltitudeCode
    );


    // ==================================================
    // Bytes 17-18
    // ==================================================

    Serial.print("Height code           : ");
    Serial.println(
        rid.heightCode
    );


    // ==================================================
    // Byte 19
    // ==================================================

    Serial.print("H Accuracy code       : ");
    Serial.println(
        rid.horizontalAccuracyCode
    );

    Serial.print("V Accuracy code       : ");
    Serial.println(
        rid.verticalAccuracyCode
    );


    // ==================================================
    // Byte 20
    // ==================================================

    Serial.print("Baro Accuracy code    : ");
    Serial.println(
        rid.barometricAltitudeAccuracyCode
    );

    Serial.print("Speed Accuracy code   : ");
    Serial.println(
        rid.speedAccuracyCode
    );


    // ==================================================
    // Bytes 21-22
    // ==================================================

    Serial.print("Timestamp valid       : ");
    Serial.println(
        rid.timestampValid
            ? "YES"
            : "NO"
    );

    Serial.print("Timestamp code        : ");
    Serial.println(
        rid.timestampCode
    );


    // ==================================================
    // Byte 23
    // ==================================================

    Serial.print("Timestamp Acc code    : ");
    Serial.println(
        rid.timestampAccuracyCode
    );


    Serial.println("==================================");
}

//==========================================
//RID_ENCODE
//==========================================
void printRidLocationEncoded()
{
    RidLocationMapped mapped =
        ridLocationMapper.map(droneState);

    uint8_t packet[
        RidLocationEncoder::MESSAGE_SIZE
    ];

    ridLocationEncoder.encode(
        mapped,
        packet
    );


    Serial.println();
    Serial.println("======= RID ENCODED =======");

    for (size_t i = 0;
         i < RidLocationEncoder::MESSAGE_SIZE;
         ++i)
    {
        Serial.print("Byte ");

        if (i < 10)
        {
            Serial.print(" ");
        }

        Serial.print(i);
        Serial.print(" : 0x");

        if (packet[i] < 0x10)
        {
            Serial.print("0");
        }

        Serial.println(
            packet[i],
            HEX
        );
    }

    Serial.println("===========================");
}

#if RID_LOCATION_PACKET_DIAGNOSTIC
// Print the actual mapper/encoder output after the loop's freshness updates.
void printRidLocationPacketDiagnostic(uint32_t nowMs)
{
    static_assert(RidLocationEncoder::MESSAGE_SIZE == 25, "Diagnostic expects 25 bytes");
    const RidLocationMapped mapped = ridLocationMapper.map(droneState);
    uint8_t packet[RidLocationEncoder::MESSAGE_SIZE];
    ridLocationEncoder.encode(mapped, packet);

    Serial.printf(
        "RID_LOCATION ms=%lu STATUS=%s CODE=%u POS=%s HEIGHT=%s TS=%u\n",
        static_cast<unsigned long>(nowMs),
        droneState.operationalStatusValid ? "YES" : "NO",
        static_cast<unsigned int>(mapped.operationalStatusCode),
        mapped.positionValid ? "YES" : "NO",
        mapped.heightCode != 0 ? "YES" : "NO",
        static_cast<unsigned int>(mapped.timestampCode)
    );

    constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
    char hexLine[3 * RidLocationEncoder::MESSAGE_SIZE];
    for (size_t i = 0; i < RidLocationEncoder::MESSAGE_SIZE; ++i)
    {
        hexLine[3 * i] = HEX_DIGITS[packet[i] >> 4];
        hexLine[3 * i + 1] = HEX_DIGITS[packet[i] & 0x0F];
        hexLine[3 * i + 2] =
            i + 1 == RidLocationEncoder::MESSAGE_SIZE ? '\0' : ' ';
    }
    Serial.println(hexLine);
}
#endif

#if RID_BASIC_ID_PACKET_DIAGNOSTIC
void printRidBasicIdPacketDiagnostic(uint32_t nowMs)
{
    static_assert(RidBasicIdEncoder::MESSAGE_SIZE == 25, "Basic ID diagnostic expects 25 bytes");
    uint8_t packet[RidBasicIdEncoder::MESSAGE_SIZE];
    const RidBasicIdEncodeResult result = ridBasicIdEncoder.encode(
        basicIdIdentity, packet, sizeof(packet)
    );
    if (result != RidBasicIdEncodeResult::SUCCESS)
    {
        Serial.printf(
            "RID_BASIC_ID ms=%lu NOT_AVAILABLE %s; configure include/RidBasicIdConfig.h\n",
            static_cast<unsigned long>(nowMs),
            RidBasicIdEncoder::resultName(result)
        );
        return;
    }

    Serial.printf(
        "RID_BASIC_ID ms=%lu ID_TYPE=%u UA_TYPE=%u ID=%s\n",
        static_cast<unsigned long>(nowMs),
        static_cast<unsigned int>(basicIdIdentity.idType),
        static_cast<unsigned int>(basicIdIdentity.uaType),
        basicIdIdentity.uasId
    );
    constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
    char hexLine[3 * RidBasicIdEncoder::MESSAGE_SIZE];
    for (size_t i = 0; i < RidBasicIdEncoder::MESSAGE_SIZE; ++i)
    {
        hexLine[3 * i] = HEX_DIGITS[packet[i] >> 4];
        hexLine[3 * i + 1] = HEX_DIGITS[packet[i] & 0x0F];
        hexLine[3 * i + 2] =
            i + 1 == RidBasicIdEncoder::MESSAGE_SIZE ? '\0' : ' ';
    }
    Serial.println(hexLine);
}
#endif

//========================================
//FRESHNESS
//=======================================

void recordMavlinkTiming(
    const mavlink_message_t& message,
    uint32_t nowMs
)
{
    // Only measure messages from the selected FC.
    if (message.sysid != TIMING_FC_SYSID ||
        message.compid != TIMING_FC_COMPID)
    {
        return;
    }

    for (auto& s : mavlinkRxStats)
    {
        if (message.msgid != s.messageId)
        {
            continue;
        }

        if (s.seen)
        {
            // Unsigned subtraction handles millis() rollover
            // for the short measurement intervals used here.
            const uint32_t dtMs =
                static_cast<uint32_t>(
                    nowMs - s.lastRxMs
                );

            s.lastIntervalMs = dtMs;
            s.intervalSumMs += dtMs;

            if (dtMs > s.maxIntervalMs)
            {
                s.maxIntervalMs = dtMs;
            }
        }

        s.seen = true;
        ++s.count;
        s.lastRxMs = nowMs;

        return;
    }
}

void printMavlinkTiming(uint32_t nowMs)
{
    Serial.println();
    Serial.println("======= MAVLINK RX TIMING =======");

    Serial.println(
        "Message                Count  Last_ms    Avg_ms   Max_ms RxAge_ms"
    );

    for (const auto& s : mavlinkRxStats)
    {
        if (!s.seen)
        {
            Serial.printf(
                "%-21s NOT RECEIVED\n",
                s.name
            );

            continue;
        }

        const uint32_t rxAgeMs =
            static_cast<uint32_t>(
                nowMs - s.lastRxMs
            );

        // One message is not enough to calculate an interval.
        if (s.count < 2)
        {
            Serial.printf(
                "%-21s %6lu %8s %9s %8s %8lu\n",
                s.name,
                static_cast<unsigned long>(s.count),
                "-",
                "-",
                "-",
                static_cast<unsigned long>(rxAgeMs)
            );

            continue;
        }

        const double avgMs =
            static_cast<double>(s.intervalSumMs) /
            static_cast<double>(s.count - 1U);

        Serial.printf(
            "%-21s %6lu %8lu %9.1f %8lu %8lu\n",
            s.name,
            static_cast<unsigned long>(s.count),
            static_cast<unsigned long>(s.lastIntervalMs),
            avgMs,
            static_cast<unsigned long>(s.maxIntervalMs),
            static_cast<unsigned long>(rxAgeMs)
        );
    }

    Serial.println("=================================");
}


void printGpiFreshness(uint32_t nowMs)
{
    // Find the existing GPI receive statistics.
    const MavlinkRxStats* gpiStats = nullptr;

    for (const auto& s : mavlinkRxStats)
    {
        if (s.messageId == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
        {
            gpiStats = &s;
            break;
        }
    }

    // Map once for this diagnostic report.
    // This function does not change DroneState.
    const RidLocationMapped mapped =
        ridLocationMapper.map(droneState);

    Serial.println();
    Serial.println("======= GPI FRESHNESS TEST =======");

    if (gpiStats != nullptr && gpiStats->seen)
    {
        const uint32_t rxAgeMs =
            static_cast<uint32_t>(
                nowMs - gpiStats->lastRxMs
            );

        Serial.printf(
            "GPI count=%lu | RxAge_ms=%lu\n",
            static_cast<unsigned long>(gpiStats->count),
            static_cast<unsigned long>(rxAgeMs)
        );
    }
    else
    {
        Serial.println(
            "GPI: NOT RECEIVED | RxAge_ms=N/A"
        );
    }

    // Validity of data currently derived from GPI.
    Serial.printf(
        "Valid: POS=%s HSPD=%s TRACK=%s VSPD=%s\n",
        droneState.positionValid ? "YES" : "NO",
        droneState.horizontalSpeedValid ? "YES" : "NO",
        droneState.trackDirectionValid ? "YES" : "NO",
        droneState.verticalSpeedValid ? "YES" : "NO"
    );

    Serial.printf(
        "Valid: HEIGHT=%s TIMESTAMP=%s\n",
        droneState.heightValid ? "YES" : "NO",
        droneState.locationTimestampValid ? "YES" : "NO"
    );

    // These sources are not invalidated by the GPI timeout.
    Serial.printf(
        "Other valid: HAE=%s PRESSURE=%s STATUS=%s\n",
        droneState.geodeticAltitudeValid ? "YES" : "NO",
        droneState.pressureAltitudeValid ? "YES" : "NO",
        droneState.operationalStatusValid ? "YES" : "NO"
    );

    // Inspect the mapper's output after freshness checking.
    Serial.printf(
        "Mapped: LAT=%ld LON=%ld VSPD=%d HEIGHT=%u TS=%u\n",
        static_cast<long>(mapped.latitudeCode),
        static_cast<long>(mapped.longitudeCode),
        static_cast<int>(mapped.verticalSpeedCode),
        static_cast<unsigned int>(mapped.heightCode),
        static_cast<unsigned int>(mapped.timestampCode)
    );

    Serial.printf(
        "Direction: EW=%u CODE=%u | HSpeed: MULT=%u CODE=%u\n",
        static_cast<unsigned int>(mapped.ewDirectionSegmentBit),
        static_cast<unsigned int>(mapped.trackDirectionCode),
        static_cast<unsigned int>(mapped.speedMultiplierBit),
        static_cast<unsigned int>(mapped.horizontalSpeedCode)
    );

    Serial.print("Operational Status Code: ");
    Serial.println(
    static_cast<uint8_t>(droneState.operationalStatus)
);

    Serial.println("=================================");
}

void printGpsRawFreshness(uint32_t nowMs)
{
    const MavlinkRxStats* gpsStats = nullptr;

    for (const auto& s : mavlinkRxStats)
    {
        if (s.messageId == MAVLINK_MSG_ID_GPS_RAW_INT)
        {
            gpsStats = &s;
            break;
        }
    }

    RidLocationMapped mapped =
        ridLocationMapper.map(droneState);

    Serial.println();
    Serial.println("======= GPS_RAW FRESHNESS TEST =======");

    if (gpsStats != nullptr && gpsStats->seen)
    {
        const uint32_t rxAgeMs =
            static_cast<uint32_t>(
                nowMs - gpsStats->lastRxMs
            );

        Serial.printf(
            "GPS_RAW count=%lu | RxAge_ms=%lu\n",
            static_cast<unsigned long>(gpsStats->count),
            static_cast<unsigned long>(rxAgeMs)
        );
    }
    else
    {
        Serial.println(
            "GPS_RAW: NOT RECEIVED | RxAge_ms=N/A"
        );
    }

    Serial.printf(
        "Valid: HAE=%s HACC=%s VACC=%s SACC=%s\n",
        droneState.geodeticAltitudeValid ? "YES" : "NO",
        droneState.horizontalAccuracyValid ? "YES" : "NO",
        droneState.verticalAccuracyValid ? "YES" : "NO",
        droneState.speedAccuracyValid ? "YES" : "NO"
    );

    Serial.printf(
        "Other valid: POS=%s PRESSURE=%s STATUS=%s\n",
        droneState.positionValid ? "YES" : "NO",
        droneState.pressureAltitudeValid ? "YES" : "NO",
        droneState.operationalStatusValid ? "YES" : "NO"
    );

    Serial.printf(
        "Mapped: HAE=%u HACC=%u VACC=%u SACC=%u\n",
        static_cast<unsigned int>(
            mapped.geodeticAltitudeCode
        ),
        static_cast<unsigned int>(
            mapped.horizontalAccuracyCode
        ),
        static_cast<unsigned int>(
            mapped.verticalAccuracyCode
        ),
        static_cast<unsigned int>(
            mapped.speedAccuracyCode
        )
    );

    Serial.println("======================================");
}

//========================================
//VOID_SETUP
//========================================

void setup()
{
    Serial.begin(115200);

    delay(2000);

#if RID_BLE_LEGACY_ENABLED
    // Initialize once before opening UART; runtime GAP operations never wait.
    ridBleBroadcaster.begin(millis());
#endif

    FCSerial.begin(
        115200,
        SERIAL_8N1,
        FC_RX,
        FC_TX
    );

    // ==================================================
    // SELF ID CONFIG
    // ==================================================

    selfId.valid = true;

    selfId.descriptionType =
        RidSelfIdDescriptionType::TEXT;

    strncpy(
        selfId.description,
        "Self ID test",
        sizeof(selfId.description) - 1
    );


    Serial.println();
    Serial.println("====================================");
    Serial.println(" M5Stick MAVLink MULTI MSG TEST");
    Serial.println("====================================");

    Serial.println("RX   : GPIO33");
    Serial.println("Baud : 115200");
    Serial.println();
}


// ======================================================
// LOOP
// ======================================================

void loop()
{
    while (FCSerial.available() > 0)
    {
        uint8_t c = FCSerial.read();

        byteCount++;

        if (mavlink_parse_char(
                MAVLINK_COMM_0,
                c,
                &msg,
                &status))
        {   
            const uint32_t rxNowMs = millis();

// This function already filters the selected FC.
recordMavlinkTiming(msg, rxNowMs);

// Use the same selected source for DroneState updates.
if (msg.sysid == TIMING_FC_SYSID &&
    msg.compid == TIMING_FC_COMPID)
{
    mavlinkAdapter.handleMessage(msg, rxNowMs);
#if RID_BLE_LEGACY_ENABLED
    // Observe timestamp fields without altering adapter/state production policy.
    if (msg.msgid == MAVLINK_MSG_ID_SYSTEM_TIME)
        ridLocationAgeMonitor.observeSystemTime(mavlink_msg_system_time_get_time_boot_ms(&msg), rxNowMs);
    else if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
        ridLocationAgeMonitor.observePosition(mavlink_msg_global_position_int_get_time_boot_ms(&msg), rxNowMs);
#endif
}

            messageCount++;
         if(!TIMING_ONLY){
            switch (msg.msgid)
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    handleHeartbeat(msg);
                    break;

                case MAVLINK_MSG_ID_SYS_STATUS:
                    handleSysStatus(msg);
                    break;

                case MAVLINK_MSG_ID_GPS_RAW_INT:
                    handleGpsRaw(msg);
                    break;

                case MAVLINK_MSG_ID_ATTITUDE:
                    handleAttitude(msg);
                    break;

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    handleGlobalPosition(msg);
                    break;

                case MAVLINK_MSG_ID_VFR_HUD:
                    handleVfrHud(msg);
                    break;
                case MAVLINK_MSG_ID_SCALED_PRESSURE:
                    handleScaledPressureDebug(msg);
                    break;
                case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
                    handleExtendedSysStateDebug(msg);
                    break;
                case MAVLINK_MSG_ID_SYSTEM_TIME:
                    handleSystemTimeDebug(msg);
                    break;
                default:
                    break;
            }
         }
    }
}


    // Check timeouts on every loop, even when UART receives nothing.
const uint32_t freshnessNowMs = millis();

// Always check freshness, even when no UART data arrives.
mavlinkAdapter.updateFreshness(freshnessNowMs);
#if RID_BLE_LEGACY_ENABLED
ridBleBroadcaster.update(freshnessNowMs, droneState, ridLocationAgeMonitor.snapshot(freshnessNowMs));
if (static_cast<uint32_t>(freshnessNowMs - lastRidBleSummaryMs) >= 5000)
{
    lastRidBleSummaryMs = freshnessNowMs;
    printRidBleSummary(ridBleBroadcaster.diagnostics(freshnessNowMs));
}
#endif
#if RID_LOCATION_PACKET_DIAGNOSTIC
if (static_cast<uint32_t>(
        freshnessNowMs - lastRidLocationPacketPrintMs
    ) >= RID_LOCATION_PACKET_PERIOD_MS)
{
    lastRidLocationPacketPrintMs = freshnessNowMs;
    printRidLocationPacketDiagnostic(freshnessNowMs);
}
#endif

#if RID_BASIC_ID_PACKET_DIAGNOSTIC
if (static_cast<uint32_t>(
        freshnessNowMs - lastRidBasicIdPacketPrintMs
    ) >= RID_BASIC_ID_PACKET_PERIOD_MS)
{
    lastRidBasicIdPacketPrintMs = freshnessNowMs;
    printRidBasicIdPacketDiagnostic(freshnessNowMs);
}
#endif

#if RID_MAVLINK_VERBOSE_DIAGNOSTIC
if (static_cast<uint32_t>(
        freshnessNowMs - lastGpsRawFreshnessPrintMs
    ) >= GPS_RAW_DEBUG_PERIOD_MS)
{
    lastGpsRawFreshnessPrintMs = freshnessNowMs;

    printGpsRawFreshness(freshnessNowMs);
}

// Print a short report once per second.
if (static_cast<uint32_t>(
        freshnessNowMs - lastGpiFreshnessPrintMs
    ) >= GPI_DEBUG_PERIOD_MS)
{
    lastGpiFreshnessPrintMs = freshnessNowMs;

    printGpiFreshness(freshnessNowMs);
}
    // Bao cao parser moi 5 giay
    if (millis() - lastReport >= 5000)
    {
        lastReport = millis();

        Serial.println();
        Serial.println("----- MAVLINK STAT -----");

        Serial.print("Bytes received : ");
        Serial.println(byteCount);

        Serial.print("Messages parsed: ");
        Serial.println(messageCount);

        Serial.println("------------------------");
    }
     if (!TIMING_ONLY &&
    millis() - lastDroneStatePrint >= 1000)
{
    lastDroneStatePrint = millis();

    printDroneState();
    printRidLocationMapped();
    printRidLocationEncoded();

    Serial.print("=============================SYSTEM MSG================================");
    updateTakeoffSnapshot();
    printRidSystemMapped();
    printRidSystemEncoded();

    Serial.print("=============================SELF ID MSG============================");
    printRidSelfId();
    printRidSelfIdEncoded();

}

const uint32_t reportNowMs = millis();

if (static_cast<uint32_t>(
        reportNowMs - lastTimingReportMs
    ) >= TIMING_REPORT_PERIOD_MS)
{
    lastTimingReportMs = reportNowMs;

    printMavlinkTiming(reportNowMs);
}
#endif
}
