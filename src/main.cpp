#include <Arduino.h>
#include <ardupilotmega/mavlink.h>
#include "DroneState.h"
#include "MavlinkAdapter.h"


HardwareSerial FCSerial(2);

// UART tu USB-TTL
static const int FC_RX = 33;
static const int FC_TX = 32;

mavlink_message_t msg;
mavlink_status_t status;

DroneStateV1 droneState;
MavlinkAdapter mavlinkAdapter(droneState);

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

// ======================================================
// SETUP
// ======================================================
void printDroneState()
{
    Serial.println();
    Serial.println("========== DRONE STATE V1 ==========");

    Serial.print("Position valid : ");
    Serial.println(droneState.positionValid ? "YES" : "NO");

    Serial.print("Latitude       : ");
    Serial.println(droneState.latitudeDeg, 7);

    Serial.print("Longitude      : ");
    Serial.println(droneState.longitudeDeg, 7);

    Serial.print("H Speed        : ");
    Serial.print(droneState.horizontalSpeedMps, 2);
    Serial.println(" m/s");

    Serial.print("Track valid    : ");
    Serial.println(droneState.trackDirectionValid ? "YES" : "NO");

    Serial.print("Track Direction: ");
    Serial.print(droneState.trackDirectionDeg, 2);
    Serial.println(" deg");

    Serial.print("V Speed        : ");
    Serial.print(droneState.verticalSpeedMps, 2);
    Serial.println(" m/s");

    Serial.print("Height valid    : ");
    Serial.println(droneState.heightValid ? "YES" : "NO");

    Serial.print("Height          : ");
    Serial.print(droneState.heightM, 2);
    Serial.println(" m");

    Serial.print("Height reference: ");

    if (droneState.heightReference ==
     RidHeightReference::ABOVE_TAKEOFF)
    {
    Serial.println("ABOVE_TAKEOFF");
    }
    else
   {
       Serial.println("AGL");
   }

   Serial.print("Geodetic valid  : ");
   Serial.println(
    droneState.geodeticAltitudeValid
        ? "YES"
        : "NO"
);

Serial.print("Geodetic alt    : ");
Serial.print(
    droneState.geodeticAltitudeM,
    2
);
Serial.println(" m HAE");

Serial.print("H Accuracy valid: ");
Serial.println(
    droneState.horizontalAccuracyValid
        ? "YES"
        : "NO"
);

Serial.print("H Accuracy      : ");
Serial.print(
    droneState.horizontalAccuracyM,
    3
);
Serial.println(" m");


Serial.print("V Accuracy valid: ");
Serial.println(
    droneState.verticalAccuracyValid
        ? "YES"
        : "NO"
);

Serial.print("V Accuracy      : ");
Serial.print(
    droneState.verticalAccuracyM,
    3
);
Serial.println(" m");


Serial.print("Speed Acc valid : ");
Serial.println(
    droneState.speedAccuracyValid
        ? "YES"
        : "NO"
);

Serial.print("Speed Accuracy  : ");
Serial.print(
    droneState.speedAccuracyMps,
    3
);
Serial.println(" m/s");

Serial.print("Pressure Alt valid: ");
Serial.println(
    droneState.pressureAltitudeValid
        ? "YES"
        : "NO"
);

Serial.print("Pressure Alt      : ");
Serial.print(
    droneState.pressureAltitudeM,
    2
);
Serial.println(" m");


Serial.print("Operational valid: ");
Serial.println(
    droneState.operationalStatusValid
        ? "YES"
        : "NO"
);

Serial.print("Operational status : ");

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

    case RidOperationalStatus::RID_SYSTEM_FAILURE:
        Serial.println("RID_SYSTEM_FAILURE");
        break;

    case RidOperationalStatus::UNDECLARED:
    default:
        Serial.println("UNDECLARED");
        break;
}

 Serial.print("Timestamp valid : ");
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
        milliseconds % 3600000ULL;

    float secondsSinceHour =
        msSinceHour / 1000.0f;

    Serial.print("Time since hour: ");
    Serial.print(secondsSinceHour, 1);
    Serial.println(" s");

    Serial.print("Timestamp Acc valid: ");
Serial.println(
    droneState.timestampAccuracyValid
        ? "YES"
        : "NO"
);

Serial.print("Timestamp Accuracy : ");
Serial.print(
    droneState.timestampAccuracyS,
    1
);
Serial.println(" s");
}


    Serial.println("====================================");
}


void setup()
{
    Serial.begin(115200);

    delay(2000);

    FCSerial.begin(
        115200,
        SERIAL_8N1,
        FC_RX,
        FC_TX
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
            mavlinkAdapter.handleMessage(msg, millis());

            messageCount++;

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
     if (millis() - lastDroneStatePrint >= 1000)
{
    lastDroneStatePrint = millis();
    printDroneState();
}
}
