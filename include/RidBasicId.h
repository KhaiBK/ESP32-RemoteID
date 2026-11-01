#pragma once

#include <stddef.h>
#include <stdint.h>

enum class RidBasicIdType : uint8_t
{
    NONE = 0,
    SERIAL_NUMBER = 1,
    CAA_REGISTRATION = 2,
    UTM_UUID = 3,
    SPECIFIC_SESSION = 4
};

enum class RidUaType : uint8_t
{
    UNDECLARED = 0,
    AEROPLANE = 1,
    HELICOPTER_OR_MULTIROTOR = 2,
    GYROPLANE = 3,
    HYBRID_LIFT = 4,
    ORNITHOPTER = 5,
    GLIDER = 6,
    KITE = 7,
    FREE_BALLOON = 8,
    CAPTIVE_BALLOON = 9,
    AIRSHIP = 10,
    FREE_FALL_PARACHUTE = 11,
    ROCKET = 12,
    TETHERED_POWERED_AIRCRAFT = 13,
    GROUND_OBSTACLE = 14,
    OTHER = 15
};

// Static identity data, independent of DroneState and MAVLink telemetry.
// V1 supports the Serial Number and CAA Registration text identifier formats.
struct RidBasicIdV1
{
    static constexpr size_t UAS_ID_SIZE = 20;

    RidBasicIdType idType = RidBasicIdType::SERIAL_NUMBER;
    RidUaType uaType = RidUaType::HELICOPTER_OR_MULTIROTOR;

    // Up to 20 ASCII characters plus a terminator. Unused storage stays zero.
    // Identifier assignment/authenticity is not established by syntax checks.
    char uasId[UAS_ID_SIZE + 1] = {};
};
