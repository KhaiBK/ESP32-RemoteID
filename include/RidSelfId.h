#pragma once

#include <stdint.h>


enum class RidSelfIdDescriptionType : uint8_t
{
    TEXT            = 0,
    EMERGENCY       = 1,
    EXTENDED_STATUS = 2
};


struct RidSelfId
{
    bool valid = false;

    RidSelfIdDescriptionType descriptionType =
        RidSelfIdDescriptionType::TEXT;

    // 23 byte truyền trên wire
    // + 1 byte NULL dùng nội bộ C/C++
    char description[24] = {};
};