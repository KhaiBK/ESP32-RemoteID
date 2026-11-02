#pragma once

#include <stdint.h>
#include <stddef.h>

#include "RidLocationMapped.h"


class RidLocationEncoder
{
public:

    static constexpr size_t MESSAGE_SIZE = 25;

    RidLocationEncoder() = default;


    void encode(
        const RidLocationMapped& mapped,
        uint8_t out[MESSAGE_SIZE]
    ) const;
    
};

////////////////////////////// SYSTEM /////////////////////////////////////////////////////////////////////////////////////

class RidSystemEncoder
{
public:

    static constexpr size_t MESSAGE_SIZE = 25;


    void encode(
        const RidSystemMapped& mapped,
        uint8_t out[MESSAGE_SIZE]
    ) const;
};

////////////////////////////// Self Id /////////////////////////////////////////////////////////////////////////////////////

class RidSelfIdEncoder
{
public:

    static constexpr size_t MESSAGE_SIZE = 25;


    bool encode(
        const RidSelfIdMapped& mapped,
        uint8_t out[MESSAGE_SIZE]
    ) const;
};