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