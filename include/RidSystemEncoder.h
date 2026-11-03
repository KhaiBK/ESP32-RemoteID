#pragma once

#include <stdint.h>
#include <stddef.h>

#include "RidSystemMapped.h"


class RidSystemEncoder
{
public:

    static constexpr size_t MESSAGE_SIZE = 25;


    void encode(
        const RidSystemMapped& mapped,
        uint8_t out[MESSAGE_SIZE]
    ) const;
};

