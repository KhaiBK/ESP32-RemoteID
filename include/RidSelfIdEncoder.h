#pragma once

#include <stddef.h>
#include <stdint.h>

#include "RidSelfId.h"


class RidSelfIdEncoder
{
public:

    static constexpr size_t MESSAGE_SIZE = 25;


    bool encode(
        const RidSelfId& selfId,
        uint8_t out[MESSAGE_SIZE]
    ) const;
};