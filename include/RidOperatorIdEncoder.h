#pragma once

#include <stddef.h>
#include <stdint.h>


enum class RidOperatorIdType : uint8_t
{
    OPERATOR_ID = 0
};


class RidOperatorIdEncoder
{
public:

    static constexpr size_t MESSAGE_SIZE = 25;
    static constexpr size_t OPERATOR_ID_SIZE = 20;

    bool encode(
        RidOperatorIdType type,
        const char* operatorId,
        uint8_t out[MESSAGE_SIZE]
    ) const;
};
