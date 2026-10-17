#pragma once

#include "DroneState.h"
#include "RidLocationMapped.h"

class RidLocationMapper
{
public:

    RidLocationMapper() = default;


    RidLocationMapped map(
        const DroneStateV1& state
    ) const;
};