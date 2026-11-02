#pragma once

#include "DroneState.h"
#include "RidLocationMapped.h"
#include "RidSystemSourceV1.h"

class RidLocationMapper
{
public:

    RidLocationMapper() = default;


    RidLocationMapped map(
        const DroneStateV1& state
    ) const;
};

///////////////////////////// SYSTEM /////////////////////////////////////////////////////////////////////////////////////

class RidSystemMapper
{
public:

    RidSystemMapped map(
        const RidSystemSourceV1& source
    ) const;
};

///////////////////////////// Self Id /////////////////////////////////////////////////////////////////////////////////////

class RidSelfIdMapper
{
public:

    RidSelfIdMapped map(
        const RidSelfIdSource& source
    ) const;
};