#pragma once

#include "DroneState.h"
#include "RidSystemMapped.h"
#include "RidSystemSource.h"


class RidSystemMapper
{
public:

    RidSystemMapped map(
        const RidSystemSource& source
    ) const;
};

