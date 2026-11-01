#pragma once

#include "RidBasicId.h"

// Device configuration, separate from encoding. No fabricated serial is supplied.
// Set UAS ID to an actually assigned identifier before expecting device packets.
// Development-only identifiers are confined to host tests.
#ifndef RID_BASIC_ID_ID_TYPE
#define RID_BASIC_ID_ID_TYPE 1
#endif

#ifndef RID_BASIC_ID_UA_TYPE
#define RID_BASIC_ID_UA_TYPE 2
#endif

#ifndef RID_BASIC_ID_UAS_ID
#define RID_BASIC_ID_UAS_ID "TEST4DEV1"
#endif

namespace RidBasicIdConfig
{
static_assert(RID_BASIC_ID_ID_TYPE >= 0 && RID_BASIC_ID_ID_TYPE <= 15, "ID Type must fit four bits");
static_assert(RID_BASIC_ID_UA_TYPE >= 0 && RID_BASIC_ID_UA_TYPE <= 15, "UA Type must fit four bits");

constexpr char UAS_ID[] = RID_BASIC_ID_UAS_ID;
static_assert(sizeof(UAS_ID) <= RidBasicIdV1::UAS_ID_SIZE + 1, "Configured UAS ID exceeds 20 characters");

inline RidBasicIdV1 identity()
{
    RidBasicIdV1 value;
    value.idType = static_cast<RidBasicIdType>(RID_BASIC_ID_ID_TYPE);
    value.uaType = static_cast<RidUaType>(RID_BASIC_ID_UA_TYPE);
    for (size_t i = 0; i < sizeof(UAS_ID); ++i)
    {
        value.uasId[i] = UAS_ID[i];
    }
    return value;
}
} // namespace RidBasicIdConfig
