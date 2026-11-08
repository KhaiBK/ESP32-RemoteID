#pragma once

#include "RidOperatorIdEncoder.h"

// DEVELOPMENT-ONLY Operator ID fixture; no official assignment is claimed.
// This configuration is separate from encoding and BLE. An empty value disables
// Operator ID through the existing encoder's rejection path.
#ifndef RID_OPERATOR_ID_TYPE
#define RID_OPERATOR_ID_TYPE 0
#endif
#ifndef RID_OPERATOR_ID_VALUE
#define RID_OPERATOR_ID_VALUE "VN.OP2026840001"
#endif

namespace RidOperatorIdConfig
{
static_assert(RID_OPERATOR_ID_TYPE >= 0 && RID_OPERATOR_ID_TYPE <= 255, "Operator ID Type must fit one byte");
constexpr RidOperatorIdType TYPE = static_cast<RidOperatorIdType>(RID_OPERATOR_ID_TYPE);
constexpr char VALUE[] = RID_OPERATOR_ID_VALUE;
static_assert(sizeof(VALUE) <= RidOperatorIdEncoder::OPERATOR_ID_SIZE + 1, "Configured Operator ID exceeds 20 characters");
} // namespace RidOperatorIdConfig
