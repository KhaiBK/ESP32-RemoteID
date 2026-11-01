**Basic ID V1 — 2026-10-27**

Static device configuration flows through `RidBasicIdV1` into the independent
`RidBasicIdEncoder`, which validates the identity and emits one 25-byte Basic ID
payload. It does not read DroneState, depend on MAVLink reception, or use the
Location mapper. The serial diagnostic is a consumer of this same encoder.

The existing Location/Vector implementation, freshness logic, MAVLink Adapter,
and upstream reference files were preserved. No BLE implementation was added.

**Configuration**

`include/RidBasicIdConfig.h` defines three compile-time settings:

```cpp
#define RID_BASIC_ID_ID_TYPE 1
#define RID_BASIC_ID_UA_TYPE 2
#define RID_BASIC_ID_UAS_ID ""
```

The defaults are Serial Number and Helicopter/Multirotor, suitable type values
for the initial ArduCopter SITL prototype. The UAS ID is deliberately unset, as
requested. Configure an actually assigned identifier in that header, or override
the macros in build definitions. The encoder contains no device identifier or
development fixture. Configuration never derives an identity from a MAVLink
system ID, ESP32 MAC, or an invented manufacturer code.

The configuration factory copies the string into an owned 21-byte input buffer:
20 identifier characters plus a terminator. An overlong compile-time string is
rejected by `static_assert`; the encoder also validates runtime inputs. ID/UA
configuration values must fit four bits.

**Input validation and scope**

- ID Type 1, Serial Number: 4 manufacturer characters, a length character
  `1..9` or `A..F`, and exactly the indicated 1..15 serial characters. Manufacturer
  and serial characters are uppercase ASCII letters/digits excluding I and O.
- ID Type 2, CAA Registration: nonempty nationality-mark component, one dot,
  nonempty registration component. Components contain uppercase ASCII letters
  and digits. I and O are permitted here. Total length is at most 20.
- Every defined UA value 0..15 is supported; the prototype uses 2. Values outside
  the nibble range are rejected rather than masked into a different type.
- An empty ID, overlong/unterminated input, invalid characters, non-ASCII bytes,
  or nonzero data after the terminator is rejected. Input is never silently
  truncated, case-folded, or otherwise rewritten into a different identifier.
- ID Types 0 and 3..255 return `UNSUPPORTED_ID_TYPE`. V1 supports the two text
  formats. Binary UTM UUID and Specific Session ID require distinct schemas and
  validation and are intentionally unsupported in this version.

Validation establishes structural syntax only. It cannot prove manufacturer-code
assignment, national registration, ownership, authenticity, or uniqueness. CAA
validation does not consult a registry or implement each country's issuing rules.
The public ANSI/CTA material used for the syntax rules is identified below.

**Exact wire layout**

| Byte | Contents |
|---|---|
| 0 | Message Type 0 in bits 7..4; Protocol Version 2 in bits 3..0. Value `02`. |
| 1 | ID Type in bits 7..4; UA Type in bits 3..0. Serial/Multirotor value `12`. |
| 2 | Identifier character 0 |
| 3 | Identifier character 1 |
| 4 | Identifier character 2 |
| 5 | Identifier character 3 |
| 6 | Identifier character 4 |
| 7 | Identifier character 5 |
| 8 | Identifier character 6 |
| 9 | Identifier character 7 |
| 10 | Identifier character 8 |
| 11 | Identifier character 9 |
| 12 | Identifier character 10 |
| 13 | Identifier character 11 |
| 14 | Identifier character 12 |
| 15 | Identifier character 13 |
| 16 | Identifier character 14 |
| 17 | Identifier character 15 |
| 18 | Identifier character 16 |
| 19 | Identifier character 17 |
| 20 | Identifier character 18 |
| 21 | Identifier character 19 |
| 22 | Reserved, zero |
| 23 | Reserved, zero |
| 24 | Reserved, zero |

Unused identifier positions are zero. A full 20-character ID occupies every ID
byte; a null terminator is not added beyond the wire field. There are no multibyte
integer fields or endian conversions in this message.

**Encoder API and failures**

```cpp
RidBasicIdEncoder encoder;
RidBasicIdV1 identity = RidBasicIdConfig::identity();
uint8_t packet[RidBasicIdEncoder::MESSAGE_SIZE];
const RidBasicIdEncodeResult result = encoder.encode(identity, packet, sizeof(packet));
// Use packet only when result == RidBasicIdEncodeResult::SUCCESS.
```

Successful encoding writes exactly 25 bytes. With a sufficiently large output
buffer, invalid identity data clears the first 25 bytes and returns a named error;
cleared error output is not a usable packet. Null or undersized buffers are
rejected without writes. Larger output buffers retain bytes after position 24.
Input and output storage must not alias. The class owns no identity or mutable
state and uses no heap allocation.

**Serial Monitor diagnostic**

`RID_BASIC_ID_PACKET_DIAGNOSTIC` in `src/main.cpp` defaults to 1. It is independent
of `TIMING_ONLY` and the existing Location diagnostic. Approximately once per
second, after the existing freshness call, it encodes Basic ID from its own
static identity configuration. No Location state is changed.

With the deliberately unset device ID, it prints:

```text
RID_BASIC_ID ms=... NOT_AVAILABLE EMPTY_IDENTIFIER; configure include/RidBasicIdConfig.h
```

No Basic ID hex packet is printed on failure. After a valid assigned ID is
configured and the user installs that build, Serial Monitor on COM5 at 115200
baud will show a `RID_BASIC_ID` summary followed by exactly 25 uppercase hex bytes
on one line. Leave `TIMING_ONLY=true`. Set `RID_BASIC_ID_PACKET_DIAGNOSTIC` to 0
and rebuild to disable only this diagnostic.

**Verification**

`python tests/run_basic_id_tests.py`: PASS, 12,884 assertions, 536 full-packet
comparisons against the actual OpenDroneID Core C encoder, zero failures. The
reference is pinned at `849c03adbe0cdd68997a462fafab397edcfe53f3`, compiled as C,
and remains outside the firmware.

All fixtures are explicitly DEVELOPMENT-ONLY, including `TEST4DEV1`; no assignment
or uniqueness is asserted. They are confined to host tests. The 25-byte golden
vector for that fixture is `02 12 54 45 53 54 34 44 45 56 31` followed by fourteen
zero bytes. OpenDroneID's packer does not itself validate CTA serial syntax;
project validation is tested separately from accepted-packet equality.

`python tests/run_location_tests.py`: PASS, 4,979 assertions, 1,238 complete
Location packets, 1,200 upstream comparisons, zero failures. Existing open source
quality observations from the Location review are unchanged.

PlatformIO build: SUCCESS, `platformio run -e m5stick-c`, exit code 0, 139.00 s.
RAM: 23716/327680 bytes (7.2%); flash: 288325/1310720 bytes (22.0%).
No upload or commit was performed.

**Files added/changed**

Added: `include/RidBasicId.h`, `include/RidBasicIdConfig.h`,
`include/RidBasicIdEncoder.h`, `src/RidBasicIdEncoder.cpp`,
`tests/basic_id_tests.cpp`, `tests/run_basic_id_tests.py`, and this document.
Changed: `src/main.cpp` for isolated Basic ID diagnostic additions and
`tests/README.md` to document the new test command. Existing Location/MAVLink
sources, old tests, reference files, and platformio.ini were not modified.

**References and remaining limits**

- [Pinned OpenDroneID Basic ID encoder](https://github.com/opendroneid/opendroneid-core-c/blob/849c03adbe0cdd68997a462fafab397edcfe53f3/libopendroneid/opendroneid.c) and its header establish the byte layout and type values used by the independent tests.
- [ANSI-hosted CTA-2063-A public-review material](https://share.ansi.org/Shared%20Documents/Standards%20Action/2019-PDFs/SAV5030.pdf), PDF pages 44–45, sections 4.1–4.3, provides manufacturer/length/serial syntax. The [CTA standard publisher page](https://shop.cta.tech/products/cta-2063) identifies the serial-number standard.
- [F3411-22a preview](https://standards.iteh.ai/catalog/standards/astm/1459a5be-dbaf-4f87-b1d0-8a3644ddf234/astm-f3411-22a) describes the identity types and CAA text format.

Remaining limits are the intentionally unconfigured real device identity,
external assignment/registry verification, and unsupported binary ID types 3/4.
Wire equality and syntax validation do not establish a real assigned identity.
