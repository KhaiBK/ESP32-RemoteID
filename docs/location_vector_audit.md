**Location/Vector audit — 2026-10-27**

The complete protocol-version-2 wire layout was checked against OpenDroneID Core C
at commit `849c03adbe0cdd68997a462fafab397edcfe53f3`, using the actual upstream C
encoder as a separate host-test implementation. The available F3411-22a preview
also supports the common data dictionary, status/accuracy definitions, magnitude
byte order, and data-age distinction. The complete normative Location/Vector
table and its notes were not available; direct clause-by-clause verification of
that table remains outside the evidence obtained here. This is a binary-format
and source audit, not a declaration of complete ASTM conformance.

Inspected: MavlinkAdapter.h/.cpp, DroneState.h, RidLocationMapper.h/.cpp,
RidLocationMapped.h, RidLocationEncoder.h/.cpp, and main.cpp. UART parsing feeds
the selected FC's messages into the Adapter; DroneState physical fields and
validity feed the Mapper; mapped codes feed the explicit 25-byte Encoder.

**Every byte, zero-based offsets**

All packing checks below pass after the narrowly scoped mapper fixes. Source
quality and applicability-age findings are separate from these packing results.

| Byte | Meaning checked | Result |
|---|---|---|
| 0 | Type 1 in high nibble, version 2 in low nibble: `12` | Pass |
| 1 | Status high nibble; bit 3 zero; height type bit 2; direction segment bit 1; speed multiplier bit 0 | Pass |
| 2 | Rounded direction component; unknown `B5` paired with segment 1 | Pass |
| 3 | Horizontal speed code; 0.25 m/s lower range, extended 0.75 m/s range with 63.75 m/s offset; unknown `FF` with multiplier 1 | Pass |
| 4 | Signed vertical speed in 0.5 m/s units; climb positive; unknown `7E` | Pass |
| 5 | Signed latitude E7, bits 7:0 | Pass |
| 6 | Latitude bits 15:8 | Pass |
| 7 | Latitude bits 23:16 | Pass |
| 8 | Latitude bits 31:24, including sign | Pass |
| 9 | Signed longitude E7, bits 7:0 | Pass |
| 10 | Longitude bits 15:8 | Pass |
| 11 | Longitude bits 23:16 | Pass |
| 12 | Longitude bits 31:24, including sign | Pass |
| 13 | Pressure-altitude code low byte | Pass |
| 14 | Pressure-altitude code high byte | Pass |
| 15 | WGS84 ellipsoid-altitude code low byte | Pass; source availability caveat below |
| 16 | Ellipsoid-altitude code high byte | Pass |
| 17 | Height code low byte; 10 m produces `E4` | Pass |
| 18 | Height code high byte; 10 m produces `07` | Pass |
| 19 | Vertical accuracy high nibble, horizontal accuracy low nibble | Pass; confidence caveat below |
| 20 | Barometric accuracy high nibble, speed accuracy low nibble | Pass; confidence caveat below |
| 21 | Timestamp low byte | Pass |
| 22 | Timestamp high byte | Pass |
| 23 | Timestamp accuracy low nibble; reserved high nibble zero | Pass after boundary fix |
| 24 | Reserved zero | Pass |

Altitude/height codes use 0.5 m increments and a -1000 m origin. Unknown is code
0; physical 0 m is code 2000. The upper code is 65535, representing 31767.5 m.
Valid height 10 m is code 2020, little-endian `E4 07`. Finite out-of-range high
values saturate; nonfinite or unavailable values become unknown. The deliberate
code-1 lower-edge policy is documented in the test README.

Timestamp codes are rounded tenths of seconds within the UTC hour. The final
half-tenth can round to 36000 (`A0 8C`); exactly the next hour becomes zero.
Unavailable timestamp is `FF FF`. Latitude and longitude are validated as a
pair; either invalid coordinate produces eight zero bytes. `(0,0)` is also the
protocol's unavailable-position representation, so that real coordinate cannot
be distinguished from unknown on this wire format. Internal validity booleans
are not serialized as extra bytes.

**Findings by severity**

| Severity | Finding | Disposition |
|---|---|---|
| Medium | Repeated +/-360 direction normalization could fail to advance for a huge finite float, blocking the mapper indefinitely. The timed baseline probe reproduced the hang. | Fixed with bounded `fmodf` normalization. Normal angles retain their encodings. |
| Low | Timestamp-accuracy thresholds formed with `code * 0.1f` rounded upward at 0.9/1.3 s; the immediately larger float incorrectly retained category 9/13. | Fixed by `code / 10.0f`; independent boundary comparisons now yield 10/14. |
| Low | Pressure altitude used 1013.2 hPa rather than 1013.25 hPa; at standard sea-level pressure the error was about -0.42 m and could alter the 0.5 m wire code. | Fixed only the constant and matching comments. No freshness policy changed. |
| High, unresolved assurance gap | GPS uncertainty is classified directly, although the source comment describes one-sigma estimates and the RID accuracy categories require 95% bounds. Raw-GPS accuracy also does not establish the accuracy of the fused GPI position/velocity solution. | Requires documented source semantics and a justified confidence/source conversion. No arbitrary multiplier was introduced. Numeric category thresholds themselves pass. |
| Medium, unresolved | Freshness measures message receipt, not sensor applicability age. A newly received GPI carrying a 60-second-old boot timestamp remains valid; reboot/time discontinuities are not detected. | Reproduced by host observation. Requires a separate applicability-age/clock policy; existing timeouts intentionally preserved. |
| Medium, unresolved availability issue | `GPS_RAW_INT` HAE requires payload length >=34. MAVLink 2 can legitimately truncate trailing zero bytes; a 33-byte packet with 100 m HAE is decoded but rejected by this guard. | Reproduced by host observation. Wire output is unknown rather than corrupt. Capability/presence handling needs a separate source-policy change. |
| Informational | Timestamp accuracy and barometric-altitude accuracy have no valid producers; they currently encode unknown. AGL has no adapter producer. Status code 4 (RID system failure) has no DroneState enum/source path. | Do not invent quality or failure state. Separate future feature work if required. |
| Informational | Above-takeoff height uses the first GPI after an observed arm transition, rather than an explicitly identified takeoff event. | Existing startup/gap protection preserved and tested. Confirm this baseline meets the intended installation's semantics. |

No encoder byte-layout, signedness, endian, reserved-bit, or unknown-sentinel
defect was found. Optional-field unknown encodings are not themselves packing
failures. Input validity still depends on the Adapter: GPI checks coordinate
bounds but does not assess estimator health or use GPS fix loss to invalidate
the fused navigation solution.

**Freshness propagation retained**

- GPI: 1000 ms; invalidates position, horizontal/vertical motion, direction,
  height, and location timestamp. Mapper emits the corresponding unknown codes.
- GPS_RAW_INT: 1000 ms; invalidates HAE and horizontal/vertical/speed accuracy.
- SCALED_PRESSURE: 1000 ms; invalidates pressure altitude.
- SYSTEM_TIME: 5000 ms holdover; invalidates synchronization and timestamp fields,
  independently of position/motion. Valid SYSTEM_TIME plus a later GPI recovers.
- HEARTBEAT and EXTENDED_SYS_STATE: independent 3000 ms limits. Fresh emergency
  heartbeat takes precedence; otherwise both sources are required for a valid
  ground/airborne classification. Stale/missing sources yield status code 0.
- First-heartbeat and receive-gap arming protection are unchanged. No temporary
  message-drop simulation remains. No diagnostic forces height valid or captures
  a new takeoff reference.

Every source check still runs in updateFreshness without an early return. The
Mapper trusts the validity flags; the diagnostic calls it after those checks.

**Verification and changed files**

Baseline tests reproduced three failing assertions (two timestamp-accuracy
boundaries and standard pressure) plus the separate direction-probe timeout.
After fixes: 4979 assertions, 1238 complete packets, 1200 byte-for-byte upstream
comparisons, zero failures. This includes 1000 deterministic varied packets,
independent whole-packet golden literals, integer timestamp boundary oracles,
accuracy threshold neighbors, signed/endian/reserved checks, nonfinite inputs,
all freshness groups, rollover, and arming recovery. The two open adapter
observations are printed separately from passing test assertions.

Existing files changed in this review:

- `src/RidLocationMapper.cpp`: the two reproduced mapper fixes.
- `src/MavlinkAdapter.cpp`: standard-pressure constant and its comment only.
- `include/DroneState.h`: pressure-reference comment only; no layout changes.
- `src/main.cpp`: isolated packet diagnostic flag, helper, schedule, and call.

New files: this report; `tests/README.md`, `tests/run_location_tests.py`,
`tests/location_vector_tests.cpp`; and `tests/reference/opendroneid/` containing
unmodified `opendroneid.c`, `opendroneid.h`, `LICENSE`, and provenance `UPSTREAM.txt`.
The vendored C library is solely a host oracle. Mapper/Encoder headers,
RidLocationMapped, the Encoder implementation, adapter header, and platformio.ini
were not changed by this review. Pre-existing working-tree edits are preserved.

PlatformIO build: SUCCESS, `platformio run -e m5stick-c`, exit code 0, 14.70 s.
RAM: 23692/327680 bytes (7.2%); flash: 287173/1310720 bytes (21.9%).
Device capture remains a manual step; no serial session or upload was performed.

**Collecting an M5Stick packet**

The temporary `RID_LOCATION_PACKET_DIAGNOSTIC` flag at the top of `main.cpp`
defaults to 1 and can be overridden with a compile definition. It works with
`TIMING_ONLY=true`. Approximately every 1000 ms after freshness updates, it maps
the current state, encodes the actual payload, and prints a short `RID_LOCATION`
summary followed by 25 uppercase two-digit hex bytes on one line. The hex line
has no label, no counter, and no transport wrapper. It is 74 visible characters
(50 hex digits plus 24 spaces), followed by the serial newline.

After you install this build on the M5Stick, open PlatformIO Serial Monitor on
the configured COM5 at 115200 baud, reset the device, and keep the FC stream
running. Save each `RID_LOCATION` summary together with the following hex line.
The summary's CODE comes from the mapped status and TS from the mapped timestamp.
An unavailable timestamp prints TS=65535 and bytes 21/22 are FF FF. Zero-filled
unknown coordinates are expected before position arrives or after it expires.
Set the flag to 0 and rebuild to disable the diagnostic. This review did not
upload firmware, change Mission Planner settings, add BLE/Basic ID, or commit.

**Sources**

- [OpenDroneID Core C, pinned revision](https://github.com/opendroneid/opendroneid-core-c/tree/849c03adbe0cdd68997a462fafab397edcfe53f3), specifically libopendroneid/opendroneid.c/.h. The project README relates protocol version 2 to F3411-22a.
- [ASTM F3411-22a catalog](https://store.astm.org/f3411-22a.html) and [available preview/catalog](https://standards.iteh.ai/catalog/standards/astm/1459a5be-dbaf-4f87-b1d0-8a3644ddf234/astm-f3411-22a). The complete Location/Vector table was not obtained.
- [MAVLink message definitions](https://mavlink.io/en/messages/common.html): GPS_RAW_INT is raw GPS; GLOBAL_POSITION_INT is the fused global estimate.
- [MAVLink 2 serialization](https://mavlink.io/en/guide/serialization.html#empty-byte-payload-truncation-mavlink-2): trailing zero truncation.
