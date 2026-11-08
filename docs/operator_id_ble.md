# Operator ID BLE integration

Branch: `integration/pr1-operator-id`. The supplied `RidOperatorIdEncoder.h/.cpp`
are reused without modification. No merge, push, commit or upload is performed.

## Architecture and configuration

```text
RidOperatorIdConfig -> existing RidOperatorIdEncoder -> cached 25-byte packet
                                                           |
existing Location/Basic encoders -> RidBleBroadcaster -> Esp32RidBleTransport
```

`include/RidOperatorIdConfig.h` sets `RID_OPERATOR_ID_TYPE=0` and
`RID_OPERATOR_ID_VALUE="VN.OP2026840001"`. This is explicitly DEVELOPMENT-ONLY
test data and is not an officially assigned Operator ID. The startup log labels
it accordingly. Configuration remains separate from encoding and BLE control.
An empty configured value suppresses Operator ID. The encoder rejects types
other than 0, null/empty inputs, identifiers longer than 20 characters, controls
and non-ASCII input; the configuration header also bounds literal length.

The new broadcaster constructor accepts type/text independently of Basic ID,
calls the existing encoder, and owns the resulting packet. It retains no caller
string pointer. Invalid Operator ID is reported at begin() and excluded from
scheduling. The original constructor still supports Location/Basic without
Operator ID, keeping existing callers and regression tests valid.

No transport, MAVLink, DroneState, Location mapper/encoder, Basic encoder, age
monitor, UART loop or production freshness policy changes are needed. Main
passes configuration into the broadcaster and prints the new message/rejection
labels through the existing diagnostics.

## Bytes and indices

| Message | Internal scheduler/counter index | ASTM type | Header (version 2) |
| --- | --- | --- | --- |
| Basic ID | 0 | 0 | `02` |
| Location | 1 | 1 | `12` |
| Operator ID | 2 | 5 | `52` |

The frame builder explicitly permits wire types 0, 1 and 5. Internal index 2
does not authorize Authentication (wire type 2). The existing legacy frame stays:

```text
1E 16 FA FF 0D counter [25-byte encoded RID message]
```

The Operator ID message contains header `52` at message byte 0, type `00` at
byte 1, printable ID bytes at 2..21 with zero padding, and reserved zeros at
22..24. The entire encoded message is copied unchanged to AD offsets 6..30.
This matches the pinned [OpenDroneID Core C Operator ID encoder](https://github.com/opendroneid/opendroneid-core-c/blob/849c03adbe0cdd68997a462fafab397edcfe53f3/libopendroneid/opendroneid.c).

Example complete AD buffer for the development fixture and counter 0:

```text
1E 16 FA FF 0D 00 52 00 56 4E 2E 4F 50 32 30 32 36 38 34 30 30 30 31 00 00 00 00 00 00 00 00
```

The BLE transport continues nonconnectable legacy, uncoded LE 1M advertising
on channels 37/38/39 with one public advertiser address, 100 ms radio interval,
and the existing asynchronous stop/configure/start sequence. No AD fields,
GATT services, BLE 5 transport or NVS storage are added.

## Scheduling and counters

The original 250 ms Location/Basic alternation gave each message roughly a
500 ms cycle plus GAP overhead. A three-way round robin would extend the
Location cycle to about 750 ms plus overhead. Instead, every other window
remains reserved for Location:

```text
Location -> Basic -> Location -> Operator -> repeat
```

With both IDs valid, nominal Location interval remains about 500 ms; each static
ID is about 1000 ms. This targets the project's ASTM limits of at most 1000 ms
for Location and 3000 ms for static messages (BUR0010/BUR0020). Windows start
from successful GAP start completions, not arbitrary loop-call timestamps.

Static rotation advances only after a successful static start. Missing/invalid
Operator gives the original Location/Basic schedule; missing Basic gives
Location/Operator; both unavailable gives Location only. The existing 750 ms
Location priority rule still defers a pending static slot after a delayed loop.
The deferred Operator slot remains next after the priority Location succeeds.
No missed-slot catch-up burst is introduced.

Each of the three internal indices owns an independent uint8 counter. Counters
start at zero, advance after that message's successful start completion, and
wrap 255 -> 0. The existing permitted policy also advances for unchanged static
payloads. Raw update/start failures consume no counter. Gap diagnostics use
1000 ms for Location and 3000 ms separately for Basic and Operator.

Location's prepare/recheck logic, timestamp/age guards, stale unknown encodings,
early stop and recovery remain in place. Operator data cannot make Location
valid or alter an ARM/DISARM transition/takeoff reference.

## Verification

```powershell
python tests/run_operator_id_tests.py
python tests/run_ble_tests.py
python tests/run_basic_id_tests.py
python tests/run_location_tests.py
pio run
```

| Check | Result |
| --- | --- |
| Operator encoder | 31,512 assertions; 1,921 complete 25-byte Core C comparisons; zero failures |
| BLE integration/regressions | 93,032 assertions; zero failures |
| Basic ID regressions | 12,884 assertions; 536 Core C comparisons; zero failures |
| Location regressions | 4,979 assertions; 1,238 packets; 1,200 Core C comparisons; zero failures |
| PlatformIO m5stick-c | SUCCESS; flash 1,092,617 / 1,310,720 bytes (83.4%); static RAM 41,708 / 327,680 bytes (12.7%) |

The Operator suite verifies a full golden packet, exact length/header/type/ID,
padding/reserved bytes, 1..20-character inputs, every nonzero byte at every
20-byte ID position, reserved/private type rejection, null/empty/long inputs,
output guards and separately compiled pinned Core C results. Core C does not
enforce all project input restrictions; rejection rules are tested separately.

The BLE suite retains its original Location/Basic tests and adds full 31-byte
Operator framing, index/type separation, byte preservation, all three counters
through multiple wraps, invalid/absent ID combinations, delayed callbacks,
millis rollover, stale Location/recovery with Operator enabled, failed Operator
publication, and overdue Location priority over a pending Operator slot.

With a simulated 10 ms loop, the maximum measured gaps were:

| Completion profile | Location | Basic | Operator |
| --- | --- | --- | --- |
| First available poll (10 ms) | 560 ms | 1120 ms | 1120 ms |
| Two deferred polls (30 ms) | 680 ms | 1360 ms | 1360 ms |

These are host callback/scheduler observations, not RF results. Long loop stalls,
controller faults, interference or phone scan loss can produce larger gaps.
Existing GAP fault handling still reports and halts updates pending reset.

## Physical ESP32 / Android test

1. Review/build this branch. Firmware upload is a separate manual step; it was
   not performed here. Keep verbose flags off for routine timing observation.
2. Open Serial Monitor at 115200 and reset. Confirm `init=OK`, `step=ready`,
   the public MAC and `Operator ID DEVELOPMENT-ONLY type=0 value=VN.OP2026840001`.
   Periodic summaries should show `len=31`, growing `ok`, no failed updates or
   timing warnings. `active=NO` can briefly occur during a GAP switch.
3. For a short capture build enable `RID_BLE_VERBOSE_DIAGNOSTIC=1` and
   `RID_BLE_PACKET_DIAGNOSTIC=1` in main.cpp (or build definitions). Verify
   `LOCATION, BASIC_ID, LOCATION, OPERATOR_ID` and the corresponding counters.
   Each AD dump must contain exactly 31 bytes. These logs do not prove reception.
4. In [nRF Connect for Android](https://github.com/nordicsemi/Android-nRF-Connect),
   grant scan/Nearby Devices permissions, enable Bluetooth and scan. Include
   unnamed/nonconnectable devices. Match the Serial MAC and inspect Service
   Data UUID `0000fffa-0000-1000-8000-00805f9b34fb`; do not connect.
5. In raw packet history verify the common prefix and RID headers at AD offset
   6: `02`, `12`, `52`. Operator type at offset 7 must be `00`, and ID text at
   offsets 8..22 must decode to the fixture. A service-data-only display shows
   27 bytes beginning `0D counter`, because its UI removes the AD header/UUID.
6. In the [OpenDroneID Android receiver](https://github.com/opendroneid/receiver-android),
   grant requested permissions and enable Bluetooth scanning. Confirm one
   transmitter retains Basic `TEST4DEV1`, live Location and Operator
   `VN.OP2026840001`. Use raw packet history if the app does not expose Operator
   ID in its current UI. Record at least 60 seconds of per-type receive intervals.
7. Check Location freshness/recovery using the existing test procedure, including
   an old-tag case if available. Verify Operator and Basic continue while stale
   Location fields become unknown and fresh Location returns normally. Confirm
   MAVLink receive counts continue. Test an empty Operator config in a separate
   manual test build: expect one rejection message and the original two-message
   schedule. Restore the fixture after that test.

Actual Operator reception, device timing/UART load and interoperability remain
untested here. BUR0030's sensor applicability-age assurance remains OPEN, as
documented in the existing Location/BLE audit; the integration changes none of
those policies. The test identity establishes no real operator registration.
This is a three-message prototype rather than a full ASTM compliance claim.
