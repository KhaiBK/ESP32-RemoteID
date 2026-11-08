# BLE 4 legacy Remote ID prototype

The existing Location, Basic ID and Operator ID encoders remain the source of
the 25-byte messages. They are wrapped in OpenDroneID BLE legacy Service Data.
This three-message prototype is not a complete Remote ID message set or a
certification claim. See [Operator ID integration](operator_id_ble.md) for its
configuration, Location-priority schedule, tests and device checklist.

## Installed stack and raw payload support

The inspected PlatformIO environment is Espressif32 6.10.0, Arduino-ESP32
2.0.17 (`framework-arduinoespressif32` 3.20017.241212), ESP-IDF 4.4.7,
with Bluedroid enabled. No new external BLE library or platform configuration
is needed. The native ESP-IDF APIs expose raw legacy advertising data and
separate configure/start/stop completion callbacks.

Although one API documentation description says "less than 31 bytes", the
[ESP-IDF 4.4.7 implementation](https://github.com/espressif/esp-idf/blob/v4.4.7/components/bt/host/bluedroid/api/esp_gap_ble_api.c#L412-L430)
rejects lengths **greater than** `ESP_BLE_ADV_DATA_LEN_MAX` (31), so exactly 31
is supported. It transfers a deep copy of the supplied data. The backend also
keeps its submitted buffer alive through completion. This verifies software
support; update reliability and reception on the M5Stick still need an on-air
test. No firmware was uploaded as part of this change.

The [raw Bluedroid write path](https://github.com/espressif/esp-idf/blob/v4.4.7/components/bt/host/bluedroid/stack/btm/btm_ble_gap.c#L1816-L1839)
sends the raw HCI advertising-data command and contains no explicit requirement
that advertising be stopped. Active updates are not categorically rejected by
the host path. Atomic on-air replacement has not been verified on this device;
the prototype retains stop/acknowledge/configure/acknowledge/start sequencing.
GAP callbacks now timestamp completions so the resulting gaps can be measured.

## Architecture

```text
UART -> MAVLink parser -> MavlinkAdapter -> updateFreshness -> DroneStateV1
                                                               |
                                        RidLocationMapper -> RidLocationEncoder
                                                               |
RidBasicIdConfig -> RidBasicIdEncoder --------------------------+
                                                               |
RidOperatorIdConfig -> RidOperatorIdEncoder --------------------+
                                                               |
                                      RidBleBroadcaster (31-byte frame/scheduler)
                                                               |
                                      Esp32RidBleTransport (native Bluedroid GAP)
                                                               |
                                      ADV_NONCONN_IND, same public BLE address
```

`RID_BLE_LEGACY_ENABLED`, default 1 in `src/main.cpp`, controls initialization
and loop integration. It can be overridden with a compiler definition of 0.
Broadcasting is independent of `TIMING_ONLY` and existing packet diagnostics.
Initialization occurs once, before the FC UART opens. Runtime processing stays
in the existing loop, immediately after `updateFreshness()`, before verbose
Serial diagnostics. No UART, Adapter, state, mapper, or encoder behavior changed.

The separate `RidLocationAgeMonitor` observes boot timestamps from the same
selected FC's SYSTEM_TIME and GPI messages. Known-old or clock-ambiguous GPI
data is invalidated only in the outgoing copy passed to the existing mapper and
encoder. Shared DroneState, production freshness, and takeoff references remain
unchanged. No encoded message bytes are patched after encoding.

`RidBleBroadcaster` is platform independent. Its injected `RidBleTransport`
allows the actual frame construction and scheduler to run in host tests.
`Esp32RidBleTransport` owns the Bluedroid stack and GAP callback exclusively;
it rejects an already-owned stack rather than overriding another BLE user.
Callbacks enqueue small completion records in a static FreeRTOS queue. Encoding,
state access, and Serial output happen only in the main loop.

The backend uses nonconnectable, nonscannable legacy advertising on channels
37, 38 and 39 with uncoded LE 1M PHY. It sets no scan response, name, Flags, Tx Power
AD structure, GATT services, or extended advertising. No application NVS or
Preferences storage is added. Arduino/SDK internal initialization is unchanged.

## Advertising Data: exactly 31 bytes

| AD offset | Bytes | Meaning |
| --- | --- | --- |
| 0 | `1E` | AD length: 30 bytes following the length byte |
| 1 | `16` | Service Data, 16-bit UUID |
| 2-3 | `FA FF` | UUID `0xFFFA`, little endian |
| 4 | `0D` | OpenDroneID application code |
| 5 | counter | Unsigned 8-bit message counter |
| 6-30 | 25-byte message | Basic ID (`02 ...`), Location (`12 ...`), or Operator ID (`52 ...`) |

This matches the [OpenDroneID Linux transmitter's legacy framing](https://github.com/opendroneid/transmitter-linux/blob/main/bluetooth.c#L145-L180).
Its extra HCI command length byte `1F` is **not** part of the AD buffer passed
to ESP-IDF. There is only one AD structure; no bytes are available for another.

Basic ID, Location and Operator ID have separate message counters, as described by the
OpenDroneID Core C transport guidance. Each starts at zero and increments after
a successful advertising-start completion, wrapping `255 -> 0`. Retransmissions
of an unchanged frame during its dwell retain the counter. Location refreshes
before starting do not consume extra counters. All messages use the same
controller and public BLE address; the MAC is printed at initialization.

For unchanged payloads the chosen permitted policy increments only that type's
counter on each successful publication. It does not increment other types' counters on
a switch. Failed configure/start requests consume no counter. Per-type storage
is sized by a COUNT constant; future types require an explicit scheduling policy
and encoder. Multipage messages are not supported in V1.

## Scheduler and failure handling

- The radio interval is 100 ms (`0x00A0` in 0.625 ms units). Each message dwells
  for 250 ms from its start acknowledgement, allowing repeated radio
  transmissions. With prompt callbacks and a responsive loop, the schedule is
  Location -> Basic -> Location -> Operator. Location updates about twice per
  second; each static ID about once per second. Absent/invalid IDs are skipped.
  This targets Location <=1 s and each static message <=3 s.
- The sequence is `configure raw -> configure acknowledgement -> start -> start
  acknowledgement -> dwell -> stop -> stop acknowledgement -> next configure`.
  No payload mutation is submitted while advertising is running. There are no
  runtime delays, spin waits, timer tasks, or blocking waits for GAP callbacks.
- Location is generated from the latest freshness-checked state when preparing
  its frame. After configure completion, it is encoded again before start;
  if bytes changed, the stopped controller is reconfigured. This includes any
  validity changes while the previous request was pending.
- A known-old or clock-ambiguous GPI clears position, horizontal/vertical speed,
  direction, height and timestamp validity in the outgoing snapshot only. Status,
  GPS-derived fields and pressure keep their existing validity. If an active
  Location packet becomes known old, an early stop is requested instead of
  completing its normal dwell. Age tracking follows the prepared packet: a new
  GPI cannot hide an older packet still advertising. Controller latency still
  requires hardware verification; a stop request does not prove RF has stopped.
- Configure/refresh/start share a 1000 ms completion deadline. Stop has its own
  1000 ms deadline. Repeated refreshes cannot extend a transaction indefinitely.
  Timers use unsigned subtraction across `millis()` wraparound. Late callbacks
  cannot start an old prepared frame after a long main-loop stall.
- After a delayed loop, Location is preferred if its last publication is at
  least 750 ms old; missed slots are not replayed in a catch-up burst. Publication
  gaps over 1000 ms for Location or 3000 ms for Basic/Operator ID produce warnings.
  Timing cannot be guaranteed during controller faults or long loop stalls;
  measured reception cadence must be checked on hardware.
- Diagnostics measure stop-completion to start-completion `restartGap_ms`, its
  maximum, stop-request to start-completion `switch_ms`, callback delivery delay,
  and per-type start-completion gaps. Native callbacks use the same clock as
  Arduino millis(). These are controller observations, not last-RF-packet to
  first-RF-packet gaps. Android packet history/RF capture remains necessary.
- SDK request errors, failed completions, unexpected events, queue overflow,
  or missing completions halt further updates and request a best-effort stop.
  Reset is required after a fault. If the controller itself fails, stopping RF
  cannot be guaranteed; Serial reports the failing operation/status. MAVLink
  parsing and production freshness continue independently of broadcaster health.

Basic ID is validated using the existing encoder at initialization. An empty,
unsupported, or invalid identity is reported and suppressed; Location continues.
No fallback identity or zero-filled Basic ID is transmitted. Valid Basic ID is
cached because configuration is static until reset. Current configuration is the
user's development-only `TEST4DEV1`, ID Type 1, UA Type 2. Syntax validation does
not establish assignment or uniqueness; no production serial is assigned yet.

## Host and firmware verification

```powershell
python tests/run_ble_tests.py
python tests/run_operator_id_tests.py
python tests/run_basic_id_tests.py
python tests/run_location_tests.py
platformio run -e m5stick-c
```

BLE results: 93,032 assertions, zero failures (including the original two-message
regressions and new three-message tests). The suite exercises complete
golden framing, UUID/application, guards, exact payload preservation, callback
ordering, 250 ms acknowledged dwell, alternation and publication deadlines,
more than two wraps of all three counters, invalid/unset Basic ID suppression,
freshness handoff and refresh while configuring, delayed-loop priority,
rollover, request/completion errors, late/missing callbacks and queue faults.
Additional tests cover both actual encoders' exact insertion and immutable
inputs, unchanged-payload counters, failed publications, freshly received
60-second-old GPI data, stale replays, unknown/ambiguous clocks and recovery,
active-packet age isolation/early stopping, and timestamped GAP gap accounting.
It uses a fake asynchronous GAP transport, not a fake encoder. It does not
simulate ESP32 RF. Results: `.pio/host-ble-tests/results.txt`.

Existing tests remain unchanged: Basic ID 12,884 assertions / 536 full-packet
OpenDroneID comparisons; Location 4,979 assertions / 1,238 packets / 1,200
OpenDroneID comparisons. Both pass. The Location suite still reports its two
previously documented adapter observations; BLE does not change those policies.

## Device observation after a separately authorized firmware upload

1. Open Serial Monitor at 115200 baud and reset. Expect `[RID BLE] init=OK`,
   the public MAC and `ADV_NONCONN_IND LE1M channels=37,38,39`. A concise BLE/age
   summary prints every five seconds by default. Expect `len=31`, increasing
   `ok`, `failed=0` and `BUR0030=OPEN`. Errors and guard transitions print
   immediately. `active=NO` can appear during a payload switch. Controller
   completion counts do not prove reception; check packet history below.
2. Use [nRF Connect for Android](https://github.com/nordicsemi/Android-nRF-Connect)
   or an equivalent scanner. Enable Bluetooth and grant requested scan/Nearby
   Devices permissions (and Location access/services if required by the Android
   version). Scan nearby and match the MAC printed on Serial. There is no
   advertised device name; include unnamed and nonconnectable advertisers.
3. Inspect advertisement packet history/raw data, rather than only the latest
   scan row. Do not connect. Inspect Service Data UUID
   `0000fffa-0000-1000-8000-00805f9b34fb`. A UUID-list filter can miss this device:
   the UUID appears in **Service Data**, not a separate service-list AD element.
4. The full raw AD must be 31 bytes: `1E 16 FA FF 0D counter ...25 RID bytes...`.
   A scanner displaying only service data after the UUID shows **27** bytes:
   `0D counter ...25 RID bytes...`. That is the same advertisement with its AD
   header and UUID removed by the UI, not a shorter transmitted packet.
5. Observe RID headers `12` for Location, `02` for Basic and `52` for Operator at AD offset 6.
   Confirm all appear from the same MAC, counters advance modulo 256 for each
   type, and repeated radio packets within one dwell may repeat the counter.
   Record a packet history to measure per-type receive gaps. RF loss can make
   observed gaps larger than the firmware's configured update period.
6. In the [OpenDroneID Android receiver](https://github.com/opendroneid/receiver-android),
   enable Bluetooth scanning and grant its requested permissions. Confirm one
   transmitter with `TEST4DEV1` and live Location/status. Its
   [BLE scanner](https://github.com/opendroneid/receiver-android/blob/master/Android/app/src/main/java/org/opendroneid/android/bluetooth/BluetoothScanner.java)
   matches Service Data UUID FFFA plus application code 0D. Other RID fields
   remain unavailable until their messages are implemented. If nothing appears,
   verify raw advertisements and permissions first; app displays can depend on
   valid position and phone support.
7. Continue MAVLink timing/freshness observation while scanning. Confirm UART
   counts continue, stale telemetry retains existing unknown encodings, and
   arming/height-reference behavior remains unchanged. To test invalid identity
   suppression separately, use an empty configuration in a future test build;
   expect a suppression message and Location advertisements only.

## Age diagnostics and BUR0030: OPEN

| Diagnostic | Meaning |
| --- | --- |
| `rx_ms` | Local time since receiving the GPI associated with this outgoing Location. |
| `timestampEstimate_ms` | Estimated current FC boot time minus the GPI boot tag. UNKNOWN without a recent SYSTEM_TIME boot anchor. |
| `sampleAge=UNVERIFIED` | True physical/estimator time of applicability is not established by the current input contract. |
| `prepared_ms` | Time since encoding/preparing or revalidating the exact outgoing bytes. |
| `advertised_ms` | Time since controller start acknowledgement, not first RF transmission. |

The estimate uses `SYSTEM_TIME.time_boot_ms + local elapsed since reception`
minus `GLOBAL_POSITION_INT.time_boot_ms`, with modular uint32 clock handling.
Its provisional 5000 ms horizon is independent of production UTC holdover and
does not change it. SYSTEM_TIME's boot clock can be observed even without a
valid UTC field. No UTC or accuracy values are manufactured. UART/bridge delays
and clock error are not bounded, so this is a timestamp estimate, not proof of
the actual sample age required by BUR0030.

Tag ages over 1000 ms suppress GPI-derived values through existing unknown
encodings. Ambiguous/future tags and a position spanning a detected backward FC
clock change are guarded too. Evidence of an already-old tag remains latched
through loss of the anchor and replays behind a previously observed FC clock.
New current tags recover without creating any arming transition or reference.
If age cannot be established, V1 retains production receive-validity behavior
and explicitly reports UNKNOWN/UNVERIFIED with `BUR0030=OPEN`.

[ArduPilot's current GPI sender](https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp#L5684-L5702)
uses the current `AP_HAL::millis()` at message generation and can retain an old
position when its location query fails. Even a recent GPI tag therefore does
not prove a recent physical solution. The actual installed FC version and its
estimator timing contract still need verification. GPS/pressure applicability
ages, transport/clock bounds and data age at RF transmission remain unresolved.

## Optional diagnostics

All verbose capture flags below default to 0 in `main.cpp`:

```text
RID_BLE_VERBOSE_DIAGNOSTIC=1       print every successful advertising update
RID_BLE_PACKET_DIAGNOSTIC=1        print the exact 31-byte AD buffer in hex
RID_MAVLINK_VERBOSE_DIAGNOSTIC=1   enable existing timing/freshness reports
RID_LOCATION_PACKET_DIAGNOSTIC=1   enable existing 25-byte Location capture
RID_BASIC_ID_PACKET_DIAGNOSTIC=1   enable existing 25-byte Basic ID capture
```

For a capture build, set the first two flags to 1 in main.cpp or supply `-D`
compiler definitions. They work with `TIMING_ONLY=true`. After each successful
start acknowledgement a `[RID BLE AD31]` label identifies the type/counter,
followed by exactly 31 space-separated hex bytes on one line. Compare with the
Android raw AD view, allowing live data/counters to advance between captures.
Use short verbose captures and restore flags to 0 for timing measurements.
An `ok` increment counts GAP start completions, not proven RF delivery.

## ASTM implementation/evidence report

The following requirements are those supplied by the project. Host/build
results do not certify the complete ASTM standard or prove RF behavior.

| Requirement | Implemented / host evidence | Remaining evidence or gap |
| --- | --- | --- |
| BB40010 | Original ESP32 legacy API, uncoded LE 1M. | RF/receiver confirmation pending. |
| BB40030 | Exact single 31-byte AD structure; golden tests; no extra AD fields or physical-layer bytes. | On-air capture pending. |
| BUR0010 | Existing 250 ms windows give about a 500 ms Location cycle plus callback overhead; host limits pass. | Real callback/reception intervals unverified. Faults or long stalls can exceed 1 s and are reported. |
| BUR0020 | Static slots alternate Basic/Operator between Location slots, refreshing each valid ID about every 1 s plus overhead. Invalid/unset identity is suppressed. | Hardware timing confirmation pending. |
| BUR0030 | Receive/tag/prepared/advertising ages distinguished; known-old GPI guard tested. | **OPEN:** true applicability age, FC semantics, latency/clock bounds and age on air. |
| BUR0040 | Repeated nonconnectable events on all three channels; no receiver ACK assumed. | Interference/saturation performance untested. GAP control faults halt updates with best-effort stop/reset required. |
| BUR0050 | Separate uint8 counters; wraparound, unchanged-payload policy and failures tested. | Confirm counters in received packets. |
| BUR0060 | No multipage message is implemented. | Not applicable to these single-page messages; required before multipage support. |

The existing 25-byte Location/Basic ID outputs were hardware-tested before this
work. New BLE transport, timing diagnostics and age guard have source/host/build
evidence only. No BLE on-air, interference or end-to-end sample-age test has been
performed here. No production identifier assignment or full ASTM compliance is
claimed. No firmware upload, commit, Preferences or NVS implementation occurs.
