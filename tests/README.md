**Location/Vector host tests**

Run from the project root:

```powershell
python tests/run_location_tests.py
```

The runner needs Python 3, GCC, G++, and the project's installed MAVLink C
headers. It locates `.pio/libdeps/*/c_library_v2` automatically. Override tools
or headers with `--cc`, `--cxx`, and `--mavlink` if needed. It makes no network
requests, opens no serial port, and runs no firmware build or upload.

The C++ test links the actual project Mapper, Encoder, and Adapter. OpenDroneID
Core C is compiled separately as C, from the unmodified vendored reference in
`reference/opendroneid`. Its commit and license are recorded beside the files.
The reference is used only by host tests and is not linked into ESP32 firmware.

Coverage includes complete 25-byte golden vectors, 1,000 deterministic varied
packets compared byte for byte with OpenDroneID, every status currently in
DroneState, unknown fields, AGL/takeoff type, height 10 m, signed coordinates,
speed/direction boundaries, altitude/timestamp boundaries, accuracy thresholds
and adjacent floating-point values, nonfinite inputs, reserved bits, buffer
guards, freshness isolation/recovery, and arming-reference protection.

The extreme-direction test runs in a child process with a three-second deadline
so a mapper hang is reported as a failure instead of hanging the test runner.

Current result: 4,979 assertions, 1,238 complete packets, and 1,200 independent
OpenDroneID packet comparisons; zero failures. Results are saved under
`.pio/host-location-tests/results.txt`. The output also reports two open adapter
observations; those are review findings, not claims of conformance.

**Reference normalization and deliberate differences**

- The reference adapter consumes physical DroneState fields and validity flags;
  it never consumes RidLocationMapped output. Upstream performs its own mapping
  and packing, including accuracy categorization.
- Project validity flags are authoritative. Unavailable values are translated to
  upstream sentinels. Valid physical speeds/altitudes beyond the wire range are
  clamped before entering upstream's stricter input API. Finite angles are
  normalized independently using double-precision remainder.
- Golden vectors were generated independently using Python `struct.pack` with
  specified integer field codes. The test stores the resulting full literals.
- UTC boundary tests use an integer-microsecond rounding oracle; the upstream
  API's float seconds lose precision immediately around some rounding points.
- A valid altitude just above -1000 m is clamped to code 1 by project policy,
  preserving the reserved unknown code 0. Upstream rounds -999.99 m to code 0.
  This case uses an explicit integer expectation instead of claiming equivalence.
- Explicitly valid zero accuracy means known zero in DroneState and selects the
  finest category. Upstream convenience helpers treat zero as unavailable.
  Separate tests exercise both valid-zero and unavailable-zero cases. The MAVLink
  adapter currently treats zero GPS uncertainty as unavailable.

Packet equality proves the tested binary representation. It does not establish
that incoming uncertainty values are 95% bounds, that GPS_RAW uncertainty bounds
the fused GLOBAL_POSITION_INT solution, or that source samples satisfy ASTM
data-age requirements. See `../docs/location_vector_audit.md`.

**Basic ID V1 host tests**

```powershell
python tests/run_basic_id_tests.py
```

This suite is independent of MAVLink and does not alter the Location test runner.
It links the actual `RidBasicIdEncoder.cpp` and separately compiled, pinned
OpenDroneID Core C. Python 3 and GCC/G++ are sufficient. Compiler paths can be
specified with `--cc` and `--cxx`; no serial, network, firmware build, or upload
is performed by this runner.

All identity fixtures are DEVELOPMENT-ONLY. `TEST4DEV1` has serial syntax for
testing, but no manufacturer-code assignment, aircraft identity, authenticity,
or uniqueness is claimed. CAA fixtures likewise assert no real registration.
These fixtures are not used in device configuration.

Results: 12,884 assertions, 536 complete 25-byte OpenDroneID comparisons, zero
failures. Coverage includes a fixed golden vector, every serial length code
1..F, all 16 UA types, every byte value at each position of a sample serial,
20-character identifiers, exact text bytes, zero padding, reserved zeros,
invalid ID/UA types, invalid serial/CAA formats, hidden text after a terminator,
non-ASCII input, null/short/oversized output buffers, and clearing stale packet
bytes on validation failure. Output is saved in
`.pio/host-basic-id-tests/results.txt`.

OpenDroneID's Basic ID encoder packs and truncates text; it does not validate
CTA serial syntax or real assignment. Byte comparisons use identifiers accepted
by the project's stricter validator. Project rejection tests exercise those
additional rules independently. See `../docs/basic_id_v1.md` for configuration,
wire layout, and unsupported binary ID types.

**BLE 4 legacy advertising host tests**

```powershell
python tests/run_ble_tests.py
```

Python 3 and G++ are required (`--cxx` can select the compiler). This runner
links the actual broadcaster, Location mapper/encoder and Basic ID encoder to
an asynchronous fake GAP transport. It opens no serial port and uses no radio.
The existing encoder/reference suites above remain unchanged.

Results: 16,313 assertions, zero failures. Coverage includes full 31-byte golden
framing, UUID/application code, buffer guards, unchanged message bytes,
alternation, per-type counters through wraparound, invalid/unset Basic ID,
callback ordering, acknowledged dwell and publication deadlines, state freshness
handoff/refresh during configuration, timer rollover, delayed loops, immediate
and completion errors, late/missing callbacks and queue faults. Output is saved
under `.pio/host-ble-tests/results.txt`. See `../docs/ble_legacy.md` for stack
inspection, radio timing, fault behavior, and Android on-air verification.

The BLE suite additionally checks unchanged-payload counter policy, successful
versus failed publications, immutable encoder bytes, a freshly received GPI
with a 60-second-old boot timestamp, unknown/ambiguous clocks, stale replay
latching and recovery, isolation of the outgoing guard from shared telemetry,
age of the actual prepared packet, early stopping, and timestamped GAP restart
gap accounting. Sensor applicability age and RF transmission are not proven by
these tests; BUR0030 remains open. Existing encoder/reference suites are preserved.
