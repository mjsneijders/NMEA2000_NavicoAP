# N2k_NavicoAP

C++ message helpers for Navico/Simrad SimNet autopilots, extending the
[mjsneijders/NMEA2000 library](https://github.com/mjsneijders/NMEA2000).

The module builds and parses NMEA 2000 message payloads using `tN2kMsg`.
It follows the library's setter/parser style, with numbered functions and
descriptive inline aliases. It does not open the CAN bus, send messages,
discover pilots, or manage autopilot state itself.

## Supported messages

All messages below can be both set and parsed, regardless of how the original
autopilot plugin uses them.

| PGN | Message | Payload | Setter priority |
|---:|---|---|---:|
| 130850 | SimNet AP command | Fast packet; 11 bytes for mode commands, 12 for course changes | 2 |
| 65305 | SimNet pilot mode, report `0x0A` | Single frame; 8 bytes | 7 |
| 65305 | SimNet device status, report `0x02` | Single frame; 8 bytes | 7 |
| 65341 | SimNet autopilot angle | Single frame; 8 bytes | 6 |
| 127237 | Heading/Track Control | Fast packet; 21 bytes | 2 |

PGN **60928 (ISO Address Claim)** remains the responsibility of NMEA2000.
PGN **130851 (AP command reply)** is not implemented. The original copied
e-paper display helpers for PGN 128538 are not retained.

See [pgns.md](pgns.md) for byte layouts, lookup values, protocol references,
and the original plugin's behavior.

## Files and requirements

- [N2k_NavicoAP.h](N2k_NavicoAP.h): public declarations, enums, and aliases.
- [N2k_NavicoAP.cpp](N2k_NavicoAP.cpp): payload encoders and decoders.
- [tests/test_navico_ap.cpp](tests/test_navico_ap.cpp): host-side regression tests.

Use a C++11 or newer compiler and the NMEA2000 library, including its
`N2kMessages.h` and PGN 127237 implementation. An actual network application
also needs a compatible NMEA2000 CAN driver and hardware.

Add the header and implementation to your application, make the NMEA2000
headers available to the compiler, and compile/link both libraries. For an
Arduino sketch, the two module files can be placed alongside the sketch.

```cpp
#include "N2k_NavicoAP.h"
```

## API overview

Each descriptive function below has a corresponding parser with the same
name prefix changed from `Set` to `Parse`.

| Message | Numbered setter/parser | Descriptive setter |
|---|---|---|
| AP command | `SetN2k130850` / `ParseN2k130850` | `SetN2kSimnetAPCommand` |
| Pilot mode | `SetN2k65305` / `ParseN2k65305`, mode overload | `SetN2kSimnetPilotMode` |
| Device status | `SetN2k65305` / `ParseN2k65305`, status overload | `SetN2kSimnetDeviceStatus` |
| Autopilot angle | `SetN2k65341` / `ParseN2k65341` | `SetN2kSimnetAutopilotAngle` |
| Heading/Track Control | `SetN2k127237` / `ParseN2k127237` | Library-provided `SetN2kHeadingTrackControl` |

### Units and unavailable values

- **Angles:** radians, not degrees. Unsigned angles use `0.0001 rad`
  resolution on the wire.
- **Course changes:** unsigned angle magnitudes with a separate port or
  starboard direction.
- **Wind angles:** returned unsigned, including values above pi. Conversion
  to a signed display angle belongs in the application.
- **PGN 127237 distances:** metres.
- **PGN 127237 rate of turn:** radians per second.
- **Unavailable numbers:** use `N2kDoubleNA` and check with `N2kIsNA`.

The proprietary angle parsers and the local 127237 heading-to-steer parser
map raw values `0xFFFD` through `0xFFFF` to `N2kDoubleNA`.

### Parsing and validation

Parsers return `true` for a matching message. They return `false` without
changing output arguments when the PGN or payload length is incorrect.
The SimNet parsers also check the Simrad manufacturer and marine-industry
header, the command type for 130850, and the requested report variant for
65305.

Reserved and spare fields are ignored when parsing. Unknown enum values
and mode bits are preserved; a successful parse is not confirmation that
the application understands or should act on the decoded value.
Setters encode the supplied fields; callers remain responsible for supplying
valid command arguments.

## Network integration

The module operates on complete payloads. NMEA2000 handles CAN identifiers,
address claiming, and fast-packet fragmentation/reassembly.

Register proprietary receive messages and advertise the PGNs your application
uses before opening the bus. For a controller that sends AP commands and
receives pilot reports, define persistent arrays such as:

```cpp
const unsigned long transmitPGNs[] PROGMEM = {130850L, 0};
const unsigned long receivePGNs[] PROGMEM = {65305L, 65341L, 127237L, 0};
const unsigned long singleFramePGNs[] PROGMEM = {65305L, 65341L, 0};
```

With your NMEA2000 driver instance named `NMEA2000`, call these during setup:

```cpp
NMEA2000.ExtendSingleFrameMessages(singleFramePGNs);
NMEA2000.ExtendTransmitMessages(transmitPGNs);
NMEA2000.ExtendReceiveMessages(receivePGNs);
```

The target library already classifies 127237 and the proprietary range
containing 130850 as fast packet. If your application also **receives**
130850 commands, register it as a known receive message:

```cpp
const unsigned long fastPacketPGNs[] PROGMEM = {130850L, 0};
// During setup:
// NMEA2000.ExtendFastPacketMessages(fastPacketPGNs);
```

Add it to the advertised receive list as well. Adapt transmit lists when
simulating reports or transmitting 127237.

Merge these arrays with any existing application extensions:
`ExtendSingleFrameMessages` and `ExtendFastPacketMessages` each replace the
previous extension list. Arrays must outlive the driver. Advertised
transmit/receive lists alone do not register messages for receive processing.

Received source addresses are available through `msg.Source`. Pilot discovery
and tracking a pilot across address changes are application responsibilities.
Pass messages from the driver's receive callback to the appropriate parsers.

## Examples

### Build an AP mode command

```cpp
void BuildHeadingCommand(tN2kMsg &msg, uint8_t pilotAddress) {
  SetN2kSimnetAPCommand(msg, pilotAddress, N2kSimnetEventHeading);
}
```

Other common events include `N2kSimnetEventStandby`, `N2kSimnetEventNav`,
`N2kSimnetEventNoDrift`, and `N2kSimnetEventWind`.

The command setter uses CAN destination **255 (broadcast)**. The selected
pilot address is carried separately in the payload. Network Group defaults
to `0xFF`.

Setters only build messages. Send a constructed message using your driver's
`SendMsg` method and handle its return value in your application.

### Build a port course-change command

```cpp
void BuildPortTenDegrees(tN2kMsg &msg, uint8_t pilotAddress) {
  const double radians = 10.0 * 3.14159265358979323846 / 180.0;
  SetN2kSimnetAPCommand(msg, pilotAddress, N2kSimnetEventChangeCourse,
                        N2kSimnetCoursePort, radians);
}
```

Use `N2kSimnetCourseStarboard` for the other direction. Always supply
direction and angle for a usable course-change command: their defaults
encode unknown/not available.

Non-course events ignore direction and angle and use an 11-byte payload.
Course changes use 12 bytes. Command byte 7 is set to `0x00`, as required
by the documented NAC-3 behavior.

### Parse an AP command

```cpp
void HandleCommand(const tN2kMsg &msg) {
  uint8_t pilotAddress;
  uint8_t networkGroup;
  tN2kSimnetAPEvent event;
  tN2kSimnetCourseDirection direction;
  double angle;

  if (!ParseN2kSimnetAPCommand(msg, pilotAddress, event, direction,
                              angle, networkGroup)) {
    return;
  }

  // Dispatch the decoded event according to your application's policy.
}
```

For non-course events, the parser returns
`N2kSimnetCourseDirectionUnknown` and `N2kDoubleNA`.

### Parse pilot mode flags

```cpp
bool ReportsHeadingMode(const tN2kMsg &msg) {
  tN2kSimnetDeviceModel model;
  uint16_t modes;
  if (!ParseN2kSimnetPilotMode(msg, model, modes)) {
    return false;
  }
  return (modes & N2kSimnetPilotHeading) != 0;
}
```

Modes are independent flags: Standby, Heading, Nav, No Drift, and Wind.
Multiple bits may be set; the module does not choose one mode by precedence.
`ReportsHeadingMode` above intentionally returns false for both nonmatching
messages and reports without the heading bit; use the parser directly when
your application needs to distinguish those cases.

To construct a mode report:

```cpp
void BuildModeReport(tN2kMsg &msg) {
  SetN2kSimnetPilotMode(msg, N2kSimnetModelNAC,
                        N2kSimnetPilotHeading | N2kSimnetPilotWind);
}
```

### Build and parse device status

```cpp
void BuildAutomaticStatus(tN2kMsg &msg) {
  SetN2kSimnetDeviceStatus(msg, N2kSimnetModelNAC,
                           N2kSimnetStatusAutomatic);
}

void HandleStatus(const tN2kMsg &msg) {
  tN2kSimnetDeviceModel model;
  tN2kSimnetAPStatus status;
  if (!ParseN2kSimnetDeviceStatus(msg, model, status)) {
    return;
  }
  // Inspect status and associate the report with msg.Source.
}
```

Known models are `N2kSimnetModelAC` and `N2kSimnetModelNAC`. Known status
values are `N2kSimnetStatusManual` and `N2kSimnetStatusAutomatic`.

Prefer the descriptive helpers for 65305. The numbered functions are
overloaded: `uint16_t` selects a mode report, while `tN2kSimnetAPStatus`
selects a status report. Passing a numeric literal instead of a typed status
selects the mode overload.

### Build and parse an autopilot angle

```cpp
void BuildWindTarget(tN2kMsg &msg) {
  const double radians = 45.0 * 3.14159265358979323846 / 180.0;
  SetN2kSimnetAutopilotAngle(msg, N2kSimnetAngleWind, radians);
}

void HandleAngle(const tN2kMsg &msg) {
  tN2kSimnetAngleMode mode;
  double angle;
  if (!ParseN2kSimnetAutopilotAngle(msg, mode, angle)) {
    return;
  }
  if (N2kIsNA(angle)) {
    // Report that no target angle is available.
    return;
  }
  // Use angle in radians, interpreted according to mode.
}
```

Angle modes are Heading, Wind, Nav, and No Drift.

## Heading/Track Control (127237)

The module reuses the NMEA2000 library's field types and encoder/decoder
instead of duplicating the standard PGN implementation.

The local numbered functions accept all 17 payload fields in wire order:

1. Rudder Limit Exceeded, Off-Heading Limit Exceeded, Off-Track Limit Exceeded,
   and Override (`tN2kOnOff`).
2. Steering Mode, Turn Mode, Heading Reference, and Commanded Rudder Direction
   (the corresponding library enums).
3. Commanded Rudder Angle, Heading-To-Steer, Track, Rudder Limit, Off-Heading
   Limit, Radius of Turn Order, Rate of Turn Order, Off-Track Limit, and
   Vessel Heading (`double`).

See the exact signatures in [N2k_NavicoAP.h](N2k_NavicoAP.h).

`SetN2k127237` delegates to `SetN2kPGN127237`.
`ParseN2k127237` delegates to `ParseN2kPGN127237` but additionally:

- Requires exactly 21 payload bytes before parsing.
- Maps raw heading-to-steer values `0xFFFD` through `0xFFFF` to
  `N2kDoubleNA`.

The library also provides `SetN2kHeadingTrackControl` and
`ParseN2kHeadingTrackControl`. Those aliases call the library directly;
they do **not** apply the local parser's additional checks.

## Testing

The [regression tests](tests/test_navico_ap.cpp) link against the real
NMEA2000 implementation, not mock message accessors. They cover:

- Exact payload bytes and lengths.
- Documented AP events and angle modes.
- Both 65305 variants and combined mode flags.
- Signed and unsigned fixed-point values.
- Unavailable-value thresholds.
- Malformed-message rejection without changing output arguments.

To run on Windows with a C++11-capable `g++` on PATH, set `$NMEA2000`
to your library checkout and run from this module's directory:

```powershell
$NMEA2000 = 'C:\path\to\NMEA2000'
g++ -std=c++11 -I. "-I$NMEA2000\src" tests\test_navico_ap.cpp N2k_NavicoAP.cpp "$NMEA2000\src\N2kMsg.cpp" "$NMEA2000\src\N2kMessages.cpp" "$NMEA2000\src\N2kStream.cpp" -o "$env:TEMP\test_navico_ap.exe"
if ($LASTEXITCODE -eq 0) { & "$env:TEMP\test_navico_ap.exe" }
```

Host-side tests passed against library revision `e0a5dd9`. They do not
verify CAN hardware, bus timing, or acceptance by a physical autopilot.
