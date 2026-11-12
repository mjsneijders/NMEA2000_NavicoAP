# NMEA 2000 PGNs used by the Navico Autopilot plugin

This note documents the NMEA 2000 PGNs the plugin sends or listens to, their
payload fields, and enumerated values relevant to the plugin.

The field layouts and lookup values below follow the [canboat PGN
database](https://github.com/canboat/canboat/tree/master/database). The
plugin-specific behavior follows the project's [`README.md`](https://github.com/mjsneijders/navico_autopilot_pi/blob/1600b6678b9fc08623645ae7a34c76ba485a571c/README.md)
and [`simnet.cpp`](https://github.com/mjsneijders/navico_autopilot_pi/blob/1600b6678b9fc08623645ae7a34c76ba485a571c/src/simnet.cpp).

## Conventions and scope

- Field offsets below are byte offsets in the reassembled PGN payload, starting
  at zero. Bit fields are transmitted least-significant bit first within each
  byte; multi-byte numeric fields are little-endian.
- `Reserved` and `Spare` bytes are part of the payload even where the plugin
  does not interpret them. NMEA 2000 special values such as *not available* can
  apply to numeric fields; those are called out where the plugin handles them.
- PGN `130850` is a proprietary SimNet fast-packet PGN. Fast-packet transport
  frames (frame sequence and payload length) are transport framing, not
  additional PGNs or command fields.
- The plugin registers only `130850` for transmit. It listens to `65305`,
  `65341`, `127237`, and `60928`. It does not listen for `130851` replies.

## PGN overview

| PGN | Name | Direction in plugin | Packet / payload |
|---:|---|---|---|
| 130850 | SimNet: AP command | Sent, one per button press | Fast packet; 11 bytes for mode commands, 12 for course changes |
| 65305 | SimNet: device status / pilot mode | Received | Single frame; 8 bytes |
| 65341 | SimNet: autopilot angle | Received | Single frame; 8 bytes |
| 127237 | Heading/Track Control | Received | Fast packet; 21 bytes |
| 60928 | ISO Address Claim | Received | Single frame; 8 bytes |

The source address of a received CAN/NMEA 2000 message is metadata, not one of
the payload fields. The plugin uses it to associate pilot reports with the
pilot's current network address.

## C++ helpers in this project

Include [N2k_NavicoAP.h](N2k_NavicoAP.h) and compile
[N2k_NavicoAP.cpp](N2k_NavicoAP.cpp) alongside the
[NMEA2000 library](https://github.com/mjsneijders/NMEA2000). The copied
e-paper display protocol (PGN 128538) has been replaced, not retained.
The helpers follow the numbered setter/parser and inline descriptive-alias
structure of the original files.

| PGN | Numbered helpers | Descriptive helpers |
|---:|---|---|
| 130850 | `SetN2k130850` / `ParseN2k130850` | `SetN2kSimnetAPCommand` / `ParseN2kSimnetAPCommand` |
| 65305, mode report | `SetN2k65305` / `ParseN2k65305` with `uint16_t` mode flags | `SetN2kSimnetPilotMode` / `ParseN2kSimnetPilotMode` |
| 65305, status report | `SetN2k65305` / `ParseN2k65305` with `tN2kSimnetAPStatus` | `SetN2kSimnetDeviceStatus` / `ParseN2kSimnetDeviceStatus` |
| 65341 | `SetN2k65341` / `ParseN2k65341` | `SetN2kSimnetAutopilotAngle` / `ParseN2kSimnetAutopilotAngle` |
| 127237 | `SetN2k127237` / `ParseN2k127237` | Library-provided `SetN2kHeadingTrackControl` / `ParseN2kHeadingTrackControl` |

PGN 127237 delegates to the library's `SetN2kPGN127237` /
`ParseN2kPGN127237`, using its existing enums and units. The local numbered
parser additionally checks the full payload length and maps raw
heading-to-steer values `0xFFFD` through `0xFFFF` to `N2kDoubleNA`, matching
the plugin. The library's descriptive parser does not add these checks.
PGN 60928 is intentionally left to the library. PGN 130851 is out of scope.

All angles passed to or returned by these helpers are **radians**, including
unsigned course-change magnitudes and wind angles. No conversion to a signed
wind display angle or plugin-specific mode precedence is applied. Distances
in 127237 are metres and rate of turn is radians per second.
Use `N2kDoubleNA` for unavailable numeric values; proprietary unsigned angle
parsers also map raw `0xFFFD` and `0xFFFE` to unavailable.

Parsers return `false` without modifying output arguments for a wrong PGN,
incorrect payload length, wrong proprietary manufacturer/industry header,
wrong command type, or a nonmatching 65305 report variant. Unknown event,
model, status, and mode values are preserved rather than rejected. Reserved
and spare fields are ignored when parsing. Setters fill them with `0xFF`,
except for command byte 7, which must be `0x00`. SimNet report setters use
priority 7 for 65305 and priority 6 for 65341; 130850 and 127237 use priority 2.
Source addresses remain `tN2kMsg::Source` metadata. The 130850 setter always
uses broadcast CAN destination 255, independently of the payload address.

```cpp
#include "N2k_NavicoAP.h"

tN2kMsg msg;
SetN2kSimnetAPCommand(msg, pilotAddress, N2kSimnetEventHeading);
// NMEA2000.SendMsg(msg);

SetN2kSimnetAPCommand(msg, pilotAddress, N2kSimnetEventChangeCourse,
                      N2kSimnetCoursePort, 10.0 * 3.141592653589793 / 180.0);

tN2kSimnetDeviceModel model;
uint16_t modes;
if (ParseN2kSimnetPilotMode(msg, model, modes)) {
  bool headingActive = (modes & N2kSimnetPilotHeading) != 0;
  // Handle the report from msg.Source.
}
```

For 130850, direction and angle apply only to `N2kSimnetEventChangeCourse`.
Supply both for a usable course-change command; defaults encode unknown/NA.
Other events ignore these arguments and parsers return unknown/NA for them.
For 65305, use the named aliases to make the report variant explicit; passing
a numeric literal instead of a typed status selects the **mode** overload.
Mode flags are independent and are not collapsed to a single active mode.

### Transport registration

The helpers operate on **reassembled payloads**, not CAN frames. NMEA2000
handles fast-packet framing. Before opening the bus, register proprietary
receive PGNs as known messages, and advertise the application's PGN lists:

```cpp
const unsigned long transmitPGNs[] PROGMEM = {130850L, 0};
const unsigned long receivePGNs[] PROGMEM = {65305L, 65341L, 127237L, 0};
const unsigned long singleFramePGNs[] PROGMEM = {65305L, 65341L, 0};
const unsigned long fastPacketPGNs[] PROGMEM = {130850L, 0};
// After constructing your NMEA2000 driver:
// NMEA2000.ExtendSingleFrameMessages(singleFramePGNs);
// NMEA2000.ExtendFastPacketMessages(fastPacketPGNs);
// NMEA2000.ExtendTransmitMessages(transmitPGNs);
// NMEA2000.ExtendReceiveMessages(receivePGNs);
```

Adapt the lists to your actual usage (for example, advertise 65305 as a
transmit PGN when simulating pilot reports). The target library already
classifies 127237 and the proprietary range containing 130850 as fast
packet. Explicitly registering 130850 as above also makes it a known receive
PGN if your application listens for commands. Merely extending the
advertised transmit/receive lists does not register known receive messages.
Merge these lists with any existing application extensions: each
`ExtendSingleFrameMessages` / `ExtendFastPacketMessages` call replaces the
previous extension list, and the arrays must outlive the driver.

### Host-side regression tests

[tests/test_navico_ap.cpp](tests/test_navico_ap.cpp) tests exact wire bytes, both command lengths, both
65305 variants, combined mode flags, all documented AP events and angle modes,
fixed-point scaling, unavailable-value thresholds, and rejection of malformed
messages without changing outputs. It links against the real library, not
mock message accessors. With a C++11 compiler on PATH, run from this directory
in PowerShell (set `$NMEA2000` to your library checkout):

```powershell
g++ -std=c++11 -I. "-I$NMEA2000\src" tests\test_navico_ap.cpp N2k_NavicoAP.cpp "$NMEA2000\src\N2kMsg.cpp" "$NMEA2000\src\N2kMessages.cpp" "$NMEA2000\src\N2kStream.cpp" -o "$env:TEMP\test_navico_ap.exe"
if ($LASTEXITCODE -eq 0) { & "$env:TEMP\test_navico_ap.exe" }
```

## PGN 130850 — SimNet: AP command

Priority 2; fast packet. The plugin broadcasts to destination address `255`.
The command contains the target autopilot's address in its payload. The
manufacturer header is Simrad (`1857`), marine industry (`4`).

| Payload byte(s) | Bits | Field | Meaning / value sent by plugin |
|---|---:|---|---|
| 0–1 | 16 | Manufacturer / reserved / industry header | `41 9F`: Simrad manufacturer code `1857`, reserved bits set, marine industry code `4` |
| 2 | 8 | Address | NMEA 2000 source address of the selected autopilot |
| 3 | 8 | Reserved | `FF` |
| 4 | 8 | Network Group | `FF` in plugin commands |
| 5 | 8 | Command Type | `0A` = AP Command |
| 6 | 8 | Event | Selects the autopilot action; see [AP event enum](#simnet-ap-event-8-bit) |
| 7 | 8 | Spare | `00`; the README notes that the NAC-3 ignores commands if this byte is `FF` |
| 8–10 | 24 | Mode-command tail | `FF FF FF` for mode changes |
| 8 | 8 | Course-change direction | `02` = port; `03` = starboard (only for event `1A`) |
| 9–10 | 16 | Course-change angle | Unsigned little-endian value in `0.0001 rad` units; e.g. `AF 00` = 175 units (~1 degree), `D1 06` = 1745 units (~10 degrees) |
| 11 | 8 | Course-change tail | `FF` (present for event `1A`; course-change payload is 12 bytes) |

For course changes, bytes 8–10 are a command-specific union: direction at byte
8 and the 16-bit angle at bytes 9–10 replace the mode-command tail. Mode
commands are 11 bytes total; course-change commands are 12 bytes.

### SimNet AP event (8-bit)

Values defined in canboat; the plugin sends the marked events:

| Value | Event | Plugin action |
|---:|---|---|
| `02` | Follow / Non Follow | — |
| `06` | Standby | Standby |
| `09` | Heading mode | Auto / heading hold |
| `0A` | Nav mode | Nav |
| `0C` | No Drift mode | No Drift |
| `0D` | Non Follow Up mode | — |
| `0E` | Follow Up mode | — |
| `0F` | Wind mode | Wind |
| `11` | Tack | — |
| `12` | Square (Turn) | — |
| `13` | C-Turn | — |
| `14` | U-Turn | — |
| `15` | Spiral (Turn) | — |
| `16` | Zig Zag (Turn) | — |
| `17` | Lazy-S (Turn) | — |
| `18` | Depth (Turn) | — |
| `1A` | Change course | Port/starboard course change |
| `3D` | Timer sync | — |
| `6B` | MOB Activated | — |
| `6C` | MOB Deactivated | — |
| `70` | Ping port end | — |
| `71` | Ping starboard end | — |

## PGN 65305 — SimNet: device status / pilot mode

Single-frame proprietary SimNet PGN, 8 bytes. The plugin uses two report
variants from this PGN. Reports from AC/NAC devices have the `41 9F` Simrad
header. The plugin recognizes model `00` (AC) and `64` (NAC).

### Report `0A` — Mode

| Payload byte(s) | Bits | Field | Meaning |
|---|---:|---|---|
| 0–1 | 16 | Manufacturer / reserved / industry header | `41 9F` identifies Simrad, marine industry |
| 2 | 8 | Model | `00` = AC; `64` = NAC |
| 3 | 8 | Report | `0A` = Mode |
| 4–5 | 16 | Mode bitfield | Bits indicate active modes; see [pilot mode bitfield](#pilot-mode-bitfield-16-bit) |
| 6–7 | 16 | Spare | Not interpreted by plugin |

The plugin treats this report as evidence that a pilot exists at the message's
source address. It uses the mode bits for state display and to confirm mode
commands. Keypad requests (`03` and `0B`) are not treated as pilot mode reports.

### Report `02` — Status

| Payload byte(s) | Bits | Field | Meaning |
|---|---:|---|---|
| 0–1 | 16 | Manufacturer / reserved / industry header | `41 9F` identifies Simrad, marine industry |
| 2 | 8 | Model | `00` = AC; `64` = NAC |
| 3 | 8 | Report | `02` = Status |
| 4 | 8 | Status | `02` = Manual / not engaged; `10` = Automatic / engaged |
| 5–7 | 24 | Spare | Not interpreted by plugin |

### Pilot mode bitfield (16-bit)

The bits are independent flags, not a single numeric enum.

| Bit | Mode |
|---:|---|
| 3 | Standby |
| 4 | Heading (plugin displays Auto) |
| 6 | Nav |
| 8 | No Drift |
| 10 | Wind |

If multiple mode bits are set, the plugin's parser checks them in order and the
last matching mode wins: Standby, No Drift, Heading, Wind, Nav.

## PGN 65341 — SimNet: autopilot angle

Single-frame proprietary SimNet PGN, 8 bytes. The plugin reads the mode and
angle fields, and uses the wind-mode angle as the target wind angle.

| Payload byte(s) | Bits | Field | Meaning |
|---|---:|---|---|
| 0–1 | 16 | Manufacturer / reserved / industry header | Simrad, marine industry |
| 2–3 | 16 | Reserved | Not interpreted by plugin |
| 4 | 8 | Mode | See [autopilot angle mode](#autopilot-angle-mode-8-bit) |
| 5 | 8 | Reserved | Not interpreted by plugin |
| 6–7 | 16 | Angle | Unsigned angle, `0.0001 rad` per unit; `FF FD` and above treated as unavailable/out of range by plugin |

The plugin converts wind angles above π radians to the equivalent negative
angle, yielding a signed `-π..π` display range.

### Autopilot angle mode (8-bit)

| Value | Mode |
|---:|---|
| `02` | Heading |
| `03` | Wind |
| `0A` | Nav |
| `0B` | No Drift |

## PGN 127237 — Heading/Track Control

Standard NMEA 2000 PGN, priority 2; fast packet; 21-byte payload. The plugin
uses the heading reference and heading-to-steer fields. It compares a changed
target heading to confirm a port/starboard course-change command.

| Payload byte(s) | Bit range within byte | Field | Meaning / encoding |
|---|---:|---|---|
| 0 | 0–1 | Rudder Limit Exceeded | Yes/no lookup |
| 0 | 2–3 | Off-Heading Limit Exceeded | Yes/no lookup |
| 0 | 4–5 | Off-Track Limit Exceeded | Yes/no lookup |
| 0 | 6–7 | Override | Yes/no lookup |
| 1 | 0–2 | Steering Mode | Lookup |
| 1 | 3–5 | Turn Mode | Lookup |
| 1 | 6–7 | Heading Reference | `0` True; `1` Magnetic; see enum |
| 2 | 0–4 | Reserved | — |
| 2 | 5–7 | Commanded Rudder Direction | Lookup |
| 3–4 | 16 | Commanded Rudder Angle | Signed angle, `0.0001 rad` per unit |
| 5–6 | 16 | Heading-To-Steer (Course) | Unsigned angle, `0.0001 rad` per unit; `FFFD` and above means unavailable/out of range |
| 7–8 | 16 | Track | Unsigned angle, `0.0001 rad` per unit |
| 9–10 | 16 | Rudder Limit | Unsigned angle, `0.0001 rad` per unit |
| 11–12 | 16 | Off-Heading Limit | Unsigned angle, `0.0001 rad` per unit |
| 13–14 | 16 | Radius of Turn Order | Signed distance, metres |
| 15–16 | 16 | Rate of Turn Order | Signed rotation rate |
| 17–18 | 16 | Off-Track Limit | Signed distance, metres |
| 19–20 | 16 | Vessel Heading | Unsigned angle, `0.0001 rad` per unit |

Angle values use the NMEA 2000 fixed-point angle representation. For the
heading-to-steer value, the plugin returns no target for `FFFD` and above. Its
heading-reference parser reports Magnetic only for enum value `1`; other
values are treated as non-magnetic.

### PGN 127237 enums

**Yes/no fields** (each 2-bit):

| Value | Meaning |
|---:|---|
| `0` | No |
| `1` | Yes |
| `2` | Error / reserved |
| `3` | Unknown / not available |

**Steering Mode** (3-bit):

| Value | Meaning |
|---:|---|
| `0` | Main Steering |
| `1` | Non-Follow-Up Device |
| `2` | Follow-Up Device |
| `3` | Heading Control Standalone |
| `4` | Heading Control |
| `5` | Track Control |

**Turn Mode** (3-bit):

| Value | Meaning |
|---:|---|
| `0` | Rudder limit controlled |
| `1` | Turn rate controlled |
| `2` | Radius controlled |

**Heading Reference** (2-bit):

| Value | Meaning |
|---:|---|
| `0` | True |
| `1` | Magnetic |
| `2` | Error |
| `3` | Reserved / unknown |

**Commanded Rudder Direction** (3-bit):

| Value | Meaning |
|---:|---|
| `0` | No Order |
| `1` | Move to starboard |
| `2` | Move to port |

## PGN 60928 — ISO Address Claim

Single-frame network-management PGN, 8-byte payload. The payload is a
little-endian 64-bit ISO NAME, divided into these fields. The plugin parses
the NAME to identify Navico/Simrad autopilots and remembers the complete NAME
so it can follow the same device if its network address changes.

| Payload bit range | Size | Field | Meaning |
|---|---:|---|---|
| 0–20 | 21 bits | Unique Number | Device identity number |
| 21–31 | 11 bits | Manufacturer Code | Manufacturer lookup |
| 32–34 | 3 bits | Device Instance Lower | ECU/device instance |
| 35–39 | 5 bits | Device Instance Upper | Function instance |
| 40–47 | 8 bits | Device Function | Function lookup; `150` identifies an autopilot in plugin selection |
| 48 | 1 bit | Spare | Reserved bit |
| 49–55 | 7 bits | Device Class | Device class lookup |
| 56–59 | 4 bits | System Instance | System instance |
| 60–62 | 3 bits | Industry Group | Industry lookup |
| 63 | 1 bit | Arbitrary Address Capable | Whether the device can claim an arbitrary source address |

### NAME values used by plugin selection

| Field | Value | Meaning |
|---|---:|---|
| Manufacturer Code | `275` | Navico |
| Manufacturer Code | `1857` | Simrad |
| Device Function | `150` | Autopilot |
| Industry Group / SimNet header industry code | `4` | Marine |

## Related, but not a plugin PGN

PGN `130851` (SimNet AP command reply) is not part of this plugin's current
send/receive path: the plugin sends broadcast `130850` commands and does not
register a listener for `130851`.

The vessel's current heading shown by the plugin comes from OpenCPN's position
fix API, not from a PGN in this list. PGN `127237` supplies the target
heading-to-steer.

## Protocol references

- [PGN 130850 — SimNet AP command](https://github.com/canboat/canboat/blob/master/database/pgns/130850-simnetApCommand.yaml)
- [PGN 65305 — SimNet pilot mode](https://github.com/canboat/canboat/blob/master/database/pgns/065305-simnetPilotMode.yaml) and [device status](https://github.com/canboat/canboat/blob/master/database/pgns/065305-simnetDeviceStatus.yaml)
- [PGN 65341 — SimNet autopilot angle](https://github.com/canboat/canboat/blob/master/database/pgns/065341-simnetAutopilotAngle.yaml)
- [PGN 127237 — Heading/Track Control](https://github.com/canboat/canboat/blob/master/database/pgns/127237-headingTrackControl.yaml)
- [PGN 60928 — ISO Address Claim](https://github.com/canboat/canboat/blob/master/database/pgns/060928-isoAddressClaim.yaml)
- [SimNet AP events lookup](https://github.com/canboat/canboat/blob/master/database/lookups/SIMNET_AP_EVENTS.yaml), [pilot-mode bitfield lookup](https://github.com/canboat/canboat/blob/master/database/lookups/SIMNET_AP_MODE_BITFIELD.yaml), and [autopilot-angle mode lookup](https://github.com/canboat/canboat/blob/master/database/lookups/SIMNET_AP_MODE.yaml)
