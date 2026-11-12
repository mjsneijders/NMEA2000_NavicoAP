#include "N2k_NavicoAP.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" uint32_t millis() { return 0; }
extern "C" void delay(uint32_t) {}

#define CHECK(condition) do { \
  if (!(condition)) { \
    std::fprintf(stderr, "FAILED at line %d: %s\n", __LINE__, #condition); \
    std::exit(1); \
  } \
} while (false)

static void Near(double Actual, double Expected, double Tolerance = 0.000051) {
  CHECK(std::fabs(Actual - Expected) < Tolerance);
}

template <size_t Size>
static void Payload(const tN2kMsg &Msg, const uint8_t (&Expected)[Size]) {
  CHECK(Msg.DataLen == static_cast<int>(Size));
  CHECK(std::memcmp(Msg.Data, Expected, Size) == 0);
}

struct Command {
  uint8_t Address;
  tN2kSimnetAPEvent Event;
  tN2kSimnetCourseDirection Direction;
  double Angle;
  uint8_t NetworkGroup;

  Command() : Address(42), Event(N2kSimnetEventWind), Direction(N2kSimnetCoursePort),
              Angle(1.0), NetworkGroup(9) {}

  bool Parse(const tN2kMsg &Msg) {
    return ParseN2kSimnetAPCommand(Msg, Address, Event, Direction, Angle, NetworkGroup);
  }

  void Unchanged() const {
    CHECK(Address == 42 && Event == N2kSimnetEventWind);
    CHECK(Direction == N2kSimnetCoursePort && Angle == 1.0 && NetworkGroup == 9);
  }
};

static void RejectCommand(const tN2kMsg &Msg) {
  Command Parsed;
  CHECK(!Parsed.Parse(Msg));
  Parsed.Unchanged();
}

static void TestCommands() {
  tN2kMsg Msg;
  Msg.Source = 23;
  Msg.Destination = 3;
  SetN2kSimnetAPCommand(Msg, 3, N2kSimnetEventStandby);
  const uint8_t Standby[] = {0x41, 0x9f, 3, 0xff, 0xff, 0x0a, 6, 0, 0xff, 0xff, 0xff};
  Payload(Msg, Standby);
  CHECK(Msg.PGN == 130850 && Msg.Priority == 2);
  CHECK(Msg.Destination == 255 && Msg.Source == 23);
  Command Parsed;
  CHECK(Parsed.Parse(Msg));
  CHECK(Parsed.Address == 3 && Parsed.Event == N2kSimnetEventStandby);
  CHECK(Parsed.Direction == N2kSimnetCourseDirectionUnknown && N2kIsNA(Parsed.Angle));
  CHECK(Parsed.NetworkGroup == 0xff);

  const tN2kSimnetAPEvent Events[] = {
    N2kSimnetEventFollowNonFollow, N2kSimnetEventStandby, N2kSimnetEventHeading,
    N2kSimnetEventNav, N2kSimnetEventNoDrift, N2kSimnetEventNonFollowUp,
    N2kSimnetEventFollowUp, N2kSimnetEventWind, N2kSimnetEventTack,
    N2kSimnetEventSquareTurn, N2kSimnetEventCTurn, N2kSimnetEventUTurn,
    N2kSimnetEventSpiralTurn, N2kSimnetEventZigZagTurn, N2kSimnetEventLazySTurn,
    N2kSimnetEventDepthTurn, N2kSimnetEventTimerSync, N2kSimnetEventMOBActivated,
    N2kSimnetEventMOBDeactivated, N2kSimnetEventPingPortEnd, N2kSimnetEventPingStarboardEnd
  };
  for (tN2kSimnetAPEvent Event : Events) {
    SetN2k130850(Msg, 17, Event, N2kSimnetCourseStarboard, 0.2, 7);
    CHECK(Msg.DataLen == 11 && Msg.Data[6] == Event && Msg.Data[7] == 0);
    CHECK(Parsed.Parse(Msg));
    CHECK(Parsed.Event == Event && Parsed.Address == 17 && Parsed.NetworkGroup == 7);
    CHECK(Parsed.Direction == N2kSimnetCourseDirectionUnknown && N2kIsNA(Parsed.Angle));
  }

  const double TenDegrees = 10.0 * 3.14159265358979323846 / 180.0;
  SetN2k130850(Msg, 3, N2kSimnetEventChangeCourse, N2kSimnetCoursePort, TenDegrees);
  const uint8_t Port[] = {0x41, 0x9f, 3, 0xff, 0xff, 0x0a, 0x1a, 0, 2, 0xd1, 6, 0xff};
  Payload(Msg, Port);
  CHECK(Parsed.Parse(Msg));
  CHECK(Parsed.Direction == N2kSimnetCoursePort);
  Near(Parsed.Angle, TenDegrees);

  SetN2k130850(Msg, 3, N2kSimnetEventChangeCourse, N2kSimnetCourseStarboard, TenDegrees / 10);
  const uint8_t Starboard[] = {0x41, 0x9f, 3, 0xff, 0xff, 0x0a, 0x1a, 0, 3, 0xaf, 0, 0xff};
  Payload(Msg, Starboard);
  CHECK(Parsed.Parse(Msg));
  CHECK(Parsed.Direction == N2kSimnetCourseStarboard);
  Near(Parsed.Angle, TenDegrees / 10);

  for (int Length = 0; Length <= 13; ++Length) {
    if (Length == 12) continue;
    tN2kMsg Invalid = Msg;
    Invalid.DataLen = Length;
    RejectCommand(Invalid);
  }
  tN2kMsg Invalid = Msg;
  Invalid.PGN = 130851;
  RejectCommand(Invalid);
  Invalid = Msg;
  Invalid.Data[0] ^= 1;
  RejectCommand(Invalid);
  Invalid = Msg;
  Invalid.Data[1] ^= 0x20;
  RejectCommand(Invalid);
  Invalid = Msg;
  Invalid.Data[5] = 0;
  RejectCommand(Invalid);
  Invalid = Msg;
  Invalid.Data[6] = N2kSimnetEventStandby;
  RejectCommand(Invalid);

  Msg.Data[1] &= 0xe7; // Reserved header bits may differ on received messages.
  CHECK(Parsed.Parse(Msg));
  SetN2k130850(Msg, 3, N2kSimnetEventChangeCourse);
  CHECK(Parsed.Parse(Msg) && N2kIsNA(Parsed.Angle));
  for (uint32_t Raw = 0xfffc; Raw <= 0xffff; ++Raw) {
    Msg.Data[9] = static_cast<uint8_t>(Raw);
    Msg.Data[10] = static_cast<uint8_t>(Raw >> 8);
    CHECK(Parsed.Parse(Msg));
    if (Raw >= 0xfffd) CHECK(N2kIsNA(Parsed.Angle));
    else Near(Parsed.Angle, 6.5532);
  }
}

static void TestDeviceReports() {
  tN2kMsg Msg;
  tN2kSimnetDeviceModel Model = N2kSimnetModelAC;
  uint16_t Mode = 0;
  tN2kSimnetAPStatus Status = N2kSimnetStatusManual;
  SetN2kSimnetPilotMode(Msg, N2kSimnetModelNAC, N2kSimnetPilotHeading | N2kSimnetPilotWind);
  const uint8_t ModePayload[] = {0x41, 0x9f, 0x64, 0x0a, 0x10, 4, 0xff, 0xff};
  Payload(Msg, ModePayload);
  CHECK(Msg.PGN == 65305 && Msg.Priority == 7);
  CHECK(ParseN2kSimnetPilotMode(Msg, Model, Mode));
  CHECK(Model == N2kSimnetModelNAC && Mode == (N2kSimnetPilotHeading | N2kSimnetPilotWind));
  CHECK(!ParseN2kSimnetDeviceStatus(Msg, Model, Status));
  CHECK(Status == N2kSimnetStatusManual);

  const uint16_t Modes[] = {0, N2kSimnetPilotStandby, N2kSimnetPilotHeading, N2kSimnetPilotNav,
                           N2kSimnetPilotNoDrift, N2kSimnetPilotWind, 0xffff};
  for (uint16_t Flags : Modes) {
    SetN2k65305(Msg, N2kSimnetModelAC, Flags);
    CHECK(ParseN2k65305(Msg, Model, Mode));
    CHECK(Model == N2kSimnetModelAC && Mode == Flags);
  }
  SetN2kSimnetDeviceStatus(Msg, N2kSimnetModelAC, N2kSimnetStatusAutomatic);
  const uint8_t StatusPayload[] = {0x41, 0x9f, 0, 2, 0x10, 0xff, 0xff, 0xff};
  Payload(Msg, StatusPayload);
  CHECK(ParseN2kSimnetDeviceStatus(Msg, Model, Status));
  CHECK(Model == N2kSimnetModelAC && Status == N2kSimnetStatusAutomatic);
  Mode = 123;
  CHECK(!ParseN2kSimnetPilotMode(Msg, Model, Mode) && Mode == 123);

  // Captured NAC manual-status example, with non-FF spare bytes.
  const uint8_t Manual[] = {0x41, 0x9f, 0x64, 2, 2, 0, 0, 0};
  std::memcpy(Msg.Data, Manual, sizeof(Manual));
  CHECK(ParseN2k65305(Msg, Model, Status));
  CHECK(Model == N2kSimnetModelNAC && Status == N2kSimnetStatusManual);
  const uint8_t KeypadReports[] = {0x03, 0x0b};
  for (uint8_t Report : KeypadReports) {
    Msg.Data[3] = Report;
    Model = N2kSimnetModelAC;
    CHECK(!ParseN2k65305(Msg, Model, Mode));
    CHECK(!ParseN2k65305(Msg, Model, Status));
    CHECK(Model == N2kSimnetModelAC && Mode == 123 && Status == N2kSimnetStatusManual);
  }
  for (int Length = 0; Length <= 9; ++Length) {
    if (Length == 8) continue;
    Msg.DataLen = Length;
    Msg.Data[3] = 2;
    CHECK(!ParseN2k65305(Msg, Model, Status));
    Msg.Data[3] = 0x0a;
    CHECK(!ParseN2k65305(Msg, Model, Mode));
  }
  Msg.DataLen = 8;
  Msg.Data[0] ^= 1;
  CHECK(!ParseN2k65305(Msg, Model, Mode));
  Msg.Data[3] = 2;
  CHECK(!ParseN2k65305(Msg, Model, Status));
  SetN2k65305(Msg, N2kSimnetModelNAC, N2kSimnetStatusManual);
  CHECK(ParseN2k65305(Msg, Model, Status) && Status == N2kSimnetStatusManual);
  Msg.PGN = 65341;
  CHECK(!ParseN2k65305(Msg, Model, Status));
}

static void TestAngles() {
  tN2kMsg Msg;
  tN2kSimnetAngleMode Mode = N2kSimnetAngleHeading;
  double Angle = 0;
  SetN2kSimnetAutopilotAngle(Msg, N2kSimnetAngleWind, 2.3562);
  const uint8_t Wind[] = {0x41, 0x9f, 0xff, 0xff, 3, 0xff, 0x0a, 0x5c};
  Payload(Msg, Wind);
  CHECK(Msg.PGN == 65341 && Msg.Priority == 6);
  CHECK(ParseN2kSimnetAutopilotAngle(Msg, Mode, Angle));
  CHECK(Mode == N2kSimnetAngleWind);
  Near(Angle, 2.3562);

  const tN2kSimnetAngleMode Modes[] = {
    N2kSimnetAngleHeading, N2kSimnetAngleWind, N2kSimnetAngleNav, N2kSimnetAngleNoDrift
  };
  for (tN2kSimnetAngleMode Value : Modes) {
    SetN2k65341(Msg, Value, 4.7124);
    CHECK(ParseN2k65341(Msg, Mode, Angle) && Mode == Value);
    Near(Angle, 4.7124); // Wind angles stay unsigned; no display-level conversion.
  }
  SetN2k65341(Msg, N2kSimnetAngleHeading, 0);
  CHECK(ParseN2k65341(Msg, Mode, Angle) && Angle == 0);
  SetN2k65341(Msg, N2kSimnetAngleHeading);
  CHECK(Msg.Data[6] == 0xff && Msg.Data[7] == 0xff);
  CHECK(ParseN2k65341(Msg, Mode, Angle) && N2kIsNA(Angle));
  for (uint32_t Raw = 0xfffc; Raw <= 0xffff; ++Raw) {
    Msg.Data[6] = static_cast<uint8_t>(Raw);
    Msg.Data[7] = static_cast<uint8_t>(Raw >> 8);
    CHECK(ParseN2k65341(Msg, Mode, Angle));
    if (Raw >= 0xfffd) CHECK(N2kIsNA(Angle));
    else Near(Angle, 6.5532);
  }
  for (int Length = 0; Length <= 9; ++Length) {
    if (Length == 8) continue;
    Msg.DataLen = Length;
    Mode = N2kSimnetAngleWind;
    Angle = 1;
    CHECK(!ParseN2k65341(Msg, Mode, Angle));
    CHECK(Mode == N2kSimnetAngleWind && Angle == 1);
  }
  Msg.DataLen = 8;
  Msg.Data[1] ^= 0x20;
  CHECK(!ParseN2k65341(Msg, Mode, Angle));
  Msg.Data[1] ^= 0x20;
  Msg.PGN = 65305;
  CHECK(!ParseN2k65341(Msg, Mode, Angle));
}

struct HeadingControl {
  tN2kOnOff Rudder, OffHeading, OffTrack, Override;
  tN2kSteeringMode Steering;
  tN2kTurnMode Turn;
  tN2kHeadingReference Reference;
  tN2kRudderDirectionOrder Direction;
  double RudderAngle, Course, Track, RudderLimit, HeadingLimit, Radius, Rate, TrackLimit, Heading;

  bool Parse(const tN2kMsg &Msg) {
    return ParseN2k127237(Msg, Rudder, OffHeading, OffTrack, Override, Steering, Turn, Reference,
                         Direction, RudderAngle, Course, Track, RudderLimit, HeadingLimit,
                         Radius, Rate, TrackLimit, Heading);
  }
};

static void TestHeadingControl() {
  tN2kMsg Msg;
  SetN2k127237(Msg, static_cast<tN2kOnOff>(1), static_cast<tN2kOnOff>(0),
               static_cast<tN2kOnOff>(2), static_cast<tN2kOnOff>(3),
               static_cast<tN2kSteeringMode>(5), static_cast<tN2kTurnMode>(1),
               N2khr_magnetic, static_cast<tN2kRudderDirectionOrder>(2),
               -0.1234, 1.2345, 2.3456, 0.4363, 0.3491, -123, -0.003125, 456, 3.2532);
  const uint8_t Expected[] = {
    0xe1, 0x4d, 0x5f, 0x2e, 0xfb, 0x39, 0x30, 0xa0, 0x5b, 0x0b, 0x11,
    0xa3, 0x0d, 0x85, 0xff, 0x9c, 0xff, 0xc8, 0x01, 0x14, 0x7f
  };
  Payload(Msg, Expected);
  CHECK(Msg.PGN == 127237 && Msg.Priority == 2);
  HeadingControl Parsed = {};
  CHECK(Parsed.Parse(Msg));
  CHECK(Parsed.Rudder == 1 && Parsed.OffHeading == 0 && Parsed.OffTrack == 2 && Parsed.Override == 3);
  CHECK(Parsed.Steering == 5 && Parsed.Turn == 1 && Parsed.Reference == N2khr_magnetic);
  CHECK(Parsed.Direction == 2);
  Near(Parsed.RudderAngle, -0.1234);
  Near(Parsed.Course, 1.2345);
  Near(Parsed.Track, 2.3456);
  Near(Parsed.RudderLimit, 0.4363);
  Near(Parsed.HeadingLimit, 0.3491);
  Near(Parsed.Radius, -123);
  Near(Parsed.Rate, -0.003125, 0.0000001);
  Near(Parsed.TrackLimit, 456);
  Near(Parsed.Heading, 3.2532);

  for (uint32_t Raw = 0xfffc; Raw <= 0xffff; ++Raw) {
    Msg.Data[5] = static_cast<uint8_t>(Raw);
    Msg.Data[6] = static_cast<uint8_t>(Raw >> 8);
    CHECK(Parsed.Parse(Msg));
    if (Raw >= 0xfffd) CHECK(N2kIsNA(Parsed.Course));
    else Near(Parsed.Course, 6.5532);
  }
  for (int Length = 0; Length <= 22; ++Length) {
    if (Length == 21) continue;
    Msg.DataLen = Length;
    Parsed.Course = 42;
    Parsed.Reference = N2khr_true;
    CHECK(!Parsed.Parse(Msg));
    CHECK(Parsed.Course == 42 && Parsed.Reference == N2khr_true);
  }
  Msg.DataLen = 21;
  Msg.PGN = 60928;
  CHECK(!Parsed.Parse(Msg));

  // Published Heading/Track Control capture with NA signed values.
  const uint8_t Capture[] = {
    0, 0x4d, 0x3f, 0xff, 0x7f, 0xcf, 0x81, 0xcf, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0x7f, 0xff, 0x7f, 0xff, 0x7f, 0x14, 0x7f
  };
  Msg.PGN = 127237;
  std::memcpy(Msg.Data, Capture, sizeof(Capture));
  CHECK(Parsed.Parse(Msg));
  CHECK(Parsed.Reference == N2khr_magnetic && Parsed.Direction == 1);
  CHECK(N2kIsNA(Parsed.RudderAngle) && N2kIsNA(Parsed.Rate) && N2kIsNA(Parsed.Radius));
  Near(Parsed.Course, 3.3231);
  Near(Parsed.Heading, 3.2532);
}

int main() {
  TestCommands();
  TestDeviceReports();
  TestAngles();
  TestHeadingControl();
  std::puts("PASS: commands, device reports, angles, heading/track control, malformed payloads");
}
