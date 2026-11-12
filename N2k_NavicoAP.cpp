#include "N2k_NavicoAP.h"

namespace {
const uint16_t SimnetHeader = 1857u | (0x03u << 11) | (4u << 13);
const uint8_t APCommand = 0x0a;
const uint8_t ModeReport = 0x0a;
const uint8_t StatusReport = 0x02;

bool IsSimnetMessage(const tN2kMsg &Msg, unsigned long PGN, int Length) {
  if (Msg.PGN != PGN || Msg.DataLen != Length) return false;
  int Index = 0;
  // Reserved header bits need not be interpreted by receivers.
  return (Msg.Get2ByteUInt(Index) & 0xe7ffu) == (SimnetHeader & 0xe7ffu);
}

double GetSimnetAngle(const tN2kMsg &Msg, int &Index) {
  const uint16_t Raw = Msg.Get2ByteUInt(Index);
  return Raw >= 0xfffd ? N2kDoubleNA : Raw * 0.0001;
}

void SetDeviceReport(tN2kMsg &Msg, tN2kSimnetDeviceModel Model, uint8_t Report) {
  Msg.SetPGN(65305L);
  Msg.Priority = 7;
  Msg.Add2ByteUInt(SimnetHeader);
  Msg.AddByte(Model);
  Msg.AddByte(Report);
}
}

void SetN2k130850(tN2kMsg &N2kMsg, uint8_t Address, tN2kSimnetAPEvent Event,
                 tN2kSimnetCourseDirection Direction, double CourseChangeAngle,
                 uint8_t NetworkGroup) {
  N2kMsg.SetPGN(130850L);
  N2kMsg.Priority = 2;
  N2kMsg.Destination = 255;
  N2kMsg.Add2ByteUInt(SimnetHeader);
  N2kMsg.AddByte(Address);
  N2kMsg.AddByte(0xff);
  N2kMsg.AddByte(NetworkGroup);
  N2kMsg.AddByte(APCommand);
  N2kMsg.AddByte(Event);
  N2kMsg.AddByte(0x00); // NAC-3 ignores commands with 0xff here.
  if (Event == N2kSimnetEventChangeCourse) {
    N2kMsg.AddByte(Direction);
    N2kMsg.Add2ByteUDouble(CourseChangeAngle, 0.0001);
  } else {
    N2kMsg.Add2ByteUInt(0xffff);
  }
  N2kMsg.AddByte(0xff);
}

bool ParseN2k130850(const tN2kMsg &N2kMsg, uint8_t &Address, tN2kSimnetAPEvent &Event,
                   tN2kSimnetCourseDirection &Direction, double &CourseChangeAngle,
                   uint8_t &NetworkGroup) {
  if (!IsSimnetMessage(N2kMsg, 130850L, 11) &&
      !IsSimnetMessage(N2kMsg, 130850L, 12)) return false;
  if (N2kMsg.Data[5] != APCommand) return false;
  const tN2kSimnetAPEvent ParsedEvent = static_cast<tN2kSimnetAPEvent>(N2kMsg.Data[6]);
  if (N2kMsg.DataLen != (ParsedEvent == N2kSimnetEventChangeCourse ? 12 : 11)) return false;

  Address = N2kMsg.Data[2];
  NetworkGroup = N2kMsg.Data[4];
  Event = ParsedEvent;
  Direction = N2kSimnetCourseDirectionUnknown;
  CourseChangeAngle = N2kDoubleNA;
  if (Event == N2kSimnetEventChangeCourse) {
    Direction = static_cast<tN2kSimnetCourseDirection>(N2kMsg.Data[8]);
    int Index = 9;
    CourseChangeAngle = GetSimnetAngle(N2kMsg, Index);
  }
  return true;
}

void SetN2k65305(tN2kMsg &N2kMsg, tN2kSimnetDeviceModel Model, uint16_t Mode) {
  SetDeviceReport(N2kMsg, Model, ModeReport);
  N2kMsg.Add2ByteUInt(Mode);
  N2kMsg.Add2ByteUInt(0xffff);
}

void SetN2k65305(tN2kMsg &N2kMsg, tN2kSimnetDeviceModel Model, tN2kSimnetAPStatus Status) {
  SetDeviceReport(N2kMsg, Model, StatusReport);
  N2kMsg.AddByte(Status);
  N2kMsg.Add2ByteUInt(0xffff);
  N2kMsg.AddByte(0xff);
}

bool ParseN2k65305(const tN2kMsg &N2kMsg, tN2kSimnetDeviceModel &Model, uint16_t &Mode) {
  if (!IsSimnetMessage(N2kMsg, 65305L, 8) || N2kMsg.Data[3] != ModeReport) return false;
  int Index = 4;
  Model = static_cast<tN2kSimnetDeviceModel>(N2kMsg.Data[2]);
  Mode = N2kMsg.Get2ByteUInt(Index);
  return true;
}

bool ParseN2k65305(const tN2kMsg &N2kMsg, tN2kSimnetDeviceModel &Model, tN2kSimnetAPStatus &Status) {
  if (!IsSimnetMessage(N2kMsg, 65305L, 8) || N2kMsg.Data[3] != StatusReport) return false;
  Model = static_cast<tN2kSimnetDeviceModel>(N2kMsg.Data[2]);
  Status = static_cast<tN2kSimnetAPStatus>(N2kMsg.Data[4]);
  return true;
}

void SetN2k65341(tN2kMsg &N2kMsg, tN2kSimnetAngleMode Mode, double Angle) {
  N2kMsg.SetPGN(65341L);
  N2kMsg.Priority = 6;
  N2kMsg.Add2ByteUInt(SimnetHeader);
  N2kMsg.Add2ByteUInt(0xffff);
  N2kMsg.AddByte(Mode);
  N2kMsg.AddByte(0xff);
  N2kMsg.Add2ByteUDouble(Angle, 0.0001);
}

bool ParseN2k65341(const tN2kMsg &N2kMsg, tN2kSimnetAngleMode &Mode, double &Angle) {
  if (!IsSimnetMessage(N2kMsg, 65341L, 8)) return false;
  int Index = 6;
  Mode = static_cast<tN2kSimnetAngleMode>(N2kMsg.Data[4]);
  Angle = GetSimnetAngle(N2kMsg, Index);
  return true;
}

void SetN2k127237(tN2kMsg &N2kMsg,
                 tN2kOnOff RudderLimitExceeded, tN2kOnOff OffHeadingLimitExceeded,
                 tN2kOnOff OffTrackLimitExceeded, tN2kOnOff Override,
                 tN2kSteeringMode SteeringMode, tN2kTurnMode TurnMode,
                 tN2kHeadingReference HeadingReference,
                 tN2kRudderDirectionOrder CommandedRudderDirection,
                 double CommandedRudderAngle, double HeadingToSteerCourse, double Track,
                 double RudderLimit, double OffHeadingLimit, double RadiusOfTurnOrder,
                 double RateOfTurnOrder, double OffTrackLimit, double VesselHeading) {
  SetN2kPGN127237(N2kMsg, RudderLimitExceeded, OffHeadingLimitExceeded, OffTrackLimitExceeded,
                 Override, SteeringMode, TurnMode, HeadingReference, CommandedRudderDirection,
                 CommandedRudderAngle, HeadingToSteerCourse, Track, RudderLimit, OffHeadingLimit,
                 RadiusOfTurnOrder, RateOfTurnOrder, OffTrackLimit, VesselHeading);
}

bool ParseN2k127237(const tN2kMsg &N2kMsg,
                   tN2kOnOff &RudderLimitExceeded, tN2kOnOff &OffHeadingLimitExceeded,
                   tN2kOnOff &OffTrackLimitExceeded, tN2kOnOff &Override,
                   tN2kSteeringMode &SteeringMode, tN2kTurnMode &TurnMode,
                   tN2kHeadingReference &HeadingReference,
                   tN2kRudderDirectionOrder &CommandedRudderDirection,
                   double &CommandedRudderAngle, double &HeadingToSteerCourse, double &Track,
                   double &RudderLimit, double &OffHeadingLimit, double &RadiusOfTurnOrder,
                   double &RateOfTurnOrder, double &OffTrackLimit, double &VesselHeading) {
  if (N2kMsg.PGN != 127237L || N2kMsg.DataLen != 21) return false;
  if (!ParseN2kPGN127237(N2kMsg, RudderLimitExceeded, OffHeadingLimitExceeded, OffTrackLimitExceeded,
                       Override, SteeringMode, TurnMode, HeadingReference, CommandedRudderDirection,
                       CommandedRudderAngle, HeadingToSteerCourse, Track, RudderLimit, OffHeadingLimit,
                       RadiusOfTurnOrder, RateOfTurnOrder, OffTrackLimit, VesselHeading)) return false;
  int Index = 5;
  if (N2kMsg.Get2ByteUInt(Index) >= 0xfffd) HeadingToSteerCourse = N2kDoubleNA;
  return true;
}
