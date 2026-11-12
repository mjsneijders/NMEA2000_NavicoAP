#ifndef N2K_NAVICO_AP_H
#define N2K_NAVICO_AP_H

#include "N2kMessages.h"
#include <stdint.h>

enum tN2kSimnetAPEvent : uint8_t {
  N2kSimnetEventFollowNonFollow = 0x02,
  N2kSimnetEventStandby = 0x06,
  N2kSimnetEventHeading = 0x09,
  N2kSimnetEventNav = 0x0a,
  N2kSimnetEventNoDrift = 0x0c,
  N2kSimnetEventNonFollowUp = 0x0d,
  N2kSimnetEventFollowUp = 0x0e,
  N2kSimnetEventWind = 0x0f,
  N2kSimnetEventTack = 0x11,
  N2kSimnetEventSquareTurn = 0x12,
  N2kSimnetEventCTurn = 0x13,
  N2kSimnetEventUTurn = 0x14,
  N2kSimnetEventSpiralTurn = 0x15,
  N2kSimnetEventZigZagTurn = 0x16,
  N2kSimnetEventLazySTurn = 0x17,
  N2kSimnetEventDepthTurn = 0x18,
  N2kSimnetEventChangeCourse = 0x1a,
  N2kSimnetEventTimerSync = 0x3d,
  N2kSimnetEventMOBActivated = 0x6b,
  N2kSimnetEventMOBDeactivated = 0x6c,
  N2kSimnetEventPingPortEnd = 0x70,
  N2kSimnetEventPingStarboardEnd = 0x71
};

enum tN2kSimnetCourseDirection : uint8_t {
  N2kSimnetCoursePort = 0x02,
  N2kSimnetCourseStarboard = 0x03,
  N2kSimnetCourseDirectionUnknown = 0xff
};

enum tN2kSimnetDeviceModel : uint8_t {
  N2kSimnetModelAC = 0x00,
  N2kSimnetModelNAC = 0x64
};

enum tN2kSimnetAPStatus : uint8_t {
  N2kSimnetStatusManual = 0x02,
  N2kSimnetStatusAutomatic = 0x10
};

// Independent flags; combine with bitwise OR.
enum tN2kSimnetPilotMode : uint16_t {
  N2kSimnetPilotStandby = 1u << 3,
  N2kSimnetPilotHeading = 1u << 4,
  N2kSimnetPilotNav = 1u << 6,
  N2kSimnetPilotNoDrift = 1u << 8,
  N2kSimnetPilotWind = 1u << 10
};

enum tN2kSimnetAngleMode : uint8_t {
  N2kSimnetAngleHeading = 0x02,
  N2kSimnetAngleWind = 0x03,
  N2kSimnetAngleNav = 0x0a,
  N2kSimnetAngleNoDrift = 0x0b
};

// PGN 130850: 11 bytes for mode commands, 12 for ChangeCourse.
// Address is the pilot's address in the payload; CAN destination is broadcast.
// CourseChangeAngle is an unsigned magnitude in radians, not degrees.
void SetN2k130850(tN2kMsg &N2kMsg, uint8_t Address, tN2kSimnetAPEvent Event,
                 tN2kSimnetCourseDirection Direction = N2kSimnetCourseDirectionUnknown,
                 double CourseChangeAngle = N2kDoubleNA, uint8_t NetworkGroup = 0xff);

// For non-course events, Direction and CourseChangeAngle are unknown/NA.
bool ParseN2k130850(const tN2kMsg &N2kMsg, uint8_t &Address, tN2kSimnetAPEvent &Event,
                   tN2kSimnetCourseDirection &Direction, double &CourseChangeAngle,
                   uint8_t &NetworkGroup);

inline void SetN2kSimnetAPCommand(tN2kMsg &N2kMsg, uint8_t Address, tN2kSimnetAPEvent Event,
                                tN2kSimnetCourseDirection Direction = N2kSimnetCourseDirectionUnknown,
                                double CourseChangeAngle = N2kDoubleNA, uint8_t NetworkGroup = 0xff) {
  SetN2k130850(N2kMsg, Address, Event, Direction, CourseChangeAngle, NetworkGroup);
}

inline bool ParseN2kSimnetAPCommand(const tN2kMsg &N2kMsg, uint8_t &Address,
                                  tN2kSimnetAPEvent &Event, tN2kSimnetCourseDirection &Direction,
                                  double &CourseChangeAngle, uint8_t &NetworkGroup) {
  return ParseN2k130850(N2kMsg, Address, Event, Direction, CourseChangeAngle, NetworkGroup);
}

// PGN 65305: report 0x0a (mode flags) and report 0x02 (status).
// Overloads select the report by the type of the mode/status argument.
void SetN2k65305(tN2kMsg &N2kMsg, tN2kSimnetDeviceModel Model, uint16_t Mode);
void SetN2k65305(tN2kMsg &N2kMsg, tN2kSimnetDeviceModel Model, tN2kSimnetAPStatus Status);
bool ParseN2k65305(const tN2kMsg &N2kMsg, tN2kSimnetDeviceModel &Model, uint16_t &Mode);
bool ParseN2k65305(const tN2kMsg &N2kMsg, tN2kSimnetDeviceModel &Model, tN2kSimnetAPStatus &Status);

inline void SetN2kSimnetPilotMode(tN2kMsg &N2kMsg, tN2kSimnetDeviceModel Model, uint16_t Mode) {
  SetN2k65305(N2kMsg, Model, Mode);
}

inline bool ParseN2kSimnetPilotMode(const tN2kMsg &N2kMsg, tN2kSimnetDeviceModel &Model, uint16_t &Mode) {
  return ParseN2k65305(N2kMsg, Model, Mode);
}

inline void SetN2kSimnetDeviceStatus(tN2kMsg &N2kMsg, tN2kSimnetDeviceModel Model, tN2kSimnetAPStatus Status) {
  SetN2k65305(N2kMsg, Model, Status);
}

inline bool ParseN2kSimnetDeviceStatus(const tN2kMsg &N2kMsg, tN2kSimnetDeviceModel &Model,
                                     tN2kSimnetAPStatus &Status) {
  return ParseN2k65305(N2kMsg, Model, Status);
}

// PGN 65341: Angle is unsigned radians, including wind angles above pi.
// Raw angles 0xfffd..0xffff parse as N2kDoubleNA.
void SetN2k65341(tN2kMsg &N2kMsg, tN2kSimnetAngleMode Mode, double Angle = N2kDoubleNA);
bool ParseN2k65341(const tN2kMsg &N2kMsg, tN2kSimnetAngleMode &Mode, double &Angle);

inline void SetN2kSimnetAutopilotAngle(tN2kMsg &N2kMsg, tN2kSimnetAngleMode Mode,
                                     double Angle = N2kDoubleNA) {
  SetN2k65341(N2kMsg, Mode, Angle);
}

inline bool ParseN2kSimnetAutopilotAngle(const tN2kMsg &N2kMsg, tN2kSimnetAngleMode &Mode, double &Angle) {
  return ParseN2k65341(N2kMsg, Mode, Angle);
}

// PGN 127237: use the library's field types and encoder/decoder.
// Angles are radians, distances metres, rate of turn radians/second.
// The library also provides Set/ParseN2kHeadingTrackControl descriptive aliases.
void SetN2k127237(tN2kMsg &N2kMsg,
                 tN2kOnOff RudderLimitExceeded, tN2kOnOff OffHeadingLimitExceeded,
                 tN2kOnOff OffTrackLimitExceeded, tN2kOnOff Override,
                 tN2kSteeringMode SteeringMode, tN2kTurnMode TurnMode,
                 tN2kHeadingReference HeadingReference,
                 tN2kRudderDirectionOrder CommandedRudderDirection,
                 double CommandedRudderAngle, double HeadingToSteerCourse, double Track,
                 double RudderLimit, double OffHeadingLimit, double RadiusOfTurnOrder,
                 double RateOfTurnOrder, double OffTrackLimit, double VesselHeading);

bool ParseN2k127237(const tN2kMsg &N2kMsg,
                   tN2kOnOff &RudderLimitExceeded, tN2kOnOff &OffHeadingLimitExceeded,
                   tN2kOnOff &OffTrackLimitExceeded, tN2kOnOff &Override,
                   tN2kSteeringMode &SteeringMode, tN2kTurnMode &TurnMode,
                   tN2kHeadingReference &HeadingReference,
                   tN2kRudderDirectionOrder &CommandedRudderDirection,
                   double &CommandedRudderAngle, double &HeadingToSteerCourse, double &Track,
                   double &RudderLimit, double &OffHeadingLimit, double &RadiusOfTurnOrder,
                   double &RateOfTurnOrder, double &OffTrackLimit, double &VesselHeading);

#endif
