#pragma once
#include <Arduino.h>
#include <NMEA2000.h>
#include "N2k_NavicoAP.h"

// Actions assignable to Zigbee fob button events in your S3 GUI
enum class APFobAction : uint8_t {
  None = 0,
  Standby,
  AutoHeading,
  WindMode,
  NavMode,
  NoDriftMode,
  Port1,
  Starboard1,
  Port10,
  Starboard10,
  TackPort,
  TackStarboard
};

class NavicoAPController {
public:
  struct State {
    bool pilotOnline = false;
    uint8_t pilotAddress = 255;
    uint16_t modeFlags = N2kSimnetPilotStandby;
    tN2kSimnetAPStatus status = N2kSimnetStatusManual;
    tN2kSimnetAngleMode angleMode = N2kSimnetAngleHeading;
    double targetAngleDeg = NAN;  // NAN when in Standby / unavailable
    char lastAlarm[48] = {0};
    uint32_t lastSeenMs = 0;
  };

  explicit NavicoAPController(tNMEA2000 &n2k) : _n2k(n2k) {}

  // Call once before NMEA2000.Open() (merge with your existing PGN arrays if needed)
  void RegisterPGNs() {
    static const unsigned long txPGNs[] PROGMEM = {130850L, 0};
    static const unsigned long rxPGNs[] PROGMEM = {65305L, 65341L, 130851L, 130856L, 0};
    static const unsigned long singleFramePGNs[] PROGMEM = {65305L, 65341L, 0};
    static const unsigned long fastPacketPGNs[] PROGMEM = {130851L, 130856L, 0};

    _n2k.ExtendSingleFrameMessages(singleFramePGNs);
    _n2k.ExtendFastPacketMessages(fastPacketPGNs);
    _n2k.ExtendTransmitMessages(txPGNs);
    _n2k.ExtendReceiveMessages(rxPGNs);
  }

  // Call inside your existing NMEA2000 message receive callback
  void HandleN2kMsg(const tN2kMsg &msg) {
    tN2kSimnetDeviceModel model;
    uint16_t modes;
    tN2kSimnetAPStatus status;

    // 1. Pilot Mode Heartbeat (65305 report 0x0A) -> discovers TP22 Source Address
    if (ParseN2kSimnetPilotMode(msg, model, modes)) {
      _state.pilotAddress = msg.Source;
      _state.pilotOnline = true;
      _state.lastSeenMs = millis();
      _state.modeFlags = modes;
      return;
    }

    // 2. Device Status Heartbeat (65305 report 0x02)
    if (ParseN2kSimnetDeviceStatus(msg, model, status)) {
      _state.pilotAddress = msg.Source;
      _state.pilotOnline = true;
      _state.lastSeenMs = millis();
      _state.status = status;
      return;
    }

    // 3. Active Target Heading / Wind Angle (65341)
    tN2kSimnetAngleMode aMode;
    double angleRad;
    if (ParseN2kSimnetAutopilotAngle(msg, aMode, angleRad)) {
      _state.angleMode = aMode;
      _state.targetAngleDeg = N2kIsNA(angleRad) ? NAN : (angleRad * 180.0 / M_PI);
      return;
    }

    // 4. Command Reply (130851)
    uint8_t targetAddr, replyStatus, netGroup;
    tN2kSimnetAPEvent event;
    if (ParseN2kSimnetAPCommandReply(msg, targetAddr, event, replyStatus, netGroup)) {
      // Optional: log or trigger a subtle UI confirmation
      return;
    }

    // 5. SimNet Alarm Text (130856) -> surface on your 7" GUI
    uint8_t alarmId, alarmState;
    uint16_t alarmCode;
    if (ParseN2kSimnetAlarm(msg, alarmId, alarmState, alarmCode,
                            _state.lastAlarm, sizeof(_state.lastAlarm))) {
      return;
    }
  }

  // Call periodically in loop() to mark pilot offline if heartbeats stop (> 5s)
  void Update() {
    if (_state.pilotOnline && (millis() - _state.lastSeenMs > 5000UL)) {
      _state.pilotOnline = false;
      _state.pilotAddress = 255;
      _state.targetAngleDeg = NAN;
    }
  }

  // Call when ESP32-C6 reports a mapped Zigbee fob button event
  bool ExecuteAction(APFobAction action) {
    if (!_state.pilotOnline || _state.pilotAddress == 255 || action == APFobAction::None) {
      return false;
    }

    tN2kMsg msg;
    const double degToRad = M_PI / 180.0;

    switch (action) {
      case APFobAction::Standby:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventStandby);
        break;
      case APFobAction::AutoHeading:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventHeading);
        break;
      case APFobAction::WindMode:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventWind);
        break;
      case APFobAction::NavMode:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventNav);
        break;
      case APFobAction::NoDriftMode:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventNoDrift);
        break;
      case APFobAction::Port1:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventChangeCourse,
                              N2kSimnetCoursePort, 1.0 * degToRad);
        break;
      case APFobAction::Starboard1:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventChangeCourse,
                              N2kSimnetCourseStarboard, 1.0 * degToRad);
        break;
      case APFobAction::Port10:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventChangeCourse,
                              N2kSimnetCoursePort, 10.0 * degToRad);
        break;
      case APFobAction::Starboard10:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventChangeCourse,
                              N2kSimnetCourseStarboard, 10.0 * degToRad);
        break;
      case APFobAction::TackPort:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventChangeCourse,
                              N2kSimnetCoursePort, 100.0 * degToRad);
        break;
      case APFobAction::TackStarboard:
        SetN2kSimnetAPCommand(msg, _state.pilotAddress, N2kSimnetEventChangeCourse,
                              N2kSimnetCourseStarboard, 100.0 * degToRad);
        break;
      default:
        return false;
    }

    return _n2k.SendMsg(msg);
  }

  const State &GetState() const { return _state; }

private:
  tNMEA2000 &_n2k;
  State _state;
};