// 1. Define your ESP32-S3 CAN transceiver pins BEFORE including NMEA2000_CAN.h
#define ESP32_CAN_TX_PIN GPIO_NUM_20
#define ESP32_CAN_RX_PIN GPIO_NUM_19

#include <Arduino.h>
#include <NMEA2000_CAN.h>       // Automatically selects ESP32 TWAI driver
#include "N2k_NavicoAP.h"       // Your cleaned-up Navico PGN library
#include "NavicoAPController.h" // The controller wrapper class

// Create the autopilot controller instance bound to the global NMEA2000 object
NavicoAPController apController(NMEA2000);

// Non-blocking 60-second timer
const uint32_t TEST_INTERVAL_MS = 60000UL;
uint32_t lastCommandMs = 0;

// Forward declaration of the NMEA 2000 receive callback
void OnN2kMessage(const tN2kMsg &N2kMsg);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting ESP32-S3 Navico AP Controller...");

  // 2. Configure standard NMEA 2000 device identity
  NMEA2000.SetProductInformation(
    "00000001",               // Serial code
    100,                      // Product code
    "Autopilot Annoyer",      // Model ID
    "1.0.0 (2026)",           // Software version
    "1.0.0"                   // Model version
  );

  NMEA2000.SetDeviceInformation(
    123456, // Unique 21-bit serial number
    140,    // Device function: Autopilot
    40,     // Device class: Steering and Control Surfaces
    2046    // Manufacturer code (2046 = generic / non-registered)
  );

  // 3. Operate as a full bus node (listens for 65305/65341/130851/130856 and sends 130850)
  NMEA2000.SetMode(tN2kMode::N2km_ListenAndNode, 25);
  NMEA2000.SetForwardOwnMessages(false);

  // 4. Register proprietary single-frame & fast-packet PGNs BEFORE NMEA2000.Open()
  apController.RegisterPGNs();

  // 5. Attach message handler and open the CAN bus
  NMEA2000.SetMsgHandler(OnN2kMessage);
  NMEA2000.Open();

  lastCommandMs = millis();
}

void loop() {
  // Continuously process incoming CAN frames and handle ISO address claiming
  NMEA2000.ParseMessages();

  // Update pilot online/offline timeout tracking
  apController.Update();

  // Every 60 seconds, send Standby (disengage / stop) to the TP22
  if (millis() - lastCommandMs >= TEST_INTERVAL_MS) {
    lastCommandMs = millis();

    const auto &state = apController.GetState();

    if (state.pilotOnline) {
      Serial.printf("60s elapsed -> Sending STANDBY to TP22 at address %u...\n",
                    state.pilotAddress);

      if (apController.ExecuteAction(APFobAction::Standby)) {
        Serial.println("STANDBY command sent successfully.");
      } else {
        Serial.println("Failed to queue STANDBY message on CAN bus.");
      }
    } else {
      Serial.println("60s elapsed -> No Simrad/Navico autopilot detected on bus yet.");
    }
  }
}

void OnN2kMessage(const tN2kMsg &N2kMsg) {
  // Pass every incoming message to the autopilot controller
  apController.HandleN2kMsg(N2kMsg);

  // Optional: Print alarm text to Serial if PGN 130856 arrives
  if (N2kMsg.PGN == 130856L) {
    Serial.printf("TP22 Alarm: %s\n", apController.GetState().lastAlarm);
  }
}
