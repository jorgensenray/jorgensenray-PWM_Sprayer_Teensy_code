/*

    Matt Elias Feb 2024
    Adapted from Machine_UDP_v5 in official AOG repo

*/

#include <EEPROM.h>
#include "AsyncUDP_Teensy41.h"
#include "QNEthernet.h"

using namespace qindesign::network;

IPAddress myIP = { 192, 168, 5, 123 };                             // IP of ESP32 stn/client, default: 192.168.137.79 to match Windows Hotspot scheme
IPAddress myNetmask = { 255, 255, 255, 0 };                        //QNEthernet requires this to be set.
IPAddress myGW = { 192, 168, 5, 1 };                               //QNEthernet requires this to be set.
IPAddress mydnsServer = { 192, 168, 5, 1 };                        //QNEthernet requires this to be set.
byte mac[6] = { 0x0A, 0x0F, myIP[0], myIP[1], myIP[2], myIP[3] };  // create unique MAC from IP as IP should already be unique

AsyncUDP udpServer;  // AOG server
AsyncUDP udp2;       // Sprayer sever
AsyncUDP udp3;       // Steer server

// Ports to listen on
const uint16_t PORT2 = 7777;    // Sprayer
const uint16_t PORT3 = 9999;    // Steer - listen for PGN 253 Steer Angle
uint16_t udpListenPort = 8888;  // listen for AOG PGNs

// Ports to send on
uint16_t udpSendPort = 9999;         // UDP port to send PGN data back to AgIO/AOG
uint16_t udpSprayerSendPort = 7777;  // UDP port to send PGN data Sprayer
/*
AsyncUDP udpServer;
uint16_t udpListenPort = 8888;  // General UDP listener port

uint16_t udpSPRAYERListenPort7777 = 7777;  // Port to listen for Sprayer C# app
uint16_t udpSPRAYERSendPort7777 = 7777;    // Port to send to Sprayer C# app

uint16_t udpAOGListenPort9999 = 9999;  // AOG mirrored listener port - used to recieve 253 PGN where we get (actualSteerAngle)
uint16_t udpSendPort = 9999;
*/

#include "PWMdefinitions.h"
#include "machine.h"
MACHINE machine;
MACHINE::States machineStates;

//const byte numMachineOutputs = 8;
//byte machineOutputPins[numMachineOutputs] = { 12, 13, 5, 23, 19, 18, 21, 22 };

const byte numMachineOutputs = 17;
//byte machineOutputPins[numMachineOutputs] = { 12, 13, 5, 23, 19, 18, 21, 22, 14, 27, 16, 17, 25, 26, 4 };
byte machineOutputPins[numMachineOutputs] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };

void setup() {
  delay(250);
  Serial.begin(115200);

  EEPROM.begin();  // Initialize EEPROM

  setupEth_udp();
  setupUDP();
  PWM_setup();

  machine.init(100);  // 100 is address for machine EEPROM storage (uses 33 bytes)
  machine.setSectionOutputsHandler(updateSectionOutputs);
  machine.setMachineOutputsHandler(updateMachineOutputs);
  machine.setUdpReplyHandler(pgnReplies);

  setOutputPinModes();
  Serial.print("\r\n\nSetup complete\r\n*******************************************\r\n");
}

void loop() {
  //delay(10);
  yield();

  //test = machine.states.gpsSpeed * 0.1;  // Convert to real speed in km/hr if needed

  // Use Serial to print test for debugging if needed
  //Serial.print("GPS Speed in km/hr: ");
  //Serial.println(test);

  machine.watchdogCheck();  // Check communication status continuously
  machine.updateStates();   // Continuously update states

  PWM_loop();  // PWN control of nozzles

  if (Serial.available()) parseSerial();
}

void parseSerial() {
  if (Serial.read() == 'm' && Serial.available()) {
    machine.debugLevel = Serial.read() - '0';
  }
}