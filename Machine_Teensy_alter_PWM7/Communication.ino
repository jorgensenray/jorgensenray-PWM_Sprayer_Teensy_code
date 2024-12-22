void setupEth_udp() {
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("\r\n\n*** Ethernet was not found. ***");  // maybe using non Ethernet Teensy?
    return;
  }

  // Initialize Ethernet with the MAC address
  Ethernet.begin(mac, myIP, mydnsServer, myGW, myNetmask);

  // Check if the IP address is correctly assigned
  Serial.print("Ethernet connection set with static IP address: ");
  Serial.println(Ethernet.localIP());
}

void setupUDP() {
    // Set up UDP listener for PORT2    7777  -  Sprayer
  if (udp2.listen(PORT2)) {
    Serial.print("Listening on port ");
    Serial.println(PORT2);
    udp2.onPacket([](AsyncUDPPacket& packet) {
      Serial.print("Received packet on port ");
      Serial.println(PORT2);
      handleUDPMessage(packet.data(), packet.length());
    });  // all the brackets and ending ; are necessary!
  } else {
    Serial.print("Failed to start listener on port ");
    Serial.println(PORT2);
  }

if (udpServer.listen(udpListenPort)) {  // 8888   Machine data
    Serial.print("UDP Listening on: ");
    Serial.println(udpListenPort);
    udpServer.onPacket([](AsyncUDPPacket packet) {
      Serial.print("Received packet on port ");
      Serial.println(udpListenPort);
      checkForPGNs(packet);
    });  // all the brackets and ending ; are necessary!
  }
  
  // Set up UDP listener for PORT3   9999  Steer module 253 PGN
  if (udp3.listen(PORT3)) {
    Serial.print("Listening on port ");
    Serial.println(PORT3);
    udp3.onPacket([](AsyncUDPPacket& packet) {
      Serial.print("Received packet on port ");
      Serial.println(PORT3);
      PGN253(packet);
    });  // all the brackets and ending ; are necessary!
  } else {
    Serial.print("Failed to start listener on port ");
    Serial.println(PORT3);
  }
}

void PGN253(AsyncUDPPacket packet) {
  if (packet.data()[3] == 253) {  // Check if this is PGN 253
    Serial.println(" PGN 253 received!");
    if (packet.length() >= 7) {                                                 // Check if length is valid for ActualSteerAngle
      int16_t actualSteerAngle = (packet.data()[5] | (packet.data()[6] << 8));  // Extract ActualSteerAngle
      float steerAngle = actualSteerAngle / 100.0;                              // Convert to the actual angle
      Serial.print(" Actual Steer Angle: ");
      Serial.println(steerAngle);
    }
  }
}


void handleUDPMessage(const uint8_t* data, size_t len) {
  String message = String((char*)data);
  Serial.println("Message received: " + message);  // Log received messages

  if (message.startsWith("REQUEST_USER_SETTINGS")) {
    Serial.println("REQUEST_USER_SETTINGS");
    loadUserSettingsFromEEPROM();
    sendUserSettingsToCSharp();
  } else if (message.startsWith("REQUEST_SENSOR_DATA")) {
    sendSensorData();
  }
  // Handle Start/Stop Sprayer Commands
  else if (message == "START_SPRAYER") {
    if (!sprayerOn) {
      sprayerOn = true;
      Serial.println("Sprayer started.");
    } else {
      Serial.println("Sprayer already running.");
    }
  } else if (message == "STOP_SPRAYER") {
    if (sprayerOn) {
      sprayerOn = false;
      Serial.println("Sprayer stopped.");
    } else {
      Serial.println("Sprayer already stopped.");
    }
  }

  // Process command strings for switches and settings
  if (message.startsWith("SET_SWITCHES")) {
    PWM_Conventional = message.indexOf("PWM_Conventional:1") != -1;
    Stagger = message.indexOf("Stagger:1") != -1;
  } else if (message.startsWith("SET_DEBUG")) {
    debugPwmLevel = message.substring(message.indexOf("debug:") + 6).toInt();
  } else if (message.startsWith("UPDATE_SETTINGS")) {
    // Parse settings and update userSettings struct
    sscanf(message.c_str(), "UPDATE_SETTINGS:GPATarget:%f,SprayWidth:%f,FlowCalibration:%f,"
                            "PSICalibration:%f,DutyCycleAdjustment:%f,PressureTarget:%f,numberNozzles:%hhu,currentDutyCycle:%f,"
                            "Hz:%u,LowBallValve:%hhu,Ball_Hyd:%hhu,WheelAngle:%hhu,Kp:%lf,Ki:%lf,Kd:%lf",
           &userSettings.GPATarget, &userSettings.SprayWidth,
           &userSettings.FlowCalibration, &userSettings.PSICalibration, &userSettings.DutyCycleAdjustment,
           &userSettings.PressureTarget, &userSettings.numberNozzles, &userSettings.currentDutyCycle,
           &userSettings.Hz, &userSettings.LowBallValve, &userSettings.Ball_Hyd,
           &userSettings.WheelAngle, &userSettings.Kp, &userSettings.Ki, &userSettings.Kd);

    //verify();
    //PrintUserVariables();
    saveuserSettingsToEEPROM();
    loadUserSettingsFromEEPROM();
  }
}


void sendSensorData() {
  char buffer[128];  // Ensure buffer is large enough for the formatted message

  int len = snprintf(buffer, sizeof(buffer),
                     "SENSOR_DATA:pressure:%.2f,onTime:%.2f,actualGPAave:%.2f,gpsSpeed:%.2f",
                     pressure, onTime, actualGPAave, gpsSpeed);

  if (len > 0 && len < (int)sizeof(buffer)) {  // Check if formatting was successful and fits within buffer
    if (!udpServer.broadcastTo(buffer, udpSprayerSendPort)) {
      Serial.println("Failed to send sensor data.");  // Log failure
    } else {
      Serial.println("Sensor data sent successfully.");  // Log success
    }
  } else {
    Serial.println("Error formatting sensor data.");  // Formatting error
  }
}


void sendUserSettingsToCSharp() {
  // Define a buffer to hold the formatted data
  char buffer[256];  // Size of the buffer should accommodate the formatted string

  // Use snprintf to format the string safely
  int len = snprintf(buffer, sizeof(buffer),
                     "USER_SETTINGS:GPATarget:%.2f,SprayWidth:%.2f,FlowCalibration:%.2f,PSICalibration:%.2f,"
                     "DutyCycleAdjustment:%.2f,PressureTarget:%.2f,numberNozzles:%hhu,currentDutyCycle:%.2f,"
                     "Hz:%u,LowBallValve:%hhu,Ball_Hyd:%hhu,WheelAngle:%hhu,Kp:%.2f,Ki:%.2f,Kd:%.2f",
                     userSettings.GPATarget, userSettings.SprayWidth, userSettings.FlowCalibration,
                     userSettings.PSICalibration, userSettings.DutyCycleAdjustment, userSettings.PressureTarget,
                     userSettings.numberNozzles, userSettings.currentDutyCycle, userSettings.Hz,
                     userSettings.LowBallValve, userSettings.Ball_Hyd, userSettings.WheelAngle,
                     userSettings.Kp, userSettings.Ki, userSettings.Kd);

  // Check if the formatted string fits in the buffer
  if (len < 0 || len >= (int)sizeof(buffer)) {
    Serial.println("Error: Buffer too small for user settings.");
    return;
  }

  // Send the formatted data over UDP
  if (!udpServer.broadcastTo(buffer, udpSprayerSendPort)) {
    Serial.println("Failed to send user settings.");  // Log failure
  } else {
    Serial.println("User settings sent successfully.");  // Log success
    PrintUserVariables();
  }
}

// callback function for Machine class to send data back to AgIO/AOG
void pgnReplies(const uint8_t* pgnData, uint8_t len, IPAddress destIP) {
  udpServer.writeTo(pgnData, len, destIP, udpSendPort);
}