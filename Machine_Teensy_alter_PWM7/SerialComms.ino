void handleSerialInput() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');  // Read a full line from Serial
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
      saveuserSettingsToEEPROM();
      loadUserSettingsFromEEPROM();
    }
  }
}
