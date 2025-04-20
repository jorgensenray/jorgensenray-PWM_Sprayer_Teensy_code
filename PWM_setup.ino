// I'm not smart enough to figure this out by myself.  ChatGPT is my friend,  Ray Jorgensen  11/7/2024

void PWM_setup() {

  pinMode(flowSensorPin, INPUT_PULLUP);
  pinMode(29, OUTPUT);  // PWM pin
  pinMode(30, OUTPUT);  // DIR pin

  pinMode(PressurePin, INPUT);

  Serial.println("🔍 Checking EEPROM...");

  // Check if EEPROM is initialized
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
    Serial.println("⚠️ EEPROM not initialized. Saving defaults...");
    saveuserSettingsToEEPROM();    // Save default values
    loadUserSettingsFromEEPROM();  // Load default values
  } else {
    loadUserSettingsFromEEPROM();  // Load existing values
  }

  //clearAndResetEEPROM(); // Reset EEPROM on startup (remove after testing)

  // Initialize the PID controller with the loaded settings
  myPID.SetTunings(userSettings.Kp, userSettings.Ki, userSettings.Kd);
  Setpoint = userSettings.PressureTarget;
  myPID.SetMode(AUTOMATIC);  // Set PID to automatic mode if required
  myPID.SetOutputLimits(-255, 255);

  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);

  pulseAvg.clear();
  GPA_Avg.clear();

  // Set the default duty cycle at startup
  userSettings.currentDutyCycle = defaultDutyCycle;
}