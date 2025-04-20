// callback function triggered by Machine class to update 1-64 "section" outputs only
void updateSectionOutputs() {
  if (debugPwmLevel == 8) {
    Serial.print("\r\n*** Section Outputs update! *** ");
  }
}

// callback function triggered by Machine class to update "machine" outputs
// this updates the Machine Module Pin Configuration outputs
// - sections 1-16, Hyd Up/Down, Tramline Right/Left, Geo Stop
void updateMachineOutputs() {
  //Serial.print("\r\nMachine Outputs update! ");

  updatePinStates();
  //setupNozzles();

  for (uint8_t i = 1; i < numMachineOutputs; i++) {
    /*
    Serial.print("\r\n- Pin ");
    Serial.print((machineOutputPins[i] < 10 ? " " : ""));
    Serial.print(machineOutputPins[i]); Serial.print(": ");
    Serial.print(machine.state.functions[machine.config.pinFunction[i - 1]]);
    Serial.print(" ");
    Serial.print(machine.functionNames[machine.config.pinFunction[i - 1]]);
    */

    //digitalWrite(machineOutputPins[i], machine.state.functions[machine.config.pinFunction[i]] == machine.config.isPinActiveHigh);  // ==, XOR
  }
}

void updatePinStates() {
  pinStates.clear();     // Clear previous states to avoid duplicates
  numActiveNozzles = 0;  // Reset the count of active nozzles

  for (uint8_t i = 1; i <= 16; i++) {  // Loop through possible nozzles
    pinNum = machineOutputPins[i];     // Get the pin number
    isPinOn = static_cast<bool>(machine.state.functions[machine.config.pinFunction[i - 1]]);

    bool found = false;  // Check if pin exists in the list
    for (auto& pinState : pinStates) {
      if (pinState.pinNumber == pinNum) {
        if (pinState.state != isPinOn) {  // If found, update state
          pinState.state = isPinOn;
        }
        if (isPinOn) {  // Count active nozzles
          numActiveNozzles++;
        }
        found = true;
        break;
      }
    }

    if (!found) {  // If not found, add it to the list
      pinStates.push_back({ pinNum, isPinOn });

      if (isPinOn) {  // Count new active nozzle
        numActiveNozzles++;
      }
    }
  }
  //Serial.print(" numActiveNozzles counted in updatePinStates ");
  //Serial.println(numActiveNozzles);
  setupNozzles();  // Ensure memory is adjusted based on active nozzles
}


void setOutputPinModes() {
  if (numMachineOutputs > 0) {
    for (uint8_t i = 0; i < numMachineOutputs; i++) {
      pinMode(machineOutputPins[i], OUTPUT);
      digitalWrite(machineOutputPins[i], LOW);  // set OFF
    }
  }
}
