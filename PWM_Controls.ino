// I'm not smart enough to figure this out by myself.  ChatGPT is my friend.  Ray Jorgensen  11/7/2024

void PWM_Controls() {
  if (numActiveNozzles >= 1) {

    PrintAOGstuff();
    //Flow();
    //Pressure();

    // Recalculate period based on the current frequency.
    period = static_cast<uint32_t>(1000.0 / userSettings.Hz);
    unsigned long cycleTime = millis() % period;
    bool globalEvenNozzlesActive = (cycleTime < (period / 2));

    for (size_t i = 0; i < pinStates.size(); i++) {

      isPinOn = pinStates[i].state;
      boomLength = (numActiveNozzles * userSettings.SprayWidth);
      uint8_t currentPin = i + 1;  // Physical pin number (pins start at 1)
      pinNum = currentPin;         // Update global pinNum for later use

      if (fabs(steerAngle) > userSettings.WheelAngle) {
        TurnComp = true;
      } else {
        TurnComp = false;
      }

      if (isPinOn == 1) {

        // Conventional PWM (no stagger): just set timing normally.
        if ((userSettings.PWM_Conventional == 1) && (i < numActiveNozzles) && (userSettings.Stagger == 0) && (TurnComp == false)) {
          if (printTimer == 5000) {
            //Serial.println(" Conventional PWM (no stagger): just set timing normally. ");
            printTimer = 0;
          }
          setPWMTiming(userSettings.Hz, newDutyCycle, i);
          ControlNozzles(i, currentPin, cycleTime);
        }

        // Even/Odd (stagger) mode.
        if ((userSettings.PWM_Conventional == 1) && (i < numActiveNozzles) && (userSettings.Stagger == 1) && (TurnComp == false)) {
          if (printTimer == 5000) {
            //Serial.println(" Even/Odd (stagger) mode. ");
            printTimer = 0;
          }
          // Pass the global toggle state to EvenOdd.
          EvenOdd(i, currentPin, globalEvenNozzlesActive);
          ControlNozzles(i, currentPin, cycleTime);
        }

        // Even/Odd (stagger) with turn compensation.
        if ((userSettings.PWM_Conventional == 1) && (i < numActiveNozzles) && (userSettings.Stagger == 1) && (TurnComp == true)) {
          if (printTimer == 5000) {
            //Serial.println(" Even/Odd (stagger) with turn compensation. ");
            printTimer = 0;
          }
          EvenOdd(i, currentPin, globalEvenNozzlesActive);
          // Only apply turn compensation if the nozzle is active (nonzero duty cycle)
          if (OnTime[i] > 0) {
            NozzleSpeed(i, currentPin);
          }
          ControlNozzles(i, currentPin, cycleTime);
        }

        // Conventional with turn compensation (even/odd off).
        if ((userSettings.PWM_Conventional == 1) && (i < numActiveNozzles) && (userSettings.Stagger == 0) && (TurnComp == true)) {
          if (printTimer == 5000) {
            //Serial.println(" Conventional with turn compensation (even/odd off). ");
            printTimer = 0;
          }
          NozzleSpeed(i, currentPin);
          ControlNozzles(i, currentPin, cycleTime);
        }

        // Conventional on/off (non-PWM)
        if (userSettings.PWM_Conventional == 0) {
          if (printTimer == 5000) {
            //Serial.println(" Conventional on/off (non-PWM) ");
            printTimer = 0;
          }
          digitalWrite(currentPin, HIGH);
        }
      }

      // isPinOn is false
      else {
        if (printTimer == 5000) {
          Serial.println(" isPinOn is false ");
          printTimer = 0;
        }
        digitalWrite(pinNum, LOW);
      }
    }
  }
}


void Flow() {

  if (Flow_Timer > 500) {
    //Serial.println("in Flow ");
    detachInterrupt(digitalPinToInterrupt(flowSensorPin));      // Suspend interrupt during calculations
    pulseAvg.addValue(pulseCount);                              // Add sensorvalue to the running average
    float pulseCountAve = pulseAvg.getAverage();                // Retrieve the averaged sensorvalue
    flowRate = (pulseCountAve / userSettings.FlowCalibration);  // Flowrate is LPM

    GPM = flowRate / 3.785;                              // Convert to GPM
    actualGPA = (GPM * 5940) / (gpsSpeed * boomLength);  // Calculate GPA
    GPA_Avg.addValue(actualGPA);                         // Add to the running average
    actualGPAave = GPA_Avg.getAverage();                 // Retrieve the averaged value

    pulseCount = 0;  // Reset the pulse counter to get ready for next cycle
    Flow_Timer = 0;  // Reset for next cycle

    attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);

    // Avoid using invalid data during startup
    if (!isStartup && gpsSpeed > 0 && actualGPA > 0) {
      float GPAError = userSettings.GPATarget - actualGPA;
      userSettings.currentDutyCycle += GPAError * 0.1;                                   // factor (0.1) for smoother control
      userSettings.currentDutyCycle = constrain(userSettings.currentDutyCycle, 0, 100);  // Constrain cycle between 0 and 100
    }

    // Calculate the new duty cycle based on actual GPA vs target GPA
    if (gpsSpeed > 0) {                              // Ensure we're moving to calculate duty cycle
      newDutyCycle = userSettings.currentDutyCycle;  // Start with the current duty cycle

      if (actualGPAave < userSettings.GPATarget) {                                                           // actual lower than target, increase
        newDutyCycle += (userSettings.GPATarget - actualGPAave) * userSettings.DutyCycleAdjustment;          // Adjust factor as needed
      } else if (actualGPAave > userSettings.GPATarget) {                                                    // actual higher than target, decrease
        newDutyCycle -= ((actualGPAave - userSettings.GPATarget) * (1 + userSettings.DutyCycleAdjustment));  // Adjust factor
      }
      newDutyCycle = constrain(newDutyCycle, 0.0, 100.0);  //Limit duty cycle to a valid range (0 to 100%)
    }
  }

  if (debugPwmLevel == 6) {
    if (printTimer > PrintFrequency) {
      Serial.print(" ✅ Hz: ");
      Serial.print(userSettings.Hz);
      Serial.print(" ✅ pressure: ");
      Serial.print(pressure);
      Serial.print(" ✅ gpsSpeed: ");
      Serial.print(gpsSpeed);
      Serial.print(" GPM: ");
      Serial.print(GPM);
      Serial.print(" actualGPAave: ");
      Serial.print(actualGPAave);
      Serial.print(" newDutyCycle: ");
      Serial.println(newDutyCycle);
      printTimer = 0;
    }
  }
}

//-------------------------------------------------------------------
// setPWMTiming: Calculate OnTime per nozzle based on duty cycle
//-------------------------------------------------------------------
void setPWMTiming(unsigned int freq, float dutyCycle, size_t nozzleIndex) {
  freq = userSettings.Hz;
  period = static_cast<uint32_t>(1000.0 / freq);
  userSettings.currentDutyCycle = dutyCycle;  // Use the passed dutyCycle

  if (nozzleIndex < numActiveNozzles) {
    OnTime[nozzleIndex] = static_cast<uint32_t>((period * dutyCycle) / 100.0);
  }

  if (debugPwmLevel == 2) {
    if (printTimer > PrintFrequency) {
      Serial.print("newDutyCycle (from parameter) ");
      Serial.print(dutyCycle);
      Serial.print(" OnTime for nozzle ");
      Serial.print(nozzleIndex);
      Serial.print(" = ");
      Serial.println(OnTime[nozzleIndex]);
      Serial.print(" userSettings.Hz ");
      Serial.println(userSettings.Hz);
      printTimer = 0;
    }
  }
}


void EvenOdd(size_t nozzleIndex, uint8_t currentPin, bool globalEvenNozzlesActive) {
  bool isEven = (currentPin % 2 == 0);

  // Debug prints showing the decision for this nozzle.
  if (debugPwmLevel == 5) {
    Serial.print("EvenOdd Debug - Nozzle: ");
    Serial.print(nozzleIndex);
    Serial.print(" | Pin: ");
    Serial.print(currentPin);
    Serial.print(" | isEven: ");
    Serial.print(isEven ? "true" : "false");
    Serial.print(" | globalEvenNozzlesActive: ");
    Serial.print(globalEvenNozzlesActive ? "true" : "false");
  }

  // Apply full duty cycle to one group and 0 to the other.
  if (globalEvenNozzlesActive && isEven) {
    if (debugPwmLevel == 5) {
      Serial.println(" => Even branch: setting full duty cycle");
    }
    setPWMTiming(userSettings.Hz, newDutyCycle, nozzleIndex);
  } else if (!globalEvenNozzlesActive && !isEven) {
    if (debugPwmLevel == 5) {
      Serial.println(" => Odd branch: setting full duty cycle");
    }
    setPWMTiming(userSettings.Hz, newDutyCycle, nozzleIndex);
  } else {
    if (debugPwmLevel == 5) {
      Serial.println(" => Inactive branch: setting 0 duty cycle");
    }
    setPWMTiming(userSettings.Hz, 0, nozzleIndex);
  }
}


void Pressure() {
  //Serial.println("in pressure ");
  if (read_pressure >= 100) {                     // Take readings every 100 milliseconds
    float sensorvalue = analogRead(PressurePin);  // Read the pressure pin
    voltage = sensorvalue * (5.0 / 1023.0);       // Convert the averaged sensor value to voltage
    voltage -= userSettings.PSICalibration;       // Apply an adjustment factor (for calibration)
    pressure = ((voltage - 0.5) * (maxPressure - minPressure)) / (4.5 - 0.5) + minPressure;
    read_pressure = 0;  // Reset the timer for the next reading

    if (userSettings.Ball_Hyd == 1) {
      Output = userSettings.PressureTarget - pressure;
      if ((Output <= userSettings.LowBallValve) && (Output > 2)) {
        Output = userSettings.LowBallValve;
      }
      if ((Output >= -userSettings.LowBallValve) && (Output < -2)) {
        Output = -userSettings.LowBallValve;
      }
      if ((Output <= 1) && (Output >= -1)) {
        (Output = 0);
      }
    } else {
      Input = pressure;
      myPID.Compute();
    }

    motor.setSpeed(Output);

    if (debugPwmLevel == 4) {
      if (printTimer == PrintFrequency) {
        Serial.print("sensorvalue ");
        Serial.print(sensorvalue);
        Serial.print(" voltage ");
        Serial.print(voltage);
        Serial.print(" pressure ");
        Serial.println(pressure);
        printTimer = 0;
      }
    }
  }
}

void ControlNozzles(size_t nozzleIndex, uint8_t currentPin, unsigned long cycleTime) {
  // If we're in even/odd mode, adjust the cycle time for odd pins.
  if (userSettings.Stagger == 1) {
    // Check if the physical pin is odd.
    if (currentPin % 2 != 0) {
      // Add half a period offset so that odd pins are shifted.
      cycleTime = (cycleTime + (period / 2)) % period;
    }
  }

  if (debugPwmLevel == 3) {
    Serial.print("🔍 ControlNozzles - Nozzle: ");
    Serial.print(nozzleIndex);
    Serial.print(" | Pin: ");
    Serial.print(currentPin);
    Serial.print(" | isPinOn: ");
    Serial.print(isPinOn);
    Serial.print(" | OnTime: ");
    Serial.print(OnTime[nozzleIndex]);
    Serial.print(" | Adjusted Cycle Time: ");
    Serial.println(cycleTime);
  }

  // BOOST PHASE HANDLING (unchanged)
  if (boostPhase[nozzleIndex]) {
    if (boostTimer[nozzleIndex] < boostTime) {
      digitalWrite(currentPin, HIGH);
      return;
    } else {
      boostPhase[nozzleIndex] = false;
      boostTimer[nozzleIndex] = 0;
      digitalWrite(currentPin, LOW);
      return;
    }
  }

  // Compare the (possibly adjusted) cycle time to the nozzle's OnTime value.
  if (cycleTime < OnTime[nozzleIndex]) {
    digitalWrite(currentPin, HIGH);
    nozzleState[nozzleIndex] = HIGH;
  } else {
    digitalWrite(currentPin, LOW);
    nozzleState[nozzleIndex] = LOW;
  }
}


void NozzleSpeed(size_t nozzleIndex, uint8_t currentPin) {
  // If EvenOdd already set this nozzle off, skip turn compensation.
  if (OnTime[nozzleIndex] == 0) {
    return;
  }

  float tractorSpeedIps = gpsSpeed * 17.6;                 // Convert tractor speed to inches per second
  float turnRadius = 360 / (2 * M_PI * steerAngle / 360);  // Calculate turn radius
  // Calculate the distance from the boom's center for this nozzle.
  float distanceFromCenter = (currentPin - (boomLength / 2.0 / userSettings.SprayWidth)) * userSettings.SprayWidth;
  float nozzleRadius = turnRadius + distanceFromCenter;
  float nozzleSpeedIps = (nozzleRadius / turnRadius) * tractorSpeedIps;
  float nozzleSpeedMph = nozzleSpeedIps / 17.6;

  if (debugPwmLevel == 7) {
    Serial.print("currentPin ");
    Serial.print(currentPin);
    Serial.print(": Speed = ");
    Serial.print(nozzleSpeedMph);
    Serial.println(" MPH");
    printTimer = 0;
  }

  // Now call dutycycleTurncomp with the actual currentPin.
  dutycycleTurncomp(nozzleIndex, currentPin, nozzleSpeedMph);
}


void dutycycleTurncomp(uint8_t nozzleIndex, uint8_t currentPin, float nozzleSpeedMph) {
  // Calculate the actual GPA for the nozzle.
  actualGPA = (GPM * 5940) / (nozzleSpeedMph * userSettings.SprayWidth);
  // Start with the base duty cycle.
  float nozzleDutyCycle = userSettings.currentDutyCycle;
  if (actualGPA < userSettings.GPATarget) {
    nozzleDutyCycle += (userSettings.GPATarget - actualGPA) * userSettings.DutyCycleAdjustment;
  } else if (actualGPA > userSettings.GPATarget) {
    nozzleDutyCycle -= ((actualGPA - userSettings.GPATarget) * (1 + userSettings.DutyCycleAdjustment));
  }

  // Constrain and update the duty cycle.
  newDutyCycle = constrain(nozzleDutyCycle, 0.0, 100.0);
  OnTime[nozzleIndex] = static_cast<uint32_t>(period * (newDutyCycle / 100.0));

  if (debugPwmLevel == 1) {
    Serial.print("Nozzle ");
    Serial.print(nozzleIndex);
    Serial.print(" (Pin ");
    Serial.print(currentPin);
    Serial.print("): newDutyCycle = ");
    Serial.print(newDutyCycle);
    Serial.print(", OnTime = ");
    Serial.print(OnTime[nozzleIndex]);
    Serial.print(": Speed = ");
    Serial.print(nozzleSpeedMph);
    Serial.println(" MPH");
    printTimer = 0;
  }
}

void PrintDebug() {
  if (debugPwmLevel == 8) {
    if (printTimer == PrintFrequency) {
      Serial.print(" gpsSpeed ");
      Serial.print(gpsSpeed);
      Serial.print(" Actual GPA: ");
      Serial.print(actualGPA);
      Serial.print(" onTime: ");
      Serial.print(onTime);
      Serial.print(" period: ");
      Serial.print(period);
      Serial.print(" Pressure: ");
      Serial.println(pressure);
      printTimer = 0;
    }
  }
}


void PrintAOG() {
  if (debugPwmLevel == 9) {
    if (printTimer == PrintFrequency) {
      Serial.print(" steerAngle ");
      Serial.print(steerAngle);
      Serial.print(" gpsSpeed ");
      Serial.print(gpsSpeed);
      Serial.print(" isPinOn ");
      Serial.print(isPinOn);
      Serial.print(" currentPin ");
      Serial.print(currentPin);
      printTimer = 0;
    }
  }
}

void Calibrate_PSI_Flow() {

  digitalWrite(1, HIGH);

  Pressure();

  if (Flow_Timer > 500) {
    detachInterrupt(digitalPinToInterrupt(flowSensorPin));  // Suspend interrupt during calculations

    // Calculate the average pulse count and flow rate (LPM)
    pulseAvg.addValue(pulseCount);
    float pulseCountAve = pulseAvg.getAverage();
    flowRate = pulseCountAve / userSettings.FlowCalibration;  // Flow rate in LPM

    // Convert to GPM and calculate the actual GPA
    GPM = flowRate / 3.785;  // Convert LPM to GPM
    actualGPA = (GPM * 5940) / (gpsSpeed * (userSettings.SprayWidth));
    GPA_Avg.addValue(actualGPA);
    actualGPAave = GPA_Avg.getAverage();

    // Update FlowCalibration based on the difference between target and actual GPA
    float errorGPA = actualGPAave - userSettings.GPATarget;

    float adjustmentFactor = 0.01;

    if ((errorGPA < 0.02) && (errorGPA > 0.00)) {
      Serial.print("Calibration complete - value = ");
      Serial.print(userSettings.FlowCalibration);
      saveuserSettingsToEEPROM();
      Serial.print(" has been saved to EEPROM. ");
    }

    userSettings.FlowCalibration += adjustmentFactor * errorGPA;

    pulseCount = 0;  // Reset pulse counter
    Flow_Timer = 0;  // Reset timer

    attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);

    // Print debug information
    Serial.print(" Output: ");
    Serial.print(Output);
    Serial.print(" | pressure: ");
    Serial.print(pressure);
    Serial.print(" | FlowCalibration: ");
    Serial.print(userSettings.FlowCalibration, 3);
    Serial.print(" | actualGPAave: ");
    Serial.print(actualGPAave);
    Serial.print(" | GPATarget: ");
    Serial.println(userSettings.GPATarget);
  }
}

void setupNozzles() {
  //if (numActiveNozzles < 1) numActiveNozzles = 1;  // Ensure the number of active nozzles is valid

  for (uint8_t i = 0; i <= numActiveNozzles; i++) {  // Initialize all values to 0
    OnTime[i] = 0;
    PwmTimer[i] = 0;
  }

  //Serial.print("Allocated OnTime & PwmTimer arrays for ");
  //Serial.print(numActiveNozzles);
  //Serial.println(" nozzles.");
}

void pwmtimerstates() {
  static unsigned long lastUpdate = 0;  // Static ensures it retains its value across calls
  for (auto& pinState : pinStates) {
    PwmTimer[pinState.pinNumber] += millis() - lastUpdate;  // Update the timer for the pin
  }
  lastUpdate = millis();  // Update the last timestamp
}

void setDebugPwmLevel(uint8_t level) {
  debugPwmLevel = level;
}

void PrintUserVariables() {
  // Debug: Print all loaded values
  //if (debugPwmLevel == 12) {
  //Serial.println("");
  //Serial.println("");
  Serial.println("GPATarget: " + String(userSettings.GPATarget));
  Serial.println("SprayWidth: " + String(userSettings.SprayWidth));
  Serial.println("FlowCalibration: " + String(userSettings.FlowCalibration));
  Serial.println("PSICalibration: " + String(userSettings.PSICalibration));
  Serial.println("DutyCycleAdjustment: " + String(userSettings.DutyCycleAdjustment));
  Serial.println("PressureTarget: " + String(userSettings.PressureTarget));
  Serial.println("numberNozzles: " + String(userSettings.numberNozzles));
  Serial.println("currentDutyCycle: " + String(userSettings.currentDutyCycle));
  Serial.println("Hz: " + String(userSettings.Hz));
  Serial.println("LowBallValve: " + String(userSettings.LowBallValve));
  Serial.println("Ball_Hyd: " + String(userSettings.Ball_Hyd));
  Serial.println("WheelAngle: " + String(userSettings.WheelAngle));
  Serial.println("Kp: " + String(userSettings.Kp));
  Serial.println("Ki: " + String(userSettings.Ki));
  Serial.println("Kd: " + String(userSettings.Kd));
  Serial.println("Parsed Unit: " + String(userSettings.unit));
  Serial.print("Parsed PWM_Conventional: ");
  Serial.println(userSettings.PWM_Conventional);
  Serial.println("Stagger: " + String(userSettings.Stagger));
  Serial.println("debugPwmLevel: " + String(userSettings.debugPwmLevel));
  Serial.println("");
  //Serial.println("");
  //}
}

void PrintAOGstuff() {
  if (debugPwmLevel == 9) {
    Serial.print(" boomLength = ");
    Serial.print(boomLength);
    Serial.print(" numActiveNozzles = ");
    Serial.print(numActiveNozzles);
    Serial.print(" gpsSpeed = ");
    Serial.print(gpsSpeed);
    Serial.print(" currentPin = ");
    Serial.print(currentPin);
    Serial.print(" isPinOn = ");
    Serial.println(isPinOn);
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}