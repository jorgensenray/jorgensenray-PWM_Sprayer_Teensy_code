// I'm not smart enough to figure this out by myself.  ChatGPT is my friend,  Ray Jorgensen  11/7/2024

#include <CytronMotorDriver.h>
#include <PID_v1.h>
#include <math.h>
#include <elapsedMillis.h>
#include <RunningAverage.h>
#include <vector>


struct PinState {
  uint8_t pinNumber;
  bool state;  // true for HIGH, false for LOW
};

std::vector<PinState> pinStates;

#define LOW 0

void addPinState(uint8_t pinNum, bool isPinOn) {
  // Add pin state to the vector
  pinStates.push_back({ pinNum, isPinOn });
}

const uint8_t maxNozzles = 17;           // Maximum number of nozzles plus one extra for safety
uint32_t PwmTimer[maxNozzles] = { 0 };   // Array for each pin's PWM timer
uint32_t OnTime[maxNozzles] = { 0 };     // Array for each pin's on time
bool nozzleState[maxNozzles] = { LOW };  // Track ON/OFF state per nozzle
// Global variable for EvenOdd timing
unsigned long lastToggleTime = 0;

// int16_t eeAddr = -1;  // -1 defaults to no EEPROM saving/loading from machine.h
// void init(int16_t _eeAddr = -1, const uint8_t _eeSize = 33)  // 33 bytes of EEPROM used from machine.h
const int USER_SETTINGS_ADDR = 400;  // Start `UserSettings` EEPROM

#define EEPROM_MAGIC_ADDR 0    // EEPROM address to store the magic number
#define EEPROM_MAGIC_VALUE 42  // Any unique value to indicate EEPROM is initialized

// Configure the motor driver.
CytronMD motor(PWM_DIR, 29, 30);  // PWM = Pin 29 White, DIR = Pin 30 Red,  Ball Valve

// Pin Definitions
//#define N1Pin 4
#define flowSensorPin 0  // 0
#define PressurePin 41   // A17
#define GPS_Standin A16  // Pot


// User Variables
struct UserSettings {
  float GPATarget = 7.0;
  float SprayWidth = 36.0;
  float FlowCalibration = 7.71;
  float PSICalibration = 0.08;
  float DutyCycleAdjustment = 0.10;
  float PressureTarget = 20.0;
  byte numberNozzles = 1;
  float currentDutyCycle = 50.0;
  unsigned int Hz = 4;
  byte LowBallValve = 28;
  byte Ball_Hyd = 1;
  byte WheelAngle = 5;
  double Kp = 1.0;
  double Ki = 0.1;
  double Kd = 0.01;
  char unit[9] = "Imperial";
  uint8_t debugPwmLevel = 0;     // Ensure this is included
  uint8_t PWM_Conventional = 0;  // Ensure these are also included
  uint8_t Stagger = 0;
};

UserSettings userSettings;  // Instantiate the struct to hold settings

// PID
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, userSettings.Kp, userSettings.Ki, userSettings.Kd, DIRECT);

byte currentPin = 0;

// Boost variables
bool boostPhase[10] = { true };  // Track boost mode per nozzle
const int boostTime = 10;        // Time in ms for boost phase
elapsedMillis boostTimer[10];    // Track boost time duration

// Variables for NozzleSpeed
float boomLength = 0;  // Spray width of total boom
float nozzleSpeedMph = 0;
bool TurnComp = false;

// Software switches
bool sprayerOn = false;     // Track the state of the sprayer
byte PWM_Conventional = 0;  // Select PWN or conventional On/Off
byte Stagger = 0;           //Select even/odd staggered firing
byte Start = 0;             // Used in calibrate

// Global variables for startup handling
bool isStartup = true;          // Flag to indicate if the system is in the startup phase
elapsedMillis startupTimer;     // Timer to track the startup delay
float defaultDutyCycle = 50.0;  // Default duty cycle at startup

// Flow variables
volatile int pulseCount = 0;
float flowRate = 0.0;
float GPM = 0.0;
double actualGPA = 7.0;
float newDutyCycle = 50.0;
double actualGPAave = 7.0;
uint8_t numActiveNozzles = 0;
uint8_t debugPwmLevel = 0;

// Pressure Variables
const float minPressure = 0.0;
const float maxPressure = 100.0;
double pressure = 20;
float voltage = 0;

// From AOG
float steerAngle = 4;
double gpsSpeed = 4.0;
bool isPinOn = false;
uint8_t pinNum = 0;

// Variables for timing
uint32_t period = 0;  // Total period of the cycle in milliseconds
double onTime = 50;   // On-time in milliseconds

// Running average objects
RunningAverage pulseAvg(6);  // Running average for sensor value (6-sample buffer)
RunningAverage GPA_Avg(6);   // Running average for voltage (6-sample buffer)

// Timer variables
elapsedMillis Flow_Timer;
elapsedMillis printTimer;
float PrintFrequency = 500;
elapsedMillis read_pressure;
elapsedMillis pwmTimer;             // Timer to control the PWM cycle
elapsedMillis timeSinceLastToggle;  // Automatically increments with elapsed time
elapsedMillis pwmCycleTimer;        // Track time for PWM cycle

// Interrupt Service Routine (ISR) for counting pulses
void pulseCounter() {
  pulseCount++;
}

//       Function name          Desciption
// 0 - debug OFF           - Turns OFF all reporting
// 1 - dutycycleTurncomp   - In a turn individual nozzle reporting
// 2 - setPWMTiming        - Sets the timing of the nozzle on cycle
// 3 - ControlNozzle       - Like it says, nozzle control - not used in a turn
// 4 - Pressure            - System pressure
// 5 - EvenOdd             - Toggle firing of even/odd nozzles
// 6 - Flow                - The heart of the system - flow rate
// 7 - NozzleSpeed         - Individual nozzle speed in a turn
// 8 - PrintDebug          - Over all system reporting
// 9 - PrintAOG            - Report variable passed from AOG
// 10 - Calibrate_PSI_Flow - Calibration function
// 11 - ActualSteerAngle   - extractActualSteerAngle
// 12 - Communication      - Communication between apps