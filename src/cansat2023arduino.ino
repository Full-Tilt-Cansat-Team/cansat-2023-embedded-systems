/*
Libraries
*/
#include "Adafruit_BMP3XX.h"
#include <Wire.h>
#include <Servo.h>

/*
Custom data types
*/

// Holds flight state
enum FlightState {
  LowPower,
  PreLaunch,
  Launch,
  Peak,
  Deployment,
  Parachute,
  Landed
};

// Holds deployment states
enum DeploymentState {
  NotDeployed,
  Deployed
};

// Holds computer mode
enum FlightMode {
  Flight,
  Simulation
};

// Holds time information
struct TimeStruct {
  int hours;
  int minutes;
  float seconds;
};

// Holds all information used in telemetry packets
struct TelemetryPacket {
  int teamId;
  int utcHours;
  int packetCount;
  TimeStruct missionTime;
  FlightMode computerMode;
  FlightState flightState;
  float altitude;
  DeploymentState probeState;
  DeploymentState heatShieldState;
  DeploymentState parachuteState;
  DeploymentState mastState;
  float temperature;
  float voltage;
  float pressure;
  TimeStruct gpsTime;
  float gpsAltitude;
  float latitude;
  float longitude;
  int satConnections;
  float tiltX;
  float tiltY;
  String cmdEcho;
};

// Flight-time variables
FlightState flightState; // Holds the determinant for state change logic

TelemetryPacket telemetry; // Holds telemetry packets

TimeStruct currentTime; // Holds global time

float altitude; // Altitude of cansat
float lastAltitude; // Altitude at last logic step
float vertVelocity; // Velocity upwards
float calibrationAltitude; // To find absolute

float temperature; // Temperature (C)
float pressure; // Pressure (hPa)
float voltage; // Voltage (volts)
int adcVol; // voltage (unitless)


Adafruit_BMP3XX bmp; // BMP388 Sensor for pressure/temperature
#define SEALEVELPRESSURE_HPA 1000

unsigned long currentCycleTime; // Used for time update every cycle
unsigned long lastCycleTime; // Used to calculate cycleTimeGap
unsigned long cycleTimeGap; // Time between cycles
unsigned long deltaTime; // How much time has passed between logic steps

bool transmitting = false; // Are we transmitting packets?

static unsigned long PACKET_GAP_TIME = 1000;
static int TEAM_ID = 9999;

// ADC pin for voltage
#define ADC_GPIO_PIN 29

// Release servo
Servo M2;
#define M2_PWM_PIN 20
#define M2_MOS_PIN 26
#define M2_CLOSED 30
#define M2_OPEN 40

// Release servo
Servo M3;
#define M3_PWM_PIN 21
#define M3_MOS_PIN 27
#define M3_CLOSED 0
#define M3_OPEN 40

int packetCount;

void setup() {
  Serial.begin(9600); // Open serial line TODO: Remove this in favor of xbee

  // M4 Setup
  M2.attach(M2_PWM_PIN);
  M2.write(M2_CLOSED);
  pinMode(M2_MOS_PIN, OUTPUT);
  digitalWrite(M2_MOS_PIN, HIGH);

  // M5 setup
  M3.attach(M3_PWM_PIN, 427, 2400);
  M3.write(M3_CLOSED);
  pinMode(M3_MOS_PIN, OUTPUT);
  digitalWrite(M3_MOS_PIN, LOW);

  flightState = LowPower;

  altitude = 0.0;
  lastAltitude = 0.0;
  vertVelocity = 0.0;


  temperature = 0;
  pressure = 0;

  packetCount = 0;

  // TODO: Impliment a command to pull time from gs
  //currentTime = createBlankTime();

  lastCycleTime = millis(); // Prime cycle execution
  deltaTime = 0; // Immediatly start flight logic

  // Setup I2c
  Wire1.setSDA(14);
  Wire1.setSCL(15);
  Wire1.begin();

  // Begin BMP
  if (!bmp.begin_I2C(0x77, &Wire1)) {
    while (true) {
      Serial.println("BMP ERROR");
      delay(1000);
    }
  }
  bmp.performReading();

  calibrationAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
};

void loop() {
  // Update our timeing control
  currentCycleTime = millis();
  cycleTimeGap = currentCycleTime - lastCycleTime;
  deltaTime += cycleTimeGap;

  // If one second has passed, perform flight logic
  if (deltaTime >= PACKET_GAP_TIME) {
    // Calculate current time
    updateTime(currentTime, deltaTime);

    // Read pressure and temperature from BMP
    bmp.performReading();
    temperature = bmp.temperature;
    pressure = bmp.pressure; // hPa
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - calibrationAltitude;
    vertVelocity = (altitude - lastAltitude) / ((float)deltaTime / 1000.0); // calculate velocity

    // Analog
    adcVol = analogRead(ADC_GPIO_PIN);
    voltage = (float)adcVol * 3.3 / 4095;

    // Flight state changes
    {
      if (flightState == LowPower) {
        // Do nothing, and don't do flight logic
      } else if (flightState == PreLaunch) {

        // When 10m passed and velocity greater than 10m/s, move to launch
        if (altitude >= 10.0 && vertVelocity >= 10.0) {
          // Start camera recording (TODO)
          // Start transmitting packets
          transmitting = true;
          // Move into launch state
          flightState = Launch;
        }

      } else if (flightState == Launch) {

        // When above 300m, transition to this stage (present to avoid accidental movment directly into landing stage)
        if (altitude >= 300.0) {
          flightState = Peak;
        }

      } else if (flightState == Peak) {

        // Peak stage transitions to deployment when below 500m and velocity is <0m/s
        if (altitude <= 500.0 && vertVelocity < 0.0) {
          // Deploy probe and heatshield (TODO)
          // Move to deploymment state
          flightState = Deployment;
        }

      } else if (flightState == Deployment) {

        // Move to parachute state when below 200m and negative velocity
        if (altitude < 200.0 && vertVelocity < 0.0) {
          // Deploy parachute (TODO)
          // Move to chute stage
          flightState = Parachute;
        }

      } else if (flightState == Parachute) {

        // Just wait until velocity is less than 2
        if (vertVelocity < 2.0 && vertVelocity > -2.0) {
          // Move to landed state
          flightState = Landed;

          // Execute wacky subroutine (TODO)
        } 

      }
    }

    


    // Fill the fields of the telemetry packet
    telemetry.teamId = TEAM_ID;
    telemetry.missionTime = currentTime;
    telemetry.computerMode = Flight;
    telemetry.flightState = flightState;
    telemetry.altitude = altitude;
    telemetry.heatShieldState = NotDeployed;
    telemetry.parachuteState = NotDeployed;
    telemetry.mastState = NotDeployed;
    telemetry.temperature = temperature; // TODO: Impliment Temperature
    telemetry.voltage = voltage; // TODO: Impliment voltage sensing
    telemetry.gpsTime = createBlankTime(); // TODO: Impliment GPS
    telemetry.gpsAltitude = 0;
    telemetry.latitude = 0;
    telemetry.longitude = 0;
    telemetry.satConnections = 10;
    telemetry.tiltX = 0;
    telemetry.tiltY = 90;

    // Assemble packet
    String packet = assemblePacket(telemetry);

    // Convert to bytes
    // TODO: Convert all strings in here to bytes natively
    uint8_t packetBytes[packet.length()];
    packet.getBytes(packetBytes, packet.length());

    // Send packet TODO: Impliment on XBEE
    Serial.write(packetBytes, packet.length());
    Serial.print("\n");

    // Update timing
    deltaTime = 0;
  }
  
  lastCycleTime = currentCycleTime;
};

// Creates a string from a time structure
String formatTime(TimeStruct tStruct) {
  String out = "";
  if (tStruct.hours < 10) {
    out += "0";
  }
  out += String(tStruct.hours);
  out += ":";

  if (tStruct.minutes < 10) {
    out += "0";
  }
  out += String(tStruct.minutes);
  out += ":";

  if (tStruct.seconds < 10) {
    out += "0";
  }
  out += String(tStruct.seconds, 2);

  return out;
};

// Function to assemble a telemetry packet
String assemblePacket(TelemetryPacket telemetry) {
  String packet = "";
  packet += telemetry.teamId;
  packet += ",";

  packet += formatTime(telemetry.missionTime);
  packet += ",";

  packet += String(telemetry.packetCount);
  packet += ",";

  if (telemetry.computerMode == Flight) {
    packet += "F";
  } else if (telemetry.computerMode == Simulation) {
    packet += "S";
  } else {
    packet += "MODE_ERROR";
  }
  packet += ",";

  if (telemetry.flightState == LowPower) {
    packet += "LOW_POWER";
  } else if (telemetry.flightState == PreLaunch) {
    packet += "FS0";
  } else if (telemetry.flightState == Launch) {
    packet += "FS1";
  } else if (telemetry.flightState == Peak) {
    packet += "FS2";
  } else if (telemetry.flightState == Deployment) {
    packet += "FS3";
  } else if (telemetry.flightState == Parachute) {
    packet += "FS4";
  } else if (telemetry.flightState == Landed) {
    packet += "FS5";
  }
  packet += ",";

  packet += String(telemetry.altitude, 1);
  packet += ",";

  if (telemetry.probeState == Deployed) {
    packet += "P";
  } else {
    packet += "N";
  }
  packet += ",";

  if (telemetry.parachuteState == Deployed) {
    packet += "C";
  } else {
    packet += "N";
  }
  packet += ",";

  if (telemetry.mastState == Deployed) {
    packet += "M";
  } else {
    packet += "N";
  }
  packet += ",";

  packet += String(telemetry.temperature, 1);
  packet += ",";

  packet += String(telemetry.voltage, 1);
  packet += ",";

  packet += formatTime(telemetry.gpsTime);
  packet += ",";

  packet += String(telemetry.gpsAltitude, 1);
  packet += ",";

  packet += String((int)telemetry.latitude);
  packet += ",";

  packet += String((int)telemetry.longitude);
  packet += ",";

  packet += String(telemetry.satConnections);
  packet += ",";

  packet += String(telemetry.tiltX, 2);
  packet += ",";

  packet += String(telemetry.tiltY, 2);
  packet += ",";

  packet += "CMD_ECHO"; //TODO: Impliment this
  packet += ",,";

  return packet;
};

// Function to conver pressure to altitude
float pressToAltitude(float hPa) {
  const float standardPressure = 1013.25;
  const float lapseRate = 0.0065; // in K/m

  float altitude = 44330.0 * (1.0 - pow(hPa / standardPressure, 1.0 / 5.255));
  return altitude;
}

// Creates a fresh time structure without logical garbage
TimeStruct createBlankTime() {
  TimeStruct t;
  t.hours = 0;
  t.minutes = 0;
  t.seconds = 0.0;
  return t;
};

// Keeps track of time using millis() updates
void updateTime(TimeStruct &toUpdate, unsigned long timeDeltaMillis) {
  float timeDeltaSeconds = (float)timeDeltaMillis / 1000.0;

  toUpdate.seconds += timeDeltaSeconds;
  if (toUpdate.seconds >= 60.0) {
    toUpdate.seconds -= 60.0;
    toUpdate.minutes += 1;
  }

  if (toUpdate.minutes >= 60) {
    toUpdate.minutes -= 60;
    toUpdate.hours += 1;
  }

  if (toUpdate.hours >= 24) {
    toUpdate.hours -= 24;
  }
}