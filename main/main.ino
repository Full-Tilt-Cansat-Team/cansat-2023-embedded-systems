/*
Libraries
*/
#include "Adafruit_BMP3XX.h"
#include "SparkFun_Ublox_Arduino_Library.h"
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <Math.h>
#include <EEPROM.h>

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

// Holds information in case of power loss
struct BackupData {
  int packets;
  float calibrationAlt;
  FlightState state;
  TimeStruct time;
};

BackupData backup;

// Will help in transitioning to XBEE (and logging)
void transmit(String toTransmit) {
  Serial.println(toTransmit);
  Serial1.println(toTransmit);
  Serial2.println(toTransmit);
}

// Flight-time variables
FlightState flightState; // Holds the determinant for state change logic

TelemetryPacket telemetry; // Holds telemetry packets

TimeStruct currentTime; // Holds global time
TimeStruct gpsTime; // Time as seen by GPS

float altitude; // Altitude of cansat
float lastAltitude; // Altitude at last logic step
float vertVelocity; // Velocity upwards
float calibrationAltitude; // To find absolute

float temperature; // Temperature (C)
float pressure; // Pressure (hPa)
float voltage; // Voltage (volts)
int adcVol; // voltage (unitless)

float gpsAltitude; // Altitude as measured by GPS (ASL)
float latitude; // GPS read latitude
float longitude; // GPS read longitude

SFE_UBLOX_GPS gps; // gps

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

Adafruit_BMP3XX bmp; // BMP388 Sensor for pressure/temperature
#define SEALEVELPRESSURE_HPA 1000

unsigned long currentCycleTime; // Used for time update every cycle
unsigned long lastCycleTime; // Used to calculate cycleTimeGap
unsigned long cycleTimeGap; // Time between cycles
unsigned long deltaTime; // How much time has passed between logic steps

unsigned long deltaTimeGPS;

bool transmitting = true; // Are we transmitting packets?
bool simulation = false; // Are we simulating?
bool simulationArmed = false;
float simulatedPressure; // Whats the simulated pressure?

// Definitions
#define PACKET_GAP_TIME 900
#define TEAM_ID 1073

// ADC pin for voltage
#define ADC_GPIO_PIN 27

// Parachute servo M1
Servo ParachuteServo;
#define PARACHUTE_PWM_PIN 19
#define PARACHUTE_MOS_PIN 22
#define PARACHUTE_CLOSED 0
#define PARACHUTE_OPEN 100

// Release servo
Servo ReleaseServo;
#define RELEASE_PWM_PIN 20
#define RELEASE_MOS_PIN 26
#define RELEASE_CLOSED 0
#define RELEASE_OPEN 40

// Backup Location
#define EEPROM_ADDR 0

// Debug flags
#define VERIFY_SENSORS false

int packetCount;

class CommandHandler {
  public:
  String buffer;
  String command;
  String arg;

  CommandHandler() {
    buffer = "";
  }

  void addToBuffer() {
    while (Serial1.available() > 0) {
      char c = (char)Serial1.read();
      Serial.print(c);
    if (c == ';') {
        Serial.print("\n");
        detectCommand(buffer);
        buffer = "";
      } else {
        buffer += (char)c;
      }
    }

    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      Serial.print(c);
    if (c == ';') {
        Serial.print("\n");
        detectCommand(buffer);
        buffer = "";
      } else {
        buffer += (char)c;
      }
    }
  }

  void detectCommand(String buffer) {
    // find the locations of the commas
    int first_comma = buffer.indexOf(',');
    int second_comma = buffer.indexOf(',', first_comma + 1);
    int third_comma = buffer.indexOf(',', second_comma + 1);

    String cmd = buffer.substring(second_comma + 1, third_comma);
    Serial.print("COMMAND: ");
    Serial.print("|");
    Serial.print(cmd);
    Serial.println("|");
    String arg = "";
    if (third_comma != -1) {
      arg = buffer.substring(third_comma+1);
      Serial.print("ARG: ");
      Serial.println(arg);
    }

    if (cmd == "CX") {
      CX(arg);
    } else if (cmd == "CAL") {
      CAL(arg);
    } else if (cmd == "ECHO") {
      ECHO(arg);
    } else if (cmd == "ST") {
      ST(arg);
    } else if (cmd == "FSS") {
      FSS(arg);
    } else if (cmd == "ST") {
      ST(arg);
    } else if (cmd == "SIM") {
      SIM(arg);
    } else if (cmd == "SIMP") {
      SIMP(arg);
    }

    telemetry.cmdEcho = String(cmd);
  }

  void ECHO(String arg) {
    transmit(arg);
  }

  void CX(String arg) {
    if (arg == "ON") {
      transmitting = true;
    } else {
      transmitting = false;
    }
  }

  void CAL(String arg) {
    calibrationAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    backup.calibrationAlt = calibrationAltitude;
  }

  void FSS(String arg) {
    if (arg == "LOW_POWER") {
      flightState = LowPower;
    } else if (arg == "PRE_LAUNCH") {
      flightState = PreLaunch;
    } else if (arg == "LAUNCH") {
      flightState = Launch;
    } else if (arg == "PEAK") {
      flightState = Peak;
    } else if (arg == "DEPLOYMENT") {
      flightState = Deployment;
    } else if (arg == "PARACHUTE") {
      flightState = Parachute;
    } else if (arg == "LANDED") {
      flightState = Landed;
    }
  }

  void ST(String arg) {
    int hour = arg.substring(0, 2).toInt();
    int minute = arg.substring(3, 5).toInt();
    float second = arg.substring(6).toFloat();

    currentTime.hours = hour;
    currentTime.minutes = minute;
    currentTime.seconds = second;

    backup.time = currentTime;
    EEPROM.put(EEPROM_ADDR, backup);

    Serial.println(arg.substring(0, 2));
    Serial.println(arg.substring(3, 5));
    Serial.println(arg.substring(6));
  }

  void SIM(String arg) {
    if (arg == "ENABLE") {
      simulationArmed = true;
    } else if (arg == "ACTIVATE" && simulationArmed) {
      simulation = true;
    } else if (arg == "DISABLE") {
      simulationArmed = false;
      simulation = false;
    }
  }

  void SIMP(String arg) {
    if (simulation) {
      simulatedPressure = arg.toFloat();
    }
  }
};

class 

// Commander for the flight computer
CommandHandler commander;

void setup() {
  Serial.begin(115200); // Open serial line TODO: Remove this in favor of xbee
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(9600); // Start Xbee Line

  // Release Setup
  pinMode(RELEASE_MOS_PIN, OUTPUT);
  ReleaseServo.attach(RELEASE_PWM_PIN, 420, 2400);
  digitalWrite(RELEASE_MOS_PIN, LOW);
  ReleaseServo.write(RELEASE_CLOSED);

  // Parachute Setup
  ParachuteServo.attach(PARACHUTE_PWM_PIN, 420, 2400);
  ParachuteServo.write(PARACHUTE_CLOSED);
  pinMode(PARACHUTE_MOS_PIN, OUTPUT);
  digitalWrite(PARACHUTE_MOS_PIN, LOW);

  // Power on to EEPROM
  EEPROM.begin(4096);

  // Load EEPROM backups
  EEPROM.get(EEPROM_ADDR, backup);

  // Place into logic spots
  flightState = backup.state;
  packetCount = backup.packets;
  currentTime = backup.time;
  calibrationAltitude = backup.calibrationAlt;

  altitude = 0.0;
  lastAltitude = 0.0;
  vertVelocity = 0.0;

  temperature = 0;
  pressure = 0;

  lastCycleTime = millis(); // Prime cycle execution
  deltaTime = 0; // Immediatly start flight logic
  deltaTimeGPS = 99999999999; // Get GPS time right away


  // Setup I2c
  Wire1.setSDA(14);
  Wire1.setSCL(15);
  Wire1.begin();

  // Openlog Serial
  Serial2.setTX(8);
  Serial2.setRX(9);
  Serial2.begin(9600);

  // Begin BMP
  if (!bmp.begin_I2C(0x77, &Wire1) && VERIFY_SENSORS) {
    while (true) {
      Serial.println("BMP ERROR");
      delay(1000);
    }
  }
  bmp.performReading();

  delay(100);

  // Begin GPS
  if (gps.begin(Wire1) == false && VERIFY_SENSORS) //Connect to the Ublox module using Wire port
  {
    while (true) {
      Serial.println("GPS ERROR");
      delay(1000);
    }
  }
  gps.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  gps.saveConfiguration(); //Save the current settings to flash and BBR

  // Initialise bno
  if (!bno.begin() && VERIFY_SENSORS)
  {
    while (true) {
      Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      delay(1000);
    }
  }

  transmit("BOOTED");

  // Create command handler
  commander = CommandHandler();

  // Buzzer pin
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
};

void loop() {
  // Update our timeing control
  currentCycleTime = millis();
  cycleTimeGap = currentCycleTime - lastCycleTime;
  deltaTime += cycleTimeGap;
  deltaTimeGPS += cycleTimeGap;

  // If one second has passed, perform flight logic
  if (deltaTime >= PACKET_GAP_TIME) {
    // Calculate current time
    updateTime(currentTime, deltaTime);
    updateTime(gpsTime, deltaTime);

    Serial.println("Time updated");

    // Read pressure and temperature from BMP
    bmp.performReading();
    temperature = bmp.temperature;
    vertVelocity = (altitude - lastAltitude) / ((float)deltaTime / 1000.0); // calculate velocity

    Serial.println("BMP READ");

    if (!simulation) {
      altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - calibrationAltitude;
      pressure = bmp.pressure;
    } else {
      altitude = pressureToAltitude(simulatedPressure);
      pressure = simulatedPressure;
    }

    Serial.println("Altitude gotten");

    // Read GPS values
    latitude = (float)gps.getLatitude() / 10000000;
    longitude = (float)gps.getLongitude() / 10000000;
    gpsAltitude = gps.getAltitude();

    Serial.println("GPS GOT");

    // Analog read
    adcVol = analogRead(ADC_GPIO_PIN);
    // ADC to V w/ Voltage Divider = (ADC Output / Maximum ADC Output) x Voltage Across R1
    voltage = (3.3 * (float)adcVol / 1023) * ((50 + 47) / 50);

    Serial.println("VOLTAGE READ");

    commander.addToBuffer();
    
    // Flight state changes
    {
      if (flightState == LowPower) {
        // Do nothing, and don't do flight logic
        digitalWrite(17, LOW);
        digitalWrite(18, LOW);
      } else if (flightState == PreLaunch) {

        // When 10m passed and velocity greater tha3n 10m/s, move to launch
        if (altitude >= 10.0 && vertVelocity >= 0.0) {
          // Start camera recording (TODO)
          // Start transmitting packets
          transmitting = true;
          // Move into launch state
          flightState = Launch;
        }

        // Keep servos on
        digitalWrite(PARACHUTE_MOS_PIN, HIGH);
        digitalWrite(RELEASE_MOS_PIN, HIGH);

      } else if (flightState == Launch) {

        // When above 550m, transition to this stage (present to avoid accidental movment directly into landing stage)
        if (altitude >= 550.0) {
          flightState = Peak;
        }

        // Servos on and closed
        digitalWrite(PARACHUTE_MOS_PIN, HIGH);
        digitalWrite(RELEASE_MOS_PIN, HIGH);
        ParachuteServo.write(PARACHUTE_CLOSED);
        ReleaseServo.write(RELEASE_CLOSED);

      } else if (flightState == Peak) {

        // Peak stage transitions to deployment when below 500m and velocity is <0m/s
        if (altitude < 500.0) {
          // Update telemetry
          telemetry.heatShieldState = Deployed;
          // Move to deploymment state
          flightState = Deployment;
        }

        // Servos on and closed
        digitalWrite(PARACHUTE_MOS_PIN, HIGH);
        digitalWrite(RELEASE_MOS_PIN, HIGH);
        ParachuteServo.write(PARACHUTE_CLOSED);
        ReleaseServo.write(RELEASE_CLOSED);

      } else if (flightState == Deployment) {

        // Move to parachute state when below 200m and negative velocity
        if (altitude < 200.0) {
          // Move to chute stage
          flightState = Parachute;

          // Open chute early
          telemetry.parachuteState = Deployed;
          ParachuteServo.write(PARACHUTE_OPEN);
          digitalWrite(PARACHUTE_MOS_PIN, HIGH);
        }

        // Release servo open
        digitalWrite(PARACHUTE_MOS_PIN, HIGH);
        digitalWrite(RELEASE_MOS_PIN, HIGH);
        ParachuteServo.write(PARACHUTE_CLOSED);
        ReleaseServo.write(RELEASE_OPEN);

      } else if (flightState == Parachute) {

        // Just wait until velocity is less than 2
        if (altitude < 10) {
          // Move to landed state
          flightState = Landed;

          // Execute wacky subroutine (TODO)
        }

        // Chute servo open
        digitalWrite(PARACHUTE_MOS_PIN, HIGH);
        digitalWrite(RELEASE_MOS_PIN, HIGH);
        ParachuteServo.write(PARACHUTE_OPEN);
        ReleaseServo.write(RELEASE_OPEN);

      } else if (flightState == Landed) {
        // Beacons
        digitalWrite(17, HIGH);
        digitalWrite(18, HIGH);
      }
    }

    Serial.println("FLIGHT STATE CHANGES");

    Serial.println("TIME GOTTEN");

    sensors_event_t orientationData;

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    Serial.println("ORIENTATION FETCHED");

    // Fill the fields of the telemetry packet
    telemetry.teamId = TEAM_ID;
    telemetry.missionTime = currentTime;
    telemetry.packetCount = packetCount;
    telemetry.computerMode = Flight;
    telemetry.flightState = flightState;
    telemetry.altitude = altitude;
    telemetry.heatShieldState = NotDeployed;
    telemetry.parachuteState = NotDeployed;
    telemetry.mastState = NotDeployed;
    telemetry.temperature = temperature;
    telemetry.voltage = voltage; // TODO: Impliment voltage sensing
    telemetry.pressure = pressure;

    telemetry.gpsTime = gpsTime;
    telemetry.gpsAltitude = gpsAltitude;
    telemetry.latitude = latitude;
    telemetry.longitude = longitude;
    telemetry.satConnections = gps.getSIV();
    telemetry.tiltX = 0;//orientationData.orientation.x;
    telemetry.tiltY = 0;//orientationData.orientation.y;

    Serial.println("TELEMETRY FETCHED");

    // Assemble packet
    String packet = assemblePacket(telemetry);

    Serial.println("TELEMETRY ASSEMBLED");

    // Convert to bytes
    // TODO: Convert all strings in here to bytes natively
    uint8_t packetBytes[packet.length()];
    packet.getBytes(packetBytes, packet.length());

    if (transmitting) {
      transmit(packet);

      packetCount++;
    }

    Serial.println("TRANSMITTED");

    // Update backups
    backup.packets = packetCount;
    backup.time = currentTime;
    backup.state = flightState;
    EEPROM.put(EEPROM_ADDR, backup);
    EEPROM.commit();

    Serial.println("UPDATED");
    Serial.println(deltaTime);

    Serial.println("");
    Serial.println("");

    // Update timing
    deltaTime = millis() - currentCycleTime;
  }

  if (deltaTimeGPS >= 60000) {
    gpsTime.hours = gps.getHour();
    gpsTime.minutes = gps.getMinute();
    gpsTime.seconds = gps.getSecond();
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
    packet += "PRE_LAUNCH";
  } else if (telemetry.flightState == Launch) {
    packet += "LAUNCH";
  } else if (telemetry.flightState == Peak) {
    packet += "PEAK";
  } else if (telemetry.flightState == Deployment) {
    packet += "DEPLOYMENT";
  } else if (telemetry.flightState == Parachute) {
    packet += "PARACHUTE";
  } else if (telemetry.flightState == Landed) {
    packet += "LANDED";
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

  packet += String(telemetry.pressure, 2);
  packet += ",";

  packet += formatTime(telemetry.gpsTime);
  packet += ",";

  packet += String(telemetry.gpsAltitude, 1);
  packet += ",";

  packet += String(telemetry.latitude, 4);
  packet += ",";

  packet += String(telemetry.longitude, 4);
  packet += ",";

  packet += String(telemetry.satConnections);
  packet += ",";

  packet += String(telemetry.tiltX, 2);
  packet += ",";

  packet += String(telemetry.tiltY, 2);
  packet += ",";

  packet += telemetry.cmdEcho;
  packet += ",,";

  return packet;
};

// Function to conver pressure to altitude
// The standard pressure at sea level in pascals
const double p0 = 101325.0;

// The temperature lapse rate in K/m
const double lapse = -0.0065;

// The universal gas constant
const double R = 8.31447;

// The molar mass of dry air in kg/mol
const double M = 0.0289644;

// The gravitational acceleration in m/s^2
const double g = 9.80665;

// The standard temperature at sea level in K
const double T0 = 288.15;

double pressureToAltitude(float p) {
  p *= 100;
  return T0 / lapse * (1 - pow(p / p0, R * lapse / (g * M)));
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
