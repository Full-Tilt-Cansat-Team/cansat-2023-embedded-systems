/*
Libraries
*/
#include "Adafruit_BMP3XX.h"
#include "SparkFun_Ublox_Arduino_Library.h"
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

// Will help in transitioning to XBEE
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

SFE_UBLOX_GPS gps; // gps

Adafruit_BMP3XX bmp; // BMP388 Sensor for pressure/temperature
#define SEALEVELPRESSURE_HPA 1000

unsigned long currentCycleTime; // Used for time update every cycle
unsigned long lastCycleTime; // Used to calculate cycleTimeGap
unsigned long cycleTimeGap; // Time between cycles
unsigned long deltaTime; // How much time has passed between logic steps

bool transmitting = true; // Are we transmitting packets?

// Definitions
#define PACKET_GAP_TIME 1000
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
  }

  void ST(String arg) {
    currentTime.hours = atoi(arg.substring(0, 2).c_str());
    currentTime.minutes = atoi(arg.substring(3,5).c_str());
    currentTime.seconds = (float)atoi(arg.substring(6,8).c_str());
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
};

// Commander for the flight computer
CommandHandler commander;

void setup() {
  Serial.begin(9600); // Open serial line TODO: Remove this in favor of xbee
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

  flightState = LowPower;

  altitude = 0.0;
  lastAltitude = 0.0;
  vertVelocity = 0.0;

  temperature = 0;
  pressure = 0;

  packetCount = 0;

  lastCycleTime = millis(); // Prime cycle execution
  deltaTime = 0; // Immediatly start flight logic

  // Setup I2c
  Wire1.setSDA(14);
  Wire1.setSCL(15);
  Wire1.begin();

  // Openlog Serial
  Serial2.setTX(8);
  Serial2.setRX(9);
  Serial2.begin(9600);

  // Begin BMP
  if (!bmp.begin_I2C(0x77, &Wire1)) {
    while (true) {
      Serial.println("BMP ERROR");
      delay(1000);
    }
  }
  bmp.performReading();

  delay(100);

  // Begin GPS
  if (gps.begin(Wire1) == false) //Connect to the Ublox module using Wire port
  {
    while (true) {
      Serial.println("GPS ERROR");
      delay(1000);
    }
  }

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

    // Analog read
    adcVol = analogRead(ADC_GPIO_PIN);
    // ADC to V w/ Voltage Divider = (ADC Output / Maximum ADC Output) x Voltage Across R1
    voltage = (3.3 * (float)adcVol / 1023) * ((50 + 47) / 50);

    commander.addToBuffer();
    
    // Flight state changes
    {
      if (flightState == LowPower) {
        // Do nothing, and don't do flight logic
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
        if (altitude >= 500.0) {
          flightState = Peak;
        }

        // Servos on and closed
        digitalWrite(PARACHUTE_MOS_PIN, HIGH);
        digitalWrite(RELEASE_MOS_PIN, HIGH);
        ParachuteServo.write(PARACHUTE_CLOSED);
        ReleaseServo.write(RELEASE_CLOSED);

      } else if (flightState == Peak) {

        // Peak stage transitions to deployment when below 500m and velocity is <0m/s
        if (altitude < 400.0) {
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

    gpsTime.hours = gps.getHour();
    gpsTime.minutes = gps.getMinute();
    gpsTime.seconds = gps.getSecond();

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
    telemetry.temperature = temperature; // TODO: Impliment Temperature
    telemetry.voltage = voltage; // TODO: Impliment voltage sensing
    telemetry.gpsTime = createBlankTime(); // TODO: Impliment GPS
    telemetry.gpsAltitude = 0;
    telemetry.latitude = gps.getLatitude();
    telemetry.longitude = gps.getLongitude();
    telemetry.satConnections = gps.getSIV();
    telemetry.tiltX = 0;
    telemetry.tiltY = 90;

    // Assemble packet
    String packet = assemblePacket(telemetry);

    // Convert to bytes
    // TODO: Convert all strings in here to bytes natively
    uint8_t packetBytes[packet.length()];
    packet.getBytes(packetBytes, packet.length());

    if (transmitting) {
      transmit(packet);

      packetCount++;
    }

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

  packet += telemetry.cmdEcho;
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
