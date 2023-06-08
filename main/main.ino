/*
Libraries
*/
#include "Adafruit_BMP3XX.h"
#include "SparkFun_Ublox_Arduino_Library.h"
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include <EEPROM.h>

#include "timing.h"
#include "enums.h"

// Debug flags
#define VERIFY_SENSORS false

// Definitions
#define PACKET_GAP_TIME 995
#define TEAM_ID 1073
#define MAX_ABORT_TIME 

// ADC pin for voltage
#define ADC_GPIO_PIN 27

// Release servo pins
Servo ReleaseServo;
#define RELEASE_PWM_PIN 15
#define RELEASE_MOS_PIN 14
#define RELEASE_CLOSED 135 //135
#define RELEASE_FALL 85 //85
#define RELEASE_PARACHUTE 0 //0

// Orientation servo pins
Servo OrientationServo;
#define ORIENT_PWM_PIN 11
#define ORIENT_MOS_PIN 10

// Flag servo pins
Servo FlagServo;
#define FLAG_PWM_PIN 4
#define FLAG_MOS_PIN 3

// Beacon pins
#define BUZZER_MOS 20 //19
#define LED_MOS LED_BUILTIN//20

// Backup Memory location
#define EEPROM_ADDR 0

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
  OptionalToggleState MRA;
  OptionalToggleState PRS;
  OptionalToggleState URA;
  OptionalToggleState FRA;
  OptionalToggleState BCS;
  OptionalToggleState CAM;
  float orX;
  float orY;
  float orZ;
  float gsZ;
  String faultMessage;
};

// Holds information in case of power loss
struct BackupData {
  int packets;
  float calibrationAlt;
  float gpsCalibrationAlt;
  FlightState state;
  TimeStruct time;
};

BackupData backup;

// Will help in transitioning to XBEE (and logging)
void transmit(String toTransmit) {
  // Prints to Serial  
  Serial.println(toTransmit);
  // Prints to XBee
  Serial1.println(toTransmit);
  // Prints to OpenLog
  Serial2.println(toTransmit);
}

// Flight-time variables
FlightState flightState; // Holds the determinant for state change logic
FlightMode computerMode; // Holds the determinant for sim change logic

TelemetryPacket telemetry; // Holds telemetry packets

TimeStruct currentTime; // Holds global time
TimeStruct gpsTime; // Time as seen by GPS

OptionalToggleState MRA;
OptionalToggleState PRS;
OptionalToggleState URA;
OptionalToggleState FRA;
OptionalToggleState BCS;
OptionalToggleState CAM;

float altitude; // Altitude of cansat
float lastAltitude; // Altitude at last logic step
float vertVelocity; // Velocity upwards
float calibrationAltitude; // To find absolute
float gpsCalibrationAltitude; // Finds relative altitude from gps

float temperature; // Temperature (C)
float pressure; // Pressure (hPa)
float voltage; // Voltage (volts)
int adcVol; // voltage (unitless)

float gpsAltitude; // Altitude as measured by GPS (ASL)
float latitude; // GPS read latitude
float longitude; // GPS read longitude

// Variables for the orientation(?)
float orX;
float orY;
float orZ;
float gsZ;

SFE_UBLOX_GPS gps; // gps

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //BNO055 for orientation sensing

Adafruit_BMP3XX bmp; // BMP388 Sensor for pressure/temperature
#define SEALEVELPRESSURE_HPA 1000

unsigned long currentCycleTime; // Used for time update every cycle
unsigned long lastCycleTime; // Used to calculate cycleTimeGap
unsigned long cycleTimeGap; // Time between cycles
unsigned long deltaTime; // How much time has passed between logic steps
unsigned long timeAbortEntered; // When the abort stage was entered

bool transmitting = true; // Are we transmitting packets?
bool simulation = false; // Are we simulating?
bool simulationArmed = false; // Required to enable simulation mode
float simulatedPressure; // Whats the simulated pressure?

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
    while (Serial2.available() > 0) {
      char c = (char)Serial2.read();
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
    } else if (cmd == "MRA") {
      MRA(arg);
    } else if (cmd == "PRS") {
      PRS(arg);
    } else if (cmd == "URA") {
      URA(arg);
    } else if (cmd == "FRA") {
      FRA(arg);
    } else if (cmd == "BCS") {
      BCS(arg);
    } else if (cmd == "CAM") {
      CAM(arg);
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
    gpsCalibrationAltitude = gps.getAltitude();
    
    backup.calibrationAlt = calibrationAltitude;
    backup.gpsCalibrationAlt = gpsCalibrationAltitude;
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
    } else if (arg == "ABORT") {
      flightState = ABORT;
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

  // Manual Release Activation
  void MRA(String arg) {
    if (arg == "ON") {
      telemetry.MRA = ToggleOn;
      telemetry.heatShieldState = Deployed;
      digitalWrite(RELEASE_MOS_PIN, HIGH);
      ReleaseServo.write(RELEASE_FALL);
      digitalWrite(RELEASE_MOS_PIN, HIGH);
      digitalWrite(BUZZER_MOS, HIGH);
      digitalWrite(LED_MOS, HIGH);
      delay(500);
      digitalWrite(BUZZER_MOS, LOW);
      digitalWrite(LED_MOS, LOW);
    } else if (arg == "OFF") {
      telemetry.MRA = ToggleOff;
      digitalWrite(RELEASE_MOS_PIN, HIGH);
      ReleaseServo.write(RELEASE_CLOSED);
      digitalWrite(RELEASE_MOS_PIN, HIGH);
      digitalWrite(BUZZER_MOS, HIGH);
      digitalWrite(LED_MOS, HIGH);
      delay(500);
      digitalWrite(BUZZER_MOS, LOW);
      digitalWrite(LED_MOS, LOW);
    }
  }

  void PRS(String arg) {
    if (arg == "ON") {
      telemetry.PRS = ToggleOn;
      telemetry.parachuteState = Deployed;
      digitalWrite(RELEASE_MOS_PIN, HIGH);
      ReleaseServo.write(RELEASE_PARACHUTE);
      digitalWrite(RELEASE_MOS_PIN, HIGH);
      digitalWrite(BUZZER_MOS, HIGH);
      digitalWrite(LED_MOS, HIGH);
      delay(500);
      digitalWrite(BUZZER_MOS, LOW);
      digitalWrite(LED_MOS, LOW);
    } else if (arg == "OFF") {
      telemetry.PRS = ToggleOff;
      digitalWrite(RELEASE_MOS_PIN, HIGH);
      ReleaseServo.write(RELEASE_CLOSED);
      digitalWrite(RELEASE_MOS_PIN, HIGH);
      digitalWrite(BUZZER_MOS, HIGH);
      digitalWrite(LED_MOS, HIGH);
      delay(500);
      digitalWrite(BUZZER_MOS, LOW);
      digitalWrite(LED_MOS, LOW);
    }
  }

  void URA(String arg) {
    if (arg == "ON") {
      telemetry.URA = ToggleOn;
      // ADD ORIENTATION
    }
    if (arg == "OFF") {
      telemetry.URA = ToggleOff;
    }
  }

  void FRA(String arg) {
    if (arg == "ON") {
      telemetry.FRA = ToggleOn;
      telemetry.mastState = Deployed;
    }
    if (arg == "OFF") {
      telemetry.FRA = ToggleOff;
      telemetry.mastState = NotDeployed;
    }
  }

  void CAM(String arg) {
    if (arg == "ON") {
      telemetry.CAM = ToggleOn;
    }
    if (arg == "OFF") {
      telemetry.CAM = ToggleOff;
    }
  }

  void BCS(String arg) {
    if (arg == "ON") {
      telemetry.BCS = ToggleOn;
      digitalWrite(BUZZER_MOS, HIGH);
      digitalWrite(LED_MOS, HIGH);
    }
    if (arg == "OFF") {
      telemetry.BCS = ToggleOff;
      digitalWrite(BUZZER_MOS, LOW);
      digitalWrite(LED_MOS, LOW);
    }
  }

};

class UprightingSystem {
  private:
    float speed;
    float last_reading;
    int direction;
  public:
    UprightingSystem(float initial_speed) {
        speed = initial_speed;
        direction = 1;
        last_reading = 0;
    }

    float control(float current_reading) {
      if (current_reading < last_reading) {
          direction *= -1;
          speed *= 0.5;
      }
      
      last_reading = current_reading;
        
      return (80 * speed * direction + 93);
    }

    void loopControl() {
      float a, b;
      digitalWrite(ORIENT_MOS_PIN, HIGH);
      while (true) {
        imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
        b = grav.z();
        a = control(b);
        OrientationServo.write(a);

        delay(100);
        
        OrientationServo.write(100);

        delay(100);
        
        Serial.println(b);
        Serial.println(a);
        Serial.println("");
        delay(1000);
        if (speed <= 0.1) {
          digitalWrite(FLAG_MOS_PIN, HIGH);
          FlagServo.write(180);
          delay(5000);
          digitalWrite(FLAG_MOS_PIN, LOW);
          break;
        }
      }
    }
};

// Commander for the flight computer
CommandHandler commander;

UprightingSystem us = UprightingSystem(1);

// Fault trackers
FaultDetected bnoFault = NoFault;
FaultDetected bmpFault = NoFault;
FaultDetected gpsFault = NoFault;
FaultDetected altitudeFault = NoFault;

void setup() {
  Serial.begin(115200); // Open serial line to computer for umbilical

  Serial2.setTX(8);
  Serial2.setRX(9);
  Serial2.begin(9600); // Start Xbee Line for uplink

  telemetry.faultMessage = ""; // No message for the moment

  // Release Servo Setup
  pinMode(RELEASE_MOS_PIN, OUTPUT);
  ReleaseServo.attach(RELEASE_PWM_PIN);
  digitalWrite(RELEASE_MOS_PIN, LOW);
  ReleaseServo.write(RELEASE_CLOSED);

  // Orient Servo Setup
  pinMode(ORIENT_MOS_PIN, OUTPUT);
  OrientationServo.attach(ORIENT_PWM_PIN, 420, 2400);
  digitalWrite(ORIENT_MOS_PIN, LOW);

  // Flag Servo Setup
  pinMode(FLAG_MOS_PIN, OUTPUT);
  FlagServo.attach(FLAG_PWM_PIN, 420, 2400);
  digitalWrite(FLAG_MOS_PIN, LOW);

  // Beacon setup
  pinMode(BUZZER_MOS, OUTPUT);
  digitalWrite(BUZZER_MOS, LOW);
  pinMode(LED_MOS, OUTPUT);
  digitalWrite(LED_MOS, LOW);

  pinMode(20, OUTPUT);
  pinMode(20, HIGH);

  // Initialize flash EEPROM block
  EEPROM.begin(4096);

  // Load EEPROM data backup
  EEPROM.get(EEPROM_ADDR, backup);

  // Loack backup states into runtime
  flightState = backup.state;
  packetCount = backup.packets;
  currentTime = backup.time;
  calibrationAltitude = backup.calibrationAlt;
  gpsCalibrationAltitude = backup.gpsCalibrationAlt;

  // Initialize toggle states
  MRA = ToggleOff;
  PRS = ToggleOff;
  URA = ToggleOff;
  FRA = ToggleOff;
  BCS = ToggleOff;
  CAM = ToggleOff;

  // Initialize altitude and velocity
  altitude = 0.0;
  lastAltitude = 0.0;
  vertVelocity = 0.0;

  // Initialize temperature and pressure
  temperature = 0;
  pressure = 0;

  lastCycleTime = millis(); // Prime cycle execution

  // Setup I2c
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire.begin();

  // Open serial line for datalogging
  Serial1.setTX(12);
  Serial1.setRX(13);
  Serial1.begin(9600);

  // Initialize and verify  BMP
  if (!bmp.begin_I2C(0x77, &Wire)) {
    if (VERIFY_SENSORS) {
      for (int i = 0; i < 10; i++) {
        Serial.println("BMP ERROR");
        delay(10);
      }
    }

    bmpFault = SingleFault;
    telemetry.faultMessage += "(BMP FAULT)";
    
  }
  bmp.performReading();

  // Initialize and verify GPS
  if (gps.begin(Wire) == false) //Connect to the Ublox module using Wire port
  {
    if (VERIFY_SENSORS) {
      for (int i = 0; i < 10; i++) {
        Serial.println("GPS ERROR");
        delay(10);
      }
    }

    gpsFault = SingleFault;
    telemetry.faultMessage += "(GPS FAULT)";
    
  }
  gps.setI2COutput(COM_TYPE_UBX); // Disable extra NMEA sentences
  gps.setNavigationFrequency(10, 1); // Set fix rate to 10Hz, max 1 second timeout
  gps.saveConfiguration(); //Save the current settings to flash and BBR

  // Initialize and verify BNO
  if (!bno.begin())
  {
    if (VERIFY_SENSORS) {
      for (int i = 0; i < 10; i++) {
        Serial.println("BNO ERROR");
        delay(10);
      }
    }

    bnoFault = SingleFault;
    telemetry.faultMessage += "(BNO FAULT)";
  }
  bno.setExtCrystalUse(true);

  if (bmpFault == SingleFault) {
    if (gpsFault == SingleFault) {
      altitudeFault = DoubleFault;
      telemetry.faultMessage += "(DOUBLE ALTITUDE FAULT)";
      flightState = ABORT;
      timeAbortEntered = millis();
    } else {
      altitudeFault = SingleFault;
      
      telemetry.faultMessage += "(SINGLE ALTITUDE FAULT)";
    }
  }

  // Create command handler
  commander = CommandHandler();

  // Fire beacon to indicate end of boot
  digitalWrite(BUZZER_MOS, HIGH);
  digitalWrite(LED_MOS, HIGH);
  delay(100);
  digitalWrite(BUZZER_MOS, LOW);
  digitalWrite(LED_MOS, LOW);
  delay(100);
  digitalWrite(BUZZER_MOS, HIGH);
  digitalWrite(LED_MOS, HIGH);
  delay(100);
  digitalWrite(BUZZER_MOS, LOW);
  digitalWrite(LED_MOS, LOW);
  delay(100);
  digitalWrite(BUZZER_MOS, HIGH);
  digitalWrite(LED_MOS, HIGH);
  delay(100);
  digitalWrite(BUZZER_MOS, LOW);
  digitalWrite(LED_MOS, LOW);
};

bool abortPreviouslySet = false;

void loop() {
  // Update time between cycles
  currentCycleTime = millis();
  cycleTimeGap = currentCycleTime - lastCycleTime;
  deltaTime += cycleTimeGap;

  // If enough time has passed, execute logic
  if (deltaTime >= PACKET_GAP_TIME) {
    updateTime(currentTime, deltaTime); // Calculate time for telemetry

    // Read pressure and temperature from BMP
    bmp.performReading();
    temperature = bmp.temperature;
    vertVelocity = (altitude - lastAltitude) / ((float)deltaTime / 1000.0); // calculate velocity

    // Poll GPS for position and altitude
    latitude = (float)gps.getLatitude() / 10000000;
    longitude = (float)gps.getLongitude() / 10000000;
    gpsAltitude = gps.getAltitude();

    // If in simulation mode, use simulated pressure, otherwise use BMP
    if (!simulation) {
      if (altitudeFault == NoFault) {
        altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - calibrationAltitude;
        pressure = bmp.pressure;
      } else if (altitudeFault == SingleFault) {
        altitude = gpsAltitude - gpsCalibrationAltitude;
      } else if (altitudeFault == DoubleFault && !abortPreviouslySet) {
        flightState == ABORT;
        abortPreviouslySet = true;
      }
    } else {
      altitude = pressureToAltitude(simulatedPressure);
      pressure = simulatedPressure;
    }

    // Read our voltage over the ADC
    adcVol = analogRead(ADC_GPIO_PIN);
    // ADC to V w/ Voltage Divider = (ADC Output / Maximum ADC Output) x Voltage Across R1
    voltage = (3.3 * (float)adcVol / 1023) * ((50 + 47) / 50);

    // Poll serial chip for any new command data
    commander.addToBuffer();
    
    // Flight state changes
    {
      if (flightState == LowPower) {
        // Do nothing, and don't do flight logic
        digitalWrite(BUZZER_MOS, LOW);
        digitalWrite(LED_MOS, LOW);
      } else if (flightState == PreLaunch) {
        
        digitalWrite(BUZZER_MOS, LOW);
        digitalWrite(LED_MOS, LOW);

        // When 10m passed and velocity greater tha3n 10m/s, move to launch
        if (altitude >= 10.0 && vertVelocity >= 0.0) {
          // Start camera recording (TODO)
          // Start transmitting packets
          transmitting = true;
          // Move into launch state
          flightState = Launch;

          // Pulse beacon
          pulseBeacon();
        }

        // Lock servos to closed
        digitalWrite(RELEASE_MOS_PIN, LOW);
        ReleaseServo.write(RELEASE_CLOSED);
        digitalWrite(RELEASE_MOS_PIN, HIGH);

      } else if (flightState == Launch) {

        // When above 550m, transition to this stage (present to avoid accidental movment directly into landing stage)
        if (altitude >= 550.0) {
          flightState = Peak;
          pulseBeacon();
        }

        // Servos on and closed
        digitalWrite(RELEASE_MOS_PIN, HIGH);
        ReleaseServo.write(RELEASE_CLOSED);

      } else if (flightState == Peak) {

        // Peak stage transitions to deployment when below 500m and velocity is <0m/s
        if (altitude < 500.0) {
          // Update telemetry
          telemetry.heatShieldState = Deployed;
          telemetry.MRA = ToggleOn;
          // Move to deploymment state
          flightState = Deployment;
          pulseBeacon();
        }

        // Servos on and closed
        digitalWrite(RELEASE_MOS_PIN, HIGH);
        ReleaseServo.write(RELEASE_CLOSED);

      } else if (flightState == Deployment) {

        // Move to parachute state when below 200m and negative velocity
        if (altitude < 200.0) {
          // Move to chute stage
          flightState = Parachute;
          telemetry.PRS = ToggleOn;
          telemetry.parachuteState = Deployed;
          pulseBeacon();
        }

        // Release servo open
        digitalWrite(RELEASE_MOS_PIN, HIGH);
        ReleaseServo.write(RELEASE_FALL);

      } else if (flightState == Parachute) {

        // Just wait until velocity is less than 2
        if (altitude < 10) {
          // Move to landed state
          flightState = Landed;
          digitalWrite(RELEASE_MOS_PIN, HIGH);
          ReleaseServo.write(RELEASE_PARACHUTE);
          pulseBeacon();
          us.loopControl();
        }

        // Chute servo open
        digitalWrite(RELEASE_MOS_PIN, HIGH);
        ReleaseServo.write(RELEASE_PARACHUTE);

      } else if (flightState == Landed) {
        // Beacons
        digitalWrite(BUZZER_MOS, HIGH);
        digitalWrite(LED_MOS, HIGH);
        
      } else if (flightState == ABORT) {
        if (millis() - timeAbortEntered >= 250000) {
          flightState = Landed;
        }
        
        // Full slowdown
        digitalWrite(RELEASE_MOS_PIN, HIGH);
        ReleaseServo.write(RELEASE_FALL);
      }
    }

    // Get orientation data from the BNO if not in low power
    sensors_event_t orientationData;
    if (flightState != LowPower) {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    }

    // Fill the fields of the telemetry packet with sensor data
    telemetry.teamId = TEAM_ID;
    telemetry.missionTime = currentTime;
    telemetry.packetCount = packetCount;
    telemetry.computerMode = Flight;
    telemetry.flightState = flightState;
    telemetry.altitude = altitude;
    telemetry.temperature = temperature;
    telemetry.voltage = voltage;
    telemetry.pressure = pressure;

    // Get time from GPS
    gpsTime.hours = gps.getHour();
    gpsTime.minutes = gps.getMinute();
    gpsTime.seconds = gps.getSecond();

    // Fill the fields of the telemetry packet with GPS data
    telemetry.gpsTime = gpsTime;
    telemetry.gpsAltitude = gpsAltitude;
    telemetry.latitude = latitude;
    telemetry.longitude = longitude;
    telemetry.satConnections = gps.getSIV();
    telemetry.tiltX = orientationData.orientation.x;
    telemetry.tiltY = orientationData.orientation.y;

    // Assemble packet
    String packet = assemblePacket(telemetry);

    // Convert to bytes and transmit
    uint8_t packetBytes[packet.length()];
    packet.getBytes(packetBytes, packet.length());

    if (transmitting) {
      transmit(packet);

      Serial.println(vertVelocity);

      packetCount++;
    }

    // Update and commit EEPROM flash block
    backup.packets = packetCount;
    backup.time = currentTime;
    backup.state = flightState;
    EEPROM.put(EEPROM_ADDR, backup);
    EEPROM.commit();

    // Update last packet time
    deltaTime = millis() - currentCycleTime;
  }
  
  lastCycleTime = currentCycleTime;
};

// Pulses the beacon
void pulseBeacon() {
  digitalWrite(BUZZER_MOS, HIGH);
  digitalWrite(LED_MOS, HIGH);
  delay(50);
  digitalWrite(BUZZER_MOS, LOW);
  digitalWrite(LED_MOS, LOW);
}

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
  } else if (telemetry.flightState == ABORT) {
    packet += "ABORT";
  }
  packet += ",";

  packet += String(telemetry.altitude, 1);
  packet += ",";

  if (telemetry.heatShieldState == Deployed) {
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

  packet += telemetry.faultMessage;

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

// Converts pressure to altitude, for use in simulation
double pressureToAltitude(float p) {
  p *= 100;
  return T0 / lapse * (1 - pow(p / p0, R * lapse / (g * M)));
};
