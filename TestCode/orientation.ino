#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

Servo orServo;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

class Uprighting {
  public:
  float iOrX, iOrY, iOrZ;
  float iGsX, iGsY, iGsZ;
  float cOrX, cOrY, cOrZ;
  float cGsX, cGsY, cGsZ;
  int sSpeed;

  Uprighting() {
    iOrX = 0;
    iOrY = 0;
    iOrZ = 0;
    iGsX = 0;
    iGsY = 0;
    iGsZ = 0;
    cOrX = 0;
    cOrY = 0;
    cOrZ = 0;
    cGsX = 0;
    cGsY = 0;
    cGsZ = 0;
    sSpeed = 100;
  }

  void updateinitials(float iorx, float iory, float iorz, float igsx, float igsy, float igsz) {
    iOrX = iorx;
    iOrY = iory;
    iOrZ = iorz;
    iGsX = igsx;
    iGsY = igsy;
    iGsZ = igsz;
  }

  void updatecurrents(float corx, float cory, float corz, float cgsx, float cgsy, float cgsz) {
    cOrX = corx;
    cOrY = cory;
    cOrZ = corz;
    cGsX = cgsx;
    cGsY = cgsy;
    cGsZ = cgsz;
    Serial.println(cGsZ);
  }

  void upright() {
    delay(5000);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    updateinitials(euler.x(), euler.y(), euler.z(), grav.x(), grav.y(), grav.z());
    delay(100);
    digitalWrite(3, HIGH);
    orServo.write(100);
    delay(500);
    digitalWrite(3, LOW);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    delay(BNO055_SAMPLERATE_DELAY_MS);
    updatecurrents(euler.x(), euler.y(), euler.z(), grav.x(), grav.y(), grav.z());
    delay(100);
    float diff_gs_z;
    diff_gs_z = cGsZ - iGsZ;
    if (diff_gs_z > 0) {
      sSpeed = 100;
      while (cGsZ < 9.65) {
        digitalWrite(3, HIGH);
        orServo.write(sSpeed);
        delay(100);
        digitalWrite(3, LOW);
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
        delay(50);
        updatecurrents(euler.x(), euler.y(), euler.z(), grav.x(), grav.y(), grav.z());
        delay(50);
      }
      digitalWrite(19, HIGH);
      digitalWrite(20, HIGH);
      delay(100);
      digitalWrite(19, LOW);
      digitalWrite(20, LOW);
      delay(100);
      digitalWrite(19, HIGH);
      digitalWrite(20, HIGH);
      delay(500);
      digitalWrite(19, LOW);
      digitalWrite(20, LOW);
      delay(100);
      digitalWrite(19, HIGH);
      digitalWrite(20, HIGH);
      delay(100);
      digitalWrite(19, LOW);
      digitalWrite(20, LOW);
      return;
    }
    else {
      sSpeed = 80;
      while (cGsZ < 9.65) {
        digitalWrite(3, HIGH);
        orServo.write(sSpeed);
        delay(100);
        digitalWrite(3, LOW);
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
        delay(50);
        updatecurrents(euler.x(), euler.y(), euler.z(), grav.x(), grav.y(), grav.z());
        delay(50);
      }
      digitalWrite(19, HIGH);
      digitalWrite(20, HIGH);
      delay(100);
      digitalWrite(19, LOW);
      digitalWrite(20, LOW);
      delay(100);
      digitalWrite(19, HIGH);
      digitalWrite(20, HIGH);
      delay(500);
      digitalWrite(19, LOW);
      digitalWrite(20, LOW);
      delay(100);
      digitalWrite(19, HIGH);
      digitalWrite(20, HIGH);
      delay(100);
      digitalWrite(19, LOW);
      digitalWrite(20, LOW);
      return;
    }
  }
};

Uprighting uprighter;
float iorx, iory, iorz, igsx, igsy, igsz;
float corx, cory, corz, cgsx, cgsy, cgsz;

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire.begin();
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  orServo.attach(4);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  uprighter = Uprighting();

  for (int i = 0; i < 20; i++) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    Serial.println(grav.z());
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  /* Display the floating point data */
  iorx = euler.x();
  iory = euler.y();
  iorz = euler.z();
  igsx = grav.x();
  igsy = grav.y();
  igsz = grav.z();
  uprighter.updateinitials(iorx, iory, iorz, igsx, igsy, igsz);

  digitalWrite(19, HIGH);
  digitalWrite(20, HIGH);
  delay(100);
  digitalWrite(19, LOW);
  digitalWrite(20, LOW);
  delay(100);
  digitalWrite(19, HIGH);
  digitalWrite(20, HIGH);
  delay(100);
  digitalWrite(19, LOW);
  digitalWrite(20, LOW);
  delay(100);
  digitalWrite(19, HIGH);
  digitalWrite(20, HIGH);
  delay(100);
  digitalWrite(19, LOW);
  digitalWrite(20, LOW);
  delay(100);
  uprighter.upright();
}


void loop(void)
{
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
