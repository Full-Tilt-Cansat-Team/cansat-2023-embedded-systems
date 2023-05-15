#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h> 
#include <utility/imumaths.h> 
#include "Adafruit_BMP3XX.h"
#include <Servo.h>
#include <SoftwareSerial.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
Adafruit_BMP3XX bmp;

#define SEALEVELPRESSURE_HPA (1013.25)

float initialAltitude;

float lastInterval = 0;
float currentInterval = 0;

SoftwareSerial xbee(1, 0);

//M5
Servo M5;

void setup() {
  delay(100);
  // put your setup code here, to run once:
  Serial.begin(115200);

  //while (!Serial);

  Wire1.setSDA(14);
  Wire1.setSCL(15);

  //Attatch servos
  M5.attach(20);
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);
  
  Serial.println("Starting!");

  /*
  if (!bno.begin()) {
    while (true) {
      Serial.println("BNO Error");
      delay(1000);
    }
  }*/
  

  //Setup openlog serial
  Serial2.setRX(9);
  Serial2.setTX(8);
  Serial2.begin(9600);

  //Write high poweron
  Serial2.println("Power cycle");

  xbee.begin(9600);

  if (!bmp.begin_I2C(0x77, &Wire1)) {
    while (true) {
      Serial.println("BMP Error");
      delay(1000);
    }
  }

  delay(3000);

  // Set initial altitude
  initialAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  initialAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

float direction = 1;

void loop() {
  currentInterval = millis();
  M5.write(360 + direction);
  if (currentInterval - lastInterval >= 1000) {
    sensors_event_t orientationData;

    //bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    
    xbee.print("Approx. Altitude = ");
    xbee.println(bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialAltitude); 

    Serial.print("Approx. Altitude = ");
    Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialAltitude);

    /*
    Serial2.print("Accel x = ");
    Serial2.println(orientationData.acceleration.x);
    */
    
    Serial2.print("Approx. Altitude = ");
    Serial2.println(bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialAltitude); 
    lastInterval = currentInterval;

    //direction += 25;
  }
}
