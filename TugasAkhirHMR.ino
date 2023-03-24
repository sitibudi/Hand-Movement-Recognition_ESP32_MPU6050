/* PINOUT ON ESP32-PICO D4
  SDA = GPIO 21
  SCL = GPIO 22
  SPI_MISO = HSPIQ (GPIO 17)
  SPI_MOSI = HSPID (GPIO 8)
  CS   = HSPICS0 (GPIO 10)
  CLK = HSPICLK (GPIO 6)


  SPI_MISO = HSPIQ(GPIO12)
SPI_MOSI = HSPID(GPIO13)
SPI_CLK = HSPICLK(GPIO14)
SPI_CS = HSPICS0(GPIO15)

MOSI: 23
MISO: 19
SCK: 18
SS: 5

*/



//=====================================================LIBRARY=======================================================================
#include <DFRobot_MLX90614.h>
#include <DFRobot_MAX30102.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "FS.h"
#include <SPI.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
 /*
Macro definition options in sensor configuration
sampleAverage: SAMPLEAVG_1 SAMPLEAVG_2 SAMPLEAVG_4
               SAMPLEAVG_8 SAMPLEAVG_16 SAMPLEAVG_32
ledMode:       MODE_REDONLY  MODE_RED_IR  MODE_MULTILED
sampleRate:    PULSEWIDTH_69 PULSEWIDTH_118 PULSEWIDTH_215 PULSEWIDTH_411
pulseWidth:    SAMPLERATE_50 SAMPLERATE_100 SAMPLERATE_200 SAMPLERATE_400
               SAMPLERATE_800 SAMPLERATE_1000 SAMPLERATE_1600 SAMPLERATE_3200
adcRange:      ADCRANGE_2048 ADCRANGE_4096 ADCRANGE_8192 ADCRANGE_16384
*/
//======================================================DONE=========================================================================


//==================================================INITIALIZATION=======================================================================

#define PIN        27 // neopixel
#define NUMPIXELS 1

// define object (instantiate an object to drive our sensor)
// Temp Sensor
DFRobot_MLX90614_I2C MLX;   // 

// Heart Beat Sensor
DFRobot_MAX30102 particleSensor; 
int32_t SPO2; //SPO2
int8_t SPO2Valid; //Flag to display if SPO2 calculation is valid
int32_t heartRate; //Heart-rate
int8_t heartRateValid; //Flag to display if heart-rate calculation is valid 


// IMU sensor
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// sdCard module 
File myFile;

// Neopixel setup
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// millis 
int current, pre_current;
int current1, pre_current1;


// Global variable
float Temp;
int pixelnum;
bool state;

//======================================================DONE=========================================================================

//====================================================FUNCTION=======================================================================
void datalog(){
  File myFile = SD.open("/test.csv", FILE_APPEND);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.print(String(current));
    myFile.print(",");
    myFile.print(String(a.acceleration.x));
    myFile.print(",");
    myFile.print(String(a.acceleration.y));
    myFile.print(",");
    myFile.print(String(a.acceleration.z));

    myFile.print(",");
    myFile.print(String(g.gyro.x));
    myFile.print(",");
    myFile.print(String(g.gyro.y));
    myFile.print(",");
    myFile.println(String(g.gyro.z));
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

//======================================================DONE=========================================================================


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
 Serial.begin(115200);
//  particleSensor.begin();
//  sensor.begin();
 pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
 pixels.setBrightness(10);
 pixels.clear();
 state = true;

  // initialize the sensor
  while( !particleSensor.begin() ){
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");

  while( NO_ERR != MLX.begin() ){
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");


  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // if (!SD.begin()) {
  //   Serial.println("initialization failed!");
  //   return;
  // }
  // Serial.println("initialization done.");

  //  File myFile = SD.open("/test.csv", FILE_WRITE);
  // if (myFile) {
  //   Serial.print("Writing to test.txt...");
  //   myFile.print("Time");
  //   myFile.print(",");
  //   myFile.print("angle x");
  //   myFile.print(",");
  //   myFile.print("angle y");
  //   myFile.print(",");
  //   myFile.print("angle z");
  //   myFile.print(",");

  //   myFile.print("gyro x");
  //   myFile.print(",");
  //   myFile.print("gyro y");
  //   myFile.print(",");
  //   myFile.println("gyro z");
   
    
  //     myFile.close();
  //   Serial.println("done.");
  // } 
  // else {
  //    Serial.print("error");
  // }



   // RANGE 
//  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
//  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
//  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  // DEGREE
//  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
//  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);

  // BANDWIDTH
//  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
//  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
//  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
//  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
//  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.println("");

  particleSensor.sensorConfiguration(/*ledBrightness=*/100, /*sampleAverage=*/SAMPLEAVG_1, \
                                  /*ledMode=*/MODE_MULTILED, /*sampleRate=*/SAMPLERATE_50, \
                                  /*pulseWidth=*/PULSEWIDTH_411, /*adcRange=*/ADCRANGE_16384);
  
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
   // MLX906 
current = millis();
current1 = millis();

  if(current - pre_current > 50){
    Temp = MLX.getObjectTempCelsius();
    mpu.getEvent(&a, &g, &temp);
    
  //    Serial.print("AccelX:");
  // Serial.print(a.acceleration.x);
  // Serial.print(",");
  // Serial.print("AccelY:");
  // Serial.print(a.acceleration.y);
  // Serial.print(",");
  // Serial.print("AccelZ:");
  // Serial.print(a.acceleration.z);
  // Serial.print(", ");
  // Serial.print("GyroX:");
  // Serial.print(g.gyro.x);
  // Serial.print(",");
  // Serial.print("GyroY:");
  // Serial.print(g.gyro.y);
  // Serial.print(",");
  // Serial.print("GyroZ:");
  // Serial.print(g.gyro.z);
  // Serial.println("");
    
     
//    Serial.print("temp : ");  Serial.print(Temp);  Serial.print("|| ");
// //  Serial.print("temp : ");  Serial.print(random(25,38));  Serial.print("|| ");
//    Serial.print("beat:"); Serial.print(particleSensor.getIR());  Serial.print("|| ");
// //  Serial.print("beat:"); Serial.print(random(0,200));  Serial.print("|| ");
  // Serial.print("AccelX:"); Serial.print(a.acceleration.x);Serial.print("|");
  // Serial.print("AccelY:"); Serial.print(a.acceleration.y);Serial.print("|");
  // Serial.print("AccelZ:"); Serial.print(a.acceleration.z);Serial.println("|");
  Serial.print(a.acceleration.x);Serial.print(",");
  Serial.print(a.acceleration.y);Serial.print(",");
  Serial.println(a.acceleration.z);
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
  pixels.show();
  pre_current = current;
  // datalog();
  }

    //  if (current1 - pre_current1 >= 500){
 
    //   }
  

  
 
   
  
 
 
// end loop
}
