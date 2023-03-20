//=====================================================LIBRARY=======================================================================
#include <DFRobot_MLX90614.h>
#include <DFRobot_MAX30102.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
//======================================================DONE=========================================================================


//==================================================INITIALIZATION=======================================================================
// define object (instantiate an object to drive our sensor)
// Temp Sensor
DFRobot_MLX90614_I2C sensor;   // 

// Heart Beat Sensor
DFRobot_MAX30102 particleSensor; 

// IMU sensor
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;


// millis 
int current, pre_current;
int current1, pre_current1;


// Global variable


//======================================================DONE=========================================================================

//====================================================FUNCTION=======================================================================


//======================================================DONE=========================================================================


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
 Serial.begin(115200);

  // initialize the sensor
  while( !particleSensor.begin() and !sensor.begin() ){
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

  particleSensor.sensorConfiguration(/*ledBrightness=*/60, /*sampleAverage=*/SAMPLEAVG_8, \
                                  /*ledMode=*/MODE_MULTILED, /*sampleRate=*/SAMPLERATE_400, \
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
    Temp = sensor.getObjectTempCelsius();
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
    
     
  Serial.print("temp : ");  Serial.print(objectTemp);  Serial.print("|| ");
  Serial.print("beat:"); Serial.print(particleSensor.getIR());  Serial.print("|| ");
  Serial.print("AccelX:"); Serial.print(a.acceleration.x);Serial.print("|");
  Serial.print("AccelY:"); Serial.print(a.acceleration.y);Serial.print("|");
  Serial.print("AccelZ:"); Serial.print(a.acceleration.z);Serial.println("|");
  pre_current = current;
  }

  // if (current1 - pre_current1 > 50){
  //   pre_current1 = current1;
   

  // }

 

}
