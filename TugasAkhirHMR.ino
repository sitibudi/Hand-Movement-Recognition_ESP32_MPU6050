//=====================================================LIBRARY=======================================================================
#include "Wire.h"
#include <MPU6050_light.h>

//======================================================DONE=========================================================================


//==================================================INITIALIZATION=======================================================================
MPU6050 mpu(Wire);
unsigned long timer = 0;

//======================================================DONE=========================================================================

//====================================================FUNCTION=======================================================================

// MPU 6050 SETUP
void 6050Setup(){
   byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  
}

//======================================================DONE=========================================================================


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
 Serial.begin(9600);
  Wire.begin();
//  pinMode();
//  pinMode();
//  pinMode();
//  pinMode();
  6050Setup()
 

}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    mpu.update();
    
    if((millis()-timer)>10){ // print data every 10ms
  Serial.print("X : ");
  Serial.print(mpu.getAngleX());
  Serial.print("\tY : ");
  Serial.print(mpu.getAngleY());
  Serial.print("\tZ : ");
  Serial.println(mpu.getAngleZ());
  
  timer = millis();  
  }
 if (mpu.getAngleY() > 70){
    Serial.println("get 70 angle");
    delay(1000);
  }

}
