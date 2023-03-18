//=====================================================LIBRARY=======================================================================
#include "Wire.h"
#include <MPU6050_light.h>
#include "FS.h"
#include <SPI.h>
#include <SD.h>
#include <stdint.h>
#define Pins_Arduino_h
//======================================================DONE=========================================================================


//==================================================INITIALIZATION=======================================================================
MPU6050 mpu(Wire);
unsigned long timer = 0;


int a; int b;
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


void datalog(){
  File myFile = SD.open("/test.txt", FILE_APPEND);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.print(String(a));
    myFile.print("   ");
    myFile.println(String(b));
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
 Serial.begin(9600);
  // MPU6050 
  Wire.begin();
   
   // SD card module
    Serial.print("Initializing SD card...");

  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
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
    a = random(0,11);
   b = random (20,26);
  datalog();
    
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


