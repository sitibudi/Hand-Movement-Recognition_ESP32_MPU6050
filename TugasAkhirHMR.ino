#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <DFRobot_MLX90614.h>
#include <Wire.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>

// #include <Arduino_FreeRTOS.h>
// #include <gesture_class_ESP32_dataForwarder_inferencing.h>
// #include <test-accelero_inferencing.h>
#include <HMR1_inferencing.h>
#include <Adafruit_NeoPixel.h>



#define FREQUENCY_HZ        60
#define INTERVAL_MS         (100 / (FREQUENCY_HZ + 1))
#define PIN        27 // neopixel
#define NUMPIXELS 1

DFRobot_MLX90614_I2C MLX;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// objeto da classe Adafruit_MPU6050
Adafruit_MPU6050 mpu;
 sensors_event_t a, g, temp;

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;

static unsigned long last_interval_ms = 0;

int current,current1,current2;
const char* ssid = "bro bor";
const char* password = "9434276267";

bool cw = false;
bool merah = false;
bool idle= false;
int state;
// HEART BEAT 
MAX30105 particleSensor;

 const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
int lastBeat = 0; //Time at which the last beat occurred


int beatsPerMinute;
int beatAvg;

int irValue;

const char* mqtt_server = "broker.mqtt-dashboard.com";  
WiFiClient espClient22;
PubSubClient client(espClient22);
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

float Temp;

//=====================NTP server time==========================================
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP,"pool.ntp.org", 25200);  // GMT(+7) * 3600 = gmtOffset_sec 
// const char* ntpServer = "pool.ntp.org";
// ===========================================================================






// define two tasks for Blink & AnalogRead
void Task0( void *pvParameters );
void Task1( void *pvParameters );

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
   Serial.begin(115200);
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
 pixels.setBrightness(10);
 pixels.clear();
  // ----------------------WIFI SETUP-----------------------------
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(F("WiFi connected"));
// ------------------------------------------------------------
  // firebaseSetup();
 particleSensor.begin();
   particleSensor.setup(); 
   mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
 
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    Task0
    ,  "Task0"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  0 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  1);

  xTaskCreatePinnedToCore(
    Task1
    ,  "Task1"
    ,  1024  // Stack size
    ,  NULL
    ,  0 // Priority
    ,  NULL 
    ,  1);

    client.setServer(mqtt_server, 1883);
  // client.setCallback(callback);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()

{
 
  //  mpu.getEvent(&a, &g, &temp);
    if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();
    
    mpu.getEvent(&a, &g, &temp);

    features[feature_ix++] = a.acceleration.x;
    features[feature_ix++] = a.acceleration.y;
    features[feature_ix++] = a.acceleration.z;

    if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      // Serial.println("Running the inference...");
      signal_t signal;
      ei_impulse_result_t result;
      int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      if (err != 0) {
        // ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
      }

      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);

      if (res != 0) return;

      // ei_printf("Predictions ");
      // ei_printf("(DSP: %d ms., Classification: %d ms.)",
      //           result.timing.dsp, result.timing.classification);
      // ei_printf(": \n");
    
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        // ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        
        if (result.classification[ix].value > 0.6) {
          if (result.classification[ix].label == "idle1")
          {
              // idle = true;
              state = 1;
            pixels.setPixelColor(0, pixels.Color(0, 255, 0));
            pixels.show();

          } 
          else if (result.classification[ix].label == "circleCW")
          {
              // cw = true;
              state = 2;
            pixels.setPixelColor(0, pixels.Color(0, 0, 255));
            pixels.show();
          } 
          // else if (result.classification[ix].label == "idle2")
          // {
      
          // //   pixels.setPixelColor(0, pixels.Color(255, 0, 0));
          // //   pixels.show();
          // //   Serial.println("move++");

          // } 
          else if (result.anomaly >= 6)
          {
            state = 3;
             
            pixels.setPixelColor(0, pixels.Color(255, 0, 0));
            pixels.show();
            
          }
        }
      }
      feature_ix = 0;
      

    }
    

  }

   if (millis() - current >= 200){
          current = millis();
         irValue = particleSensor.getIR();
          Temp = MLX.getObjectTempCelsius();
          

          client.connect("ESP32Pico");
              snprintf (msg, MSG_BUFFER_SIZE,"%d", state);
              client.publish("mqtt_perintah",msg);

    // client.connect("ESP8266Client22");
    // snprintf (msg, MSG_BUFFER_SIZE, "circle:%ld", cw);
    // client.publish("mqtt_perintah",msg);
    // merah = false;
    // cw = false;
        
   }

   if (millis() - current1 >= 5000){
        current1 = millis();

        client.connect("ESP32Pico");
        snprintf (msg, MSG_BUFFER_SIZE, "%.2f",Temp);
        client.publish("mqtt_temp",msg);


   }

  if (millis() - current2 >2000){
    current2 = millis();
        client.connect("ESP32Pico");
        snprintf (msg, MSG_BUFFER_SIZE, "%d", beatAvg);
        client.publish("mqtt_BPM",msg);
  }

  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Task0(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
 
  
  
  for (;;) // A Task shall never return or exit.
  { 
   
  //  Serial.print(a.acceleration.x); Serial.print("|");
  //  Serial.print(a.acceleration.y); Serial.print("|");
  //  Serial.println(a.acceleration.z);vTaskDelay(10);
  //   vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
 
  }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void Task1(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
 


while(1)
  // for (;;)
  {
      
  if (checkForBeat(irValue) == true)
  {
      // if (millis() - current1 >= 100){
      //     current1 = millis();
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
      }
   
  
  // Serial.print("IR=");
  // Serial.print(irValue);
  // Serial.print(", BPM=");
  // Serial.print(beatsPerMinute);
  // Serial.print(", Avg BPM=");
  // Serial.println(beatAvg-40);
 
    
        
  }
 
    // Serial.println(cw);
    // Serial.println(merah);
    // Serial.println(idle);
    // vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
}
