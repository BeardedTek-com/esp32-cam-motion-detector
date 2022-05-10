#include <LoopbackStream.h>
#include <PipedStream.h>

#include <StreamUtils.h>
#include <ArduinoJson.h>

/**
 * ESP32 Camera as Motion Detector
 * Based on https://eloquentarduino.com/projects/esp32-arduino-motion-detection
 * Tested using ESP32 wrover dev board w/ qqvga camera only
 * 
 * In Arduino IDE you need the following extra libraries installed:
 *   - EloquantArduino
 *   - PubSubClient
 */

// REQUIRED SETTINGS:

// Include WiFi and MQTT PubSub
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi Setup *REQUIRED*
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// MQTT Broker Setup *REQUIRED*
const char* mqtt_server = "YOUR_MQTT_BROKER";       // MQTT Broker IP/URL
const int mqtt_port = 1883;                         // MQTT Broker Port
const char* uniqueID = "OfficeMultiSensor";         // Unique Client Name

// Home Assistant Setup
// State/Attrib Topics
const char* haStateTopic = "homeassistant/sensor/office_multi_sensor/state";
const char* haAttribTopic = "homeassistant/sensor/office_multi_sensor/state";
//Motion Sensor
const char* haMotionConfigTopic = "homeassistant/binary_sensor/office_motion_sensor/config";
const char* haMotionName = "Office Motion Sensor";
// Temp Sensor
const char* haTempStateTopic = "homeassistant/sensor/office_multi_sensor/state";
const char* haTempConfigTopic = "homeassistant/sensor/office_temp_sensor/config";
const char* haTempName = "Office Temperature Sensor";
// RH Sensor
const char* haRHStateTopic = "homeassistant/sensor/office_multi_sensor/state";
const char* haRHConfigTopic = "homeassistant/sensor/office_humidity_sensor/config";
const char* haRHName = "Office Humidity Sensor";

//MQTT Topics to trigger action
const char* MQTTledEnable = "motion/office/settings/ledEnable";   // enable/disable LED
const char* MQTTheartbeat = "motion/office/settings/heartbeat";   // set heartbeat interval
const char* MQTTmotionenable = "motion/office/settings/enable";   // "enable" or "disable" motion detection
const char* MQTTmotionlockout = "motion/office/settings/lockout"; // time in ms to keep motion triggered
const char* MQTTstatus = "motion/office/settings/status";                  // JSON status message

// Support Variables
const int MQTTheartbeatMIN = 59999;                               // 1 minute
const int MQTTheartbeatMAX = 300001;                              // 5 minutes
const int MQTTmotionlockoutMIN = 999;
const int MQTTmotionlockoutMAX = 60001;

// MQTT Sub Topics
const char* subtopic = "motion/office/settings/+";

// LED Setup
#define LEDPIN 12

//DHT Setup
#define DHT_PIN 13
#define DHT_TYPE DHT11
bool tempEnabled = true;
bool humidityEnabled = true;
bool reportTempF = true;
unsigned long dhtWait = 60000; // in miliseconds


// NO NEED TO EDIT BELOW THIS LINE UNLESS YOU WANT TO CHANGE THINGS


#include "eloquent.h"
#include "eloquent/vision/motion/naive.h"
#include "DHT.h"


// uncomment based on your camera and resolution

//#include "eloquent/vision/camera/ov767x/gray/vga.h"
//#include "eloquent/vision/camera/ov767x/gray/qvga.h"
//#include "eloquent/vision/camera/ov767x/gray/qqvga.h"
//#include "eloquent/vision/camera/esp32/aithinker/gray/vga.h"
//#include "eloquent/vision/camera/esp32/aithinker/gray/qvga.h"
//#include "eloquent/vision/camera/esp32/aithinker/gray/qqvga.h"
//#include "eloquent/vision/camera/esp32/wrover/gray/vga.h"
//#include "eloquent/vision/camera/esp32/wrover/gray/qvga.h"
#include "eloquent/vision/camera/esp32/wrover/gray/qqvga.h"
//#include "eloquent/vision/camera/esp32/eye/gray/vga.h"
//#include "eloquent/vision/camera/esp32/eye/gray/qvga.h"
//#include "eloquent/vision/camera/esp32/eye/gray/qqvga.h"
//#include "eloquent/vision/camera/esp32/m5/gray/vga.h"
//#include "eloquent/vision/camera/esp32/m5/gray/qvga.h"
//#include "eloquent/vision/camera/esp32/m5/gray/qqvga.h"
//#include "eloquent/vision/camera/esp32/m5wide/gray/vga.h"
//#include "eloquent/vision/camera/esp32/m5wide/gray/qvga.h"
//#include "eloquent/vision/camera/esp32/m5wide/gray/qqvga.h"



// Setup Detection Variables
bool ledEnable = true;
bool motionEnabled = true;
bool detection = false;
bool motionEvent = false;
unsigned long motionLockoutTime = 5000;
unsigned long detectionStartTime = millis();
unsigned long lastHeartbeat = millis();
bool initialHeartbeat = false;
unsigned long heartbeatInterval = 60000;                          // Heartbeat interval in ms (60000 = 60 seconds)

// Setup DHT
DHT dht(DHT_PIN, DHT_TYPE);
unsigned long dhtLastRead = millis();
float dhtF = 0;
float dhtT = 0;
float dhtH = 0;
float dhtHIF = 0;
float dhtHIC = 0;

// Setup Debugging
bool debug = false;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

#define THUMB_WIDTH 32
#define THUMB_HEIGHT 24


Eloquent::Vision::Motion::Naive<THUMB_WIDTH, THUMB_HEIGHT> detector;

void setup() {
    dht.begin();
    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, LOW);
    delay(4000);
    Serial.begin(115200);

    // turn on high freq for fast streaming speed
    camera.setHighFreq();

    if (!camera.begin())
        eloquent::abort(Serial, "Camera init error");
    if (debug){
      Serial.println("Camera init OK");
    }

    // wait for at least 10 frames to be processed before starting to detect
    // motion (false triggers at start)
    // then, when motion is detected, don't trigger for the next 10 frames
    detector.startSinceFrameNumber(10);
    detector.debounceMotionTriggerEvery(10);

    // or, in one call
    detector.throttle(10);

    // trigger motion when at least 10% of pixels change intensity by
    // at least 15 out of 255
    detector.setPixelChangesThreshold(0.1);
    detector.setIntensityChangeThreshold(15);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
const char* True = "true";
const char* False = "false";

void publishHATempConfig(){
/*   
*    Publish sensor status to MQTT
*/
  // Create Buffer
  char buffer[256]; // Used for all publish commands

  // Temp Sensor
  // Build JSON
  StaticJsonDocument<1000> haTemp;

  // Add Values
  haTemp["device_class"] = "temperature";
  haTemp["name"] = haTempName;
  haTemp["state_topic"] = haStateTopic;
  haTemp["value_template"] = "{{value_json.temperature}}";
  haTemp["json_attributes_topic"] = haAttribTopic;

  // Publish haTemp Config
  serializeJsonPretty(haTemp, buffer);
  client.publish(haTempConfigTopic,buffer,true);
}
void publishHARHConfig(){
/*   
*    Publish sensor status to MQTT
*/
  // Create Buffer
  char buffer[1000]; // Used for all publish commands

  // Humidity Sensor
  // Build JSON
  StaticJsonDocument<1000> haRH;

  // Add Values
  haRH["device_class"] = "humidity";
  haRH["name"] = haRHName;
  haRH["state_topic"] = haStateTopic;
  haRH["value_template"] = "{{value_json.humidity}}";
  haRH["json_attributes_topic"] = haAttribTopic;

  // Publish haRH Config
  serializeJsonPretty(haRH, buffer);
  client.publish(haRHConfigTopic,buffer,true);
}

void publishHAMotionConfig(){
/*   
*    Publish sensor status to MQTT
*/
  // Create Buffer
  char buffer[1000]; // Used for all publish commands


  // Motion Sensor
  // Build JSON
  StaticJsonDocument<1000> haMotion;
  
  // Add Values
  haMotion["device_class"] = "motion";
  haMotion["name"] = haMotionName;
  haMotion["state_topic"] = haStateTopic;
  haMotion["value_template"] = "{{value_json.motion}}";
  haMotion["json_attributes_topic"] = haAttribTopic;
  
  //Publish haMotion Config
  serializeJsonPretty(haMotion, buffer);
  client.publish(haMotionConfigTopic,buffer,true);
}


void publishState(){
/*   
 *    Publish sensor status to MQTT
 */
  Serial.println("Executing publishState()");
  // Build JSON
  // Allocate memory for the document
  StaticJsonDocument<1000> haState;
  
  // Add some values
  
  haState["heartbeat"] = lastHeartbeat;
  haState["heartbeatInterval"] = heartbeatInterval;
  haState["motionLockoutTime"] = motionLockoutTime;
  haState["ledEnable"] = ledEnable?"Enabled":"Disabled";
  haState["motionEnabled"]= motionEnabled?"Enabled":"Disabled";
  
  if (reportTempF){
    haState["temperature"] = dhtF;
    haState["HeatIndex"] = dhtHIF;
    haState["units"] = "°F";
  }
  else{
    haState["temperatureCelsius"] = dhtT;
    haState["HeatIndexCelsius"] = dhtHIC;
    haState["units"] = "°C";
  }
  haState["humidity"] = dhtH;
  haState["motion"] = detection?"ON":"OFF";
  
  // Create Buffer
  Serial.println("creating buffer");
  char buffer[1000];
  Serial.println("buffer created");
  serializeJsonPretty(haState, buffer);
  Serial.print("Publish State: ");
  Serial.println(client.publish(haStateTopic,buffer)?"OK":"Error");
  Serial.println(buffer);
}

void callback(char* topic, byte* message, unsigned int length) {
  if (debug){
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
  }
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    if (debug){
      Serial.print((char)message[i]);
    }
    messageTemp += (char)message[i];
  }
  if (debug){
    Serial.println();
  }

  // MQTT Actions

  // LED Enable/Disable
  if (String(topic) == MQTTledEnable){
    if (messageTemp == "true"){
      if (ledEnable == false){
        if (debug){
          Serial.println("LED disabled");
        }
        ledEnable = true;
      }
    if (messageTemp == "false"){
      if (ledEnable == true){
        ledEnable = false;
      }
    }
    }
  }

  
  // Heartbeat Interval
  if (String(topic) == MQTTheartbeat){
    unsigned long newHeartbeatInterval = strtoul(messageTemp.c_str(),NULL,0);
    if (debug){
      Serial.print("Received new Heartbeat Interval: ");
      Serial.println(newHeartbeatInterval);
    }
    if ((newHeartbeatInterval-MQTTheartbeatMIN)*(newHeartbeatInterval-MQTTheartbeatMAX)){
      heartbeatInterval = newHeartbeatInterval;
      if (debug){
        Serial.print("Heartbeat Interval Set To :");
        Serial.print(newHeartbeatInterval);
        Serial.println(" via MQTT");
        client.publish(heartbeatIntervalPub,messageTemp.c_str());
      }
    }
    else{
      if (debug){
        Serial.print("New Heartbeat Interval Must be > ");
        Serial.println(MQTTheartbeatMIN);
        Serial.print("Heartbeat Interval Remains: ");
        Serial.println(heartbeatInterval);
      }
    }    
  }

  
  // Motion Lockout Time
  if (String(topic) == MQTTmotionlockout){
    unsigned long newLockoutTime = strtoul(messageTemp.c_str(),NULL,0);
    if (debug){
      Serial.print("Received new lockout: ");
      Serial.println(newLockoutTime);
    }
    if ((newLockoutTime-MQTTmotionlockoutMIN)*(newLockoutTime-MQTTmotionlockoutMAX)){
      motionLockoutTime = newLockoutTime;
      if (debug){
        Serial.print("Motion Lockout Time Set To :");
        Serial.print(newLockoutTime);
        Serial.println(" via MQTT");
        client.publish(lockoutTimePub,messageTemp.c_str());
      }
    }
    else{
      if (debug){
        Serial.print("New Lockout Time Must be > ");
        Serial.println(MQTTmotionlockoutMIN);
        Serial.print("Lockout Time Remains: ");
        Serial.println(motionLockoutTime);
      }
    }    
  }

  // Motion Detection Enable/Disable
  if (String(topic) == MQTTmotionenable){
    if (messageTemp == "enable"){
      if (motionEnabled == false){
        motionEnabled = true;
        if (debug){
          Serial.println("Motion Sensor Enabled");
          client.publish(MQTTmotionenable, "Enabled");
        }
      }
    }
    if (messageTemp == "disable"){
      if (motionEnabled == true){
        motionEnabled = false;
        if (debug){
          Serial.println("MotionSensor Disabled");
          client.publish(MQTTmotionenable, "Disabled");
        }
      }
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(uniqueID)) {
      client.setBufferSize(1024);
      Serial.print ("connected as");
      Serial.println(uniqueID);
      // Subscribe
      client.subscribe(subtopic);
      Serial.print("Subscribed to ");
      Serial.println(subtopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void detect_motion(){
    if (camera.capture()){
        // perform motion detection on resized image for fast detection
        camera.image.resize<THUMB_WIDTH, THUMB_HEIGHT>();
        //camera.image.printAsJsonTo(Serial);
        detector.update(camera.image);
    
        // if motion is detected, print coordinates to serial in JSON format
        if (detector.isMotionDetected()) {
            detection = true;
        }
        else{
            detection = false;
        }
        // release memory
        camera.free();
    }
    else {
      delay(1000);
      detect_motion();
    }
}

int readDHT(){
  if (debug){
    Serial.println();
    Serial.print(millis());
    Serial.println(": Reading DHT11");
  }
  dhtH = dht.readHumidity();
  dhtT = dht.readTemperature();
  dhtF = dht.readTemperature(true);
  if (isnan(dhtH) || isnan(dhtT) || isnan(dhtF)){
    Serial.print("ERROR: failed to read DHT11 on Pin #");
    Serial.println(DHT_PIN);
  }
  else{
    dhtLastRead = millis();
    dhtHIF = dht.computeHeatIndex(dhtF,dhtH);
    dhtHIC = dht.computeHeatIndex(dhtT,dhtH);
    if (debug){
      Serial.println("DHT Sensor Readings:");
      Serial.print("Humidity: ");
      Serial.print(dhtH);
      Serial.println("%RH");
      Serial.print("Temp: ");
      Serial.print(dhtF);
      Serial.print("°F ");
      Serial.print(dhtT);
      Serial.println("°C");
      Serial.print("Heat Index: ");
      Serial.print(dhtHIF);
      Serial.print("°F ");
      Serial.print(dhtHIC);
      Serial.print("°C");
    }
    return 0;
  }
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();                                // Listen to MQTT Broker

  // Get current execution time
  unsigned long currentTime = millis();         // Used for HEARTBEAT and MOTION DETECTION

  // BEGIN HEARTBEAT
  if (!initialHeartbeat){
    // This allows us to do initial setup of the sensor upon the first heartbeat...
    // Publish Home Assistant MQTT Config
  }
  else{
    // Send First Heartbeat
    unsigned long timeoutHeartbeat = lastHeartbeat + heartbeatInterval;

    if (currentTime > timeoutHeartbeat){
      if (debug){
        Serial.print("Heartbeat: ");
        Serial.println(currentTime);
      }
      publishState();
      lastHeartbeat = millis();
    }
  }
  if (!initialHeartbeat){
    if (debug){
      Serial.print("Heartbeat: ");
      Serial.println(currentTime);
    }
    
    // Publish Home Assistant Configs
    publishHATempConfig();
    publishHARHConfig();
    publishHAMotionConfig();

    // Publish Initial State
    publishState();
    
    
    lastHeartbeat = millis();
    initialHeartbeat = true;
  }
  // END HEARTBEAT
  
  // BEGIN MOTION DETECTION
  if (motionEnabled){
    if (motionEvent){
      unsigned long timeout = detectionStartTime + motionLockoutTime;
      if (currentTime > timeout){
        detect_motion();
        if (!detection){
          publishState();
          digitalWrite(LEDPIN, LOW);            // Turn off LED
          motionEvent = false;
        }
      }
    }
    else{
      detect_motion();
      if (detection){
        publishState();
        digitalWrite(LEDPIN, HIGH);               // Turn on LED
        detectionStartTime = millis();
        motionEvent = true;
      }
    }
  }
  // END MOTION DETECTION

  // BEGIN DHT
  if (tempEnabled || humidityEnabled){
    unsigned long dhtTimeout = dhtLastRead + dhtWait;
    if (currentTime >= dhtTimeout || dhtLastRead == 0){
      readDHT();
    }
  }
  // END DHT
}
