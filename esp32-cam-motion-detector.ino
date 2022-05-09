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
const char* mqtt_server = "YOUR_MQTT_BROKER";           // MQTT Broker IP/URL
const int mqtt_port = 1883;                         // MQTT Broker Port

// Home Assistant Setup - https://www.home-assistant.io/docs/mqtt/discovery/
const char* haConfigTopic = "homeassistant/binary_sensor/office_motion_sensor/config";
const char* haStateTopic = "homeassistant/binary_sensor/office_motion_sensor/state";
const char* sensorName = "Office Motion Sensor";
const char* uniqueID = "office_motion_sensor";

// MQTT Pub Topics **REQUIRED**
const char* onlinetopic = "motion/office/online";                 // Tells us it's online. (heartbeat)
const char* motiontopic = "motion/office/status";                 // Tells us if there's motion or not
const char* lockoutTimePub = "motion/office/newlockout";          // Tells us new Motion Lockout time when set
const char* heartbeatIntervalPub = "motion/office/newHeartbeat";  // Tells us new Heartbeat Interval when set

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

// LED Pin
#define LEDPIN 12

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



/* NO NEED TO EDIT BELOW THIS LINE UNLESS YOU WANT TO CHANGE THINGS */


#include "eloquent.h"
#include "eloquent/vision/motion/naive.h"
#include <ArduinoJson.h>





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


WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

#define THUMB_WIDTH 32
#define THUMB_HEIGHT 24


Eloquent::Vision::Motion::Naive<THUMB_WIDTH, THUMB_HEIGHT> detector;


void setup() {
  
    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, LOW);
    delay(4000);
    Serial.begin(115200);

    // turn on high freq for fast streaming speed
    camera.setHighFreq();

    if (!camera.begin())
        eloquent::abort(Serial, "Camera init error");

    Serial.println("Camera init OK");

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

void publishHAConfig(){
/*   
*    Publish sensor status to MQTT
*/
  // Build JSON
  // Allocate memory for the document
  StaticJsonDocument<256> doc;
  
  // Add some values
  doc["device_class"] = "motion";
  doc["name"] = sensorName;
  doc["unique_id"] = uniqueID;
  doc["state_topic"] = haStateTopic;
  doc["value_template"] = "{{value_json.status}}";
  
  // Create Buffer
  char buffer[300];
  
  size_t n = serializeJsonPretty(doc, buffer);
  Serial.println(buffer);
  client.publish(haConfigTopic,buffer, true);
}

void publishState(){
/*   
 *    Publish sensor status to MQTT
 */
 
  // Build JSON
  // Allocate memory for the document
  StaticJsonDocument<256> doc;
  
  // Add some values
  
  doc["status"] = detection?"ON":"OFF";
  doc["heartbeat"] = lastHeartbeat;
  doc["heartbeatInterval"] = heartbeatInterval;
  doc["motionLockoutTime"] = motionLockoutTime;
  doc["motionEnabled"]= motionEnabled?"Enabled":"Disabled";
  doc["ledEnable"] = ledEnable?"Enabled":"Disabled";
  
  // Create Buffer
  char buffer[300];
  
  size_t n = serializeJsonPretty(doc, buffer);
  Serial.println(buffer);
  client.publish(haStateTopic,buffer);
}


void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // MQTT Actions

  // LED Enable/Disable
  if (String(topic) == MQTTledEnable){
    if (messageTemp == "true"){
      if (ledEnable == false){
        Serial.println("LED disabled");
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
    Serial.print("Received new Heartbeat Interval: ");
    Serial.println(newHeartbeatInterval);
    if ((newHeartbeatInterval-MQTTheartbeatMIN)*(newHeartbeatInterval-MQTTheartbeatMAX)){
      heartbeatInterval = newHeartbeatInterval;
      Serial.print("Heartbeat Interval Set To :");
      Serial.print(newHeartbeatInterval);
      Serial.println(" via MQTT");
      client.publish(heartbeatIntervalPub,messageTemp.c_str());
    }
    else{
      Serial.print("New Heartbeat Interval Must be > ");
      Serial.println(MQTTheartbeatMIN);
      Serial.print("Heartbeat Interval Remains: ");
      Serial.println(heartbeatInterval);
    }    
  }

  
  // Motion Lockout Time
  if (String(topic) == MQTTmotionlockout){
    unsigned long newLockoutTime = strtoul(messageTemp.c_str(),NULL,0);
    Serial.print("Received new lockout: ");
    Serial.println(newLockoutTime);
    if ((newLockoutTime-MQTTmotionlockoutMIN)*(newLockoutTime-MQTTmotionlockoutMAX)){
      motionLockoutTime = newLockoutTime;
      Serial.print("Motion Lockout Time Set To :");
      Serial.print(newLockoutTime);
      Serial.println(" via MQTT");
      client.publish(lockoutTimePub,messageTemp.c_str());
    }
    else{
      Serial.print("New Lockout Time Must be > ");
      Serial.println(MQTTmotionlockoutMIN);
      Serial.print("Lockout Time Remains: ");
      Serial.println(motionLockoutTime);
    }    
  }

  // Motion Detection Enable/Disable
  if (String(topic) == MQTTmotionenable){
    if (messageTemp == "enable"){
      if (motionEnabled == false){
        motionEnabled = true;
        Serial.println("Motion Sensor Enabled");
        client.publish(MQTTmotionenable, "Enabled");
      }
    }
    if (messageTemp == "disable"){
      if (motionEnabled == true){
        motionEnabled = false;
        Serial.println("MotionSensor Disabled");
        client.publish(MQTTmotionenable, "Disabled");
      }
    }
  }
  // Status Message with JSON payload
  if (String(topic) == MQTTstatus){
    if (messageTemp == "get"){
      publishState();
    }
  }
  
  
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe(subtopic);
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
    publishHAConfig();
  }
  else{
    // Send First Heartbeat
    unsigned long timeoutHeartbeat = lastHeartbeat + heartbeatInterval;
    if (currentTime > timeoutHeartbeat){
      Serial.print("Heartbeat: ");
      Serial.println(currentTime);
      publishState();
      lastHeartbeat = millis();
    }
  }
  if (!initialHeartbeat){
    Serial.print("Heartbeat: ");
    Serial.println(currentTime);
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
          Serial.print("Publishing to ");
          Serial.println(haStateTopic);
          publishState();
          digitalWrite(LEDPIN, LOW);            // Turn off LED
          motionEvent = false;
        }
      }
    }
    else{
      detect_motion();
      if (detection){
        Serial.print("Publishing to ");
        Serial.println(haStateTopic);
        publishState();
        digitalWrite(LEDPIN, HIGH);               // Turn on LED
        detectionStartTime = millis();
        motionEvent = true;
      }
    }
  }
  // END MOTION DETECTION
}
