/*
 * 1-channel switch firmware for WiFi Curtain/Blinds module based on ESP8266:  
 * https://s.click.aliexpress.com/e/_d7tJGEP
 * 
 * For more information on wiring, flashing, etc. see https://ibn.by/2020/02/25/dee-firmware/
 * 
 * Works via MQTT over WiFi, as well as with directly attached to wall switch.
 * 
 * Working modes:
 *   * RED LED on                      - module powered up.
 *   * BLUE LED blinks                 - module in setup mode. At this moment open WiFi hotspot is enabled.
 *                                       Once enabled, one can connect 'Dee-xxxxx' WiFi network and open
 *                                       configuration page on 192.168.4.1 address.
 *   * BLUE LED on                     - module connected to WiFi network.
 *   * BLUE LED off                    - module is offline. 
 *
 * Operation controls:
 *   * Enter setup mode                - quickly press/release switch 5 times
 *
 * Configuration (setup mode):
 *   Following parameters can be found under 'Configure WiFi' menu item:
 *    * SSID/password - connection to WiFi
 *    * mqtt server      - MQTT server address (or host name)
 *    * mqtt port        - MQTT port
 *    * mqtt client name - just quess :-)
 *    * mqtt user
 *    * mqtt password
 *    * mqtt output topic          - topic for output of relay current state. Value is 0/1 for off/on states. 
 *                                   Additionally contains a manual switch state by providing a dot in the end.
 *                                   Examples:
 *                                    '0.' - relay is off, appropriate switch is pressed
 *                                    '1' - relay is on, appropriate switch released
 *                                    '1.' - relay is on, appropriate switch pressed.
 *    * mqtt commands topic         - topic for commands input.
 *    * respect switch states       - when disabled - each manual switch state change triggers relay state change. 
 *                                    when enabled - pressing manual switch, turns on relay. If relay already was on, 
 *                                    then does nothing. 
 *    * invert switch keys          - inverts manual switch logic (also makes effect on mqtt output of '.' symbol). 
 * 
 * Supports following commands over MQTT /cmd topics:
 *  * 1    - turn on appropriate relay (closes)
 *  * 0    - turn off appropriate relay (opens)
 *  * set  - enter setup mode. Analogue to pressing 'Pairing Button' on module.
 *              Example:
 *                'set' - open WiFi hotspot 'Dee-xxxxx' will be enabled for 3 minutes. After 3 minutes will exit setup mode.
 * 
 * 
 * Author: Anar Ibragimoff (anar@ibn.by)
 * 
 */
#include <FS.h>                 // this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino
#include <DNSServer.h>          // Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>   // Local WebServer used to serve the configuration portal
#include <WiFiManager.h>        // https://github.com/tzapu/WiFiManager/tree/development

#include <Ticker.h>

#include <WiFiUdp.h>
#include <mDNSResolver.h>       // https://github.com/madpilot/mDNSResolver

#include <ArduinoJson.h>        // https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>       // https://github.com/knolleary/pubsubclient

// GPIOs
#define LED_RED 0
#define LED_BLUE 16
#define SWITCH 4
#define BUTTON 12
#define RELAY 13

#define CONFIG_TIMEOUT_MS 300000

#define LOOP_DELAY_MS 10
#define MANUAL_SETUP_MAX_DELAY_MS 500
#define MANUAL_SETUP_COUNTER 5

// commands
#define CMD_ON "1"
#define CMD_OFF "0"
#define CMD_SETUP "set"


// --------------------------------------------------
#define CONFIG_FILE "/config.json"

#define CHECKBOX "true' type='checkbox' style='width: auto;"
#define CHECKBOX_CHECKED "true' type='checkbox' checked style='width: auto;"

// MQTT config
char mqttServer[40]   = "TiffanyAching.local";
char mqttPort[6]      = "1883";
char mqttClientName[40];
char mqttUser[40]     = "moist";
char mqttPassword[40] = "password";
char mqttOutTopic[40] = "ibnhouse/light/kitchen/sta";
char mqttInTopic[40]  = "ibnhouse/light/kitchen/cmd";

boolean offline = true;

WiFiClient espClient;
WiFiManager wifiManager;
WiFiManagerParameter customMqttServer("mqtt_server", "mqtt server", "mqtt server", 40);
WiFiManagerParameter customMqttPort("mqtt_port", "mqtt port", "port", 6);
WiFiManagerParameter customMqttClientName("mqtt_client_name", "mqtt client name", "mqtt client name", 16);
WiFiManagerParameter customMqttUser("mqtt_user", "mqtt user", "mqtt user", 16);
WiFiManagerParameter customMqttPassword("mqtt_password", "mqtt password", "mqtt password", 16);
WiFiManagerParameter customMqttOutTopic("mqtt_out_topic", "mqtt output topic", "mqtt out topic", 40);
WiFiManagerParameter customMqttInTopic("mqtt_in_topic", "mqtt commands topic", "mqtt in topic", 40);
WiFiManagerParameter customRespectSwitchState("respect_switch", "", CHECKBOX, 70);
WiFiManagerParameter customRespectSwitchStateLabel("respect switch states");
WiFiManagerParameter customInvertSwitch("invert_switch", "", CHECKBOX, 70);
WiFiManagerParameter customInvertSwitchLabel("invert switch keys");
bool wifiManagerSetupRunning = false;
bool restart = false;
unsigned long wifiManagerSetupStart;

PubSubClient mqttClient(espClient);
unsigned long mqttConnectAttempt = 0;
unsigned long mqttConnectDelay = 0;
WiFiUDP udp;
mDNSResolver::Resolver mDnsResolver(udp);
IPAddress mqttServerIp = INADDR_NONE;

boolean invertSwitch = false;
boolean respectSwitchState = true;
boolean sState;
boolean lState;
unsigned long manualSetupActivatorTime;
int manualSetupModeCounter;

Ticker ledTicker;

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(SWITCH, INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(BUTTON, INPUT);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP 

  // enable Auto Light Sleep (to reduce consumption during delay() periods in main loop)
  // enabling this + delay() in the main loop reduces consumption by module 0.8w -> 0.5w
  wifi_set_sleep_type(LIGHT_SLEEP_T); 

  digitalWrite(LED_RED, HIGH); // turn it on
  digitalWrite(LED_BLUE, HIGH); // turn it off

  sState = ( digitalRead(SWITCH) == LOW ); 

  manualSetupModeCounter = 0;

  startWifiManager(false);
  
  // MQTT connection
  mqttClient.setCallback(mqttCallback);

}

void loop() {

  gpioLoop();

  wifimanagerLoop();

  if (!offline) {
    mqttLoop();
  }

  // try to save power by delaying, which forces a light sleep mode.
  delay(LOOP_DELAY_MS);

  if (restart) {
    ESP.restart();
  }
}

void gpioLoop() {
  
  // is WifiManager configuration portal requested?
  if ( digitalRead(BUTTON) == LOW ) {
    startWifiManager(true);
  }

  // process manual switches
  boolean sChanged = false;
  if ( digitalRead(SWITCH) == LOW ) {
    if (sState == false) {
      sChanged = true;
    }
    sState = true;
  } else {
    if (sState == true) {
      sChanged = true;
    }
    sState = false;
  }

  if (sChanged) {
    if (respectSwitchState) {
      if (invertSwitch) {
        if (sState) {
          disableL();
        } else {
          enableL();
        }
      } else {
        if (sState) {
          enableL();
        } else {
          disableL();
        }
      }
    } else {
      if (lState) {
        disableL();
      } else {
        enableL();
      }
    }
  }

  if (sChanged) {
    unsigned long manualSetupActivatorDelay = millis() - manualSetupActivatorTime;
    if (manualSetupActivatorDelay < MANUAL_SETUP_MAX_DELAY_MS){
      if (++manualSetupModeCounter == MANUAL_SETUP_COUNTER) {
        startWifiManager(true);
      }
    } else {
      manualSetupModeCounter = 0;
    }
    manualSetupActivatorTime = millis();
  }

}

void wifimanagerLoop() {

  wifiManager.process();
  if (wifiManagerSetupRunning) {
    if ((millis() - wifiManagerSetupStart) > CONFIG_TIMEOUT_MS) {
      wifiManager.stopConfigPortal();
      wifiManagerSetupStopped();
    }
  } else {
    if (offline) {
      digitalWrite(LED_BLUE, HIGH); // turn it off
    } else {
      digitalWrite(LED_BLUE, LOW); // turn it on
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (offline) {
      mDnsResolver.setLocalIP(WiFi.localIP());
      mqttServerIp = mDnsResolver.search(mqttServer);
      // MQTT connection
      if(mqttServerIp != INADDR_NONE) {
        mqttClient.setServer(mqttServerIp, atoi(mqttPort));
      } else {
        mqttClient.setServer(mqttServer, atoi(mqttPort));
      }
    }
    offline = false;
  } else {
    offline = true;
  }

  if (!offline) {
    mDnsResolver.loop();
  }
  
}

void ledTick()
{
  int ledState = digitalRead(LED_BLUE);
  digitalWrite(LED_BLUE, !ledState);
}

//callback notifying us of the need to save config
void saveParamsCallback () {
  
  //save the custom parameters to FS
  //read updated parameters
  strcpy(mqttServer, customMqttServer.getValue());
  strcpy(mqttPort, customMqttPort.getValue());
  strcpy(mqttClientName, customMqttClientName.getValue());
  strcpy(mqttUser, customMqttUser.getValue());
  strcpy(mqttPassword, customMqttPassword.getValue());
  strcpy(mqttOutTopic, customMqttOutTopic.getValue());
  strcpy(mqttInTopic, customMqttInTopic.getValue());
  respectSwitchState = strcmp("true", customRespectSwitchState.getValue()) == 0 ? true : false;
  invertSwitch = strcmp("true", customInvertSwitch.getValue()) == 0 ? true : false;

  DynamicJsonDocument json(1024);
  json["mqtt_server"] = mqttServer;
  json["mqtt_port"] = mqttPort;
  json["mqtt_client_name"] = mqttClientName;
  json["mqtt_user"] = mqttUser;
  json["mqtt_password"] = mqttPassword;
  json["mqtt_out_topic"] = mqttOutTopic;
  json["mqtt_in_topic"] = mqttInTopic;
  json["respect_switch_state"] = respectSwitchState ? "true" : "false";
  json["invert_switch"] = invertSwitch ? "true" : "false";

  File configFile = SPIFFS.open(CONFIG_FILE, "w");
  serializeJson(json, configFile);
  configFile.close();
  //end save
  
  wifiManagerSetupStopped();
}

void wifiManagerSetupStarted(WiFiManager *myWiFiManager) {
  ledTicker.attach_ms(1000, ledTick); // start slow blinking
  wifiManagerSetupRunning = true;
  wifiManagerSetupStart = millis();
}

void wifiManagerSetupStopped() {
  //ESP.restart();
  restart = true; // don't restart immediately. let WifiManager finish handleWifiSave() execution

  //ledTicker.detach();
  //digitalWrite(LED_BLUE, LOW); // turn it off
  //wifiManagerSetupRunning = false;

  //WiFi.hostname(mqttClientName);
}

void startWifiManager(boolean onDemand) {

  if (wifiManagerSetupRunning) {
    return;
  }

  if (!onDemand) {
    String apName = "Dee-" + String(ESP.getChipId(), HEX);
    strcpy(mqttClientName, apName.c_str());
    
    if (SPIFFS.begin()) {
      if (SPIFFS.exists("/config.json")) {
        //file exists, reading and loading
        File configFile = SPIFFS.open(CONFIG_FILE, "r");
        if (configFile) {
          size_t size = configFile.size();
          // Allocate a buffer to store contents of the file.
          std::unique_ptr<char[]> buf(new char[size]);
  
          configFile.readBytes(buf.get(), size);
          DynamicJsonDocument json(1024);
          DeserializationError jsonError = deserializeJson(json, buf.get());
          if (!jsonError) {
            if (json.containsKey("mqtt_server") && strlen(json["mqtt_server"]) > 0) strcpy(mqttServer, json["mqtt_server"]);
            if (json.containsKey("mqtt_port") && strlen(json["mqtt_port"]) > 0) strcpy(mqttPort, json["mqtt_port"]);
            if (json.containsKey("mqtt_client_name") && strlen(json["mqtt_client_name"]) > 0) strcpy(mqttClientName, json["mqtt_client_name"]);
            if (json.containsKey("mqtt_user") && strlen(json["mqtt_user"]) > 0) strcpy(mqttUser, json["mqtt_user"]);
            if (json.containsKey("mqtt_password") && strlen(json["mqtt_password"]) > 0) strcpy(mqttPassword, json["mqtt_password"]);
            if (json.containsKey("mqtt_out_topic") && strlen(json["mqtt_out_topic"]) > 0) strcpy(mqttOutTopic, json["mqtt_out_topic"]);
            if (json.containsKey("mqtt_in_topic") && strlen(json["mqtt_in_topic"]) > 0) strcpy(mqttInTopic, json["mqtt_in_topic"]);
            if (json.containsKey("respect_switch_state") && strlen(json["respect_switch_state"]) > 0) respectSwitchState = strcmp("true", json["respect_switch_state"]) == 0 ? true : false;
            if (json.containsKey("invert_switch") && strlen(json["invert_switch"]) > 0) invertSwitch = strcmp("true", json["invert_switch"]) == 0 ? true : false;
          }
        }
      }
    }
    //end read
  
    WiFi.hostname(mqttClientName);
  
    customMqttServer.setValue(mqttServer, 40);
    customMqttPort.setValue(mqttPort, 6);
    customMqttClientName.setValue(mqttClientName, 40);
    customMqttUser.setValue(mqttUser, 40);
    customMqttPassword.setValue(mqttPassword, 40);
    customMqttOutTopic.setValue(mqttOutTopic, 40);
    customMqttInTopic.setValue(mqttInTopic, 40);
  
    wifiManager.setSaveParamsCallback(saveParamsCallback);
    wifiManager.setAPCallback(wifiManagerSetupStarted);
  
    wifiManager.setConfigPortalTimeout(CONFIG_TIMEOUT_MS / 1000);
    wifiManager.setConfigPortalBlocking(false);
  
    //add all your parameters here
    wifiManager.addParameter(&customMqttServer);
    wifiManager.addParameter(&customMqttPort);
    wifiManager.addParameter(&customMqttClientName);
    wifiManager.addParameter(&customMqttUser);
    wifiManager.addParameter(&customMqttPassword);
    wifiManager.addParameter(&customMqttOutTopic);
    wifiManager.addParameter(&customMqttInTopic);
    wifiManager.addParameter(&customRespectSwitchState);
    wifiManager.addParameter(&customRespectSwitchStateLabel);
    wifiManager.addParameter(&customInvertSwitch);
    wifiManager.addParameter(&customInvertSwitchLabel);

  }
  
  // refresh dirty hacked boolean values
  customRespectSwitchState.setValue(respectSwitchState ? CHECKBOX_CHECKED : CHECKBOX, 70);
  customInvertSwitch.setValue(invertSwitch ? CHECKBOX_CHECKED : CHECKBOX, 70);
  
  if (onDemand) {
    wifiManager.startConfigPortal(mqttClientName);
  } else {
    wifiManager.autoConnect(mqttClientName);
  }

}

void enableL() {
  digitalWrite(RELAY, HIGH); // turn on
  lState = true;
  mqttCommunicate();
}

void disableL(){
  digitalWrite(RELAY, LOW); // turn off
  lState = false;
  mqttCommunicate();
}

void mqttLoop() {
  if (!mqttClient.connected()) {
    if(!mqttReconnect()){
      return;
    }
  }
  mqttClient.loop();
}

void mqttCommunicate() {
  if (!offline && mqttClient.connected()) {
    char mqttMsg[3];
    mqttMsg[2] = 0;
    mqttMsg[0] = lState ? '1' : '0';
    if (invertSwitch) {
      mqttMsg[1] = sState ? 0 : '.';
    } else {
      mqttMsg[1] = sState ? '.' : 0;
    }
    mqttClient.publish(mqttOutTopic, mqttMsg, true);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {

  char payloadCopy[10];
  int _length = std::min<unsigned int>(length, 9);
  strncpy(payloadCopy, (char*)payload, _length);
  payloadCopy[_length] = 0x0;

  String cmd = String(payloadCopy);
  if (cmd.startsWith(CMD_SETUP)){
    startWifiManager(true);
  }

  if (cmd.startsWith(CMD_ON)){
    enableL();
  }
  if (cmd.startsWith(CMD_OFF)){
    disableL();
  }

}

boolean mqttReconnect() {
  if (!mqttClient.connected()) {
    if (millis() - mqttConnectAttempt > mqttConnectDelay ) { // don't attempt more often than a delay
      mqttConnectDelay += 1000; // increase reconnect attempt delay by 1 second
      if (mqttConnectDelay > 60000) { // don't attempt more frequently than once a minute
        mqttConnectDelay = 60000;
      }
      // Attempt to connect
      mqttConnectAttempt = millis();
      if (mqttClient.connect(mqttClientName, mqttUser, mqttPassword)) {
        // Once connected resubscribe
        mqttClient.subscribe(mqttInTopic);
        mqttConnectDelay = 0;
        return true;
      }
    }
  }
  return false;
}
