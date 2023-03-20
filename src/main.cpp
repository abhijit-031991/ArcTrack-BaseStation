#define TINY_GSM_MODEM_SIM800

#include <Arduino.h>
#include <permaDefs.h>
#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <elapsedMillis.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include <Ticker.h>
#include <SPIFFS.h>


SoftwareSerial GSMS(SIMRX, SIMTX);

// Definittions //

#define SerialMon Serial
#define SerialAT GSMS



// Library Definitions //

TinyGsm modem(GSMS);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);
uint32_t lastReconnectAttempt = 0;
BluetoothSerial SerialBT;
Ticker hbtimer;
// elapsedMillis bTime;
// elapsedMillis mTime;

//////////////////////////////////////////////////////////////
//**********              VARIABLES               **********//
//////////////////////////////////////////////////////////////

// Boolean Variables 
bool loraStatus = false;
bool BTConStatus = false;
bool netConStatus = false;
bool mqttConStatus = false;
bool repeatPackage = false;
bool heartBeat = false;
bool hbflag = false;
bool localStorage = false;
bool autoDownload = false;
bool syncData = false;

// Location Variables
float lat = 0.0;
float lng = 0.0;
uint8_t altitude = 0.0;

// Packet variables
unsigned long packetCount = 0;

// Other Variables
int packetSize;

//////////////////////////////////////////////////////////////
/////                   FUNCTIONS                        /////
//////////////////////////////////////////////////////////////

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.println((char*)broker);
  Serial.flush();
  // Connect to MQTT Broker
  // boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect(StationID);
  mqtt.setKeepAlive(10000);

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  mqttConStatus = true;  
  // mqtt.subscribe(settingsSupTopic);
  return mqtt.connected();

}

void mqttCallback(char* topic, byte* payload, unsigned int len) {

  SerialMon.print(F("Message arrived ["));
  SerialMon.print(topic);
  SerialMon.print(F("]: "));
  SerialMon.println();

  Serial.println(len);
  char x[len+2];
  strncpy(x, (char*)payload, len+2);
  delay(2000);
  String top = (char*)topic;
  Serial.println(x);
  Serial.println(F("DONE"));

}

void mqttSetup(){
// MQTT Broker setup
  mqtt.setServer((char*)broker, 1883);
  mqtt.setCallback(mqttCallback);
  
  if (!mqtt.connected()) {
    SerialMon.println(F("MQTT Disconnected"));
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()){
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
}

void gprsConnect(){
  if (!modem.isGprsConnected()) {
    SerialMon.println(F("GPRS not connected!"));
    SerialMon.print(F("Connecting to ")); SerialMon.println(apn);
    Serial.flush();
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(F(" fail"));
      return;
    }
    if (modem.isGprsConnected()){ 
      SerialMon.println(F("GPRS reconnected")); 
    }
  }
  delay(100);
}

void networkInit(){
  modem.init();
  String modemInfo = modem.getModemInfo();
  SerialMon.print(F("Modem Info: "));
  SerialMon.println(modemInfo);

  SerialMon.print(F("Waiting for network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(F("fail"));
    // delay(10000);
    return;
  }
  SerialMon.println(F("success"));

  if (modem.isNetworkConnected()) { 
    SerialMon.print(F("Network : ")); Serial.println(modem.getOperator());
    SerialMon.print(F("Signal : ")); Serial.println(modem.getSignalQuality()); 
    netConStatus = true;
    delay(100);

    gprsConnect();
    
    mqttSetup();
  }else{
    SerialMon.println(F("No Network"));
  }  
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
// Callback function implementation
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println();
    Serial.println("Client Connected");
    BTConStatus = true;
  }
 
  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
    BTConStatus = false;
  }
}

String initStats(){
  char data[128];
  StaticJsonDocument<128> doc;
  doc[F("lora")] = loraStatus;
  doc[F("net")] = netConStatus;
  doc[F("mqtt")] = mqttConStatus;
  doc[F("bat")] = modem.getBattPercent();
  doc[F("sig")] = modem.getSignalQuality();
  doc[F("V")] = modem.getBattVoltage();
  serializeJson(doc, data);
  return data;
}

void startHeartbeat(bool x){
  if (x)
  {
    hbflag = true;
  }   
}

void heartBeatPing(){
  char d[136];
  hb h;
  if (!modem.isNetworkConnected())
  {
    Serial.println(F("Network Unavailable"));
    networkInit();
  }else{
    gprsConnect();
    mqttSetup();
  }

  StaticJsonDocument<136> doc;
  doc[F("id")] = StationID;
  doc[F("Alt")] = altitude;
  doc[F("pkts")] = packetCount;
  doc[F("Bat")] = modem.getBattPercent();
  doc[F("Sig")] = modem.getSignalQuality();
  h.stid = STID;
  h.cnt = packetCount;
  h.bat = modem.getBattPercent();
  h.csq = modem.getSignalQuality();
  serializeJson(doc, d);
  Serial.println(d);

  if (mqtt.connected())
  {
    if (mqtt.publish(telemetryTopic, (char*)d))
    {
      Serial.println(F("Heart-Beat Transmitted"));
      Serial.flush(); 
      mqtt.disconnect();
      modem.gprsDisconnect();
    } 
  }else{
    mqtt.disconnect();
    modem.gprsDisconnect();
  }
  
  modem.poweroff();

  if (repeatPackage == true)
  {    
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&h, sizeof(h));
    LoRa.endPacket();
  }
}

void savePings(ping x){

  String y;
  StaticJsonDocument<200> doc;
  doc[F("cnt")] = x.cnt;
  doc[F("dTyp")] = x.devtyp;
  doc[F("id")] = x.ta;
  doc[F("lat")] = x.la;
  doc[F("lng")] = x.ln;
  doc[F("mor")] = x.mortality;
  doc[F("rssi")] = LoRa.packetRssi();
  serializeJson(doc, y);

  File file = SPIFFS.open("/runTime.txt", FILE_APPEND);
  if (!file) {
    Serial.println(F("Error opening file"));
    return;
  }
  if (file.print(y))
  {
    Serial.println(F("Filed"));
  }  
  file.close();
}

void dataTransfer(){
  digitalWrite(SIMCTL, HIGH);
  delay(1100);
  digitalWrite(SIMCTL, LOW);
  if (!modem.isNetworkConnected())
  {
    networkInit();
  }
  if (modem.isNetworkConnected() && !modem.isGprsConnected())
  {
    gprsConnect();
  }
  if (modem.isGprsConnected() && !mqtt.connected());
  {
    mqttConnect();
  }  

  
  if (mqtt.connected())
  {
    File f = SPIFFS.open("/runTime.txt", FILE_READ);
    if (!f) {
      Serial.println("Failed to open file for reading");
      return;
    }
    char sect[150];
    int sectionSize = 0;
    while (f.available()) {
      // Read a character from the file
      char c = f.read();

      // Append the character to the current section
      sect[sectionSize] = c;
      sectionSize++;

      // Check if the current section ends with '}'
      if (c == '}') {
        
        if (mqtt.connected())
        {
          Serial.println(sect);
          mqtt.publish(telemetryTopic, sect);
        }
        
        sectionSize = 0;
      }    
    }
    f.close();    
  } 
  File f = SPIFFS.open("/runTime.txt", FILE_WRITE);
  SPIFFS.remove(F("/runTime.txt"));  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  GSMS.begin(9600, SWSERIAL_8N1);
  WiFi.mode(WIFI_OFF);
  SerialBT.register_callback(callback);
  SerialBT.begin(BTNAME);
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  pinMode(SIMCTL, OUTPUT);
  digitalWrite(SIMCTL, HIGH);
  delay(1100);
  digitalWrite(SIMCTL, LOW);


  Serial.println(F("Starting System. Awaiting BT Connection"));
  while (BTConStatus == false)
  {
    Serial.print(F("."));
    delay(100);    
  }
  
  //*****************************************************//
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(867E6)) {
    Serial.println(F("Starting LoRa failed!"));
    // while (1);
    loraStatus = false;
  }else{
    Serial.println(F("LoRa Radio Started"));
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setSpreadingFactor(12);
    // LoRa.setGain(0);
    loraStatus = true;
  }
  networkInit();
  mqtt.disconnect();
  // modem.gprsDisconnect();
  SerialBT.println(initStats());
  String x;
  bool d = true;
  while (d == true)
  { 
    if (SerialBT.available() > 0)
    {
      x = SerialBT.readString();
      Serial.println(x);
      if (x.indexOf("{") != -1 && x.indexOf("}") != -1)
      {
        Serial.println(F("deserializing.."));
        StaticJsonDocument<136> doc;
        deserializeJson(doc, x);
        repeatPackage = doc[F("RPT")];
        lat = doc[F("lat")];
        lng = doc[F("lng")];
        altitude = doc[F("alt")];
        heartBeat = doc[F("HBT")];
        localStorage = doc[F("LCL")];
        autoDownload = doc[F("ADWD")];
        syncData = doc[F("SYNC")];
        d = false; 
      }      
    }                    
  }
  heartBeatPing();
  if (SPIFFS.open("/runTime.txt", FILE_WRITE))
  {
    Serial.println(F("delete file"));
    SPIFFS.remove(F("/runTime.txt"));
  }
  SerialBT.end();
  btStop();
  Serial.println(F("SYSTEM READY"));   
  hbtimer.attach(300, startHeartbeat, heartBeat);   
}

void loop() {
  if (hbflag == true)
  {
    digitalWrite(SIMCTL, HIGH);
    delay(1100);
    digitalWrite(SIMCTL, LOW);
    heartBeatPing();
    delay(5000);
    dataTransfer();
    modem.poweroff();
    hbflag = false;
    Serial.println(ESP.getFreeHeap());
  } 

  packetSize = LoRa.parsePacket();
  if (packetSize > 0)
  {
    Serial.print(F("Received packet size'")); Serial.println(packetSize);
  }
    
  if (packetSize == 14)
  {
    ping p; 

    while (LoRa.available())
    {
      LoRa.readBytes((uint8_t*)&p, packetSize);
    }
    Serial.println(p.cnt);
    Serial.println(p.devtyp);
    Serial.println(p.la, 6);
    Serial.println(p.ln, 6);
    Serial.println(p.mortality);
    Serial.println(p.ta);
    Serial.println(LoRa.packetRssi());
    packetCount = packetCount + 1;
    savePings(p);
  }  
}