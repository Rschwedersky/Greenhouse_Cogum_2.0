#include <SPI.h>
#include <WiFiClientSecure.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "Wire.h"
#include "SHT31.h"
#include "LittleFS.h"
#include <ezTime.h>
#include <LiquidCrystal_I2C.h>
#include "../data/secrets.h"

#define SHT31_ADDRESS   0x44

uint32_t start;
uint32_t stop;
SHT31 sht;
LiquidCrystal_I2C lcd(0x27,16,2);

const int releHum = D6;   // GPIO12
const int releTemp = D7;  // GPIO13
const int vent = D0;      // GPIO16
const int led = D3;  // GPIO15

float t = 0.0;
float h = 0.0;

float humLow = 91;
float humHigh = 92;
int humOn = 0;

float tempLow = 10;
float tempHigh = 30;
int tempOn = 0;

int ledOn = 0;
int ventOn = 0;

unsigned long previousMillis = 0; //stoe last time SHT was updated
const long interval = 1000;
unsigned long relayActivatedMillis = 0;
const unsigned long stabilizationTime = 60000; // Delay for stabilization in milliseconds (e.g., 1 minute)
bool relayDelayActive = false;

/**** Secure WiFi Connectivity Initialisation *****/
WiFiClientSecure espClient;

/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (100)
char msg[MSG_BUFFER_SIZE];


/****** Load secrets from LittleFS ******/
void loadSecrets() {
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }

  File file = LittleFS.open("/secrets.h", "r");
  if (!file) {
    Serial.println("Failed to open secrets file");
    return;
  }

  while (file.available()) {
    Serial.write(file.read());  // For debugging, you can see the file's content.
  }

  file.close();
}


/************* Connect to WiFi ***********/
void setup_wifi() {
  delay(10);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());
}


/************* Connect to MQTT Broker ***********/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      
      client.subscribe("input_led");   // subscribe the topics here
      client.subscribe("input_hum_low");
      client.subscribe("input_hum_high");
      client.subscribe("input_temp_low");
      client.subscribe("input_temp_high");
      client.subscribe("input_vent"); //TODO rodrigo
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
/***** Call back Method for Receiving MQTT messages and Switching LED ****/

void callback(char* topic, byte* payload, unsigned int length) {
  String incommingMessage = "";
  for (int i = 0; i < length; i++) incommingMessage+=(char)payload[i];
    Serial.println("Message arrived ["+String(topic)+"]"+incommingMessage);
    //Humididy Income
    if(strcmp(topic,"input_hum_low") == 0){
        humLow = incommingMessage.toFloat();
      }
    if(strcmp(topic,"input_hum_high") == 0){ 
        humHigh = incommingMessage.toFloat();
      }
    //Temperature Income
    if(strcmp(topic,"input_temp_low") == 0){
        tempLow = incommingMessage.toFloat();
      }
    if(strcmp(topic,"input_temp_high") == 0){ 
        tempHigh = incommingMessage.toFloat();
      }
  
  
    //Led Income
    if( strcmp(topic,"input_led") == 0){
      if (incommingMessage.equals("1")) {
        digitalWrite(led, HIGH);
        ledOn = 1;
      }   // Turn the LED on
      else {
        digitalWrite(led, LOW);
        ledOn = 0;
      }  // Turn the LED off
    }
    //vent Income
    if( strcmp(topic,"input_vent") == 0){
      if (incommingMessage.equals("1")) {
        digitalWrite(vent, HIGH);
        ventOn = 1;
      }   // Turn on?? TODO
      else {
        if (humOn == 0){
          digitalWrite(vent, LOW);
          ventOn = 0;
        }
      }  // Turn off
    }

}
/**** Method for Publishing MQTT Messages **********/
void publishMessage(const char* topic, String payload , boolean retained){
  if (client.publish(topic, payload.c_str(), true))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}

/**** Application Initialisation Function******/
void setup() {
  
  Wire.begin();
  sht.begin(SHT31_ADDRESS);
  pinMode(led, OUTPUT); //set up LED TODO
  pinMode(releHum, OUTPUT);
  pinMode(vent, OUTPUT); //set up LED TODO
  pinMode(releTemp, OUTPUT);
  lcd.begin (16,2);
  lcd.setBacklight(HIGH);

   // Load secrets from LittleFS
  loadSecrets();
  Serial.begin(9600);
  while (!Serial) delay(1);
  setup_wifi();
  waitForSync();
  Timezone myTZ;
  myTZ.setLocation(F("America/Sao_Paulo"));
  
  #ifdef ESP8266
    espClient.setInsecure();
  #else
    espClient.setCACert(root_ca);      // enable this line and the the "certificate" code for secure connection
  #endif
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}
void loop() {
  events(); //data lib
  if (!client.connected()) reconnect(); // check if client is connected
  client.loop();
  sht.read();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;
    // Read temperature as Celsius (the default)
    float currentT = sht.getTemperature();

    // if temperature read failed, we don't want to change t value
    if (isnan(currentT)) {
    Serial.println("Failed to read from SHT sensor!");
    Serial.println(currentT);
    }
    else {
    t = currentT;
    Serial.println(t);
    }

    // Read Humidity
    float currentH = sht.getHumidity();
    {
      // if humidity read failed, we don't want to change h value 
      if (isnan(currentH)) {
        Serial.println("Failed to read from SHT sensor!");
      }
      else {
        h = currentH;
        Serial.println(h);
      }
    }
    //hum
    if (h < humLow){
      digitalWrite(releHum, HIGH);
      digitalWrite(vent, HIGH);
      humOn = 1 ;
      ventOn = 1;
    } 
    else if (humLow < h and h < humHigh and humOn==1){
      digitalWrite(releHum, HIGH);
      digitalWrite(vent, HIGH);
      ventOn = 1;
    }
    else{
      digitalWrite(releHum, LOW);
      digitalWrite(vent, LOW);
      humOn = 0 ;
      ventOn = 0;  
    } 
    // Temp control
    if (t < tempLow) {
        // Temperature is below the lower threshold, turn off the refrigerator
        digitalWrite(releTemp, LOW);
        tempOn = 0;
    } 
    else if (t >= tempLow && t <= tempHigh && tempOn == 1) {
        // Temperature is within the desired range and refrigerator is already on, keep it on
        digitalWrite(releTemp, HIGH);
    } 
    else if (t > tempHigh && !relayDelayActive) {
      // Temperature is above the upper threshold, turn on the refrigerator
      digitalWrite(releTemp, HIGH);
      tempOn = 1;

      // Start the stabilization delay timer
      relayActivatedMillis = currentMillis;
      relayDelayActive = true;
    }

    // Check if the stabilization delay has passed
    if (relayDelayActive && (currentMillis - relayActivatedMillis >= stabilizationTime)) {
      relayDelayActive = false;
    }
  }
lcd.setCursor(0,0); //SETA A POSIÇÃO DO CURSOR
lcd.print("Hum:"); //IMPRIME O TEXTO NO DISPLAY LCD
lcd.print(h,1); //IMPRIME O TEXTO NO DISPLAY LCD
lcd.print(" Temp"); //IMPRIME O TEXTO NO DISPLAY LCD
lcd.print(t,1);
lcd.setCursor(0,1); 
lcd.print("Min: ");
lcd.print(humLow,0);
lcd.print(" Max: ");
lcd.print(humHigh,0);
/* digitalWrite(led, HIGH);
digitalWrite(releTemp, HIGH);
digitalWrite(releHum, HIGH);
digitalWrite(vent, HIGH); */


  DynamicJsonDocument doc(1024);

  doc["time"]= dateTime(ISO8601);
  
  doc["humidity"] = h;
  doc["humidityLow"] = humLow;
  doc["humidityHigh"] = humHigh;

  doc["temperature"] = t;
  doc["temperatureLow"] = tempLow;
  doc["temperatureHigh"] = tempHigh;

  doc["ledOn"] = ledOn;
  doc["ventOn"] = ventOn;
  doc["humOn"] = humOn;
  doc["tempOn"] = tempOn;

  char mqtt_message[256];
  serializeJson(doc, mqtt_message);
  publishMessage("Cogum Sensor Controller", mqtt_message, true);

  delay(1000);
}
