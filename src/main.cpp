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

#define SHT31_ADDRESS   0x44

uint32_t start;
uint32_t stop;
SHT31 sht;
LiquidCrystal_I2C lcd(0x27,16,2);

const int releHum= 12;
const int releTemp= 13;
const int vent= 14;
const int led = 4; //todo

float t = 0.0;
float h = 0.0;

float humLow = 75;
float humHigh = 85;
int humOn = 0;

float tempLow = 10;
float tempHigh = 30;
int tempOn = 0;

int ledOn = 0;
int ventOn = 0;

unsigned long previousMillis = 0; //stoe last time SHT was updated
const long interval = 1000;
/****** WiFi Connection Details *******/
const char* ssid = "Pura Vivo";
const char* password = "rodrigo1000";

/******* MQTT Broker Connection Details *******/
const char* mqtt_server = "daf2ed2d22014cc5821a43f3fdcf44a9.s1.eu.hivemq.cloud";
const char* mqtt_username = "@Rshw";
const char* mqtt_password = "@Rshw123456789";
const int mqtt_port =8883;

/**** Secure WiFi Connectivity Initialisation *****/
WiFiClientSecure espClient;

/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (100)
char msg[MSG_BUFFER_SIZE];


/****** root certificate *********/

const char* test_root_ca= \
  "-----BEGIN CERTIFICATE-----\n" \
  "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\n" \
  "-----END CERTIFICATE-----\n";

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
        digitalWrite(led, LOW);
        ledOn = 1;
      }   // Turn the LED on
      else {
        digitalWrite(led, HIGH);
        ledOn = 0;
      }  // Turn the LED off
    }
    //vent Income
    if( strcmp(topic,"input_vent") == 0){
      if (incommingMessage.equals("1")) {
        digitalWrite(vent, LOW);
        ventOn = 1;
      }   // Turn on?? TODO
      else {
        digitalWrite(vent, HIGH);
        ventOn = 0;
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
  lcd.begin (16,2);
  lcd.setBacklight(HIGH);
  Serial.begin(9600);
  while (!Serial) delay(1);
  setup_wifi();
  Timezone myTZ;
  myTZ.setLocation(F("Brazil/East"));
  waitForSync();
  
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
      digitalWrite(releHum, LOW);
      humOn = 1 ;
    } 
    else if (humLow < h and h < humHigh and humOn==1){
      digitalWrite(releHum, LOW);
    }
    else{
      digitalWrite(releHum, HIGH);
      humOn = 0 ;  
    } 
    //temp
    if (t < tempLow){
      digitalWrite(releTemp, LOW);
      tempOn = 1 ;
    } 
    else if (tempLow < t and t < tempHigh and tempOn==1){
      digitalWrite(releTemp, LOW);
    }
    else{
      digitalWrite(releTemp, HIGH);
      tempOn = 0 ;  
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
