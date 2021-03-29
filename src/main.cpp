#include <Arduino.h>

// LCD
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>

// DHT11
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// Wifi / MQTT
#include <WiFi.h>
#include <PubSubClient.h>

// Tone
#include <Tone32.h>

// DHT 11 PIN
#define DHTPIN 13 // Digital pin connected to the DHT sensor

// Define DHT Type
#define DHTTYPE DHT11 // DHT 11

// Define Tone PIN
#define BUZZER_PIN 12
#define BUZZER_CHANNEL 0
// Motion Sensor PIN
int sensorPin = 14;

//LCD
const int PIN_SCL = 32;
const int PIN_SDA = 33;
LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display
int show = -1;
bool home = true;

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
float temp = 0;
float hum = 0;

// MQTT
const char *mqtt_server = "test.mosquitto.org";
const char *topicSensorDht11Temp = "YNOV/BDX/DHT11/24:62:AB:FD:C8:18/TEMP";
const char *topicSensorDht11Hum = "YNOV/BDX/DHT11/24:62:AB:FD:C8:18/HUM";
const char *topicMovementDectector = "YNOV/BDX/DHT11/24:62:AB:FD:C8:18/DETECTOR";

// Wifi
const char *ssid = "Non Mais Oh ca va pas ";
const char *password = "*************";
WiFiClient espClient;
PubSubClient client(espClient);

// Timer
unsigned long previousMillisEcran = 0;
unsigned long previousMillisMqtt = 0;
unsigned long previousMillisPing = 0;
unsigned long previousMillisSensor = 0;
int cpt = 0;

bool played = false;

int tempo = 250;

int melody[] = {
    NOTE_D5, -4, NOTE_E5, -4, NOTE_A4, 4, //1
    NOTE_E5, -4, NOTE_FS5, -4, NOTE_A5, 16, NOTE_G5, 16, NOTE_FS5, 8,
    NOTE_D5, -4, NOTE_E5, -4, NOTE_A4, 2,
    NOTE_A4, 16, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 8, NOTE_D5, 16,
    NOTE_D5, -4, NOTE_E5, -4, NOTE_A4, 4, //repeat from 1
    NOTE_E5, -4, NOTE_FS5, -4, NOTE_A5, 16, NOTE_G5, 16, NOTE_FS5, 8,
    NOTE_D5, -4, NOTE_E5, -4, NOTE_A4, 2,
    NOTE_A4, 16, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 8, NOTE_D5, 16,
    REST, 4, NOTE_B4, 8, NOTE_CS5, 8, NOTE_D5, 8, NOTE_D5, 8, NOTE_E5, 8, NOTE_CS5, -8,
    NOTE_B4, 16, NOTE_A4, 2, REST, 4,

    REST, 8, NOTE_B4, 8, NOTE_B4, 8, NOTE_CS5, 8, NOTE_D5, 8, NOTE_B4, 4, NOTE_A4, 8, //7
    NOTE_A5, 8, REST, 8, NOTE_A5, 8, NOTE_E5, -4, REST, 4,
    NOTE_B4, 8, NOTE_B4, 8, NOTE_CS5, 8, NOTE_D5, 8, NOTE_B4, 8, NOTE_D5, 8, NOTE_E5, 8, REST, 8,
    REST, 8, NOTE_CS5, 8, NOTE_B4, 8, NOTE_A4, -4, REST, 4,
    REST, 8, NOTE_B4, 8, NOTE_B4, 8, NOTE_CS5, 8, NOTE_D5, 8, NOTE_B4, 8, NOTE_A4, 4,
    NOTE_E5, 8, NOTE_E5, 8, NOTE_E5, 8, NOTE_FS5, 8, NOTE_E5, 4, REST, 4,

    NOTE_D5, 2, NOTE_E5, 8, NOTE_FS5, 8, NOTE_D5, 8, //13
    NOTE_E5, 8, NOTE_E5, 8, NOTE_E5, 8, NOTE_FS5, 8, NOTE_E5, 4, NOTE_A4, 4,
    REST, 2, NOTE_B4, 8, NOTE_CS5, 8, NOTE_D5, 8, NOTE_B4, 8,
    REST, 8, NOTE_E5, 8, NOTE_FS5, 8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_FS5, -8, NOTE_FS5, -8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,

    NOTE_E5, -8, NOTE_E5, -8, NOTE_D5, -8, NOTE_CS5, 16, NOTE_B4, -8, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16, //18
    NOTE_D5, 4, NOTE_E5, 8, NOTE_CS5, -8, NOTE_B4, 16, NOTE_A4, 8, NOTE_A4, 8, NOTE_A4, 8,
    NOTE_E5, 4, NOTE_D5, 2, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_FS5, -8, NOTE_FS5, -8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_A5, 4, NOTE_CS5, 8, NOTE_D5, -8, NOTE_CS5, 16, NOTE_B4, 8, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,

    NOTE_D5, 4, NOTE_E5, 8, NOTE_CS5, -8, NOTE_B4, 16, NOTE_A4, 4, NOTE_A4, 8, //23
    NOTE_E5, 4, NOTE_D5, 2, REST, 4,
    REST, 8, NOTE_B4, 8, NOTE_D5, 8, NOTE_B4, 8, NOTE_D5, 8, NOTE_E5, 4, REST, 8,
    REST, 8, NOTE_CS5, 8, NOTE_B4, 8, NOTE_A4, -4, REST, 4,
    REST, 8, NOTE_B4, 8, NOTE_B4, 8, NOTE_CS5, 8, NOTE_D5, 8, NOTE_B4, 8, NOTE_A4, 4,
    REST, 8, NOTE_A5, 8, NOTE_A5, 8, NOTE_E5, 8, NOTE_FS5, 8, NOTE_E5, 8, NOTE_D5, 8,

    REST, 8, NOTE_A4, 8, NOTE_B4, 8, NOTE_CS5, 8, NOTE_D5, 8, NOTE_B4, 8, //29
    REST, 8, NOTE_CS5, 8, NOTE_B4, 8, NOTE_A4, -4, REST, 4,
    NOTE_B4, 8, NOTE_B4, 8, NOTE_CS5, 8, NOTE_D5, 8, NOTE_B4, 8, NOTE_A4, 4, REST, 8,
    REST, 8, NOTE_E5, 8, NOTE_E5, 8, NOTE_FS5, 4, NOTE_E5, -4,
    NOTE_D5, 2, NOTE_D5, 8, NOTE_E5, 8, NOTE_FS5, 8, NOTE_E5, 4,
    NOTE_E5, 8, NOTE_E5, 8, NOTE_FS5, 8, NOTE_E5, 8, NOTE_A4, 8, NOTE_A4, 4,

    REST, -4, NOTE_A4, 8, NOTE_B4, 8, NOTE_CS5, 8, NOTE_D5, 8, NOTE_B4, 8, //35
    REST, 8, NOTE_E5, 8, NOTE_FS5, 8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_FS5, -8, NOTE_FS5, -8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_E5, -8, NOTE_E5, -8, NOTE_D5, -8, NOTE_CS5, 16, NOTE_B4, 8, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_D5, 4, NOTE_E5, 8, NOTE_CS5, -8, NOTE_B4, 16, NOTE_A4, 4, NOTE_A4, 8,

    NOTE_E5, 4, NOTE_D5, 2, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16, //40
    NOTE_FS5, -8, NOTE_FS5, -8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_A5, 4, NOTE_CS5, 8, NOTE_D5, -8, NOTE_CS5, 16, NOTE_B4, 8, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_D5, 4, NOTE_E5, 8, NOTE_CS5, -8, NOTE_B4, 16, NOTE_A4, 4, NOTE_A4, 8,
    NOTE_E5, 4, NOTE_D5, 2, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,

    NOTE_FS5, -8, NOTE_FS5, -8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16, //45
    NOTE_A5, 4, NOTE_CS5, 8, NOTE_D5, -8, NOTE_CS5, 16, NOTE_B4, 8, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_D5, 4, NOTE_E5, 8, NOTE_CS5, -8, NOTE_B4, 16, NOTE_A4, 4, NOTE_A4, 8,
    NOTE_E5, 4, NOTE_D5, 2, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_FS5, -8, NOTE_FS5, -8, NOTE_E5, -4, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16, //45

    NOTE_A5, 4, NOTE_CS5, 8, NOTE_D5, -8, NOTE_CS5, 16, NOTE_B4, 8, NOTE_A4, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_B4, 16,
    NOTE_D5, 4, NOTE_E5, 8, NOTE_CS5, -8, NOTE_B4, 16, NOTE_A4, 4, NOTE_A4, 8,

    NOTE_E5, 4, NOTE_D5, 2, REST, 4};

// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;

void play()
{
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2)
  {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0)
    {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    }
    else if (divider < 0)
    {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(BUZZER_PIN, melody[thisNote], noteDuration * 0.9, BUZZER_CHANNEL);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
  }
}

void setup_wifi()
{
  delay(100);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.println("Attempting MQTT connection...");
    // Precise client id name and add a random string
    String clientId = "ESP3266-YNOV-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    Serial.print("Client : " + clientId);
    if (client.connect(clientId.c_str()))
    {
      Serial.println(" [connected]");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void mqttInit()
{
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
}

void mqttSubscribe()
{
  client.subscribe(topicSensorDht11Temp);
  client.subscribe(topicSensorDht11Hum);
}

///////////////////////////////////////////////////////////
/////////////////// SET UP ////////////////////////////////
///////////////////////////////////////////////////////////

void setup()
{
  int error;
  Serial.begin(115200);
  // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  Serial.println("LCD...");

  pinMode(sensorPin, INPUT);

  while (!Serial)
    ;
  Serial.println("Dose: check for LCD");
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  if (error == 0)
  {
    Serial.println(": LCD found.");
    show = 0;
    lcd.begin(16, 2); // initialize the lcd
  }
  else
  {
    Serial.println(": LCD not found.");
  } // if

  // Print temperature sensor details.
  sensor_t sensor;
  delayMS = sensor.min_delay / 1000;
  Serial.println(delayMS);

  // Setup Wifi Connection
  delay(2000);
  Serial.println("Setup start");
  setup_wifi();
  Serial.println("\nConnected, IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());
  Serial.println("Setup End");

  // Set up MQTT Connection
  mqttInit();
}

///////////////////////////////////////////////////////////
/////////////////// LOOP //////////////////////////////////
///////////////////////////////////////////////////////////

void loop()
{
  mqttReconnect();
  client.loop();

  // if (millis() - previousMillisPing >= 5000)
  // {
  //   client.publish("YNOV/BDX", "Ping");
  //   previousMillisPing = millis();
  // }

  if (millis() - previousMillisSensor >= 5000)
  {
    if (digitalRead(sensorPin) == HIGH)
    {
      cpt = cpt + 5;
      Serial.println("mouvement detecte");
      Serial.println(cpt);
      played = true;
    }
    else
    {
      cpt = 0;
      played = false;
    }
    if (digitalRead(sensorPin) == HIGH && played)
    {
      play();
    }

    previousMillisSensor = millis();
  }

  if (millis() - previousMillisEcran >= 5000)
  {
    if (home)
    {
      lcd.setBacklight(255);
      lcd.home();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Welcome to your");
      lcd.setCursor(0, 1);
      lcd.print("house !");
      home = false;
    }
    else
    {
      // Delay between measurements.
      // delay(delayMS);

      // Create Event sensor
      sensors_event_t event;

      dht.temperature().getEvent(&event);
      if (isnan(event.temperature))
      {
        Serial.println(F("Error reading temperature!"));
      }
      else
      {
        temp = event.temperature;
        Serial.print(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.println(F("°C"));
      }

      // Get humidity event and print its value.
      dht.humidity().getEvent(&event);
      if (isnan(event.relative_humidity))
      {
        Serial.println(F("Error reading humidity!"));
      }
      else
      {
        hum = event.relative_humidity;
        Serial.print(F("Humidity: "));
        Serial.print(event.relative_humidity);
        Serial.println(F("%"));
      }

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.print(temp);
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print("Humidite:");
      lcd.print(hum);
      lcd.print("%");
    }

    previousMillisEcran = millis();
  }

  if (millis() - previousMillisMqtt >= 10000)
  {
    // Creation des variables
    char temp_msg[10], hum_msg[10];

    sprintf(temp_msg, "%.2f", temp);
    sprintf(hum_msg, "%.2f", hum);

    if (client.connected())
    {
      client.publish(topicSensorDht11Temp, temp_msg);
      client.publish(topicSensorDht11Hum, hum_msg);
      Serial.println("Published");
    }
    // mqtt(topicSensorDht11Temp, temp_msg);
    // mqtt(topicSensorDht11Hum, hum_msg);

    previousMillisMqtt = millis();
  }
}