#include <InputDebounce.h>
#include <Ultrasonic.h>
#include <WiFiManager.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

#include <PingSerial.h>
#include <SoftwareSerial.h> // Arduino issue: a library can't include other libs
#include <ArduinoJson.h>

#include <ArduinoOTA.h>
#include <PubSubClient.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306Wemos.h>
// Wemos OLED Display Shield uses pins D2, D1, D5

// Replace with your network credentials
//#define AT_HOME 1

#include "Secrets.h"

// local port to listen for NTP UDP packets
unsigned int localPort = 2390;
IPAddress timeServerIP; // time.nist.gov NTP server address 129.6.15.28 or pool
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE ]; // buffer to hold incoming and outgoing message
WiFiUDP udp;


// screen size on Wemos OLED 64x48 pixels
#define OLED_RESET 0   // GPIO0
Adafruit_SSD1306Wemos display(OLED_RESET);

WiFiClient*   espClient = 0;
PubSubClient* mqttClient = 0;

char pubTopic[] = "rhatcher/home/laundry/sump";
char subTopic[] = "rhatcher/home/laundry/sump";

unsigned long lastMsgSent_ms = 0;
const     int msgLen = 255;
char          prevMsg[msgLen] = "prevMsg";
char          currMsg[msgLen];

// Synchronization loop for long intervals (more than 32 seconds)
#define TimeLap(t1) (long)((unsigned long)millis()-(unsigned long)t1)
//#define MSG_INTERVAL  60000 // report only every minute if nothing has changed
// shorten time interval for test purposes
#define MSG_INTERVAL  20000  // every 20 seconds
unsigned long msg_time = millis();
unsigned long dht_time = msg_time;

int nloop = 0;

// Here our US-100 is connected (software serial) to pins:
const int US100_TX = D5;
const int US100_RX = D6;

SoftwareSerial SerialUS100(US100_RX, US100_TX);

PingSerial us100(SerialUS100,30,1200); // valid from 30 -1200 mm

bool ping_enabled = true;
unsigned int pingSpeed = 100; // how fequenty are we sending out a ping (ms)
                              // 50ms would be 20x a second
unsigned long pingTimer = 0;  // Holds the next ping time

bool temp_enabled = TRUE;
unsigned int tempSpeed = 3000;
unsigned long tempTimer = 0;

int dist_mm = 0;
int temp_C  = 0;

// Inside the brackets, 512 is the size of the pool in bytes.
// Don't forget to change this value to match your JSON document.
// Use arduinojson.org/assistant to compute the capacity.
StaticJsonBuffer<512> jsonBuffer;


//This callback routine for receiving messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  display.clearDisplay();
  // 6x8 pixel characters ... last line
  display.setCursor(0,40);
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    display.print((char)payload[i]);
  }
  Serial.println();
  display.display();
  delay(1000);
}

void mqttReconnect() {
  // Loop until we're reconnected
  while ( ! mqttClient->connected() ) {
    delay(200);
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);  // I do not think this is necessary. Is the client ID important?
    // Attempt to connect
    if ( mqttClient->connect(clientId.c_str(),mqtt_user,mqtt_pass) ) {
      // This is the major change to make this sketch work. Added username, mqttpass
      Serial.println("connected");
      // ... and resubscribe
      mqttClient->subscribe(subTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient->state());
      Serial.println(" try again in 1 seconds");
      // Wait 1 seconds before retrying
      delay(1000);
    }
  }
}

void read_ping() {

  // Reading
  bool isDHTTime = (TimeLap(dht_time)>0);
  if ( ! isDHTTime ) return;
  dht_time += 4000; // wait 4000ms until next reading

}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}


void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  // get the OLED up-and-running
  delay(200);  // wait
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true);
  delay(200);
  Serial.println("display .. begin() called");
  display.display();
  delay(500);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("setup()");
  display.display();

  Serial.println("");
  Serial.print("FreeHeap: ");
  Serial.println(String(ESP.getFreeHeap()));

  // 68 pixel characters
  display.setCursor(0,16);
  display.print("heap:");
  display.println(String(ESP.getFreeHeap()));
  display.display();

  randomSeed(micros());
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(BUILTIN_LED, HIGH);

  WiFi.mode(WIFI_STA);
//  WiFi.hostname("SumpMon1");  // setHostname() for ESP32, hostname() for ESP8266
  // /Users/rhatcher/Library/Arduino15/packages/esp8266//hardware/esp8266/2.4.1/libraries//ESP8266WiFi/src/ESP8266WiFiSTA.h

  //WiFi.setAutoReconnect(true); ... apparently .. .no
  WiFi.begin(ssid, password);
  Serial.print("WiFi.begin on ");
  Serial.print(ssid);
  Serial.print(" with MAC ");
  Serial.println(WiFi.macAddress());
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
    return;
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("LaundrySumpMon-OTA");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("WiFi Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // 68 pixel characters
  display.setCursor(0,8);
  display.println(WiFi.localIP());
  display.display();

  espClient  = new WiFiClient;
  mqttClient = new PubSubClient(*espClient);

  mqttClient->setServer(mqtt_server,1883);
  mqttClient->setCallback(mqttCallback);

  // udp for NTP
  udp.begin(localPort);

  delay(10000);
}

void loop() {

  byte ping_data_available;
  byte ping_msg;

  if ( mqttClient ) { // RWH
  ArduinoOTA.handle();

  if ( ! mqttClient->connected() ) {
    mqttReconnect();
  }
  mqttClient->loop();  // look for messages we subscribed to
  }

  jsonBuffer.clear();
  JsonObject& jsonRoot = jsonBuffer.createObject();

/*
  if ( false ) {
  // has it been long enough since last, or is message new?
  //if ( isMsgTime || newMsg ) {
    msg_time += MSG_INTERVAL;
    // strcpy(dest,src)
    strcpy(prevMsg,currMsg);

    WiFi.hostByName(ntpServerName, timeServerIP);
    sendNTPpacket(timeServerIP);
    delay(500); // wait for reply
    int cb;
    int ntries=0;
    const int ntriesMax = 20;
    while ( ! ( cb = udp.parsePacket() ) && ( ++ntries < ntriesMax ) ) {
      delay(500);  // still waiting
    }
    Serial.print("udp  ntries ");
    Serial.println(ntries);
    if ( ! cb ) {
      // never got a response
      Serial.println("No NTP response!");
    } else {
      Serial.print("NTP packet received, length=");
      Serial.println(cb);
      // read the NTP packet
      udp.read(packetBuffer, NTP_PACKET_SIZE); // into buffer
      // timestamp starts at byte 40 of the received packet and is four bytes long
      unsigned long highWord = word(packetBuffer[40],packetBuffer[41]);
      unsigned long lowWord  = word(packetBuffer[42],packetBuffer[43]);
      // NTP is time in seconds since Jan 1 1900
      unsigned long secSince1900 = highWord << 16 | lowWord;
      Serial.print("seconds since Jan 1 1900 = ");
      Serial.println(secSince1900);
      const unsigned long seventyYears = 2208988800UL;
      unsigned long epoch = secSince1900 - seventyYears;
      Serial.println(epoch);

      Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
      Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
      Serial.print(':');
      if ( ((epoch % 3600) / 60) < 10 ) {
        // In the first 10 minutes of each hour, we'll want a leading '0'
         Serial.print('0');
      }
      Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
      Serial.print(':');
      if ( (epoch % 60) < 10 ) {
        // In the first 10 seconds of each minute, we'll want a leading '0'
        Serial.print('0');
      }
      Serial.println(epoch % 60); // print the second

    }
*/
    // blink on
    digitalWrite(BUILTIN_LED, LOW);


    static uint16_t last_dist_mm = 0;
    static int      last_temp_C  = 0;

    ping_data_available = us100.data_available();

    if (ping_data_available & DISTANCE) {
      Serial.print("Distance: ");
      last_dist_mm = us100.get_distance();
      Serial.println(last_dist_mm);
      JsonArray& distValues = jsonRoot.createNestedArray("distance");
      distValues.add(last_dist_mm);
      distValues.add("mm");
    }
    if (ping_data_available & TEMPERATURE) {
      Serial.print("Temperature: ");
      last_temp_C = us100.get_temperature();
      Serial.println(last_temp_C);
      JsonArray& tempValues = jsonRoot.createNestedArray("temperature");
      tempValues.add(last_temp_C);
      tempValues.add("*C");
    }

    if ( ping_data_available ) {
      //root.printTo(Serial);
      //Serial.println();
      jsonRoot.prettyPrintTo(Serial);
      Serial.println();
    }

    if (ping_enabled && (millis() >= pingTimer)) {
      // pingSpeed milliseconds since last ping, do another ping.
      pingTimer = millis() + pingSpeed;      // Set the next ping time.
      us100.request_distance();
      //Serial.println("Distance requested");
    }

    if (temp_enabled && (millis() >= tempTimer)) {
      tempTimer = millis() + tempSpeed;
      us100.request_temperature();
      //Serial.println("Temperature requested");
    }

    
    Serial.print("Publish message: ");  // Send a message to serial window for debugging.
    Serial.println(currMsg);
    if ( mqttClient) {
      Serial.print("call publish to ");
      Serial.print(pubTopic);
      mqttClient->publish(pubTopic, currMsg);
  /*  } */

    // start at the top
    display.clearDisplay();
    display.setCursor(0,0);

    display.println(currMsg);
    display.display();

    delay(100);
    digitalWrite(BUILTIN_LED, HIGH);

  }

}
