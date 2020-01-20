#include <FS.h>
#include "RTClib.h"
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

bool shouldSaveConfig = false;
RTC_PCF8523 rtc;

#include <ArduinoJson.h>
#include <Arduino.h>
#include <Stream.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <Wire.h>
#include <energyic_UART.h>
#if !defined(ARDUINO_ARCH_SAMD)
#include <SoftwareSerial.h>
#else

#endif

struct clientData {
  int Vrms1[8];
  int Crms1[8];
  int id[8];
  int realPower1[8];
  int powerFactor[8];
  int freq1[8];
  int yer[8];
  int mnth[8];
  int dy[8];
  int hr[8];
  int minu[8];
  int sec[8];

};
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//AWS
#include "sha256.h"
#include "Utils.h"

//WEBSockets
#include <Hash.h>
#include <WebSocketsClient.h>

//MQTT PAHO
#include <SPI.h>
#include <IPStack.h>
#include <Countdown.h>
#include <MQTTClient.h>

//AWS MQTT Websocket
#include "Client.h"
#include "AWSWebSocketClient.h"
#include "CircularByteBuffer.h"

#if defined(ESP8266)
SoftwareSerial ATMSerial(13, 14, false, 256);
#endif

#ifdef AVR_ATmega32U4 //32u4 board
SoftwareSerial ATMSerial(11, 13); //RX, TX
#endif

#if defined(ARDUINO_ARCH_SAMD)
#include "wiring_private.h" // pinPeripheral() function
//Feather M0 
#define PIN_SerialATM_RX       12ul
#define PIN_SerialATM_TX       11ul
#define PAD_SerialATM_RX       (SERCOM_RX_PAD_3)
#define PAD_SerialATM_TX       (UART_TX_PAD_0)

// Using SERCOM1 on M0 to communicate with ATM90E26
Uart ATMSerial(&sercom1, PIN_SerialATM_RX, PIN_SerialATM_TX, PAD_SerialATM_RX, PAD_SerialATM_TX);
#endif

ATM90E26_UART eic(&ATMSerial);

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


char auth[36] = "";
char server[50] = "";

//  --------- Config ---------- //
//AWS IOT config, change these:
//char wifi_ssid[]       = "Devil's Home";
//char wifi_password[]   = "password@ml";
char aws_endpoint[]    = "";
char aws_key[]         = "";
char aws_secret[]      = "";
char aws_region[]      = "";
const char* aws_topic  = "";
int port = 443;

// If stuff isn't working right, watch the console:
#define DEBUG_PRINT 1      //default debugging process



//MQTT config
const int maxMQTTpackageSize = 512;   //max package size you can transfer
const int maxMQTTMessageHandlers = 1;    // comes along mqtt broker
// ---------- /Config ----------//

//DHT dht(DHTPIN, DHTTYPE);
ESP8266WiFiMulti WiFiMulti;

AWSWebSocketClient awsWSclient(256,5000);    // amount of max data you can transfer, can be changed accordingly

IPStack ipstack(awsWSclient);

MQTT::Client<IPStack, Countdown, maxMQTTpackageSize, maxMQTTMessageHandlers> *client = NULL;

//# of connections
long connection = 0;

//generate random mqtt clientID
char* generateClientID () {
  char* cID = new char[23]();
  for (int i=0; i<22; i+=1)
    cID[i]=(char)random(1, 256);
  return cID;
}

//count messages arrived
int arrivedcount = 0;

//callback to handle mqtt messages
void messageArrived(MQTT::MessageData& md)
{
  MQTT::Message &message = md.message;

  if (DEBUG_PRINT) {                                                                     // general acknowldegment what will reflect in aws
    Serial.print("Message ");
    Serial.print(++arrivedcount);
    Serial.print(" arrived: qos ");
    Serial.print(message.qos);
    Serial.print(", retained ");
    Serial.print(message.retained);
    Serial.print(", dup ");
    Serial.print(message.dup);
    Serial.print(", packetid ");
    Serial.println(message.id);
    Serial.print("Payload ");
    char* msg = new char[message.payloadlen+1]();
    memcpy (msg,message.payload,message.payloadlen);
    Serial.println(msg);



//--------------------------------------------------------------------------

 // char JSONMessage[] = "{\"state\":{\"reported\":{\"test_value1\":297, \"test_value2\":123}}}";
  Serial.print("Initial string value: ");
  Serial.println(msg);
 
 StaticJsonBuffer<300> JSONBuffer;   //Memory pool
  JsonObject& parsed = JSONBuffer.parseObject(msg); //Parse message
 
  if (!parsed.success()) {   //Check for errors in parsing
 
    Serial.println("Parsing failed");
    delay(5000);
    return;
 
  }
 
//  const char * sensorType = parsed["SensorType"]; //Get sensor type value
  ///int value = parsed["Value"];                                         //Get value of sensor measurement
 
//  Serial.println(sensorType);
// Serial.println(value);

  const char* Vrms1 = parsed["state"]["reported"]["Vrms1"];
  const char* Crms1 = parsed["state"]["reported"]["Crms1"];
 // const char* Rtc1 = parsed["state"]["reported"]["Rtc1"];


      
  int convVrms=atoi(Vrms1);            //convert json into integer

  Serial.print("The Voltage is ");
  Serial.println(convVrms);

  int convCrms=atoi(Crms1);            //convert json into integer

  Serial.print("The Current is ");
  Serial.println(convCrms);

  }
}

//connects to websocket layer and mqtt layer
bool connect () {

    if (client == NULL) {
      client = new MQTT::Client<IPStack, Countdown, maxMQTTpackageSize, maxMQTTMessageHandlers>(ipstack);
    } else {

      if (client->isConnected ()) {    
        client->disconnect ();
      }  
      delete client;
      client = new MQTT::Client<IPStack, Countdown, maxMQTTpackageSize, maxMQTTMessageHandlers>(ipstack);
    }


    //delay is not necessary... it just help us to get a "trustful" heap space value
    delay (1500);
    if (DEBUG_PRINT) {
      Serial.print (millis ());
      Serial.print (" - conn: ");
      Serial.print (++connection);
      Serial.print (" - (");
      Serial.print (ESP.getFreeHeap ());
      Serial.println (")");
    }

   int rc = ipstack.connect(aws_endpoint, port);
    if (rc != 1)
    {
      if (DEBUG_PRINT) {
        Serial.println("error connection to the websocket server");
      }
      return false;
    } else {
      if (DEBUG_PRINT) {
        Serial.println("websocket layer connected");
      }
    }
    
    if (DEBUG_PRINT) {
      Serial.println("MQTT connecting");
    }
    
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    char* clientID = generateClientID ();
    data.clientID.cstring = clientID;
    rc = client->connect(data);
    delete[] clientID;
    if (rc != 0)
    {
      if (DEBUG_PRINT) {
        Serial.print("error connection to MQTT server");
        Serial.println(rc);
        return false;
      }
    }
    if (DEBUG_PRINT) {
      Serial.println("MQTT connected");
    }
    return true;
}


//subscribe to a mqtt topic
void subscribe () {
   //subscrip to a topic
    int rc = client->subscribe(aws_topic, MQTT::QOS0, messageArrived);
    if (rc != 0) {
      if (DEBUG_PRINT) {
        Serial.print("rc from MQTT subscribe is ");
        Serial.println(rc);
      }
      return;
    }
    if (DEBUG_PRINT) {
      Serial.println("MQTT subscribed");
    }
}


void setup() {

#ifndef ESP8266
while (!Serial); // for Leonardo/Micro/Zero
#endif
    Serial.begin (115200);   
  delay(1000); // wait for console opening

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  //if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    //rtc.adjust(DateTime(2020, 1, 20), (5, 00, 00)); 
  
WiFiManagerParameter custom_ts_token("ts", "", auth, 33);
WiFiManagerParameter custom_server("serv", "", server, 50);


WiFiManager wifiManager;
wifiManager.setDebugOutput(false);

wifiManager.setSaveConfigCallback(saveConfigCallback);

wifiManager.addParameter(&custom_ts_token);
wifiManager.addParameter(&custom_server);

wifiManager.autoConnect("Minion", "password");

Serial.println("connected...yeey :");

Serial.print("Key:");
Serial.println(auth);
Serial.print("Server:");
Serial.println(server);
strcpy(auth, custom_ts_token.getValue());
strcpy(server, custom_server.getValue());


    //fill AWS parameters    
    awsWSclient.setAWSRegion(aws_region);
    awsWSclient.setAWSDomain(aws_endpoint);
    awsWSclient.setAWSKeyID(aws_key);
    awsWSclient.setAWSSecretKey(aws_secret);
    awsWSclient.setUseSSL(true);


}


void loop() {

 unsigned short s_status = eic.GetSysStatus();
  if(s_status == 0xFFFF)
  {
  #if defined(ESP8266)
    //Read failed reset ESP, this is heavy
    ESP.restart();
  #endif
  }
  DateTime now = rtc.now();
 
 // DateTime future (now + TimeSpan(7,12,30,6));
  
  float id = ESP.getChipId();
  float Vrms1 = eic.GetLineVoltage();
  float Crms1 = eic.GetLineCurrent();
  float realPower1 = eic.GetActivePower();
  float powerFactor1 = eic.GetPowerFactor();
  float freq1 = eic.GetFrequency();
  float yer = now.year();
  float mnth = now.month();
  float dy = now.day();
  float hr = now.hour();
  float minu = now.minute();
  float sec = now.second();


 String aa = String(id);
 String bb = String(Vrms1);
 String cc = String(Crms1);
 String dd = String(realPower1);
 String ee = String(powerFactor1);
 String ff = String(freq1);
 
 String gg = String(yer);
 String hh = String(mnth);
 String ii = String(dy);
 String jj = String(hr);
 String kk = String(minu);
 String ll = String(sec);


 //String str = String(now.year(), DEC) + '/' + String(now.month(), DEC) + '/' + String(now.day(), DEC) + " " + String(now.hour(), DEC) + ':' + String(now.minute(), DEC) + ':' + String(now.second(), DEC);

 String values = "{\"state\":{\"reported\":{\"Id\":"+aa+",\"Voltage\":"+bb+",\"Current\":"+cc+",\"Active Power\":"+dd+",\"Power Factor\":"+ee+",\"Frequency\":"+ff+",\"Year\":"+gg+",\"Month\":"+hh+",\"Day\":"+ii+",\"Hour\":"+jj+",\"Minute\":"+kk+",\"Second\":"+ll+"}}}";             // where publishing happens
 
  // http://stackoverflow.com/questions/31614364/arduino-joining-string-and-char
  const char *publish_message = values.c_str();

  //keep the mqtt up and running
  if (awsWSclient.connected ()) {    
    
      client->yield();
      
    //  subscribe (); 
      //publish 
      MQTT::Message message;
      char buf[1000];            //if you have large number of data, you can change the buffer size 
      strcpy(buf, publish_message);
      message.qos = MQTT::QOS0;
      message.retained = false;
      message.dup = false;
      message.payload = (void*)buf;
      message.payloadlen = strlen(buf)+1;
      int rc = client->publish(aws_topic, message);  
  } else {
    //handle reconnection
    connect ();
  }  
}
