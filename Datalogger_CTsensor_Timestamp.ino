#include <SPI.h>
#include <SD.h>
#include "RTClib.h"

RTC_PCF8523 rtc;    //Pcf8523 RTC that we are using in Adalogger ( Website I have searched)

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const int ACPin = A0; //set esp signal read pin
const int analogPin = A0;

#define ACTectionRange 20;    //set Non-invasive AC Current Sensor tection range (5A,10A,20A)  Standard defination from data sheet.
#define VREF 3.26          // My esp ref voltage, checked by multimeter  (3.3 usually, but when I check in multimeter)             

const int chipSelect = 15;  //Pin 15 from Schematic

float Rate_of_Current_Value_per_min = 0;
float Current_Value = 0;

float readACCurrentValue()
{
  float ACCurrtntValue = 0;
  float peakVoltage = 0;
  float voltageVirtualValue = 0;  //Vrms
  for (int i = 0; i < 5; i++)
  {
    peakVoltage += analogRead(ACPin);   //read peak voltage
    delay(1);
  }
  peakVoltage = peakVoltage / 5;
  voltageVirtualValue = peakVoltage * 0.707; 
    /*If the circuit is amplified by n times, so we sld divided by n. For now I'm assuming it as 2. And should study circuit schematic and discuss with Gokul */
  voltageVirtualValue = (voltageVirtualValue / 1024 * VREF ) / 2;

  ACCurrtntValue = voltageVirtualValue * ACTectionRange;   // Standard eqation of CT sensor from Datasheet

  return ACCurrtntValue;
}
void setup() 
{
  
  Serial.begin(115200);
  Serial.print("Initializing SD card...");


  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) 
  {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  if (! rtc.begin()) 
  {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.initialized()) 
   {
     Serial.println("RTC is NOT running!");
     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   }
}

void loop() 
{

  //for (int k = 0; k < 60 ; k++)
  //{
   
    DateTime now = rtc.now();  

    int y=now.year();
    String yer = String(y);    //converting each values into string
    int m=now.month();
    String mnth = String(m);
    int d=now.day();
    String dy = String(d);
    int h=now.hour();
    String hr = String(h);
    int mi = now.minute();
    String mint = String(mi);
    int s = now.second();
    String secn = String(s);

     
    
    float ACCurrentValue = readACCurrentValue(); //read AC Current Value
    String dataString = "";
    dataString += String(ACCurrentValue);
    File dataFile = SD.open("CTsensor.txt", FILE_WRITE);
    if (dataFile) 
    {
    dataFile.println(dataString +"/"+yer+"/"+mnth+"/"+dy+"/"+hr+"/"+mint+"/"+secn);   // String addiction, seperation using "/"
    dataFile.close();
   
    // print to the serial port too for cross verify. 
    Serial.println(dataString);
 
    }
  
  // if the file isn't open, show up an error:
    else 
    {
    Serial.println("error opening CTsensor.txt");
    }
    Current_Value += ACCurrentValue;
    delay(1000);
  //}

  Rate_of_Current_Value_per_min = Current_Value / 60;   //If required per min
  Rate_of_Current_Value_per_min = 0;
  Current_Value = 0;
  
  delay(3000);
}
