
//Code from Edison
#include <EEPROM.h>
#include "GravityTDS.h"

#define TdsSensorPin A0
GravityTDS gravityTds;

float temperature = 25,tdsValue = 0;


//Other Code
#include <Wire.h>

#define DEBUG 1

// Library for virtual serial ports over normal pins
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(13, 12); //(rx, tx)

bool doUpdateStatus = false;


int x = 0;
////////////////////////////////////////////////////////////////////////////////
void loop(void)
{
  delay(1);
  
  
  // Commands: "start", "stop" - can be changed to anything lol
  while (bluetooth.available() >= 4) {
    switch (bluetooth.read()) {
      case 's':
        bluetooth.read(); // Ignore (probably) 't'
        switch (bluetooth.read()) {
          case 'a': // "start"
            doUpdateStatus = true;
            digitalWrite(13, HIGH);
            
            bluetooth.read(); // Ignore 'r'
            while (bluetooth.available() == 0);
            bluetooth.read(); // Ignore 't'
            break;
            
          case 'o': // "stop"
            doUpdateStatus = false;
            digitalWrite(13, LOW);
            
            bluetooth.read(); // Ignore 'p'
        }
        break;
    }
  }
  
	static unsigned long lastRefreshTime = 0;
	if (millis() - lastRefreshTime >= 1000) {
    //increment the random x counter
    x++;
    if (x>=100){
    x=0;
    }

    //read the value of the sensor
    gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate
    tdsValue = gravityTds.getTdsValue();  // then get the value
    Serial.print(tdsValue,0);
    Serial.println("hi");
  
    lastRefreshTime += 1000;
    
    if (doUpdateStatus) {
      //ORIGINAL CODE: keep it the same until I understand how exactly it works
      // Every update there are temperatures and water pH level sent coded into binary form of:
      // 't', 
      // integer part of value in Celcius of first termometer, 
      // fractional part of value in Celcius of first termometer,
      // integer part of value in Celcius of second termometer, 
      // fractional part of value in Celcius of second termometer,
      // 'w'
      // integer value of water pH level, 
      // fractional part of water pH level.
      
      bluetooth.write('t'); 
     for (byte i = 0; i < 2; i++) {
        bluetooth.write(static_cast<byte>(static_cast<int>(tdsValue)));
        bluetooth.write(static_cast<byte>(static_cast<int>((tdsValue - static_cast<int>(tdsValue)) * 100)));
      }

      bluetooth.write('w');
      bluetooth.write(static_cast<byte>(static_cast<int>(10)));
      bluetooth.write(static_cast<byte>(static_cast<int>((10 - static_cast<int>(10)) * 100)));
    }
	}
}


////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  Serial.begin(9600);
  
  bluetooth.begin(9600);

  //setting up the tds sensor
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}
