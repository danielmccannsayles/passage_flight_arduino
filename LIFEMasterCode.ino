// Motor/Pump Pins
int motor1pin1 = 7; int motor1pin2 = 8;
// UV Pins
int UVpin1 = 6; int UVpin2 = 5;

// Flowmeter pins
//#include "flowmeter.h"
int flowPin = 2;    //This is the input pin on the Arduino
double flowRate;    //This is the value we intend to calculate.
volatile int count;

// TDS setup
#include <EEPROM.h>
#include "GravityTDS.h"
#define TdsSensorPin A1 //Analog 1
GravityTDS gravityTds;

// Water Heighst Setup
int HeightPin1 = 3;  // the cell and 4.7K pulldown are connected to analog pin A0
int HeightPin2 = 4;
int WaterHeight1;  // variable to hold the optical float sensor
int WaterHeight2;


//PI Constants
double kp = 1.5;
double ki = 0.1;
double kd = 1.9;

//PI Constant
double setPoint = 2; //2L/min is ideal speed

unsigned long currentTime, previousTime;
double elapsedTime, startTime, error, cumError, lastError, input, out, rateError;

float temperature = 25, tdsValue = 0;

//Bluetooth
#include <Wire.h>

#define DEBUG 1

// Library for virtual serial ports over normal pins
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(12, 11); //(rx, tx)

bool doUpdateStatus = false;


int x = 0;

//Loop Variables
int Res1 = 0; int Res2 = 0; //Reservoir Logic Gates
//Init to 0; Not touching

int Speed = 0;//Pump Speed
//Init Pump Speed to 0
//Max without membrane 3 m/s 255 ENA
//Min without membrane 1 m/s 100 ENA

double flowR; //Flowrate
int tdsVal; //TDS Value

// used to update every second
static unsigned long lastRefreshTime = millis();

void setup() {
  // Motor Code
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(3, OUTPUT);

  //UV Light
  pinMode(9, OUTPUT);

  // TDS Sensor
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization

  //Flowmeter
  pinMode(flowPin, INPUT);           //Sets the pin as an input
  attachInterrupt(0, Time, RISING);

  Serial.begin(9600);
  bluetooth.begin(9600);

  bluetooth.write('t');
  bluetooth.write("hello");
}

//Program Loop and Logic

void loop() {
  sensorRead();

  //Reservoir Case Structures
  if (Res1 == 0) {
    pump_off();
    //serial.write("Filling Up Water...");
    //serial.println();
    UV_off;
    delay(1000);
  }

  else if (Res1 == 1) {

    //if (initialize_pump == 0) {
    //pump_on(255);
    //delay(5000);
    //int intialize_pump = 1;
    //pump_off();
    //}
    if (Res2 == 0) {
      startTime = millis() / 1000;
      pump_on(Speed);
      for (int i = 0; i <= 50; i++) {
        sensorRead();
        //Pump PI Control
        Speed = Speed + (computePI(flowRate) / 0.096); //Linearization of Speed Range for Pump (Slope: 0.096)
        //Serial.println(Speed);
        if (Speed >= 255) {
          pump_off();
          //serial.write("Replace Micron Filter");
          //serial.println();
          delay(30000);
          Speed = 0;
        }
        else if (Speed < 0) {
          Speed = 100;
        }
        pump_on(Speed);
        //serial.write("System Active");
        //serial.println();
        //serial.print(Speed);
        delay(1000);
        UV_on();
      }
    }
    //}
  }
  else if (Res2 == 1) {
    pump_off();
    UV_on();
    //serial.write("Stop Pouring Water!");
    //serial.println();
    for (int i = 0; i <= 5; i++) { //Pump off and wait 5 seconds for system to settle
      flowR = sensorRead();
      delay(1000);
    }

    //serial.write("UV Filtering, Please Wait");
    for (int i = 0; i <= 240; i++) { //UV Filter on for 240 seconds
      delay(1000);
    }
    UV_off();
    //serial.write("Ready to Drink");
  }

}

//Function Definitions
void Time()
{
  count++; //Every time this function is called, increment "count" by 1
}

void pump_on(int speed) {
  //Controlling speed (150 < Speed < 255)
  analogWrite(3, speed); //ENB pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
}

void pump_off() {
  analogWrite(3, 0); //ENB pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
}

void UV_on() {
  // UV (High/Low)
  analogWrite(9, 125); //ENA pin
  digitalWrite(UVpin1, HIGH);
  digitalWrite(UVpin2, LOW);
}

void UV_off() {
  analogWrite(9, 0); //ENA pin
  digitalWrite(UVpin1, LOW);
  digitalWrite(UVpin2, LOW);
}

double Flowmeter() {
  //Flowmeter

  //Controlling spin direction of motors:
  count = 0;      // Reset the counter so we start counting from 0 again
  interrupts();   //Enables interrupts on the Arduino
  delay (1000);   //Wait 1 second
  noInterrupts(); //Disable the interrupts on the Arduino

  //Start the math
  flowRate = (count * 2.25); //Take counted pulses in the last second and multiply by 2.25mL
  flowRate = flowRate * 60;         //Convert seconds to minutes, giving you mL / Minute
  flowRate = flowRate / 1000;

  //Serial Print Readings
  //Serial.print(flowRate); //Plot Var
  //Serial.write(" L/min");
  //Serial.println();
  return flowRate;
}

int TDS() {
  //TDS Sensor
  gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate
  int tdsValue = gravityTds.getTdsValue();  // then get the value
  //Serial.print(tdsVal); //Plot Var
  //Serial.write("ppm");
  //Serial.println();
  return tdsValue;
}

int WaterLevel(int pin) {
  int WaterHeight = analogRead(pin); // take a reading from the optical sensor pin

  //Serial.print("Water Height Reading = ");   // Print string to console
  //Serial.println(WaterHeight);   // the analog reading of the optical sensor

  if (WaterHeight < 500) {
    //Serial.write(" - TOUCHING WATER - YES");
    //Serial.println();
    return 1; //Touching Water
  }
  else {
    //Serial.write(" - TOUCHING WATER - NO");
    //Serial.println();
    return 0; //Not Touching Water
  }
}

double sensorRead() {
  //Sensor Readings
  Res1 = WaterLevel(HeightPin1);
  Res2 = WaterLevel(HeightPin2);
  //Serial.println();
  flowR = Flowmeter();
  tdsVal = TDS();

  //Serial.write("----------");
  //Serial.println();

  sendData();
}

double computePI(double inp) {

  currentTime = (millis() / 1000) - startTime; //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation

  error = setPoint - inp;  // determine error
  cumError += error * elapsedTime;  // compute integral
  rateError = (error - lastError)/elapsedTime;
  out = (kp * error) + (ki * cumError) + (kd*rateError); //PI output
  //Serial.write("Error");
  //Serial.println(error);
  //Serial.write("CError");
  //Serial.println(cumError);
  lastError = error;   //remember current error
  previousTime = currentTime;  //remember current time
  //serial.write("Out");
  //serial.println(out);

  return out;  //have function return the PI output
}

void sendData()
{
  // ORIGINAL CODE: keep it the same until I understand how exactly it works
  //  Every update there are temperatures and water pH level sent coded into binary form of:
  //  't',
  //  integer part of value in Celcius of first termometer,
  //  fractional part of value in Celcius of first termometer,
  //  integer part of value in Celcius of second termometer,
  //  fractional part of value in Celcius of second termometer,
  //  'w'
  //  integer value of water pH level,
  //  fractional part of water pH level.

  bluetooth.write('t');

  bluetooth.write(static_cast<byte>(static_cast<int>(tdsVal)));
  bluetooth.write(static_cast<byte>(static_cast<int>((tdsVal - static_cast<int>(tdsVal)) * 100)));

  bluetooth.write(static_cast<byte>(static_cast<int>(flowR)));
  bluetooth.write(static_cast<byte>(static_cast<int>((flowR - static_cast<int>(flowR)) * 100)));

  bluetooth.write('w');
  bluetooth.write(static_cast<byte>(static_cast<int>(Res1)));
  bluetooth.write(static_cast<byte>(static_cast<int>(Res2)));

}
