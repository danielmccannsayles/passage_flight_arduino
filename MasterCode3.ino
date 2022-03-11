// Motor/Pump Pins
int motor1pin1 = 7; int motor1pin2 = 8;

// UV Pins
int UVpin1=5; int UVpin2=6;

// Flowmeter pins
int flowPin = 0;    //This is the input pin on the Arduino
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
double kp = 3;
double ki = 9;

//PI Constant
double setPoint = 3; //set point to 3L/min
 
unsigned long currentTime, previousTime;
double elapsedTime, error, cumError, lastError, input, output;

float temperature = 25,tdsValue = 0;

//Bluetooth
#include <Wire.h>

#define DEBUG 1

// Library for virtual serial ports over normal pins
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(13, 12); //(rx, tx)

bool doUpdateStatus = false;


int x = 0;

//Loop Variables
int Res1 = 0; int Res2 = 0; //Reservoir Logic Gates
//Init to 0; Not touching

int Speed = 150; //Pump Speed
//Init Pump Speed to 150
//Max without membrane 6.75L/min 255 ENA
//Min without membrane 3L/min 150 ENA

int flowR; //Flowrate
int tdsVal; //TDS Value

void setup() {
  // Motor Code
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(9, OUTPUT);

  //UV Light
  pinMode(4,OUTPUT);

  // TDS Sensor
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization

  //Flowmeter
  pinMode(flowPin, INPUT);           //Sets the pin as an input
  attachInterrupt(0, Time, RISING);
  Serial.begin(9600);
}

//Program Loop and Logic

void loop() {
sensorRead();
  
//Reservoir Case Structures
if (Res1==0){ 
  pump_off();
  Serial.print("Filling Up Water...");
  UV_off;
  delay(1000);
}

else if (Res1==1){
  if (Res2==0){
    pump_on(Speed);
    //Pump PI Control
    if (flowR>1){
        Speed = computePI(flowR)/0.036; //Linear Interpolation of Speed Range for Pump (Slope: 0.036)
    }
    Serial.print("System Active");
    UV_off;
    delay(10000);
  }
  else if (Res2==1){
    pump_off();
    Serial.print("Stop Pouring Water!");
    delay(1000);
    for (int i = 0; i <= 5; i++) { //Pump off and wait 5 seconds for system to settle
      sensorRead();
      delay(1000);
    }
    UV_on();
    Serial.print("UV Filtering, Please Wait");
    for (int i=0; i<=200; i++){ //UV Filter on for 200 seconds
      delay(1000);
    }
    UV_off();
    Serial.print("Ready to Drink");
    }
  }

//Filter Life Case Structures
if (Speed>255){
  pump_off();
  Serial.print("Replace Micron Filter");
  delay(1000);
}
else if (Speed <150){
  Speed=150;
}
}

//Function Definitions
void Time()
{
   count++; //Every time this function is called, increment "count" by 1
}

void pump_on(int speed){   
  //Controlling speed (150 < Speed < 255)
  analogWrite(9, speed); //ENB pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
}

void pump_off(){
  analogWrite(9, 0); //ENB pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
}

void UV_on(){
  // UV (High/Low)
  analogWrite(4, 170); //ENA pin
  digitalWrite(UVpin1, HIGH);
  digitalWrite(UVpin2, LOW);
}

void UV_off(){ 
  analogWrite(4, 0); //ENA pin
  digitalWrite(UVpin1, LOW);
  digitalWrite(UVpin2, LOW);
}

int Flowmeter(){
  //Flowmeter
  int count;
  
  //Controlling spin direction of motors:
  count = 0;      // Reset the counter so we start counting from 0 again
  interrupts();   //Enables interrupts on the Arduino
  delay (1000);   //Wait 1 second
  noInterrupts(); //Disable the interrupts on the Arduino

  //Start the math
  int flowRate = (count * 2.25); //Take counted pulses in the last second and multiply by 2.25mL
  flowRate = flowRate * 60;   //Convert seconds to minutes, giving you mL / Minute
  flowRate = flowRate / 1000;   //Convert mL to Liters, giving you Liters / Minute
  return flowRate;
}

int TDS(){
  //TDS Sensor   
  gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate
  int tdsValue = gravityTds.getTdsValue();  // then get the value
  tdsValue=tdsValue*10/9;
  return tdsValue;
}

int WaterLevel(int pin) {
  int WaterHeight = analogRead(pin); // take a reading from the optical sensor pin
  
  Serial.print("Water Height Reading = ");   // Print string to console
  Serial.println(WaterHeight);   // the analog reading of the optical sensor
  
  if (WaterHeight < 100) {
    Serial.println(" - TOUCHING WATER - YES"); 
    return 1; //Touching Water
  }
  else if (WaterHeight < 400) {
    Serial.println(" - TOUCHING WATER - NO");
    return 0; //Not Touching Water 
  }
  else {
    Serial.println(" - TOUCHING WATER - NO");
    return 0; //Not Touching Water
  }
}

void sensorRead(){
  Serial.println("");
  Serial.println("");
//Sensor Readings
  Res1=WaterLevel(HeightPin1);
  Res2=WaterLevel(HeightPin2);
  flowR=Flowmeter();
  tdsVal=TDS();
  
  Serial.println("");

//Serial Print Readings
  Serial.print(flowR); //Plot Var
  Serial.print("L/min");
  Serial.println("");
  
  Serial.print(tdsVal); //Plot Var 
  Serial.print("ppm");
  Serial.println("");
}

int computePI(double inp){   
  currentTime = millis();  //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation
        
  error = setPoint - inp;  // determine error
  cumError += error * elapsedTime;  // compute integral 
  double out = kp*error + ki*cumError ;   //PI output            

  lastError = error;   //remember current error
  previousTime = currentTime;  //remember current time
 
  return out;  //have function return the PI output
}
