#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <PID_v1.h>

#define RelayPin A0 //SSR input


//Define Variables 
double Input, Output;
double Setpoint = 100;

//Specify the links and initial tuning parameters
PID myPID(&Input,  &Output, &Setpoint, 2, 5, 1, DIRECT);

int WindowSize = 5000; //5 seconds
unsigned long windowStartTime;


// Create thermocouple instances
#define MAX1DO   4
#define MAX1CS   3
#define MAX1CLK  2

#define MAX2DO   7
#define MAX2CS   6
#define MAX2CLK  5

#define MAX3DO   10
#define MAX3CS   9
#define MAX3CLK  8

#define MAX4DO   13
#define MAX4CS   12
#define MAX4CLK  11


// initialize the Thermocouples
Adafruit_MAX31855 thermocouple1(MAX1CLK, MAX1CS, MAX1DO);
Adafruit_MAX31855 thermocouple2(MAX2CLK, MAX2CS, MAX2DO);
Adafruit_MAX31855 thermocouple3(MAX3CLK, MAX3CS, MAX3DO);
Adafruit_MAX31855 thermocouple4(MAX4CLK, MAX4CS, MAX4DO);

Adafruit_ADS1115 ads(0x48);

float Voltage = 0.0;

int thermistor_25 = 10000;

float refCurrent = 0.0001;

void setup() {
  Serial.begin(9600);
  
  ads.begin(); //16-bit ADC
  
  while (!Serial) delay(1); 

  Serial.println("TEMPERATURE READINGS");
  // wait for MAX chip to stabilize
  delay(1000);

  //Solid State Relay
  pinMode(RelayPin, OUTPUT);
  
  windowStartTime = millis();

  //initialize the variables we're linked to

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {

  int16_t adc0; // we read from the ADC, we have a sixteen bit integer as a result

  adc0 = ads.readADC_SingleEnded(0); // Read ADC value from ADS1115

  Voltage = adc0 * (5.0 / 65535); // Replace 5.0 with whatever the actual Vcc of your Arduino is

  float resistance = (Voltage / refCurrent); // Using Ohm's Law to calculate resistance of thermistor

   // basic readout test, just print the current temp
   Serial.print("\nInternal Temp 1= ");
   Serial.println(thermocouple1.readInternal(), " \n");
   Serial.print("Internal Temp 2= ");
   Serial.println(thermocouple2.readInternal(), " \n");
   Serial.print("Internal Temp 3= ");
   Serial.println(thermocouple3.readInternal(), " \n");
   Serial.print("Internal Temp 4= ");
   Serial.println(thermocouple4.readInternal(), " \n");

   double c1 = thermocouple1.readCelsius();
   if (isnan(c1)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("C1 = "); 
     Serial.println(c1);
     Serial.print("AIN0 1:  "); // Print ADC value to Serial Monitor
     Serial.print( adc0);
   }

   double c2 = thermocouple2.readCelsius();
   if (isnan(c2)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("\n C2 = "); 
     Serial.println(c2);
     Serial.print("AIN0 2: "); 
     Serial.print(adc0);
   }
   
   double c3 = thermocouple3.readCelsius();
   if (isnan(c3)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("\n C3 = "); 
     Serial.println(c3);
     Serial.print("AIN0 3: "); 
     Serial.print(adc0);
   }
   
    double c4 = thermocouple4.readCelsius();
   if (isnan(c4)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("\n C4 = "); 
     Serial.println(c4);
     Serial.print("AIN0 4: "); 
     Serial.print(adc0);
   }
   
 
   delay(1000);

   
   //SSR 

  Input = (c1+c2+c3+c4)/4; //Finds the average temperature from all four thermocouples 
                           //Adjust precision estimates for marginally increased uncertainty 
  myPID.Compute();

  
  //turn the output pin on/off based on pid output

  unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output > now - windowStartTime) 
  {
    digitalWrite(RelayPin, HIGH);
    Serial.print("\n ON");
  }
  else 
  {
    digitalWrite(RelayPin, LOW);
    Serial.print("\n OFF");
  }
  
}
