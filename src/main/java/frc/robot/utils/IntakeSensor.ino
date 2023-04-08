//version 2 2/3/23 display rolling average 
//This example shows how to use continuous mode to take
//range measurements with the VL53L0X. It is based on
//vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.
//The range readings are in units of mm. 

#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_LEDBackpack.h>

VL53L0X sensor;
const int ledPin = 13;
const int distArraySize = 10;
const int MAX_VALID_DISTANCE = 1000;  // observed 8190-8191 when out of range 
unsigned int currentDistance = 0;
unsigned int avgDistance = 0;
unsigned int distanceArray[] = {0,0,0,0,0, 0,0,0,0,0};  // array size 10

Adafruit_7segment matrix = Adafruit_7segment(); 

void setupLEDmatrix() {
#ifndef __AVR_ATtiny85__
  Serial.println("7 Segment Backpack Test");
#endif
  matrix.begin(0x71);
}
void displayLEDmatrix(int n) {
    matrix.println(n);
    matrix.writeDisplay();
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
  setupLEDmatrix();
  displayLEDmatrix(-9);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
  displayLEDmatrix(-8);
}

void displaySerial(unsigned int distance) {
  if( distance < 1000 ) {
    Serial.println(distance);
  }
  if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
}

void displayDEBUGavg(unsigned int current) {
  if( current > 0 && current <= MAX_VALID_DISTANCE ) {
  Serial.print("INVALID "); Serial.print(current); 
  Serial.print(". VALID array=");
  for( int i=0; i<distArraySize; i++ ) {
    Serial.print(distanceArray[i]); Serial.print(" ");
  }
  Serial.println(".");
  }
}
  
void displayAvg( unsigned int distance ) {
  if( distance > 0 && distance <= 8189 ) {
    digitalWrite( ledPin, HIGH );
    displayLEDmatrix(distance);    
  } else {
    digitalWrite( ledPin, LOW );
    displayLEDmatrix(-1);
  }
}

unsigned int getAvg( unsigned int current, uint32_t count ) {
  int index = count % distArraySize;   // range 0 to 9 
  unsigned int avg=0;
  distanceArray[index] = current;
  for( int i=0; i<distArraySize; i++ ) {
    avg += distanceArray[i];    
  }
  avg /= distArraySize;
  return avg;
}

uint32_t loopCount=0;
void loop()
{
  currentDistance= sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) { 
    displaySerial(9999); 
  } else {
//    displaySerial( currentDistance );
    avgDistance = getAvg(currentDistance, loopCount++);
    displayAvg( avgDistance );
  }
  delay(100);
}