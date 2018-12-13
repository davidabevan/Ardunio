// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

#include "DHT.h"

#define DHTPIN 2     // what digital pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);
  int dehumidifier = 4;
//int heater = 2;
int led = 3;
int i = 0;
 int relaystate = digitalRead(dehumidifier); 
//if relaystate =0 relaystate =on;
const int  HumidLeeway = 1;
int SetPoint = 69; //57;//Goes on at 59 and off at 55
//int  hum = (h);
void setup() {
  pinMode(dehumidifier, OUTPUT);
 // pinMode(heater, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(5,OUTPUT);// Set D5 up as GND
  digitalWrite(5,LOW);
  pinMode(6,OUTPUT);
  digitalWrite(6,HIGH);
  Serial.begin(9600);
  Serial.println("DHTxx test!");

  dht.begin();
}

void loop() {

  // Wait a few seconds between measurements.
  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = 34+dht.readHumidity();//DHT 11 Adjustment for low readings
  // Read temperature as Celsius (the default)
  float t = -8+ dht.readTemperature ();//calibration
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
 int relaystate = digitalRead(dehumidifier); 
  Serial.print("  Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F"); 
  Serial.print("RelayState :"); 
Serial.print(relaystate);
//  RELAY TRIGGER 1
 // if(h < SetPoint-HumidLeeway) {// Relay Debounce
 //if (h >= SetPoint) {
   // digitalWrite(dehumidifier, LOW);
 // } 
 // else  
 // {
   // digitalWrite(dehumidifier, HIGH);
   // }
   
  //Relay with Debounce
  if(h < SetPoint-HumidLeeway) 
  {
  digitalWrite(dehumidifier, HIGH);
  }
if(h > SetPoint+HumidLeeway) 
  {
  digitalWrite(dehumidifier, LOW);
  }
    }
  //if (DHT.temperature<18) {
  //if (theTemperature<18) {
    //digitalWrite(heater,LOW);
  //} else {
    //digitalWrite(heater,HIGH);
  
  

//  i=0;
//  while (i<10) {
//    digitalWrite(led,HIGH);
//    delay(1000);
//    digitalWrite(led,LOW);
//    delay(1000);
//    i++;
//  }
