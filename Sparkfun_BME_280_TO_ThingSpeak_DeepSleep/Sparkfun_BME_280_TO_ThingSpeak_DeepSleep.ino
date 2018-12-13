/******************************************************************************
I2C_ReadAllData.ino
BME280 Arduino and Teensy example
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/SparkFun_BME280_Arduino_Library

This sketch configures the BME280 to read all measurements.  The sketch also
displays the BME280's physical memory and what the driver perceives the
calibration words to be.

Resources:
Uses Wire.h for I2C operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
#include <ESP8266WiFi.h>
//const char* ssid     = "TNCAPC2653F";
//const char* password = "A7252C69DF";
const char* ssid     = "PLUSNET-9P9QR6";
const char* password = "3bb7bc7cd3";
const char* host = "api.thingspeak.com";
//int counter;//Thingspeak counter
//const char* thingspeak_key = "6538YM84385NBODK";//St_Blazey
//const char* thingspeak_key = "XWYQUSZFRTC4VYRM";//Stenalees
const char* thingspeak_key = "G0A35KRSSPS2RA53";//Stenalees
#include <stdint.h>
#include "SparkFunBME280.h"
//Library allows either I2C or SPI, so include both.
#include "Wire.h"
#include "SPI.h"
extern "C"{
#include "user_interface.h"//Needed for DeepSleep
}
//Global sensor object
BME280 mySensor;
//Conect pins reset and D0 via 1k resistor on wemos D1
void turnOff(int pin) {//Shuts power to pins off
  pinMode(pin, OUTPUT);
  digitalWrite(pin, 1);
}
void setup()
{
  int counter=0;
    // disable all output to save power
  //turnOff(0); //Not these two as they run I2C
 // turnOff(2);
  turnOff(4);
  turnOff(5);
  turnOff(12);
  turnOff(13);
  turnOff(14);
  turnOff(15);
	//***Driver settings********************************//
	//commInterface can be I2C_MODE or SPI_MODE
	//specify chipSelectPin using arduino pin names
	//specify I2C address.  Can be 0x77(default) or 0x76
	//Default I2C adress used, but mod .cpp for wemos d1 mini to use (0, 2)
	//For I2C, enable the following and disable the SPI section
	mySensor.settings.commInterface = I2C_MODE;
	mySensor.settings.I2CAddress = 0x76;
	
	//For SPI enable the following and dissable the I2C section
	//mySensor.settings.commInterface = SPI_MODE;
	//mySensor.settings.chipSelectPin = 10;


	//***Operation settings*****************************//
	
	//renMode can be:
	//  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
	mySensor.settings.runMode = 3; //Normal mode
	
	//tStandby can be:
	//  0, 0.5ms
	//  1, 62.5ms
	//  2, 125ms
	//  3, 250ms
	//  4, 500ms
	//  5, 1000ms
	//  6, 10ms
	//  7, 20ms
	mySensor.settings.tStandby = 0;
	
	//filter can be off or number of FIR coefficients to use:
	//  0, filter off
	//  1, coefficients = 2
	//  2, coefficients = 4
	//  3, coefficients = 8
	//  4, coefficients = 16
	mySensor.settings.filter = 0;
	
	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.tempOverSample = 1;

	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.pressOverSample = 1;
	
	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.humidOverSample = 1;
	
	Serial.begin(9600);
	Serial.print("Program Started\n");
	Serial.print("Starting BME280... result of .begin(): 0x");
	//Calling .begin() causes the settings to be loaded
	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
	Serial.println(mySensor.begin(), HEX);//Specify i2c pins in cpp for Wemos D1 mini 0 2

	Serial.print("Displaying ID, reset and ctrl regs\n");
	
	Serial.print("ID(0xD0): 0x");
	Serial.println(mySensor.readRegister(BME280_CHIP_ID_REG), HEX);
	Serial.print("Reset register(0xE0): 0x");
	Serial.println(mySensor.readRegister(BME280_RST_REG), HEX);
	Serial.print("ctrl_meas(0xF4): 0x");
	Serial.println(mySensor.readRegister(BME280_CTRL_MEAS_REG), HEX);
	Serial.print("ctrl_hum(0xF2): 0x");
	Serial.println(mySensor.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);

	Serial.print("\n\n");

	Serial.print("Displaying all regs\n");
	uint8_t memCounter = 0x80;
	uint8_t tempReadData;
	for(int rowi = 8; rowi < 16; rowi++ )
	{
		Serial.print("0x");
		Serial.print(rowi, HEX);
		Serial.print("0:");
		for(int coli = 0; coli < 16; coli++ )
		{
			tempReadData = mySensor.readRegister(memCounter);
			Serial.print((tempReadData >> 4) & 0x0F, HEX);//Print first hex nibble
			Serial.print(tempReadData & 0x0F, HEX);//Print second hex nibble
			Serial.print(" ");
			memCounter++;
		}
		Serial.print("\n");
	}
	
	
	Serial.print("\n\n");
	
	Serial.print("Displaying concatenated calibration words\n");
	Serial.print("dig_T1, uint16: ");
	Serial.println(mySensor.calibration.dig_T1);
	Serial.print("dig_T2, int16: ");
	Serial.println(mySensor.calibration.dig_T2);
	Serial.print("dig_T3, int16: ");
	Serial.println(mySensor.calibration.dig_T3);
	
	Serial.print("dig_P1, uint16: ");
	Serial.println(mySensor.calibration.dig_P1);
	Serial.print("dig_P2, int16: ");
	Serial.println(mySensor.calibration.dig_P2);
	Serial.print("dig_P3, int16: ");
	Serial.println(mySensor.calibration.dig_P3);
	Serial.print("dig_P4, int16: ");
	Serial.println(mySensor.calibration.dig_P4);
	Serial.print("dig_P5, int16: ");
	Serial.println(mySensor.calibration.dig_P5);
	Serial.print("dig_P6, int16: ");
	Serial.println(mySensor.calibration.dig_P6);
	Serial.print("dig_P7, int16: ");
	Serial.println(mySensor.calibration.dig_P7);
	Serial.print("dig_P8, int16: ");
	Serial.println(mySensor.calibration.dig_P8);
	Serial.print("dig_P9, int16: ");
	Serial.println(mySensor.calibration.dig_P9);
	
	Serial.print("dig_H1, uint8: ");
	Serial.println(mySensor.calibration.dig_H1);
	Serial.print("dig_H2, int16: ");
	Serial.println(mySensor.calibration.dig_H2);
	Serial.print("dig_H3, uint8: ");
	Serial.println(mySensor.calibration.dig_H3);
	Serial.print("dig_H4, int16: ");
	Serial.println(mySensor.calibration.dig_H4);
	Serial.print("dig_H5, int16: ");
	Serial.println(mySensor.calibration.dig_H5);
	Serial.print("dig_H6, uint8: ");
	Serial.println(mySensor.calibration.dig_H6);
		
	Serial.println();
  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

int value = 0;

 int counter = 0;// Thingspeak counter
void loop()
{
  counter ++;
	//Each loop, take a reading.
	//Start with temperature, as that data is needed for accurate compensation.
	//Reading the temperature updates the compensators of the other functions
	//in the background.

	Serial.print("Temperature: ");
	Serial.print(mySensor.readTempC(), 2);
	Serial.println(" degrees C");

	Serial.print("Temperature: ");
	Serial.print(mySensor.readTempF(), 2);
	Serial.println(" degrees F");

	Serial.print("Pressure: ");
	Serial.print(mySensor.readFloatPressure(), 2);
	Serial.println(" Pa");

	Serial.print("Altitude: ");
	Serial.print(mySensor.readFloatAltitudeMeters(), 2);
	Serial.println("m");

	Serial.print("Altitude: ");
	Serial.print(mySensor.readFloatAltitudeFeet(), 2);
	Serial.println("ft");	

	Serial.print("%RH: ");
	Serial.print(mySensor.readFloatHumidity(), 2);
	Serial.println(" %");
	 Serial.println("counter");//Debug
    Serial.println(counter);
	Serial.println();
	 delay(5000);
  ++value;

  Serial.print("connecting to ");
  Serial.println(host);
  
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }

  String temp = String(mySensor.readTempC());
  String humidity = String(mySensor.readFloatHumidity());
  String pressure = String(mySensor.readFloatPressure());
  String count = String(counter);
  String url = "/update?key=";
  url += thingspeak_key;
  url += "&field1=";
  url += temp;
  url += "&field2=";
  url += humidity;
  url += "&field3=";
  url += pressure;
  url += "&field4=";
  url += count;
  Serial.print("Requesting URL: ");
  Serial.println(url);
  
  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + 
               "Connection: close\r\n\r\n");
  delay(10);
  
  // Read all the lines of the reply from server and print them to Serial
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
   
    }
   Serial.println();
  Serial.println("closing connection. going to sleep...");
	delay(1000);
 // go to deepsleep for 10 minutes
  system_deep_sleep_set_option(0);
  system_deep_sleep(10 * 60 * 1000000);

}
