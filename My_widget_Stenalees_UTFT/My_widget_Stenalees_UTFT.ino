

//Wiring Diagram
//****************************************************
//                                   LCD_PIN
//TFT LCD Pin VCC                 *1
//TFT LCD Pin GND                 *2
//TFT LCD Pin CS  to GPIO_5       *3
//TFT LCD Pin RST to RST          *4
//TFT LCD Pin DC to GPIO_2        *5
//TFT LCD Pin MOSI to GPIO_13     *6
//TFT LCD Pin CLK to GPIO_14      *7
//TFT LCD Pin LED to +3.3 V.      *8
//TFT LCD Pin MISO ( not use )    *9
//****************************************************
#include <functional>
#include <algorithm>
#include <TimeLib.h> 
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
//#define ST7735
#define ILI9340
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <JsonListener.h>
#include "SSD1306Wire.h"
#include "OLEDDisplayUi.h"
#include "Wire.h"
#include "WundergroundClient.h"
//#include "WeatherStationFonts.h"
//#include "WeatherStationImages.h"
#include "TimeClient.h"
#include "ThingspeakClient.h"
#include <Adafruit_ILI9341.h>
extern unsigned int Wifi[];
//For printing Thingspeak values on utft using printNumF
//float tempC;
float hum= 0;
//****UTFT******
#include <UTFT.h>
extern uint8_t SmallFont[];
// Modify the line below to match your display and wiring:
UTFT tft ( ILI9341_S5P, 13, 14, 5, 4, 2 );//Serial SW
// UTFT myGLCD= tft(ILI9341_S5P, 13, 14, 5, 2);
#if defined(ST7735)
#  include <Adafruit_ST7735.h>
#  define ColorPrimary ST7735_BLACK
#  define ColorBG ST7735_WHITE
#  define ColorRED ST7735_RED
#elif defined(ILI9340)
#  include "Adafruit_ILI9340.h"
#  define ColorPrimary ILI9340_BLACK
#  define ColorBG ILI9340_WHITE
#  define ColorRED ILI9340_RED
#endif
#define TFT_CS     5
#define TFT_DC     2
#define SERIAL_OUT Serial
//#define thingspeak.getFieldValue(1) (h);
//#define thingspeak.getFieldValue(0) t;
//#if defined(ST7735)
  //Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC);
  //Point_t displayCenter = { // 160x128 with rotation=3
   // ST7735_TFTHEIGHT_18 / 2,
   // ST7735_TFTWIDTH / 2
  //};
//#elif defined(ILI9340)
 // Adafruit_ILI9340 tft = Adafruit_ILI9340(TFT_CS, TFT_DC, -1);
 // Point_t displayCenter = { // 160x128 with rotation=3
   // ILI9340_TFTHEIGHT / 2,
   // ILI9340_TFTWIDTH / 2
  //};
//#endif

/***************************
 * Begin Settings
 **************************/
// Please read http://blog.squix.org/weatherstation-getting-code-adapting-it
// for setup instructions

// WIFI
const char* WIFI_SSID = "PLUSNET-9P9QR6";
const char* WIFI_PWD = "3bb7bc7cd3";


// Setup
const int UPDATE_INTERVAL_SECS = 10 * 60; // Update every 10 minutes

// Display Settings
//const int I2C_DISPLAY_ADDRESS = 0x3c;
//const int SDA_PIN = 0;
//const int SDC_PIN = 2;

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// trying to get thingspeak to 
const float t=0;

// TimeClient settings
const float UTC_OFFSET = 1;

// Wunderground Settings
const boolean IS_METRIC = true;
const String WUNDERGRROUND_API_KEY = "0e1262cc5f6a4bf9";
const String WUNDERGRROUND_LANGUAGE = "ENG";
const String WUNDERGROUND_COUNTRY = "UK";
const String WUNDERGROUND_CITY = "St_Austell";

//Forcast io settings
// Go to forecast.io and register for an API KEY
String forecastApiKey = "c697820a5ac9cdda78df4fafc3f802f5";

//Thingspeak Settings St Blazey
//const String THINGSPEAK_CHANNEL_ID = "66395";//St Blazey 
//const String THINGSPEAK_API_READ_KEY = "LIVECL6USLFI5CK4";//St Blazey
//Thingspeak settings Stenalees
const String THINGSPEAK_CHANNEL_ID = "146904";//Stenalees
const String THINGSPEAK_API_READ_KEY = "68BFR4TRY0FB6W6V";//Stenalees
//OLEDDisplayUi   ui( &tft );
/***************************
 * End Settings
 **************************/
 TimeClient timeClient(UTC_OFFSET);

// Set to false, if you prefere imperial/inches, Fahrenheit
WundergroundClient wunderground(IS_METRIC);

ThingspeakClient thingspeak;

// flag changed in the ticker function every 10 minutes
bool readyForWeatherUpdate = false;

String lastUpdate = "--";

Ticker ticker;

//declaring prototypes
void drawProgress(OLEDDisplay *display, int percentage, String label);
void updateData(OLEDDisplay *display);
void drawDateTime(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void drawCurrentWeather(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void drawForecast(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void drawThingspeak(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
void drawForecastDetails(OLEDDisplay *display, int x, int y, int dayIndex);

void setReadyForWeatherUpdate();
//float tempC = thingspeak.getFieldValue(0);
//float f = dht.readTemperature(true);
void setup() {
  //*****UTFT******
  // myGLCD.InitLCD (  );
  tft.InitLCD (  );
    tft.setFont ( SmallFont );
  SERIAL_OUT.begin(9600);
  SERIAL_OUT.println("ILI9341 Test!");
  //tft.begin  ();
//   tft.setRotation(3);
  // tft.fillScreen(ILI9341_BLACK);
   tft.clrScr (  );
//tft.setColor ( 64, 64, 64 );
  tft.setColor(ILI9341_WHITE);
//   tft.setCursor(0, 0);
 // tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.print ( String("* Universal Color TFT Display Library *"), CENTER, 1 );
  tft.setColor(ILI9341_YELLOW);// tft.setTextSize(2);
  //tft.print ( String(1234.56), CENTER, 10);
  WiFi.begin(WIFI_SSID, WIFI_PWD);
tft.drawBitmap (40, 17, 244, 206, Wifi);
delay(5000);
  int counter = 0;
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
//    tft.print(".");
    tft.fillScr(ILI9341_BLACK);
    tft.setColor(ILI9341_YELLOW);// tft.setTextSize(2);
    tft.print (String("Connecting to WiFi"), CENTER, 10);
SERIAL_OUT.println (WIFI_SSID);
//tft.print (WIFI_SSID , CENTER, 10);
    // tft.printNumI (WIFI_SSID  (  ), CENTER, 225 );
     counter++;
//     tft.drawBitmap(WiFi_Logo_bits);
    }
// tft.print ("");
  
  tft.print(String("Local IP: "), CENTER, 50);
  SERIAL_OUT.println (WiFi.localIP());   
 
// delay (3000);

  }
 void loop() { 
  // delay (30000);
  int tempC;
  thingspeak.getFieldValue(0).toInt();
tempC=toInt;
// thingspeak.getFieldValue(0)= tempC;
//( tempC )= thingspeak.getFieldValue(0);
  tft.fillScr(ILI9341_BLACK);
//updatetime();
//******thingspeak***********
  thingspeak.getLastChannelItem(THINGSPEAK_CHANNEL_ID, THINGSPEAK_API_READ_KEY);
// tft.setCursor(20,10);
// thingspeak.getFieldValue(0) = tempC;
  tft.print(String  ("Updating Thingspeak"),CENTER, 10);
  delay (3000);
  
  tft.fillScr(ILI9341_BLACK);
 // tft.setCursor(30,10);
  tft.print(String("Thingspeak"), CENTER, 10);
  //tft.setCursor(30,50);
 tft.print (String ("Temperature  "), 30, 50);
 //tft.setCursor(30,50);
Serial.print (thingspeak.getFieldValue(0) + "C"), 30, 50;
// tft.setCursor(30,70);
 tft.print(String("Humidity  "), 30, 70);
Serial.print (thingspeak.getFieldValue(1) + "%");
 //tft.printNumF (thingspeak.getFieldValue(1),2 , 10, 10);
 tft.printNumI (tempC, 130, 50); 
// tft.printNumI (String(thingspeak.getFieldValue(0)) 130, 60);
 delay (30000);
//--------Weather underground----------
// wunderground.updateConditions(WUNDERGRROUND_API_KEY, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);
//  wunderground.updateForecast(WUNDERGRROUND_API_KEY, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);
//tft.print (wunderground.getHumidity());
 // display->drawString(96 + x, 10 + y, wunderground.getPressure());
  //display->drawString(32 + x, 38 + y, wunderground.getPrecipitationToday());
   delay (30000);
 }
 //*********************************************
// updatetime();
//void updatetime() {


//  thingspeak.getLastChannelItem(THINGSPEAK_CHANNEL_ID, THINGSPEAK_API_READ_KEY);

 void updateData(OLEDDisplay *display) {
  drawProgress(display, 10, "Updating time...");
  //tft.setCursor(0,10);
//  tft.print  ("Updating time..."), 0, 10;
  timeClient.updateTime();
  drawProgress(display, 30, "Updating conditions...");
  wunderground.updateConditions(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);
  drawProgress(display, 50, "Updating forecasts...");
  wunderground.updateForecast(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);
  drawProgress(display, 80, "Updating thingspeak...");
  thingspeak.getLastChannelItem(THINGSPEAK_CHANNEL_ID, THINGSPEAK_API_READ_KEY);
  lastUpdate = timeClient.getFormattedTime();
  readyForWeatherUpdate = false;
  drawProgress(display, 100, "Done...");
  delay(1000);
 }
 void setReadyForWeatherUpdate() {
  Serial.println("Setting readyForUpdate to true");
  readyForWeatherUpdate = true;
 }
