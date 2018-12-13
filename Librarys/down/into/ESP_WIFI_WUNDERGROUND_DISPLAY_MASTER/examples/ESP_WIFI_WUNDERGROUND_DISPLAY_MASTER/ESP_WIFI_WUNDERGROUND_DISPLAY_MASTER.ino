

//Wiring Diagram
//****************************************************
//                                   LCD_PIN
//TFT LCD Pin VCC   +3.3v         *1
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
#include "TimeClient.h"
#include "ThingspeakClient.h"
#include <Adafruit_ILI9341.h>
//******Define Screen Pins**********
#define TFT_CS     5
#define TFT_DC     2
#define SERIAL_OUT Serial
#define ILI9341_GREY 0x5AEB
//********Setup Bitmap Array*********
//extern unsigned int Arduino[];//Splash screen.****Had to Disable these due to low memory
//extern unsigned int  Wifi[];//WiFi connected logo
//********************************Weather Icons***************************
//********Vcloud large icons 175*120
extern unsigned int clear[];
extern unsigned int chancerain[];
extern unsigned int overcast[];
extern unsigned int Rain[];
extern unsigned int scatteredclouds[];
extern unsigned int mostlycloudy[];
extern unsigned int fog[];
extern unsigned int PartlyCloudy[];
extern unsigned int cloudy[];
extern unsigned int Hazy[];// Load into programme memory "Low storage Space"
 
//****Counter for date functiont to run once.
int Counter = 0; // Set up a counter to run date function once.
// also now used for thingspeak and wundergroun
//const int UPDATE_INTERVAL_SECS = 10 * 60; // Update every 10 minutes
//test to see if makes clock tick NO
/***************************
   Begin Settings
 **************************/
// Please read http://blog.squix.org/weatherstation-getting-code-adapting-it
// for setup instructions

// WIFI
//const char* WIFI_SSID = "PLUSNET-9P9QR6";//Turned off trying wifi persistance
//const char* WIFI_PWD = "3bb7bc7cd3";
//const char* WIFI_SSID = "TNCAPC2653F";
//const char* WIFI_PWD = "A7252C69DF";


// Display Settings Oled
//const int I2C_DISPLAY_ADDRESS = 0x3c;
//const int SDA_PIN = 0;
//const int SDC_PIN = 2;

// Use hardware SPI (on ESP8266, #13, #12, #11)
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);


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

/***************************
   End Settings
 **************************/
TimeClient timeClient(UTC_OFFSET);

// Set to false, if you prefere imperial/inches, Fahrenheit
WundergroundClient wunderground(IS_METRIC);

ThingspeakClient thingspeak;

// flag changed in the ticker function every 10 minutes
bool readyForWeatherUpdate = false;

String lastUpdate = "--";
//Ticker ticker;//Test to see if this makes clock tick NO

void setup() {
  Serial = SERIAL_OUT;
  Serial.begin(9600);
  Serial.println("ILI9341 Test!");
  tft.begin  ();
  tft.setRotation(3);
  // tft.fillScreen(ILI9341_BLACK);
  tft.fillScreen(ILI9341_WHITE);
  //*******Screesize is 320*240 rotation 3 x, y = Height, Length....
  //Centres             160 120
  //tft.drawBitmap (24, 27, 272, 185, Arduino);//Startup Splash screen. Caused me a Bug could not connect removed memory footprint fixed it.See line 40.
  //

  delay(2000);
  //*****temp wifi********************************************************************************
  WiFi.begin();//(WIFI_SSID, WIFI_PWD);//WiFi persistence

  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    tft.print(".");
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
    tft.println("Connecting to WiFi");
    //    tft.println(WIFI_SSID);//wifi persistence
    // counter++;
  }
  tft.println("");
  tft.print("Local IP: ");
  tft.println(WiFi.localIP());
  //End Tempwifi
// *************************IMPORTANT DISCONNECT POWER AND REBOOT IF WIFI WONT CONNECT***********************************************************
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  delay(3000);
  //tft.fillScreen(ILI9341_WHITE);
  //tft.drawBitmap (40, 17, 244, 206, Wifi);//*******************Disabled due to low memory
  delay (5000);
}     //End setup******** dont lose cost me hours of debugging when i ballsed up

void Get_Date () {  //**********Gets Date from Wunderground..Function mod by DB.....24/08/2016********************
  // ********************Run function once as Wunderground capps usage limits.......
  tft.fillScreen(ILI9341_BLACK);
  wunderground.updateForecast(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);//does this give day index
  wunderground.updateConditions(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);// date and current conditions
  tft.setCursor(20, 10);
  tft.print("Date");
  String date = wunderground.getDate();//**********USE________ Gets date and names it
  tft.setCursor(20, 50);
  tft.print( date);// Displays date DD MMM YYYY
}
//***************************************************************************************************************
void Get_UpDate () {  //**********Gets weather data from Wunderground..Function mod by DB.....24/08/2016********************
  // ********************Run function once as Wunderground capps usage limits.......

  wunderground.updateForecast(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);//does this give day index
  wunderground.updateConditions(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);// date and current conditions
}

void Update_Thingspeak () {//****** Now updating Thingspeak using a counter 29/08/2016 db******************
 thingspeak.getLastChannelItem(THINGSPEAK_CHANNEL_ID, THINGSPEAK_API_READ_KEY);
  tft.setCursor(20, 10);
  tft.println  ("Updating Thingspeak");
  delay (3000);
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(30, 10);
  tft.print("Thingspeak");
  tft.setCursor(30, 50);
  tft.print  ("Temperature  ");
  //tft.setCursor(30,50);
  tft.print (thingspeak.getFieldValue(0) + "C");
  tft.setCursor(30, 70);
  tft.print("Humidity  ");
  tft.print (thingspeak.getFieldValue(1) + "%");
  delay (3000);
//   ticker.attach(UPDATE_INTERVAL_SECS, setReadyForWeatherUpdate);//Test to see if makes clock tick
}
void loop() {
  if (Counter < 1) {          // Run function once as Wunderground capps usage limits.......
    Get_Date ();
  }
  Counter = Counter + 1;
  Serial.print("Counter: ");//Debug
  Serial.println(Counter);
  if (Counter == 60) { //This updates wunderground on device
    //return;// newline after counter*******Not working yet
    Get_UpDate();
    Counter = 1; //Reset Counter not 0 as this triggers date function
  }

  delay (3000);
  tft.fillScreen(ILI9341_BLACK);
if (Counter == 1) {     //Thingspeak update timer. See line 189 now a function
  Update_Thingspeak ();
}
  else 
  if (Counter==30){
     Update_Thingspeak ();
}
  //******thingspeak***********
  // *******To do set timer for Thingspeak
 // thingspeak.getLastChannelItem(THINGSPEAK_CHANNEL_ID, THINGSPEAK_API_READ_KEY);
 // tft.setCursor(20, 10);
 // tft.println  ("Updating Thingspeak");
 // delay (3000);
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(30, 10);
  tft.print("Thingspeak");
  tft.setCursor(30, 50);
  tft.print  ("Temperature  ");
  //tft.setCursor(30,50);
  tft.print (thingspeak.getFieldValue(0) + "C");
  tft.setCursor(30, 70);
  tft.print("Humidity  ");
  tft.print (thingspeak.getFieldValue(1) + "%");
  delay (3000);
  //--------Weather underground----------DATE____________Now defined as function db---- Get_Date SEE LINE 135

  //****************Display Time*************************
  // tft.fillScreen(ILI9341_BLACK);
  timeClient.updateTime();// time not displayed if disabled
  String time = timeClient.getFormattedTime();// gets time and gives it a name ******USE
  // textWidth = display->getStringWidth(time);
  tft.setCursor(30, 100);
  tft.print("Time ");
  tft.print( time);// To do. Get time to count when displayed
  delay(5000);
  //*****************WUnderGround Conditions (Today)****************************
  tft.fillScreen(ILI9341_BLACK);
  // wunderground.updateConditions(WUNDERGRROUND_API_KEY, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);//Get Date function use this at Setup
  //void updateAstronomy(String apiKey, String language, String country, String city);
  // no need to run again especially with Capp.
  //Name wunderground json objects.
  // string getMoonPctIlum();
  //String getMoonAge();
  // String getMoonPhase();
  // String getSunriseTime();
  // String getSunsetTime();
  // String getMoonriseTime();
  // String getMoonsetTime();
  // String getWindSpeed();
  //String getWindDir();//Not recognised!!!!! Is this the updated Wunderground library i wonder......
  // String getCurrentTemp();
  //  String getTodayIcon();
  //  String getMeteoconIcon(String iconText);
  //  String getWeatherText();
  //  String getForecastIcon(int period);
  //  String getForecastTitle(int period);
  //  String getForecastLowTemp(int period);
  //String getForecastHighTemp(int period);
  int dayIndex;//set up string for dayindex for forecast high/low temps and forecast icons
  String temp = wunderground.getCurrentTemp() + " C";
  String pressure = wunderground.getPressure();
  String rain = wunderground.getPrecipitationToday();
  String day = wunderground.getForecastTitle(dayIndex).substring(0, 3);
  String LowTemp = wunderground.getForecastLowTemp(dayIndex);
  String HighTemp = wunderground.getForecastHighTemp(dayIndex);//dayIndex now set
  String Humidity = wunderground.getHumidity();
  String Windspeed = wunderground.getWindSpeed();// working now
  tft.setCursor(20, 10);
  tft.print("WunderGround");
  tft.setCursor(20, 30);
  tft.print("Current Conditions Now");
  tft.setCursor(20, 50);
  tft.print("Pressure ");
  tft.print(pressure) + " mBar";
  tft.setCursor(20, 70);
  tft.print("Precipitation ");
  tft.print(rain) + " mm";
  tft.setCursor(20, 90);
  tft.print("Temperature ");
  tft.print(temp);// Cant get "C" to work put in json get string
  tft.setCursor(20, 110);
  tft.print("     Temperatures");
  tft.setCursor(20, 130);
  tft.print("HighTemp     LowTemp");
  tft.setCursor(20, 150);
  tft.print("   ------       ");
  tft.setCursor(20, 150);
  tft.print(HighTemp);
  tft.setCursor(175, 150);
  tft.print(LowTemp);
  tft.setCursor(20, 170);
  tft.print("Relative Humidity ");
  tft.print(Humidity);
  tft.setCursor(20, 190);
  tft.print("WindSpeed ");
  tft.print(Windspeed);//  workingnow
  delay (7000);
  //To Do wind strength and direction
  //To Do Time at the bottome again?????
  //Temps colour
  //

  //*********************************************
  // //--------Weather underground----------Forecast____________
  //  wunderground.updateForecast(WUNDERGRROUND_API_KEY, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);
  //String getForecastIcon(int period);
  //String getForecastTitle(int period);
  String weatherIcon = wunderground.getTodayIcon();
  String weathertext = wunderground.getWeatherText();
// tft.setTextColor(ILI9341_GREY);
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(20, 10);
  tft.print("Todays Forecast");
  tft.setCursor(20, 30);
  //tft.print(weatherIcon);// Prints B at the moment.
  tft.print(weathertext);
  Serial.print (weathertext);//debug
  //tft.drawBitmap (30, 50, 42, 42, clear);
  //*******Today Weather Icons display function************
  //*******getting correct icons for weather printed
   //*******Screesize is 320*240 rotation 3 x, y = Height, Length....
  //Centres             160 120
  if (weathertext == "Clear") {
    tft.drawBitmap (30, 75, 175, 120, clear);
  }
  else if (weathertext == "Chancerain") {
    tft.drawBitmap (30, 75, 175, 120, chancerain);
  }
  else if (weathertext == "Overcast") {
    tft.drawBitmap (30, 75, 175, 120, overcast);
  }
  else if (weathertext == "Cloudy") {
    tft.drawBitmap (30, 75, 175, 120, cloudy);
  }
  else if (weathertext == "Rain") {
    tft.drawBitmap (30, 75, 175, 120, Rain);
  }

  else if (weathertext == "Mostly Cloudy") {
    tft.drawBitmap (30, 75, 175, 120, mostlycloudy);//vcloud icon
  }
   else if (weathertext == "Partly Cloudy") {
    tft.drawBitmap (30, 75, 175, 120, PartlyCloudy);//Vcloud icon
  }
   else if (weathertext == "Scattered Clouds") {
    tft.drawBitmap (30, 75, 175, 120, scatteredclouds);
   }
   else if (weathertext == "Fog") {
    tft.drawBitmap (30, 75, 175, 120, fog);
   }
   else if (weathertext == "LightRain") {
    tft.drawBitmap (30, 75, 175, 120, chancerain);
   }
   else if (weathertext == "Scattered Clouds") {
    tft.drawBitmap (30, 75, 175, 120, scatteredclouds);
   }
   else if (weathertext == "LightDrizzle") {
    tft.drawBitmap (30, 75, 175, 120, fog);
   }
   else if (weathertext == "Haze")  {
    tft.drawBitmap (30, 75, 175, 120, fog);
   }
   else if (weathertext == "Mist"){
    tft.drawBitmap (30, 75, 175, 120, fog);
    
   }
  delay(5000);
}
// To Do 3 day forecast icons
// wind direction and strength on right hand side
// texttual forecasts
//news feed
//************************************************************************


