/*
  An example analogue clock using a TFT LCD screen to show the time
 use of some of the drawing commands with the modified Adafruit_ILI9341_AS library.
 Based on:
 http://www.instructables.com/id/Arduino-TFT-display-and-font-library/?ALLSTEPS
 
 This examples uses the hardware SPI only. Non-hardware SPI
 is just too slow (~8 times slower!)
 
 Gilchrist 6/2/2014 1.0
 Updated by Alan Senior 18/1/2015
 Updated by Roger Schaefer to use NTP June 2015
 */
///////////////////////////////////////////////////////////////////////////////////////////
//       This version uses the Timezone library to set timezone and daylight savings time
//////////////////////////////////////////////////////////////////////////////////////////
//
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
// For the Adafruit shield, these are the default.
#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#define TFT_DC 2
#define TFT_CS 5

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

#include <SPI.h>
#include <Time.h>              // http://www.arduino.cc/playground/Code/Time
#include <Timezone.h>          // https://github.com/JChristensen/Timezone
//MOD
#include <TimeLib.h>



time_t utc, local;

#define ILI9341_GREY 0x5AEB
#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

//Adafruit_ILI9341 tft = Adafruit_ILI9341(cs, dc, rst);       // Invoke custom library

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
float sdeg=0, mdeg=0, hdeg=0;
int osx=120, osy=120, omx=120, omy=120, ohx=120, ohy=120;  // Saved H, M, S x & y coords

uint32_t targetTime = 0;                    // for next 1 second timeout
//uint8_t hh=conv2d(__TIME__), mm=conv2d(__TIME__+3), ss=conv2d(__TIME__+6);  // Get H, M, S from compile time
uint8_t hh, mm, ss = 0;
boolean initial = 1;

void setup() {
 int x0=0, x1=0;  // y0=0, y1=0;
int y0=0, y1=0;
 Serial.begin(9600);
  if(timeStatus()!= timeSet) 
 
     Serial.println("Waiting to sync with NTP");
  else
     Serial.println("NTP has set the system time");
      //mod
  setSyncProvider( requestSync);  //set function to call when sync required
   Serial.println("Waiting for sync message");
  tft.begin();
  tft.setRotation(3);
  
  //tft.fillScreen(ILI9341_BLACK);
  //tft.fillScreen(ILI9341_RED);
  //tft.fillScreen(ILI9341_GREEN);
  //tft.fillScreen(ILI9341_BLUE);
  //tft.fillScreen(ILI9341_BLACK);
  tft.fillScreen(ILI9341_GREY);  
  tft.setTextColor(ILI9341_YELLOW, ILI9341_GREY);  // Adding a background colour erases previous text automatically
  tft.setCursor(10, 60);
  tft.setTextSize(2);
 // tft.print("Waiting to sync with NTP");
  
  delay (2000);
  tft.fillScreen(ILI9341_GREY);
  DrawClockFace();
  // Draw text at position 120,260 using fonts 4
  // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . a p m
  // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
 // tft.drawCentreString("Date",255,10,2);
  //tft.drawCentreString("day",255,205,2);
  targetTime = millis() + 1000;
  
}
//-----------------------------------------------------------End Setup
void loop() {
 // time_t t;
  //if (targetTime < millis()) {
    AdvanceClock(now());
    //if (timeStatus() == timeSet) {
    DisplayDate(now());
    
  
//  processSyncMessage();  
  //mod
  if (Serial.available()) {
    processSyncMessage();
  }
  if (timeStatus()!= timeNotSet) {
    digitalClockDisplay();  
  }
  if (timeStatus() == timeSet) {
    digitalWrite(13, HIGH); // LED on if synced
  } else {
    digitalWrite(13, LOW);  // LED off if needs refresh
  }
  delay(1000); 
  }
  void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}
void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

if(Serial.find(TIME_HEADER)) {
//  if(Serial.find(T)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
       //setTime(myTZ.toLocal(pctime));
     }
  }
}
time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}
//--------------------------------------------------------
void DrawClockFace(){
    
  // Draw clock face
  tft.fillCircle(120, 120, 118, ILI9341_GREEN);
  tft.fillCircle(120, 120, 110, ILI9341_BLACK);

  // Draw 12 lines
  for(int i = 0; i<360; i+= 30) {
   int sx = cos((i-90)*0.0174532925);
   int sy = sin((i-90)*0.0174532925);
   int x0 = sx*114+120;
   int y0 = sy*114+120;
   int x1 = sx*100+120;
   int y1 = sy*100+120;

    tft.drawLine(x0, y0, x1, y1, ILI9341_GREEN);
  }

  // Draw 60 dots
  for(int i = 0; i<360; i+= 6) {
   int sx = cos((i-90)*0.0174532925);
   int sy = sin((i-90)*0.0174532925);
   int x0 = sx*102+120;
   int y0 = sy*102+120;
    // Draw minute markers
    tft.drawPixel(x0, y0, ILI9341_WHITE);
    
    // Draw main quadrant dots
    if(i==0 || i==180) tft.fillCircle(x0, y0, 2, ILI9341_WHITE);
    if(i==90 || i==270) tft.fillCircle(x0, y0, 2, ILI9341_WHITE);
  }

  tft.fillCircle(120, 121, 3, ILI9341_WHITE);
}
//-----------------------------------------------------------
void AdvanceClock(time_t t){
  hh = hour(t);
  mm = minute(t);
  ss = second(t);
    targetTime = millis()+1000;
    ss++;              // Advance second
    if (ss==60) {
      ss=0;
      mm++;            // Advance minute
      if(mm>59) {
        mm=0;
        hh++;          // Advance hour
        if (hh>23) {
          hh=0;
        }
      }
    }

    // Pre-compute hand degrees, x & y coords for a fast screen update
    sdeg = ss*6;                  // 0-59 -> 0-354
    mdeg = mm*6+sdeg*0.01666667;  // 0-59 -> 0-360 - includes seconds
    hdeg = hh*30+mdeg*0.0833333;  // 0-11 -> 0-360 - includes minutes and seconds
    hx = cos((hdeg-90)*0.0174532925);    
    hy = sin((hdeg-90)*0.0174532925);
    mx = cos((mdeg-90)*0.0174532925);    
    my = sin((mdeg-90)*0.0174532925);
    sx = cos((sdeg-90)*0.0174532925);    
    sy = sin((sdeg-90)*0.0174532925);

    if (ss==0 || initial) {
      initial = 0;
      // Erase hour and minute hand positions every minute
      tft.drawLine(ohx, ohy, 120, 121, ILI9341_BLACK);
      ohx = hx*62+121;    
      ohy = hy*62+121;
      tft.drawLine(omx, omy, 120, 121, ILI9341_BLACK);
      omx = mx*84+120;    
      omy = my*84+121;
    }

      // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
      tft.drawLine(osx, osy, 120, 121, ILI9341_BLACK);
      osx = sx*90+121;    
      osy = sy*90+121;
      tft.drawLine(osx, osy, 120, 121, ILI9341_RED);
      tft.drawLine(ohx, ohy, 120, 121, ILI9341_WHITE);
      tft.drawLine(omx, omy, 120, 121, ILI9341_WHITE);
      tft.drawLine(osx, osy, 120, 121, ILI9341_RED);

    tft.fillCircle(120, 121, 3, ILI9341_RED);
}
//---------------------------------------
void DisplayDate(time_t t){
  String Date;
  char DateStr[40];
  Date = (monthShortStr(month(t)));
  Date = Date + ' ';
  Date = Date + (day(t));
  //Date = Date + ", ";
  //Date = Date + (year(t));
  Date.toCharArray(DateStr,35);
//  tft.drawCentreString(DateStr,255,10,2);
  Date = (dayShortStr(weekday()));
  Date = " " + Date + " ";
  Date.toCharArray(DateStr,35);
  //tft.drawCentreString("         ",255,205,2);
//  tft.drawCentreString(DateStr,255,205,2);
}
//-----------------------------------------------------------End Loop
static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}
//--------------------------------------------------------
