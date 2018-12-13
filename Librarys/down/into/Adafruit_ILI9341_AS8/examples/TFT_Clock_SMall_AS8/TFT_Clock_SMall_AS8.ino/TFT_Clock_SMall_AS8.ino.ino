/*
  An example analogue clock using a TFT LCD screen to show the time
 use of some of the drawing commands with the modified Adafruit_ILI9341_AS library.
 For a more accurate clock, it would be better to use the RTClib library.
 But this is just a demo. 
 
 This examples uses the hardware SPI only. Non-hardware SPI
 is just too slow (~8 times slower!)
 
  #### Needs Fonts 4 only ####
 
 Gilchrist 6/2/2014 1.0
 */

#define DEBUG
#include <Adafruit_GFX_AS8.h>    // Core graphics library
#include <Adafruit_ILI9341_AS8.h> // Hardware-specific library

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

Adafruit_ILI9341_AS8 tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Adafruit_ILI9341_AS8 tft;

#define ILI9341_GREY 0x5AEB

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
float sdeg=0, mdeg=0, hdeg=0;
uint16_t osx=120, osy=120, omx=120, omy=120, ohx=120, ohy=120;  // Saved H, M, S x & y coords
uint16_t x0=0, x1=0, y0=0, y1=0;
uint32_t targetTime = 0;                    // for next 1 second timeout
uint8_t hh=conv2d(__TIME__), mm=conv2d(__TIME__+3), ss=conv2d(__TIME__+6);  // Get H, M, S from compile time
boolean initial = 1;

void setup(void) {
  tft.reset();
  delay(10);
  tft.begin(0x9341);
  tft.setRotation(3);
  
  //tft.fillScreen(ILI9341_BLACK);
  //tft.fillScreen(ILI9341_RED);
  //tft.fillScreen(ILI9341_GREEN);
  //tft.fillScreen(ILI9341_BLUE);
  //tft.fillScreen(ILI9341_BLACK);
  tft.fillScreen(ILI9341_GREY);
  
  tft.setTextColor(ILI9341_WHITE, ILI9341_GREY);  // Adding a background colour erases previous text automatically
  //display is 240*320 pixels
  // Draw clock face
  tft.fillCircle(120, 120, 65, ILI9341_GREEN);// x coords y coords /diameter
  tft.fillCircle(120, 120, 60, ILI9341_BLACK);

  // Draw 12 lines
  for(int i = 0; i<360; i+= 30) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*57+120;//changes diameter
    y0 = sy*57+120;
    x1 = sx*50+120;
    y1 = sy*50+120;

    tft.drawLine(x0, y0, x1, y1, ILI9341_GREEN);
  }

  // Draw 60 dots
  for(int i = 0; i<360; i+= 6) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*58+120;//changes diameter
    y0 = sy*58+120;
    // Draw minute markers
    tft.drawPixel(x0, y0, ILI9341_WHITE);
    
    // Draw main quadrant dots
    if(i==0 || i==180) tft.fillCircle(x0, y0, 2, ILI9341_WHITE);
    if(i==90 || i==270) tft.fillCircle(x0, y0, 2, ILI9341_WHITE);
  }

  tft.fillCircle(120, 121, 3, ILI9341_WHITE);

  // Draw text at position 120,260 using fonts 4
  // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . a p m
  // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
  tft.drawCentreString("Time flies",120,200,2);

  targetTime = millis() + 1000; 
}

void loop() {
  if (targetTime < millis()) {
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
      ohx = hx*31+121;  //length of hour hand  
      ohy = hy*31+121;//  31=length  121=startcentre
      tft.drawLine(omx, omy, 120, 121, ILI9341_BLACK);
      omx = mx*42+120;  //length of minute hand  
      omy = my*42+121;
    }

      // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
      tft.drawLine(osx, osy, 120, 121, ILI9341_BLACK);
      osx = sx*45+121;   //length of second hand 
      osy = sy*45+121;
      tft.drawLine(osx, osy, 120, 121, ILI9341_RED);
      tft.drawLine(ohx, ohy, 120, 121, ILI9341_WHITE);
      tft.drawLine(omx, omy, 120, 121, ILI9341_WHITE);
      tft.drawLine(osx, osy, 120, 121, ILI9341_RED);

    tft.fillCircle(120, 121, 3, ILI9341_RED);
  }
}

static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

