/*
 An example digital clock using a TFT LCD screen to show the time.
 Demonstrates use of the font printing routines. (Time updates but date does not.)
 
 For a more accurate clock, it would be better to use the RTClib library.
 But this is just a demo. 

 Based on clock sketch by Gilchrist 6/2/2014 1.0
 
 #### Needs Fonts 2, 4, 6, 7 only ####
 
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

uint32_t targetTime = 0;                    // for next 1 second timeout
uint8_t hh=conv2d(__TIME__), mm=conv2d(__TIME__+3), ss=conv2d(__TIME__+6);  // Get H, M, S from compile time

byte omm = 99;
boolean initial = 1;
byte xcolon = 0;
unsigned int colour = 0;

void setup(void) {
  tft.reset();
  delay(10);
  tft.begin(0x9341);
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);

  targetTime = millis() + 1000; 
}

void loop() {
  if (targetTime < millis()) {
    targetTime = millis()+1000;
    ss++;              // Advance second
    if (ss==60) {
      ss=0;
      omm = mm;
      mm++;            // Advance minute
      if(mm>59) {
        mm=0;
        hh++;          // Advance hour
        if (hh>23) {
          hh=0;
        }
      }
    }

    if (ss==0 || initial) {
      initial = 0;

      tft.setTextColor(ILI9341_BLUE, ILI9341_BLACK);
      tft.drawCentreString(__DATE__,120,54,2); // Next size up font 2

      tft.setTextColor(0xF81F, ILI9341_BLACK); // Pink
      tft.drawCentreString("12.34",120,100,6); // Large font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 . : a p m
    }

    // Update digital time
    byte xpos = 50;
    byte ypos = 0;
    if (omm != mm) { // Only redraw every minute to minimise flicker
      // Uncomment ONE of the next 2 lines, using the ghost image demonstrates text overlay as time is drawn over it
      tft.setTextColor(0x39C4, ILI9341_BLACK);  // Leave a 7 segment ghost image, comment out next line!
      //tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK); // Set font colour to back to wipe image
      // Font 7 is to show a pseudo 7 segment display.
      // Font 7 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 0 : .
      tft.drawString("88:88",xpos,ypos,7); // Overwrite the text to clear it
      tft.setTextColor(0xFBE0); // Orange
      omm = mm;

      if (hh<10) xpos+= tft.drawChar('0',xpos,ypos,7);
      xpos+= tft.drawNumber(hh,xpos,ypos,7);
      xcolon=xpos;
      xpos+= tft.drawChar(':',xpos,ypos,7);
      if (mm<10) xpos+= tft.drawChar('0',xpos,ypos,7);
      tft.drawNumber(mm,xpos,ypos,7);
    }

    if (ss%2) { // Flash the colon
      tft.setTextColor(0x39C4, ILI9341_BLACK);
      xpos+= tft.drawChar(':',xcolon,ypos,7);
      tft.setTextColor(0xFBE0, ILI9341_BLACK);
      //tft.invertDisplay(0); // Test display inversion
    }
    else {
      //tft.invertDisplay(1); // Test display inversion
      tft.drawChar(':',xcolon,ypos,7);
      colour = random(0xFFFF);
      // Erase the text with a rectangle
      tft.fillRect (0, 74, 240, 20, ILI9341_BLACK);
      tft.setTextColor(colour);
      tft.drawRightString("Colour:",120,74,4); // Right justified string drawing to x position 120
      String scolour = String(colour,HEX);
      scolour.toUpperCase();
      char buffer[20];
      scolour.toCharArray(buffer,20);
      tft.drawString(buffer,128,74,4);
    }
  }
}

static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}



