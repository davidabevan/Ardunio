/*
 An example showing rainbow colours on a 2.2" TFT LCD screen
 and to show a basic example of font use.
 
 The existing Adafruit font is still in the library
 Only new font sizes 2,4,6 and 7 are implemented in the Adafruit_GFX_AS library.
 
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

unsigned long targetTime = 0;
byte red = 31;
byte green = 0;
byte blue = 0;
byte state = 0;
unsigned int colour = red << 11;

void setup(void) {
  tft.reset();
  delay(10);
  tft.begin(0x9341);
  tft.setRotation(2);
  tft.fillScreen(ILI9341_BLACK);

  targetTime = millis() + 1000; 
}

void loop() {

  if (targetTime < millis()) {
    targetTime = millis()+10000;
    for (int i = 0; i<240; i++) {
      tft.drawFastVLine(i, 0, tft.height(), colour);
      switch (state) {
      case 0: 
        green +=2;
        if (green == 64) {
          green=63; 
          state = 1;
        }
        break;
      case 1: 
        red--;
        if (red == 255) {
          red = 0;
          state = 2; 
        }
        break;
      case 2: 
        blue ++;
        if (blue == 32) {
          blue=31; 
          state = 3; 
        }
        break;
      case 3: 
        green -=2;
        if (green ==255) {
          green=0; 
          state = 4; 
        }
        break;
      case 4: 
        red ++;
        if (red == 32) {
          red = 31; 
          state = 5; 
        }
        break;
      case 5: 
        blue --;
        if (blue == 255) {
          blue = 0; 
          state = 0; 
        }
        break;
      }
      colour = red<<11 | green<<5 | blue;
    }
    
    // The standard ADAFruit font still works as berfore
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK); // Note these fonts do not plot the background colour
    tft.setCursor (68, 5);
    tft.print("Original ADAfruit font!");
    
    // The new larger fonts do not use the .setCursor call, coords are embedded
    tft.setTextColor(ILI9341_BLACK); // Do not plot the background colour
    // Overlay the black text on top of the rainbow plot (the advantage of not drawing the backgorund colour!)
    tft.drawCentreString("Font size 2",120,14,2); // Draw text centre at position 120, 14 using font 2
    tft.drawCentreString("Font size 4",120,30,4); // Draw text centre at position 120, 30 using font 4
    tft.drawCentreString("12.34",120,54,6); // Draw text centre at position 120, 54 using font 6
    tft.drawCentreString("12.34 is in font size 6",120,92,2); // Draw text centre at position 120, 92 using font 2
    // Note the x position is the top of the font!
    
    // draw a floating point number
    float pi = 3.14159; // Value to print
    int precision = 3;  // Number of digits after decimal point
    int xpos = 90;      // x position
    int ypos = 110;     // y position
    int font = 2;       // font number only 2,4,6,7 valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 0 : . a p m
    xpos+=tft.drawFloat(pi,precision,xpos,ypos,font); // Draw rounded number and return new xpos delta for next print position
    tft.drawString(" is pi",xpos,ypos,font); // Continue printing from new x position
  }
}






