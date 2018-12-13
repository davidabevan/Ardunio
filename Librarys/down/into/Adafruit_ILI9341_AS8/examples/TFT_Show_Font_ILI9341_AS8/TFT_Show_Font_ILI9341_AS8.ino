/*
 Sow all the fonts.
 
 Only font sizes 2, 4, 6 and 7 are implemented in the Adafruit_GFX_AS8 library.
 
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
  tft.setRotation(1);
}

void loop() {

    tft.setTextSize(1);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_GREEN);

    tft.drawString(" !\"#$%&'()*+,-./0123456",0,0,2);
    tft.drawString("789:;<=>?@ABCDEFGHIJKL",0,16,2);
    tft.drawString("MNOPQRSTUVWXYZ[\\]^_`",0,32,2);
    tft.drawString("abcdefghijklmnopqrstuvw",0,48,2);
    int xpos=0;
    xpos+=tft.drawString("xyz{|}~",0,64,2);
    tft.drawChar(127,xpos,64,2);
    delay(4000);
    
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_GREEN);

    tft.drawString(" !\"#$%&'()*+,-.",0,0,4);
    tft.drawString("/0123456789:;",0,26,4);
    tft.drawString("<=>?@ABCDE",0,52,4);
    tft.drawString("FGHIJKLMNO",0,78,4);
    tft.drawString("PQRSTUVWX",0,104,4);
    
    delay(4000);
    tft.fillScreen(ILI9341_BLACK);
    tft.drawString("YZ[\\]^_`abc",0,0,4);
    tft.drawString("defghijklmno",0,26,4);
    tft.drawString("pqrstuvwxyz",0,52,4);
    xpos=0;
    xpos+=tft.drawString("{|}~",0,78,4);
    tft.drawChar(127,xpos,78,4);
    delay(4000);
    
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_BLUE);

    tft.drawString("012345",0,0,6);
    tft.drawString("6789",0,40,6);
    tft.drawString("apm-:.",0,80,6);
    delay(4000);

    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);

    tft.drawString("0123",0,0,7);
    tft.drawString("4567",0,60,7);
    delay(4000);

    tft.fillScreen(ILI9341_BLACK);
    tft.drawString("890:.",0,0,7);
    tft.drawString("",0,60,7);
    delay(4000);
    
    tft.setTextSize(2);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_GREEN);

    tft.drawString(" !\"#$%&'()*+,-./0123456",0,0,2);
    tft.drawString("789:;<=>?@ABCDEFGHIJKL",0,32,2);
    tft.drawString("MNOPQRSTUVWXYZ[\\]^_`",0,64,2);
    tft.drawString("abcdefghijklmnopqrstuvw",0,96,2);
    xpos=0;
    xpos+=tft.drawString("xyz{|}~",0,128,2);
    tft.drawChar(127,xpos,128,2);
    delay(4000);
    
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_GREEN);

    tft.drawString(" !\"#$%&'()*+,-.",0,0,4);
    tft.drawString("/0123456789:;",0,52,4);
    tft.drawString("<=>?@ABCDE",0,104,4);
    tft.drawString("FGHIJKLMNO",0,156,4);
    tft.drawString("PQRSTUVWX",0,208,4);
    delay(4000);
    tft.fillScreen(ILI9341_BLACK);
    tft.drawString("YZ[\\]^_`abc",0,0,4);
    tft.drawString("defghijklmno",0,52,4);
    tft.drawString("pqrstuvwxyz",0,104,4);
    xpos=0;
    xpos+=tft.drawString("{|}~",0,156,4);
    tft.drawChar(127,xpos,156,4);
    delay(4000);
    
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_BLUE);

    tft.drawString("01234",0,0,6);
    tft.drawString("56789",0,80,6);
    tft.drawString("apm-:.",0,160,6);
    delay(4000);

    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);

    tft.drawString("0123",0,0,7);
    tft.drawString("4567",0,120,7);
    delay(4000);

    tft.fillScreen(ILI9341_BLACK);
    tft.drawString("890:.",0,0,7);
    tft.drawString("",0,120,7);
    delay(4000);
    
    tft.setTextColor(ILI9341_MAGENTA, ILI9341_WHITE);

    tft.drawString(">>That's all<< ",0,180,4);
    delay(4000);
}

void rainbow(int ystart, int ylen)
{
    red = 31;
    green = 0;
    blue = 0;
    state = 0;
    colour = red << 11;
    
    for (int i = 0; i<160; i++) {
      tft.drawFastVLine(i, ystart, ylen, colour);
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
}





