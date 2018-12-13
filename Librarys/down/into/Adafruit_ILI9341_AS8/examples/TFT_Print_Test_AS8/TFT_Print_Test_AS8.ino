/*
Demonstrate (and test) the use of the standard "print" class with proportional fonts

Enable fonts 2, 4 and 7 in "Load_fonts.h" that can be found within the Adafruit_GFX_AS8 library folder

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

void setup(void) {
  tft.reset();
  delay(10);
  tft.begin(0x9341);
  tft.setRotation(2);
}

void loop() {
  
  // Fill screen with grey so we can see the effect of printing with a background colour defined
  tft.fillScreen(ILI9341_GREY);
  
  // Set "cursor" at top left corner of display (0,0) and select font 2
  // (cursor will move to next line automatically during printing with 'println')
  tft.setCursor(0, 0, 2);
  // Set the font colour to be white with a black background, set text size multiplier to 1
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);  tft.setTextSize(1);
  // We can now plot text on screen using the "print" class
  tft.println("Hello World!");
  
  // Set the font colour to be yellow with no background, set to font 7
  tft.setTextColor(ILI9341_YELLOW); tft.setTextFont(7);
  tft.println(1234.56);
  
  // Set the font colour to be red with black background, set to font 7
  tft.setTextColor(ILI9341_RED,ILI9341_BLACK);    tft.setTextFont(4);
  tft.println(0xDEADBEEF, HEX);

  // Set the font colour to be green with black background, set to font 4
  tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.setTextFont(4);
  tft.println("Groop");
  tft.println("I implore thee,");

  // Change to font 2
  tft.setTextFont(2);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  // This next line is deliberately made too long for the display width to test text wrapping onto next line
  tft.println("Or I will rend thee in the gobberwarts with my blurglecruncheon, see if I don't!");
  
  // Test some print formatting functions
  float fnumber = 123.45;
   // Set the font colour to be blue with no background, set to font 4
  tft.setTextColor(ILI9341_BLUE);    tft.setTextFont(4);
  tft.print("Float = "); tft.println(fnumber);           // Print floating point number
  tft.print("Binary = "); tft.println((int)fnumber, BIN); // Print as integer value in binary
  tft.print("Hexadecimal = "); tft.println((int)fnumber, HEX); // Print as integer number in Hexadecimal
  delay(10000);
}



