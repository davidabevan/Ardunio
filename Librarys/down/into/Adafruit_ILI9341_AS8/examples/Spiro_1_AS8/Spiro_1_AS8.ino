// Spiro
// Rainbow patern generator

// IMPORTANT: Adafruit_ILI9341_AS8 LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// DEFAULT IS THE UNO SHIELD
// SEE RELEVANT COMMENTS IN Adafruit_ILI9341_AS8.h FOR SETUP.

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

// Assign human-readable names to some common 16-bit color values:
#define	ILI9341_BLACK   0x0000
#define	ILI9341_BLUE    0x001F
#define	ILI9341_RED     0xF800
#define	ILI9341_GREEN   0x07E0
#define ILI9341_CYAN    0x07FF
#define ILI9341_MAGENTA 0xF81F
#define ILI9341_YELLOW  0xFFE0
#define ILI9341_WHITE   0xFFFF

Adafruit_ILI9341_AS8 tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Adafruit_ILI9341_AS8 tft;

unsigned long runTime = 0;

float sx = 0, sy = 0;
uint16_t x0 = 0, x1 = 0, y0 = 0, y1 = 0;

void setup()
{
  randomSeed(analogRead(A0));
  // Setup the LCD
  tft.reset();
  delay(10);
  tft.begin(0x9341);
  tft.setRotation(3);
}

void loop()
{
  runTime = millis();

  tft.fillScreen(ILI9341_BLACK);
  int n = random(2, 19), r = random(20, 100), colour = 0; //rainbow();
  
  for (long i = 0; i < (360 * n); i++) {
    sx = cos((i / n - 90) * 0.0174532925);
    sy = sin((i / n - 90) * 0.0174532925);
    x0 = sx * (120 - r) + 159;
    y0 = sy * (120 - r) + 119;


    sy = cos(((i % 360) - 90) * 0.0174532925);
    sx = sin(((i % 360) - 90) * 0.0174532925);
    x1 = sx * r + x0;
    y1 = sy * r + y0;
    tft.drawPixel(x1, y1, rainbow(i % 128)); //colour);
  }
  
  r = random(20, 100);//r = r / random(2,4);
  for (long i = 0; i < (360 * n); i++) {
    sx = cos((i / n - 90) * 0.0174532925);
    sy = sin((i / n - 90) * 0.0174532925);
    x0 = sx * (120 - r) + 159;
    y0 = sy * (120 - r) + 119;


    sy = cos(((i % 360) - 90) * 0.0174532925);
    sx = sin(((i % 360) - 90) * 0.0174532925);
    x1 = sx * r + x0;
    y1 = sy * r + y0;
    tft.drawPixel(x1, y1, rainbow(i % 128)); //colour);
  }


  delay(2000);
}

unsigned int rainbow(int value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to red = blue
  //int value = random (128);
  byte red = 0; // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;// Green is the middle 6 bits
  byte blue = 0; // Blue is the bottom 5 bits

  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}


