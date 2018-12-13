// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

#ifndef _ADAFRUIT_ILI9341_AS8_H_
#define _ADAFRUIT_ILI9341_AS8_H_

#if ARDUINO >= 100
 #include "Arduino.h"
 #include "Print.h"
#else
 #include "WProgram.h"
#endif

#include "Adafruit_GFX_AS8.h"
#include <avr/pgmspace.h>

// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F
#define ILI9341_GREY        0x5AEB

class Adafruit_ILI9341_AS8 : public Adafruit_GFX_AS8 {

 public:

  Adafruit_ILI9341_AS8(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t rst);
  Adafruit_ILI9341_AS8(void);

  void     begin(uint16_t id = 0x9325);
  void     drawPixel(int16_t x, int16_t y, uint16_t color);
  void     drawPixel(uint16_t color);
  void     drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
  void     drawFastHLine(int16_t x0, int16_t y0, int16_t w, uint16_t color);
  void     drawFastVLine(int16_t x0, int16_t y0, int16_t h, uint16_t color);
  void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
  void     fillScreen(uint16_t color);
  void     reset(void);
  void     setRegisters8(uint8_t *ptr, uint8_t n);
  void     setRegisters16(uint16_t *ptr, uint8_t n);
  void     setRotation(uint8_t x);
  void     invertDisplay(boolean i);
       // These methods are public in order for BMP examples to work:
  void     setAddrWindow(int x1, int y1, int x2, int y2),
           drawStart(void),
           drawEnd(void),
           pushColors(uint16_t *data, uint8_t len),
           pushColors(uint8_t *data, uint8_t len);
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b),
           readPixel(int16_t x, int16_t y),
           readID(void);
  uint32_t readReg(uint8_t r);

 private:

  void     init(),
           // These items may have previously been defined as macros
           // in pin_magic.h.  If not, function versions are declared:
#ifndef write8
    write8(uint8_t value),
#endif
#ifndef setWriteDir
    setWriteDir(void),
#endif
#ifndef setReadDir
    setReadDir(void),
#endif
#ifndef writeRegister8
    writeRegister8(uint8_t a, uint8_t d),
#endif
#ifndef writeRegister16
    writeRegister16(uint16_t a, uint16_t d),
#endif
    writeRegister24(uint8_t a, uint32_t d),
    writeRegister32(uint8_t a, uint32_t d),
#ifndef writeRegisterPair
    writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d),
#endif
    setLR(void),
    flood(uint16_t color, uint32_t len);
  uint8_t  driver;

#ifndef read8
  uint8_t  read8fn(void);
  #define  read8isFunctionalized
#endif

#ifndef USE_ADAFRUIT_SHIELD_PINOUT

  #ifdef __AVR__
    volatile uint8_t *csPort    , *cdPort    , *wrPort    , *rdPort;
             uint8_t  csPinSet  ,  cdPinSet  ,  wrPinSet  ,  rdPinSet  ,
                      csPinUnset,  cdPinUnset,  wrPinUnset,  rdPinUnset,
                      _reset;
  #endif
  #if defined(__SAM3X8E__)
	     Pio      *csPort    , *cdPort    , *wrPort    , *rdPort;
	     uint32_t csPinSet  ,  cdPinSet  ,  wrPinSet  ,  rdPinSet  ,
	              csPinUnset,  cdPinUnset,  wrPinUnset,  rdPinUnset,
	              _reset;
#endif
  
#endif
};

// For compatibility with sketches written for older versions of library.
// Color function name was changed to 'color565' for parity with 2.2" LCD
// library.
#define Color565 color565

#endif
