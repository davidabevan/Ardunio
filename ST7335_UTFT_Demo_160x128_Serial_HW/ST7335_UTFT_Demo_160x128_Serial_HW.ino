// UTFT_Demo_160x128_Serial_HW
// Copyright (C)2015 Dimitris Lampridis. All right reserved
//
// based on original UTFT_Demo_160x128_Serial:
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// This program is a demo of how to use most of the functions
// of the library with a supported display module.
//
// This demo was made for modules with a screen resolution
// of 160x128 pixels.
//
// This demo implements the serial interface via hardware.
//
// This program requires the UTFT library.
//

#include <UTFT.h>
#include <SPI.h> // not necessary if Arduino IDE version >=1.6.6
#include <pgmspace.h>
// Declare which fonts we will be using
extern uint8_t SmallFont[];
extern unsigned int homett[0x1000];
// Modify the line below to match your display and wiring:
//UTFT(byte model, int RS, int WR, int CS, int RST, int DC);
//in UTFT, RS is the data (MOSI), WR is the clock (SCK)
//UTFT myGLCD ( ST7735, 5, 4, 2 );//Serial_HW
UTFT myGLCD ( ST7735, 13, 14, 5, 4, 2 );//serial_SW
/*
 * In addition to the original bit-banging communication method 
 * of the UTFT library, this project also introduces support 
 * for true hardware SPI communication 
 * (only supported on ESP8266 for now, sorry for that) for serial interface 
 * displays. A new UTFT constructor has been added for this purpose:

UTFT(byte model, int CS, int RST, int DC=0);

When the above constructor is invoked, the library will automatically 
use the hardware SPI pins for SCK and MOSI (GPIO14 and GPIO13 respectively on the ESP8266).
 */
void setup (  ) {
    randomSeed ( analogRead ( 0 ) );

    // Setup the LCD
    myGLCD.InitLCD (  );
    myGLCD.setFont ( SmallFont );
}

void loop (  ) {
   

    // Clear the screen and draw the frame
    myGLCD.clrScr (  );
    //Boxes
  myGLCD.setColor(VGA_NAVY);
  myGLCD.fillRect(0,288,476,480);

//  myGLCD.fillRect(0,304,476,344);
  
  myGLCD.setColor(VGA_NAVY);
  myGLCD.fillRect(0,0,540,283); //Main out window
  myGLCD.setColor(VGA_BLACK);
  myGLCD.fillRect(10,10,530,278); //Main black window
  myGLCD.setColor(VGA_NAVY);
  myGLCD.fillRect(540,278,800,283);
  
  //Draw outdoor temp 
  myGLCD.fillRect(14, 14  , 188, 100);
  myGLCD.setColor(VGA_BLACK);
  myGLCD.fillRect(18, 18  , 184, 96);
  myGLCD.setColor(VGA_NAVY);
  
  //Draw Feelslike temp
  myGLCD.fillRect(14, 104  , 188, 190);
  myGLCD.setColor(VGA_BLACK);
  myGLCD.fillRect(18, 108  , 184, 186);
  myGLCD.setColor(VGA_NAVY);
  
  //Draw humidity
  myGLCD.fillRect(192, 14  , 316, 190);
  myGLCD.setColor(VGA_BLACK);
  myGLCD.fillRect(196, 18  , 312, 186);
  myGLCD.setColor(VGA_NAVY);
  
  //Draw pressure
  myGLCD.fillRect(14, 194  , 316, 250);
  myGLCD.setColor(VGA_BLACK);
  myGLCD.fillRect(18, 198  , 312, 246);
  myGLCD.setColor(VGA_NAVY);
  
  
  //draw home temp window
  myGLCD.setColor(VGA_NAVY);
  myGLCD.fillRect(320,10,530,86);
  myGLCD.setColor(VGA_BLACK);
  myGLCD.fillRect(324,17,526,82);
 // myGLCD.drawBitmap (332, 16, 64, 64, homett);
myGLCD.drawBitmap (16, 16, 64, 64, homett);
  myGLCD.setColor(VGA_NAVY);
  myGLCD.fillRect(320,  86,330, 283);
  
  //Forecast days background
  myGLCD.setColor(VGA_BLACK);
  myGLCD.fillRect(8,324,157,360);
  myGLCD.fillRect(164,324,313,360);
  myGLCD.fillRect(320,324,469,360);
  



    
   myGLCD.printNumI ( millis (  ), CENTER, 115 );

    delay ( 10000 );
}
