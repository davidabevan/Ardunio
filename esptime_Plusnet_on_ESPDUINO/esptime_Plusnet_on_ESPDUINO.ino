/*
 Udp NTP Client
 Get the time from a Network Time Protocol (NTP) time server
 Demonstrates use of UDP sendPacket and ReceivePacket
 For more on NTP time servers and the messages needed to communicate with them,
 see http://en.wikipedia.org/wiki/Network_Time_Protocol
 created 4 Sep 2010
 by Michael Margolis
 modified 9 Apr 2012
 by Tom Igoe
 updated for the ESP8266 12 Apr 2015 
 by Ivan Grokhotkov
 This code is in the public domain.
 */

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

char ssid[] = "PLUSNET-9P9QR6";  //  your network SSID (name)
char pass[] = "3bb7bc7cd3";       // your network password


unsigned int localPort = 2390;      // local port to listen for UDP packets

IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
#include <Arduino.h>
#include <Adafruit_ILI9341.h>

//Wiring Diagram
//****************************************************
//TFT LCD Pin VCC                 *
//TFT LCD Pin GND                 *
//TFT LCD Pin CS  to GPIO_5       *
//TFT LCD Pin RST to RST          *
//TFT LCD Pin DC to GPIO_2        *
//TFT LCD Pin MOSI to GPIO_13     *
//TFT LCD Pin CLK to GPIO_14      *
//TFT LCD Pin LED to +3.3 V.      *
//TFT LCD Pin MISO ( not use )    *
//****************************************************
// For the Adafruit shield, these are the default.
#define TFT_DC 2
#define TFT_CS 5

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println();
   tft.begin();
   tft.setRotation(3);
   tft.fillScreen(ILI9341_BLACK);
   tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
tft.setTextWrap(false); // Don't wrap text to next line

  // We start by connecting to a WiFi network
  tft.print("Connecting to ");
  tft.println(ssid);
  WiFi.begin(ssid, pass);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    tft.print(".");
  }
  tft.println("");
  
  tft.println("WiFi connected");
  tft.println("IP address: ");
  tft.println(WiFi.localIP());

  tft.println("Starting UDP");
  udp.begin(localPort);
  tft.print("Local port: ");
  tft.println(udp.localPort());
 
}

void loop()
{
   tft.fillScreen(ILI9341_BLUE);//fillscreen allows for screen refresh of time data
    tft.setCursor(20, 50);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  
  int cb = udp.parsePacket();
  if (!cb) {
    tft.println("no packet yet");
  }
  else {
    tft.print("packet received, length=");
    tft.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
   // Serial.print("Seconds since Jan 1 1900 = " );
   // Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
   // Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
   // Serial.println(epoch);

//tft.fillScreen(ILI9341_BLACK);
    // print the hour, minute and second:
    tft.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    tft.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    tft.print(':');
    if ( ((epoch % 3600) / 60) < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      tft.print('0');
    }
    tft.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    tft.print(':');
    if ( (epoch % 60) < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      tft.print('0');
    }
    tft.println(epoch % 60); // print the second
  }
  // wait ten seconds before asking for the time again
  delay(10000);
  
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  tft.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
