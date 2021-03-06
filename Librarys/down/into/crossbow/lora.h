/*
  This file was derived from Arduino LoRa library originally licensed as MIT
  https://github.com/sandeepmistry/arduino-LoRa
*/

#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>

#define LORA_DEFAULT_SS_PIN    10
#define LORA_DEFAULT_RESET_PIN 9
#define LORA_DEFAULT_DIO0_PIN  2

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

class LoRaClass {
public:
  LoRaClass();

  int begin(long frequency);
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket();

  void endPacketAsync();
  bool isTransmitting();

  int parsePacket(int size = 0);
  int packetRssi();
  float packetSnr();

  void write(uint8_t byte);
  void write(uint8_t buffer[], size_t size);
  int available();
  int read();
  int fastRead();
  void read(uint8_t buffer[], uint8_t size);

  void onReceive(void(*callback)(int));

  void receive(int size = 0);
  void idle();
  void sleep();

  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();

  byte random();

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);

private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  void readRegister(uint8_t address, uint8_t buffer[], size_t size);
  void writeRegister(uint8_t address, uint8_t buffer[], size_t size);
  void bufferTransfer(uint8_t address, uint8_t buffer[], uint8_t size);

  static void onDio0Rise();

private:
  SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;
  int _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
};

extern LoRaClass LoRa;

#endif
