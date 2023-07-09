
#include "my_MCP3008.h"
#include <SPI.h>

bool my_MCP3008::begin(uint8_t cs, SPIClass* theSPI) {
  hwSPI = true;

  this->cs = cs;

  pinMode(this->cs, OUTPUT);
  digitalWrite(this->cs, HIGH);
  _spi = theSPI;
  _spi->begin();
  /* SPI.begin(); */
  return true;
}

bool my_MCP3008::begin(uint8_t sck, uint8_t mosi, uint8_t miso,
                             uint8_t cs) {
  hwSPI = false;

  this->sck = sck;
  this->mosi = mosi;
  this->miso = miso;
  this->cs = cs;

  pinMode(this->sck, OUTPUT);
  pinMode(this->mosi, OUTPUT);
  pinMode(this->miso, INPUT);
  pinMode(this->cs, OUTPUT);

  digitalWrite(this->sck, LOW);
  digitalWrite(this->mosi, LOW);
  digitalWrite(this->cs, HIGH);

  return true;
}

int my_MCP3008::readADC(uint8_t channel) {
  if ((channel < 0) || (channel > 7))
    return -1;
  return SPIxADC(channel, false);
}

int my_MCP3008::readADCDifference(uint8_t differential) {
  if ((differential < 0) || (differential > 7))
    return -1;
  return SPIxADC(differential, true);
}

// SPI transfer for ADC read
int my_MCP3008::SPIxADC(uint8_t channel, bool differential) {
  byte command, sgldiff;

  if (differential) {
    sgldiff = 0;
  } else {
    sgldiff = 1;
  }

  command = ((0x01 << 7) |             // start bit
             (sgldiff << 6) |          // single or differential
             ((channel & 0x07) << 3)); // channel number

  if (hwSPI) {
    byte b0, b1, b2;

    _spi->beginTransaction(
        SPISettings(MCP3008_SPI_MAX, MCP3008_SPI_ORDER, MCP3008_SPI_MODE));
    digitalWrite(cs, LOW);

    b0 = _spi->transfer(command);
    b1 = _spi->transfer(0x00);
    b2 = _spi->transfer(0x00);

    digitalWrite(cs, HIGH);
    _spi->endTransaction();

    return 0x3FF & ((b0 & 0x01) << 9 | (b1 & 0xFF) << 1 | (b2 & 0x80) >> 7);

  } else {

    uint16_t outBuffer, inBuffer = 0;

    digitalWrite(cs, LOW);

    // 5 command bits + 1 null bit + 10 data bits = 16 bits
    outBuffer = command << 8;
    for (int c = 0; c < 16; c++) {
      digitalWrite(mosi, (outBuffer >> (15 - c)) & 0x01);
      digitalWrite(sck, HIGH);
      digitalWrite(sck, LOW);
      inBuffer <<= 1;
      if (digitalRead(miso))
        inBuffer |= 0x01;
    }

    digitalWrite(cs, HIGH);

    return inBuffer & 0x3FF;
  }
}
