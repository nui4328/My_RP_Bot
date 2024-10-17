#include "MCP3208.h"
#include <SPI.h>

bool my_MCP3208::begin(uint8_t cs, SPIClass* theSPI) {
  hwSPI = true;

  this->cs = cs;

  pinMode(this->cs, OUTPUT);
  digitalWrite(this->cs, HIGH);
  _spi = theSPI;
  _spi->begin();

  return true;
}

bool my_MCP3208::begin(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t cs) {
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

int my_MCP3208::readADC(uint8_t channel) {
  if ((channel < 0) || (channel > 7))
    return -1;
  return SPIxADC(channel, false);
}

int my_MCP3208::readADCDifference(uint8_t differential) {
  if ((differential < 0) || (differential > 7))
    return -1;
  return SPIxADC(differential, true);
}

// SPI transfer for ADC read
int my_MCP3208::SPIxADC(uint8_t channel, bool differential) {
  byte command, sgldiff;

  if (differential) {
    sgldiff = 0;
  } else {
    sgldiff = 1;
  }

  // เริ่ม bit สำหรับการสั่งงาน MCP3208 (มีการขยายบิตสำหรับการอ่าน 12 บิต)
  command = ((0x01 << 6) |             // start bit
             (sgldiff << 5) |          // single or differential
             ((channel & 0x07) << 2)); // channel number

  if (hwSPI) {
    byte b0, b1, b2;

    _spi->beginTransaction(
        SPISettings(MCP3208_SPI_MAX, MCP3208_SPI_ORDER, MCP3208_SPI_MODE));
    digitalWrite(cs, LOW);

    b0 = _spi->transfer(command);  // ส่งคำสั่ง
    b1 = _spi->transfer(0x00);     // รับข้อมูลบิตที่ 11-4
    b2 = _spi->transfer(0x00);     // รับข้อมูลบิตที่ 3-0

    digitalWrite(cs, HIGH);
    _spi->endTransaction();

    // ประมวลผลข้อมูล 12 บิตจาก 3 ไบต์ที่รับมา
    return 0xFFF & ((b0 & 0x01) << 11 | (b1 & 0xFF) << 3 | (b2 & 0xE0) >> 5);

  } else {
    uint16_t outBuffer, inBuffer = 0;

    digitalWrite(cs, LOW);

    // 5 command bits + 1 null bit + 12 data bits = 18 bits
    outBuffer = command << 8;
    for (int c = 0; c < 18; c++) {
      digitalWrite(mosi, (outBuffer >> (17 - c)) & 0x01);
      digitalWrite(sck, HIGH);
      digitalWrite(sck, LOW);
      inBuffer <<= 1;
      if (digitalRead(miso))
        inBuffer |= 0x01;
    }

    digitalWrite(cs, HIGH);

    return inBuffer & 0xFFF;  // คืนค่าข้อมูลที่อ่านได้ใน 12 บิต
  }
}
