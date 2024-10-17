
#ifndef my_MCP3008_h
#define my_MCP3008_h

#include <Arduino.h>
#include <SPI.h>

#define MCP3008_SPI_MAX_5V 3600000         ///< SPI MAX Value on 5V pin
#define MCP3008_SPI_MAX_3V 1350000         ///< SPI MAX Value on 3V pin
#define MCP3008_SPI_MAX MCP3008_SPI_MAX_3V ///< SPI MAX Value
#define MCP3008_SPI_ORDER MSBFIRST         ///<  SPI ORDER
#define MCP3008_SPI_MODE SPI_MODE0         ///< SPI MODE

class my_MCP3008 {
public:
  bool begin(uint8_t cs = SS, SPIClass* theSPI = &SPI);
  bool begin(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t cs);
  int readADC(uint8_t channel);
  int readADCDifference(uint8_t differential);

private:
  uint8_t cs;
  uint8_t mosi;
  uint8_t miso;
  uint8_t sck;
  bool hwSPI;
  int SPIxADC(uint8_t channel, bool differential);
  SPIClass* _spi;
};

#endif
