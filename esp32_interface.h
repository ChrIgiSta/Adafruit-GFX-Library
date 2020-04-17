#ifndef _ADAFRUIT_ESP32_INTERFACE_H_
#define _ADAFRUIT_ESP32_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>

#include "../../../main/settings.h"

#include "esp_attr.h"


#define HIGH 1
#define LOW 0

#define OUTPUT 0
#define INPUT 1

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

#define MSBFIRST 1
#define MSBLAST  0

//#define ICACHE_RODATA_ATTR  _SECTION_ATTR_IMPL(".irom1", __COUNTER__)
#define PROGMEM   ROMFN_ATTR
#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))


//#define ESP8266 1


void digitalWrite(uint16_t aPinNumber, bool aState);

bool digitalRead(uint16_t aPinNumber);

void pinMode(uint16_t aPinNumber, uint8_t aPinMode);

void delay(uint32_t aMs);

void yield(void);


class SPIClass
{
public:
    SPIClass(void);

    void begin(void);
    uint8_t transfer(uint8_t aByte);
    void setBitOrder(uint8_t aByteOrder);
    void setDataMode(uint8_t aSpiMode);
    void setFrequency(uint32_t aFrequency);
    void write(uint8_t aByte);
    void write16(uint16_t aWord);
    void write32(uint32_t aDoubleWord);

protected:

private:
    uint32_t __spiTranslate32(uint32_t data);
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SPI)
extern SPIClass SPI;
#endif

#endif /* _ADAFRUIT_ESP32_INTERFACE_H_ */
