
#include "esp32_interface.h"

#include "../../../main/peripheral/gpio.h"
#include "../../../main/peripheral/spi.h"
#include "../../../main/utils/time_util.h"
#include "../../../main/utils/logging.h"


static const char *IF_TAG = "DispIf";


void pinMode(uint16_t aPinNumber, uint8_t aPinMode)
{
    // init in main, leaf blank
    logDebug(LOG_REGION_DRIVER, "PinMode", (char*)IF_TAG);
    return;
}

void digitalWrite(uint16_t aPinNumber, bool aState)
{
    gpioSetOutput(aPinNumber, aState);
}

bool digitalRead(uint16_t aPinNumber)
{
    return gpioGetState(aPinNumber);
}

void delay(uint32_t aMs)
{
    timeUtilsDelayMs(aMs);
}


SPIClass::SPIClass(void)
{
}

void SPIClass::begin(void)
{
    return;
}

void SPIClass::setBitOrder(uint8_t aBitOrder)
{
    return;
}

void SPIClass::setDataMode(uint8_t aSpiMode)
{
    return;
}

uint8_t SPIClass::transfer(uint8_t aByte)
{
    uint8_t rx = 0;

    if (spiTransfer(SPI_DEVICE_LCD, &aByte, 1, NULL, true, &rx) != SPI_ERROR_NONE)
    {
        logError(LOG_REGION_DRIVER, "Transfer Display", (char*)IF_TAG);
        return 0;
    }
    return rx;
}
