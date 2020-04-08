
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
    logDebug(LOG_REGION_DRIVER, "Digital Write:", (char*)IF_TAG);
    printf(" -> GPIO %d, State: %d\r\n", aPinNumber, aState);
    gpioSetOutput(aPinNumber, aState);
}

bool digitalRead(uint16_t aPinNumber)
{
    logDebug(LOG_REGION_DRIVER, "Digital Read:", (char*)IF_TAG);
    printf(" -> GPIO %d\r\n", aPinNumber);
    return gpioGetState(aPinNumber);
}

void delay(uint32_t aMs)
{
    logDebug(LOG_REGION_DRIVER, "Delay", (char*)IF_TAG);
    timeUtilsDelayMs(aMs);
}


SPIClass::SPIClass(void)
{
}

void SPIClass::begin(void)
{
    logDebug(LOG_REGION_DRIVER, "begin", (char*)IF_TAG);
    return;
}

void SPIClass::setBitOrder(uint8_t aBitOrder)
{
    logDebug(LOG_REGION_DRIVER, "setBitOrder", (char*)IF_TAG);
    return;
}

void SPIClass::setDataMode(uint8_t aSpiMode)
{
    logDebug(LOG_REGION_DRIVER, "setDataMode", (char*)IF_TAG);
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
    logDebug(LOG_REGION_DRIVER, "Transfer Display", (char*)IF_TAG);
    printf(" -> Data: %d\r\n", aByte);

    return rx;
}
