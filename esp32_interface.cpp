
#include "esp32_interface.h"

#include "../../../main/peripheral/gpio.h"
#include "../../../main/peripheral/spi.h"
#include "../../../main/utils/time_util.h"
#include "../../../main/utils/logging.h"


#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "freertos/semphr.h"

//#include "esp32-hal-spi.h"
//#include "esp32-hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "rom/gpio.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "soc/rtc.h"




struct spi_struct_t {
    spi_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};

typedef struct spi_struct_t spi_t;

#if CONFIG_DISABLE_HAL_LOCKS
#define SPI_MUTEX_LOCK()
#define SPI_MUTEX_UNLOCK()

static spi_t _spi_bus_array[4] = {
    {(volatile spi_dev_t *)(DR_REG_SPI0_BASE), 0},
    {(volatile spi_dev_t *)(DR_REG_SPI1_BASE), 1},
    {(volatile spi_dev_t *)(DR_REG_SPI2_BASE), 2},
    {(volatile spi_dev_t *)(DR_REG_SPI3_BASE), 3}
};
#else
#define SPI_MUTEX_LOCK()    do {} while (xSemaphoreTake(spi->lock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK()  xSemaphoreGive(spi->lock)

static spi_t _spi_bus_array[4] = {
    {(volatile spi_dev_t *)(DR_REG_SPI0_BASE), NULL, 0},
    {(volatile spi_dev_t *)(DR_REG_SPI1_BASE), NULL, 1},
    {(volatile spi_dev_t *)(DR_REG_SPI2_BASE), NULL, 2},
    {(volatile spi_dev_t *)(DR_REG_SPI3_BASE), NULL, 3}
};
#endif



static const char *IF_TAG = "DispIf";

static spi_t spi3;
static spi_t *spi;


void pinMode(uint16_t aPinNumber, uint8_t aPinMode)
{
    // init in main, leaf blank
    logDebug(LOG_REGION_DRIVER, "PinMode", (char*)IF_TAG);
    return;
}

void digitalWrite(uint16_t aPinNumber, bool aState)
{
    gpioSetOutputNoError(aPinNumber, aState);
}

bool digitalRead(uint16_t aPinNumber)
{
    return gpioGetState(aPinNumber);
}

void delay(uint32_t aMs)
{
    timeUtilsDelayMs(aMs);
}

void yield(void)
{
    // ?? Used in fast mode (ESP Mode)
    timeUtilsDelayMs(0);
    return;
}


SPIClass::SPIClass(void)
{
    spi3.dev = &SPI3;
    spi = &spi3;
}

uint32_t SPIClass::__spiTranslate32(uint32_t data)
{
    union {
        uint32_t l;
        uint8_t b[4];
    } out;
    out.l = data;
    return out.b[3] | (out.b[2] << 8) | (out.b[1] << 16) | (out.b[0] << 24);
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

void SPIClass::setFrequency(uint32_t aFrequency)
{
    return;
}

uint8_t SPIClass::transfer(uint8_t aByte)
{
    uint8_t rx = 0;

    spiTransfer(SPI_DEVICE_LCD, &aByte, 1, NULL, true, &rx);
    
    return rx;
}

void SPIClass::write(uint8_t aByte)
{
    //SPI_MUTEX_LOCK();
    // SPI3.mosi_dlen.usr_mosi_dbitlen = 7;
    // SPI3.miso_dlen.usr_miso_dbitlen = 0;
    // SPI3.data_buf[0] = aByte;
    // SPI3.cmd.usr = 1;
    // while(spi->dev->cmd.usr);
    //SPI_MUTEX_UNLOCK();
    spiTransfer(SPI_DEVICE_LCD, &aByte, 1, NULL, true, NULL);
}

void SPIClass::write16(uint16_t aWord)
{
    if(!SPI3.ctrl.wr_bit_order)
    {
        aWord = (aWord >> 8) | (aWord << 8);
    }
    //SPI_MUTEX_LOCK();
    // SPI3.mosi_dlen.usr_mosi_dbitlen = 15;
    // SPI3.miso_dlen.usr_miso_dbitlen = 0;
    // SPI3.data_buf[0] = aWord;
    // SPI3.cmd.usr = 1;
    // while(SPI3.cmd.usr);
    //SPI_MUTEX_UNLOCK();
    spiTransfer(SPI_DEVICE_LCD, (uint8_t*)&aWord, 2, NULL, true, NULL);
}

void SPIClass::write32(uint32_t aDoubleWord)
{
    if(!SPI3.ctrl.wr_bit_order)
    {
        aDoubleWord = __spiTranslate32(aDoubleWord);
    }
    //SPI_MUTEX_LOCK();
    // SPI3.mosi_dlen.usr_mosi_dbitlen = 31;
    // SPI3.miso_dlen.usr_miso_dbitlen = 0;
    // SPI3.data_buf[0] = aDoubleWord;
    // SPI3.cmd.usr = 1;
    // while(SPI3.cmd.usr);
    //SPI_MUTEX_UNLOCK();
    spiTransfer(SPI_DEVICE_LCD, (uint8_t*)&aDoubleWord, 4, NULL, true, NULL);
}
