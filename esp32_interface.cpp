
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
#include "esp32/rom/ets_sys.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp32/rom/gpio.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "soc/rtc.h"

#define CONFIG_DISABLE_HAL_LOCKS 1


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
#define SPI_MUTEX_LOCK()    //do {} while (xSemaphoreTake(spi->lock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK()  //xSemaphoreGive(spi->lock)

static spi_t _spi_bus_array[4] = {
    {(volatile spi_dev_t *)(DR_REG_SPI0_BASE), NULL, 0},
    {(volatile spi_dev_t *)(DR_REG_SPI1_BASE), NULL, 1},
    {(volatile spi_dev_t *)(DR_REG_SPI2_BASE), NULL, 2},
    {(volatile spi_dev_t *)(DR_REG_SPI3_BASE), NULL, 3}
};
#endif


#include "soc/gpio_sig_map.h"

void IRAM_ATTR pinMatrixOutAttach(uint8_t pin, uint8_t function, bool invertOut, bool invertEnable)
{
    gpio_matrix_out(pin, function, invertOut, invertEnable);
}
// void pinMatrixOutDetach(uint8_t pin, bool invertOut, bool invertEnable);

void IRAM_ATTR pinMatrixInAttach(uint8_t pin, uint8_t signal, bool inverted)
{
    gpio_matrix_in(pin, signal, inverted);
}
// void pinMatrixInDetach(uint8_t signal, bool high, bool inverted);


static const char *IF_TAG = "DispIf";

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

// #define SPI_CLK_IDX(p)  ((p==0)?SPICLK_OUT_IDX:((p==1)?SPICLK_OUT_IDX:((p==2)?HSPICLK_OUT_IDX:((p==3)?VSPICLK_OUT_IDX:0))))
// #define SPI_MISO_IDX(p) ((p==0)?SPIQ_OUT_IDX:((p==1)?SPIQ_OUT_IDX:((p==2)?HSPIQ_OUT_IDX:((p==3)?VSPIQ_OUT_IDX:0))))
// #define SPI_MOSI_IDX(p) ((p==0)?SPID_IN_IDX:((p==1)?SPID_IN_IDX:((p==2)?HSPID_IN_IDX:((p==3)?VSPID_IN_IDX:0))))

// #define SPI_SPI_SS_IDX(n)   ((n==0)?SPICS0_OUT_IDX:((n==1)?SPICS1_OUT_IDX:((n==2)?SPICS2_OUT_IDX:SPICS0_OUT_IDX)))
// #define SPI_HSPI_SS_IDX(n)   ((n==0)?HSPICS0_OUT_IDX:((n==1)?HSPICS1_OUT_IDX:((n==2)?HSPICS2_OUT_IDX:HSPICS0_OUT_IDX)))
// #define SPI_VSPI_SS_IDX(n)   ((n==0)?VSPICS0_OUT_IDX:((n==1)?VSPICS1_OUT_IDX:((n==2)?VSPICS2_OUT_IDX:VSPICS0_OUT_IDX)))
// #define SPI_SS_IDX(p, n)   ((p==0)?SPI_SPI_SS_IDX(n):((p==1)?SPI_SPI_SS_IDX(n):((p==2)?SPI_HSPI_SS_IDX(n):((p==3)?SPI_VSPI_SS_IDX(n):0))))

// #define SPI_CS_MASK_ALL 0x7

SPIClass::SPIClass(void)
{
    spi = &_spi_bus_array[0];

    // // Bus Init
    // spi->dev->slave.trans_done = 0;
    // spi->dev->slave.slave_mode = 0;
    // spi->dev->pin.val = 0;
    // spi->dev->user.val = 0;
    // spi->dev->user1.val = 0;
    // spi->dev->ctrl.val = 0;
    // spi->dev->ctrl1.val = 0;
    // spi->dev->ctrl2.val = 0;
    // spi->dev->clock.val = 0;

    // spi->num = 0;

    // DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI3_CLK_EN);
    // DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI3_RST);

    // int8_t sck = 18;
    // pinMode(sck, OUTPUT);
    // pinMatrixOutAttach(sck, SPI_CLK_IDX(spi->num), false, false);

    // int8_t miso = 19;
    // pinMode(sck, INPUT);
    // pinMatrixInAttach(miso, SPI_MISO_IDX(spi->num), false);

    // int8_t mosi = 23;
    // pinMode(mosi, OUTPUT);
    // pinMatrixOutAttach(mosi, SPI_MOSI_IDX(spi->num), false, false);

    // int8_t ss = 5;
    // pinMode(ss, OUTPUT);
    // pinMatrixOutAttach(ss, SPI_SS_IDX(spi->num, 0), false, false);
    // spi->dev->pin.val &= ~((1 << 0) & SPI_CS_MASK_ALL);

    // spi->dev->clock.val = 0x013c1001;
    // // Mode
    // spi->dev->pin.ck_idle_edge = 0;
    // spi->dev->user.ck_out_edge = 0;

    // // MSB first
    // spi->dev->ctrl.wr_bit_order = 0;
    // spi->dev->ctrl.rd_bit_order = 0;
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
    // spi->dev->mosi_dlen.usr_mosi_dbitlen = 7;
    // spi->dev->miso_dlen.usr_miso_dbitlen = 7;
    // spi->dev->data_buf[0] = aByte;
    // spi->dev->cmd.usr = 1;
    // while(spi->dev->cmd.usr);
    
    // return spi->dev->data_buf[0];
    return rx;
}

void SPIClass::write(uint8_t aByte)
{
    // //SPI_MUTEX_LOCK();
    // spi->dev->mosi_dlen.usr_mosi_dbitlen = 7;
    // spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    // spi->dev->data_buf[0] = aByte;
    // spi->dev->cmd.usr = 1;
    // while(spi->dev->cmd.usr);
    // // //SPI_MUTEX_UNLOCK();

    spiTransfer(SPI_DEVICE_LCD, &aByte, 1, NULL, true, NULL);
}

void SPIClass::write16(uint16_t aWord)
{
    if(!spi->dev->ctrl.wr_bit_order)
    {
        aWord = (aWord >> 8) | (aWord << 8);
    }
    //SPI_MUTEX_LOCK();
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 15;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    spi->dev->data_buf[0] = aWord;
    spi->dev->cmd.usr = 1;
    while(spi->dev->cmd.usr);
    //SPI_MUTEX_UNLOCK();
    
    //spiTransfer(SPI_DEVICE_LCD, (uint8_t*)&aWord, 2, NULL, true, NULL);
}

void SPIClass::write32(uint32_t aDoubleWord)
{
    if(!spi->dev->ctrl.wr_bit_order)
    {
        aDoubleWord = __spiTranslate32(aDoubleWord);
    }
    //SPI_MUTEX_LOCK();
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 31;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    spi->dev->data_buf[0] = aDoubleWord;
    spi->dev->cmd.usr = 1;
    while(spi->dev->cmd.usr);
    //SPI_MUTEX_UNLOCK();

    //spiTransfer(SPI_DEVICE_LCD, (uint8_t*)&aDoubleWord, 4, NULL, true, NULL);
}
