/*
  spi.c - SPI support for SD card plugins

  Part of grblHAL driver for GD32F4xx

  Copyright (c) 2020-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#include "main.h"
#include "driver.h"

#if SPI_ENABLE

#define SPIport(p) SPIportI(p)
#define SPIportI(p) SPI##p

#define SPIPORT SPIport(SPI_PORT)

// Derive SPI pin mapping from SPI_PORT (STM32F4xx style).
#if SPI_PORT == 0
  #define SPI_SCK_PORT      GPIOA
  #define SPI_SCK_PIN       5
  #define SPI_MISO_PORT     GPIOA
  #define SPI_MISO_PIN      6
  #define SPI_MOSI_PORT     GPIOA
  #define SPI_MOSI_PIN      7
  #define SPI_AF            GPIO_AF_5
#else
  #error "Add SPI pin mapping for SPI_PORT"
#endif

static spi_parameter_struct spi_init_struct = {
    .trans_mode           = SPI_TRANSMODE_FULLDUPLEX,
    .device_mode          = SPI_MASTER,
    .frame_size           = SPI_FRAMESIZE_8BIT,
    .clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE,
    .nss                  = SPI_NSS_SOFT,
    .prescale             = SPI_PSC_256,
    .endian               = SPI_ENDIAN_MSB
};

spi_cap_t spi_start (spi_slave_t *device)
{
    static bool init = false;

    if(!init) {

        rcu_periph_clock_enable(RCU_GPIOA);
        rcu_periph_clock_enable(RCU_SPI0);

        gpio_af_set((uint32_t)SPI_SCK_PORT, SPI_AF, 1U << SPI_SCK_PIN);
        gpio_af_set((uint32_t)SPI_MISO_PORT, SPI_AF, 1U << SPI_MISO_PIN);
        gpio_af_set((uint32_t)SPI_MOSI_PORT, SPI_AF, 1U << SPI_MOSI_PIN);

        gpio_mode_set((uint32_t)SPI_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, 1U << SPI_SCK_PIN);
        gpio_mode_set((uint32_t)SPI_MISO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, 1U << SPI_MISO_PIN);
        gpio_mode_set((uint32_t)SPI_MOSI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, 1U << SPI_MOSI_PIN);

        gpio_output_options_set((uint32_t)SPI_SCK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, 1U << SPI_SCK_PIN);
        gpio_output_options_set((uint32_t)SPI_MISO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, 1U << SPI_MISO_PIN);
        gpio_output_options_set((uint32_t)SPI_MOSI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, 1U << SPI_MOSI_PIN);

        spi_i2s_deinit(SPIPORT);
        spi_init(SPIPORT, &spi_init_struct);
        spi_enable(SPIPORT);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = (void *)SPI_SCK_PORT,
            .pin = SPI_SCK_PIN,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = (void *)SPI_MISO_PORT,
            .pin = SPI_MISO_PIN,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = (void *)SPI_MOSI_PORT,
            .pin = SPI_MOSI_PIN,
            .mode = { .mask = PINMODE_NONE }
        };

        hal.periph_port.register_pin(&sck);
        hal.periph_port.register_pin(&sdo);
        hal.periph_port.register_pin(&sdi);

        init = true;
    }

    return (spi_cap_t){ .started = On };
}

uint8_t spi_get_byte (void)
{
    spi_i2s_data_transmit(SPIPORT, 0xFF);

    while(RESET == spi_i2s_flag_get(SPIPORT, SPI_FLAG_RBNE));

    return (uint8_t)spi_i2s_data_receive(SPIPORT);
}

uint8_t spi_put_byte (uint8_t byte)
{
    spi_i2s_data_transmit(SPIPORT, byte);

    while(RESET == spi_i2s_flag_get(SPIPORT, SPI_FLAG_TBE));
    while(RESET == spi_i2s_flag_get(SPIPORT, SPI_FLAG_RBNE));

    // Clear overrun by reading data and status registers
    (void)SPI_DATA(SPIPORT);
    (void)SPI_STAT(SPIPORT);

    return (uint8_t)spi_i2s_data_receive(SPIPORT);
}

bool spi_write (uint8_t *data, uint16_t len)
{
    while(len--)
        spi_put_byte(*data++);

    return true;
}

bool spi_read (uint8_t *data, uint16_t len)
{
    while(len--)
        *data++ = spi_get_byte();

    return true;
}

bool spi_select (spi_slave_t *device)
{
    if(device == NULL)
        return false;

    uint16_t prescaler;
    switch(device->f_clock) {
        case 2:  prescaler = SPI_PSC_2;  break;
        case 4:  prescaler = SPI_PSC_4;  break;
        case 8:  prescaler = SPI_PSC_8;  break;
        case 16: prescaler = SPI_PSC_16; break;
        case 32: prescaler = SPI_PSC_32; break;
        case 64: prescaler = SPI_PSC_64; break;
        case 128:prescaler = SPI_PSC_128;break;
        default: prescaler = SPI_PSC_256;break;
    }

    spi_disable(SPIPORT);
    SPI_CTL0(SPIPORT) &= ~SPI_CTL0_PSC;
    SPI_CTL0(SPIPORT) |= prescaler;
    spi_enable(SPIPORT);

    DIGITAL_OUT((uint32_t)device->cs_port, device->cs_pin, 0);

    return true;
}

bool spi_deselect (spi_slave_t *device)
{
    if(device == NULL)
        return false;

    DIGITAL_OUT((uint32_t)device->cs_port, device->cs_pin, 1);

    return true;
}

#endif // SPI_ENABLE
