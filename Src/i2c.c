/*
  i2c.c - I2C support for GD32F4xx

  Part of grblHAL driver for GD32F4xx

  Copyright (c) 2018-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include <main.h>

#include "i2c.h"
#include "grbl/hal.h"

#if I2C_ENABLE

#ifndef I2C_KHZ
#define I2C_KHZ 400
#endif

#if I2C_PORT == 1

#ifdef I2C1_ALT_PINMAP
  #define I2C_SCL_PIN 6
  #define I2C_SDA_PIN 7
#else
  #define I2C_SCL_PIN 8
  #define I2C_SDA_PIN 9
#endif

#define I2C_GPIO            GPIOB
#define I2C_GPIO_AF         GPIO_AF_4
#define I2C_CLKENA()        rcu_periph_clock_enable(RCU_I2C0)
#define I2C_IRQn            I2C0_EV_IRQn
#define I2C_IRQHandler      I2C0_EV_IRQHandler

#elif I2C_PORT == 2

#define I2C_SCL_PIN         10
#define I2C_SDA_PIN         11
#define I2C_GPIO            GPIOB
#define I2C_GPIO_AF         GPIO_AF_4
#define I2C_CLKENA()        rcu_periph_clock_enable(RCU_I2C1)
#define I2C_IRQn            I2C1_EV_IRQn
#define I2C_IRQHandler      I2C1_EV_IRQHandler

#else

#error "Unsupported I2C_PORT value"

#endif

#define I2Cport(p) I2CportI(p)
#define I2CportI(p) I2C ## p

#define I2CPORT I2Cport(I2C_PORT)

static uint8_t keycode = 0;
static keycode_callback_ptr keypad_callback = NULL;
static volatile bool await_rx = false;

static inline bool wait_ready (void)
{
    while(await_rx || i2c_flag_get(I2CPORT, I2C_FLAG_I2CBSY) != RESET) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    return true;
}

static inline bool wait_flag (i2c_flag_enum flag, uint32_t timeout_ms)
{
    uint32_t timeout = hal_get_tick() + timeout_ms;
    while(i2c_flag_get(I2CPORT, flag) == RESET) {
        if(hal_get_tick() > timeout)
            return false;
        if(!hal.stream_blocking_callback())
            return false;
    }
    return true;
}

i2c_cap_t i2c_start (void)
{
    static i2c_cap_t cap = {};

    if(cap.started)
        return cap;

    rcu_periph_clock_enable((I2C_GPIO == GPIOA) ? RCU_GPIOA :
                            (I2C_GPIO == GPIOB) ? RCU_GPIOB :
                            (I2C_GPIO == GPIOC) ? RCU_GPIOC :
                            (I2C_GPIO == GPIOD) ? RCU_GPIOD :
                            (I2C_GPIO == GPIOE) ? RCU_GPIOE :
                            (I2C_GPIO == GPIOF) ? RCU_GPIOF :
                            (I2C_GPIO == GPIOG) ? RCU_GPIOG :
                            (I2C_GPIO == GPIOH) ? RCU_GPIOH : RCU_GPIOI);

    gpio_af_set((uint32_t)I2C_GPIO, I2C_GPIO_AF, (1U << I2C_SCL_PIN) | (1U << I2C_SDA_PIN));
    gpio_mode_set((uint32_t)I2C_GPIO, GPIO_MODE_AF, GPIO_PUPD_PULLUP, (1U << I2C_SCL_PIN) | (1U << I2C_SDA_PIN));
    gpio_output_options_set((uint32_t)I2C_GPIO, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, (1U << I2C_SCL_PIN) | (1U << I2C_SDA_PIN));

    I2C_CLKENA();

    i2c_clock_config(I2CPORT, I2C_KHZ * 1000, I2C_DTCY_2);
    i2c_mode_addr_config(I2CPORT, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0);
    i2c_enable(I2CPORT);

    static const periph_pin_t scl = {
        .function = Output_SCK,
        .group = PinGroup_I2C,
        .port = (void *)I2C_GPIO,
        .pin = I2C_SCL_PIN,
        .mode = { .mask = PINMODE_OD }
    };

    static const periph_pin_t sda = {
        .function = Bidirectional_SDA,
        .group = PinGroup_I2C,
        .port = (void *)I2C_GPIO,
        .pin = I2C_SDA_PIN,
        .mode = { .mask = PINMODE_OD }
    };

    hal.periph_port.register_pin(&scl);
    hal.periph_port.register_pin(&sda);

    cap.started = On;

    return cap;
}

bool i2c_probe (i2c_address_t i2cAddr)
{
    if(!wait_ready())
        return false;

    i2c_start_on_bus(I2CPORT);
    if(!wait_flag(I2C_FLAG_SBSEND, 10))
        return false;

    i2c_master_addressing(I2CPORT, i2cAddr << 1, I2C_TRANSMITTER);
    bool ok = wait_flag(I2C_FLAG_ADDSEND, 10);

    i2c_stop_on_bus(I2CPORT);
    wait_ready();

    return ok;
}

static bool i2c_master_tx (i2c_address_t i2cAddr, uint8_t *buf, size_t size)
{
    if(!wait_ready())
        return false;

    i2c_start_on_bus(I2CPORT);
    if(!wait_flag(I2C_FLAG_SBSEND, 100))
        return false;

    i2c_master_addressing(I2CPORT, i2cAddr << 1, I2C_TRANSMITTER);
    if(!wait_flag(I2C_FLAG_ADDSEND, 100))
        return false;
    i2c_flag_clear(I2CPORT, I2C_FLAG_ADDSEND);

    while(size--) {
        if(!wait_flag(I2C_FLAG_TBE, 100))
            return false;
        i2c_data_transmit(I2CPORT, *buf++);
    }

    if(!wait_flag(I2C_FLAG_BTC, 100))
        return false;

    i2c_stop_on_bus(I2CPORT);

    return true;
}

bool i2c_send (i2c_address_t i2cAddr, uint8_t *buf, size_t size, bool block)
{
    bool ok = i2c_master_tx(i2cAddr, buf, size);
    return ok && (!block || wait_ready());
}

static bool i2c_master_rx (i2c_address_t i2cAddr, uint8_t *buf, size_t size)
{
    if(!wait_ready())
        return false;

    if(size == 0)
        return true;

    i2c_ack_config(I2CPORT, size > 1 ? I2C_ACK_ENABLE : I2C_ACK_DISABLE);

    i2c_start_on_bus(I2CPORT);
    if(!wait_flag(I2C_FLAG_SBSEND, 100))
        return false;

    i2c_master_addressing(I2CPORT, i2cAddr << 1, I2C_RECEIVER);
    if(!wait_flag(I2C_FLAG_ADDSEND, 100))
        return false;
    i2c_flag_clear(I2CPORT, I2C_FLAG_ADDSEND);

    while(size--) {
        if(size == 0)
            i2c_ack_config(I2CPORT, I2C_ACK_DISABLE);

        if(!wait_flag(I2C_FLAG_RBNE, 100))
            return false;

        *buf++ = i2c_data_receive(I2CPORT);
    }

    i2c_stop_on_bus(I2CPORT);

    return true;
}

bool i2c_receive (i2c_address_t i2cAddr, uint8_t *buf, size_t size, bool block)
{
    bool ok = i2c_master_rx(i2cAddr, buf, size);
    return ok && (!block || wait_ready());
}

bool i2c_transfer (i2c_transfer_t *i2c, bool read)
{
    if(!wait_ready())
        return false;

    if(!read) {
        // Write word address then data
        uint8_t addr_buf[2];
        size_t addr_len = i2c->word_addr_bytes;
        if(addr_len == 2) {
            addr_buf[0] = (uint8_t)(i2c->word_addr >> 8);
            addr_buf[1] = (uint8_t)i2c->word_addr;
        } else
            addr_buf[0] = (uint8_t)i2c->word_addr;

        if(!i2c_master_tx(i2c->address, addr_buf, addr_len))
            return false;

        return i2c_master_tx(i2c->address, i2c->data, i2c->count);
    }

    // Read: write word address (no stop), then repeated-start read
    i2c_start_on_bus(I2CPORT);
    if(!wait_flag(I2C_FLAG_SBSEND, 100))
        return false;

    i2c_master_addressing(I2CPORT, i2c->address << 1, I2C_TRANSMITTER);
    if(!wait_flag(I2C_FLAG_ADDSEND, 100))
        return false;
    i2c_flag_clear(I2CPORT, I2C_FLAG_ADDSEND);

    size_t addr_len = i2c->word_addr_bytes;
    while(addr_len--) {
        uint8_t addr_byte = (addr_len == 1 && i2c->word_addr_bytes == 2) ?
                            (uint8_t)(i2c->word_addr >> 8) : (uint8_t)i2c->word_addr;
        if(!wait_flag(I2C_FLAG_TBE, 100))
            return false;
        i2c_data_transmit(I2CPORT, addr_byte);
    }
    if(!wait_flag(I2C_FLAG_BTC, 100))
        return false;

    return i2c_master_rx(i2c->address, i2c->data, i2c->count);
}

bool i2c_get_keycode (i2c_address_t i2cAddr, keycode_callback_ptr callback)
{
    if((await_rx = wait_ready())) {
        keycode = 0;
        keypad_callback = callback;
        // Polling driver: receive one byte synchronously and invoke callback.
        uint8_t rx;
        if(i2c_master_rx(i2cAddr, &rx, 1)) {
            keycode = rx;
            if(keypad_callback && keycode != 0) {
                keypad_callback(keycode);
                keypad_callback = NULL;
            }
            await_rx = false;
            return true;
        }
    }

    await_rx = false;
    return false;
}

#if TRINAMIC_ENABLE && TRINAMIC_I2C

static uint16_t axis = 0xFF;
static const uint8_t tmc_addr = I2C_ADR_I2CBRIDGE << 1;

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    uint8_t buffer[5] = {0};
    TMC_spi_status_t status = 0;

    if(driver.axis != axis) {
        buffer[0] = driver.axis | 0x80;
        i2c_master_tx(tmc_addr >> 1, buffer, 1);
        axis = driver.axis;
    }

    i2c_transfer_t i2c = {
        .address = tmc_addr >> 1,
        .word_addr = reg->addr.idx,
        .word_addr_bytes = 1,
        .data = buffer,
        .count = 5
    };
    i2c_transfer(&i2c, true);

    status = buffer[0];
    reg->payload.value = buffer[4];
    reg->payload.value |= buffer[3] << 8;
    reg->payload.value |= buffer[2] << 16;
    reg->payload.value |= buffer[1] << 24;

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    uint8_t buffer[4];
    TMC_spi_status_t status = 0;

    if(driver.axis != axis) {
        buffer[0] = driver.axis | 0x80;
        i2c_master_tx(tmc_addr >> 1, buffer, 1);
        axis = driver.axis;
    }

    buffer[0] = (reg->payload.value >> 24) & 0xFF;
    buffer[1] = (reg->payload.value >> 16) & 0xFF;
    buffer[2] = (reg->payload.value >> 8) & 0xFF;
    buffer[3] = reg->payload.value & 0xFF;

    i2c_transfer_t i2c = {
        .address = tmc_addr >> 1,
        .word_addr = reg->addr.idx,
        .word_addr_bytes = 1,
        .data = buffer,
        .count = 4
    };

    reg->addr.write = 1;
    i2c_transfer(&i2c, false);
    reg->addr.write = 0;

    return status;
}

#endif // TRINAMIC_ENABLE && TRINAMIC_I2C

void I2C_IRQHandler (void)
{
    // Polling driver: no interrupt handling required.
}

#endif // I2C_ENABLE
