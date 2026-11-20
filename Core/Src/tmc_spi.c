/*
  tmc_spi.c - driver code for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2023-2026 Terje Io
  SoftSPI implementation Copyright (c) 2021 fitch22

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

#include "driver.h"

#if TRINAMIC_SPI_ENABLE

#include "trinamic/common.h"

// XXXXX replace with something better...
inline static void delay (uint32_t delay)
{
    volatile uint32_t dly = delay;

    while(--dly)
        __ASM volatile ("nop");
}

#if defined(TRINAMIC_SOFTSPI)

// Per driver CS and soft SPI ports, supports 40 bit datagrams

#if TRINAMIC_SPI_ENABLE & TRINAMIC_SPI_20BIT
#error "20 bit Trinamic SPI datagrams not yet supported!"
#endif

#define spi_get_byte() sw_spi_xfer(0)
#define spi_put_byte(d) sw_spi_xfer(d)

static struct {
    gpio_port_t port;
    uint16_t pin;
} cs[TMC_N_MOTORS_MAX];

static uint8_t sw_spi_xfer (uint8_t byte)
{
  uint_fast8_t msk = 0x80, res = 0;

  DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 0);

  do {
    DIGITAL_OUT(TRINAMIC_MOSI_PORT, TRINAMIC_MOSI_PIN, (byte & msk) != 0);
    msk >>= 1;
    delay(10);
    res = (res << 1) | DIGITAL_IN(TRINAMIC_MISO_PORT, TRINAMIC_MISO_PIN);
    DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 1);
    delay(10);
    if (msk)
      DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 0);
  } while (msk);

  return (uint8_t)res;
}

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
  TMC_spi_status_t status;

  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

  datagram->payload.value = 0;

  datagram->addr.write = 0;
  spi_put_byte(datagram->addr.value);
  spi_put_byte(0);
  spi_put_byte(0);
  spi_put_byte(0);
  spi_put_byte(0);

  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);
  delay(10);
  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

  status = spi_put_byte(datagram->addr.value);
  datagram->payload.data[3] = spi_get_byte();
  datagram->payload.data[2] = spi_get_byte();
  datagram->payload.data[1] = spi_get_byte();
  datagram->payload.data[0] = spi_get_byte();

  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

  return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
  TMC_spi_status_t status;

  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

  datagram->addr.write = 1;
  status = spi_put_byte(datagram->addr.value);
  spi_put_byte(datagram->payload.data[3]);
  spi_put_byte(datagram->payload.data[2]);
  spi_put_byte(datagram->payload.data[1]);
  spi_put_byte(datagram->payload.data[0]);

  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

  return status;
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
  if (gpio->group == PinGroup_MotorChipSelect) {
    switch (gpio->function) {

    case Output_MotorChipSelectX:
      cs[X_AXIS].port = (gpio_port_t)gpio->port;
      cs[X_AXIS].pin = gpio->pin;
      break;

    case Output_MotorChipSelectY:
      cs[Y_AXIS].port = (gpio_port_t)gpio->port;
      cs[Y_AXIS].pin = gpio->pin;
      break;

    case Output_MotorChipSelectZ:
      cs[Z_AXIS].port = (gpio_port_t)gpio->port;
      cs[Z_AXIS].pin = gpio->pin;
      break;

    case Output_MotorChipSelectM3:
      cs[3].port = (gpio_port_t)gpio->port;
      cs[3].pin = gpio->pin;
      break;

    case Output_MotorChipSelectM4:
      cs[4].port = (gpio_port_t)gpio->port;
      cs[4].pin = gpio->pin;
      break;

    default:
      break;
    }
  }
}

static void if_init(uint8_t motors, axes_signals_t enabled)
{
  static bool init_ok = false;

  UNUSED(motors);

  if (!init_ok) {

    // Set all output pins: push-pull, no pull-up, slow
    gpio_mode_set(TRINAMIC_MOSI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 1 << TRINAMIC_MOSI_PIN);
    gpio_output_options_set(TRINAMIC_MOSI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, 1 << TRINAMIC_MOSI_PIN);

    gpio_mode_set(TRINAMIC_SCK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 1 << TRINAMIC_SCK_PIN);
    gpio_output_options_set(TRINAMIC_SCK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, 1 << TRINAMIC_SCK_PIN);
    DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 1);

    // Set the input pin: input with pull-up
    gpio_mode_set(TRINAMIC_MISO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 1 << TRINAMIC_MISO_PIN);

    hal.enumerate_pins(true, add_cs_pin, NULL);
  }
}

void tmc_spi_init (void)
{
  static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};

  trinamic_if_init(&driver_if);
}

#else // HW SPI

#ifdef TRINAMIC_SPI_PORT

#define SPIport(p) SPIportI(p)
#define SPIportI(p) SPI ## p

// SPI instance translation STM32 -> GD32: GD32 numbers its SPI peripherals one lower
// than STM32 does (STM32 SPI1 == GD32 SPI0 on APB2, STM32 SPI2 == GD32 SPI1 and
// STM32 SPI3 == GD32 SPI2, both on APB1). Board maps keep STM32 numbering.
// NOTE: STM32 SPI4 has no GD32F425 equivalent - the device only implements
// SPI0, SPI1 and SPI2 (see startup_gd32f425re.S, no SPI3/SPI4/SPI5 vectors).
#if SPI_PORT == 11
#define TMC_SPI_PORT SPIport(0)
#elif TRINAMIC_SPI_PORT == 2
#define TMC_SPI_PORT SPIport(1)
#elif TRINAMIC_SPI_PORT == 3
#define TMC_SPI_PORT SPIport(2)
#elif TRINAMIC_SPI_PORT == 4
#error "TRINAMIC_SPI_PORT 4: STM32 SPI4 is not available on GD32F425 (device only implements SPI0, SPI1 and SPI2)"
#else
// Deliberate narrowing vs the STM32 original (which silently accepted any value):
// value 1 is ambiguous here (GD32 SPI1 == STM32 SPI2) and no board map uses it.
#error "Unsupported TRINAMIC_SPI_PORT value"
#endif

#define spi_start(d) _spi_start(d)
#define spi_get_byte() _spi_get_byte()
#define spi_put_byte(d) _spi_put_byte(d)

static spi_parameter_struct spi_port = {
    .device_mode          = SPI_MASTER,
    .trans_mode           = SPI_TRANSMODE_FULLDUPLEX,
    .frame_size           = SPI_FRAMESIZE_8BIT,
    .clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE,
    .nss                  = SPI_NSS_SOFT,
    .prescale             = SPI_PSC_32,
    .endian               = SPI_ENDIAN_MSB
};

static uint8_t _spi_get_byte (void)
{
    spi_i2s_data_transmit(TMC_SPI_PORT, 0xFF); // Writing dummy data into Data register

    while(!spi_i2s_flag_get(TMC_SPI_PORT, SPI_FLAG_RBNE));

    return (uint8_t)spi_i2s_data_receive(TMC_SPI_PORT);
}

static uint8_t _spi_put_byte (uint8_t byte)
{
    spi_i2s_data_transmit(TMC_SPI_PORT, byte);

    while(!spi_i2s_flag_get(TMC_SPI_PORT, SPI_FLAG_TBE));
    while(!spi_i2s_flag_get(TMC_SPI_PORT, SPI_FLAG_RBNE));

    // Clear overrun by reading data and status registers
    (void)SPI_DATA(TMC_SPI_PORT);
    (void)SPI_STAT(TMC_SPI_PORT);

    return (uint8_t)spi_i2s_data_receive(TMC_SPI_PORT);
}

// NOTE: named spi_hw_init, not spi_init, to avoid a clash with the GD32 SPL spi_init().
static void spi_hw_init (void)
{
    static bool init_ok = false;

    if(!init_ok) {

        init_ok = true;

#if TRINAMIC_SPI_PORT == 2

        rcu_periph_clock_enable(RCU_SPI1);

        gpio_af_set(GPIOC, GPIO_AF_5, GPIO_PIN_2|GPIO_PIN_3);
        gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2|GPIO_PIN_3);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_2|GPIO_PIN_3);

        gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_13);
        gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_13);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = (void *)GPIOB,
            .pin = 13,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = (void *)GPIOC,
            .pin = 3,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = (void *)GPIOC,
            .pin = 2,
            .mode = { .mask = PINMODE_NONE }
        };

#elif TRINAMIC_SPI_PORT == 3

        rcu_periph_clock_enable(RCU_SPI2);

        gpio_af_set(GPIOC, GPIO_AF_6, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);
        gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = (void *)GPIOC,
            .pin = 10,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = (void *)GPIOC,
            .pin = 11,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = (void *)GPIOC,
            .pin = 12,
            .mode = { .mask = PINMODE_NONE }
        };

#elif TRINAMIC_SPI_PORT == 4

        // STM32 SPI4 (GPIOE: SCK = 12, MISO = 13, MOSI = 14) - there is no GD32F425
        // equivalent: the device only implements SPI0, SPI1 and SPI2 (see
        // startup_gd32f425re.S, no SPI3/SPI4/SPI5 vectors exist).
#error "TRINAMIC_SPI_PORT 4: STM32 SPI4 is not available on GD32F425 (device only implements SPI0, SPI1 and SPI2)"

#endif

        // The extra deinit (peripheral reset) is needed because GD32's spi_init()
        // does not clear the CRC enable bit - HAL_SPI_Init does (CRC disabled).
        spi_i2s_deinit(TMC_SPI_PORT);
        spi_init(TMC_SPI_PORT, &spi_port);
        spi_enable(TMC_SPI_PORT);

        hal.periph_port.register_pin(&sck);
        hal.periph_port.register_pin(&sdo);
        hal.periph_port.register_pin(&sdi);
    }
}

#endif // TRINAMIC_SPI_PORT

#ifndef TRINAMIC_SPI_PORT
static spi_slave_t dev = {
    .f_clock = 32 // same divider as the STM32 HAL SPI_BAUDRATEPRESCALER_32, spi_select() maps it to SPI_PSC_32
};
#endif

#if TRINAMIC_SPI_ENABLE & TRINAMIC_SPI_CS_SINGLE

#if TRINAMIC_SPI_ENABLE & TRINAMIC_SPI_20BIT
#error "20 bit Trinamic SPI datagrams not yet supported!"
#endif

static uint_fast8_t n_motors;
static TMC_spi_datagram_t datagram[TMC_N_MOTORS_MAX];

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    static TMC_spi_status_t status = 0;

    uint8_t res;
    uint_fast8_t idx = n_motors;

    datagram[driver.seq].addr.value = reg->addr.value;
    datagram[driver.seq].addr.write = 0;

#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs.port, cs.pin, 0);
#else
    spi_select(&dev);
#endif

    do {
        spi_put_byte(datagram[--idx].addr.value);
        spi_put_byte(0);
        spi_put_byte(0);
        spi_put_byte(0);
        spi_put_byte(0);
    } while(idx);

    delay(100);
#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs.port, cs.pin, 1);
#else
    spi_deselect(&dev);
#endif
    delay(50);
#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs.port, cs.pin, 0);
#else
    spi_select(&dev);
#endif

    idx = n_motors;
    do {
        res = spi_put_byte(datagram[--idx].addr.value);

        if(idx == driver.seq) {
            status = res;
            reg->payload.data[3] = spi_get_byte();
            reg->payload.data[2] = spi_get_byte();
            reg->payload.data[1] = spi_get_byte();
            reg->payload.data[0] = spi_get_byte();
        } else {
            spi_get_byte();
            spi_get_byte();
            spi_get_byte();
            spi_get_byte();
        }
    } while(idx);

    delay(100);
#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs.port, cs.pin, 1);
#else
    spi_deselect(&dev);
#endif
    delay(50);

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    TMC_spi_status_t status = 0;

    uint8_t res;
    uint_fast8_t idx = n_motors;

    memcpy(&datagram[driver.seq], reg, sizeof(TMC_spi_datagram_t));
    datagram[driver.seq].addr.write = 1;

#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs.port, cs.pin, 0);
#else
    spi_select(&dev);
#endif

    do {
        res = spi_put_byte(datagram[--idx].addr.value);
        spi_put_byte(datagram[idx].payload.data[3]);
        spi_put_byte(datagram[idx].payload.data[2]);
        spi_put_byte(datagram[idx].payload.data[1]);
        spi_put_byte(datagram[idx].payload.data[0]);

        if(idx == driver.seq) {
            status = res;
            datagram[idx].addr.idx = 0; // TMC_SPI_STATUS_REG;
            datagram[idx].addr.write = 0;
        }
    } while(idx);

    delay(100);
#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs.port, cs.pin, 1);
#else
    spi_deselect(&dev);
#endif
    delay(50);

    return status;
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
    if(gpio->function == Output_MotorChipSelect) {
        dev.cs_pin = gpio->pin;
        dev.cs_port = gpio->port;
    }
}

static void if_init (uint8_t motors, axes_signals_t axisflags)
{
    n_motors = motors;
    hal.enumerate_pins(true, add_cs_pin, NULL);
}

void tmc_spi_init (void)
{
    trinamic_driver_if_t driver = {
        .on_drivers_init = if_init
    };

#ifdef TRINAMIC_SPI_PORT
    spi_hw_init();
#else
    spi_start(&dev);
#endif

    uint_fast8_t idx = TMC_N_MOTORS_MAX;
    do {
        datagram[--idx].addr.idx = 0; //TMC_SPI_STATUS_REG;
    } while(idx);

    trinamic_if_init(&driver);
}

#else // separate CS pins

static struct {
    gpio_port_t port;
    uint16_t pin;
} cs[TMC_N_MOTORS_MAX];

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);
#else
    dev.cs_pin = cs[driver.id].pin;
    dev.cs_port = (void *)cs[driver.id].port;
    spi_select(&dev);
#endif

    datagram->payload.value = 0;

    datagram->addr.write = 0;
    spi_put_byte(datagram->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);
#else
    spi_deselect(&dev);
#endif
    delay(10);
#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);
#else
    spi_select(&dev);
#endif

    status = spi_put_byte(datagram->addr.value);
    datagram->payload.data[3] = spi_get_byte();
    datagram->payload.data[2] = spi_get_byte();
    datagram->payload.data[1] = spi_get_byte();
    datagram->payload.data[0] = spi_get_byte();

#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);
#else
    spi_deselect(&dev);
#endif

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);
#else
    dev.cs_pin = cs[driver.id].pin;
    dev.cs_port = (void *)cs[driver.id].port;
    spi_select(&dev);
#endif

    datagram->addr.write = 1;
    status = spi_put_byte(datagram->addr.value);
    spi_put_byte(datagram->payload.data[3]);
    spi_put_byte(datagram->payload.data[2]);
    spi_put_byte(datagram->payload.data[1]);
    spi_put_byte(datagram->payload.data[0]);

#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);
#else
    spi_deselect(&dev);
#endif

    return status;
}

TMC_spi20_datagram_t tmc_spi20_write (trinamic_motor_t driver, TMC_spi20_datagram_t *datagram)
{
    TMC_spi20_datagram_t status = {0};

#ifdef TRINAMIC_SPI_PORT
    while(spi_i2s_flag_get(TMC_SPI_PORT, SPI_FLAG_TRANS)) {};
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);
#else
    dev.cs_pin = cs[driver.id].pin;
    dev.cs_port = (void *)cs[driver.id].port;
    spi_select(&dev);
#endif

    status.data[2] = spi_put_byte(datagram->data[2]);
    status.data[1] = spi_put_byte(datagram->data[1]);
    status.data[0] = spi_put_byte(datagram->data[0]);

#ifdef TRINAMIC_SPI_PORT
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);
#else
    spi_deselect(&dev);
#endif

    status.value >>= 4;

    return status;
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
    if (gpio->group == PinGroup_MotorChipSelect)
      switch (gpio->function) {

        case Output_MotorChipSelectX:
            cs[X_AXIS].port = (gpio_port_t)gpio->port;
            cs[X_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectY:
            cs[Y_AXIS].port = (gpio_port_t)gpio->port;
            cs[Y_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectZ:
            cs[Z_AXIS].port = (gpio_port_t)gpio->port;
            cs[Z_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM3:
            cs[3].port = (gpio_port_t)gpio->port;
            cs[3].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM4:
            cs[4].port = (gpio_port_t)gpio->port;
            cs[4].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM5:
            cs[5].port = (gpio_port_t)gpio->port;
            cs[5].pin = gpio->pin;
            break;

        default:
            break;
    }
}

static void if_init (uint8_t motors, axes_signals_t enabled)
{
    UNUSED(motors);

    hal.enumerate_pins(true, add_cs_pin, NULL);
}

void tmc_spi_init (void)
{
    static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};

#ifdef TRINAMIC_SPI_PORT
    spi_hw_init();
#else
    spi_start(&dev);
#endif
    trinamic_if_init(&driver_if);
}
#endif
#endif // HW SPI
#endif // TRINAMIC_SPI_ENABLE
