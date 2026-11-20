/*

  serial.c - serial port implementation for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

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

#include <string.h>

#include "main.h"
#include "driver.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/stream.h"

#ifdef SERIAL_PORT
#define HAVE_UART0 // SERIAL_PORT is user-defined; 0 = USART0 is a valid port (GD32 0-based numbering)
static stream_rx_buffer_t rxbuf = {0};
static stream_tx_buffer_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command;
static const io_stream_t *serialInit (uint32_t baud_rate);
#else
#define SERIAL_PORT 0
#endif

#ifdef SERIAL1_PORT
#ifndef HAVE_UART0
#error "Add SERIAL_PORT before adding SERIAL1_PORT!"
#endif
#define HAVE_UART1
static stream_rx_buffer_t rxbuf1 = {0};
static stream_tx_buffer_t txbuf1 = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command1;
static const io_stream_t *serial1Init(uint32_t baud_rate);
#else
#define SERIAL1_PORT 0
#endif

#ifdef SERIAL2_PORT
#ifndef HAVE_UART1
#error "Add SERIAL1_PORT before adding SERIAL2_PORT!"
#endif
#define HAVE_UART2
static stream_rx_buffer_t rxbuf2 = {0};
static stream_tx_buffer_t txbuf2 = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command2;
static const io_stream_t *serial2Init(uint32_t baud_rate);
#else
#define SERIAL2_PORT 0
#endif

#ifdef HAVE_UART0

#if defined(HAVE_UART1) && SERIAL_PORT == SERIAL1_PORT || defined(HAVE_UART2) && SERIAL_PORT == SERIAL2_PORT
#error Conflicting use of UART peripherals!
#endif

// Port numbering is GD32 0-based: N selects instance N (0..4 = USART0, USART1,
// USART2, UART3, UART4) with its default pins - port 0 uses the PB6/PB7 remap,
// the board default - and NX a remap of instance N. Exception: 10 is a legacy
// encoding for USART0 on its default PA9/PA10 pins (as in the previous port).
// There is no STM32F4 USART6 counterpart on GD32F425. The APB1/APB2 clock
// domain does not need to be selected here: usart_baudrate_set() derives it.
#if SERIAL_PORT == 10          // legacy encoding: USART0 on its default pins PA9/PA10
#define UART0            usart(0)
#define UART0_IRQ        usartINT(0)
#define UART0_IRQHandler usartHANDLER(0)
#define UART0_CLK_En()   usartCLKEN(0)
#elif (SERIAL_PORT >= 11 && SERIAL_PORT < 19)
#define UART0            usart(1)
#define UART0_IRQ        usartINT(1)
#define UART0_IRQHandler usartHANDLER(1)
#define UART0_CLK_En()   usartCLKEN(1)
#elif (SERIAL_PORT >= 20 && SERIAL_PORT < 29)
#define UART0            usart(2)
#define UART0_IRQ        usartINT(2)
#define UART0_IRQHandler usartHANDLER(2)
#define UART0_CLK_En()   usartCLKEN(2)
#elif (SERIAL_PORT >= 30 && SERIAL_PORT < 39)
#define UART0            usart(3)
#define UART0_IRQ        usartINT(3)
#define UART0_IRQHandler usartHANDLER(3)
#define UART0_CLK_En()   usartCLKEN(3)
#elif (SERIAL_PORT >= 40 && SERIAL_PORT < 49)
#define UART0            usart(4)
#define UART0_IRQ        usartINT(4)
#define UART0_IRQHandler usartHANDLER(4)
#define UART0_CLK_En()   usartCLKEN(4)
#else
#define UART0            usart(SERIAL_PORT)
#define UART0_IRQ        usartINT(SERIAL_PORT)
#define UART0_IRQHandler usartHANDLER(SERIAL_PORT)
#define UART0_CLK_En()   usartCLKEN(SERIAL_PORT)
#endif

#if SERIAL_PORT == 0    // USART0 remapped to PB6/PB7
#define UART0_TX_PIN 6
#define UART0_RX_PIN 7
#define UART0_PORT GPIOB
#define UART0_AF GPIO_AF_7
#elif SERIAL_PORT == 10 // USART0 PA9/PA10
#define UART0_TX_PIN 9
#define UART0_RX_PIN 10
#define UART0_PORT GPIOA
#define UART0_AF GPIO_AF_7
#elif SERIAL_PORT == 1  // USART1 PA2/PA3
#define UART0_TX_PIN 2
#define UART0_RX_PIN 3
#define UART0_PORT GPIOA
#define UART0_AF GPIO_AF_7
#elif SERIAL_PORT == 11 // USART1 remapped PD5/PD6
#define UART0_TX_PIN 5
#define UART0_RX_PIN 6
#define UART0_PORT GPIOD
#define UART0_AF GPIO_AF_7
#elif SERIAL_PORT == 2  // USART2 PB10/PB11
#define UART0_TX_PIN 10
#define UART0_RX_PIN 11
#define UART0_PORT GPIOB
#define UART0_AF GPIO_AF_7
#elif SERIAL_PORT == 21 // USART2 remapped PC10/PC11
#define UART0_TX_PIN 10
#define UART0_RX_PIN 11
#define UART0_PORT GPIOC
#define UART0_AF GPIO_AF_7
#elif SERIAL_PORT == 22 // USART2 remapped PD8/PD9
#define UART0_TX_PIN 8
#define UART0_RX_PIN 9
#define UART0_PORT GPIOD
#define UART0_AF GPIO_AF_7
#elif SERIAL_PORT == 3  // UART3 PC10/PC11
#define UART0_TX_PIN 10
#define UART0_RX_PIN 11
#define UART0_PORT GPIOC
#define UART0_AF GPIO_AF_8
#elif SERIAL_PORT == 31 // UART3 remapped PA0/PA1
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define UART0_PORT GPIOA
#define UART0_AF GPIO_AF_8
#elif SERIAL_PORT == 4  // UART4 PC12 (TX) / PD2 (RX)
#define UART0_TX_PIN 12
#define UART0_RX_PIN 2
#define UART0_TX_PORT GPIOC
#define UART0_RX_PORT GPIOD
#define UART0_AF GPIO_AF_8
#else
#error Code has to be added to support serial port
#endif

#ifdef UART0_PORT
#define UART0_TX_PORT UART0_PORT
#define UART0_RX_PORT UART0_PORT
#endif

#endif // SERIAL_PORT

#ifdef HAVE_UART1

#if defined(HAVE_UART0) && SERIAL1_PORT == SERIAL_PORT || defined(HAVE_UART2) && SERIAL1_PORT == SERIAL2_PORT
#error Conflicting use of UART peripherals!
#endif

#if SERIAL1_PORT == 10          // legacy encoding: USART0 on its default pins PA9/PA10
#define UART1            usart(0)
#define UART1_IRQ        usartINT(0)
#define UART1_IRQHandler usartHANDLER(0)
#define UART1_CLK_En()   usartCLKEN(0)
#elif (SERIAL1_PORT >= 11 && SERIAL1_PORT < 19)
#define UART1            usart(1)
#define UART1_IRQ        usartINT(1)
#define UART1_IRQHandler usartHANDLER(1)
#define UART1_CLK_En()   usartCLKEN(1)
#elif (SERIAL1_PORT >= 20 && SERIAL1_PORT < 29)
#define UART1            usart(2)
#define UART1_IRQ        usartINT(2)
#define UART1_IRQHandler usartHANDLER(2)
#define UART1_CLK_En()   usartCLKEN(2)
#elif (SERIAL1_PORT >= 30 && SERIAL1_PORT < 39)
#define UART1            usart(3)
#define UART1_IRQ        usartINT(3)
#define UART1_IRQHandler usartHANDLER(3)
#define UART1_CLK_En()   usartCLKEN(3)
#elif (SERIAL1_PORT >= 40 && SERIAL1_PORT < 49)
#define UART1            usart(4)
#define UART1_IRQ        usartINT(4)
#define UART1_IRQHandler usartHANDLER(4)
#define UART1_CLK_En()   usartCLKEN(4)
#else
#define UART1            usart(SERIAL1_PORT)
#define UART1_IRQ        usartINT(SERIAL1_PORT)
#define UART1_IRQHandler usartHANDLER(SERIAL1_PORT)
#define UART1_CLK_En()   usartCLKEN(SERIAL1_PORT)
#endif

#if SERIAL1_PORT == 0    // USART0 remapped to PB6/PB7
#define UART1_TX_PIN 6
#define UART1_RX_PIN 7
#define UART1_PORT GPIOB
#define UART1_AF GPIO_AF_7
#elif SERIAL1_PORT == 10 // USART0 PA9/PA10
#define UART1_TX_PIN 9
#define UART1_RX_PIN 10
#define UART1_PORT GPIOA
#define UART1_AF GPIO_AF_7
#elif SERIAL1_PORT == 1  // USART1 PA2/PA3
#define UART1_TX_PIN 2
#define UART1_RX_PIN 3
#define UART1_PORT GPIOA
#define UART1_AF GPIO_AF_7
#elif SERIAL1_PORT == 11 // USART1 remapped PD5/PD6
#define UART1_TX_PIN 5
#define UART1_RX_PIN 6
#define UART1_PORT GPIOD
#define UART1_AF GPIO_AF_7
#elif SERIAL1_PORT == 2  // USART2 PB10/PB11
#define UART1_TX_PIN 10
#define UART1_RX_PIN 11
#define UART1_PORT GPIOB
#define UART1_AF GPIO_AF_7
#elif SERIAL1_PORT == 21 // USART2 remapped PC10/PC11
#define UART1_TX_PIN 10
#define UART1_RX_PIN 11
#define UART1_PORT GPIOC
#define UART1_AF GPIO_AF_7
#elif SERIAL1_PORT == 22 // USART2 remapped PD8/PD9
#define UART1_TX_PIN 8
#define UART1_RX_PIN 9
#define UART1_PORT GPIOD
#define UART1_AF GPIO_AF_7
#elif SERIAL1_PORT == 3  // UART3 PC10/PC11
#define UART1_TX_PIN 10
#define UART1_RX_PIN 11
#define UART1_PORT GPIOC
#define UART1_AF GPIO_AF_8
#elif SERIAL1_PORT == 31 // UART3 remapped PA0/PA1
#define UART1_TX_PIN 0
#define UART1_RX_PIN 1
#define UART1_PORT GPIOA
#define UART1_AF GPIO_AF_8
#elif SERIAL1_PORT == 4  // UART4 PC12 (TX) / PD2 (RX)
#define UART1_TX_PIN 12
#define UART1_RX_PIN 2
#define UART1_TX_PORT GPIOC
#define UART1_RX_PORT GPIOD
#define UART1_AF GPIO_AF_8
#else
#error Code has to be added to support serial port 1
#endif

#ifdef UART1_PORT
#define UART1_TX_PORT UART1_PORT
#define UART1_RX_PORT UART1_PORT
#endif

#endif // SERIAL1_PORT

#ifdef HAVE_UART2

#if defined(HAVE_UART0) && SERIAL2_PORT == SERIAL_PORT || defined(HAVE_UART1) && SERIAL2_PORT == SERIAL1_PORT
#error Conflicting use of UART peripherals!
#endif

#if SERIAL2_PORT == 10          // legacy encoding: USART0 on its default pins PA9/PA10
#define UART2            usart(0)
#define UART2_IRQ        usartINT(0)
#define UART2_IRQHandler usartHANDLER(0)
#define UART2_CLK_En()   usartCLKEN(0)
#elif (SERIAL2_PORT >= 11 && SERIAL2_PORT < 19)
#define UART2            usart(1)
#define UART2_IRQ        usartINT(1)
#define UART2_IRQHandler usartHANDLER(1)
#define UART2_CLK_En()   usartCLKEN(1)
#elif (SERIAL2_PORT >= 20 && SERIAL2_PORT < 29)
#define UART2            usart(2)
#define UART2_IRQ        usartINT(2)
#define UART2_IRQHandler usartHANDLER(2)
#define UART2_CLK_En()   usartCLKEN(2)
#elif (SERIAL2_PORT >= 30 && SERIAL2_PORT < 39)
#define UART2            usart(3)
#define UART2_IRQ        usartINT(3)
#define UART2_IRQHandler usartHANDLER(3)
#define UART2_CLK_En()   usartCLKEN(3)
#elif (SERIAL2_PORT >= 40 && SERIAL2_PORT < 49)
#define UART2            usart(4)
#define UART2_IRQ        usartINT(4)
#define UART2_IRQHandler usartHANDLER(4)
#define UART2_CLK_En()   usartCLKEN(4)
#else
#define UART2            usart(SERIAL2_PORT)
#define UART2_IRQ        usartINT(SERIAL2_PORT)
#define UART2_IRQHandler usartHANDLER(SERIAL2_PORT)
#define UART2_CLK_En()   usartCLKEN(SERIAL2_PORT)
#endif

#if SERIAL2_PORT == 0    // USART0 remapped to PB6/PB7
#define UART2_TX_PIN 6
#define UART2_RX_PIN 7
#define UART2_PORT GPIOB
#define UART2_AF GPIO_AF_7
#elif SERIAL2_PORT == 10 // USART0 PA9/PA10
#define UART2_TX_PIN 9
#define UART2_RX_PIN 10
#define UART2_PORT GPIOA
#define UART2_AF GPIO_AF_7
#elif SERIAL2_PORT == 1  // USART1 PA2/PA3
#define UART2_TX_PIN 2
#define UART2_RX_PIN 3
#define UART2_PORT GPIOA
#define UART2_AF GPIO_AF_7
#elif SERIAL2_PORT == 11 // USART1 remapped PD5/PD6
#define UART2_TX_PIN 5
#define UART2_RX_PIN 6
#define UART2_PORT GPIOD
#define UART2_AF GPIO_AF_7
#elif SERIAL2_PORT == 2  // USART2 PB10/PB11
#define UART2_TX_PIN 10
#define UART2_RX_PIN 11
#define UART2_PORT GPIOB
#define UART2_AF GPIO_AF_7
#elif SERIAL2_PORT == 21 // USART2 remapped PC10/PC11
#define UART2_TX_PIN 10
#define UART2_RX_PIN 11
#define UART2_PORT GPIOC
#define UART2_AF GPIO_AF_7
#elif SERIAL2_PORT == 22 // USART2 remapped PD8/PD9
#define UART2_TX_PIN 8
#define UART2_RX_PIN 9
#define UART2_PORT GPIOD
#define UART2_AF GPIO_AF_7
#elif SERIAL2_PORT == 3  // UART3 PC10/PC11
#define UART2_TX_PIN 10
#define UART2_RX_PIN 11
#define UART2_PORT GPIOC
#define UART2_AF GPIO_AF_8
#elif SERIAL2_PORT == 31 // UART3 remapped PA0/PA1
#define UART2_TX_PIN 0
#define UART2_RX_PIN 1
#define UART2_PORT GPIOA
#define UART2_AF GPIO_AF_8
#elif SERIAL2_PORT == 4  // UART4 PC12 (TX) / PD2 (RX)
#define UART2_TX_PIN 12
#define UART2_RX_PIN 2
#define UART2_TX_PORT GPIOC
#define UART2_RX_PORT GPIOD
#define UART2_AF GPIO_AF_8
#else
#error Code has to be added to support serial port 2
#endif

#ifdef UART2_PORT
#define UART2_TX_PORT UART2_PORT
#define UART2_RX_PORT UART2_PORT
#endif

#endif // SERIAL2_PORT

#ifdef HAVE_UART0

static bool uart_release (uint8_t instance);
static const io_stream_status_t *get_uart_status (uint8_t instance);

static io_stream_status_t stream_status[] = {
#ifdef HAVE_UART0
    {
        .baud_rate = 115200,
        .format = {
            .width = Serial_8bit,
            .stopbits = Serial_StopBits1,
            .parity = Serial_ParityNone,
        }
    },
#endif
#ifdef HAVE_UART1
    {
        .baud_rate = 115200,
        .format = {
            .width = Serial_8bit,
            .stopbits = Serial_StopBits1,
            .parity = Serial_ParityNone,
        }
    },
#endif
#ifdef HAVE_UART2
    {
        .baud_rate = 115200,
        .format = {
            .width = Serial_8bit,
            .stopbits = Serial_StopBits1,
            .parity = Serial_ParityNone,
        }
    }
#endif
};

static io_stream_properties_t serial[] = {
#ifdef HAVE_UART0
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serialInit,
      .release = uart_release,
      .get_status = get_uart_status
    },
#endif
#ifdef HAVE_UART1
    {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serial1Init,
      .release = uart_release,
      .get_status = get_uart_status
    },
#endif
#ifdef HAVE_UART2
    {
      .type = StreamType_Serial,
      .instance = 2,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serial2Init,
      .release = uart_release,
      .get_status = get_uart_status
    }
#endif
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

#ifdef HAVE_UART0

    static const periph_pin_t tx0 = {
        .function = Output_TX,
        .group = PinGroup_UART + 0,
        .port  = (void *)UART0_TX_PORT,
        .pin   = UART0_TX_PIN,
        .mode  = { .mask = PINMODE_OUTPUT }
    };

    static const periph_pin_t rx0 = {
        .function = Input_RX,
        .group = PinGroup_UART + 0,
        .port = (void *)UART0_RX_PORT,
        .pin = UART0_RX_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&rx0);
    hal.periph_port.register_pin(&tx0);

#endif

#ifdef HAVE_UART1

    static const periph_pin_t tx1 = {
        .function = Output_TX,
        .group = PinGroup_UART + 1,
        .port  = (void *)UART1_TX_PORT,
        .pin   = UART1_TX_PIN,
        .mode  = { .mask = PINMODE_OUTPUT }
    };

    static const periph_pin_t rx1 = {
        .function = Input_RX,
        .group = PinGroup_UART + 1,
        .port = (void *)UART1_RX_PORT,
        .pin = UART1_RX_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&rx1);
    hal.periph_port.register_pin(&tx1);

#endif

#ifdef HAVE_UART2

    static const periph_pin_t tx2 = {
        .function = Output_TX,
        .group = PinGroup_UART + 2,
        .port  = (void *)UART2_TX_PORT,
        .pin   = UART2_TX_PIN,
        .mode  = { .mask = PINMODE_OUTPUT }
    };

    static const periph_pin_t rx2 = {
        .function = Input_RX,
        .group = PinGroup_UART + 2,
        .port = (void *)UART2_RX_PORT,
        .pin = UART2_RX_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&rx2);
    hal.periph_port.register_pin(&tx2);

#endif

    stream_register_streams(&streams);
}

static const io_stream_status_t *get_uart_status (uint8_t instance)
{
    stream_status[instance].flags = serial[instance].flags;

    return &stream_status[instance];
}

static bool uart_release (uint8_t instance)
{
    bool ok;

    if((ok = serial[instance].flags.claimed))
        serial[instance].flags.claimed = Off;

    return ok;
}

#ifdef RS485_DIR_PORT

static void rs485SetDirection (bool tx)
{
    DIGITAL_OUT(RS485_DIR_PORT, RS485_DIR_PIN, tx);
}

#endif // RS485_DIR_PORT

#else

void serialRegisterStreams (void) {}; // No serial ports!

#endif

#ifdef HAVE_UART0

//
// Returns number of free characters in serial input buffer
//
static uint16_t serialRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serialRxCount (void)
{
    uint32_t tail = rxbuf.tail, head = rxbuf.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes a character to the serial output stream
//
static bool serialPutC (const uint8_t c)
{
    uint16_t next_head = BUFNEXT(txbuf.head, txbuf);    // Get pointer to next free slot in buffer

    while(txbuf.tail == next_head) {                    // While TX buffer full
        if(!hal.stream_blocking_callback())             // check if blocking for space,
            return false;                               // exit if not (leaves TX buffer in an inconsistent state)
    }
    txbuf.data[txbuf.head] = c;                         // Add data to buffer,
    txbuf.head = next_head;                             // update head pointer and
    usart_interrupt_enable(UART0, USART_INT_TBE);                      // enable TX interrupts

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serialWriteS (const char *s)
{
    uint8_t c, *ptr = (uint8_t *)s;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
static void serialWrite (const uint8_t *s, uint16_t length)
{
    uint8_t *ptr = (uint8_t *)s;

    while(length--)
        serialPutC(*ptr++);
}

//
// Flushes the serial output buffer
//
static void serialTxFlush (void)
{
    usart_interrupt_disable(UART0, USART_INT_TBE);     // Disable TX interrupts
    txbuf.tail = txbuf.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serialTxCount (void)
{
    uint32_t tail = txbuf.tail, head = txbuf.head;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + (usart_flag_get(UART0, USART_FLAG_TC) == RESET ? 1 : 0);
}

//
// serialGetC - returns -1 if no data available
//
static int32_t serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;            // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    int32_t data = (int32_t)rxbuf.data[tail];   // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);          // and update pointer

    return data;
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool serialSetBaudRate (uint32_t baud_rate)
{
    stream_status[0].baud_rate = baud_rate;

    usart_disable(UART0);
    usart_baudrate_set(UART0, baud_rate);
    usart_enable(UART0);

    return true;
}

static bool serialSetFormat (serial_format_t format)
{
    stream_status[0].format = format;

    if(format.parity == Serial_ParityNone) {
        usart_word_length_set(UART0, USART_WL_8BIT);
        usart_parity_config(UART0, USART_PM_NONE);
    } else {
        // STM32 sets the M bit so the frame is 9 bits (8 data + parity); GD32
        // needs an explicit WL change or the frame becomes 7 data + parity.
        usart_word_length_set(UART0, USART_WL_9BIT);
        usart_parity_config(UART0, format.parity == Serial_ParityEven ? USART_PM_EVEN : USART_PM_ODD);
    }

    return true;
}

static bool serialDisable (bool disable)
{
    if(disable)
        usart_interrupt_disable(UART0, USART_INT_RBNE);
    else
        usart_interrupt_enable(UART0, USART_INT_RBNE);

    return true;
}

static bool serialEnqueueRtCommand (uint8_t c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

static const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 0,
        .is_connected = stream_connected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_n =  serialWrite,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .get_tx_buffer_count = serialTxCount,
        .reset_write_buffer = serialTxFlush,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .disable_rx = serialDisable,
        .set_baud_rate = serialSetBaudRate,
        .set_format = serialSetFormat,
#if MODBUS_RTU_STREAM == 0 && defined(RS485_DIR_PORT)
        .set_direction = rs485SetDirection,
#endif
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(!serial[0].flags.claimable || serial[0].flags.claimed)
        return NULL;

    serial[0].flags.claimed = On;

    if(!serial[0].flags.init_ok) {

        UART0_CLK_En();

        gpio_af_set(UART0_TX_PORT, UART0_AF, 1 << UART0_TX_PIN);
        gpio_af_set(UART0_RX_PORT, UART0_AF, 1 << UART0_RX_PIN);
        gpio_mode_set(UART0_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, 1 << UART0_TX_PIN);
        gpio_mode_set(UART0_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, 1 << UART0_RX_PIN);
        gpio_output_options_set(UART0_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, 1 << UART0_TX_PIN);
        gpio_output_options_set(UART0_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, 1 << UART0_RX_PIN);

        usart_deinit(UART0);
        usart_word_length_set(UART0, USART_WL_8BIT);
        usart_stop_bit_set(UART0, USART_STB_1BIT);
        usart_receive_config(UART0, USART_RECEIVE_ENABLE);
        usart_transmit_config(UART0, USART_TRANSMIT_ENABLE);

        nvic_irq_enable(UART0_IRQ, 1, 0);
        usart_interrupt_enable(UART0, USART_INT_RBNE);

        serial[0].flags.init_ok = On;
    }

    stream_set_defaults(&stream, baud_rate);

    return &stream;
}

void UART0_IRQHandler (void)
{
    if(RESET != usart_interrupt_flag_get(UART0, USART_INT_FLAG_RBNE)) {
        char data = usart_data_receive(UART0);
        if(!enqueue_realtime_command((uint8_t)data)) {          // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer
            if(next_head == rxbuf.tail)                         // If buffer full
                rxbuf.overflow = 1;                             // flag overflow
            else {
                rxbuf.data[rxbuf.head] = (uint8_t)data;         // if not add data to buffer
                rxbuf.head = next_head;                         // and update pointer
            }
        }
    }

    if(RESET != usart_interrupt_flag_get(UART0, USART_INT_FLAG_TBE)) {
        uint_fast16_t tail = txbuf.tail;            // Get buffer pointer
        usart_data_transmit(UART0, txbuf.data[tail]);               // Send next character
        txbuf.tail = tail = BUFNEXT(tail, txbuf);   // and increment pointer
        if(tail == txbuf.head)                      // If buffer empty then
            usart_interrupt_disable(UART0, USART_INT_TBE);         // disable UART TX interrupt
   }
}

#endif // SERIAL_PORT

#ifdef HAVE_UART1

//
// Returns number of free characters in serial input buffer
//
static uint16_t serial1RxFree (void)
{
    uint32_t tail = rxbuf1.tail, head = rxbuf1.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serial1RxCount (void)
{
    uint32_t tail = rxbuf1.tail, head = rxbuf1.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serial1RxFlush (void)
{
    rxbuf1.tail = rxbuf1.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serial1RxCancel (void)
{
    rxbuf1.data[rxbuf1.head] = ASCII_CAN;
    rxbuf1.tail = rxbuf1.head;
    rxbuf1.head = BUFNEXT(rxbuf1.head, rxbuf1);
}

//
// Writes a character to the serial output stream
//
static bool serial1PutC (const uint8_t c)
{
    uint32_t next_head = BUFNEXT(txbuf1.head, txbuf1);   // Set and update head pointer

    while(txbuf1.tail == next_head) {           // While TX buffer full
        if(!hal.stream_blocking_callback())     // check if blocking for space,
            return false;                       // exit if not (leaves TX buffer in an inconsistent state)
        usart_interrupt_enable(UART1, USART_INT_TBE);          // Enable TX interrupts???
    }

    txbuf1.data[txbuf1.head] = c;               // Add data to buffer
    txbuf1.head = next_head;                    // and update head pointer

    usart_interrupt_enable(UART1, USART_INT_TBE);              // Enable TX interrupts

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serial1WriteS (const char *s)
{
    uint8_t c, *ptr = (uint8_t *)s;

    while((c = *ptr++) != '\0')
        serial1PutC(c);
}

// Writes a number of characters from a buffer to the serial output stream, blocks if buffer full
//
static void serial1Write (const uint8_t *s, uint16_t length)
{
    uint8_t *ptr = (uint8_t *)s;

    while(length--)
        serial1PutC(*ptr++);
}

//
// Flushes the serial output buffer
//
static void serial1TxFlush (void)
{
    usart_interrupt_disable(UART1, USART_INT_TBE);     // Disable TX interrupts
    txbuf1.tail = txbuf1.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serial1TxCount (void)
{
    uint32_t tail = txbuf1.tail, head = txbuf1.head;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + (usart_flag_get(UART1, USART_FLAG_TC) == RESET ? 1 : 0);
}

//
// serialGetC - returns -1 if no data available
//
static int32_t serial1GetC (void)
{
    uint_fast16_t tail = rxbuf1.tail;           // Get buffer pointer

    if(tail == rxbuf1.head)
        return -1; // no data available

    int32_t data =(int32_t) rxbuf1.data[tail];  // Get next character
    rxbuf1.tail = BUFNEXT(tail, rxbuf1);        // and update pointer

    return data;
}

static bool serial1SuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf1, suspend);
}

static bool serial1SetBaudRate (uint32_t baud_rate)
{
    stream_status[1].baud_rate = baud_rate;

    usart_disable(UART1);
    usart_baudrate_set(UART1, baud_rate);
    usart_enable(UART1);

    return true;
}

static bool serial1SetFormat (serial_format_t format)
{
    stream_status[1].format = format;

    if(format.parity == Serial_ParityNone) {
        usart_word_length_set(UART1, USART_WL_8BIT);
        usart_parity_config(UART1, USART_PM_NONE);
    } else {
        // STM32 sets the M bit so the frame is 9 bits (8 data + parity); GD32
        // needs an explicit WL change or the frame becomes 7 data + parity.
        usart_word_length_set(UART1, USART_WL_9BIT);
        usart_parity_config(UART1, format.parity == Serial_ParityEven ? USART_PM_EVEN : USART_PM_ODD);
    }

    return true;
}

static bool serial1Disable (bool disable)
{
    if(disable)
        usart_interrupt_disable(UART1, USART_INT_RBNE);
    else
        usart_interrupt_enable(UART1, USART_INT_RBNE);

    return true;
}

static bool serial1EnqueueRtCommand (uint8_t c)
{
    return enqueue_realtime_command1(c);
}

static enqueue_realtime_command_ptr serial1SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command1;

    if(handler)
        enqueue_realtime_command1 = handler;

    return prev;
}

static const io_stream_t *serial1Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 1,
        .is_connected = stream_connected,
        .read = serial1GetC,
        .write = serial1WriteS,
        .write_n =  serial1Write,
        .write_char = serial1PutC,
        .enqueue_rt_command = serial1EnqueueRtCommand,
        .get_rx_buffer_free = serial1RxFree,
        .get_rx_buffer_count = serial1RxCount,
        .get_tx_buffer_count = serial1TxCount,
        .reset_write_buffer = serial1TxFlush,
        .reset_read_buffer = serial1RxFlush,
        .cancel_read_buffer = serial1RxCancel,
        .suspend_read = serial1SuspendInput,
        .disable_rx = serial1Disable,
        .set_baud_rate = serial1SetBaudRate,
        .set_format = serial1SetFormat,
#if MODBUS_RTU_STREAM == 1 && defined(RS485_DIR_PORT)
        .set_direction = rs485SetDirection,
#endif
        .set_enqueue_rt_handler = serial1SetRtHandler
    };

    if(!serial[1].flags.claimable || serial[1].flags.claimed)
        return NULL;

    serial[1].flags.claimed = On;

    if(!serial[1].flags.init_ok) {

        UART1_CLK_En();

        gpio_af_set(UART1_TX_PORT, UART1_AF, 1 << UART1_TX_PIN);
        gpio_af_set(UART1_RX_PORT, UART1_AF, 1 << UART1_RX_PIN);
        gpio_mode_set(UART1_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, 1 << UART1_TX_PIN);
        gpio_mode_set(UART1_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, 1 << UART1_RX_PIN);
        gpio_output_options_set(UART1_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, 1 << UART1_TX_PIN);
        gpio_output_options_set(UART1_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, 1 << UART1_RX_PIN);

        usart_deinit(UART1);
        usart_word_length_set(UART1, USART_WL_8BIT);
        usart_stop_bit_set(UART1, USART_STB_1BIT);
        usart_receive_config(UART1, USART_RECEIVE_ENABLE);
        usart_transmit_config(UART1, USART_TRANSMIT_ENABLE);

        nvic_irq_enable(UART1_IRQ, 1, 0);
        usart_interrupt_enable(UART1, USART_INT_RBNE);

        serial[1].flags.init_ok = On;
    }

    stream_set_defaults(&stream, baud_rate);

    return &stream;
}

void UART1_IRQHandler (void)
{
    if(RESET != usart_interrupt_flag_get(UART1, USART_INT_FLAG_RBNE)) {
        char data = usart_data_receive(UART1);
        if(!enqueue_realtime_command1((uint8_t)data)) {         // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf1.head, rxbuf1);  // Get and increment buffer pointer
            if(next_head == rxbuf1.tail)                        // If buffer full
                rxbuf1.overflow = 1;                            // flag overflow
            else {
                rxbuf1.data[rxbuf1.head] = (uint8_t)data;       // if not add data to buffer
                rxbuf1.head = next_head;                        // and update pointer
            }
        }
    }

    if(RESET != usart_interrupt_flag_get(UART1, USART_INT_FLAG_TBE)) {
        uint_fast16_t tail = txbuf1.tail;           // Get buffer pointer
        usart_data_transmit(UART1, txbuf1.data[tail]);              // Send next character
        txbuf1.tail = tail = BUFNEXT(tail, txbuf1); // and increment pointer
        if(tail == txbuf1.head)                     // If buffer empty then
            usart_interrupt_disable(UART1, USART_INT_TBE);         // disable UART TX interrupt
   }
}

#endif // SERIAL1_PORT

#ifdef HAVE_UART2

//
// Returns number of free characters in serial input buffer
//
static uint16_t serial2RxFree (void)
{
    uint32_t tail = rxbuf2.tail, head = rxbuf2.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serial2RxCount (void)
{
    uint32_t tail = rxbuf2.tail, head = rxbuf2.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serial2RxFlush (void)
{
    rxbuf2.tail = rxbuf2.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serial2RxCancel (void)
{
    rxbuf2.data[rxbuf2.head] = ASCII_CAN;
    rxbuf2.tail = rxbuf2.head;
    rxbuf2.head = BUFNEXT(rxbuf2.head, rxbuf2);
}

//
// Writes a character to the serial output stream
//
static bool serial2PutC (const uint8_t c)
{
    uint32_t next_head = BUFNEXT(txbuf2.head, txbuf2);   // Set and update head pointer

    while(txbuf2.tail == next_head) {           // While TX buffer full
        if(!hal.stream_blocking_callback())     // check if blocking for space,
            return false;                       // exit if not (leaves TX buffer in an inconsistent state)
        usart_interrupt_enable(UART2, USART_INT_TBE);          // Enable TX interrupts???
    }

    txbuf2.data[txbuf2.head] = c;               // Add data to buffer
    txbuf2.head = next_head;                    // and update head pointer

    usart_interrupt_enable(UART2, USART_INT_TBE);              // Enable TX interrupts

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serial2WriteS (const char *s)
{
    uint8_t c, *ptr = (uint8_t *)s;

    while((c = *ptr++) != '\0')
        serial2PutC(c);
}

// Writes a number of characters from a buffer to the serial output stream, blocks if buffer full
//
static void serial2Write (const uint8_t *s, uint16_t length)
{
    uint8_t *ptr = (uint8_t *)s;

    while(length--)
        serial2PutC(*ptr++);
}

//
// Flushes the serial output buffer
//
static void serial2TxFlush (void)
{
    usart_interrupt_disable(UART2, USART_INT_TBE);     // Disable TX interrupts
    txbuf2.tail = txbuf2.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serial2TxCount (void)
{
    uint32_t tail = txbuf2.tail, head = txbuf2.head;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + (usart_flag_get(UART2, USART_FLAG_TC) == RESET ? 1 : 0);
}

//
// serialGetC - returns -1 if no data available
//
static int32_t serial2GetC (void)
{
    uint_fast16_t tail = rxbuf2.tail;           // Get buffer pointer

    if(tail == rxbuf2.head)
        return -1; // no data available

    int32_t data = (int32_t)rxbuf2.data[tail];  // Get next character
    rxbuf2.tail = BUFNEXT(tail, rxbuf2);        // and update pointer

    return data;
}

static bool serial2SuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf2, suspend);
}

static bool serial2SetBaudRate (uint32_t baud_rate)
{
    stream_status[2].baud_rate = baud_rate;

    usart_disable(UART2);
    usart_baudrate_set(UART2, baud_rate);
    usart_enable(UART2);

    return true;
}

static bool serial2SetFormat (serial_format_t format)
{
    stream_status[2].format = format;

    if(format.parity == Serial_ParityNone) {
        usart_word_length_set(UART2, USART_WL_8BIT);
        usart_parity_config(UART2, USART_PM_NONE);
    } else {
        // STM32 sets the M bit so the frame is 9 bits (8 data + parity); GD32
        // needs an explicit WL change or the frame becomes 7 data + parity.
        usart_word_length_set(UART2, USART_WL_9BIT);
        usart_parity_config(UART2, format.parity == Serial_ParityEven ? USART_PM_EVEN : USART_PM_ODD);
    }

    return true;
}

static bool serial2Disable (bool disable)
{
    if(disable)
        usart_interrupt_disable(UART2, USART_INT_RBNE);
    else
        usart_interrupt_enable(UART2, USART_INT_RBNE);

    return true;
}

static bool serial2EnqueueRtCommand (uint8_t c)
{
    return enqueue_realtime_command2(c);
}

static enqueue_realtime_command_ptr serial2SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command2;

    if(handler)
        enqueue_realtime_command2 = handler;

    return prev;
}

static const io_stream_t *serial2Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 2,
        .is_connected = stream_connected,
        .read = serial2GetC,
        .write = serial2WriteS,
        .write_n =  serial2Write,
        .write_char = serial2PutC,
        .enqueue_rt_command = serial2EnqueueRtCommand,
        .get_rx_buffer_free = serial2RxFree,
        .get_rx_buffer_count = serial2RxCount,
        .get_tx_buffer_count = serial2TxCount,
        .reset_write_buffer = serial2TxFlush,
        .reset_read_buffer = serial2RxFlush,
        .cancel_read_buffer = serial2RxCancel,
        .suspend_read = serial2SuspendInput,
        .disable_rx = serial2Disable,
        .set_baud_rate = serial2SetBaudRate,
        .set_format = serial2SetFormat,
#if MODBUS_RTU_STREAM == 2 && defined(RS485_DIR_PORT)
        .set_direction = rs485SetDirection,
#endif
        .set_enqueue_rt_handler = serial2SetRtHandler
    };

    if(!serial[2].flags.claimable || serial[2].flags.claimed)
        return NULL;

    serial[2].flags.claimed = On;

    if(!serial[2].flags.init_ok) {

        UART2_CLK_En();

        gpio_af_set(UART2_TX_PORT, UART2_AF, 1 << UART2_TX_PIN);
        gpio_af_set(UART2_RX_PORT, UART2_AF, 1 << UART2_RX_PIN);
        gpio_mode_set(UART2_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, 1 << UART2_TX_PIN);
        gpio_mode_set(UART2_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, 1 << UART2_RX_PIN);
        gpio_output_options_set(UART2_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, 1 << UART2_TX_PIN);
        gpio_output_options_set(UART2_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, 1 << UART2_RX_PIN);

        usart_deinit(UART2);
        usart_word_length_set(UART2, USART_WL_8BIT);
        usart_stop_bit_set(UART2, USART_STB_1BIT);
        usart_receive_config(UART2, USART_RECEIVE_ENABLE);
        usart_transmit_config(UART2, USART_TRANSMIT_ENABLE);

        nvic_irq_enable(UART2_IRQ, 1, 0);
        usart_interrupt_enable(UART2, USART_INT_RBNE);

        serial[2].flags.init_ok = On;
    }

    stream_set_defaults(&stream, baud_rate);

    return &stream;
}

void UART2_IRQHandler (void)
{
    if(RESET != usart_interrupt_flag_get(UART2, USART_INT_FLAG_RBNE)) {
        char data = usart_data_receive(UART2);
        if(!enqueue_realtime_command2((uint8_t)data)) {         // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf2.head, rxbuf2);  // Get and increment buffer pointer
            if(next_head == rxbuf2.tail)                        // If buffer full
                rxbuf2.overflow = 1;                            // flag overflow
            else {
                rxbuf2.data[rxbuf2.head] = (uint8_t)data;       // if not add data to buffer
                rxbuf2.head = next_head;                        // and update pointer
            }
        }
    }

    if(RESET != usart_interrupt_flag_get(UART2, USART_INT_FLAG_TBE)) {
        uint_fast16_t tail = txbuf2.tail;           // Get buffer pointer
        usart_data_transmit(UART2, txbuf2.data[tail]);              // Send next character
        txbuf2.tail = tail = BUFNEXT(tail, txbuf2); // and increment pointer
        if(tail == txbuf2.head)                     // If buffer empty then
            usart_interrupt_disable(UART2, USART_INT_TBE);         // disable UART TX interrupt
   }
}

#endif // SERIAL2_PORT
