// Src/serial.c
// GD32F4xx serial stream driver for grblHAL.
// Adapted from the GD32F30x reference pattern.

#include <string.h>

#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "driver.h"

#define USART_IRQHandler    usartHANDLER(SERIAL_PORT)

/* Primary serial port from board map */
#define USART               usart(SERIAL_PORT)
#define USART_IRQn          usartINT(SERIAL_PORT)
#define UART_TX_RX_GPIO     UART0_TX_PORT
#define UART_TX_GPIO_PIN    BIT(UART0_TX_PIN)
#define UART_RX_GPIO_PIN    BIT(UART0_RX_PIN)
#define UART_GPIO_AF        UART0_GPIO_AF

static stream_rx_buffer_t rxbuf = {0};
static stream_tx_buffer_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static io_stream_properties_t serial[] = {
    {
        .type = StreamType_Serial,
        .instance = 0,
        .flags.claimable = On,
        .flags.claimed = Off,
        .flags.can_set_baud = Off,
        .claim = serialInit
    }
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    static const periph_pin_t tx0 = {
        .function = Output_TX,
        .group = PinGroup_UART1,
        .port  = (void *)UART_TX_RX_GPIO,
        .pin   = UART0_TX_PIN,
        .mode  = { .mask = PINMODE_OUTPUT },
        .description = "UART1"
    };

    static const periph_pin_t rx0 = {
        .function = Input_RX,
        .group = PinGroup_UART1,
        .port  = (void *)UART_TX_RX_GPIO,
        .pin   = UART0_RX_PIN,
        .mode  = { .mask = PINMODE_NONE },
        .description = "UART1"
    };

    hal.periph_port.register_pin(&rx0);
    hal.periph_port.register_pin(&tx0);

    stream_register_streams(&streams);
}

// Returns number of free characters in serial input buffer
static uint16_t serialRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

// Flushes the serial input buffer
static void serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

// Flushes and adds a CAN character to the serial input buffer
static void serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

// Writes a character to the serial output stream
static bool serialPutC (const uint8_t c)
{
    uint16_t next_head = BUFNEXT(txbuf.head, txbuf);  // Get pointer to next free slot in buffer

    while(txbuf.tail == next_head) {                  // While TX buffer full
        if(!hal.stream_blocking_callback())           // check if blocking for space,
            return false;                             // exit if not (leaves TX buffer in an inconsistent state)
    }

    txbuf.data[txbuf.head] = c;                       // Add data to buffer,
    txbuf.head = next_head;                           // update head pointer and
    usart_interrupt_enable(USART, USART_INT_TBE);     // enable TX interrupts
    return true;
}

// Writes a null terminated string to the serial output stream, blocks if buffer full
static void serialWriteS (const char *s)
{
    char c, *ptr = (char *)s;
    while((c = *ptr++) != '\0')
        serialPutC(c);
}

// serialGetC - returns -1 if no data available
static int32_t serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer
    if(tail == rxbuf.head)
        return -1; // no data available
    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer
    return (int32_t)data;
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
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

const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .is_connected = stream_connected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed || baud_rate != 115200)
        return NULL;

    serial[0].flags.claimed = On;

    /* RCU Config */
    rcu_periph_clock_enable(RCU_GPIOB);
    usartCLKEN(SERIAL_PORT);

    /* Connect port to USARTx_Tx and USARTx_Rx (AF7 remap) */
    gpio_af_set(UART_TX_RX_GPIO, UART_GPIO_AF, UART_TX_GPIO_PIN | UART_RX_GPIO_PIN);
    gpio_mode_set(UART_TX_RX_GPIO, GPIO_MODE_AF, GPIO_PUPD_PULLUP, UART_TX_GPIO_PIN | UART_RX_GPIO_PIN);
    gpio_output_options_set(UART_TX_RX_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, UART_TX_GPIO_PIN);

    /* USART configure 115200 8N1 */
    usart_deinit(USART);
    usart_word_length_set(USART, USART_WL_8BIT);
    usart_stop_bit_set(USART, USART_STB_1BIT);
    usart_parity_config(USART, USART_PM_NONE);
    usart_baudrate_set(USART, baud_rate);
    usart_hardware_flow_rts_config(USART, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART, USART_CTS_DISABLE);
    usart_receive_config(USART, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART, USART_TRANSMIT_ENABLE);
    usart_enable(USART);

    nvic_irq_enable(USART_IRQn, 1, 0);
    usart_interrupt_enable(USART, USART_INT_RBNE);

    return &stream;
}

void USART_IRQHandler (void)
{
    if(RESET != usart_interrupt_flag_get(USART, USART_INT_FLAG_RBNE)) {
        char data = usart_data_receive(USART);
        if(!enqueue_realtime_command(data)) {                   // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer
            if(next_head == rxbuf.tail)                         // If buffer full
                rxbuf.overflow = 1;                             // flag overflow
            else {
                rxbuf.data[rxbuf.head] = data;                  // if not add data to buffer
                rxbuf.head = next_head;                         // and update pointer
            }
        }
    }

    if((RESET != usart_interrupt_flag_get(USART, USART_INT_FLAG_TBE)) && (RESET != usart_flag_get(USART, USART_FLAG_TBE))) {
        uint_fast16_t tail = txbuf.tail;                    // Get buffer pointer
        usart_data_transmit(USART, txbuf.data[tail]);       // Send next character
        txbuf.tail = tail = BUFNEXT(tail, txbuf);           // and increment pointer
        if(tail == txbuf.head)                              // If buffer empty then
            usart_interrupt_disable(USART, USART_INT_TBE);  // disable UART TX interrupt
    }
}
