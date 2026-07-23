#include "driver.h"

#if USB_SERIAL_CDC

#include "usb_serial.h"
#include "usbd_cdc_if.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "cdc_acm_core.h"
#include "usbd_core.h"
#include "drv_usbd_int.h"

static stream_rx_buffer_t rxbuf = {0};
static stream_block_tx_buffer2_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

usb_core_driver usbd_cdc;

volatile usb_linestate_t usb_linestate = {0};

static bool is_connected(void)
{
    return usb_linestate.pin.dtr && (hal.get_elapsed_ticks() - usb_linestate.timestamp) >= 15;
}

static uint16_t usbRxFree(void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;
    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static void usbRxFlush(void)
{
    rxbuf.tail = rxbuf.head;
}

static void usbRxCancel(void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

static inline bool usb_write(void)
{
    static uint8_t dummy = 0;

    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;

    while(CDC_Transmit_FS(&usbd_cdc, (uint8_t *)txbuf.s, txbuf.length) == USBD_BUSY) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    if((txbuf.length % 64U) == 0U) {
        while(CDC_Transmit_FS(&usbd_cdc, &dummy, 0U) == USBD_BUSY) {
            if(!hal.stream_blocking_callback())
                return false;
        }
    }

    txbuf.use_tx2data = !txbuf.use_tx2data;
    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;
    txbuf.length = 0;

    return true;
}

static bool usbPutC(const uint8_t c)
{
    static uint8_t buf[1];
    *buf = c;

    while(CDC_Transmit_FS(&usbd_cdc, buf, 1U) == USBD_BUSY) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    return true;
}

static void usbWriteS(const char *s)
{
    size_t length = strlen(s);

    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!usb_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!usb_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        if(s[length - 1] == ASCII_LF)
            usb_write();
    }
}

static void usbWrite(const uint8_t *s, uint16_t length)
{
    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!usb_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!usb_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        usb_write();
    }
}

static int32_t usbGetC(void)
{
    uint_fast16_t tail = rxbuf.tail;

    if(tail == rxbuf.head)
        return -1;

    char data = rxbuf.data[tail];
    rxbuf.tail = BUFNEXT(tail, rxbuf);

    return (int32_t)data;
}

static bool usbSuspendInput(bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool usbEnqueueRtCommand(uint8_t c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr usbSetRtHandler(enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

static void usb_gpio_init(void)
{
    /* force USB re-enumeration: pull DP low then release */
    rcu_periph_clock_enable(RCU_GPIOA);

    gpio_mode_set(USB_DP_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(USB_DP_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_12);
    gpio_bit_reset(USB_DP_PORT, GPIO_PIN_12);

    for(volatile uint32_t i = 0; i < 0xFFFFU; i++)
        __NOP();

    /* configure PA11/PA12 as USB OTG FS alternate function */
    gpio_mode_set(USB_DM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(USB_DM_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    gpio_mode_set(USB_DP_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(USB_DP_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    gpio_af_set(USB_DM_PORT, GPIO_AF_10, GPIO_PIN_11);
    gpio_af_set(USB_DP_PORT, GPIO_AF_10, GPIO_PIN_12);
}

static void usb_clock_init(void)
{
    rcu_periph_clock_enable(RCU_USBFS);
}

const io_stream_t *usbInit(void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.is_usb = On,
        .state.linestate_event = On,
        .is_connected = is_connected,
        .read = usbGetC,
        .write = usbWriteS,
        .write_char = usbPutC,
        .write_n = usbWrite,
        .enqueue_rt_command = usbEnqueueRtCommand,
        .get_rx_buffer_free = usbRxFree,
        .reset_read_buffer = usbRxFlush,
        .cancel_read_buffer = usbRxCancel,
        .suspend_read = usbSuspendInput,
        .set_enqueue_rt_handler = usbSetRtHandler
    };

    usb_gpio_init();
    usb_clock_init();

    usbd_init(&usbd_cdc, USB_CORE_ENUM_FS, &cdc_desc, usbd_cdc_if_class());

    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(USBFS_IRQn, 3U, 0U);

    usbd_connect(&usbd_cdc);

    txbuf.s = txbuf.data;
    txbuf.max_length = BLOCK_TX_BUFFER_SIZE;

    return &stream;
}

void usbBufferInput(uint8_t *data, uint32_t length)
{
    while(length--) {
        if(!enqueue_realtime_command(*data)) {
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);
            if(next_head == rxbuf.tail)
                rxbuf.overflow = 1;
            else {
                rxbuf.data[rxbuf.head] = *data;
                rxbuf.head = next_head;
            }
        }
        data++;
    }
}

void usb_udelay(const uint32_t usec)
{
    uint64_t start = hal.get_micros();
    while(hal.get_micros() - start < (uint64_t)usec);
}

void usb_mdelay(const uint32_t msec)
{
    hal.delay_ms(msec, NULL);
}

void USBFS_IRQHandler(void)
{
    usbd_isr(&usbd_cdc);
}

#endif // USB_SERIAL_CDC
