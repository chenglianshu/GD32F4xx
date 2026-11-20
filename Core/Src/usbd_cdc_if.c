#include "usbd_cdc_if.h"
#include "usb_serial.h"
#include "cdc_acm_core.h"
#include "usb_cdc.h"
#include "grbl/hal.h"
#include "grbl/stream.h"

#if USB_SERIAL_CDC

static usb_class_core cdc_class_hooked;
static volatile uint8_t tx_done = 1U;

uint8_t CDC_Transmit_FS(usb_core_driver *udev, uint8_t *Buf, uint16_t Len)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    // Fail fast until the host has configured the device - like STM32's
    // USBD_CDC_TransmitPacket does while the class is not initialized.
    // Without this, the boot banner (written before enumeration completes)
    // would arm an IN transfer that a following bus reset silently flushes
    // away, leaving tx_done stuck at 0 and the transmitter dead until reboot.
    if(udev->dev.cur_status != USBD_CONFIGURED)
        return USBD_FAIL;

    if(tx_done == 0U)
        return USBD_BUSY;

    tx_done = 0U;
    cdc->packet_sent = 0U;

    usbd_ep_send(udev, CDC_DATA_IN_EP, Buf, Len);

    return USBD_OK;
}

void CDC_Receive_FS(usb_core_driver *udev, uint8_t *Buf, uint32_t Len)
{
    (void)udev;
    usbBufferInput(Buf, Len);
}

static uint8_t cdc_acm_init_hook(usb_dev *udev, uint8_t config_index)
{
    uint8_t status = cdc_class.init(udev, config_index);

    if(status == USBD_OK) {
        tx_done = 1U; // mirror STM32: USBD_CDC_Init resets TxState on every (re)enumeration,
                      // healing any transfer lost to a bus reset
        cdc_acm_data_receive(udev);
    }

    return status;
}

static uint8_t cdc_acm_deinit_hook(usb_dev *udev, uint8_t config_index)
{
    tx_done = 1U;

    return cdc_class.deinit(udev, config_index);
}

static uint8_t cdc_acm_req_hook(usb_dev *udev, usb_req *req)
{
    uint8_t status = cdc_class.req_proc(udev, req);

    if(req->bRequest == SET_CONTROL_LINE_STATE) {
        usb_linestate.pin.dtr = (req->wValue & CDC_ACTIVATE_SIGNAL_DTR) != 0;
        usb_linestate.pin.rts = (req->wValue & CDC_ACTIVATE_CARRIER_SIGNAL_RTS) != 0;
        usb_linestate.timestamp = hal.get_elapsed_ticks();
        // Resends the welcome banner ~200 ms after DTR and forwards the
        // linestate event - usb_serial.c advertises this via
        // state.linestate_event = On, so it must actually be delivered.
        stream_usb_linestate_changed(0, usb_linestate.pin);
    }

    return status;
}

static uint8_t cdc_acm_in_hook(usb_dev *udev, uint8_t ep_num)
{
    uint8_t status = cdc_class.data_in(udev, ep_num);

    tx_done = 1U;

    return status;
}

static uint8_t cdc_acm_out_hook(usb_dev *udev, uint8_t ep_num)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    uint8_t status = cdc_class.data_out(udev, ep_num);

    if(cdc->receive_length != 0U)
        CDC_Receive_FS((usb_core_driver *)udev, (uint8_t *)(cdc->data), cdc->receive_length);

    cdc->receive_length = 0U;
    cdc_acm_data_receive(udev);

    return status;
}

usb_class_core *usbd_cdc_if_class(void)
{
    if(cdc_class_hooked.init == NULL) {
        cdc_class_hooked = cdc_class;
        cdc_class_hooked.init     = cdc_acm_init_hook;
        cdc_class_hooked.deinit   = cdc_acm_deinit_hook;
        cdc_class_hooked.req_proc = cdc_acm_req_hook;
        cdc_class_hooked.data_in  = cdc_acm_in_hook;
        cdc_class_hooked.data_out = cdc_acm_out_hook;
    }

    return &cdc_class_hooked;
}

#endif // USB_SERIAL_CDC
