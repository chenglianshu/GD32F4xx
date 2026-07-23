#pragma once

#include <stdint.h>
#include "usbd_core.h"

uint8_t CDC_Transmit_FS(usb_core_driver *udev, uint8_t *Buf, uint16_t Len);
void CDC_Receive_FS(usb_core_driver *udev, uint8_t *Buf, uint32_t Len);

usb_class_core *usbd_cdc_if_class(void);
