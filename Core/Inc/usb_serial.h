#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver.h"

typedef struct {
    serial_linestate_t pin;
    uint32_t timestamp;
} usb_linestate_t;

extern volatile usb_linestate_t usb_linestate;

const io_stream_t *usbInit(void);
void usbBufferInput(uint8_t *data, uint32_t length);
