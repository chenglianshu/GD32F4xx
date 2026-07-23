// Inc/serial.h
// Low level functions for transmitting bytes via the serial port.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "grbl/stream.h"

#define RX_BUFFER_HWM 900
#define RX_BUFFER_LWM 300

const io_stream_t *serialInit (uint32_t baud_rate);
void serialRegisterStreams (void);
