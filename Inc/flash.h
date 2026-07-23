// Inc/flash.h
// Flash driver for GD32F4xx grblHAL port.

#pragma once

#include <stdbool.h>
#include <stdint.h>

bool memcpy_from_flash (uint8_t *dest);
bool memcpy_to_flash (uint8_t *source);
