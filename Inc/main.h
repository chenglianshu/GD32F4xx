// Inc/main.h
#pragma once

#include "gd32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern uint32_t SystemCoreClock;

// Compatibility alias for STM32-derived code / grblHAL macros
#define HSE_VALUE  HXTAL_VALUE

// NVIC vector table base addresses (from V1.1 firmware)
#define NVIC_VectTab_RAM    ((uint32_t)0x20000000)
#define NVIC_VectTab_FLASH  ((uint32_t)0x08000000)

// System tick and delay helpers (implemented in main.c)
uint32_t hal_get_tick(void);
void delay_ms(uint32_t ms);
void delay_1ms(uint32_t count);
void HAL_Delay(uint32_t Delay);