// Inc/timers.h
// Timer driver code for GD32F4xx grblHAL port.

#pragma once

#include "main.h"
#include "grbl/hal.h"

/* Internal API */

hal_timer_t timer_claim (uint32_t timer);
bool timer_is_claimed (uint32_t timer);
uint32_t timer_clk_enable (uint32_t timer);
uint32_t timer_get_clock_hz (uint32_t timer);
timer_cap_t timer_get_cap (uint32_t timer);
timer_resolution_t timer_get_resolution (uint32_t timer);

/* HAL API */

hal_timer_t timerClaim (timer_cap_t cap, uint32_t timebase);
bool timerCfg (hal_timer_t timer, timer_cfg_t *cfg);
bool timerStart (hal_timer_t timer, uint32_t period);
bool timerStop (hal_timer_t timer);

/* Stepper-specific helpers */

void stepper_timer_init (void);
void stepper_timer_load (uint32_t ticks);
void stepper_timer_isr (void);
