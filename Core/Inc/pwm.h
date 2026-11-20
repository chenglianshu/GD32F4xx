/*

  pwm.h - driver code for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2024 Terje Io

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

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t pin;
    uint8_t af;
    uint32_t port;
    uint32_t timer;
    volatile uint32_t *ccr;
    volatile uint32_t *ccmr;
    uint32_t ois;
    uint32_t ocm;
    uint32_t ocmc;
    uint32_t en;
    uint32_t pol;
} pwm_signal_t;

const pwm_signal_t *pwm_claim (uint32_t port, uint8_t pin);
bool pwm_enable (const pwm_signal_t *pwm);
bool pwm_config (const pwm_signal_t *pwm, uint32_t prescaler, uint32_t period, bool inverted);
bool pwm_is_available (uint32_t port, uint8_t pin);
uint32_t pwm_get_clock_hz (const pwm_signal_t *pwm);
