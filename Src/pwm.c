/*
  pwm.c - driver code for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#include "driver.h"

#include <string.h>

// Helper macros to generate PWM signal table entries for GD32F4xx.
// Channel mapping: STM32 CC1/CC2/CC3/CC4 -> GD32 CH0/CH1/CH2/CH3.
#define _pwm_ccr(t, c)        &TIMER_CH##c##CV(t)
#define _pwm_ccmr_ch01(t)     &TIMER_CHCTL0(t)
#define _pwm_ccmr_ch23(t)     &TIMER_CHCTL1(t)
#define _pwm_ois_ch0          TIMER_CTL1_ISO0
#define _pwm_ois_ch1          TIMER_CTL1_ISO1
#define _pwm_ois_ch2          TIMER_CTL1_ISO2
#define _pwm_ois_ch3          TIMER_CTL1_ISO3
#define _pwm_en_ch0           TIMER_CHCTL2_CH0EN
#define _pwm_en_ch1           TIMER_CHCTL2_CH1EN
#define _pwm_en_ch2           TIMER_CHCTL2_CH2EN
#define _pwm_en_ch3           TIMER_CHCTL2_CH3EN
#define _pwm_pol_ch0          TIMER_CHCTL2_CH0P
#define _pwm_pol_ch1          TIMER_CHCTL2_CH1P
#define _pwm_pol_ch2          TIMER_CHCTL2_CH2P
#define _pwm_pol_ch3          TIMER_CHCTL2_CH3P
#define _pwm_ocm_ch0          TIMER_OC_MODE_PWM0
#define _pwm_ocm_ch1          (TIMER_OC_MODE_PWM0 << 8)
#define _pwm_ocm_ch2          TIMER_OC_MODE_PWM0
#define _pwm_ocm_ch3          (TIMER_OC_MODE_PWM0 << 8)
#define _pwm_ocmc_ch0         (TIMER_CHCTL0_CH0COMCTL | TIMER_CHCTL0_CH0MS)
#define _pwm_ocmc_ch1         (TIMER_CHCTL0_CH1COMCTL | TIMER_CHCTL0_CH1MS)
#define _pwm_ocmc_ch2         (TIMER_CHCTL1_CH2COMCTL | TIMER_CHCTL1_CH2MS)
#define _pwm_ocmc_ch3         (TIMER_CHCTL1_CH3COMCTL | TIMER_CHCTL1_CH3MS)

#define PWM_ENTRY(p, n, a, t, ch) { \
    .port = (uint32_t)(p), \
    .pin = (n), \
    .af = (a), \
    .timer = timerN(t), \
    .ccr = _pwm_ccr(timerN(t), ch), \
    .ccmr = (ch < 2 ? _pwm_ccmr_ch01(timerN(t)) : _pwm_ccmr_ch23(timerN(t))), \
    .ois = _pwm_ois_ch##ch, \
    .ocm = _pwm_ocm_ch##ch, \
    .ocmc = _pwm_ocmc_ch##ch, \
    .en = _pwm_en_ch##ch, \
    .pol = _pwm_pol_ch##ch \
}

// PWM output pin table for GD32F4xx.
// Covers the most common timer PWM pins on GD32F4xx devices.
// Timer numbering: TIMER0..TIMER7 map to STM32 TIM1..TIM8.
static const pwm_signal_t pwm_pin[] = {
#if !IS_TIMER_CLAIMED(TIMER0_BASE)
    // TIMER0 (advanced timer, AF1)
    PWM_ENTRY(GPIOA, 8,  GPIO_AF_1, 0, 0),
    PWM_ENTRY(GPIOA, 9,  GPIO_AF_1, 0, 1),
    PWM_ENTRY(GPIOA, 10, GPIO_AF_1, 0, 2),
    PWM_ENTRY(GPIOA, 11, GPIO_AF_1, 0, 3),
#endif
#if !IS_TIMER_CLAIMED(TIMER1_BASE)
    // TIMER1 (general-purpose timer, AF1)
    PWM_ENTRY(GPIOA, 0,  GPIO_AF_1, 1, 0),
    PWM_ENTRY(GPIOA, 1,  GPIO_AF_1, 1, 1),
    PWM_ENTRY(GPIOA, 2,  GPIO_AF_1, 1, 2),
    PWM_ENTRY(GPIOA, 3,  GPIO_AF_1, 1, 3),
    PWM_ENTRY(GPIOA, 5,  GPIO_AF_1, 1, 0),
    PWM_ENTRY(GPIOA, 15, GPIO_AF_1, 1, 0),
    PWM_ENTRY(GPIOB, 3,  GPIO_AF_1, 1, 0),
    PWM_ENTRY(GPIOB, 10, GPIO_AF_1, 1, 2),
    PWM_ENTRY(GPIOB, 11, GPIO_AF_1, 1, 3),
#endif
#if !IS_TIMER_CLAIMED(TIMER2_BASE)
    // TIMER2 (general-purpose timer, AF2)
    PWM_ENTRY(GPIOA, 6,  GPIO_AF_2, 2, 0),
    PWM_ENTRY(GPIOA, 7,  GPIO_AF_2, 2, 1),
    PWM_ENTRY(GPIOB, 0,  GPIO_AF_2, 2, 2),
    PWM_ENTRY(GPIOB, 1,  GPIO_AF_2, 2, 3),
    PWM_ENTRY(GPIOB, 4,  GPIO_AF_2, 2, 0),
    PWM_ENTRY(GPIOB, 5,  GPIO_AF_2, 2, 1),
    PWM_ENTRY(GPIOC, 6,  GPIO_AF_2, 2, 0),
    PWM_ENTRY(GPIOC, 7,  GPIO_AF_2, 2, 1),
    PWM_ENTRY(GPIOC, 8,  GPIO_AF_2, 2, 2),
    PWM_ENTRY(GPIOC, 9,  GPIO_AF_2, 2, 3),
#endif
#if !IS_TIMER_CLAIMED(TIMER3_BASE)
    // TIMER3 (general-purpose timer, AF2)
    PWM_ENTRY(GPIOB, 6,  GPIO_AF_2, 3, 0),
    PWM_ENTRY(GPIOB, 7,  GPIO_AF_2, 3, 1),
    PWM_ENTRY(GPIOB, 8,  GPIO_AF_2, 3, 2),
    PWM_ENTRY(GPIOB, 9,  GPIO_AF_2, 3, 3),
    PWM_ENTRY(GPIOD, 12, GPIO_AF_2, 3, 0),
    PWM_ENTRY(GPIOD, 13, GPIO_AF_2, 3, 1),
    PWM_ENTRY(GPIOD, 14, GPIO_AF_2, 3, 2),
    PWM_ENTRY(GPIOD, 15, GPIO_AF_2, 3, 3),
#endif
#if !IS_TIMER_CLAIMED(TIMER7_BASE)
    // TIMER7 (advanced timer, AF3)
    PWM_ENTRY(GPIOC, 6,  GPIO_AF_3, 7, 0),
    PWM_ENTRY(GPIOC, 7,  GPIO_AF_3, 7, 1),
    PWM_ENTRY(GPIOC, 8,  GPIO_AF_3, 7, 2),
    PWM_ENTRY(GPIOC, 9,  GPIO_AF_3, 7, 3),
    PWM_ENTRY(GPIOA, 0,  GPIO_AF_3, 7, 0),
    PWM_ENTRY(GPIOA, 7,  GPIO_AF_3, 7, 1),
    PWM_ENTRY(GPIOB, 6,  GPIO_AF_3, 7, 0),
    PWM_ENTRY(GPIOB, 7,  GPIO_AF_3, 7, 1),
    PWM_ENTRY(GPIOB, 8,  GPIO_AF_3, 7, 2),
    PWM_ENTRY(GPIOB, 9,  GPIO_AF_3, 7, 3),
    PWM_ENTRY(GPIOE, 5,  GPIO_AF_3, 7, 0),
    PWM_ENTRY(GPIOE, 6,  GPIO_AF_3, 7, 1),
#endif
};

typedef struct {
    uint32_t timer;
    volatile uint32_t *ccr;
} pwm_claimed_t;

static uint_fast8_t n_claimed = 0;
static pwm_claimed_t pwm_claimed[8] = {0};

bool pwm_is_available (uint32_t port, uint8_t pin)
{
    const pwm_signal_t *pwm = NULL;
    uint_fast8_t i = sizeof(pwm_pin) / sizeof(pwm_signal_t);

    do {
        i--;
        if(port == pwm_pin[i].port && pin == pwm_pin[i].pin)
            pwm = &pwm_pin[i];
    } while(i && pwm == NULL);

    if(pwm && (i = n_claimed)) do {
        i--;
        if(pwm->timer == pwm_claimed[i].timer)
            return pwm->ccr != pwm_claimed[i].ccr;
    } while(i);

    return pwm && !timer_is_claimed(pwm->timer);
}

const pwm_signal_t *pwm_claim (uint32_t port, uint8_t pin)
{
    const pwm_signal_t *pwm = NULL;

    if(pwm_is_available(port, pin)) {

        uint_fast8_t i = sizeof(pwm_pin) / sizeof(pwm_signal_t);

        do {
            i--;
            if(port == pwm_pin[i].port && pin == pwm_pin[i].pin)
                pwm = &pwm_pin[i];
        } while(i && pwm == NULL);

        if(pwm) {

            uint32_t timer = 0;

            if((i = n_claimed)) do {
                if(pwm->timer == pwm_claimed[--i].timer)
                    timer = pwm_claimed[i].timer;
            } while(i && timer == 0);

            if(timer || timer_claim(pwm->timer)) {
                pwm_claimed[n_claimed].timer = pwm->timer;
                pwm_claimed[n_claimed++].ccr = pwm->ccr;
            } else
                pwm = NULL;
        }
    }

    return pwm;
}

bool pwm_enable (const pwm_signal_t *pwm)
{
    timer_clk_enable(pwm->timer);

    gpio_af_set((uint32_t)pwm->port, pwm->af, 1U << pwm->pin);
    gpio_mode_set((uint32_t)pwm->port, GPIO_MODE_AF, GPIO_PUPD_NONE, 1U << pwm->pin);
    gpio_output_options_set((uint32_t)pwm->port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, 1U << pwm->pin);

    return true;
}

bool pwm_config (const pwm_signal_t *pwm, uint32_t prescaler, uint32_t period, bool inverted)
{
    if(pwm == NULL)
        return false;

    uint32_t timer = pwm->timer;

    TIMER_CTL0(timer) &= ~TIMER_CTL0_CEN;

    timer_parameter_struct init;
    timer_struct_para_init(&init);
    init.prescaler = prescaler - 1;
    init.alignedmode = TIMER_COUNTER_EDGE;
    init.counterdirection = TIMER_COUNTER_UP;
    init.period = period - 1;
    init.clockdivision = TIMER_CKDIV_DIV1;
    init.repetitioncounter = 0;
    timer_init(timer, &init);

    *pwm->ccmr &= ~pwm->ocmc;
    *pwm->ccmr |= pwm->ocm;
    *pwm->ccr = 0;

    if(timer == timerN(0) || timer == timerN(7))
        TIMER_CCHP(timer) |= TIMER_CCHP_ROS | TIMER_CCHP_IOS;

    TIMER_CHCTL2(timer) &= ~pwm->en;

    if(inverted) {
        TIMER_CHCTL2(timer) |= pwm->pol;
        TIMER_CTL1(timer) |= pwm->ois;
    } else {
        TIMER_CHCTL2(timer) &= ~pwm->pol;
        TIMER_CTL1(timer) &= ~pwm->ois;
    }
    TIMER_CHCTL2(timer) |= pwm->en;

    TIMER_CTL0(timer) |= TIMER_CTL0_CEN;

    return true;
}

uint32_t pwm_get_clock_hz (const pwm_signal_t *pwm)
{
    return timer_get_clock_hz(pwm->timer);
}
