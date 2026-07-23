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

// PWM pin table for CNC_ED1 V1.1
// Primary spindle/laser PWM on TIMER0 CH2 (PA10).
static const pwm_signal_t pwm_pin[] = {
    {
        .port = (uint32_t)SPINDLE_PWM_PORT,
        .pin = SPINDLE_PWM_PIN,
        .af = SPINDLE_PWM_AF,
        .timer = timerN(SPINDLE_PWM_TIMER),
        .channel = SPINDLE_PWM_CHANNEL,
        .clock_hz = 84000000
    },
#ifdef LASER_PWM_PIN
    {
        .port = (uint32_t)LASER_PWM_PORT,
        .pin = LASER_PWM_PIN,
        .af = LASER_PWM_AF,
        .timer = timerN(LASER_PWM_TIMER),
        .channel = LASER_PWM_CHANNEL,
        .clock_hz = 84000000
    }
#endif
};

static uint_fast8_t n_claimed = 0;
static const pwm_signal_t *pwm_claimed[2] = {0};

bool pwm_is_available (uint32_t port, uint8_t pin)
{
    const pwm_signal_t *pwm = NULL;
    uint_fast8_t i = sizeof(pwm_pin) / sizeof(pwm_signal_t);

    do {
        i--;
        if(port == pwm_pin[i].port && pin == pwm_pin[i].pin)
            pwm = &pwm_pin[i];
    } while(i && pwm == NULL);

    if(pwm) {
        for(i = 0; i < n_claimed; i++) {
            if(pwm_claimed[i] == pwm)
                return false;
        }
    }

    return pwm != NULL;
}

const pwm_signal_t *pwm_claim (uint32_t port, uint8_t pin)
{
    const pwm_signal_t *pwm = NULL;
    uint_fast8_t i = sizeof(pwm_pin) / sizeof(pwm_signal_t);

    if(!pwm_is_available(port, pin))
        return NULL;

    do {
        i--;
        if(port == pwm_pin[i].port && pin == pwm_pin[i].pin)
            pwm = &pwm_pin[i];
    } while(i && pwm == NULL);

    if(pwm && n_claimed < sizeof(pwm_claimed) / sizeof(pwm_claimed[0]))
        pwm_claimed[n_claimed++] = pwm;

    return pwm;
}

bool pwm_enable (const pwm_signal_t *pwm)
{
    if(pwm == NULL)
        return false;

    rcu_periph_clock_enable(pwm->port == GPIOA ? RCU_GPIOA :
                            pwm->port == GPIOB ? RCU_GPIOB :
                            pwm->port == GPIOC ? RCU_GPIOC :
                            pwm->port == GPIOD ? RCU_GPIOD :
                            pwm->port == GPIOE ? RCU_GPIOE :
                            pwm->port == GPIOF ? RCU_GPIOF :
                            pwm->port == GPIOG ? RCU_GPIOG :
                            pwm->port == GPIOH ? RCU_GPIOH : RCU_GPIOI);
    timerCLKEN(SPINDLE_PWM_TIMER);

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

    timer_disable(timer);

    timer_parameter_struct init;
    timer_struct_para_init(&init);
    init.prescaler = prescaler - 1;
    init.alignedmode = TIMER_COUNTER_EDGE;
    init.counterdirection = TIMER_COUNTER_UP;
    init.period = period - 1;
    init.clockdivision = TIMER_CKDIV_DIV1;
    init.repetitioncounter = 0;
    timer_init(timer, &init);

    timer_oc_parameter_struct oc = {0};
    oc.outputstate  = TIMER_CCX_ENABLE;
    oc.outputnstate = TIMER_CCXN_DISABLE;
    oc.ocpolarity   = inverted ? TIMER_OC_POLARITY_LOW : TIMER_OC_POLARITY_HIGH;
    oc.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    oc.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    oc.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(timer, pwm->channel, &oc);

    timer_channel_output_pulse_value_config(timer, pwm->channel, 0);
    timer_channel_output_mode_config(timer, pwm->channel, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(timer, pwm->channel, TIMER_OC_SHADOW_DISABLE);

    if(timer == timerN(0) || timer == timerN(7))
        timer_primary_output_config(timer, ENABLE);

    timer_auto_reload_shadow_enable(timer);
    timer_enable(timer);

    return true;
}

uint32_t pwm_get_clock_hz (const pwm_signal_t *pwm)
{
    return pwm ? pwm->clock_hz : 0;
}

void pwm_set_value (const pwm_signal_t *pwm, uint_fast16_t value)
{
    if(pwm)
        timer_channel_output_pulse_value_config(pwm->timer, pwm->channel, value);
}
