/*
  ioports_analog.c - driver code for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2023-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#include "driver.h"

#if defined(AUXOUTPUT0_PWM_PORT) || defined(AUXOUTPUT1_PWM_PORT) || defined(AUXOUTPUT2_PWM_PORT) ||\
     defined(AUXOUTPUT0_ANALOG_PORT) || defined(AUXOUTPUT1_ANALOG_PORT) || defined(AUXOUTPUT2_ANALOG_PORT) ||\
      defined(AUXINPUT0_ANALOG_PORT) || defined(AUXINPUT1_ANALOG_PORT)

#ifdef AUXOUTPUT0_PWM_PORT
#define PWM_OUT0 1
#else
#define PWM_OUT0 0
#endif

#ifdef AUXOUTPUT1_PWM_PORT
#define PWM_OUT1 1
#else
#define PWM_OUT1 0
#endif

#ifdef AUXOUTPUT2_PWM_PORT
#define PWM_OUT2 1
#else
#define PWM_OUT2 0
#endif

#ifdef AUXOUTPUT0_ANALOG_PORT
#define DAC_OUT0 1
#else
#define DAC_OUT0 0
#endif

#ifdef AUXOUTPUT1_ANALOG_PORT
#define DAC_OUT1 1
#else
#define DAC_OUT1 0
#endif

#define AUX_ANALOG_PWM_OUT (PWM_OUT0 + PWM_OUT1 + PWM_OUT2)
#define AUX_ANALOG_DAC_OUT (DAC_OUT0 + DAC_OUT1)
#define AUX_ANALOG_OUT (AUX_ANALOG_PWM_OUT + AUX_ANALOG_DAC_OUT)

#include "pwm.h"

#include "grbl/ioports.h"

typedef struct {
    gpio_port_t port;
    uint8_t pin;
    uint8_t adc_index;  // 0 = ADC0, 1 = ADC1, 2 = ADC2
    uint32_t adc;       // ADC0, ADC1 or ADC2 base address
    uint32_t ch;
} adc_map_t;

static const adc_map_t adc_map[] = {
    { GPIOA,  0, 0, ADC0, ADC_CHANNEL_0 },
    { GPIOA,  1, 0, ADC0, ADC_CHANNEL_1 },
    { GPIOA,  2, 0, ADC0, ADC_CHANNEL_2 },
    { GPIOA,  3, 0, ADC0, ADC_CHANNEL_3 },
    { GPIOA,  4, 1, ADC1, ADC_CHANNEL_4 },
    { GPIOA,  5, 1, ADC1, ADC_CHANNEL_5 },
    { GPIOA,  6, 1, ADC1, ADC_CHANNEL_6 },
    { GPIOA,  7, 1, ADC1, ADC_CHANNEL_7 },
    { GPIOB,  0, 1, ADC1, ADC_CHANNEL_8 },
    { GPIOB,  1, 1, ADC1, ADC_CHANNEL_9 },
    { GPIOC,  0, 0, ADC0, ADC_CHANNEL_10 },
    { GPIOC,  1, 0, ADC0, ADC_CHANNEL_11 },
    { GPIOC,  2, 0, ADC0, ADC_CHANNEL_12 },
    { GPIOC,  3, 0, ADC0, ADC_CHANNEL_13 },
    { GPIOC,  4, 1, ADC1, ADC_CHANNEL_14 },
    { GPIOC,  5, 1, ADC1, ADC_CHANNEL_15 },
    { GPIOF,  3, 2, ADC2, ADC_CHANNEL_9 },
    { GPIOF,  4, 2, ADC2, ADC_CHANNEL_14 },
    { GPIOF,  5, 2, ADC2, ADC_CHANNEL_15 },
    { GPIOF,  6, 2, ADC2, ADC_CHANNEL_4 },
    { GPIOF,  7, 2, ADC2, ADC_CHANNEL_5 },
    { GPIOF,  8, 2, ADC2, ADC_CHANNEL_6 },
    { GPIOF,  9, 2, ADC2, ADC_CHANNEL_7 },
    { GPIOF, 10, 2, ADC2, ADC_CHANNEL_8 }
};

static io_ports_data_t analog;
static input_signal_t *aux_in_analog;
static output_signal_t *aux_out_analog;

#if AUX_ANALOG_DAC_OUT

typedef struct {
    gpio_port_t port;
    uint8_t pin;
    uint32_t dac;
} dac_map_t;

static const dac_map_t dac_map[] = {
    { GPIOA, 4, DAC0 },
    { GPIOA, 5, DAC1 }
};

static const dac_map_t *dac_get_port (gpio_port_t port, uint8_t pin)
{
    const dac_map_t *map = NULL;
    uint_fast8_t idx = sizeof(dac_map) / sizeof(dac_map_t);

    do {
        idx--;
        if(port == dac_map[idx].port && pin == dac_map[idx].pin)
            map = &dac_map[idx];
    } while(idx && map == NULL);

    return map;
}

static float dac_get_value (xbar_t *output)
{
    float value = 1.0f;
    const dac_map_t *port;

    if((port = dac_get_port(aux_out_analog[output->id].port, aux_out_analog[output->id].pin)))
        value = (float)dac_output_value_get(port->dac);

    return value;
}

static void dac_out (uint8_t p, float value)
{
    const dac_map_t *port;

    if((port = dac_get_port(aux_out_analog[p].port, aux_out_analog[p].pin)))
        dac_data_set(port->dac, DAC_ALIGN_12B_R, (uint16_t)value);
}

static bool dac_init (xbar_t *output, pwm_config_t *config, bool persistent)
{
    const dac_map_t *port;

    if((port = dac_get_port(aux_out_analog[output->id].port, aux_out_analog[output->id].pin))) {

        rcu_periph_clock_enable(RCU_DAC);
        rcu_periph_clock_enable(RCU_GPIOA);

        gpio_mode_set((uint32_t)port->port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, 1U << port->pin);

        dac_enable(port->dac);
        dac_data_set(port->dac, DAC_ALIGN_12B_R, 0);
    }

    if(port == NULL && !aux_out_analog[output->id].mode.claimed)
        hal.port.claim(Port_Analog, Port_Output, &output->id, "N/A");

    return !!port;
}

#endif // AUX_ANALOG_DAC_OUT

#if AUX_ANALOG_PWM_OUT

static float pwm_get_value (xbar_t *output)
{
    return output->id < analog.out.n_ports ? aux_out_analog[output->id].pwm->value : -1.0f;
}

static void pwm_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports && aux_out_analog[port].pwm) {

        uint_fast16_t pwm_value = ioports_compute_pwm_value(&aux_out_analog[port].pwm->data, value);
        const pwm_signal_t *pwm = aux_out_analog[port].pwm->port;

        aux_out_analog[port].pwm->value = value;

        if(pwm_value == aux_out_analog[port].pwm->data.off_value) {
            if(aux_out_analog[port].pwm->data.always_on) {
                *pwm->ccr = aux_out_analog[port].pwm->data.off_value;
                if(pwm->timer == timerN(0) || pwm->timer == timerN(7))
                    TIMER_CCHP(pwm->timer) |= TIMER_CCHP_POEN;
                *pwm->ccr = 0;
            } else {
                if(pwm->timer == timerN(0) || pwm->timer == timerN(7))
                    TIMER_CCHP(pwm->timer) |= TIMER_CCHP_POEN;
                *pwm->ccr = 0;
            }
        } else {
            *pwm->ccr = pwm_value;
            if(pwm->timer == timerN(0) || pwm->timer == timerN(7))
                TIMER_CCHP(pwm->timer) |= TIMER_CCHP_POEN;
        }
    }
}

static bool init_pwm (xbar_t *output, pwm_config_t *config, bool persistent)
{
    bool ok;

    if(aux_out_analog[output->id].pwm == NULL) {

        pwm_out_t *pwm;

        if((pwm = calloc(1, sizeof(pwm_out_t)))) {
            if((pwm->port = pwm_claim((uint32_t)output->port, output->pin))) {
                pwm_enable(pwm->port);
                aux_out_analog[output->id].pwm = pwm;
            } else
                free(pwm);
        }
    }

    if((ok = !!aux_out_analog[output->id].pwm)) {

        uint32_t prescaler = 0, clock_hz = pwm_get_clock_hz(aux_out_analog[output->id].pwm->port);

        do {
            prescaler++;
            ok = ioports_precompute_pwm_values(config, &aux_out_analog[output->id].pwm->data, clock_hz / prescaler);
        } while(ok && aux_out_analog[output->id].pwm->data.period > 65530);

        if(ok) {

            pwm_config(aux_out_analog[output->id].pwm->port, prescaler, aux_out_analog[output->id].pwm->data.period, config->invert);

            aux_out_analog[output->id].mode.pwm = !config->servo_mode;
            aux_out_analog[output->id].mode.servo_pwm = config->servo_mode;

            pwm_out(output->id, config->min);
        }
    }

    if(!ok && !aux_out_analog[output->id].mode.claimed)
        hal.port.claim(Port_Analog, Port_Output, &output->id, "N/A");

    return ok;
}

#endif // AUX_ANALOG_PWM_OUT

#if AUX_ANALOG_OUT

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {
#if AUX_ANALOG_DAC_OUT && AUX_ANALOG_PWM_OUT
        if(aux_out_analog[port].mode.pwm || aux_out_analog[port].mode.servo_pwm)
            pwm_out(port, value);
        else
            dac_out(port, value);
#elif AUX_ANALOG_DAC_OUT
        dac_out(port, value);
#else
        pwm_out(port, value);
#endif
    }

    return port < analog.out.n_ports;
}

#endif // AUX_ANALOG_OUT

static uint32_t adc_last_channel[3] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

static inline int adc_index (uint32_t adc)
{
    return adc == ADC0 ? 0 : adc == ADC1 ? 1 : adc == ADC2 ? 2 : -1;
}

static inline int32_t adc_read (uint32_t adc, uint32_t channel)
{
    int32_t value = -1;
    int idx;

    if(adc && (idx = adc_index(adc)) >= 0) {

        if(adc_last_channel[idx] != channel) {
            adc_regular_channel_config(adc, 0, (uint8_t)channel, ADC_SAMPLETIME_3);
            adc_last_channel[idx] = channel;
        }

        adc_software_trigger_enable(adc, ADC_REGULAR_CHANNEL);

        uint32_t timeout = hal_get_tick() + 2;
        while((RESET == adc_flag_get(adc, ADC_FLAG_EOC)) && hal_get_tick() <= timeout);

        if(adc_flag_get(adc, ADC_FLAG_EOC))
            value = (int32_t)adc_regular_data_read(adc);
    }

    return value;
}

static float analog_in_state (xbar_t *input)
{
    return input->id < analog.in.n_ports ? (float)adc_read((uint32_t)aux_in_analog[input->id].adc, aux_in_analog[input->id].channel) : -1.0f;
}

static int32_t wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    return port < analog.in.n_ports ? adc_read((uint32_t)aux_in_analog[port].adc, aux_in_analog[port].channel) : -1;
}

static bool set_function (xbar_t *port, pin_function_t function)
{
    if(port->mode.input)
        aux_in_analog[port->id].id = function;
    else
        aux_out_analog[port->id].id = function;

    return true;
}

static xbar_t *get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;
    xbar_t *info = NULL;

    memset(&pin, 0, sizeof(xbar_t));

    pin.set_function = set_function;

    switch(dir) {

        case Port_Input:
            if(port < analog.in.n_ports) {
                pin.id = port;
                pin.mode = aux_in_analog[port].mode;
                pin.cap = aux_in_analog[port].cap;
                pin.function = aux_in_analog[port].id;
                pin.group = aux_in_analog[port].group;
                pin.pin = aux_in_analog[port].pin;
                pin.port = (void *)aux_in_analog[port].port;
                pin.description = aux_in_analog[port].description;
                pin.get_value = analog_in_state;
                info = &pin;
            }
            break;

        case Port_Output:
#if AUX_ANALOG_OUT
            if(port < analog.out.n_ports) {
                pin.id = port;
                pin.port = (void *)aux_out_analog[port].port;
                pin.mode = aux_out_analog[port].mode;
                pin.mode.pwm &= !pin.mode.servo_pwm;
                XBAR_SET_CAP(pin.cap, pin.mode);
                pin.function = aux_out_analog[port].id;
                pin.group = aux_out_analog[port].group;
                pin.pin = aux_out_analog[port].pin;
                pin.port = (void *)aux_out_analog[port].port;
                pin.description = aux_out_analog[port].description;
#if AUX_ANALOG_DAC_OUT && AUX_ANALOG_PWM_OUT
                pin.get_value = pin.mode.pwm || pin.mode.servo_pwm ? pwm_get_value : dac_get_value;
                pin.config = pin.mode.pwm || pin.mode.servo_pwm ? init_pwm : dac_init;
#elif AUX_ANALOG_DAC_OUT
                pin.get_value = dac_get_value;
                pin.config = dac_init;
#else
                pin.get_value = pwm_get_value;
                pin.config = init_pwm;
#endif
                if(!aux_out_analog[port].mode.pwm)
                    pin.cap.resolution = Resolution_12bit;
                info = &pin;
            }
#endif // AUX_ANALOG_OUT
            break;
    }

    return info;
}

static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Input && port < analog.in.n_ports)
        aux_in_analog[port].description = description;
    else if(port < analog.out.n_ports)
        aux_out_analog[port].description = description;
}

static bool adc_init_peripheral (uint32_t adc)
{
    if(adc == ADC0)
        rcu_periph_clock_enable(RCU_ADC0);
    else if(adc == ADC1)
        rcu_periph_clock_enable(RCU_ADC1);
    else if(adc == ADC2)
        rcu_periph_clock_enable(RCU_ADC2);
    else
        return false;

    adc_clock_config(ADC_ADCCK_PCLK2_DIV4);
    adc_special_function_config(adc, ADC_SCAN_MODE, DISABLE);
    adc_special_function_config(adc, ADC_CONTINUOUS_MODE, DISABLE);
    adc_data_alignment_config(adc, ADC_DATAALIGN_RIGHT);
    adc_resolution_config(adc, ADC_RESOLUTION_12B);
    adc_channel_length_config(adc, ADC_REGULAR_CHANNEL, 1);
    adc_external_trigger_config(adc, ADC_REGULAR_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_enable(adc);

    return true;
}

void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    io_analog_t ports = {
        .ports = &analog,
#if AUX_ANALOG_OUT
        .analog_out = analog_out,
#endif
        .get_pin_info = get_pin_info,
        .wait_on_input = wait_on_input,
        .set_pin_description = set_pin_description
    };

    aux_in_analog = aux_inputs->pins.inputs;
    aux_out_analog = aux_outputs->pins.outputs;

    analog.in.n_ports = aux_inputs->n_pins;
    analog.out.n_ports = aux_outputs->n_pins;

    if(ioports_add_analog(&ports)) {

        if(aux_inputs->n_pins) {

            uint_fast8_t i;
            bool adc_initialized[3] = {false, false, false};

            for(i = 0; i < aux_inputs->n_pins; i++) {

                uint_fast8_t j = sizeof(adc_map) / sizeof(adc_map_t);
                bool found = false;

                do {
                    j--;
                    if(adc_map[j].port == aux_inputs->pins.inputs[i].port && adc_map[j].pin == aux_inputs->pins.inputs[i].pin) {

                        uint32_t adc = adc_map[j].adc;
                        int adc_idx = adc_index(adc);

                        if(adc_idx >= 0 && (!adc_initialized[adc_idx] || adc_init_peripheral(adc))) {

                            adc_initialized[adc_idx] = true;

                            gpio_mode_set((uint32_t)aux_inputs->pins.inputs[i].port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, aux_inputs->pins.inputs[i].bit);

                            aux_inputs->pins.inputs[i].adc = (void *)adc;
                            aux_inputs->pins.inputs[i].channel = adc_map[j].ch;
                            found = true;
                        }
                        break;
                    }
                } while(j);

                if(!found) {
                    aux_inputs->pins.inputs[i].adc = NULL;
                    analog.in.n_ports--;
                }
            }
        }

#if AUX_ANALOG_OUT

        if(analog.out.n_ports) {

            xbar_t *pin;
            uint_fast8_t i;
            pwm_config_t config = {
                .freq_hz = 5000.0f,
                .min = 0.0f,
                .max = 100.0f,
                .off_value = 0.0f,
                .min_value = 0.0f,
                .max_value = 100.0f,
                .invert = Off
            };

            for(i = 0; i < analog.out.n_ports; i++) {
                if((pin = get_pin_info(Port_Output, i)))
                    pin->config(pin, &config, false);
            }
        }

#endif // AUX_ANALOG_OUT
    }
}

#else

void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    (void)aux_inputs;
    (void)aux_outputs;
}

#endif
