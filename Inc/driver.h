// Inc/driver.h
// Driver code for GD32F4xx grblHAL port.
// Structure and naming aligned with STM32F4xx reference driver.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "my_machine.h"
#include "main.h"
#include "pwm.h"

#if defined(BOARD_CNC_ED1_V20)
  #include "boards/cnc_ed1_v20_map.h"
#else
  #error "Board not defined"
#endif

#if defined(_WIZCHIP_) && _WIZCHIP_ > 0
#undef ETHERNET_ENABLE
#define ETHERNET_ENABLE 1
#endif

#define OPTS_POSTPROCESSING

#include "grbl/driver_opts.h"

#include "timers.h"

#include "grbl/driver_opts2.h"

// -----------------------------------------------------------------------------
// HAL type mappings
// -----------------------------------------------------------------------------
typedef uint32_t gpio_port_t;

// -----------------------------------------------------------------------------
// GPIO bit operation macros
// -----------------------------------------------------------------------------
#define GPIO_GET_INDEX(port) \
    ((port) == GPIOA ? 0U : \
     (port) == GPIOB ? 1U : \
     (port) == GPIOC ? 2U : \
     (port) == GPIOD ? 3U : \
     (port) == GPIOE ? 4U : \
     (port) == GPIOF ? 5U : \
     (port) == GPIOG ? 6U : \
     (port) == GPIOH ? 7U : \
     (port) == GPIOI ? 8U : 0U)

// GD32 SPL gpio_xxx functions expect a bitmask (GPIO_PIN_x = BIT(x)),
// while grblHAL board maps use raw pin numbers. Convert here.
// Use direct GD32 register access (GPIO_ISTAT / GPIO_BOP / GPIO_BC) for speed.
#define DIGITAL_IN(gpio_periph, pin) \
    (!!(GPIO_ISTAT(gpio_periph) & BIT(pin)))

#define DIGITAL_OUT(gpio_periph, pin, on) \
    do { \
        if (on) \
            GPIO_BOP(gpio_periph) = (uint32_t)BIT(pin); \
        else \
            GPIO_BC(gpio_periph) = (uint32_t)BIT(pin); \
    } while(0)

// -----------------------------------------------------------------------------
// Timer token macros (match STM32 naming style)
// -----------------------------------------------------------------------------
#define _timer(t)       TIMER##t
#define timer(t)        _timer(t)
#define _timerN(t)      TIMER##t
#define timerN(t)       _timerN(t)
#define _timerBase(t)   TIMER##t
#define timerBase(t)    _timerBase(t)
#define _timerINT(t)    TIMER##t##_IRQn
#define timerINT(t)     _timerINT(t)
#define _timerHANDLER(t) TIMER##t##_IRQHandler
#define timerHANDLER(t) _timerHANDLER(t)
#define _timerCLKEN(t)  rcu_periph_clock_enable(RCU_TIMER##t)
#define timerCLKEN(t)   _timerCLKEN(t)
#define _timerCH(c)     TIMER_CH_##c
#define timerCH(c)      _timerCH(c)

// -----------------------------------------------------------------------------
// UART token macros (match STM32 naming style)
// -----------------------------------------------------------------------------
#define _usart(t)       USART##t
#define usart(t)        _usart(t)
#define _usartBase(t)   USART##t
#define usartBase(t)    _usartBase(t)
#define _usartINT(t)    USART##t##_IRQn
#define usartINT(t)     _usartINT(t)
#define _usartHANDLER(t) USART##t##_IRQHandler
#define usartHANDLER(t) _usartHANDLER(t)
#define _usartCLKEN(t)  rcu_periph_clock_enable(RCU_USART##t)
#define usartCLKEN(t)   _usartCLKEN(t)

// -----------------------------------------------------------------------------
// Pin mode defaults (match STM32 driver.h)
// -----------------------------------------------------------------------------
#ifndef STEP_PINMODE
#define STEP_PINMODE PINMODE_OUTPUT
#endif

#ifndef DIRECTION_PINMODE
#define DIRECTION_PINMODE PINMODE_OUTPUT
#endif

#ifndef STEPPERS_ENABLE_PINMODE
#define STEPPERS_ENABLE_PINMODE PINMODE_OUTPUT
#endif

#ifndef STEP_PULSE_TOFF_MIN
#define STEP_PULSE_TOFF_MIN 2.0f
#endif

#ifndef STEP_PULSE_TON_LATENCY
#define STEP_PULSE_TON_LATENCY 0.95f
#endif

#ifndef STEP_PULSE_TOFF_LATENCY
#define STEP_PULSE_TOFF_LATENCY 0.85f
#endif

// -----------------------------------------------------------------------------
// Signal structures (match STM32 field names/order, GD32 types)
// -----------------------------------------------------------------------------
typedef struct {
    pin_function_t id;
    pin_cap_t cap;
    pin_mode_t mode;
    uint8_t pin;
    uint32_t bit;
    gpio_port_t port;
    pin_group_t group;
    uint8_t user_port;
    volatile bool active;
    ioport_interrupt_callback_ptr interrupt_callback;
    void *adc;
    uint32_t channel; // ADC channel
    const char *description;
} input_signal_t;

typedef struct {
    float value;
    ioports_pwm_t data;
    const pwm_signal_t *port;
} pwm_out_t;

typedef struct {
    pin_function_t id;
    pin_cap_t cap;
    pin_mode_t mode;
    uint8_t pin;
    uint32_t bit;
    gpio_port_t port;
    pin_group_t group;
    pwm_out_t *pwm;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

// -----------------------------------------------------------------------------
// Public driver API
// -----------------------------------------------------------------------------
bool driver_init (void);
void Driver_IncTick (void);
void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode);

void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);

uint32_t hal_get_tick(void);
void delay_ms(uint32_t ms);

void stepper_timer_init(void);
void stepper_timer_load(uint32_t ticks);

// Steppers use TIMER3 on the CNC_ED1 V1.1 board.
#define STEPPER_TIMER_N     3
#define STEPPER_TIMER       timerBase(STEPPER_TIMER_N)

// Spindle PWM uses TIMER0 CH2 (PA10).
#define SPINDLE_PWM_TIMER_N     SPINDLE_PWM_TIMER
#define SPINDLE_PWM_TIMER_BASE  timerBase(SPINDLE_PWM_TIMER_N)

#if SPINDLE_ENCODER_ENABLE

#ifndef RPM_COUNTER_N
#define RPM_COUNTER_N   3
#endif
#ifndef RPM_TIMER_N
#define RPM_TIMER_N     2
#endif

#define RPM_COUNTER                 timer(RPM_COUNTER_N)
#define RPM_COUNTER_BASE            timerBase(RPM_COUNTER_N)
#define RPM_COUNTER_CLKEN           timerCLKEN(RPM_COUNTER_N)
#define RPM_COUNTER_IRQn            timerINT(RPM_COUNTER_N)
#define RPM_COUNTER_IRQHandler      timerHANDLER(RPM_COUNTER_N)

#define RPM_TIMER                   timer(RPM_TIMER_N)
#define RPM_TIMER_BASE              timerBase(RPM_TIMER_N)
#define RPM_TIMER_CLKEN             timerCLKEN(RPM_TIMER_N)
#define RPM_TIMER_IRQn              timerINT(RPM_TIMER_N)
#define RPM_TIMER_IRQHandler        timerHANDLER(RPM_TIMER_N)

#endif // SPINDLE_ENCODER_ENABLE

// Primary serial port.
#define SERIAL_PORT_N       SERIAL_PORT
#define SERIAL_USART        usartBase(SERIAL_PORT_N)
