/*

  driver.h - driver code for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2026 Terje Io

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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "main.h"
#include "pwm.h"

#if defined(_WIZCHIP_) && _WIZCHIP_ > 0
#undef ETHERNET_ENABLE
#define ETHERNET_ENABLE 1
#endif

#define OPTS_POSTPROCESSING

#include "grbl/driver_opts.h"

#include "timers.h"

// ---------------------------------------------------------------------------
// GPIO abstraction (GD32 std peripheral library).
// Board maps use GPIOx base addresses and raw pin numbers.
// ---------------------------------------------------------------------------
typedef uint32_t gpio_port_t;

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

// Bit-band addressing, STM32 style: single atomic read/write to the peripheral
// bit-band alias word instead of BOP/BC register writes. This is the variant
// that was found (as an uncommitted local change to the previous port) to be
// required for reliable high feed rates on this board.
//   bit_word_addr = 0x42000000 + (reg - 0x40000000) * 32 + bit * 4
// GD32F4 GPIO offsets: ISTAT (input status) = +0x10, OCTL (output control) = +0x14.
#define GD32_BITBAND_ADDR(gpio_periph, reg_offs, pin) \
    ((volatile uint32_t *)(0x42000000UL + (((uint32_t)(gpio_periph) + (reg_offs)) - 0x40000000UL) * 32UL + (uint32_t)(pin) * 4UL))

#define DIGITAL_IN(gpio_periph, pin) \
    (*GD32_BITBAND_ADDR(gpio_periph, 0x10U, pin))

#define DIGITAL_OUT(gpio_periph, pin, on) \
    (*GD32_BITBAND_ADDR(gpio_periph, 0x14U, pin) = (on) ? 1U : 0U)

// ---------------------------------------------------------------------------
// Timer token macros
// ---------------------------------------------------------------------------
#define timer(t)        timerN(t)
#define timerN(t)       TIMER##t
#define timerBase(t)    timerbase(t)
#define timerbase(t)    TIMER##t
#define timerINT(t)     timerint(t)
#define timerint(t)     TIMER##t##_IRQn
#define timerHANDLER(t) timerhandler(t)
#define timerhandler(t) TIMER##t##_IRQHandler
#define timerCLKEN(t)   timerclken(t)
#define timerclken(t)   rcu_periph_clock_enable(RCU_TIMER##t)

// Timer base addresses as plain integer constants (usable in preprocessor conditionals,
// GD32's TIMERx macros are typed address expressions and cannot be used in #if).
#define TIMER0_BASE   0x40010000U
#define TIMER1_BASE   0x40000000U
#define TIMER2_BASE   0x40000400U
#define TIMER3_BASE   0x40000800U
#define TIMER4_BASE   0x40000C00U
#define TIMER5_BASE   0x40001000U
#define TIMER6_BASE   0x40001400U
#define TIMER7_BASE   0x40010400U

// ---------------------------------------------------------------------------
// USART token macros
// ---------------------------------------------------------------------------
#define usart(t)        usartN(t)
#define usartN(t)       USART##t
#define usartINT(t)     usartint(t)
#define usartint(t)     USART##t##_IRQn
#define usartHANDLER(t) usarthandler(t)
#define usarthandler(t) USART##t##_IRQHandler
#define usartCLKEN(t)   usartclken(t)
#define usartclken(t)   rcu_periph_clock_enable(RCU_USART##t)

// GD32 names the 4th/5th serial instances UART3/UART4 (there is no USART3/USART4).
// Aliases below let the token-pasting macros above work for all five instances.
// Every derived token needs its own alias: an object-like "#define USART3 UART3"
// does NOT match the single token "USART3_IRQn" produced by USART##3##_IRQn.
#define USART3              UART3
#define USART4              UART4
#define USART3_IRQn         UART3_IRQn
#define USART4_IRQn         UART4_IRQn
#define USART3_IRQHandler   UART3_IRQHandler
#define USART4_IRQHandler   UART4_IRQHandler
#define RCU_USART3          RCU_UART3
#define RCU_USART4          RCU_UART4

// Configuration, do not change here

#define CNC_BOOSTERPACK     0

// Define GPIO output mode options

#define GPIO_SHIFT0   0
#define GPIO_SHIFT1   1
#define GPIO_SHIFT2   2
#define GPIO_SHIFT3   3
#define GPIO_SHIFT4   4
#define GPIO_SHIFT5   5
#define GPIO_SHIFT6   6
#define GPIO_SHIFT7   7
#define GPIO_SHIFT8   8
#define GPIO_SHIFT9   9
#define GPIO_SHIFT10 10
#define GPIO_SHIFT11 11
#define GPIO_SHIFT12 12
#define GPIO_SHIFT13 13
#define GPIO_MAP     14
#define GPIO_BITBAND 15

#ifndef IS_NUCLEO_DEVKIT
#define IS_NUCLEO_DEVKIT 0
#endif

#ifndef CONTROL_ENABLE
#define CONTROL_ENABLE (CONTROL_HALT|CONTROL_FEED_HOLD|CONTROL_CYCLE_START)
#endif

#ifdef BOARD_BTT_SKR_PRO_1_2
#define BOARD_BTT_SKR_PRO_1_1
#endif

#if defined(BOARD_CNC_ED1_V20)
  #include "boards/cnc_ed1_v20_map.h"
#elif defined(BOARD_CNC_BOOSTERPACK)
  #include "boards/cnc_boosterpack_map.h"
#elif defined(BOARD_CNC3040)
  #include "boards/cnc3040_map.h"
#elif defined(BOARD_BLACKPILL)
  #include "boards/blackpill_map.h"
#elif defined(BOARD_BLACKPILL_ALT2)
  #include "boards/blackpill_alt2_map.h"
#elif defined(BOARD_DEVTRONIC_CNC_V2)
  #include "boards/Devtronic_CNC_Controller_V2_map.h"
#elif defined(BOARD_DEVTRONIC_CNC)
  #include "boards/Devtronic_CNC_Controller_map.h"
#elif defined(BOARD_BTT_SKR_PRO_1_1)
  #include "boards/btt_skr_pro_v1_1_map.h"
#elif defined(BOARD_BTT_SKR_20)
  #include "boards/btt_skr_2.0_map.h"
#elif defined(BOARD_BTT_SKR_20_DAC)
  #include "boards/btt_skr_2.0_dac_map.h"
#elif defined(BOARD_BTT_OCTOPUS_PRO)
  #include "boards/btt_octopus_pro_map.h"
#elif defined(BOARD_FYSETC_S6)
  #include "boards/fysetc_s6_map.h"
#elif defined(BOARD_PROTONEER_3XX)
  #include "boards/protoneer_3.xx_map.h"
#elif defined(BOARD_GENERIC_UNO)
  #include "boards/uno_map.h"
#elif defined(BOARD_MORPHO_CNC)
  #include "boards/st_morpho_map.h"
#elif defined(BOARD_MORPHO_DAC_CNC)
  #include "boards/st_morpho_dac_map.h"
#elif defined(BOARD_MINI_BLACKPILL)
  #include "boards/mini_blackpill_map.h"
#elif defined(BOARD_FLEXI_HAL)
  #include "boards/flexi_hal_map.h"
#elif defined(BOARD_STM32F401_UNI)
  #include "boards/stm32f401_uni_map.h"
#elif defined(BOARD_HALCYON_V1)
  #include "boards/halcyon_v1_map.h"
#elif defined(BOARD_MKS_ROBIN_NANO_30)
  #include "boards/mks_robin_nano_v3.0_map.h"
#elif defined(BOARD_LONGBOARD32) || defined(BOARD_LONGBOARD32_EXT)
  #include "boards/longboard32_map.h"
#elif defined(BOARD_MKS_EAGLE)
  #include "boards/mks_eagle_map.h"
#elif defined(BOARD_STM32F407VET6_DEV)
  #include "boards/stm32f407vet6_dev_board.h"
#elif defined(BOARD_MY_MACHINE)
  #include "boards/my_machine_map.h"
#else // default board
  #include "boards/generic_map.h"
#endif

#if ETHERNET_ENABLE && defined(_WIZCHIP_) && !defined(SPI_IRQ_PIN)
#error "Board does not support ethernet!"
#endif

// Define timer allocations.

// Steppers use TIMER3 (16-bit) - as in the previous working port. The 32-bit
// TIMER4 variant was found to lose steps at high feed rates on this board.
#define STEPPER_TIMER_N             3
#define STEPPER_TIMER_BASE          TIMER3_BASE
#define STEPPER_TIMER               timer(STEPPER_TIMER_N)
#define STEPPER_TIMER_CLKEN()       timerCLKEN(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQn          timerINT(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQHandler    timerHANDLER(STEPPER_TIMER_N)

#if SPINDLE_ENCODER_ENABLE

#ifndef RPM_COUNTER_N
#define RPM_COUNTER_N   3
#endif
#ifndef RPM_TIMER_N
#define RPM_TIMER_N     2
#endif

#define RPM_COUNTER                 timer(RPM_COUNTER_N)
#define RPM_COUNTER_BASE            timerBase(RPM_COUNTER_N)
#define RPM_COUNTER_CLKEN()         timerCLKEN(RPM_COUNTER_N)
#define RPM_COUNTER_IRQn            timerINT(RPM_COUNTER_N)
#define RPM_COUNTER_IRQHandler      timerHANDLER(RPM_COUNTER_N)

#define RPM_TIMER                   timer(RPM_TIMER_N)
#define RPM_TIMER_BASE              timerBase(RPM_TIMER_N)
#define RPM_TIMER_CLKEN()           timerCLKEN(RPM_TIMER_N)
#define RPM_TIMER_IRQn              timerINT(RPM_TIMER_N)
#define RPM_TIMER_IRQHandler        timerHANDLER(RPM_TIMER_N)

#endif //  SPINDLE_ENCODER_ENABLE

// STM32 auto-claims TIM7 for the TMC2209 UART (TRINAMIC_UART_ENABLE == 2);
// the GD32 equivalent of TIM7 is TIMER6 - restore that auto-claim here.
#if TRINAMIC_UART_ENABLE == 2 && !defined(TMC_UART_TIMER_N)
#define TMC_UART_TIMER_N 6
#endif

#ifdef TMC_UART_TIMER_N
#define TMC_UART_TIMER_BASE         timerBase(TMC_UART_TIMER_N)
#define IS_TIMER_CLAIMED(INSTANCE) (((INSTANCE) == STEPPER_TIMER_BASE) || \
                                    ((INSTANCE) == TMC_UART_TIMER_BASE))
#else
#define IS_TIMER_CLAIMED(INSTANCE) (((INSTANCE) == STEPPER_TIMER_BASE))
#endif

// Adjust these values to get more accurate step pulse timings when required, e.g if using high step rates.
// NOTE: step output mode, number of axes and compiler optimization setting may all affect these values.

// Minimum pulse off time.
#ifndef STEP_PULSE_TOFF_MIN
#define STEP_PULSE_TOFF_MIN 2.0f
#endif
// Time from main stepper interrupt to pulse output, must be less than STEP_PULSE_TOFF_MIN.
#ifndef STEP_PULSE_TON_LATENCY
#if SPINDLE_SYNC_ENABLE
#define STEP_PULSE_TON_LATENCY 1.35f
#else
#define STEP_PULSE_TON_LATENCY 0.95f
#endif
#endif
// Time from step out to step reset.
#ifndef STEP_PULSE_TOFF_LATENCY
#define STEP_PULSE_TOFF_LATENCY 0.85f
#endif
// Only used when step injection is enabled (stepper spindle and plasma THC)
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.0f // microseconds
#endif

// End configuration

#include "grbl/driver_opts2.h"

#if I2C_ENABLE && !defined(I2C_PORT)
#define I2C_PORT 2 // GPIOB, SCL_PIN = 10, SDA_PIN = 11
#endif

#if SPI_ENABLE && !defined(SPI_PORT)
#define SPI_PORT 1
#endif

#ifndef STEP_PINMODE
#define STEP_PINMODE PINMODE_OUTPUT
#endif

#ifndef DIRECTION_PINMODE
#define DIRECTION_PINMODE PINMODE_OUTPUT
#endif

#ifndef STEPPERS_ENABLE_PINMODE
#define STEPPERS_ENABLE_PINMODE PINMODE_OUTPUT
#endif

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
    uint32_t channel; // ADC channel;
    const char *description;
} input_signal_t;

typedef struct {
    float value;
    ioports_pwm_t data;
    const pwm_signal_t *port;
} pwm_out_t;

typedef struct {
    pin_function_t id;
    gpio_port_t port;
    uint8_t pin;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
    pwm_out_t *pwm;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

bool driver_init (void);
void Driver_IncTick (void);
void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode);
#ifdef HAS_BOARD_INIT
void board_init (void);
#endif

void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);

#endif // __DRIVER_H__
