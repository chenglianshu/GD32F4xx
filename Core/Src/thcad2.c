/*
  thcad2.c - analog input from frequency, for Mesa THCAD2 converter and GD32F4xx ARM processors

  Ported from the STM32F4xx driver, GD32F4xx SPL implementation.

  Part of grblHAL

  Copyright (c) 2025 Terje Io

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

#include "driver.h"

#if THCAD2_ENABLE

#include "grbl/plugins.h"
#include "grbl/ioports.h"

// Timer register mapping (GD32 SPL register macros), STM32F4xx names kept for the code below.
#define tmr_cr1(timer)      TIMER_CTL0(timer)
#define tmr_cr2(timer)      TIMER_CTL1(timer)
#define tmr_smcr(timer)     TIMER_SMCFG(timer)
#define tmr_dier(timer)     TIMER_DMAINTEN(timer)
#define tmr_egr(timer)      TIMER_SWEVG(timer)
#define tmr_cnt(timer)      TIMER_CNT(timer)
#define tmr_psc(timer)      TIMER_PSC(timer)
#define tmr_arr(timer)      TIMER_CAR(timer)

#define TIM_CR1_CKD_1       BIT(9)
#define TIM_CR1_URS         TIMER_CTL0_UPS
#define TIM_CR1_OPM         TIMER_CTL0_SPM
#define TIM_CR1_CEN         TIMER_CTL0_CEN
#define TIM_CR2_MMS_1       TIMER_TRI_OUT_SRC_UPDATE
#define TIM_CR2_TI1S        TIMER_CTL1_TI0S
#define TIM_DIER_UDE        TIMER_DMAINTEN_UPDEN
#define TIM_EGR_UG          TIMER_SWEVG_UPG

#define TIM_SMCR_SMS_0      BIT(0)
#define TIM_SMCR_SMS_1      BIT(1)
#define TIM_SMCR_SMS_2      BIT(2)
#define TIM_SMCR_ETF_2      BIT(10)
#define TIM_SMCR_ETF_3      BIT(11)
#define TIM_SMCR_TS_0       BIT(4)
#define TIM_SMCR_TS_2       BIT(6)
#define TIM_SMCR_ECE        TIMER_SMCFG_SMC1

// GD32 SPL does not define the STM32 CMSIS GPIOx_BASE names, values are identical.
#ifndef GPIOA_BASE
#define GPIOA_BASE 0x40020000U
#endif
#ifndef GPIOB_BASE
#define GPIOB_BASE 0x40020400U
#endif

 // TIMER1_CH1 (TIM2_CH2 on STM32F4xx) - only PA1 and PB3 currently supported
#if !defined(THCAD2_PORT) || !((THCAD2_PORT == GPIOA_BASE && THCAD2_PIN == 1) || (THCAD2_PORT == GPIOB_BASE && THCAD2_PIN == 3))
#error "THCAD2: no port/pin or unsupported port/pin"
#endif

// Timer numbering follows the GD32 driver convention: TIMERn is STM32F4xx TIM(n+1).
// PULSE_COUNTER: STM32F4xx TIM2 -> TIMER1, GATE_TIMER: STM32F4xx TIM3 -> TIMER2.
// Gate timer update DMA request (TIMER2_UP, TIM3_UP on STM32F4xx) is
// DMA0 channel 2, subperipheral 5 - GD32F4xx user manual Rev 2.5 Table 10-2
// "Peripheral requests to DMA0", PERIEN = 101 column channel 2:
// TIMER2_CH3 / TIMER2_UP / TIMER2_TG. STM32F4xx equivalent: DMA1 Stream2, channel 5.

#define PULSE_COUNTER_N     1
#define GATE_TIMER_N        2

#define PULSE_COUNTER       timer(PULSE_COUNTER_N)
#define PULSE_COUNTER_BASE  timerBase(PULSE_COUNTER_N)
#define PULSE_COUNTER_CLKEN() timerCLKEN(PULSE_COUNTER_N)

#define GATE_TIMER          timer(GATE_TIMER_N)
#define GATE_TIMER_BASE     timerBase(GATE_TIMER_N)
#define GATE_TIMER_CLKEN()  timerCLKEN(GATE_TIMER_N)

#define DMA_INSTANCE        DMA0
#define DMA_CHANNEL         DMA_CH2
#define DMA_SUBPERIPH       DMA_SUBPERI5
#define DMA_INSTANCE_CLKEN() rcu_periph_clock_enable(RCU_DMA0)
#define DMA_TX_IRQ          DMA0_Channel2_IRQn
#define DMA_TX_IRQ_HANDLER  DMA0_Channel2_IRQHandler

static float value = -1.0f;
static enumerate_pins_ptr on_enumerate_pins;
static io_ports_data_t analog = { .external = true };

// STM32F4xx HAL DMA_HandleTypeDef equivalent, single data mode (no FIFO concept).
// The STM32F4xx Init.FIFOMode ENABLE setting has no GD32F4xx equivalent and no
// behavioural effect for a one halfword transfer per request.
static dma_single_data_parameter_struct pwm_dma = {
    .periph_inc = DMA_PERIPH_INCREASE_DISABLE,
    .memory_inc = DMA_MEMORY_INCREASE_DISABLE,
    .periph_memory_width = DMA_PERIPH_WIDTH_16BIT,
    .circular_mode = DMA_CIRCULAR_MODE_DISABLE,
    .direction = DMA_PERIPH_TO_MEMORY,
    .number = 1,
    .priority = DMA_PRIORITY_MEDIUM
};

uint counter_slice, gate_slice;
uint timer_dma_chan;
uint16_t csr_stopval = 0;
uint32_t timers_enable;

static xbar_t thcad2 = {
    .id = 0,
    .pin = THCAD2_PIN,
    .function = Input_Analog_Aux0,
    .group = PinGroup_AuxInputAnalog,
    .port = &value,
    .cap = {
        .input = On,
        .analog = On,
        .resolution = Resolution_12bit,
        .external = On,
        .claimable = On
    },
    .mode = {
        .output = Off,
        .analog = On
    }
};

static float thcad2_in_state (xbar_t *input)
{
    return value;
}

static int32_t thcad2_wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    return port < analog.in.n_ports ? (int32_t)value : -1;
}

static bool set_pin_function (xbar_t *input, pin_function_t function)
{
    if(input->id == thcad2.id)
        thcad2.function = function;

    return input->id == thcad2.id;
}

static xbar_t *thcad2_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    memcpy(&pin, &thcad2, sizeof(xbar_t));

    if(dir == Port_Input && port < analog.in.n_ports) {
        pin.pin += analog.in.pin_base;
        pin.get_value = thcad2_in_state;
        pin.set_function = set_pin_function;
        info = &pin;
    }

    return info;
}

static void thcad2_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Input && port < analog.in.n_ports)
        thcad2.description = description;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    memcpy(&pin, &thcad2, sizeof(xbar_t));

    if(!low_level)
        pin.port = "THCAD2:";

    pin_info(&pin, data);
}

static void get_next_port (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxInputAnalog)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

static inline void get_sample (void)
{
    value = (float)csr_stopval;

    tmr_dier(GATE_TIMER) |= TIM_DIER_UDE;

    // HAL_DMA_Start_IT() equivalent: re-arm a single halfword transfer CNT -> csr_stopval.
    dma_channel_disable(DMA_INSTANCE, DMA_CHANNEL);
    // Clear error flags too - unlike HAL_DMA_IRQHandler (which clears all), the
    // error interrupts are not enabled, so anything but FTF would stick forever.
    dma_flag_clear(DMA_INSTANCE, DMA_CHANNEL, DMA_FLAG_FTF | DMA_FLAG_SDE | DMA_FLAG_TAE);
    dma_single_data_mode_init(DMA_INSTANCE, DMA_CHANNEL, &pwm_dma);
    dma_interrupt_enable(DMA_INSTANCE, DMA_CHANNEL, DMA_INT_FTF);
    dma_channel_enable(DMA_INSTANCE, DMA_CHANNEL);

    tmr_egr(PULSE_COUNTER) |= TIM_EGR_UG;
}

void thcad2_init (void)
{
    io_analog_t ports = {
        .ports = &analog,
        .get_pin_info = thcad2_get_pin_info,
        .wait_on_input = thcad2_wait_on_input,
        .set_pin_description = thcad2_set_pin_description
    };

    hal.enumerate_pins(false, get_next_port, &thcad2.function);

    analog.in.n_ports = 1;

    if(timer_claim(GATE_TIMER) &&
        timer_claim(PULSE_COUNTER) &&
         ioports_add_analog(&ports)) {

        GATE_TIMER_CLKEN();
        tmr_psc(GATE_TIMER) = timer_clk_enable(GATE_TIMER) / 10000000UL * 100 - 1;
        tmr_arr(GATE_TIMER) = 1020;
        tmr_cr1(GATE_TIMER) = TIM_CR1_CKD_1|TIM_CR1_URS|TIM_CR1_OPM;
        tmr_smcr(GATE_TIMER) = TIM_SMCR_TS_0;
        tmr_smcr(GATE_TIMER) |= TIM_SMCR_SMS_1|TIM_SMCR_SMS_2;

        PULSE_COUNTER_CLKEN();
#if 1
        tmr_smcr(PULSE_COUNTER) = TIM_SMCR_SMS_0|TIM_SMCR_SMS_1|TIM_SMCR_SMS_2|TIM_SMCR_ETF_2|TIM_SMCR_ETF_3|TIM_SMCR_TS_0|TIM_SMCR_TS_2;
#else
        tmr_smcr(PULSE_COUNTER) = TIM_SMCR_ECE;
#endif
        tmr_psc(PULSE_COUNTER) = 0;
        tmr_arr(PULSE_COUNTER) = 65535;
        tmr_cr2(PULSE_COUNTER) |= TIM_CR2_MMS_1|TIM_CR2_TI1S;
        tmr_cr1(PULSE_COUNTER) |= TIM_CR1_CEN;

        gpio_mode_set(THCAD2_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, 1 << THCAD2_PIN);
        gpio_output_options_set(THCAD2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, 1 << THCAD2_PIN);
#if PULSE_COUNTER_N == 1
        gpio_af_set(THCAD2_PORT, GPIO_AF_1, 1 << THCAD2_PIN); // TIMER1_CH1
#else
        gpio_af_set(THCAD2_PORT, GPIO_AF_2, 1 << THCAD2_PIN);
#endif

        static const periph_pin_t ssp = {
            .function = Input_SpindlePulse,
            .group = PinGroup_AuxInputAnalog,
            .port = (void *)THCAD2_PORT,
            .pin = THCAD2_PIN,
            .mode = { .mask = PINMODE_NONE }
        };

        hal.periph_port.register_pin(&ssp);

        DMA_INSTANCE_CLKEN();

        pwm_dma.periph_addr = (uint32_t)&tmr_cnt(PULSE_COUNTER);
        pwm_dma.memory0_addr = (uint32_t)&csr_stopval;
        pwm_dma.number = 1;
        dma_single_data_mode_init(DMA_INSTANCE, DMA_CHANNEL, &pwm_dma);
        dma_channel_subperipheral_select(DMA_INSTANCE, DMA_CHANNEL, DMA_SUBPERIPH);

        nvic_irq_enable(DMA_TX_IRQ, 0U, 0U);

        get_sample();
    }

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = onEnumeratePins;
}

ISR_CODE void DMA_TX_IRQ_HANDLER(void)
{
    // HAL_DMA_IRQHandler() equivalent, only the full transfer finish interrupt is enabled.
    dma_flag_clear(DMA_INSTANCE, DMA_CHANNEL, DMA_FLAG_FTF);

    get_sample();
}

#endif // THCAD2_ENABLE
