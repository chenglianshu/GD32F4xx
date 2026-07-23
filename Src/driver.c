/*

  driver.c - driver code for GD32F4xx ARM processors

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

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include "main.h"
#include "driver.h"
#include "serial.h"
#include "flash.h"

#include "grbl/task.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"
#include "grbl/state_machine.h"
#include "grbl/machine_limits.h"
#include "sdcard/sdcard.h"

#include "grbl/encoders.h"
#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif
#if USB_SERIAL_ENABLE
#include "usb_serial.h"
#endif
#if FLASH_ENABLE
#include "flash.h"
#endif
#if ETHERNET_ENABLE
#include "enet.h"
#endif

#if DRIVER_SPINDLE_ENABLE
void driver_spindles_init (void);
#endif

#if ENCODER_ENABLE
void driver_encoders_init (void);
#endif

#if AUX_ANALOG_ENABLE
void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
#endif

static uint32_t systick_safe_read = 0, cycles2us_factor = 0;
static bool IOInitDone = false;
static uint32_t aux_irq = 0;
static pin_group_pins_t limit_inputs = {0};

#define STEPPER_TIMER_DIV 4

// Direct register access macros for the stepper ISR to avoid SPL function call overhead.
#define STEPPER_INTF            TIMER_INTF(STEPPER_TIMER)
#define STEPPER_DMAINTEN        TIMER_DMAINTEN(STEPPER_TIMER)
#define STEPPER_CNT             TIMER_CNT(STEPPER_TIMER)
#define STEPPER_CAR             TIMER_CAR(STEPPER_TIMER)
#define STEPPER_CH1CV           TIMER_CH1CV(STEPPER_TIMER)
#define STEPPER_CH2CV           TIMER_CH2CV(STEPPER_TIMER)

#define stepper_ifg(flag)       ((STEPPER_INTF & STEPPER_DMAINTEN & (flag)) != 0)
#define stepper_ie_clear(src)   (STEPPER_DMAINTEN &= ~(src))
#define stepper_ie_set(src)     (STEPPER_DMAINTEN |= (src))
#define stepper_ifg_clear(flag) (STEPPER_INTF = ~(flag))

#define DRIVER_IRQMASK (LIMIT_MASK|DEVICES_IRQ_MASK)

static struct {
    // t_* parameters are timer ticks
    uint32_t t_min_period;
    uint32_t t_on; // delayed pulse
    uint32_t t_off;
    uint32_t t_on_off_min;
    uint32_t t_off_min;
    uint32_t t_dly_off_min;
    axes_signals_t out;
#if STEP_INJECT_ENABLE
    uint32_t length;
    uint32_t delay;
    struct {
        hal_timer_t timer;
        axes_signals_t claimed;
        volatile axes_signals_t axes;
        volatile axes_signals_t out;
    } inject;
#endif
} step_pulse = {};

static periph_signal_t *periph_pins = NULL;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
#if SAFETY_DOOR_ENABLE
static pin_debounce_t debounce = {0};
#endif
static input_signal_t *pin_irq[16] = {0};
#ifdef Z_LIMIT_POLL
static input_signal_t *z_limit_pin = NULL;
static bool z_limits_irq_enabled = false;
#endif

#if defined(I2C_STROBE_PIN) || SPI_IRQ_BIT

#if defined(I2C_STROBE_PIN)
static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };
#endif

#if SPI_IRQ_BIT
static driver_irq_handler_t spi_irq = { .type = IRQ_SPI };
#endif

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok = false;

    switch(irq) {

#if defined(I2C_STROBE_PIN)
        case IRQ_I2C_Strobe:
            if((ok = i2c_strobe.callback == NULL))
                i2c_strobe.callback = handler;
            break;
#endif

#if SPI_IRQ_BIT
        case IRQ_SPI:
            if((ok = spi_irq.callback == NULL))
                spi_irq.callback = handler;
            break;
#endif

        default:
            break;
    }

    return ok;
}

#endif // defined(I2C_STROBE_PIN) || SPI_IRQ_BIT

// -----------------------------------------------------------------------------
// Pin tables
// -----------------------------------------------------------------------------

static output_signal_t outputpin[] = {
    { .id = Output_StepX,           .port = X_STEP_PORT,            .pin = X_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
    { .id = Output_StepY,           .port = Y_STEP_PORT,            .pin = Y_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
    { .id = Output_StepZ,           .port = Z_STEP_PORT,            .pin = Z_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#ifdef A_AXIS
    { .id = Output_StepA,           .port = A_STEP_PORT,            .pin = A_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = B_STEP_PORT,            .pin = B_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = C_STEP_PORT,            .pin = C_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef U_AXIS
    { .id = Output_StepU,           .port = U_STEP_PORT,            .pin = U_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef V_AXIS
    { .id = Output_StepV,           .port = V_STEP_PORT,            .pin = V_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef W_AXIS
    { .id = Output_StepW,           .port = W_STEP_PORT,            .pin = W_STEP_PIN,              .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef X2_STEP_PIN
    { .id = Output_StepX_2,         .port = X2_STEP_PORT,           .pin = X2_STEP_PIN,             .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef Y2_STEP_PIN
    { .id = Output_StepY_2,         .port = Y2_STEP_PORT,           .pin = Y2_STEP_PIN,             .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
#ifdef Z2_STEP_PIN
    { .id = Output_StepZ_2,         .port = Z2_STEP_PORT,           .pin = Z2_STEP_PIN,             .group = PinGroup_StepperStep,   .mode = {STEP_PINMODE} },
#endif
    { .id = Output_DirX,            .port = X_DIRECTION_PORT,       .pin = X_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
    { .id = Output_DirY,            .port = Y_DIRECTION_PORT,       .pin = Y_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
    { .id = Output_DirZ,            .port = Z_DIRECTION_PORT,       .pin = Z_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#ifdef A_AXIS
    { .id = Output_DirA,            .port = A_DIRECTION_PORT,       .pin = A_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = B_DIRECTION_PORT,       .pin = B_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = C_DIRECTION_PORT,       .pin = C_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef U_AXIS
    { .id = Output_DirU,            .port = U_DIRECTION_PORT,       .pin = U_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef V_AXIS
    { .id = Output_DirV,            .port = V_DIRECTION_PORT,       .pin = V_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef W_AXIS
    { .id = Output_DirW,            .port = W_DIRECTION_PORT,       .pin = W_DIRECTION_PIN,         .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,          .port = X2_DIRECTION_PORT,      .pin = X2_DIRECTION_PIN,        .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,          .port = Y2_DIRECTION_PORT,      .pin = Y2_DIRECTION_PIN,        .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,          .port = Z2_DIRECTION_PORT,      .pin = Z2_DIRECTION_PIN,        .group = PinGroup_StepperDir,    .mode = {DIRECTION_PINMODE} },
#endif
#ifdef STEPPERS_POWER_PORT
    { .id = Output_StepperPower,    .port = STEPPERS_POWER_PORT,    .pin = STEPPERS_POWER_PIN,      .group = PinGroup_StepperPower },
#endif
#if !TRINAMIC_MOTOR_ENABLE
#ifdef STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnable,   .port = STEPPERS_ENABLE_PORT,   .pin = STEPPERS_ENABLE_PIN,     .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef X_ENABLE_PORT
    { .id = Output_StepperEnableX,  .port = X_ENABLE_PORT,          .pin = X_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Y_ENABLE_PORT
    { .id = Output_StepperEnableY,  .port = Y_ENABLE_PORT,          .pin = Y_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Z_ENABLE_PORT
    { .id = Output_StepperEnableZ,  .port = Z_ENABLE_PORT,          .pin = Z_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef A_ENABLE_PORT
    { .id = Output_StepperEnableA,  .port = A_ENABLE_PORT,          .pin = A_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef B_ENABLE_PORT
    { .id = Output_StepperEnableB,  .port = B_ENABLE_PORT,          .pin = B_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef C_ENABLE_PORT
    { .id = Output_StepperEnableC,  .port = C_ENABLE_PORT,          .pin = C_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef U_ENABLE_PORT
    { .id = Output_StepperEnableU,  .port = U_ENABLE_PORT,          .pin = U_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef V_ENABLE_PORT
    { .id = Output_StepperEnableV,  .port = V_ENABLE_PORT,          .pin = V_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef W_ENABLE_PORT
    { .id = Output_StepperEnableW,  .port = W_ENABLE_PORT,          .pin = W_ENABLE_PIN,            .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef X2_ENABLE_PIN
    { .id = Output_StepperEnableX,  .port = X2_ENABLE_PORT,         .pin = X2_ENABLE_PIN,           .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Y2_ENABLE_PIN
    { .id = Output_StepperEnableY,  .port = Y2_ENABLE_PORT,         .pin = Y2_ENABLE_PIN,           .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#ifdef Z2_ENABLE_PIN
    { .id = Output_StepperEnableZ,  .port = Z2_ENABLE_PORT,         .pin = Z2_ENABLE_PIN,           .group = PinGroup_StepperEnable, .mode = {STEPPERS_ENABLE_PINMODE} },
#endif
#endif // TRINAMIC_MOTOR_ENABLE
#ifdef MOTOR_CS_PIN
    { .id = Output_MotorChipSelect,     .port = MOTOR_CS_PORT,      .pin = MOTOR_CS_PIN,            .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSX_PIN
    { .id = Output_MotorChipSelectX,    .port = MOTOR_CSX_PORT,     .pin = MOTOR_CSX_PIN,           .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSY_PIN
    { .id = Output_MotorChipSelectY,    .port = MOTOR_CSY_PORT,     .pin = MOTOR_CSY_PIN,           .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSZ_PIN
    { .id = Output_MotorChipSelectZ,    .port = MOTOR_CSZ_PORT,     .pin = MOTOR_CSZ_PIN,           .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM3_PIN
    { .id = Output_MotorChipSelectM3,   .port = MOTOR_CSM3_PORT,    .pin = MOTOR_CSM3_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM4_PIN
    { .id = Output_MotorChipSelectM4,   .port = MOTOR_CSM4_PORT,    .pin = MOTOR_CSM4_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM5_PIN
    { .id = Output_MotorChipSelectM5,   .port = MOTOR_CSM5_PORT,    .pin = MOTOR_CSM5_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM6_PIN
    { .id = Output_MotorChipSelectM6,   .port = MOTOR_CSM6_PORT,    .pin = MOTOR_CSM6_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM7_PIN
    { .id = Output_MotorChipSelectM7,   .port = MOTOR_CSM7_PORT,    .pin = MOTOR_CSM7_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CS_PIN
    { .id = Output_MotorChipSelect,     .port = MOTOR_CS_PORT,      .pin = MOTOR_CS_PIN,            .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_UARTX_PIN
    { .id = Bidirectional_MotorUARTX,   .port = MOTOR_UARTX_PORT,   .pin = MOTOR_UARTX_PIN,         .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTY_PIN
    { .id = Bidirectional_MotorUARTY,   .port = MOTOR_UARTY_PORT,   .pin = MOTOR_UARTY_PIN,         .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTZ_PIN
    { .id = Bidirectional_MotorUARTZ,   .port = MOTOR_UARTZ_PORT,   .pin = MOTOR_UARTZ_PIN,         .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM3_PIN
    { .id = Bidirectional_MotorUARTM3,  .port = MOTOR_UARTM3_PORT,  .pin = MOTOR_UARTM3_PIN,        .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM4_PIN
    { .id = Bidirectional_MotorUARTM4,  .port = MOTOR_UARTM4_PORT,  .pin = MOTOR_UARTM4_PIN,        .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM5_PIN
    { .id = Bidirectional_MotorUARTM5,  .port = MOTOR_UARTM5_PORT,  .pin = MOTOR_UARTM5_PIN,        .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM6_PIN
    { .id = Bidirectional_MotorUARTM6,  .port = MOTOR_UARTM6_PORT,  .pin = MOTOR_UARTM6_PIN,        .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM7_PIN
    { .id = Bidirectional_MotorUARTM7,  .port = MOTOR_UARTM7_PORT,  .pin = MOTOR_UARTM7_PIN,        .group = PinGroup_MotorUART },
#endif
#ifdef FLASH_CS_PORT
    { .id = Output_FlashCS,         .port = FLASH_CS_PORT,          .pin = FLASH_CS_PIN,            .group = PinGroup_SPICS },
#endif
#ifdef SD_CS_PORT
    { .id = Output_SdCardCS,        .port = SD_CS_PORT,             .pin = SD_CS_PIN,               .group = PinGroup_SPICS },
#endif
#ifdef SPI_CS_PORT
    { .id = Output_SPICS0,           .port = SPI_CS_PORT,           .pin = SPI_CS_PIN,              .group = PinGroup_SPICS },
#endif
#ifdef SPI_CS1_PORT
    { .id = Output_SPICS1,           .port = SPI_CS1_PORT,          .pin = SPI_CS1_PIN,             .group = PinGroup_SPICS },
#endif
#ifdef SPI_CS2_PORT
    { .id = Output_SPICS2,           .port = SPI_CS2_PORT,          .pin = SPI_CS2_PIN,             .group = PinGroup_SPICS },
#endif
#ifdef SPI_CS3_PORT
    { .id = Output_SPICS3,           .port = SPI_CS3_PORT,          .pin = SPI_CS3_PIN,             .group = PinGroup_SPICS },
#endif
#ifdef SPI_RST_PORT
    { .id = Output_SPIRST,          .port = SPI_RST_PORT,           .pin = SPI_RST_PIN,             .group = PinGroup_SPI },
#endif
#if defined(MODBUS_RTU_STREAM) && defined(RS485_DIR_PORT)
    { .id = Output_RS485_Direction, .port = RS485_DIR_PORT,         .pin = RS485_DIR_PIN,           .group = PinGroup_UART + MODBUS_RTU_STREAM },
#endif
#ifdef LED_R_PORT
    { .id = Output_LED_R,           .port = LED_R_PORT,             .pin = LED_R_PIN,               .group = PinGroup_LED },
#endif
#ifdef LED_G_PORT
    { .id = Output_LED_G,           .port = LED_G_PORT,             .pin = LED_G_PIN,               .group = PinGroup_LED },
#endif
#ifdef LED_B_PORT
    { .id = Output_LED_B,           .port = LED_B_PORT,             .pin = LED_B_PIN,               .group = PinGroup_LED },
#endif
#ifdef LED_W_PORT
    { .id = Output_LED_W,           .port = LED_W_PORT,             .pin = LED_W_PIN,               .group = PinGroup_LED },
#endif
#ifdef SPINDLE_PWM_PIN
    { .id = Output_SpindlePWM,      .port = SPINDLE_PWM_PORT,       .pin = SPINDLE_PWM_PIN,         .group = PinGroup_SpindlePWM },
#endif
#ifdef AUXOUTPUT0_PORT
    { .id = Output_Aux0,            .port = AUXOUTPUT0_PORT,        .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PORT
    { .id = Output_Aux1,            .port = AUXOUTPUT1_PORT,        .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PORT
    { .id = Output_Aux2,            .port = AUXOUTPUT2_PORT,        .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT3_PORT
    { .id = Output_Aux3,            .port = AUXOUTPUT3_PORT,        .pin = AUXOUTPUT3_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT4_PORT
    { .id = Output_Aux4,            .port = AUXOUTPUT4_PORT,        .pin = AUXOUTPUT4_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT5_PORT
    { .id = Output_Aux5,            .port = AUXOUTPUT5_PORT,        .pin = AUXOUTPUT5_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT6_PORT
    { .id = Output_Aux6,            .port = AUXOUTPUT6_PORT,        .pin = AUXOUTPUT6_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT7_PORT
    { .id = Output_Aux7,            .port = AUXOUTPUT7_PORT,        .pin = AUXOUTPUT7_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT8_PORT
    { .id = Output_Aux8,            .port = AUXOUTPUT8_PORT,        .pin = AUXOUTPUT8_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT9_PORT
    { .id = Output_Aux9,            .port = AUXOUTPUT9_PORT,        .pin = AUXOUTPUT9_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT10_PORT
    { .id = Output_Aux10,           .port = AUXOUTPUT10_PORT,       .pin = AUXOUTPUT10_PIN,         .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT11_PORT
    { .id = Output_Aux11,           .port = AUXOUTPUT11_PORT,       .pin = AUXOUTPUT11_PIN,         .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT12_PORT
    { .id = Output_Aux12,           .port = AUXOUTPUT12_PORT,       .pin = AUXOUTPUT12_PIN,         .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT13_PORT
    { .id = Output_Aux13,           .port = AUXOUTPUT13_PORT,       .pin = AUXOUTPUT13_PIN,         .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT14_PORT
    { .id = Output_Aux14,           .port = AUXOUTPUT14_PORT,       .pin = AUXOUTPUT14_PIN,         .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT15_PORT
    { .id = Output_Aux15,           .port = AUXOUTPUT15_PORT,       .pin = AUXOUTPUT15_PIN,         .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT0_ANALOG_PORT
    { .id = Output_Analog_Aux0,     .port = AUXOUTPUT0_ANALOG_PORT, .pin = AUXOUTPUT0_ANALOG_PIN,   .group = PinGroup_AuxOutputAnalog },
#elif defined(AUXOUTPUT0_PWM_PORT)
    { .id = Output_Analog_Aux0,     .port = AUXOUTPUT0_PWM_PORT,    .pin = AUXOUTPUT0_PWM_PIN,      .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
#ifdef AUXOUTPUT1_ANALOG_PORT
    { .id = Output_Analog_Aux1,     .port = AUXOUTPUT1_ANALOG_PORT, .pin = AUXOUTPUT1_ANALOG_PIN,   .group = PinGroup_AuxOutputAnalog }
#elif defined(AUXOUTPUT1_PWM_PORT)
    { .id = Output_Analog_Aux1,     .port = AUXOUTPUT1_PWM_PORT,    .pin = AUXOUTPUT1_PWM_PIN,      .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
#ifdef AUXOUTPUT2_ANALOG_PORT
    { .id = Output_Analog_Aux2,     .port = AUXOUTPUT2_ANALOG_PORT, .pin = AUXOUTPUT2_ANALOG_PIN,   .group = PinGroup_AuxOutputAnalog }
#elif defined(AUXOUTPUT2_PWM_PORT)
    { .id = Output_Analog_Aux2,     .port = AUXOUTPUT2_PWM_PORT,    .pin = AUXOUTPUT2_PWM_PIN,      .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
};

static input_signal_t inputpin[] = {
// Limit input pins must be consecutive in this array
    { .id = Input_LimitX,         .port = X_LIMIT_PORT,       .pin = X_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,       .port = X2_LIMIT_PORT,      .pin = X2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef X_LIMIT_PIN_MAX
    { .id = Input_LimitX_Max,     .port = X_LIMIT_PORT_MAX,   .pin = X_LIMIT_PIN_MAX,     .group = PinGroup_Limit },
#endif
    { .id = Input_LimitY,         .port = Y_LIMIT_PORT,       .pin = Y_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_2,       .port = Y2_LIMIT_PORT,      .pin = Y2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef Y_LIMIT_PIN_MAX
    { .id = Input_LimitY_Max,     .port = Y_LIMIT_PORT_MAX,   .pin = Y_LIMIT_PIN_MAX,     .group = PinGroup_Limit },
#endif
    { .id = Input_LimitZ,         .port = Z_LIMIT_PORT,       .pin = Z_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef Z2_LIMIT_PIN
    { .id = Input_LimitZ_2,       .port = Z2_LIMIT_PORT,      .pin = Z2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef Z_LIMIT_PIN_MAX
    { .id = Input_LimitZ_Max,     .port = Z_LIMIT_PORT_MAX,   .pin = Z_LIMIT_PIN_MAX,     .group = PinGroup_Limit },
#endif
#ifdef A_LIMIT_PIN
    { .id = Input_LimitA,         .port = A_LIMIT_PORT,       .pin = A_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef B_LIMIT_PIN
    { .id = Input_LimitB,         .port = B_LIMIT_PORT,       .pin = B_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef C_LIMIT_PIN
    { .id = Input_LimitC,         .port = C_LIMIT_PORT,       .pin = C_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef U_LIMIT_PIN
    { .id = Input_LimitU,         .port = U_LIMIT_PORT,       .pin = U_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef V_LIMIT_PIN
    { .id = Input_LimitV,         .port = V_LIMIT_PORT,       .pin = V_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef W_LIMIT_PIN
    { .id = Input_LimitW,         .port = W_LIMIT_PORT,       .pin = W_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
// HOME input pins must be consecutive in this array
#ifdef X_HOME_PIN
    { .id = Input_HomeX,          .port = X_HOME_PORT,        .pin = X_HOME_PIN,          .group = PinGroup_Home },
#endif
#ifdef X2_HOME_PIN
    { .id = Input_HomeX_2,        .port = X2_HOME_PORT,       .pin = X2_HOME_PIN,         .group = PinGroup_Home },
#endif
#ifdef Y_HOME_PIN
    { .id = Input_HomeY,          .port = Y_HOME_PORT,        .pin = Y_HOME_PIN,          .group = PinGroup_Home },
#endif
#ifdef Y2_HOME_PIN
    { .id = Input_HomeY_2,        .port = Y2_HOME_PORT,       .pin = Y2_HOME_PIN,         .group = PinGroup_Home },
#endif
#ifdef Z_HOME_PIN
    { .id = Input_HomeZ,          .port = Z_HOME_PORT,        .pin = Z_HOME_PIN,          .group = PinGroup_Home },
#endif
#ifdef Z2_HOME_PIN
    { .id = Input_HomeZ_2,        .port = Z2_HOME_PORT,       .pin = Z2_HOME_PIN,         .group = PinGroup_Home },
#endif
#ifdef A_HOME_PIN
    { .id = Input_HomeA,          .port = A_HOME_PORT,        .pin = A_HOME_PIN,          .group = PinGroup_Home },
#endif
#ifdef B_HOME_PIN
    { .id = Input_HomeB,          .port = B_HOME_PORT,        .pin = B_HOME_PIN,          .group = PinGroup_Home },
#endif
#ifdef C_HOME_PIN
    { .id = Input_HomeC,          .port = C_HOME_PORT,        .pin = C_HOME_PIN,          .group = PinGroup_Home },
#endif
#ifdef U_HOME_PIN
    { .id = Input_HomeU,          .port = U_HOME_PORT,        .pin = U_HOME_PIN,          .group = PinGroup_Home },
#endif
#ifdef V_HOME_PIN
    { .id = Input_HomeV,          .port = V_HOME_PORT,        .pin = V_HOME_PIN,          .group = PinGroup_Home },
#endif
#ifdef W_HOME_PIN
    { .id = Input_HomeW,          .port = W_HOME_PORT,        .pin = W_HOME_PIN,          .group = PinGroup_Home },
#endif
#ifdef MOTOR_FAULT_PIN
    { .id = Input_MotorFaultX,    .port = X_MOTOR_FAULT_PORT,  .pin = X_MOTOR_FAULT_PIN,  .group = PinGroup_Motor_Fault },
#endif
#ifdef SPINDLE_INDEX_PIN
    { .id = Input_SpindleIndex,   .port = SPINDLE_INDEX_PORT, .pin = SPINDLE_INDEX_PIN,   .group = PinGroup_SpindleIndex },
#endif
#ifdef SPI_IRQ_PORT
    { .id = Input_SPIIRQ,         .port = SPI_IRQ_PORT,       .pin = SPI_IRQ_PIN,         .group = PinGroup_SPI },
#endif
#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
    { .id = Input_SdCardDetect,   .port = SD_DETECT_PORT,     .pin = SD_DETECT_PIN,       .group = PinGroup_SdCard },
#endif
// Aux input pins must be consecutive in this array
#ifdef AUXINPUT0_PIN
    { .id = Input_Aux0,           .port = AUXINPUT0_PORT,     .pin = AUXINPUT0_PIN,       .group = PinGroup_AuxInput, .description = "Aux in 0" },
#endif
#ifdef AUXINPUT1_PIN
    { .id = Input_Aux1,           .port = AUXINPUT1_PORT,     .pin = AUXINPUT1_PIN,       .group = PinGroup_AuxInput, .description = "Aux in 1" },
#endif
#ifdef AUXINPUT2_PIN
    { .id = Input_Aux2,           .port = AUXINPUT2_PORT,     .pin = AUXINPUT2_PIN,       .group = PinGroup_AuxInput, .description = "Aux in 2" },
#endif
#ifdef AUXINPUT3_PIN
    { .id = Input_Aux3,           .port = AUXINPUT3_PORT,     .pin = AUXINPUT3_PIN,       .group = PinGroup_AuxInput, .description = "Aux in 3" },
#endif
#ifdef AUXINPUT4_PIN
    { .id = Input_Aux4,           .port = AUXINPUT4_PORT,     .pin = AUXINPUT4_PIN,       .group = PinGroup_AuxInput, .description = "Aux in 4" },
#endif
#ifdef AUXINPUT5_PIN
    { .id = Input_Aux5,           .port = AUXINPUT5_PORT,     .pin = AUXINPUT5_PIN,       .group = PinGroup_AuxInput, .description = "Aux in 5" },
#endif
#ifdef AUXINPUT6_PIN
    { .id = Input_Aux6,           .port = AUXINPUT6_PORT,     .pin = AUXINPUT6_PIN,       .group = PinGroup_AuxInput, .description = "Aux in 6" },
#endif
#ifdef AUXINPUT7_PIN
    { .id = Input_Aux7,           .port = AUXINPUT7_PORT,     .pin = AUXINPUT7_PIN,       .group = PinGroup_AuxInput, .description = "Aux in 7" },
#endif
#ifdef AUXINPUT8_PIN
    { .id = Input_Aux8,           .port = AUXINPUT8_PORT,     .pin = AUXINPUT8_PIN,       .group = PinGroup_AuxInput, .description = "Aux in 8" },
#endif
#ifdef AUXINPUT9_PIN
    { .id = Input_Aux9,           .port = AUXINPUT9_PORT,     .pin = AUXINPUT9_PIN,       .group = PinGroup_AuxInput, .description = "Aux in 9" },
#endif
#ifdef AUXINPUT10_PIN
    { .id = Input_Aux10,          .port = AUXINPUT10_PORT,    .pin = AUXINPUT10_PIN,      .group = PinGroup_AuxInput, .description = "Aux in 10" },
#endif
#ifdef AUXINPUT11_PIN
    { .id = Input_Aux11,          .port = AUXINPUT11_PORT,    .pin = AUXINPUT11_PIN,      .group = PinGroup_AuxInput, .description = "Aux in 11" },
#endif
#ifdef AUXINPUT12_PIN
    { .id = Input_Aux12,          .port = AUXINPUT12_PORT,    .pin = AUXINPUT12_PIN,      .group = PinGroup_AuxInput, .description = "Aux in 12" },
#endif
#ifdef AUXINPUT13_PIN
    { .id = Input_Aux13,          .port = AUXINPUT13_PORT,    .pin = AUXINPUT13_PIN,      .group = PinGroup_AuxInput, .description = "Aux in 13" },
#endif
#ifdef AUXINPUT14_PIN
    { .id = Input_Aux14,          .port = AUXINPUT14_PORT,    .pin = AUXINPUT14_PIN,      .group = PinGroup_AuxInput, .description = "Aux in 14" },
#endif
#ifdef AUXINPUT15_PIN
    { .id = Input_Aux15,          .port = AUXINPUT15_PORT,    .pin = AUXINPUT15_PIN,      .group = PinGroup_AuxInput, .description = "Aux in 15" },
#endif
#if N_AUX_DIN > 16
#ifdef AUXINPUT16_PIN
    { .id = Input_Aux16,          .port = AUXINPUT16_PORT,    .pin = AUXINPUT16_PIN,      .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT17_PIN
    { .id = Input_Aux17,          .port = AUXINPUT17_PORT,    .pin = AUXINPUT17_PIN,      .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT18_PIN
    { .id = Input_Aux18,          .port = AUXINPUT18_PORT,    .pin = AUXINPUT18_PIN,      .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT19_PIN
    { .id = Input_Aux19,          .port = AUXINPUT19_PORT,    .pin = AUXINPUT19_PIN,      .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT20_PIN
    { .id = Input_Aux20,          .port = AUXINPUT20_PORT,    .pin = AUXINPUT20_PIN,      .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT21_PIN
    { .id = Input_Aux21,          .port = AUXINPUT21_PORT,    .pin = AUXINPUT21_PIN,      .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT22_PIN
    { .id = Input_Aux22,          .port = AUXINPUT22_PORT,    .pin = AUXINPUT22_PIN,      .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT23_PIN
    { .id = Input_Aux23,          .port = AUXINPUT23_PORT,    .pin = AUXINPUT23_PIN,      .group = PinGroup_AuxInput },
#endif
#endif // N_AUX_DIN > 16
#ifdef AUXINPUT0_ANALOG_PIN
    { .id = Input_Analog_Aux0,    .port = AUXINPUT0_ANALOG_PORT, .pin = AUXINPUT0_ANALOG_PIN, .group = PinGroup_AuxInputAnalog, .description = "Analog aux in 0" },
#endif
#ifdef AUXINPUT1_ANALOG_PIN
    { .id = Input_Analog_Aux1,    .port = AUXINPUT1_ANALOG_PORT, .pin = AUXINPUT1_ANALOG_PIN, .group = PinGroup_AuxInputAnalog, .description = "Analog aux in 1" }
#endif
};

// -----------------------------------------------------------------------------
// EXTI interrupt handling
// -----------------------------------------------------------------------------

static limit_signals_t limitsGetState (void);
static control_signals_t systemGetState (void);
static void core_pin_debounce (void *pin);
static void aux_pin_debounce (void *pin);
static void aux_irq_handler (uint8_t port, bool state);

static uint32_t getElapsedTicks (void);
static uint64_t getElapsedMicros (void);
static void driver_delay (uint32_t ms, delay_callback_ptr callback);
static void bitsSetAtomic (volatile uint_fast16_t *value, uint_fast16_t bits);
static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *value, uint_fast16_t bits);
static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *value, uint_fast16_t v);

#if DRIVER_PROBES
static bool probeGetState (void *input);
#endif

#if STEP_INJECT_ENABLE
static void step_inject_on (void *context);
static void step_inject_off (void *context);
static ISR_CODE void stepperOutputStep (axes_signals_t step_out, axes_signals_t dir_out);
static void stepperClaimMotor (uint_fast8_t axis_id, bool claim);
#endif

#ifdef MPG_MODE_PIN
static void mpg_select (void *data);
static void mpg_enable (void *data);
#endif

__attribute__((weak)) void motor_fault_add_pin (input_signal_t *input, xbar_t *pin);
#ifdef USE_EXPANDERS
__attribute__((weak)) bool input_add_expander_pin (xbar_t *pin);
#endif

void encoder_pin_claimed (uint8_t port, xbar_t *pin);

void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode)
{
    uint32_t port_index = GPIO_GET_INDEX(input->port);

    exti_line_enum line = (exti_line_enum)(1U << input->pin);

    syscfg_exti_line_config((uint8_t)(EXTI_SOURCE_GPIOA + port_index), input->pin);

    exti_trig_type_enum trig;
    if (irq_mode == IRQ_Mode_Rising)
        trig = EXTI_TRIG_RISING;
    else if (irq_mode == IRQ_Mode_Falling)
        trig = EXTI_TRIG_FALLING;
    else
        trig = EXTI_TRIG_BOTH;

    exti_init(line, EXTI_INTERRUPT, trig);
    exti_interrupt_flag_clear(line);
    exti_interrupt_enable(line);

    uint8_t irqn;
    if (input->pin < 5)
        irqn = EXTI0_IRQn + input->pin;
    else if (input->pin < 10)
        irqn = EXTI5_9_IRQn;
    else
        irqn = EXTI10_15_IRQn;

    nvic_irq_enable((uint8_t)irqn, 1U, 0U);

    pin_irq[input->pin] = (input_signal_t *)input;
}

static void core_pin_debounce (void *pin)
{
    input_signal_t *input = (input_signal_t *)pin;

#if SAFETY_DOOR_ENABLE
    if(input->id == Input_SafetyDoor)
        debounce.safety_door = Off;
#endif
#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
    if(input->group & PinGroup_SdCard)
        sdcard_detect(!DIGITAL_IN(SD_DETECT_PORT, SD_DETECT_PIN));
#endif

    if(input->mode.irq_mode == IRQ_Mode_Change ||
         DIGITAL_IN(input->port, input->pin) == (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1)) {

        if(input->group & (PinGroup_Control))
            hal.control.interrupt_callback(systemGetState());

        if(input->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            limit_signals_t state = limitsGetState();
            if(limit_signals_merge(state).value)
                hal.limits.interrupt_callback(state);
        }
    }

#ifndef Z_LIMIT_POLL
    exti_interrupt_enable((exti_line_enum)input->bit); // Reenable pin interrupt
#else
    if(input != z_limit_pin)
        exti_interrupt_enable((exti_line_enum)input->bit); // Reenable pin interrupt
#endif
}

static inline void core_pin_irq (uint32_t bit)
{
    input_signal_t *input;
    if((input = pin_irq[__builtin_ffs(bit) - 1])) {
        if(input->mode.debounce && task_add_delayed(core_pin_debounce, input, 40)) {
            exti_interrupt_disable((exti_line_enum)input->bit); // Disable pin interrupt
        } else
            core_pin_debounce(input);
    }
}

static void aux_pin_debounce (void *pin)
{
    input_signal_t *input = (input_signal_t *)pin;

#if SAFETY_DOOR_ENABLE
    if(input->id == Input_SafetyDoor)
        debounce.safety_door = Off;
#endif

    if(input->mode.irq_mode == IRQ_Mode_Change ||
          DIGITAL_IN(input->port, input->pin) == (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1))
        ioports_event(input);

    exti_interrupt_enable((exti_line_enum)input->bit); // Reenable pin interrupt
}

static inline void aux_pin_irq (uint32_t bit)
{
    input_signal_t *input;
    if((input = pin_irq[__builtin_ffs(bit) - 1]) && input->group == PinGroup_AuxInput) {
        if(input->mode.debounce && task_add_delayed(aux_pin_debounce, input, 40)) {
            exti_interrupt_disable((exti_line_enum)input->bit); // Disable pin interrupt
#if SAFETY_DOOR_ENABLE
            if(input->id == Input_SafetyDoor)
                debounce.safety_door = input->mode.debounce;
#endif
        } else
            ioports_event(input);
    }
}

#ifdef MPG_MODE_PIN

static void mpg_select (void *data)
{
    stream_mpg_enable(DIGITAL_IN(MPG_MODE_PORT, MPG_MODE_PIN) == 0);
}

static void mpg_enable (void *data)
{
    if(sys.mpg_mode != (DIGITAL_IN(MPG_MODE_PORT, MPG_MODE_PIN) == 0))
        stream_mpg_enable(true);
}

#endif // MPG_MODE_PIN

static void aux_irq_handler (uint8_t port, bool state)
{
    aux_ctrl_t *aux_in;
    control_signals_t signals = {};

    if((aux_in = aux_ctrl_in_get(port))) {
        switch(aux_in->function) {
#ifdef I2C_STROBE_PIN
            case Input_I2CStrobe:
                if(i2c_strobe.callback)
                    i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
                break;
#endif
#ifdef MPG_MODE_PIN
            case Input_MPGSelect:
                task_add_immediate(mpg_select, NULL);
                break;
#endif
            default:
                break;
        }
        signals.mask |= aux_in->signal.mask;
        if(aux_in->irq_mode == IRQ_Mode_Change)
            signals.deasserted = hal.port.wait_on_input(Port_Digital, aux_in->port, WaitMode_Immediate, 0.0f) == 0;
    }

    if(signals.mask) {
        if(!signals.deasserted)
            signals.mask |= systemGetState().mask;
        hal.control.interrupt_callback(signals);
    }
}


// -----------------------------------------------------------------------------
// Stepper callbacks
// -----------------------------------------------------------------------------

static void stepperEnable (axes_signals_t enable, bool hold)
{
    (void)hold;

#ifdef STEPPERS_ENABLE_PORT
    // CNC_ED1 V1.1 uses a single active-low shared enable pin (PB8) for all axes.
    // Drive the pin directly from the logical enable request: low when any current
    // axis is requested enabled, high when none are. This makes the shared enable
    // independent of $4/enable_invert settings, which is appropriate for fixed-
    // polarity board-level enable wiring.
    DIGITAL_OUT(STEPPERS_ENABLE_PORT, STEPPERS_ENABLE_PIN, (enable.mask & AXES_BITMASK) ? 0 : 1);
#else
    enable.mask ^= settings.steppers.enable_invert.mask;

    for (uint_fast8_t i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        if (outputpin[i].group == PinGroup_StepperEnable) {
            uint32_t axis_bit = 1U << (outputpin[i].id - Output_StepperEnableX);
            DIGITAL_OUT(outputpin[i].port, outputpin[i].pin, (enable.mask & axis_bit) ? 1 : 0);
        }
    }
#endif
}

static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK}, false);

    timer_interrupt_disable(STEPPER_TIMER, TIMER_INT_UP);
    timer_disable(STEPPER_TIMER);
    timer_counter_value_config(STEPPER_TIMER, 0);
    timer_autoreload_value_config(STEPPER_TIMER, hal.f_step_timer / 500); // ~2ms wake-up delay
    timer_event_software_generate(STEPPER_TIMER, TIMER_EVENT_SRC_UPG);
    timer_interrupt_flag_clear(STEPPER_TIMER, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(STEPPER_TIMER, TIMER_INT_UP);
    timer_enable(STEPPER_TIMER);
}

// Set stepper step output pins
static inline __attribute__((always_inline)) void stepper_step_out (axes_signals_t step_out)
{
    uint32_t step = step_out.mask ^ settings .steppers.step_invert.mask;

    DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, (step & X_AXIS_BIT) ? 1 : 0);
    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, (step & Y_AXIS_BIT) ? 1 : 0);
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, (step & Z_AXIS_BIT) ? 1 : 0);
#ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_PIN, (step & A_AXIS_BIT) ? 1 : 0);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_PIN, (step & B_AXIS_BIT) ? 1 : 0);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PORT, C_STEP_PIN, (step & C_AXIS_BIT) ? 1 : 0);
#endif
#ifdef U_AXIS
    DIGITAL_OUT(U_STEP_PORT, U_STEP_PIN, (step & U_AXIS_BIT) ? 1 : 0);
#endif
#ifdef V_AXIS
    DIGITAL_OUT(V_STEP_PORT, V_STEP_PIN, (step & V_AXIS_BIT) ? 1 : 0);
#endif
#ifdef W_AXIS
    DIGITAL_OUT(W_STEP_PORT, W_STEP_PIN, (step & W_AXIS_BIT) ? 1 : 0);
#endif
}

// Set stepper direction output pins
static inline __attribute__((always_inline)) void stepper_dir_out (axes_signals_t dir_out)
{
    uint32_t dir = dir_out.mask ^ settings.steppers.dir_invert.mask;

    DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_PIN, (dir & X_AXIS_BIT) ? 1 : 0);
    DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_PIN, (dir & Y_AXIS_BIT) ? 1 : 0);
    DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_PIN, (dir & Z_AXIS_BIT) ? 1 : 0);
#ifdef A_AXIS
    DIGITAL_OUT(A_DIRECTION_PORT, A_DIRECTION_PIN, (dir & A_AXIS_BIT) ? 1 : 0);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_DIRECTION_PORT, B_DIRECTION_PIN, (dir & B_AXIS_BIT) ? 1 : 0);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_DIRECTION_PORT, C_DIRECTION_PIN, (dir & C_AXIS_BIT) ? 1 : 0);
#endif
#ifdef U_AXIS
    DIGITAL_OUT(U_DIRECTION_PORT, U_DIRECTION_PIN, (dir & U_AXIS_BIT) ? 1 : 0);
#endif
#ifdef V_AXIS
    DIGITAL_OUT(V_DIRECTION_PORT, V_DIRECTION_PIN, (dir & V_AXIS_BIT) ? 1 : 0);
#endif
#ifdef W_AXIS
    DIGITAL_OUT(W_DIRECTION_PORT, W_DIRECTION_PIN, (dir & W_AXIS_BIT) ? 1 : 0);
#endif
}

static void stepperGoIdle (bool clear_signals)
{
    timer_interrupt_disable(STEPPER_TIMER, TIMER_INT_UP);
    if (clear_signals) {
        stepper_dir_out((axes_signals_t){0});
        stepper_step_out((axes_signals_t){0});
    }
}

ISR_CODE static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    // TIMER3 is 16-bit: clamp to 0xFFFF instead of the STM32 32-bit reference's 0x000FFFFF.
    STEPPER_CAR = cycles_per_tick < 0x10000UL ? max(cycles_per_tick, step_pulse.t_min_period) : 0xFFFFUL;
}

ISR_CODE static inline __attribute__((always_inline)) void _stepper_step_out (axes_signals_t step_out)
{
    stepper_step_out(step_out);

    if (stepper_ifg(TIMER_INT_FLAG_UP) || STEPPER_CNT < step_pulse.t_on_off_min) {
        STEPPER_CNT = step_pulse.t_on_off_min;
        NVIC_ClearPendingIRQ(timerINT(STEPPER_TIMER_N));
    }

    uint32_t cnt = STEPPER_CNT;
    STEPPER_CH1CV = cnt - step_pulse.t_off;
    STEPPER_INTF = 0;
    stepper_ie_set(TIMER_INT_CH1);
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void stepperPulseStart (stepper_t *stepper)
{
    if (stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        stepper_dir_out(stepper->dir_out);
    }

    if (stepper->step_out.bits)
        _stepper_step_out(stepper->step_out);
}

// Start a stepper pulse, delay version.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if (stepper->dir_changed.bits) {

        stepper_dir_out(stepper->dir_out);

        if (stepper->step_out.bits) {

            if (stepper->step_out.bits & stepper->dir_changed.bits) {

                step_pulse.out = stepper->step_out; // Store out_bits

                if (STEPPER_CNT < step_pulse.t_dly_off_min) {
                    STEPPER_CNT = step_pulse.t_dly_off_min;
                    NVIC_ClearPendingIRQ(timerINT(STEPPER_TIMER_N));
                }

                uint32_t cnt = STEPPER_CNT;
                STEPPER_CH2CV = cnt - step_pulse.t_on;
                STEPPER_INTF = 0;
                stepper_ie_set(TIMER_INT_CH2);
            } else
                _stepper_step_out(stepper->step_out);
        }

        stepper->dir_changed.bits = 0;
        return;
    }

    if (stepper->step_out.bits)
        _stepper_step_out(stepper->step_out);
}


static uint32_t get_free_mem (void)
{
    extern uint8_t _end; /* Symbol defined in the linker script */
    extern uint8_t _estack; /* Symbol defined in the linker script */
    extern uint32_t _Min_Stack_Size; /* Symbol defined in the linker script */
    const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;

    return stack_limit - (uint32_t)&_end - mallinfo().uordblks;
}

// Stepper timer interrupt handler (three-phase: delayed pulse, pulse off, segment boundary)
ISR_CODE void TIMER3_IRQHandler (void)
{
    // Delayed step pulse handler
    if (stepper_ifg(TIMER_INT_FLAG_CH2)) {
        stepper_ie_clear(TIMER_INT_CH2);
        _stepper_step_out(step_pulse.out);
    }
    // Step pulse off handler
    else if (stepper_ifg(TIMER_INT_FLAG_CH1)) {
        stepper_ie_clear(TIMER_INT_CH1);
        stepper_step_out((axes_signals_t){0});
        if (stepper_ifg(TIMER_INT_FLAG_UP) || STEPPER_CNT < step_pulse.t_off_min) {
            STEPPER_CNT = step_pulse.t_off_min;
        }
        stepper_ifg_clear(TIMER_INT_FLAG_CH1);
    }
    // Stepper timeout handler
    else if (stepper_ifg(TIMER_INT_FLAG_UP)) {
        stepper_ifg_clear(TIMER_INT_FLAG_UP);
        if (hal.stepper.interrupt_callback)
            hal.stepper.interrupt_callback();
    }
}

// -----------------------------------------------------------------------------
// Limit callbacks
// -----------------------------------------------------------------------------

static limit_signals_t limitsGetState (void)
{
    limit_signals_t signals = {0};

    signals.min.mask = settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
    signals.min2.mask = settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
    signals.max.mask = settings.limits.invert.mask;
#endif

    signals.min.x = DIGITAL_IN(X_LIMIT_PORT, X_LIMIT_PIN) == 0;
    signals.min.y = DIGITAL_IN(Y_LIMIT_PORT, Y_LIMIT_PIN) == 0;
    signals.min.z = DIGITAL_IN(Z_LIMIT_PORT, Z_LIMIT_PIN) == 0;
#ifdef A_LIMIT_PIN
    signals.min.a = DIGITAL_IN(A_LIMIT_PORT, A_LIMIT_PIN) == 0;
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = DIGITAL_IN(B_LIMIT_PORT, B_LIMIT_PIN) == 0;
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = DIGITAL_IN(C_LIMIT_PORT, C_LIMIT_PIN) == 0;
#endif
#ifdef U_LIMIT_PIN
    signals.min.u = DIGITAL_IN(U_LIMIT_PORT, U_LIMIT_PIN) == 0;
#endif
#ifdef V_LIMIT_PIN
    signals.min.v = DIGITAL_IN(V_LIMIT_PORT, V_LIMIT_PIN) == 0;
#endif
#ifdef W_LIMIT_PIN
    signals.min.w = DIGITAL_IN(W_LIMIT_PORT, W_LIMIT_PIN) == 0;
#endif

#ifdef X2_LIMIT_PIN
    signals.min2.x = DIGITAL_IN(X2_LIMIT_PORT, X2_LIMIT_PIN) == 0;
#endif
#ifdef Y2_LIMIT_PIN
    signals.min2.y = DIGITAL_IN(Y2_LIMIT_PORT, Y2_LIMIT_PIN) == 0;
#endif
#ifdef Z2_LIMIT_PIN
    signals.min2.z = DIGITAL_IN(Z2_LIMIT_PORT, Z2_LIMIT_PIN) == 0;
#endif

#ifdef X_LIMIT_PIN_MAX
    signals.max.x = DIGITAL_IN(X_LIMIT_PORT_MAX, X_LIMIT_PIN_MAX) == 0;
#endif
#ifdef Y_LIMIT_PIN_MAX
    signals.max.y = DIGITAL_IN(Y_LIMIT_PORT_MAX, Y_LIMIT_PIN_MAX) == 0;
#endif
#ifdef Z_LIMIT_PIN_MAX
    signals.max.z = DIGITAL_IN(Z_LIMIT_PORT_MAX, Z_LIMIT_PIN_MAX) == 0;
#endif
#ifdef A_LIMIT_PIN_MAX
    signals.max.a = DIGITAL_IN(A_LIMIT_PORT_MAX, A_LIMIT_PIN_MAX) == 0;
#endif
#ifdef B_LIMIT_PIN_MAX
    signals.max.b = DIGITAL_IN(B_LIMIT_PORT_MAX, B_LIMIT_PIN_MAX) == 0;
#endif
#ifdef C_LIMIT_PIN_MAX
    signals.max.c = DIGITAL_IN(C_LIMIT_PORT_MAX, C_LIMIT_PIN_MAX) == 0;
#endif
#ifdef U_LIMIT_PIN_MAX
    signals.max.u = DIGITAL_IN(U_LIMIT_PORT_MAX, U_LIMIT_PIN_MAX) == 0;
#endif
#ifdef V_LIMIT_PIN_MAX
    signals.max.v = DIGITAL_IN(V_LIMIT_PORT_MAX, V_LIMIT_PIN_MAX) == 0;
#endif
#ifdef W_LIMIT_PIN_MAX
    signals.max.w = DIGITAL_IN(W_LIMIT_PORT_MAX, W_LIMIT_PIN_MAX) == 0;
#endif

    if(settings.limits.invert.mask) {
        signals.min.value ^= settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
        signals.min2.mask ^= settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
        signals.max.value ^= settings.limits.invert.mask;
#endif
    }

    return signals;
}

static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    bool disable = !on;
    axes_signals_t pin;
    input_signal_t *limit;
    uint_fast8_t idx = limit_inputs.n_pins;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    do {
        limit = &limit_inputs.pins.inputs[--idx];
        if(on && homing_cycle.mask) {
            pin = xbar_fn_to_axismask(limit->id);
            disable = limit->group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
        }
        gpio_irq_enable(limit, disable ? IRQ_Mode_None : limit->mode.irq_mode);
    } while(idx);
}

// -----------------------------------------------------------------------------
// Coolant callbacks
// -----------------------------------------------------------------------------

static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant.invert.mask;
#ifdef COOLANT_FLOOD_PIN
    DIGITAL_OUT(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, mode.flood);
#endif
#ifdef COOLANT_MIST_PIN
    DIGITAL_OUT(COOLANT_MIST_PORT, COOLANT_MIST_PIN, mode.mist);
#endif
}

static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = { .mask = settings.coolant.invert.mask };

#ifdef COOLANT_FLOOD_PIN
    state.flood = DIGITAL_IN(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN);
#endif
#ifdef COOLANT_MIST_PIN
    state.mist = DIGITAL_IN(COOLANT_MIST_PORT, COOLANT_MIST_PIN);
#endif
    state.value ^= settings.coolant.invert.mask;

    return state;
}

// -----------------------------------------------------------------------------
// Spindle callbacks
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Probe callback
// -----------------------------------------------------------------------------

#if DRIVER_PROBES

// Returns the probe triggered pin state.
static bool probeGetState (void *input)
{
    return DIGITAL_IN(((input_signal_t *)input)->port, ((input_signal_t *)input)->pin);
}

#endif // DRIVER_PROBES

// -----------------------------------------------------------------------------
// Peripheral pin registration
// -----------------------------------------------------------------------------

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while(last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
}

static char *port2char (gpio_port_t port)
{
    static char name[3] = "P?";

    uint8_t index = 0;
    if (port == GPIOA) index = 0;
    else if (port == GPIOB) index = 1;
    else if (port == GPIOC) index = 2;
    else if (port == GPIOD) index = 3;
    else if (port == GPIOE) index = 4;
    else if (port == GPIOF) index = 5;
    else if (port == GPIOG) index = 6;
    else if (port == GPIOH) index = 7;
    else if (port == GPIOI) index = 8;

    name[1] = 'A' + index;
    return name;
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {0};

    uint8_t i, id = 0;

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.id = id++;
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.port = low_level ? (void *)inputpin[i].port : (void *)port2char(inputpin[i].port);
        pin.description = inputpin[i].description;
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;

        pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.id = id++;
        pin.pin = outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = low_level ? (void *)outputpin[i].port : (void *)port2char(outputpin[i].port);
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.id = id++;
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.port = low_level ? ppin->pin.port : (void *)port2char((gpio_port_t)(uintptr_t)ppin->pin.port);
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description == NULL ? xbar_group_to_description(ppin->pin.group) : ppin->pin.description;

        pin_info(&pin, data);
    } while((ppin = ppin->next));
}

// -----------------------------------------------------------------------------
// Settings changed handler
// -----------------------------------------------------------------------------

void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
#if USE_STEPDIR_MAP
    stepdirmap_init(settings);
#endif

    if(IOInitDone) {

        hal.stepper.go_idle(true);

#ifdef SQUARING_ENABLED
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif

        float sl = (float)hal.f_step_timer / 1000000.0f;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            step_pulse.t_on = (uint32_t)ceilf(sl * (max(STEP_PULSE_TOFF_MIN, settings->steppers.pulse_delay_microseconds) - STEP_PULSE_TOFF_LATENCY));
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else {
            step_pulse.t_on = 0;
            hal.stepper.pulse_start = stepperPulseStart;
        }

        step_pulse.t_min_period = (uint32_t)ceilf(sl * (settings->steppers.pulse_microseconds + STEP_PULSE_TOFF_MIN));
        step_pulse.t_off = (uint32_t)ceilf(sl * (settings->steppers.pulse_microseconds - STEP_PULSE_TOFF_LATENCY));
        step_pulse.t_off_min = (uint32_t)ceilf(sl * (STEP_PULSE_TOFF_MIN - STEP_PULSE_TON_LATENCY));
        step_pulse.t_on_off_min = step_pulse.t_off + step_pulse.t_off_min;
        step_pulse.t_dly_off_min = step_pulse.t_on + step_pulse.t_on_off_min;

#if STEP_INJECT_ENABLE

        timer_cfg_t step_inject_cfg = {
            .single_shot = On,
            .timeout_callback = step_inject_off
        };
        step_inject_cfg.irq0_callback = step_pulse.delay ? step_inject_on : NULL;
        step_inject_cfg.irq0 = step_pulse.delay;

        step_pulse.length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            step_pulse.delay = (uint32_t)(10.0f * settings->steppers.pulse_delay_microseconds) - 1;
            if(step_pulse.delay > (uint32_t)(10.0f * STEP_PULSE_LATENCY))
                step_pulse.delay = max(10, step_pulse.delay - (uint32_t)(10.0f * STEP_PULSE_LATENCY));
        } else
            step_pulse.delay = 0;

        hal.timer.configure(step_pulse.inject.timer, &step_inject_cfg);

#endif

        /*************************
         *  Control pins config  *
         *************************/

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<0)
        nvic_irq_disable(EXTI0_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<1)
        nvic_irq_disable(EXTI1_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<2)
        nvic_irq_disable(EXTI2_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<3)
        nvic_irq_disable(EXTI3_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<4)
        nvic_irq_disable(EXTI4_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & 0x03E0
        nvic_irq_disable(EXTI5_9_IRQn);
#endif
#if (DRIVER_IRQMASK|AUXINPUT_MASK) & 0xFC00
        nvic_irq_disable(EXTI10_15_IRQn);
#endif

        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
        input_signal_t *input;

        do {

            input = &inputpin[--i];

            if(input->group == PinGroup_AuxInputAnalog)
                continue;

            if(input->group != PinGroup_AuxInput)
                input->mode.irq_mode = IRQ_Mode_None;

            switch(input->id) {

                case Input_LimitX:
                case Input_LimitX_2:
                case Input_LimitX_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.x ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;

                case Input_LimitY:
                case Input_LimitY_2:
                case Input_LimitY_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.y ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_2:
                case Input_LimitZ_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.z ? PullMode_None : PullMode_Up;
#ifdef Z_LIMIT_POLL
                    if(input->id != Input_LimitZ)
#endif
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;
#ifdef A_AXIS
                case Input_LimitA:
                case Input_LimitA_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.a ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;
#endif
#ifdef B_AXIS
                case Input_LimitB:
                case Input_LimitB_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.b ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;
#endif
#ifdef C_AXIS
                case Input_LimitC:
                case Input_LimitC_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.c ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;
#endif
#ifdef U_AXIS
                case Input_LimitU:
                case Input_LimitU_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.u ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;
#endif
#ifdef V_AXIS
                case Input_LimitV:
                case Input_LimitV_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.v ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;
#endif
#ifdef W_AXIS
                case Input_LimitW:
                case Input_LimitW_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.w ? PullMode_None : PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;
#endif
#if HOME_MASK
                case Input_HomeX:
                case Input_HomeX_2:
                    input->mode.pull_mode = PullMode_Up; // settings->limits.disable_pullup.x ? PullMode_None : PullMode_Up;
                    break;

                case Input_HomeY:
                case Input_HomeY_2:
                    input->mode.pull_mode = PullMode_Up; // settings->limits.disable_pullup.y ? PullMode_None : PullMode_Up;
                    break;

                case Input_HomeZ:
                case Input_HomeZ_2:
                    input->mode.pull_mode = PullMode_Up; // settings->limits.disable_pullup.z ? PullMode_None : PullMode_Up;
                    break;
#ifdef A_AXIS
                case Input_HomeA:
                    input->mode.pull_mode = PullMode_Up; // settings->limits.disable_pullup.a ? PullMode_None : PullMode_Up;
                    break;
#endif
#ifdef B_AXIS
                case Input_HomeB:
                    input->mode.pull_mode = PullMode_Up; // settings->limits.disable_pullup.b ? PullMode_None : PullMode_Up;
                    break;
#endif
#ifdef C_AXIS
                case Input_HomeC:
                    input->mode.pull_mode = PullMode_Up; // settings->limits.disable_pullup.c ? PullMode_None : PullMode_Up;
                    break;
#endif
#ifdef U_AXIS
                case Input_HomeU:
                    input->mode.pull_mode = PullMode_Up; // settings->limits.disable_pullup.u ? PullMode_None : PullMode_Up;
                    break;
#endif
#ifdef V_AXIS
                case Input_HomeV:
                    input->mode.pull_mode = PullMode_Up; // settings->limits.disable_pullup.v ? PullMode_None : PullMode_Up;
                    break;
#endif
#ifdef W_AXIS
                case Input_HomeW:
                    input->mode.pull_mode = PullMode_Up; // settings->limits.disable_pullup.w ? PullMode_None : PullMode_Up;
                    break;
#endif
#endif // HOME_MASK
                case Input_SPIIRQ:
                    input->mode.pull_mode = true;
                    input->mode.irq_mode = IRQ_Mode_Falling;
                    break;

                case Input_SpindleIndex:
                    input->mode.pull_mode = true;
                    input->mode.irq_mode = IRQ_Mode_Falling;
                    break;

#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
                case Input_SdCardDetect:
                    input->mode.pull_mode = PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    input->mode.debounce = On;
                    break;
#endif

                default:
                    break;
            }

            if(input->group == PinGroup_AuxInput) {
                if(input->cap.irq_mode != IRQ_Mode_None) {
                    // Map interrupt to pin
                    syscfg_exti_line_config((uint8_t)(EXTI_SOURCE_GPIOA + GPIO_GET_INDEX(input->port)), input->pin);
                }
            }

            if(input->group == PinGroup_Motor_Fault)
                input->mode.inverted = bit_istrue(settings->motor_fault_invert.mask, bit(xbar_fault_pin_to_axis(input->id)));

            {
                uint8_t pupd = GPIO_PUPD_NONE;

                if(input->mode.pull_mode == PullMode_Up)
                    pupd = GPIO_PUPD_PULLUP;
                else if(input->mode.pull_mode == PullMode_Down)
                    pupd = GPIO_PUPD_PULLDOWN;

                gpio_mode_set(input->port, GPIO_MODE_INPUT, pupd, input->bit);
            }

            {
                exti_line_enum line = (exti_line_enum)input->bit;

                exti_interrupt_disable(line);

                if(input->mode.irq_mode != IRQ_Mode_None) {

                    exti_trig_type_enum trig;
                    switch(input->mode.irq_mode) {
                        case IRQ_Mode_Rising:
                            trig = EXTI_TRIG_RISING;
                            break;
                        case IRQ_Mode_Falling:
                            trig = EXTI_TRIG_FALLING;
                            break;
                        default:
                            trig = EXTI_TRIG_BOTH;
                            break;
                    }

                    exti_init(line, EXTI_INTERRUPT, trig);
                    exti_interrupt_flag_clear(line);
                    exti_interrupt_enable(line);
                }
            }

        } while(i);

        uint32_t irq_mask = DRIVER_IRQMASK|aux_irq;

        for(uint32_t bit = 0; bit < 16; bit++) {
            if(irq_mask & (1U << bit))
                exti_interrupt_flag_clear((exti_line_enum)(1U << bit));
        }

        if(irq_mask & (1<<0)) {
            nvic_irq_enable(EXTI0_IRQn, 2U, 0U);
        }
        if(irq_mask & (1<<1)) {
            nvic_irq_enable(EXTI1_IRQn, 2U, 0U);
        }
        if(irq_mask & (1<<2)) {
            nvic_irq_enable(EXTI2_IRQn, 2U, 0U);
        }
        if(irq_mask & (1<<3)) {
            nvic_irq_enable(EXTI3_IRQn, 2U, 0U);
        }
        if(irq_mask & (1<<4)) {
            nvic_irq_enable(EXTI4_IRQn, 2U, 0U);
        }
        if(irq_mask & 0x03E0) {
            nvic_irq_enable(EXTI5_9_IRQn, 2U, 0U);
        }
        if(irq_mask & 0xFC00) {
            nvic_irq_enable(EXTI10_15_IRQn, 2U, 0U);
        }

        hal.limits.enable(settings->limits.flags.hard_enabled, (axes_signals_t){0});
        aux_ctrl_irq_enable(settings, aux_irq_handler);
    }
}

// -----------------------------------------------------------------------------
// Control signals state
// -----------------------------------------------------------------------------

static control_signals_t systemGetState (void)
{
    control_signals_t signals = { settings.control_invert.mask };

#if defined(RESET_PIN) && !ESTOP_ENABLE
    signals.reset = DIGITAL_IN(RESET_PORT, RESET_PIN);
#endif
#if defined(RESET_PIN) && ESTOP_ENABLE
    signals.e_stop = DIGITAL_IN(RESET_PORT, RESET_PIN);
#endif
#ifdef FEED_HOLD_PIN
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_PORT, FEED_HOLD_PIN);
#endif
#ifdef CYCLE_START_PIN
    signals.cycle_start = DIGITAL_IN(CYCLE_START_PORT, CYCLE_START_PIN);
#endif
#ifdef SAFETY_DOOR_PIN
    if(debounce.safety_door)
        signals.safety_door_ajar = !settings.control_invert.safety_door_ajar;
    else
        signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, SAFETY_DOOR_PIN);
#endif
#ifdef MOTOR_FAULT_PIN
    signals.motor_fault = DIGITAL_IN(MOTOR_FAULT_PORT, MOTOR_FAULT_PIN);
#endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return aux_ctrl_scan_status(signals);
}

// -----------------------------------------------------------------------------
// Aux input/output claim helpers
// -----------------------------------------------------------------------------

__attribute__((weak)) void motor_fault_add_pin (input_signal_t *input, xbar_t *pin)
{
    // NOOP
}

#ifdef USE_EXPANDERS
__attribute__((weak)) bool input_add_expander_pin (xbar_t *pin)
{
    return false;
}
#endif

static aux_ctrl_t *aux_ctrl_get_fn (aux_gpio_t gpio)
{
    aux_ctrl_t *ctrl_pin = NULL;

    if(sizeof(aux_ctrl) / sizeof(aux_ctrl_t)) {
        uint_fast8_t idx;
        for(idx = 0; ctrl_pin == NULL && aux_ctrl[idx].gpio.pin != 0xFF && idx < sizeof(aux_ctrl) / sizeof(aux_ctrl_t); idx++) {
            if(aux_ctrl[idx].gpio.pin == gpio.pin && aux_ctrl[idx].gpio.port == gpio.port)
                ctrl_pin = &aux_ctrl[idx];
        }
    }

    return ctrl_pin;
}

static void aux_assign_irq (void)
{
    uint32_t i, j, irq = 0;
    input_signal_t *input, *input2;
    aux_ctrl_t *aux;
    pin_group_pins_t aux_digital_in = {};

    const control_signals_t main_signals = { .reset = On, .e_stop = On, .feed_hold = On, .cycle_start = On };

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {

        input = &inputpin[i];

        if(input->group == PinGroup_AuxInput) {

            input->bit = 1 << input->pin;

            if(aux_digital_in.pins.inputs == NULL)
                aux_digital_in.pins.inputs = input;

            input->user_port = aux_digital_in.n_pins++;
            input->id = (pin_function_t)(Input_Aux0 + input->user_port);
            input->mode.pull_mode = PullMode_Up;
            input->cap.pull_mode = PullMode_UpDown;
            input->cap.irq_mode = (DRIVER_IRQMASK & input->bit) ? IRQ_Mode_None : IRQ_Mode_Edges;

            aux = aux_ctrl_get_fn((aux_gpio_t){ .port = (void *)input->port, .pin = input->pin });

            if(input->cap.irq_mode == IRQ_Mode_None) {
                if(aux && (xbar_is_probe_in(aux->function) || xbar_is_motor_fault_in(aux->function)))
                    input->id = aux->function;
            } else {

                if(aux)
                    input->id = aux->function;

                if(irq & input->bit) { // duplicate IRQ

                    if(aux == NULL || xbar_is_motor_fault_in(aux->function))
                        input->cap.irq_mode = IRQ_Mode_None;
                    else for(j = 0; j < aux_digital_in.n_pins - 1; j++) {
                        input2 = &aux_digital_in.pins.inputs[j];
                        if(input->pin == input2->pin) {
                            if(input->id < input2->id || (aux->signal.bits & main_signals.bits)) {
                                input2->cap.irq_mode = IRQ_Mode_None;
                                if(!(xbar_is_probe_in(input2->id)))
                                    input2->id = (pin_function_t)(Input_Aux0 + input2->user_port);
                            } else {
                                input->cap.irq_mode = IRQ_Mode_None;
                                if(!(xbar_is_probe_in(input->id)))
                                    input->id = (pin_function_t)(Input_Aux0 + input->user_port);
                            }
                        }
                    }
                } else
                    irq |= input->bit;
            }
        }
    }
}

static bool aux_claim_explicit (aux_ctrl_t *aux_ctrl)
{
    xbar_t *pin;

#ifdef USE_EXPANDERS
    if(aux_ctrl->gpio.port == (void *)EXPANDER_PORT)
        return input_add_expander_pin((xbar_t *)aux_ctrl->input);
#endif

    if(aux_ctrl->input == NULL) {

        uint_fast8_t i = sizeof(inputpin) / sizeof(input_signal_t);

        do {
            --i;
            if(inputpin[i].group == PinGroup_AuxInput && inputpin[i].user_port == aux_ctrl->port)
                aux_ctrl->input = &inputpin[i];
        } while(i && aux_ctrl->input == NULL);
    }

    if((pin = aux_ctrl_claim_port(aux_ctrl))) {

        if(xbar_is_motor_fault_in(aux_ctrl->function))
            motor_fault_add_pin(aux_ctrl->input, pin);

        else switch(aux_ctrl->function) {
#if PROBE_ENABLE
            case Input_Probe:
                hal.driver_cap.probe = probe_add(Probe_Default, aux_ctrl->port, pin->cap.irq_mode, aux_ctrl->input, probeGetState);
                break;
#endif
#if PROBE2_ENABLE
            case Input_Probe2:
                hal.driver_cap.probe2 = probe_add(Probe_2, aux_ctrl->port, pin->cap.irq_mode, aux_ctrl->input, probeGetState);
                break;
#endif
#if TOOLSETTER_ENABLE
            case Input_Toolsetter:
                hal.driver_cap.toolsetter = probe_add(Probe_Toolsetter, aux_ctrl->port, pin->cap.irq_mode, aux_ctrl->input, probeGetState);
                break;
#endif
#if SAFETY_DOOR_ENABLE || (defined(RESET_PIN) && !ESTOP_ENABLE)
  #if defined(RESET_PIN) && !ESTOP_ENABLE
            case Input_Reset:
  #endif
  #if SAFETY_DOOR_ENABLE
            case Input_SafetyDoor:
  #endif
                ((input_signal_t *)aux_ctrl->input)->mode.debounce = ((input_signal_t *)aux_ctrl->input)->cap.debounce && hal.driver_cap.software_debounce;
                break;
#endif
            default:
#if ENCODER_ENABLE
                encoder_pin_claimed(aux_ctrl->port, pin);
#endif
                break;
        }
    }

    return aux_ctrl->port != IOPORT_UNASSIGNED;
}

static uint64_t getElapsedMicros (void)
{
    uint32_t ms = hal_get_tick();
    uint32_t cyc = DWT->CYCCNT;
    uint32_t cyc_per_ms = SystemCoreClock / 1000UL;
    uint32_t cyc_per_us = SystemCoreClock / 1000000UL;
    uint32_t frac = cyc % cyc_per_ms;

    return (uint64_t)ms * 1000ULL + (uint64_t)(frac / cyc_per_us);
}

#if STEP_INJECT_ENABLE

static inline __attribute__((always_inline)) void inject_step (axes_signals_t step_out, axes_signals_t axes)
{
    uint_fast8_t idx = N_AXIS - 1;

    if(!step_out.bits)
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits;

    step_out.bits ^= settings.steppers.step_invert.bits;

    do {
        if(axes.bits & (1 << (N_AXIS - 1))) {

            switch(idx) {

                case X_AXIS:
                    DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, step_out.x);
#ifdef X2_STEP_PIN
                    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_PIN, step_out.x);
#endif
                    break;

                case Y_AXIS:
                    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, step_out.y);
#ifdef Y2_STEP_PIN
                    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_PIN, step_out.y);
#endif
                    break;

                case Z_AXIS:
                    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, step_out.z);
#ifdef Z2_STEP_PIN
                    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_PIN, step_out.z);
#endif
                    break;
#ifdef A_AXIS
                case A_AXIS:
                    DIGITAL_OUT(A_STEP_PORT, A_STEP_PIN, step_out.a);
                    break;
#endif
#ifdef B_AXIS
                case B_AXIS:
                    DIGITAL_OUT(B_STEP_PORT, B_STEP_PIN, step_out.b);
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
                    DIGITAL_OUT(C_STEP_PORT, C_STEP_PIN, step_out.c);
                    break;
#endif
#ifdef U_AXIS
                case U_AXIS:
                    DIGITAL_OUT(U_STEP_PORT, U_STEP_PIN, step_out.u);
                    break;
#endif
#ifdef V_AXIS
                case V_AXIS:
                    DIGITAL_OUT(V_STEP_PORT, V_STEP_PIN, step_out.v);
                    break;
#endif
#ifdef W_AXIS
                case W_AXIS:
                    DIGITAL_OUT(W_STEP_PORT, W_STEP_PIN, step_out.w);
                    break;
#endif
            }
        }
        idx--;
        axes.bits <<= 1;
    } while(axes.bits & AXES_BITMASK);
}

static void stepperClaimMotor (uint_fast8_t axis_id, bool claim)
{
    if(claim)
        step_pulse.inject.claimed.mask |= ((1 << axis_id) & AXES_BITMASK);
    else {
        step_pulse.inject.claimed.mask &= ~(1 << axis_id);
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits;
    }
}

ISR_CODE void stepperOutputStep (axes_signals_t step_out, axes_signals_t dir_out)
{
    if(step_out.bits) {

        uint_fast8_t idx = N_AXIS - 1;
        axes_signals_t axes = { .bits = (step_out.bits & AXES_BITMASK) };

        step_pulse.inject.out = step_out;
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits | step_out.bits;
        dir_out.bits ^= settings.steppers.dir_invert.bits;

        do {
            if(axes.bits & (1 << (N_AXIS - 1))) {

                switch(idx) {

                    case X_AXIS:
                        DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_PIN, dir_out.x);
#ifdef X2_DIRECTION_PIN
                        DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, dir_out.x ^ settings.steppers.ganged_dir_invert.x);
#endif
                        break;

                    case Y_AXIS:
                        DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_PIN, dir_out.y);
#ifdef Y2_DIRECTION_PIN
                        DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, dir_out.y ^ settings.steppers.ganged_dir_invert.y);
#endif
                        break;

                    case Z_AXIS:
                        DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_PIN, dir_out.z);
#ifdef Z2_DIRECTION_PIN
                        DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, dir_out.z ^ settings.steppers.ganged_dir_invert.z);
#endif
                        break;
#ifdef A_AXIS
                    case A_AXIS:
                        DIGITAL_OUT(A_DIRECTION_PORT, A_DIRECTION_PIN, dir_out.a);
                        break;
#endif
#ifdef B_AXIS
                    case B_AXIS:
                        DIGITAL_OUT(B_DIRECTION_PORT, B_DIRECTION_PIN, dir_out.b);
                        break;
#endif
#ifdef C_AXIS
                    case C_AXIS:
                        DIGITAL_OUT(C_DIRECTION_PORT, C_DIRECTION_PIN, dir_out.c);
                        break;
#endif
#ifdef U_AXIS
                    case U_AXIS:
                        DIGITAL_OUT(U_DIRECTION_PORT, U_DIRECTION_PIN, dir_out.u);
                        break;
#endif
#ifdef V_AXIS
                    case V_AXIS:
                        DIGITAL_OUT(V_DIRECTION_PORT, V_DIRECTION_PIN, dir_out.v);
                        break;
#endif
#ifdef W_AXIS
                    case W_AXIS:
                        DIGITAL_OUT(W_DIRECTION_PORT, W_DIRECTION_PIN, dir_out.w);
                        break;
#endif
                }
            }
            idx--;
            axes.bits <<= 1;
        } while(axes.bits & AXES_BITMASK);

        if(step_pulse.delay == 0)
            inject_step(step_out, step_out);

        hal.timer.start(step_pulse.inject.timer, step_pulse.length);
    }
}

void step_inject_on (void *context)
{
    inject_step(step_pulse.inject.out, step_pulse.inject.out);
}

void step_inject_off (void *context)
{
    axes_signals_t axes = { .bits = step_pulse.inject.out.bits };

    step_pulse.inject.out.bits = 0;
    step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits;

    inject_step((axes_signals_t){0}, axes);
}

#endif // STEP_INJECT_ENABLE

// -----------------------------------------------------------------------------
// Driver setup and init
// -----------------------------------------------------------------------------

static bool driver_setup (settings_t *settings)
{
    rcu_periph_clock_enable(RCU_SYSCFG);

    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL2);

    (void)timer_clk_enable(STEPPER_TIMER);

    /*************************
     *  Output signals init  *
     *************************/

    uint32_t i;
    axes_signals_t st_enable = st_get_enable_out();

    // Switch on stepper driver power before enabling other output pins
    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        if(outputpin[i].group == PinGroup_StepperPower) {
            outputpin[i].bit = 1U << outputpin[i].pin;
            gpio_mode_set(outputpin[i].port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, outputpin[i].bit);
            gpio_output_options_set(outputpin[i].port,
                                    outputpin[i].mode.open_drain ? GPIO_OTYPE_OD : GPIO_OTYPE_PP,
                                    GPIO_OSPEED_200MHZ,
                                    outputpin[i].bit);
            DIGITAL_OUT(outputpin[i].port, outputpin[i].pin, 1);
        }
    }

    hal.delay_ms(100, NULL);

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        if(!(outputpin[i].group == PinGroup_StepperPower ||
              outputpin[i].group == PinGroup_AuxOutputAnalog ||
               outputpin[i].id == Output_SpindlePWM ||
                outputpin[i].id == Output_Spindle1PWM)) {

            outputpin[i].bit = 1U << outputpin[i].pin;

            if(outputpin[i].group == PinGroup_MotorChipSelect ||
                outputpin[i].group == PinGroup_MotorUART ||
                 outputpin[i].group == PinGroup_SPICS ||
                  (outputpin[i].group == PinGroup_StepperEnable && (st_enable.mask & xbar_fn_to_axismask(outputpin[i].id).mask)))
                gpio_bit_set(outputpin[i].port, outputpin[i].bit);

            gpio_mode_set(outputpin[i].port,
                          GPIO_MODE_OUTPUT,
                          GPIO_PUPD_NONE,
                          outputpin[i].bit);
            gpio_output_options_set(outputpin[i].port,
                                    outputpin[i].mode.open_drain ? GPIO_OTYPE_OD : GPIO_OTYPE_PP,
                                    GPIO_OSPEED_200MHZ,
                                    outputpin[i].bit);
        }
    }

    // Stepper init
    timer_parameter_struct init;
    timer_struct_para_init(&init);
    timer_deinit(STEPPER_TIMER);
    init.prescaler = STEPPER_TIMER_DIV - 1;
    init.alignedmode = TIMER_COUNTER_EDGE;
    init.counterdirection = TIMER_COUNTER_DOWN;
    init.period = 0U;
    init.clockdivision = TIMER_CKDIV_DIV1;
    init.repetitioncounter = 0;
    timer_init(STEPPER_TIMER, &init);

    // Enable auto-reload shadow so ARR updates are buffered and transferred on update event,
    // matching STM32 reference behavior and avoiding timing glitches.
    TIMER_CTL0(STEPPER_TIMER) |= TIMER_CTL0_ARSE;

    nvic_irq_enable(timerINT(STEPPER_TIMER_N), 0U, 0U);
    timer_interrupt_enable(STEPPER_TIMER, TIMER_INT_UP);
    timer_enable(STEPPER_TIMER);

#if SDCARD_SDIO

    sdcard_events_t *card = sdcard_init();
    card->on_mount = sdcard_mount;
    card->on_unmount = sdcard_unmount;

    sdcard_mount(NULL);

#elif SDCARD_ENABLE

    DIGITAL_OUT(SD_CS_PORT, SD_CS_PIN, 1);

    sdcard_init();

#endif

#if LITTLEFS_ENABLE

#include "sdcard/fs_littlefs.h"
#include "sdcard/macros.h"

    fs_littlefs_mount("/", eeprom_littlefs_hal());
#endif

    IOInitDone = settings->version.id == 23;

    hal.settings_changed(settings, (settings_changed_flags_t){0});

#if SDCARD_ENABLE && defined(SD_DETECT_PIN)
    if(!DIGITAL_IN(SD_DETECT_PORT, SD_DETECT_PIN))
        sdcard_detect(true);
#endif

    return IOInitDone;
}

bool driver_init (void)
{
    hal.info = "GD32F425RET6 grblHAL";
    hal.driver_version = "240716";
    hal.driver_url = GRBL_URL "/GD32F4xx";
    hal.board = BOARD_NAME;
    hal.f_mcu = SystemCoreClock / 1000000UL;
    hal.f_step_timer = timer_clk_enable(STEPPER_TIMER) / STEPPER_TIMER_DIV;
    hal.step_us_min = 2.0f;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.get_free_mem = get_free_mem;
    hal.driver_setup = driver_setup;
    hal.delay_ms = driver_delay;
    hal.settings_changed = settings_changed;

    hal.timer.claim = timerClaim;
    hal.timer.configure = timerCfg;
    hal.timer.start = timerStart;
    hal.timer.stop = timerStop;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
    hal.stepper.motor_iterator = motor_iterator;

    hal.limits.get_state = limitsGetState;
    hal.limits.enable = limitsEnable;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.control.get_state = systemGetState;

    hal.reboot = NVIC_SystemReset;
    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
#if defined(I2C_STROBE_PIN) || SPI_IRQ_BIT
    hal.irq_claim = irq_claim;
#endif
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_micros = getElapsedMicros;
    hal.get_elapsed_ticks = getElapsedTicks;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

#if SPI_ENABLE
    spi_start(NULL);
#endif

    serialRegisterStreams();

#if USB_SERIAL_CDC
    stream_connect(usbInit());
#else
    if(!stream_connect_instance(SERIAL_STREAM, BAUD_RATE))
        while(true); // Cannot boot if no communication channel is available!
#endif

    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();
    hal.motor_fault_cap = get_motor_fault_cap();
#if SPINDLE_ENCODER_ENABLE
    hal.driver_cap.spindle_encoder = On;
#endif
    hal.coolant_cap.bits = COOLANT_ENABLE;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;

#if EEPROM_ENABLE
    if(!i2c_eeprom_init())
        task_run_on_startup(task_raise_alarm, (void *)Alarm_NVS_Failed);
#elif FLASH_ENABLE
    hal.nvs.type = NVS_Flash;
    hal.nvs.size_max = 1024 * 16;
    hal.nvs.memcpy_from_flash = memcpy_from_flash;
    hal.nvs.memcpy_to_flash = memcpy_to_flash;
#else
    hal.nvs.type = NVS_None;
#endif

#if LED_RGB
    hal.rgb0.out = rgb_out;
    hal.rgb0.out_masked = rgb_out_masked;
    hal.rgb0.num_devices = 1;
  #ifdef LED_W_PIN
    hal.rgb0.cap = (rgb_color_t){ .R = 1, .G = 1, .B = 1, .W = 1 };
  #else
    hal.rgb0.cap = (rgb_color_t){ .R = 1, .G = 1, .B = 1 };
  #endif
#endif

    static pin_group_pins_t aux_digital_in = {0}, aux_digital_out = {0},
                            aux_analog_in = {0}, aux_analog_out = {0};

    uint32_t i;
    input_signal_t *input;

    aux_assign_irq();

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {

        input = &inputpin[i];
        input->mode.input = input->cap.input = On;
        input->bit = 1 << input->pin;

        switch(input->group) {

            case PinGroup_AuxInput:
                if(aux_digital_in.pins.inputs == NULL)
                    aux_digital_in.pins.inputs = input;

                aux_digital_in.n_pins++;

                if(!(input->id >= Input_Aux0 && input->id <= Input_AuxMax)) {
                    input->id = Input_Aux0 + input->user_port;
                    aux_ctrl_remap_explicit((aux_gpio_t){ .port = (void *)input->port, .pin = input->pin }, input->user_port, input);
                }

                if((input->cap.debounce = input->cap.irq_mode != IRQ_Mode_None)) {
                    aux_irq |= input->bit;
                    pin_irq[__builtin_ffs(input->bit) - 1] = input;
                }
                break;

            case PinGroup_AuxInputAnalog:
                if(aux_analog_in.pins.inputs == NULL)
                    aux_analog_in.pins.inputs = input;
                input->id = (pin_function_t)(Input_Analog_Aux0 + aux_analog_in.n_pins++);
                input->mode.analog = input->cap.analog = On;
                break;

            case PinGroup_Limit:
            case PinGroup_LimitMax:
                if(limit_inputs.pins.inputs == NULL)
                    limit_inputs.pins.inputs = input;
                if(LIMIT_MASK & input->bit)
                    pin_irq[__builtin_ffs(input->bit) - 1] = input;
#ifdef Z_LIMIT_POLL
                if(input->id == Input_LimitZ)
                    z_limit_pin = input;
#endif
                limit_inputs.n_pins++;
                break;

            case PinGroup_SdCard:
                if(input->bit & DEVICES_IRQ_MASK)
                    pin_irq[__builtin_ffs(input->bit) - 1] = input;
                break;

            default: break;
        }
    }

    output_signal_t *output;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {

        output = &outputpin[i];
        output->mode.output = On;

        switch(output->group) {

            case PinGroup_AuxOutput:
                if(aux_digital_out.pins.outputs == NULL)
                    aux_digital_out.pins.outputs = output;
                output->id = (pin_function_t)(Output_Aux0 + aux_digital_out.n_pins);
                aux_out_remap_explicit((aux_gpio_t){ .port = (void *)output->port, .pin = output->pin }, aux_digital_out.n_pins, output);
                aux_digital_out.n_pins++;
                break;

            case PinGroup_AuxOutputAnalog:
                if(aux_analog_out.pins.outputs == NULL)
                    aux_analog_out.pins.outputs = output;
                output->mode.analog = On;
                output->id = (pin_function_t)(Output_Analog_Aux0 + aux_analog_out.n_pins++);
                break;

            case PinGroup_SpindlePWM:
                aux_out_remap_explicit((aux_gpio_t){ .port = (void *)output->port, .pin = output->pin }, 0, output);
                break;

            default: break;
        }
    }

    if(aux_digital_in.n_pins || aux_digital_out.n_pins)
        ioports_init(&aux_digital_in, &aux_digital_out);

    if(aux_analog_in.n_pins || aux_analog_out.n_pins)
        ioports_init_analog(&aux_analog_in, &aux_analog_out);

    io_expanders_init();
    aux_ctrl_claim_ports(aux_claim_explicit, NULL);

    extern bool aux_out_claim_explicit (aux_ctrl_out_t *aux_ctrl);
    aux_ctrl_claim_out_ports(aux_out_claim_explicit, NULL);

#if DRIVER_SPINDLE_ENABLE || DRIVER_SPINDLE1_ENABLE
    extern void driver_spindles_init (void);
    driver_spindles_init();
#endif

#if STEP_INJECT_ENABLE
    if((step_pulse.inject.timer = hal.timer.claim((timer_cap_t){ .periodic = Off }, 100))) {
        hal.stepper.output_step = stepperOutputStep;
        hal.stepper.claim_motor = stepperClaimMotor;
    }
#endif

#if TRINAMIC_SPI_ENABLE
    extern void tmc_spi_init (void);
    tmc_spi_init();
#elif TRINAMIC_UART_ENABLE
    extern void tmc_uart_init (void);
    tmc_uart_init();
#endif

#if SPINDLE_ENCODER_ENABLE || (QEI_ENABLE && defined(QEI_PORT))
    driver_encoders_init();
#endif

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#include "grbl/plugins_init.h"

#if MPG_ENABLE == 1
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, NULL);
    if(hal.driver_cap.mpg_mode)
        task_run_on_startup(mpg_enable, NULL);
#elif MPG_ENABLE == 2
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, stream_mpg_check_enable);
#endif

    return hal.version == 10;
}

// -----------------------------------------------------------------------------
// Delay / IRQ / atomic helpers
// -----------------------------------------------------------------------------

// Interrupt handler for 1 ms interval timer
void Driver_IncTick (void)
{
#ifdef Z_LIMIT_POLL
    static bool z_limit_state = false;
    if(z_limits_irq_enabled) {
        bool z_limit = DIGITAL_IN(Z_LIMIT_PORT, Z_LIMIT_PIN) ^ settings.limits.invert.z;
        if(z_limit_state != z_limit) {
            if((z_limit_state = z_limit)) {
                if(!(z_limit_pin->mode.debounce && task_add_delayed(core_pin_debounce, z_limit_pin, 40)))
                    hal.limits.interrupt_callback(limitsGetState());
            }
        }
    }
#endif

    if(delay.ms && !(--delay.ms)) {
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}

static uint32_t getElapsedTicks (void)
{
    return hal_get_tick();
}

static void driver_delay (uint32_t ms, delay_callback_ptr callback)
{
    if((delay.ms = ms) > 0) {
        if(!(delay.callback = callback)) {
            while(delay.ms)
                grbl.on_execute_delay(state_get());
        }
    } else {
        delay.callback = NULL;
        if(callback)
            callback();
    }
}

static void bitsSetAtomic (volatile uint_fast16_t *value, uint_fast16_t bits)
{
    __disable_irq();
    *value |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *value, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *value;
    *value &= ~bits;
    __enable_irq();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *value, uint_fast16_t v)
{
    __disable_irq();
    uint_fast16_t prev = *value;
    *value = v;
    __enable_irq();
    return prev;
}

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<0)

void EXTI0_IRQHandler(void)
{
    uint32_t ifg = exti_interrupt_flag_get(EXTI_0) ? (1<<0) : 0;

    if(ifg) {
        exti_interrupt_flag_clear(EXTI_0);
#if (LIMIT_MASK|SD_DETECT_BIT) & (1<<0)
        core_pin_irq(ifg);
#elif SPI_IRQ_BIT & (1<<0)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#elif AUXINPUT_MASK & (1<<0)
        aux_pin_irq(ifg);
#elif SPINDLE_INDEX_BIT & (1<<0)
        spindle_encoder_index_event();
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<1)

ISR_CODE void EXTI1_IRQHandler(void)
{
    uint32_t ifg = exti_interrupt_flag_get(EXTI_1) ? (1<<1) : 0;

    if(ifg) {
        exti_interrupt_flag_clear(EXTI_1);
#if (LIMIT_MASK|SD_DETECT_BIT) & (1<<1)
        core_pin_irq(ifg);
#elif SPI_IRQ_BIT & (1<<1)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#elif AUXINPUT_MASK & (1<<1)
        aux_pin_irq(ifg);
#elif SPINDLE_INDEX_BIT & (1<<1)
        spindle_encoder_index_event();
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<2)

ISR_CODE void EXTI2_IRQHandler(void)
{
    uint32_t ifg = exti_interrupt_flag_get(EXTI_2) ? (1<<2) : 0;

    if(ifg) {
        exti_interrupt_flag_clear(EXTI_2);
#if (LIMIT_MASK|SD_DETECT_BIT) & (1<<2)
        core_pin_irq(ifg);
#elif SPI_IRQ_BIT & (1<<2)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#elif AUXINPUT_MASK & (1<<2)
        aux_pin_irq(ifg);
#elif SPINDLE_INDEX_BIT & (1<<2)
        spindle_encoder_index_event();
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<3)

ISR_CODE void EXTI3_IRQHandler(void)
{
    uint32_t ifg = exti_interrupt_flag_get(EXTI_3) ? (1<<3) : 0;

    if(ifg) {
        exti_interrupt_flag_clear(EXTI_3);
#if (LIMIT_MASK|SD_DETECT_BIT) & (1<<3)
        core_pin_irq(ifg);
#elif SPI_IRQ_BIT & (1<<3)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#elif AUXINPUT_MASK & (1<<3)
        aux_pin_irq(ifg);
#elif SPINDLE_INDEX_BIT & (1<<3)
        spindle_encoder_index_event();
#endif
    }
}

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (1<<4)

ISR_CODE void EXTI4_IRQHandler(void)
{
    uint32_t ifg = exti_interrupt_flag_get(EXTI_4) ? (1<<4) : 0;

    if(ifg) {
        exti_interrupt_flag_clear(EXTI_4);
#if (LIMIT_MASK|SD_DETECT_BIT) & (1<<4)
        core_pin_irq(ifg);
#elif SPI_IRQ_BIT & (1<<4)
        if(spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#elif AUXINPUT_MASK & (1<<4)
        aux_pin_irq(ifg);
#elif SPINDLE_INDEX_BIT & (1<<4)
        spindle_encoder_index_event();
#endif
    }
}

#endif

#if ((DRIVER_IRQMASK|AUXINPUT_MASK) & 0x03E0)

ISR_CODE void EXTI9_5_IRQHandler(void)
{
    uint32_t ifg = 0;
    for (uint8_t pin = 5; pin <= 9; pin++) {
        exti_line_enum line = (exti_line_enum)(1U << pin);
        if (exti_interrupt_flag_get(line)) {
            exti_interrupt_flag_clear(line);
            ifg |= 1U << pin;
        }
    }

    if(ifg) {
#if SPI_IRQ_BIT & 0x03E0
        if((ifg & SPI_IRQ_BIT) && spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#endif
#if SPINDLE_INDEX_BIT & 0x03E0
        if(ifg & SPINDLE_INDEX_BIT)
            spindle_encoder_index_event();
#endif
#if QEI_SELECT_BIT & 0x03E0
        if(ifg & QEI_SELECT_BIT) {
            // TODO: qei_select_handler() and debounce_start() come from encoder/encoder.c plugin.
            //       Kept as no-op until that plugin is ported.
        }
#endif
#if (LIMIT_MASK|SD_DETECT_BIT) & 0x03E0
        if(ifg & (LIMIT_MASK|SD_DETECT_BIT))
            core_pin_irq(ifg);
#endif
#if AUXINPUT_MASK & 0x03E0
        if(ifg & aux_irq)
            aux_pin_irq(ifg & aux_irq);
#endif
    }
}

void EXTI5_9_IRQHandler(void) __attribute__((alias("EXTI9_5_IRQHandler")));

#endif

#if (DRIVER_IRQMASK|AUXINPUT_MASK) & (0xFC00)

static void EXTI10_15_handler (uint32_t ifg)
{
#if SPI_IRQ_BIT & 0xFC00
    if((ifg & SPI_IRQ_BIT) && spi_irq.callback)
        spi_irq.callback(0, DIGITAL_IN(SPI_IRQ_PORT, SPI_IRQ_PIN) == 0);
#endif
#if SPINDLE_INDEX_BIT & 0xFC00
    if(ifg & SPINDLE_INDEX_BIT)
        spindle_encoder_index_event();
#endif
#if (LIMIT_MASK|SD_DETECT_BIT) & 0xFC00
    if(ifg & (LIMIT_MASK|SD_DETECT_BIT))
        core_pin_irq(ifg);
#endif
#if AUXINPUT_MASK & 0xFC00
    if(ifg & aux_irq)
        aux_pin_irq(ifg & aux_irq);
#endif
}

ISR_CODE void EXTI15_10_IRQHandler(void)
{
    uint32_t ifg = 0;
    for (uint8_t pin = 10; pin <= 15; pin++) {
        exti_line_enum line = (exti_line_enum)(1U << pin);
        if (exti_interrupt_flag_get(line)) {
            exti_interrupt_flag_clear(line);
            ifg |= 1U << pin;
        }
    }

    if(ifg)
        EXTI10_15_handler(ifg);
}

void EXTI10_15_IRQHandler(void) __attribute__((alias("EXTI15_10_IRQHandler")));

#endif
