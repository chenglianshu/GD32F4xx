/*
  cnc_ed1_v20_map.h - driver code for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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

#if N_ABC_MOTORS > 1
#error "CNC_ED1 V2.0 supports 4 motors max."
#endif

#if TRINAMIC_ENABLE
#error "Trinamic plugin not supported!"
#endif

#if !defined(GD32F4xx) || HSE_VALUE != 8000000
#error "This board has a GD32F4xx processor with an 8MHz crystal, select a corresponding build!"
#endif

#undef BOARD_NAME
#define BOARD_NAME "CNC_ED1 V2.0"
#define BOARD_URL  "https://github.com/lunakepio"

#ifndef HSE_VALUE
#define HSE_VALUE 8000000U
#endif

#define SERIAL_PORT     0   // GPIOB: TX = 6, RX = 7

// I2C port selection (uncomment and set I2C_ENABLE=1 in my_machine.h to enable).
// I2C_PORT 1 -> I2C0 on GPIOB: default SCL=PB8/SDA=PB9, alternate SCL=PB6/SDA=PB7 (I2C1_ALT_PINMAP)
// I2C_PORT 2 -> I2C1 on GPIOB: SCL=PB10/SDA=PB11
//#define I2C_PORT            1
//#define I2C1_ALT_PINMAP

// Define step pulse output pins.
#define X_STEP_PORT             GPIOC
#define X_STEP_PIN              13
#define Y_STEP_PORT             GPIOB
#define Y_STEP_PIN              5
#define Z_STEP_PORT             GPIOB
#define Z_STEP_PIN              3
#define STEP_OUTMODE            GPIO_BITBAND
//#define STEP_PINMODE            PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOB
#define X_DIRECTION_PIN         9
#define Y_DIRECTION_PORT        GPIOB
#define Y_DIRECTION_PIN         4
#define Z_DIRECTION_PORT        GPIOD
#define Z_DIRECTION_PIN         2
#define DIRECTION_OUTMODE       GPIO_BITBAND
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOB
#define STEPPERS_ENABLE_PIN     8
//#define STEPPERS_ENABLE_PINMODE PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOC
#define X_LIMIT_PIN             9
#define Y_LIMIT_PORT            GPIOC
#define Y_LIMIT_PIN             1
#define Z_LIMIT_PORT            GPIOC
#define Z_LIMIT_PIN             0
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOC
#define M3_STEP_PIN             15
#define M3_DIRECTION_PORT       GPIOC
#define M3_DIRECTION_PIN        14
#endif

#define AUXOUTPUT0_PORT         GPIOA // Spindle/laser enable (CNC_ENABLE)
#define AUXOUTPUT0_PIN          0
#define AUXOUTPUT1_PORT         GPIOA // Beeper
#define AUXOUTPUT1_PIN          8
#define AUXOUTPUT2_PORT         GPIOA // LED
#define AUXOUTPUT2_PIN          12
#define AUXOUTPUT3_PORT         GPIOA // Spindle PWM
#define AUXOUTPUT3_PIN          10
#define AUXOUTPUT4_PORT         GPIOA // Coolant flood/mist
#define AUXOUTPUT4_PIN          11
#define AUXOUTPUT5_PORT         GPIOB // Generic aux
#define AUXOUTPUT5_PIN          1
#define AUXOUTPUT6_PORT         GPIOB // Generic aux
#define AUXOUTPUT6_PIN          0

// Define driver spindle pins.
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT0_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT        AUXOUTPUT3_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT3_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT4_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT4_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PORT          GPIOC // Safety door
#define AUXINPUT0_PIN           8
#define AUXINPUT1_PORT          GPIOB // Probe
#define AUXINPUT1_PIN           15
#define AUXINPUT2_PORT          GPIOB // Reset/EStop
#define AUXINPUT2_PIN           12
#define AUXINPUT3_PORT          GPIOB // Feed hold
#define AUXINPUT3_PIN           13
#define AUXINPUT4_PORT          GPIOB // Cycle start
#define AUXINPUT4_PIN           14

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT2_PORT
#define RESET_PIN               AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT3_PORT
#define FEED_HOLD_PIN           AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT4_PORT
#define CYCLE_START_PIN         AUXINPUT4_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT1_PORT
#define PROBE_PIN               AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif

#if SDCARD_ENABLE
#define SPI_PORT                0 // GPIOA, SCK_PIN = 5, MISO_PIN = 6, MOSI_PIN = 7 (SPI0)
#define SD_CS_PORT              GPIOC
#define SD_CS_PIN               4
#endif

// USB-CDC pins are fixed by GD32F4xx USB OTG FS hardware.
#define USB_DM_PORT             GPIOA
#define USB_DM_PIN              11
#define USB_DP_PORT             GPIOA
#define USB_DP_PIN              12
