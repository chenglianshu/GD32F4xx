

#pragma once

#define BOARD_CNC_ED1_V20

#define BOARD_NAME "CNC_ED1_V20"

// -----------------------------------------------------------------------------
// Clock
// -----------------------------------------------------------------------------
#ifndef HSE_VALUE
#define HSE_VALUE 8000000U
#endif

// -----------------------------------------------------------------------------
// Primary serial (grblHAL stream) - USART0 remapped to PB6/PB7
// -----------------------------------------------------------------------------
#define SERIAL_PORT         0
#define UART0_TX_PORT       GPIOB
#define UART0_TX_PIN        6
#define UART0_RX_PORT       GPIOB
#define UART0_RX_PIN        7

// USART0 remapped function: PB6 = TX, PB7 = RX
#define UART0_GPIO_PORT     GPIOB
#define UART0_GPIO_AF       GPIO_AF_7

// -----------------------------------------------------------------------------
// Auxiliary serial ports as present on the original board
// -----------------------------------------------------------------------------
// USART1 - LCD
#define UART1_TX_PORT       GPIOA
#define UART1_TX_PIN        2
#define UART1_RX_PORT       GPIOA
#define UART1_RX_PIN        3

// USART2 - Bluetooth
#define UART2_TX_PORT       GPIOB
#define UART2_TX_PIN        10
#define UART2_RX_PORT       GPIOB
#define UART2_RX_PIN        11

// UART5 - touch LCD
#define UART5_TX_PORT       GPIOC
#define UART5_TX_PIN        6
#define UART5_RX_PORT       GPIOC
#define UART5_RX_PIN        7

// -----------------------------------------------------------------------------
// Shared motor enable
// -----------------------------------------------------------------------------
#define STEPPERS_ENABLE_PORT    GPIOB
#define STEPPERS_ENABLE_PIN     8

// -----------------------------------------------------------------------------
// X axis
// -----------------------------------------------------------------------------
#define X_STEP_PORT         GPIOC
#define X_STEP_PIN          13
#define X_DIRECTION_PORT    GPIOB
#define X_DIRECTION_PIN     9
#define X_LIMIT_PORT        GPIOC
#define X_LIMIT_PIN         9

// -----------------------------------------------------------------------------
// Y axis
// -----------------------------------------------------------------------------
#define Y_STEP_PORT         GPIOB
#define Y_STEP_PIN          5
#define Y_DIRECTION_PORT    GPIOB
#define Y_DIRECTION_PIN     4
#define Y_LIMIT_PORT        GPIOC
#define Y_LIMIT_PIN         1

// -----------------------------------------------------------------------------
// Z axis
// -----------------------------------------------------------------------------
#define Z_STEP_PORT         GPIOB
#define Z_STEP_PIN          3
#define Z_DIRECTION_PORT    GPIOD
#define Z_DIRECTION_PIN     2
#define Z_LIMIT_PORT        GPIOC
#define Z_LIMIT_PIN         0

// -----------------------------------------------------------------------------
// A axis (rotary / 4th axis) - mapped as motor 3
// -----------------------------------------------------------------------------
#define M3_AVAILABLE        1
#define M3_STEP_PORT        GPIOC
#define M3_STEP_PIN         15
#define M3_DIRECTION_PORT   GPIOC
#define M3_DIRECTION_PIN    14
// No dedicated A limit pin is defined in the original source.


// -----------------------------------------------------------------------------
// Probe
// -----------------------------------------------------------------------------
#define PROBE_PORT          GPIOB
#define PROBE_PIN           15

// -----------------------------------------------------------------------------
// Spindle / laser PWM (TIMER0)
// -----------------------------------------------------------------------------
// Primary spindle/CNC PWM on TIMER0 CH2 (PA10)
#define SPINDLE_PWM_PORT    GPIOA
#define SPINDLE_PWM_PIN     10
#define SPINDLE_PWM_TIMER   0
#define SPINDLE_PWM_CHANNEL TIMER_CH_2
#define SPINDLE_PWM_AF      GPIO_AF_1

// Secondary laser PWM on TIMER0 CH1 (PA9)
#define LASER_PWM_PORT      GPIOA
#define LASER_PWM_PIN       9
#define LASER_PWM_TIMER     0
#define LASER_PWM_CHANNEL   TIMER_CH_1
#define LASER_PWM_AF        GPIO_AF_1

// Optional spindle enable/direction pins (commented out by default)
// #define SPINDLE_ENABLE_PORT     GPIOA
// #define SPINDLE_ENABLE_PIN      8
// #define SPINDLE_DIRECTION_PORT  GPIOA
// #define SPINDLE_DIRECTION_PIN   9

// -----------------------------------------------------------------------------
// Coolant (M7/M8)
// -----------------------------------------------------------------------------
// Flood and mist share the same physical pin on this board.
#define COOLANT_FLOOD_PORT  GPIOA
#define COOLANT_FLOOD_PIN   11
#define COOLANT_MIST_PORT   GPIOA
#define COOLANT_MIST_PIN    11

// -----------------------------------------------------------------------------
// SPI0 (SD card)
// -----------------------------------------------------------------------------
#define SPI_PORT            0
#define SPI_SCK_PORT        GPIOA
#define SPI_SCK_PIN         5
#define SPI_MISO_PORT       GPIOA
#define SPI_MISO_PIN        6
#define SPI_MOSI_PORT       GPIOA
#define SPI_MOSI_PIN        7
#define SPI_AF              GPIO_AF_5

#if SDCARD_ENABLE
#define SD_CS_PORT          GPIOC
#define SD_CS_PIN           4
#endif

// -----------------------------------------------------------------------------
// CNC/Laser enable, beeper, LED
// -----------------------------------------------------------------------------
#define CNC_ENABLE_PORT     GPIOA
#define CNC_ENABLE_PIN      0

#define BEEPER_PORT         GPIOA
#define BEEPER_PIN          8

#define LED_PORT            GPIOA
#define LED_PIN             12

// -----------------------------------------------------------------------------
// Auxiliary digital outputs
// -----------------------------------------------------------------------------
#define AUXOUTPUT0_PORT     GPIOA       // CNC/laser enable
#define AUXOUTPUT0_PIN      0
#define AUXOUTPUT1_PORT     GPIOA       // Beeper
#define AUXOUTPUT1_PIN      8
#define AUXOUTPUT2_PORT     GPIOA       // LED
#define AUXOUTPUT2_PIN      12
#define AUXOUTPUT3_PORT     GPIOA       // Shared flood/mist coolant
#define AUXOUTPUT3_PIN      11
#define AUXOUTPUT4_PORT     GPIOB       // Motor DC IN1 / generic aux
#define AUXOUTPUT4_PIN      1
#define AUXOUTPUT5_PORT     GPIOB       // Motor DC IN2 / generic aux
#define AUXOUTPUT5_PIN      0

// -----------------------------------------------------------------------------
// Optional digital aux inputs (commented out by default)
// AUXINPUT0_PIN through AUXINPUT23_PIN may be defined to add generic inputs.
// -----------------------------------------------------------------------------
// #define AUXINPUT0_PORT        GPIOA
// #define AUXINPUT0_PIN         0
// #define AUXINPUT1_PORT        GPIOA
// #define AUXINPUT1_PIN         1
// #define AUXINPUT2_PORT        GPIOA
// #define AUXINPUT2_PIN         2
// #define AUXINPUT3_PORT        GPIOA
// #define AUXINPUT3_PIN         3
// #define AUXINPUT4_PORT        GPIOA
// #define AUXINPUT4_PIN         4
// #define AUXINPUT5_PORT        GPIOA
// #define AUXINPUT5_PIN         5
// #define AUXINPUT6_PORT        GPIOA
// #define AUXINPUT6_PIN         6
// #define AUXINPUT7_PORT        GPIOA
// #define AUXINPUT7_PIN         7
// #define AUXINPUT8_PORT        GPIOA
// #define AUXINPUT8_PIN         8
// #define AUXINPUT9_PORT        GPIOA
// #define AUXINPUT9_PIN         9
// #define AUXINPUT10_PORT       GPIOA
// #define AUXINPUT10_PIN        10
// #define AUXINPUT11_PORT       GPIOA
// #define AUXINPUT11_PIN        11
// #define AUXINPUT12_PORT       GPIOA
// #define AUXINPUT12_PIN        12
// #define AUXINPUT13_PORT       GPIOA
// #define AUXINPUT13_PIN        13
// #define AUXINPUT14_PORT       GPIOA
// #define AUXINPUT14_PIN        14
// #define AUXINPUT15_PORT       GPIOA
// #define AUXINPUT15_PIN        15
// #define AUXINPUT16_PORT       GPIOB
// #define AUXINPUT16_PIN        0
// #define AUXINPUT17_PORT       GPIOB
// #define AUXINPUT17_PIN        1
// #define AUXINPUT18_PORT       GPIOB
// #define AUXINPUT18_PIN        2
// #define AUXINPUT19_PORT       GPIOB
// #define AUXINPUT19_PIN        3
// #define AUXINPUT20_PORT       GPIOB
// #define AUXINPUT20_PIN        4
// #define AUXINPUT21_PORT       GPIOB
// #define AUXINPUT21_PIN        5
// #define AUXINPUT22_PORT       GPIOB
// #define AUXINPUT22_PIN        6
// #define AUXINPUT23_PORT       GPIOB
// #define AUXINPUT23_PIN        7

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// #define RESET_PORT          GPIOA
// #define RESET_PIN           2
// #define FEED_HOLD_PORT      GPIOA
// #define FEED_HOLD_PIN       2
// #define CYCLE_START_PORT    GPIOA
// #define CYCLE_START_PIN     2
// #define SAFETY_DOOR_PORT    GPIOA
// #define SAFETY_DOOR_PIN     2
// #define MOTOR_FAULT_PORT    GPIOA
// #define MOTOR_FAULT_PIN     2

// -----------------------------------------------------------------------------
// Optional door / flame sensors (commented out in original source)
// -----------------------------------------------------------------------------
// #define DOOR_PORT           GPIOA
// #define DOOR_PIN            2
// #define FLAME_PORT          GPIOC
// #define FLAME_PIN           13

// -----------------------------------------------------------------------------
// Optional analog aux I/O (commented out by default)
// Enable by uncommenting and setting AUX_ANALOG_ENABLE=1 in Inc/my_machine.h
// -----------------------------------------------------------------------------
// #define AUXINPUT0_ANALOG_PORT   GPIOA
// #define AUXINPUT0_ANALOG_PIN    0
// #define AUXINPUT1_ANALOG_PORT   GPIOA
// #define AUXINPUT1_ANALOG_PIN    1

// #define AUXOUTPUT0_ANALOG_PORT  GPIOA
// #define AUXOUTPUT0_ANALOG_PIN   4   // DAC0
// #define AUXOUTPUT1_ANALOG_PORT  GPIOA
// #define AUXOUTPUT1_ANALOG_PIN   5   // DAC1
// #define AUXOUTPUT0_PWM_PORT     GPIOA
// #define AUXOUTPUT0_PWM_PIN      6

// -----------------------------------------------------------------------------
// Optional digital aux outputs (commented out by default)
// AUXOUTPUT6_PORT through AUXOUTPUT15_PORT may be defined to add generic outputs.
// -----------------------------------------------------------------------------
// #define AUXOUTPUT6_PORT       GPIOB
// #define AUXOUTPUT6_PIN        12
// #define AUXOUTPUT7_PORT       GPIOB
// #define AUXOUTPUT7_PIN        13
// #define AUXOUTPUT8_PORT       GPIOB
// #define AUXOUTPUT8_PIN        14
// #define AUXOUTPUT9_PORT       GPIOB
// #define AUXOUTPUT9_PIN        15
// #define AUXOUTPUT10_PORT      GPIOC
// #define AUXOUTPUT10_PIN       8
// #define AUXOUTPUT11_PORT      GPIOC
// #define AUXOUTPUT11_PIN       9
// #define AUXOUTPUT12_PORT      GPIOC
// #define AUXOUTPUT12_PIN       10
// #define AUXOUTPUT13_PORT      GPIOC
// #define AUXOUTPUT13_PIN       11
// #define AUXOUTPUT14_PORT      GPIOC
// #define AUXOUTPUT14_PIN       12
// #define AUXOUTPUT15_PORT      GPIOC
// #define AUXOUTPUT15_PIN       13

// -----------------------------------------------------------------------------
// Optional SPI IRQ / SD card detect (commented out by default)
// -----------------------------------------------------------------------------
// #define SPI_IRQ_PORT          GPIOB
// #define SPI_IRQ_PIN           2
// #define SD_DETECT_PORT        GPIOC
// #define SD_DETECT_PIN         5

// -----------------------------------------------------------------------------
// Optional quadrature encoder / spindle encoder (commented out by default)
// -----------------------------------------------------------------------------
// #define QEI_PORT                0   // index into encoders[]
// #define QEI_A_PORT              GPIOA
// #define QEI_A_PIN               6
// #define QEI_B_PORT              GPIOA
// #define QEI_B_PIN               7
//
// #define SPINDLE_INDEX_PORT      GPIOA
// #define SPINDLE_INDEX_PIN       0
// #define RPM_TIMER_N             2

// -----------------------------------------------------------------------------
// Optional quadrature encoder / spindle encoder (commented out by default)
// -----------------------------------------------------------------------------
// #define QEI_PORT                0   // index into encoders[]
// #define QEI_A_PORT              GPIOA
// #define QEI_A_PIN               6
// #define QEI_B_PORT              GPIOA
// #define QEI_B_PIN               7
//
// #define SPINDLE_INDEX_PORT      GPIOA
// #define SPINDLE_INDEX_PIN       0
// #define RPM_TIMER_N             2

// -----------------------------------------------------------------------------
// USB-CDC pins (PA11/PA12, fixed by GD32F4xx USB OTG FS hardware)
// -----------------------------------------------------------------------------
#define USB_DM_PORT         GPIOA
#define USB_DM_PIN          11
#define USB_DP_PORT         GPIOA
#define USB_DP_PIN          12
