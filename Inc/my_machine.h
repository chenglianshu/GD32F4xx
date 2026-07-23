// Inc/my_machine.h
// Default board and feature selection for GD32F425RET6 grblHAL port.
// When OVERRIDE_MY_MACHINE is defined in platformio.ini, this file is used.

#pragma once

#undef BOARD_CNC_ED1_V20
#define BOARD_CNC_ED1_V20
#undef BOARD_NAME
#define BOARD_NAME "CNC_ED1_V20"

#define N_AXIS          4
#define N_SPINDLE       1
#define SERIAL_PORT     0

// Phase 1: keep only core features
#define SDCARD_ENABLE       1
#define USB_SERIAL_ENABLE   0
#define USB_SERIAL_CDC      0
#define I2C_ENABLE          0
#define SPI_ENABLE          1
#define EEPROM_ENABLE       0
#define FLASH_ENABLE        1
#define TRINAMIC_ENABLE     0
#define ETHERNET_ENABLE     0
#define BLUETOOTH_ENABLE    0
#define NEOPIXEL_ENABLE     0
#define KEYPAD_ENABLE       0
#define ODOMETER_ENABLE     0
#define PROBE_ENABLE        1
#define LIMITS_ENABLE       1
#define PWM_SPINDLE_ENABLE  1

// Laser plugins ported from STM32F4xx laser project
#define PPI_ENABLE          0
#define LASER_COOLANT_ENABLE 0
#define LASER_OVD_ENABLE    0
#define LB_CLUSTERS_ENABLE  0

// Advanced features: files ported from STM32F4xx but disabled by default
#define AUX_ANALOG_ENABLE   0
#define ENCODER_ENABLE      0
// NOTE: DRIVER_SPINDLE_ENABLE is auto-derived from spindle pin macros
// and cannot be easily overridden here. driver_spindles.c is compiled
// but currently contains a NOOP stub to avoid conflicts with driver.c.
