/*
  my_machine.h - configuration for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used.

#undef BOARD_CNC_ED1_V20
#define BOARD_CNC_ED1_V20

#define N_AXIS          4
#define N_SPINDLE       1
#define SERIAL_PORT     0

// Configuration
// Uncomment to enable.

// Communication options
#define SDCARD_ENABLE           1   // Run G-code programs from SD card (2 to also enable YModem upload)
//#define USB_SERIAL_ENABLE       1   // Use native USB as virtual serial port
//#define USB_SERIAL_CDC          1   // Use USB CDC class for serial communication
//#define BLUETOOTH_ENABLE        2   // Bluetooth (2 = HC-05 module)
//#define I2C_ENABLE              1
#define SPI_ENABLE              1
//#define ETHERNET_ENABLE         1   // Requires networking plugin
//#define WEBUI_ENABLE            3   // ESP3D-WEBUI plugin

// Non-volatile storage
//#define EEPROM_ENABLE          16   // I2C EEPROM/FRAM (16=2KB, 32=4KB, ...)
#define FLASH_ENABLE            1   // Use internal flash for settings storage
//#define EEPROM_IS_FRAM          1

// Spindle options
#define PWM_SPINDLE_ENABLE      1   // Enable PWM spindle control
//#define SPINDLE0_ENABLE         SPINDLE_HUANYANG1
//#define SPINDLE1_ENABLE         SPINDLE_PWM0
//#define SPINDLE_OFFSET          1   // Laser XY offset plugin

// Motion / motor options
//#define TRINAMIC_ENABLE      2130   // TMC2130 stepper drivers (SPI)
//#define TRINAMIC_ENABLE      2209   // TMC2209 stepper drivers (UART)
//#define TRINAMIC_R_SENSE      110   // Sense resistor in milliohms
//#define X_GANGED                1
//#define Y_GANGED                1
//#define Z_GANGED                1
//#define X_AUTO_SQUARE           1
//#define Y_AUTO_SQUARE           1
//#define Z_AUTO_SQUARE           1

// Laser plugins
//#define PPI_ENABLE              1   // Laser pulses-per-inch plugin
//#define LASER_COOLANT_ENABLE    1
//#define LASER_OVD_ENABLE        1   // Laser overdrive PWM output
//#define LB_CLUSTERS_ENABLE      1   // LaserBurn cluster support

// Input / probe options
#define PROBE_ENABLE            1   // Probe input
#define LIMITS_ENABLE           1   // Hard limit switches
#define SAFETY_DOOR_ENABLE      0   // Safety door input
//#define ESTOP_ENABLE            0   // E-stop behavior (reset pin triggers status report only)
//#define PROBE2_ENABLE           1
//#define TOOLSETTER_ENABLE       1
//#define MOTOR_FAULT_ENABLE      1
//#define MOTOR_WARNING_ENABLE    1

// Advanced features
//#define MPG_ENABLE              1   // Manual pulse generator (MPG) interface
//#define KEYPAD_ENABLE           1   // I2C keypad (2 = serial keypad, shares MPG port)
//#define DISPLAY_ENABLE          9   // I2C display protocol
//#define MACROS_ENABLE           2   // Macro commands plugin
//#define ODOMETER_ENABLE         1
//#define AUX_ANALOG_ENABLE       1
//#define ENCODER_ENABLE          1   // 1 = spindle encoder, 2 = MPG + QEI encoder
//#define NEOPIXEL_ENABLE         1
//#define RGB_LED_ENABLE          2
//#define PWM_SERVO_ENABLE        1
//#define BLTOUCH_ENABLE          1
//#define EVENTOUT_ENABLE         1

// IO expanders
//#define MCP3221_ENABLE          1
//#define PCA9654E_ENABLE         1
//#define FNC_EXPANDER_ENABLE     1

#ifdef _WIZCHIP_
#define ETHERNET_ENABLE 1
#endif

#if ETHERNET_ENABLE || WEBUI_ENABLE
#define TELNET_ENABLE       1
#define WEBSOCKET_ENABLE    1
#endif

// The following symbols have the default values as shown, uncomment and change as needed.
//#define NETWORK_HOSTNAME        "grblHAL"
//#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
//#define NETWORK_IP              "192.168.5.1"
//#define NETWORK_GATEWAY         "192.168.5.1"
//#define NETWORK_MASK            "255.255.255.0"
