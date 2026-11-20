/*

  serial.h - stream interface for serial ports

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io

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

/*

GD32 0-based port numbering: N = instance N (0..4 = USART0, USART1, USART2,
UART3, UART4) on its default pins, NX = a remap of instance N. Exception: 10
is the legacy encoding for USART0 on its default pins.

0  - GPIOB: TX =  6, RX =  7 - USART0 remap (board default)
10 - GPIOA: TX =  9, RX = 10 - USART0 default (legacy encoding)
1  - GPIOA: TX =  2, RX =  3 - USART1 default
11 - GPIOD: TX =  5, RX =  6 - USART1 remap
2  - GPIOB: TX = 10, RX = 11 - USART2 default
21 - GPIOC: TX = 10, RX = 11 - USART2 remap
22 - GPIOD: TX =  8, RX =  9 - USART2 remap
3  - GPIOC: TX = 10, RX = 11 - UART3 default (AF8)
31 - GPIOA: TX =  0, RX =  1 - UART3 remap (AF8)
4  - GPIOC: TX = 12, GPIOD: RX = 2 - UART4 (AF8)

*/

#pragma once

void serialRegisterStreams (void);

/*EOF*/
