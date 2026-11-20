/*

  flash.c - driver code for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2023 Terje Io

  This code reads/writes the whole RAM-based emulated EPROM contents from/to flash

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

#include <string.h>

#include "driver.h"
#include "grbl/hal.h"

#if FLASH_ENABLE

// GD32F425RE: 512K flash => 16K (sectors 0-3), 64K (sector 4), 128K (sectors 5-7).
// Reserve the last 128K sector (sector 7 @ 0x08060000) for NVS/EEPROM emulation.
#define EEPROM_Emul_Sector      CTL_SECTOR_NUMBER_7
#define EEPROM_Emul_Start       0x08060000U

// Hooks implemented by driver.c: the CPU stalls for the whole sector erase
// (code runs from the same single-bank flash), so electrical noise on input
// lines gets latched by EXTI hardware during the stall and is processed as
// spurious control events (e.g. an unwanted feed hold) when it resumes.
// The driver masks input EXTI lines and clears pending flags around the
// erase/program to suppress that.
void __attribute__((weak)) nvs_flash_io_suspend (void) {}
void __attribute__((weak)) nvs_flash_io_resume (void) {}

bool memcpy_from_flash (uint8_t *dest)
{
    memcpy(dest, (uint8_t *)EEPROM_Emul_Start, hal.nvs.size);

    return true;
}

bool memcpy_to_flash (uint8_t *source)
{
    if (!memcmp(source, (uint8_t *)EEPROM_Emul_Start, hal.nvs.size))
        return true;

    fmc_state_enum status = FMC_READY;

    nvs_flash_io_suspend();

    fmc_unlock();

    // Clear any sticky status flags so a retry is not blocked by a previous error
    // (GD32 fmc_ready_wait() aborts immediately if an error flag is still set).
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);

    // Retry erase once if it fails (ref issue #121)
    if((status = fmc_sector_erase(EEPROM_Emul_Sector)) != FMC_READY) {
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
        status = fmc_sector_erase(EEPROM_Emul_Sector);
    }

    uint16_t *data = (uint16_t *)source;
    uint32_t address = EEPROM_Emul_Start, remaining = (uint32_t)hal.nvs.size;

    while(remaining && status == FMC_READY) {
        if((status = fmc_halfword_program(address, *data++)) == FMC_READY)
            status = fmc_halfword_program(address + 2, *data++);
        address += 4;
        remaining -= 4;
    }

    fmc_lock();

    nvs_flash_io_resume();

    return status == FMC_READY;
}

#endif
