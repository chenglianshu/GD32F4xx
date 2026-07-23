// Src/flash.c
// Flash driver code for GD32F4xx grblHAL port.
// Reads/writes the whole RAM-based emulated EEPROM contents from/to flash.

#include <string.h>

#include "driver.h"
#include "grbl/hal.h"

#if FLASH_ENABLE

// Use sector 6 (0x08040000, 128KB) for settings storage.
// Code occupies sectors 0-5 (~168KB), so sector 6 is free.
#define EEPROM_SECTOR           CTL_SECTOR_NUMBER_6
#define EEPROM_START_ADDRESS    0x08040000U

bool memcpy_from_flash (uint8_t *dest)
{
    memcpy(dest, (uint8_t *)EEPROM_START_ADDRESS, hal.nvs.size);
    return true;
}

bool memcpy_to_flash (uint8_t *source)
{
    if (!memcmp(source, (uint8_t *)EEPROM_START_ADDRESS, hal.nvs.size))
        return true;

    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_WPERR);

    fmc_state_enum status = fmc_sector_erase(EEPROM_SECTOR);

    uint16_t *data = (uint16_t *)source;
    uint32_t address = EEPROM_START_ADDRESS, remaining = (uint32_t)hal.nvs.size;

    while(remaining && status == FMC_READY) {
        status = fmc_halfword_program(address, *data++);
        if(status == FMC_READY)
            status = fmc_halfword_program(address + 2, *data++);
        address += 4;
        remaining -= 4;
    }

    fmc_lock();

    return status == FMC_READY;
}

#endif
