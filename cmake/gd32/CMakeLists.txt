cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

# ---------------------------------------------------------------------------
# Defines
# ---------------------------------------------------------------------------
set(GD32_DEFINES
    GD32F425
    GD32F4
    GD32F4xx
    STM32F407xx
    USE_STDPERIPH_DRIVER
    USE_FREERTOS
    HXTAL_VALUE=8000000U
    $<$<CONFIG:Debug>:DEBUG>
)

# ---------------------------------------------------------------------------
# Common include paths (grbl-first: grbl/task.h has the same name as FreeRTOS task.h)
# ---------------------------------------------------------------------------
set(GD32_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/boards
    ${CMAKE_SOURCE_DIR}/grbl
    ${CMAKE_SOURCE_DIR}/sdcard
    ${CMAKE_SOURCE_DIR}/laser
    ${CMAKE_SOURCE_DIR}/spindle
    ${CMAKE_SOURCE_DIR}/spindle/vfd
    ${CMAKE_SOURCE_DIR}/eeprom
    ${CMAKE_SOURCE_DIR}/encoder
    ${CMAKE_SOURCE_DIR}/fatfs/src
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_standard_peripheral/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/GD/GD32F4xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/device/core/Include
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/device/class/cdc/Include
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/driver/Include
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/ustd/common
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/ustd/class/cdc
)

# FreeRTOS include dirs (kept separate to avoid task.h clash with grbl/task.h)
set(FREERTOS_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/GD/GD32F4xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_standard_peripheral/Include
)

# ---------------------------------------------------------------------------
# Application sources (grblHAL driver + grbl core + plugins)
# ---------------------------------------------------------------------------
file(GLOB GD32_APPLICATION_SOURCES ${CMAKE_SOURCE_DIR}/Core/Src/*.c)
list(REMOVE_ITEM GD32_APPLICATION_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/Src/systick.c
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c       # built as the RTOS entry object below
)

file(GLOB GRBL_SOURCES
    ${CMAKE_SOURCE_DIR}/grbl/*.c
    ${CMAKE_SOURCE_DIR}/grbl/kinematics/*.c
)

file(GLOB SUBMODULE_SOURCES
    ${CMAKE_SOURCE_DIR}/boards/*.c
    ${CMAKE_SOURCE_DIR}/sdcard/*.c
    ${CMAKE_SOURCE_DIR}/laser/*.c
    ${CMAKE_SOURCE_DIR}/spindle/*.c
    ${CMAKE_SOURCE_DIR}/spindle/vfd/*.c
    ${CMAKE_SOURCE_DIR}/eeprom/*.c
    ${CMAKE_SOURCE_DIR}/encoder/*.c
)

set(STARTUP_SOURCES ${CMAKE_SOURCE_DIR}/startup_gd32f425re.S)

# ---------------------------------------------------------------------------
# GD32 standard peripheral library + CMSIS system
# ---------------------------------------------------------------------------
file(GLOB GD32_SPL_SOURCES ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_standard_peripheral/Source/*.c)
set(SYSTEM_SOURCES ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/GD/GD32F4xx/Source/system_gd32f4xx.c)

# ---------------------------------------------------------------------------
# USB device library (CDC ACM)
# ---------------------------------------------------------------------------
set(USB_SOURCES
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/device/core/Source/usbd_core.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/device/core/Source/usbd_enum.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/device/core/Source/usbd_transc.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/device/class/cdc/Source/cdc_acm_core.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/driver/Source/drv_usb_core.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/driver/Source/drv_usb_dev.c
    ${CMAKE_SOURCE_DIR}/Drivers/GD32F4xx_usb_library/driver/Source/drv_usbd_int.c
)

# ---------------------------------------------------------------------------
# FatFs core (grblHAL provides its own diskio)
# ---------------------------------------------------------------------------
set(FATFS_SOURCES
    ${CMAKE_SOURCE_DIR}/fatfs/src/ff.c
    ${CMAKE_SOURCE_DIR}/fatfs/src/ffsystem.c
    ${CMAKE_SOURCE_DIR}/fatfs/src/ffunicode.c
)

# ---------------------------------------------------------------------------
# FreeRTOS kernel
# ---------------------------------------------------------------------------
set(FREERTOS_SOURCES
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

# ---------------------------------------------------------------------------
# Interface library for includes and symbols (non-FreeRTOS sources)
# ---------------------------------------------------------------------------
add_library(gd32 INTERFACE)
target_include_directories(gd32 INTERFACE ${GD32_INCLUDE_DIRS})
target_compile_definitions(gd32 INTERFACE ${GD32_DEFINES})

# GD32 HAL/SPL + CMSIS system
add_library(GD32_Drivers OBJECT)
target_sources(GD32_Drivers PRIVATE ${GD32_SPL_SOURCES} ${SYSTEM_SOURCES})
target_link_libraries(GD32_Drivers PUBLIC gd32)

# FatFs
add_library(fat_fs OBJECT)
target_sources(fat_fs PRIVATE ${FATFS_SOURCES})
target_link_libraries(fat_fs PUBLIC gd32)

# USB device library
add_library(UsbLib OBJECT)
target_sources(UsbLib PRIVATE ${USB_SOURCES})
target_link_libraries(UsbLib PUBLIC gd32)

# FreeRTOS kernel (own include dirs, no grbl, to keep FreeRTOS/task.h)
add_library(FreeRTOS OBJECT)
target_sources(FreeRTOS PRIVATE ${FREERTOS_SOURCES})
target_include_directories(FreeRTOS PRIVATE ${FREERTOS_INCLUDE_DIRS})
target_compile_definitions(FreeRTOS PRIVATE ${GD32_DEFINES})

# RTOS entry (main.c): FreeRTOS includes first, then the common dirs
add_library(RtosEntry OBJECT)
target_sources(RtosEntry PRIVATE ${CMAKE_SOURCE_DIR}/Core/Src/main.c)
target_include_directories(RtosEntry PRIVATE ${FREERTOS_INCLUDE_DIRS} ${GD32_INCLUDE_DIRS})
target_compile_definitions(RtosEntry PRIVATE ${GD32_DEFINES})

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${GD32_APPLICATION_SOURCES}
    ${GRBL_SOURCES}
    ${SUBMODULE_SOURCES}
    ${STARTUP_SOURCES}
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    gd32
    GD32_Drivers
    FreeRTOS
    fat_fs
    UsbLib
    RtosEntry
    -Wl,--start-group c gcc m stdc++ nosys -Wl,--end-group
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
