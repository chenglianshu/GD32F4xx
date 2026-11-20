/*

  main.c - driver code for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

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

#include "main.h"
#include "driver.h"
#include "grbl/grbllib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

// Tick bookkeeping used by grblHAL (see driver.c: getElapsedMicros/getElapsedTicks).
volatile uint32_t uwTick = 0;
volatile uint32_t cycle_count = 0;

static void MX_GPIO_Init (void);
static void DWT_Init (void);

// grblHAL runs as a single, highest priority task (its core is cooperative).
static void grblTask (void *argument)
{
    (void)argument;

    grbl_enter();   // never returns
}

// FreeRTOS owns SysTick, so grblHAL's 1 ms tick is driven from the RTOS tick hook.
void vApplicationTickHook (void)
{
    uwTick++;
    cycle_count = DWT->CYCCNT;
    Driver_IncTick();
}

int main (void)
{
    // With no bootloader the vector table starts at 0x0800_0000.
    __disable_irq();
    SCB->VTOR = 0x08000000;
    __DSB();
    __enable_irq();

    // FreeRTOS/CMSIS-RTOS2 use 4 bits for preemption priority.
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    MX_GPIO_Init();
    DWT_Init();

    osKernelInitialize();

    const osThreadAttr_t grblTaskAttr = {
        .name = "grbl",
        .stack_size = 4096,
        .priority = (osPriority_t)osPriorityRealtime
    };

    if(osThreadNew(grblTask, NULL, &grblTaskAttr) == NULL)
        while(1) {
        }

    osKernelStart();

    while(1) {
    }
}

uint32_t hal_get_tick (void)
{
    return uwTick;
}

void delay_ms (uint32_t ms)
{
    // SysTick (lowest priority ISR) cannot preempt a higher-priority ISR, and
    // is not running before the scheduler starts: waiting on uwTick there
    // would hang forever. Busy-wait on DWT cycles in those contexts instead.
    // NOTE: the DWT path is limited to ~21 s (32-bit CYCCNT @ 200 MHz).
    if((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        uint32_t start = DWT->CYCCNT, cycles = ms * (SystemCoreClock / 1000);
        while((DWT->CYCCNT - start) < cycles);
        return;
    }

    uint32_t start = uwTick;

    while((uwTick - start) < ms);
}

void HAL_Delay (uint32_t Delay)
{
    delay_ms(Delay);
}

// FreeRTOS stack overflow check (configCHECK_FOR_STACK_OVERFLOW 2).
void vApplicationStackOverflowHook (TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();
    for( ;; );
}

static void MX_GPIO_Init (void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOG);
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOI);
}

static void DWT_Init (void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
