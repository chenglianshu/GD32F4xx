/*!
    \file    systick.c
    \brief   the systick configuration file

    \version 2025-07-31, V3.3.2, firmware for GD32F4xx
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "systick.h"
#ifdef USE_FREERTOS
#include "cmsis_os2.h"
#endif

volatile static uint32_t delay;

#ifdef USE_FREERTOS
/*!
    \brief    busy-wait 1ms delay by programming SysTick directly
    \note     used before the FreeRTOS scheduler runs (SysTick is owned by the
              RTOS afterwards, so delay_decrement() is not called).
*/
static void delay_busy_wait_1ms(uint32_t count)
{
    uint32_t reload;

    if(0U == count) {
        return;
    }

    reload = (SystemCoreClock / 1000U) - 1U;
    SysTick->LOAD = reload;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while(count-- != 0U) {
        while(0U == (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {
        }
    }

    SysTick->CTRL = 0U;
    SysTick->VAL  = 0U;
}
#endif /* USE_FREERTOS */

/*!
    \brief    configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
#ifndef USE_FREERTOS
    /* setup systick timer for 1000Hz interrupts */
    if(SysTick_Config(SystemCoreClock / 1000U)) {
        /* capture error */
        while(1) {
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
#endif
}

/*!
    \brief    delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void delay_1ms(uint32_t count)
{
#ifdef USE_FREERTOS
    if((0U != count) && (osKernelRunning == osKernelGetState())) {
        (void)osDelay(count);
        return;
    }
    delay_busy_wait_1ms(count);
#else
    delay = count;

    while(0U != delay) {
    }
#endif
}

/*!
    \brief    delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/
void delay_decrement(void)
{
    if(0U != delay) {
        delay--;
    }
}
