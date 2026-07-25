// Src/main.c
#include "driver.h"
#include "grbl/grbl.h"
#include "grbl/grbllib.h"

volatile static uint32_t sys_tick = 0;
volatile uint32_t cycle_count = 0;
volatile static uint32_t delay;

void systick_config(void);

static void MX_GPIO_Init(void)
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

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

int main(void)
{
    // With no bootloader the vector table starts at 0x0800_0000.
    __disable_irq();
    SCB->VTOR = 0x08000000;
    __DSB();
    __enable_irq();

    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    MX_GPIO_Init();
    DWT_Init();

    // 1 ms SysTick @ 168 MHz HCLK
    systick_config();

    grbl_enter();
}



void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        /* capture error */
        while (1)
        {
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

void delay_1ms(uint32_t count)
{
    delay = count;

    while (0U != delay)
    {
    }
}

void delay_decrement(void)
{
    if (0U != delay)
    {
        delay--;
    }
}

void SysTick_Handler(void)
{
    cycle_count = DWT->CYCCNT;
    sys_tick++;
    delay_decrement();
    Driver_IncTick();
}

uint32_t hal_get_tick(void)
{
    return sys_tick;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = sys_tick;
    while ((sys_tick - start) < ms);
}

void HAL_Delay(uint32_t Delay)
{
	#if defined(USE_FREERTOS_RTOS)
	vTaskDelay(Delay);
	#else
	delay_1ms(Delay);
	#endif
}