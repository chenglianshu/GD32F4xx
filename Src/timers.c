// Src/timers.c
// Timer driver code for GD32F4xx grblHAL port.

#include "driver.h"
#include "grbl/hal.h"

typedef struct {
    uint32_t timer;
    timer_resolution_t resolution;
    bool claimed;
    timer_cfg_t cfg;
    IRQn_Type irq;
    uint32_t freq_hz;
} dtimer_t;

static dtimer_t timers[] = {
    { .timer = TIMER0,  .irq = TIMER0_UP_TIMER9_IRQn,       .resolution = Timer_16bit },
    { .timer = TIMER1,  .irq = TIMER1_IRQn,                 .resolution = Timer_32bit },
    { .timer = TIMER2,  .irq = TIMER2_IRQn,                 .resolution = Timer_16bit },
    { .timer = TIMER3,  .irq = TIMER3_IRQn,                 .resolution = Timer_16bit },
    { .timer = TIMER4,  .irq = TIMER4_IRQn,                 .resolution = Timer_16bit },
    { .timer = TIMER5,  .irq = TIMER5_DAC_IRQn,             .resolution = Timer_32bit },
    { .timer = TIMER6,  .irq = TIMER6_IRQn,                 .resolution = Timer_16bit },
    { .timer = TIMER7,  .irq = TIMER7_UP_TIMER12_IRQn,      .resolution = Timer_16bit },
};

static dtimer_t *timer_get (uint32_t timer)
{
    uint_fast8_t idx, n_timers = sizeof(timers) / sizeof(dtimer_t);

    for(idx = 0; idx < n_timers; idx++) {
        if(timers[idx].timer == timer)
            return &timers[idx];
    }

    return NULL;
}

timer_resolution_t timer_get_resolution (uint32_t timer)
{
    dtimer_t *dtimer;

    return (dtimer = timer_get(timer)) ? dtimer->resolution : (timer_resolution_t)0;
}

timer_cap_t timer_get_cap (uint32_t timer)
{
    dtimer_t *dtimer;

    return (dtimer = timer_get(timer)) ? (timer_cap_t){ .periodic = 1, .up = 1, .comp1 = 1, .comp2 = 1, .comp3 = 1 } : (timer_cap_t){0};
}

hal_timer_t timer_claim (uint32_t timer)
{
    bool claimed = false;
    dtimer_t *dtimer;

    if((claimed = (dtimer = timer_get(timer)) && !dtimer->claimed))
        dtimer->claimed = true;

    return claimed ? dtimer : NULL;
}

bool timer_is_claimed (uint32_t timer)
{
    dtimer_t *dtimer = timer_get(timer);

    return dtimer && dtimer->claimed;
}

uint32_t timer_get_clock_hz (uint32_t timer)
{
    dtimer_t *dtimer = timer_get(timer);

    return dtimer ? dtimer->freq_hz : 0;
}

uint32_t timer_clk_enable (uint32_t timer)
{
    dtimer_t *dtimer = timer_get(timer);

    if(!dtimer)
        return 0;

    switch(timer) {
        case TIMER0:
            rcu_periph_clock_enable(RCU_TIMER0);
            dtimer->freq_hz = rcu_clock_freq_get(CK_APB2) * 2;
            break;
        case TIMER1:
            rcu_periph_clock_enable(RCU_TIMER1);
            dtimer->freq_hz = rcu_clock_freq_get(CK_APB1) * 2;
            break;
        case TIMER2:
            rcu_periph_clock_enable(RCU_TIMER2);
            dtimer->freq_hz = rcu_clock_freq_get(CK_APB1) * 2;
            break;
        case TIMER3:
            rcu_periph_clock_enable(RCU_TIMER3);
            dtimer->freq_hz = rcu_clock_freq_get(CK_APB1) * 2;
            break;
        case TIMER4:
            rcu_periph_clock_enable(RCU_TIMER4);
            dtimer->freq_hz = rcu_clock_freq_get(CK_APB1) * 2;
            break;
        case TIMER5:
            rcu_periph_clock_enable(RCU_TIMER5);
            dtimer->freq_hz = rcu_clock_freq_get(CK_APB1) * 2;
            break;
        case TIMER6:
            rcu_periph_clock_enable(RCU_TIMER6);
            dtimer->freq_hz = rcu_clock_freq_get(CK_APB1) * 2;
            break;
        case TIMER7:
            rcu_periph_clock_enable(RCU_TIMER7);
            dtimer->freq_hz = rcu_clock_freq_get(CK_APB1) * 2;
            break;
        case TIMER8:
        case TIMER9:
        case TIMER10:
        case TIMER11:
        case TIMER12:
        case TIMER13:
            // Shared IRQs with TIMER0/TIMER7 on GD32F425; not supported in this table.
            return 0;
        default:
            return 0;
    }

    return dtimer->freq_hz;
}

hal_timer_t timerClaim (timer_cap_t cap, uint32_t timebase)
{
    (void)cap;

    hal_timer_t t;
    uint_fast8_t idx, n_timers = sizeof(timers) / sizeof(dtimer_t);

    for(idx = 0; idx < n_timers; idx++) {
        if((t = timers[idx].claimed ? NULL : &timers[idx])) {

            uint32_t f = timer_clk_enable(timers[idx].timer) / 1000;

            timers[idx].claimed = On;

            timer_parameter_struct init;
            timer_struct_para_init(&init);
            init.prescaler = ((f * timebase) / 1000000) - 1;
            init.alignedmode = TIMER_COUNTER_EDGE;
            init.counterdirection = TIMER_COUNTER_UP;
            init.period = 0xFFFFFFFFU;
            init.clockdivision = TIMER_CKDIV_DIV1;
            init.repetitioncounter = 0;
            timer_init(timers[idx].timer, &init);

            nvic_irq_enable(timers[idx].irq, 0, 1);
            break;
        }
    }

    return t;
}

bool timerCfg (hal_timer_t timer, timer_cfg_t *cfg)
{
    bool ok;

    if((ok = timer != NULL)) {

        memcpy(&((dtimer_t *)timer)->cfg, cfg, sizeof(timer_cfg_t));

        if(cfg->single_shot)
            timer_single_pulse_mode_config(((dtimer_t *)timer)->timer, TIMER_SP_MODE_SINGLE);
        else
            timer_single_pulse_mode_config(((dtimer_t *)timer)->timer, TIMER_SP_MODE_REPETITIVE);

        if(cfg->timeout_callback)
            timer_interrupt_enable(((dtimer_t *)timer)->timer, TIMER_INT_UP);
        else
            timer_interrupt_disable(((dtimer_t *)timer)->timer, TIMER_INT_UP);

        nvic_irq_enable(((dtimer_t *)timer)->irq, 0, 1);
    }

    return ok;
}

bool timerStart (hal_timer_t timer, uint32_t period)
{
    dtimer_t *dtimer = (dtimer_t *)timer;

    if(dtimer == NULL)
        return false;

    if(dtimer->cfg.period != period || (TIMER_CTL0(dtimer->timer) & TIMER_CTL0_CEN) == 0) {

        dtimer->cfg.period = period;

        timer_autoreload_value_config(dtimer->timer, period - 1);
        timer_counter_value_config(dtimer->timer, 0);
        timer_interrupt_flag_clear(dtimer->timer, TIMER_INT_FLAG_UP);
        timer_enable(dtimer->timer);
    }

    return true;
}

bool timerStop (hal_timer_t timer)
{
    dtimer_t *dtimer = (dtimer_t *)timer;

    if(dtimer == NULL)
        return false;

    timer_interrupt_disable(dtimer->timer, TIMER_INT_UP);
    timer_disable(dtimer->timer);

    return true;
}

// -----------------------------------------------------------------------------
// IRQ handlers
// -----------------------------------------------------------------------------

static void timer_irq_handler (dtimer_t *dtimer)
{
    if (dtimer == NULL || dtimer->cfg.timeout_callback == NULL)
        return;

    if (timer_interrupt_flag_get(dtimer->timer, TIMER_INT_FLAG_UP)) {
        timer_interrupt_flag_clear(dtimer->timer, TIMER_INT_FLAG_UP);
        dtimer->cfg.timeout_callback(dtimer->cfg.context);
    }
}

#define TIMER_IRQ_DISPATCH(n) \
    void TIMER##n##_IRQHandler(void) { \
        timer_irq_handler(timer_get(TIMER##n)); \
    }

TIMER_IRQ_DISPATCH(1)
TIMER_IRQ_DISPATCH(2)
TIMER_IRQ_DISPATCH(4)
TIMER_IRQ_DISPATCH(5)
TIMER_IRQ_DISPATCH(6)

void TIMER0_UP_TIMER9_IRQHandler (void)
{
    timer_irq_handler(timer_get(TIMER0));
}

void TIMER7_UP_TIMER12_IRQHandler (void)
{
    timer_irq_handler(timer_get(TIMER7));
}

// -----------------------------------------------------------------------------
// Stepper timer (TIMER3)
// -----------------------------------------------------------------------------

void stepper_timer_init (void)
{
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL2);
    timerCLKEN(STEPPER_TIMER_N);

    timer_parameter_struct init;
    timer_struct_para_init(&init);
    init.prescaler = 0;                       // 84 MHz timer clock (APB1 x 2)
    init.alignedmode = TIMER_COUNTER_EDGE;
    init.counterdirection = TIMER_COUNTER_UP;
    init.period = 0xFFFFFFFFU;
    init.clockdivision = TIMER_CKDIV_DIV1;
    init.repetitioncounter = 0;
    timer_init(STEPPER_TIMER, &init);

    nvic_irq_enable(timerINT(STEPPER_TIMER_N), 2U, 0U);
    timer_interrupt_enable(STEPPER_TIMER, TIMER_INT_UP);
    timer_enable(STEPPER_TIMER);
}

void stepper_timer_load (uint32_t ticks)
{
    timer_autoreload_value_config(STEPPER_TIMER, ticks);
    timer_counter_value_config(STEPPER_TIMER, 0);
    timer_event_software_generate(STEPPER_TIMER, TIMER_EVENT_SRC_UPG);
}

