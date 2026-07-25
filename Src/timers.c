// Src/timers.c
// Timer driver code for GD32F4xx grblHAL port.

#include "driver.h"
#include "grbl/hal.h"

typedef struct {
    uint32_t timer;
    timer_resolution_t resolution;
    bool claimed;
    timer_cfg_t cfg;
    timer_cap_t cap;
    IRQn_Type irq;
    uint32_t freq_hz;
} dtimer_t;

static dtimer_t timers[] = {
#if !IS_TIMER_CLAIMED(TIMER0_BASE)
    { .timer = TIMER0,  .irq = TIMER0_UP_TIMER9_IRQn,  .resolution = Timer_16bit,
      .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1 } },
#endif
#if !IS_TIMER_CLAIMED(TIMER1_BASE)
    { .timer = TIMER1,  .irq = TIMER1_IRQn,            .resolution = Timer_32bit,
      .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1 } },
#endif
#if !IS_TIMER_CLAIMED(TIMER2_BASE)
    { .timer = TIMER2,  .irq = TIMER2_IRQn,            .resolution = Timer_16bit,
      .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1 } },
#endif
#if IS_TIMER_CLAIMED(TIMER3_BASE)
    { .timer = TIMER3,  .irq = TIMER3_IRQn,            .resolution = Timer_16bit,
      .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1 } },
#endif
#if !IS_TIMER_CLAIMED(TIMER4_BASE)
    { .timer = TIMER4,  .irq = TIMER4_IRQn,            .resolution = Timer_16bit,
      .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1 } },
#endif
#if !IS_TIMER_CLAIMED(TIMER5_BASE)
    { .timer = TIMER5,  .irq = TIMER5_DAC_IRQn,        .resolution = Timer_16bit,
      .cap = {0} },
#endif
#if !IS_TIMER_CLAIMED(TIMER6_BASE)
    { .timer = TIMER6,  .irq = TIMER6_IRQn,            .resolution = Timer_16bit,
      .cap = {0} },
#endif
#if !IS_TIMER_CLAIMED(TIMER7_BASE)
    { .timer = TIMER7,  .irq = TIMER7_UP_TIMER12_IRQn, .resolution = Timer_16bit,
      .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1 } },
#endif
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

    return (dtimer = timer_get(timer)) ? dtimer->cap : (timer_cap_t){0};
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

            TIMER_CTL0(timers[idx].timer) |= TIMER_CTL0_SPM | TIMER_CTL0_DIR | TIMER_CTL0_ARSE;
            TIMER_INTF(timers[idx].timer) &= ~(TIMER_INT_FLAG_UP | TIMER_INT_FLAG_CH0 | TIMER_INT_FLAG_CH1 | TIMER_INT_FLAG_CH2);
            TIMER_CNT(timers[idx].timer) = 0;

            nvic_irq_enable(timers[idx].irq, 0, 1);
            break;
        }
    }

    return t;
}

bool timerCfg (hal_timer_t timer, timer_cfg_t *cfg)
{
    bool ok;
    uint32_t t = ((dtimer_t *)timer)->timer;

    if((ok = timer != NULL)) {

        memcpy(&((dtimer_t *)timer)->cfg, cfg, sizeof(timer_cfg_t));

        if(cfg->single_shot)
            TIMER_CTL0(t) |= TIMER_CTL0_SPM;
        else
            TIMER_CTL0(t) &= ~TIMER_CTL0_SPM;

        if(ok && cfg->irq0_callback && (ok = ((dtimer_t *)timer)->cap.comp1))
            TIMER_DMAINTEN(t) |= TIMER_INT_CH0;
        else
            TIMER_DMAINTEN(t) &= ~TIMER_INT_CH0;

        if(ok && cfg->irq1_callback && (ok = ((dtimer_t *)timer)->cap.comp2))
            TIMER_DMAINTEN(t) |= TIMER_INT_CH1;
        else
            TIMER_DMAINTEN(t) &= ~TIMER_INT_CH1;

        if(ok && cfg->irq2_callback && (ok = ((dtimer_t *)timer)->cap.comp3))
            TIMER_DMAINTEN(t) |= TIMER_INT_CH2;
        else
            TIMER_DMAINTEN(t) &= ~TIMER_INT_CH2;

        if(ok && cfg->timeout_callback)
            TIMER_DMAINTEN(t) |= TIMER_INT_UP;
        else
            TIMER_DMAINTEN(t) &= ~TIMER_INT_UP;

        nvic_irq_enable(((dtimer_t *)timer)->irq, 0, 1);
    }

    return ok;
}

bool timerStart (hal_timer_t timer, uint32_t period)
{
    uint32_t t = ((dtimer_t *)timer)->timer;

    if(!(TIMER_CTL0(t) & TIMER_CTL0_CEN) ||
         (TIMER_CTL0(t) & TIMER_CTL0_SPM) ||
          period != ((dtimer_t *)timer)->cfg.period) {

        ((dtimer_t *)timer)->cfg.period = period;

        period--;

        if(((dtimer_t *)timer)->cfg.irq1_callback) {
            TIMER_CH1CV(t) = period;
            period += ((dtimer_t *)timer)->cfg.irq1;
        }

        if(((dtimer_t *)timer)->cfg.irq0_callback) {
            TIMER_CH0CV(t) = period;
            period += ((dtimer_t *)timer)->cfg.irq0;
        }

        TIMER_CAR(t) = period;

        if(!(TIMER_CTL0(t) & TIMER_CTL0_CEN)) {
            TIMER_SWEVG(t) = TIMER_EVENT_SRC_UPG;
            TIMER_INTF(t) &= ~(TIMER_INT_FLAG_UP | TIMER_INT_FLAG_CH0 | TIMER_INT_FLAG_CH1 | TIMER_INT_FLAG_CH2);
            if(!(TIMER_DMAINTEN(t) & TIMER_INT_UP)) {
                TIMER_DMAINTEN(t) |= TIMER_INT_UP;
                TIMER_SWEVG(t) = TIMER_EVENT_SRC_UPG;
            }
            TIMER_CTL0(t) |= TIMER_CTL0_CEN;
        }
    }

    return true;
}

bool timerStop (hal_timer_t timer)
{
    uint32_t t = ((dtimer_t *)timer)->timer;

    TIMER_DMAINTEN(t) = 0;
    TIMER_CTL0(t) &= ~TIMER_CTL0_CEN;

    return true;
}

// -----------------------------------------------------------------------------
// IRQ handlers
// -----------------------------------------------------------------------------

__attribute__((always_inline)) static inline void _irq_handler (uint32_t timer, timer_cfg_t *cfg)
{
    uint32_t irq = TIMER_INTF(timer) & TIMER_DMAINTEN(timer);

    TIMER_INTF(timer) &= ~(TIMER_INT_FLAG_UP | TIMER_INT_FLAG_CH0 | TIMER_INT_FLAG_CH1 | TIMER_INT_FLAG_CH2);

    if(irq & TIMER_INT_FLAG_UP)
        cfg->timeout_callback(cfg->context);

    if(irq & TIMER_INT_FLAG_CH0)
        cfg->irq0_callback(cfg->context);

    if(irq & TIMER_INT_FLAG_CH1)
        cfg->irq1_callback(cfg->context);

    if(irq & TIMER_INT_FLAG_CH2)
        cfg->irq2_callback(cfg->context);
}

#if !IS_TIMER_CLAIMED(TIMER0_BASE)

enum {
  TIMER0_TIDX = -1,
  TIMER0_IDX
};

void TIMER0_UP_TIMER9_IRQHandler (void)
{
    _irq_handler(TIMER0, &timers[TIMER0_IDX].cfg);
}

#endif // TIMER0

#if !IS_TIMER_CLAIMED(TIMER1_BASE)

enum {
  TIMER1_TIDX = TIMER0_TIDX,
  TIMER1_IDX
};

void TIMER1_IRQHandler (void)
{
    _irq_handler(TIMER1, &timers[TIMER1_IDX].cfg);
}

#endif // TIMER1

#if !IS_TIMER_CLAIMED(TIMER2_BASE)

enum {
  TIMER2_TIDX = TIMER1_TIDX,
  TIMER2_IDX
};

void TIMER2_IRQHandler (void)
{
    _irq_handler(TIMER2, &timers[TIMER2_IDX].cfg);
}

#endif // TIMER2

#if !IS_TIMER_CLAIMED(TIMER4_BASE)

enum {
  TIMER4_TIDX = TIMER2_TIDX,
  TIMER4_IDX
};

void TIMER4_IRQHandler (void)
{
    _irq_handler(TIMER4, &timers[TIMER4_IDX].cfg);
}

#endif // TIMER4

#if !IS_TIMER_CLAIMED(TIMER5_BASE)

enum {
  TIMER5_TIDX = TIMER4_TIDX,
  TIMER5_IDX
};

void TIMER5_IRQHandler (void)
{
    _irq_handler(TIMER5, &timers[TIMER5_IDX].cfg);
}

#endif // TIMER5

#if !IS_TIMER_CLAIMED(TIMER6_BASE)

enum {
  TIMER6_TIDX = TIMER5_TIDX,
  TIMER6_IDX
};

void TIMER6_IRQHandler (void)
{
    _irq_handler(TIMER6, &timers[TIMER6_IDX].cfg);
}

#endif // TIMER6

#if !IS_TIMER_CLAIMED(TIMER7_BASE)

enum {
  TIMER7_TIDX = TIMER6_TIDX,
  TIMER7_IDX
};

void TIMER7_UP_TIMER12_IRQHandler (void)
{
    _irq_handler(TIMER7, &timers[TIMER7_IDX].cfg);
}

#endif // TIMER7

