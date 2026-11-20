/*

  timers.c - driver code for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2024-2026 Terje Io

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

#include "driver.h"

typedef struct {
    uint32_t timer;
    bool claimed;
    timer_cfg_t cfg;
    timer_cap_t cap;
    IRQn_Type irq;
    uint32_t freq_hz;
} dtimer_t;

#define LAST_TIDX -1

// GD32F425 timers: TIMER0/TIMER7 advanced, TIMER1-4 general (TIMER1/TIMER4 32bit), TIMER5/6 basic.
// The chip also has TIMER8-13 - they are deliberately NOT in the pool because
// their interrupts share vectors with TIMER0/TIMER7 events (TIMER8..10 ride the
// TIMER0_BRK/UP/TRG_COM vectors, TIMER11..13 the TIMER7 ones), which would
// require merged-ISR dispatch. Pool of 8 minus stepper/RPM claims is ample.
// Timer N corresponds to STM32 TIM(N+1); capabilities are hardcoded (no STM32-HAL instance macros on GD32).
static dtimer_t timers[] = {
#if !IS_TIMER_CLAIMED(TIMER0_BASE)
    {
        .timer = TIMER0,
        .irq = TIMER0_UP_TIMER9_IRQn,
        .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1, .resolution = Timer_16bit }
    },
#endif
#if !IS_TIMER_CLAIMED(TIMER1_BASE)
    {
        .timer = TIMER1,
        .irq = TIMER1_IRQn,
        .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1, .resolution = Timer_32bit }
    },
#endif
#if !IS_TIMER_CLAIMED(TIMER2_BASE)
    {
        .timer = TIMER2,
        .irq = TIMER2_IRQn,
        .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1, .resolution = Timer_16bit }
    },
#endif
#if !IS_TIMER_CLAIMED(TIMER3_BASE)
    {
        .timer = TIMER3,
        .irq = TIMER3_IRQn,
        .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1, .resolution = Timer_16bit }
    },
#endif
#if !IS_TIMER_CLAIMED(TIMER4_BASE)
    {
        .timer = TIMER4,
        .irq = TIMER4_IRQn,
        .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1, .resolution = Timer_32bit }
    },
#endif
#if !IS_TIMER_CLAIMED(TIMER5_BASE)
    {
        .timer = TIMER5,
        .irq = TIMER5_DAC_IRQn,
        .cap = {0}
    },
#endif
#if !IS_TIMER_CLAIMED(TIMER6_BASE)
    {
        .timer = TIMER6,
        .irq = TIMER6_IRQn,
        .cap = {0}
    },
#endif
#if !IS_TIMER_CLAIMED(TIMER7_BASE)
    {
        .timer = TIMER7,
        .irq = TIMER7_UP_TIMER12_IRQn,
        .cap = { .comp1 = 1, .comp2 = 1, .comp3 = 1, .resolution = Timer_16bit }
    }
#endif
};

static dtimer_t *timer_get (uint32_t timer)
{
    dtimer_t *dtimer = NULL;
    uint_fast8_t idx = sizeof(timers) / sizeof(dtimer_t);

    do {
        if(timers[--idx].timer == timer) {
            dtimer = &timers[idx];
            break;
        }
    } while(idx);

    return dtimer;
}

timer_resolution_t timer_get_resolution (uint32_t timer)
{
    dtimer_t *dtimer;

    return (timer_resolution_t)((dtimer = timer_get(timer)) ? dtimer->cap.resolution : 0);
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
    bool apb2_clock = false;
    dtimer_t *dtimer = timer_get(timer);

    if(dtimer) {

        switch(timer) {

#if !IS_TIMER_CLAIMED(TIMER0_BASE)
            case TIMER0:
                apb2_clock = true;
                rcu_periph_clock_enable(RCU_TIMER0);
                break;
#endif
#if !IS_TIMER_CLAIMED(TIMER1_BASE)
            case TIMER1:
                rcu_periph_clock_enable(RCU_TIMER1);
                break;
#endif
#if !IS_TIMER_CLAIMED(TIMER2_BASE)
            case TIMER2:
                rcu_periph_clock_enable(RCU_TIMER2);
                break;
#endif
#if !IS_TIMER_CLAIMED(TIMER3_BASE)
            case TIMER3:
                rcu_periph_clock_enable(RCU_TIMER3);
                break;
#endif
#if !IS_TIMER_CLAIMED(TIMER4_BASE)
            case TIMER4:
                rcu_periph_clock_enable(RCU_TIMER4);
                break;
#endif
#if !IS_TIMER_CLAIMED(TIMER5_BASE)
            case TIMER5:
                rcu_periph_clock_enable(RCU_TIMER5);
                break;
#endif
#if !IS_TIMER_CLAIMED(TIMER6_BASE)
            case TIMER6:
                rcu_periph_clock_enable(RCU_TIMER6);
                break;
#endif
#if !IS_TIMER_CLAIMED(TIMER7_BASE)
            case TIMER7:
                apb2_clock = true;
                rcu_periph_clock_enable(RCU_TIMER7);
                break;
#endif
        }

        dtimer->freq_hz = (apb2_clock ? rcu_clock_freq_get(CK_APB2) : rcu_clock_freq_get(CK_APB1)) * 2;
    }

    return dtimer ? dtimer->freq_hz : 0;
}

hal_timer_t timerClaim (timer_cap_t cap, uint32_t timebase)
{
    hal_timer_t t;
    uint_fast8_t idx, n_timers = sizeof(timers) / sizeof(dtimer_t);

    for(idx = 0; idx < n_timers; idx++) {
        if((t = timers[idx].claimed && timers[idx].cap.resolution != cap.resolution ? NULL : &timers[idx])) {

            uint32_t f = timer_clk_enable(timers[idx].timer) / 1000;
            uint32_t timer = timers[idx].timer;

            timers[idx].claimed = On;
            TIMER_CTL0(timer) |= TIMER_CTL0_SPM|TIMER_CTL0_DIR|TIMER_CKDIV_DIV4|TIMER_CTL0_ARSE /*|TIMER_CTL0_UPS*/;
            TIMER_PSC(timer) = ((f * timebase) / 1000000) - 1;
            TIMER_INTF(timer) &= ~(TIMER_INT_FLAG_UP|TIMER_INT_FLAG_CH0|TIMER_INT_FLAG_CH1);
            TIMER_CNT(timer) = 0;

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

        uint32_t t = ((dtimer_t *)timer)->timer;

        memcpy(&((dtimer_t *)timer)->cfg, cfg, sizeof(timer_cfg_t));

        if(cfg->single_shot)
            TIMER_CTL0(t) |= TIMER_CTL0_SPM;
        else
            TIMER_CTL0(t) &= ~TIMER_CTL0_SPM;

        if(cfg->irq0_callback && (ok = ((dtimer_t *)timer)->cap.comp1))
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
        //    TIMER_CNT(t) = TIMER_CAR(t);
            TIMER_INTF(t) &= ~(TIMER_INT_FLAG_UP|TIMER_INT_FLAG_CH0|TIMER_INT_FLAG_CH1);
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

__attribute__((always_inline)) static inline void _irq_handler (uint32_t timer, timer_cfg_t *cfg)
{
    uint32_t irq = TIMER_INTF(timer) & TIMER_DMAINTEN(timer);

    TIMER_INTF(timer) &= ~(TIMER_INT_FLAG_UP|TIMER_INT_FLAG_CH0|TIMER_INT_FLAG_CH1|TIMER_INT_FLAG_CH2);

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
  TIMER0_TIDX = LAST_TIDX,
  TIMER0_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIMER0_IDX

void TIMER0_UP_TIMER9_IRQHandler (void)
{
    _irq_handler(TIMER0, &timers[TIMER0_IDX].cfg);
}

#endif // TIMER0

#if !IS_TIMER_CLAIMED(TIMER1_BASE)

enum {
  TIMER1_TIDX = LAST_TIDX,
  TIMER1_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIMER1_IDX

void TIMER1_IRQHandler (void)
{
    _irq_handler(TIMER1, &timers[TIMER1_IDX].cfg);
}

#endif // TIMER1

#if !IS_TIMER_CLAIMED(TIMER2_BASE)

enum {
  TIMER2_TIDX = LAST_TIDX,
  TIMER2_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIMER2_IDX

void TIMER2_IRQHandler (void)
{
    _irq_handler(TIMER2, &timers[TIMER2_IDX].cfg);
}

#endif // TIMER2

#if !IS_TIMER_CLAIMED(TIMER3_BASE)

enum {
  TIMER3_TIDX = LAST_TIDX,
  TIMER3_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIMER3_IDX

void TIMER3_IRQHandler (void)
{
    _irq_handler(TIMER3, &timers[TIMER3_IDX].cfg);
}

#endif // TIMER3

#if !IS_TIMER_CLAIMED(TIMER4_BASE)

enum {
  TIMER4_TIDX = LAST_TIDX,
  TIMER4_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIMER4_IDX

void TIMER4_IRQHandler (void)
{
    _irq_handler(TIMER4, &timers[TIMER4_IDX].cfg);
}

#endif // TIMER4

#if !IS_TIMER_CLAIMED(TIMER5_BASE)

enum {
  TIMER5_TIDX = LAST_TIDX,
  TIMER5_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIMER5_IDX

void TIMER5_DAC_IRQHandler (void)
{
    _irq_handler(TIMER5, &timers[TIMER5_IDX].cfg);
}

#endif // TIMER5

#if !IS_TIMER_CLAIMED(TIMER6_BASE)

enum {
  TIMER6_TIDX = LAST_TIDX,
  TIMER6_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIMER6_IDX

void TIMER6_IRQHandler (void)
{
    _irq_handler(TIMER6, &timers[TIMER6_IDX].cfg);
}

#endif // TIMER6

#if !IS_TIMER_CLAIMED(TIMER7_BASE)

enum {
  TIMER7_TIDX = LAST_TIDX,
  TIMER7_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIMER7_IDX

void TIMER7_UP_TIMER12_IRQHandler (void)
{
    _irq_handler(TIMER7, &timers[TIMER7_IDX].cfg);
}

#endif // TIMER7
