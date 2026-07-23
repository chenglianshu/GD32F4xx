/*

  encoders.c - driver code for GD32F4xx ARM processors

  *** WIP ***

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

#if ENCODER_ENABLE

#include "grbl/task.h"
#include "grbl/encoders.h"
#include "grbl/spindle_sync.h"

// -----------------------------------------------------------------------------
// Timer register mapping helpers (GD32 SPL register macros)
// -----------------------------------------------------------------------------

#define tmr_cr1(timer)      TIMER_CTL0(timer)
#define tmr_cr2(timer)      TIMER_CTL1(timer)
#define tmr_smcr(timer)     TIMER_SMCFG(timer)
#define tmr_dier(timer)     TIMER_DMAINTEN(timer)
#define tmr_sr(timer)       TIMER_INTF(timer)
#define tmr_egr(timer)      TIMER_SWEVG(timer)
#define tmr_ccmr1(timer)    TIMER_CHCTL0(timer)
#define tmr_ccmr2(timer)    TIMER_CHCTL1(timer)
#define tmr_ccer(timer)     TIMER_CHCTL2(timer)
#define tmr_cnt(timer)      TIMER_CNT(timer)
#define tmr_psc(timer)      TIMER_PSC(timer)
#define tmr_arr(timer)      TIMER_CAR(timer)
#define tmr_ccr1(timer)     TIMER_CH0CV(timer)
#define tmr_ccr2(timer)     TIMER_CH1CV(timer)
#define tmr_ccr3(timer)     TIMER_CH2CV(timer)
#define tmr_ccr4(timer)     TIMER_CH3CV(timer)

#define TIM_CR1_DIR         TIMER_CTL0_DIR
#define TIM_CR1_CEN         TIMER_CTL0_CEN
#define TIM_CR1_URS         TIMER_CTL0_UPS
#define TIM_CR1_CKD_1       BIT(9)
#define TIM_EGR_UG          TIMER_SWEVG_UPG
#define TIM_DIER_UIE        TIMER_DMAINTEN_UPIE
#define TIM_CCER_CC1E       TIMER_CHCTL2_CH0EN

#define TIM_SMCR_SMS_0      BIT(0)
#define TIM_SMCR_SMS_1      BIT(1)
#define TIM_SMCR_SMS_2      BIT(2)
#define TIM_SMCR_ETF_0      BIT(8)
#define TIM_SMCR_ETF_1      BIT(9)
#define TIM_SMCR_ETF_2      BIT(10)
#define TIM_SMCR_ETF_3      BIT(11)
#define TIM_SMCR_TS_0       BIT(4)
#define TIM_SMCR_TS_1       BIT(5)
#define TIM_SMCR_TS_2       BIT(6)
#define TIM_SMCR_ECE        TIMER_SMCFG_SMC1

#define TIM_CCMR1_CC1S_0    BIT(0)
#define TIM_CCMR1_CC2S_0    BIT(8)

typedef struct {
    uint8_t pin_a;
    uint8_t pin_b;
    uint8_t af;
    uint32_t port_a;
    uint32_t port_b;
    uint32_t timer;
} gd32_qei_hw_t;

typedef struct {
    uint8_t pin;
    uint8_t af;
    bool ecm; // external clock mode
    uint32_t port;
    uint32_t timer;
} gd32_pcnt_hw_t;

typedef struct {
    encoder_t encoder;
    encoder_data_t data;
    encoder_cfg_t settings;
    encoder_event_t event;
    void *context;
    int32_t count;
    int16_t count_h;
    int32_t vel_count;
    volatile uint32_t vel_timeout;
    uint32_t vel_timestamp;
    spindle_encoder_t sp;
    hal_timer_t t;
    __IO uint32_t *cnt; // shortcut to timer CNT register
    __IO uint32_t *ccr; // shortcut to timer CCR3 register
    const gd32_qei_hw_t *st_encoder;
    encoder_on_event_ptr on_event;
} gd32_encoder_t;

#if SPINDLE_ENCODER_ENABLE

typedef struct {
    int16_t count_h;
    __IO uint32_t *cr1; // shortcut to timer CR1 register
    __IO uint32_t *cnt; // shortcut to timer CNT register
    __IO uint32_t *ccr; // shortcut to timer CCR3 register
    uint32_t timer;
    settings_changed_ptr settings_changed;
} spindle_encoder_hw_t;

static struct {
    uint32_t count_h;
    __IO uint32_t *cnt; // shortcut to timer CNT register
    __IO uint32_t *egr; // shortcut to timer EGR register
} timestamp;

#define TIMESTAMP               (*timestamp.cnt | timestamp.count_h)
#define TIMESTAMP_RESOLUTION    1 // microseconds

#ifdef SPINDLE_PULSE_PIN

static const gd32_pcnt_hw_t counters[] = {
#ifdef TIMER7
    { .port = GPIOA, .pin = 0, .af = GPIO_AF_3, .timer = timer(7), .ecm = false },
#endif
    { .port = GPIOA, .pin = 5, .af = GPIO_AF_1, .timer = timer(1), .ecm = false },
    { .port = GPIOA, .pin = 15, .af = GPIO_AF_1, .timer = timer(1), .ecm = false },
    { .port = GPIOB, .pin = 8, .af = GPIO_AF_1, .timer = timer(1), .ecm = false },
    { .port = GPIOA, .pin = 12, .af = GPIO_AF_1, .timer = timer(0), .ecm = false },
    { .port = GPIOE, .pin = 7, .af = GPIO_AF_1, .timer = timer(0), .ecm = false },
    { .port = GPIOB, .pin = 4, .af = GPIO_AF_2, .timer = timer(2), .ecm = true },
    { .port = GPIOD, .pin = 2, .af = GPIO_AF_2, .timer = timer(2), .ecm = false },
    { .port = GPIOE, .pin = 0, .af = GPIO_AF_2, .timer = timer(3), .ecm = false }
};

#endif

static spindle_encoder_hw_t sp_encoder;
static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .tics_per_irq = 4
};
static on_spindle_programmed_ptr on_spindle_programmed = NULL;

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    bool stopped;
    uint32_t pulse_length, rpm_timer_delta;

    spindle_encoder_counter_t encoder;

    __disable_irq();

    if(spindle_data.ccw != !!(tmr_cr1(sp_encoder.timer) & TIM_CR1_DIR)) {
        if((spindle_data.ccw = !spindle_data.ccw) /*&& spindle_encoder.timer.pulse_length == 0*/)
            *sp_encoder.ccr = (uint16_t)(*sp_encoder.cnt - spindle_encoder.tics_per_irq);
        else
            *sp_encoder.ccr = (uint16_t)(*sp_encoder.cnt + spindle_encoder.tics_per_irq);
    }

    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    rpm_timer_delta = TIMESTAMP - spindle_encoder.timer.last_pulse;

    __enable_irq();

    // If no spindle pulses during last 250 ms assume RPM is 0
    if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (uint16_t)(((uint16_t)*sp_encoder.cnt - (uint16_t)encoder.last_count)) * pulse_length;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = encoder.index_count;
            spindle_data.pulse_count = encoder.pulse_count + (uint32_t)((uint16_t)*sp_encoder.cnt - (uint16_t)encoder.last_count);
            spindle_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            if(!stopped)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            break;

        case SpindleData_AtSpeed:
            if(!stopped)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            spindle_data.state_programmed.at_speed = !spindle_data.at_speed_enabled || (spindle_data.rpm >= spindle_data.rpm_low_limit && spindle_data.rpm <= spindle_data.rpm_high_limit);
            spindle_data.state_programmed.encoder_error = spindle_encoder.error_count > 0;
            break;

        case SpindleData_AngularPosition:
            spindle_data.angular_position = (float)encoder.index_count +
                    ((float)((uint16_t)encoder.last_count - (uint16_t)encoder.last_index) +
                              (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
                                spindle_encoder.pulse_distance;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    while(spindle_encoder.spin_lock);

    uint32_t timeout = hal_get_tick() + 1000; // 1 second

    uint32_t index_count = spindle_encoder.counter.index_count + 2;
    if(spindleGetData(SpindleData_RPM)->rpm > 0.0f) { // wait for index pulse if running

        while(index_count != spindle_encoder.counter.index_count && hal_get_tick() <= timeout);

//        if(hal_get_tick() > timeout)
//            alarm?
    }

    timestamp.count_h = 0;
    *timestamp.egr |= TIM_EGR_UG; // Reset timestamp timer

    tmr_cr1(sp_encoder.timer) &= ~TIM_CR1_CEN;

    spindle_data.ccw = !!(tmr_cr1(sp_encoder.timer) & TIM_CR1_DIR);
    spindle_encoder.timer.last_pulse =
    spindle_encoder.timer.last_index = TIMESTAMP;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;

    tmr_egr(sp_encoder.timer) |= TIM_EGR_UG;
    *sp_encoder.ccr = spindle_data.ccw ? -spindle_encoder.tics_per_irq : spindle_encoder.tics_per_irq;
    tmr_cr1(sp_encoder.timer) |= TIM_CR1_CEN;
}

static void onSpindleProgrammed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    if(on_spindle_programmed)
        on_spindle_programmed(spindle, state, rpm, mode);

    if(spindle->get_data == spindleGetData) {
        spindle_set_at_speed_range(spindle, &spindle_data, rpm);
        spindle_data.state_programmed.on = state.on;
        spindle_data.state_programmed.ccw = state.ccw;
    }
}

static void spindle_encoder_cfg (settings_t *settings, settings_changed_flags_t changed)
{
    static const spindle_data_ptrs_t encoder_data = {
        .get = spindleGetData,
        .reset = spindleDataReset
    };

    static bool event_claimed = false;

    sp_encoder.settings_changed(settings, changed);

    if((hal.spindle_data.get = settings->spindle.ppr > 0 ? spindleGetData : NULL)) {

        hal.driver_cap.spindle_encoder_index_event = On;

        if(spindle_encoder.ppr != settings->spindle.ppr) {

            spindle_ptrs_t *spindle;

            hal.spindle_data.reset = spindleDataReset;
            if((spindle = spindle_get(0)))
                spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);

            if(!event_claimed) {
                event_claimed = true;
                on_spindle_programmed = grbl.on_spindle_programmed;
                grbl.on_spindle_programmed = onSpindleProgrammed;
            }

            spindle_encoder.ppr = settings->spindle.ppr;
            spindle_encoder.tics_per_irq = max(1, spindle_encoder.ppr / 32);
            if(spindle_encoder.tics_per_irq & 0x1) spindle_encoder.tics_per_irq++;
            spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
            spindle_encoder.maximum_tt = 250000UL / TIMESTAMP_RESOLUTION; // 250ms
            spindle_encoder.rpm_factor = (60.0f * 1000000.0f / TIMESTAMP_RESOLUTION) / (float)spindle_encoder.ppr;
            spindleDataReset();
        }
    } else {
        spindle_encoder.ppr = 0;
        hal.spindle_data.reset = NULL;
        hal.driver_cap.spindle_encoder_index_event = Off;
    }

    spindle_bind_encoder(spindle_encoder.ppr ? &encoder_data : NULL);
}

ISR_CODE void spindle_encoder_index_event (void)
{
    uint32_t rpm_count = *sp_encoder.cnt;
    spindle_encoder.timer.last_index = TIMESTAMP;

    if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
        spindle_encoder.error_count++;

    spindle_encoder.counter.last_index = rpm_count;
    spindle_encoder.counter.index_count++;

    if(hal.spindle_encoder_on_index)
        hal.spindle_encoder_on_index(spindle_encoder.counter.index_count);
}

ISR_CODE void spindle_encoder_irq (void *context)
{
    spindle_encoder.spin_lock = true;

    spindle_encoder_hw_t *qei = (spindle_encoder_hw_t *)context;

    __disable_irq();
    uint32_t tval = TIMESTAMP;
    uint16_t cval = *qei->cnt;
    __enable_irq();

    if(*qei->cr1 & TIM_CR1_DIR)
        *qei->ccr = (uint16_t)(*qei->ccr - spindle_encoder.tics_per_irq);
    else
        *qei->ccr = (uint16_t)(*qei->ccr + spindle_encoder.tics_per_irq);

    spindle_encoder.counter.pulse_count += (uint16_t)(cval - (uint16_t)spindle_encoder.counter.last_count);
    spindle_encoder.counter.last_count = cval;
    spindle_encoder.timer.pulse_length = tval - spindle_encoder.timer.last_pulse;
    spindle_encoder.timer.last_pulse = tval;

    spindle_encoder.spin_lock = false;
}

ISR_CODE static void spindle_encoder_overflow (void *context)
{
    spindle_encoder_hw_t *qei = (spindle_encoder_hw_t *)context;

    if(tmr_cr1(qei->timer) & TIM_CR1_DIR)
        qei->count_h--;
    else
        qei->count_h++;
}

ISR_CODE static void tstamp_overflow (void *context)
{
    timestamp.count_h = ((timestamp.count_h >> 16) + 1) << 16;
}

#endif // SPINDLE_ENCODER_ENABLE

#if QEI_ENABLE && defined(QEI_PORT)

static const gd32_qei_hw_t encoders[] = {
    { .port_a = GPIOA, .pin_a = 6, .port_b = GPIOA, .pin_b = 7, .af = GPIO_AF_2, .timer = timer(2) }
};

static uint_fast8_t n_encoders = 0;
static gd32_encoder_t qei[QEI_ENABLE] = {0};

static encoder_data_t *encoder_get_data (encoder_t *encoder)
{
    return &((gd32_encoder_t *)encoder->hw)->data;
}

ISR_CODE static void encoder_overflow (void *context)
{
    gd32_encoder_t *qei = (gd32_encoder_t *)context;

    if(tmr_cr1(qei->st_encoder->timer) & TIM_CR1_DIR)
        qei->count_h--;
    else
        qei->count_h++;
}

static bool encoder_configure (encoder_t *encoder, encoder_cfg_t *settings)
{
    gd32_encoder_t *qei = (gd32_encoder_t *)encoder->hw;

    if(qei->vel_timeout != settings->vel_timeout)
        qei->vel_timestamp = hal.get_elapsed_ticks();

    memcpy(&qei->settings, settings, sizeof(encoder_cfg_t));

    timer_cfg_t cfg = {
        .context = qei,
        .timeout_callback = encoder_overflow
    };

    timerCfg(qei->t, &cfg);

    return true;
}

static void encoder_reset (encoder_t *encoder)
{
    gd32_encoder_t *qei = (gd32_encoder_t *)encoder->hw;

    qei->vel_timeout = 0;
    qei->data.position = qei->count = qei->vel_count = 0;
    qei->vel_timestamp = hal_get_tick();
//    qei->vel_timeout = qei->encoder.axis != 0xFF ? QEI_VELOCITY_TIMEOUT : 0;
    tmr_cnt(qei->st_encoder->timer) = 0; //stop/start timer?
}

static void encoder_poll (void *data)
{
    gd32_encoder_t *qei = (gd32_encoder_t *)data;

    if(qei->on_event && qei->count != tmr_cnt(qei->st_encoder->timer)) {

        qei->data.position = (qei->count_h << 16) | tmr_cnt(qei->st_encoder->timer);

        qei->event.position_changed = On;

        // encoder->timer->CR1 & TIM_CR1_DIR -> 0 = up, !=0 down

        qei->on_event(&qei->encoder, &qei->event, qei->context);

        if(qei->vel_timeout && !(--qei->vel_timeout)) {
            qei->data.velocity = abs(qei->count - qei->vel_count) * 1000 / (hal_get_tick() - qei->vel_timestamp);
            qei->vel_timestamp = hal_get_tick();
            qei->vel_timeout = qei->settings.vel_timeout;
            if((qei->event.position_changed = qei->data.velocity == 0))
                qei->on_event(&qei->encoder, &qei->event, qei->context);
            qei->vel_count = qei->count;
        }

        qei->count = tmr_cnt(qei->st_encoder->timer);
    }
}

static bool encoder_claim (encoder_t *encoder, encoder_on_event_ptr event_handler, void *context)
{
    gd32_encoder_t *qei = (gd32_encoder_t *)encoder->hw;

    if(event_handler == NULL || qei->on_event)
        return false;

    timer_cfg_t cfg = {
        .context = qei,
        .encoder_mode = true,
        .timeout_callback = encoder_overflow
    };

    timerCfg(qei->t, &cfg);

    qei->context = context;
    qei->on_event = event_handler;
    qei->encoder.reset = encoder_reset;
    qei->encoder.get_data = encoder_get_data;
    qei->encoder.configure = encoder_configure;
    tmr_dier(qei->st_encoder->timer) |= TIM_DIER_UIE;
    tmr_cr1(qei->st_encoder->timer) = TIM_CR1_CEN|TIM_CR1_URS;

    task_add_systick(encoder_poll, qei);

    return true;
}

static bool encoder_add (uint32_t id)
{
    if(id > sizeof(encoders) / sizeof(gd32_qei_hw_t) || n_encoders == QEI_ENABLE)
        return false;

    hal_timer_t timer;
    const gd32_qei_hw_t *encoder = &encoders[id];

    if((timer = timer_claim(encoder->timer))) {

        gd32_encoder_t *hw = &qei[n_encoders++];

        timer_clk_enable(encoder->timer);

        gpio_mode_set((uint32_t)encoder->port_a, GPIO_MODE_AF, GPIO_PUPD_NONE, 1U << encoder->pin_a);
        gpio_output_options_set((uint32_t)encoder->port_a, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, 1U << encoder->pin_a);
        gpio_af_set((uint32_t)encoder->port_a, encoder->af, 1U << encoder->pin_a);

        gpio_mode_set((uint32_t)encoder->port_b, GPIO_MODE_AF, GPIO_PUPD_NONE, 1U << encoder->pin_b);
        gpio_output_options_set((uint32_t)encoder->port_b, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, 1U << encoder->pin_b);
        gpio_af_set((uint32_t)encoder->port_b, encoder->af, 1U << encoder->pin_b);

        tmr_smcr(encoder->timer) = TIM_SMCR_SMS_0|TIM_SMCR_SMS_1|TIM_SMCR_ETF_0|TIM_SMCR_ETF_1;
        tmr_arr(encoder->timer) = 0xFFFF;
        tmr_ccmr1(encoder->timer) = TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_0;

        hw->t = timer;
        hw->cnt = &tmr_cnt(encoder->timer);
        hw->ccr = &tmr_ccr3(encoder->timer);
        hw->st_encoder = encoder;
        hw->encoder.hw = hw;
        hw->encoder.claim = encoder_claim;
//        hw->encoder.caps.spindle_rpm = On;

#if SPINDLE_ENCODER_ENABLE

        if(sp_encoder.settings_changed == NULL && timer_get_cap(encoder->timer).comp3) {

            sp_encoder.settings_changed = hal.settings_changed;
            hal.settings_changed = spindle_encoder_cfg;

            sp_encoder.cr1 = &tmr_cr1(encoder->timer);
            sp_encoder.cnt = &tmr_cnt(encoder->timer);
            sp_encoder.ccr = &tmr_ccr3(encoder->timer);
            sp_encoder.timer = encoder->timer;

            timer_cfg_t cfg = {
                .context = &sp_encoder,
                .timeout_callback = timer_get_resolution(encoder->timer) == Timer_16bit ? spindle_encoder_overflow : NULL,
                .irq2_callback = spindle_encoder_irq
            };

            timerCfg(timer, &cfg);

            tmr_cr1(encoder->timer) = TIM_CR1_CEN|TIM_CR1_URS;

        } else
#endif
        encoder_register(&hw->encoder);
    }

    return !!timer;
}

#endif // QEI_ENABLE && defined(QEI_PORT)

void encoder_pin_claimed (uint8_t port, xbar_t *pin)
{
#if QEI_ENABLE && defined(QEI_A_PIN) && defined(QEI_B_PIN)
    _encoder_pin_claimed(port, pin);
#endif
}

// Stub for the higher-level encoder plugin init. The full encoder/encoder.c plugin
// (settings, MPG jogging, etc.) is not ported yet; this stub allows the hardware
// driver in this file to compile and link when ENCODER_ENABLE is non-zero.
bool encoder_init (void)
{
    return false;
}

#if SPINDLE_ENCODER_ENABLE || (QEI_ENABLE && defined(QEI_PORT))

void driver_encoders_init (void)
{
#if SPINDLE_ENCODER_ENABLE

    hal_timer_t timer;

    if((timer = timer_claim(RPM_TIMER))) {

        uint32_t freq_hz = timer_clk_enable(RPM_TIMER);
        tmr_psc(RPM_TIMER) = freq_hz / 1000000UL * TIMESTAMP_RESOLUTION - 1;
        if(timer_get_resolution(RPM_TIMER) == Timer_16bit) {
            timer_cfg_t cfg = {
                .timeout_callback = tstamp_overflow
            };
            timerCfg(timer, &cfg);
        }
        tmr_cr1(RPM_TIMER) = TIM_CR1_CKD_1|TIM_CR1_URS|TIM_CR1_CEN;

        timestamp.cnt = &tmr_cnt(RPM_TIMER);
        timestamp.egr = &tmr_egr(RPM_TIMER);
    }

#ifdef SPINDLE_PULSE_PIN

    uint_fast8_t idx = sizeof(counters) / sizeof(gd32_pcnt_hw_t);

    if(idx) do {
        idx--;
        if(counters[idx].port == SPINDLE_PULSE_PORT && counters[idx].pin == SPINDLE_PULSE_PIN && (timer = timer_claim(counters[idx].timer))) {

            sp_encoder.timer = counters[idx].timer;

            timer_clk_enable(sp_encoder.timer);

            sp_encoder.cr1 = &tmr_cr1(sp_encoder.timer);
            sp_encoder.cnt = &tmr_cnt(sp_encoder.timer);
            sp_encoder.ccr = &tmr_ccr1(sp_encoder.timer);
            tmr_psc(sp_encoder.timer) = 0;
            tmr_arr(sp_encoder.timer) = 65535;
//            tmr_ccer(sp_encoder.timer) = TIM_CCER_CC1E; // Fails with F411 for some reason
            tmr_smcr(sp_encoder.timer) = counters[idx].ecm ? (TIM_SMCR_SMS_0|TIM_SMCR_SMS_1|TIM_SMCR_SMS_2|TIM_SMCR_ETF_2|TIM_SMCR_ETF_3|TIM_SMCR_TS_0|TIM_SMCR_TS_2) : TIM_SMCR_ECE;

            nvic_irq_enable(RPM_COUNTER_IRQn, 0U, 0U);

            gpio_mode_set((uint32_t)SPINDLE_PULSE_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, 1U << SPINDLE_PULSE_PIN);
            gpio_output_options_set((uint32_t)SPINDLE_PULSE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, 1U << SPINDLE_PULSE_PIN);
            gpio_af_set((uint32_t)SPINDLE_PULSE_PORT, counters[idx].af, 1U << SPINDLE_PULSE_PIN);

            timer_cfg_t cfg = {
                .context = &sp_encoder,
                .timeout_callback = timer_get_resolution(sp_encoder.timer) == Timer_16bit ? spindle_encoder_overflow : NULL,
                .irq0_callback = spindle_encoder_irq
            };

            timerCfg(timer, &cfg);

            static const periph_pin_t ssp = {
                .function = Input_SpindlePulse,
                .group = PinGroup_SpindlePulse,
                .port = (void *)SPINDLE_PULSE_PORT,
                .pin = SPINDLE_PULSE_PIN,
                .mode = { .mask = PINMODE_NONE }
            };

            hal.periph_port.register_pin(&ssp);

            sp_encoder.settings_changed = hal.settings_changed;
            hal.settings_changed = spindle_encoder_cfg;

            break;
        }
    } while(idx);

#endif // SPINDLE_PULSE_PIN
#endif // SPINDLE_ENCODER_ENABLE

#ifdef QEI_PORT
    encoder_add(QEI_PORT);
#endif
}

#endif // SPINDLE_ENCODER_ENABLE || (QEI_ENABLE && defined(QEI_PORT))

#endif // ENCODER_ENABLE
