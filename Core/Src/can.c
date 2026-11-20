/*

  can.c - CAN bus driver code for GD32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2022 Jon Escombe
  Copyright (c) 2024 Terje Io

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

#ifdef CAN_PORT

#include "grbl/task.h"
#include "grbl/canbus.h"

/* Position of the extended identifier in the 32-bit filter registers - the bxCAN layout
 * is identical to STM32, but the GD32 library provides no macro for it (STM32: CAN_RI0R_EXID_Pos).
 */
#define CAN_EXID_Pos 3

/* STM32 CAN1 is mapped to GD32 CAN0, the peripheral is passed to the library calls instead. */
static can_parameter_struct hcan1 = {
    .working_mode = CAN_NORMAL_MODE,
    .time_triggered = DISABLE,
    .auto_bus_off_recovery = DISABLE,
    .auto_wake_up = DISABLE,
    .auto_retrans = DISABLE,
    /* Polarity inverted vs. the STM32 HAL: ENABLE here clears the receive FIFO overwrite
     * disable (RFLM) bit, i.e. the same register value as ReceiveFifoLocked = DISABLE.
     */
    .rec_fifo_overwrite = ENABLE,
    .trans_fifo_order = DISABLE
};
static can_rx_enqueue_fn rx_enqueue;
static can_rx_ptr rx_callbacks[28] = {0};

static void can_msp_init (void);

bool can_put (canbus_message_t message, bool ext_id)
{
    uint8_t TxMailbox;
    can_trasnmit_message_struct TxHeader = {0};

    TxHeader.tx_dlen = message.len;
    TxHeader.tx_ff = ext_id ? CAN_FF_EXTENDED : CAN_FF_STANDARD;
    TxHeader.tx_ft = CAN_FT_DATA;
    if(ext_id)
        TxHeader.tx_efid = message.id;
    else
        TxHeader.tx_sfid = message.id;
    memcpy(TxHeader.tx_data, message.data, sizeof(TxHeader.tx_data));

    TxMailbox = can_message_transmit(CAN0, &TxHeader);

    return TxMailbox != CAN_NOMAILBOX;
}

bool can_add_filter (uint32_t id, uint32_t mask, bool ext_id, can_rx_ptr callback)
{
    static uint8_t bank = 0, fmi = 0;
    static can_filter_parameter_struct sFilterConfig = {
        .filter_mode = CAN_FILTERMODE_MASK,
        .filter_bits = CAN_FILTERBITS_32BIT,
        .filter_fifo_number = CAN_FIFO0,
        .filter_enable = ENABLE
    };

    if(bank == 14)
        return false;

    if(ext_id) {

        // not tested!

        id <<= CAN_EXID_Pos;
        mask <<= CAN_EXID_Pos;

        if(sFilterConfig.filter_bits == CAN_FILTERBITS_16BIT &&
            sFilterConfig.filter_list_low == sFilterConfig.filter_list_high &&
             sFilterConfig.filter_mask_low == sFilterConfig.filter_mask_high)
            fmi++;

        sFilterConfig.filter_number = bank;
        sFilterConfig.filter_list_low = (uint16_t)id;
        sFilterConfig.filter_list_high = id >> 16;
        sFilterConfig.filter_mask_low = (uint16_t)mask;
        sFilterConfig.filter_mask_high = mask >> 16;
        sFilterConfig.filter_bits = CAN_FILTERBITS_32BIT;

    } else {

        id <<= 5;
        mask <<= 5;

        if(fmi && sFilterConfig.filter_list_low == sFilterConfig.filter_list_high && sFilterConfig.filter_mask_low == sFilterConfig.filter_mask_high) {
            bank--;
            sFilterConfig.filter_list_high = id;
            sFilterConfig.filter_mask_high = mask;
        } else {
            sFilterConfig.filter_number = bank;
            sFilterConfig.filter_list_low = id;
            sFilterConfig.filter_mask_low = mask;
            sFilterConfig.filter_list_high = id;
            sFilterConfig.filter_mask_high = mask;
            rx_callbacks[fmi + 1] = callback;
        }
        sFilterConfig.filter_bits = CAN_FILTERBITS_16BIT;
    }
    rx_callbacks[fmi] = callback;
    fmi++;
    bank++;

    /* SlaveStartFilterBank = 14 - written by every HAL_CAN_ConfigFilter() call in the
     * STM32 driver, the GD32 library has a separate call for it.
     */
    can1_filter_start_bank(14);
    can_filter_init(&sFilterConfig);

    return true;
}

bool can_stop (void)
{
    // HAL_CAN_Stop(): leave INIT mode only, registers/filters preserved.
    // NOTE: no can_deinit() here - the SPL deinit is an RCU peripheral reset that
    // wipes all filter banks while the static bank/fmi counters below survive,
    // breaking a stop->start cycle. STM32's HAL_CAN_DeInit (no MspDeInit defined)
    // never touches the hardware, so neither do we.
    can_working_mode_set(CAN0, CAN_MODE_INITIALIZE);

    return true;
}

bool can_set_baud (uint32_t baud)
{
    report_message("A hard reset of the controller is required after changing the CAN baud rate", Message_Info);

    return true;
}

bool can_start (uint32_t baud, can_rx_enqueue_fn callback)
{
    uint8_t unknown_rate = 0;

    rx_enqueue = callback;
    /*
     * Can bit time calculations taken from http://www.bittiming.can-wiki.info/ - pre-calculated
     * for supported CAN peripheral clock speeds and baud rates.
     *
     * Unable to query the CAN peripheral clock speed until it's initialised, however the
     * GD32F4 CAN peripheral appears to always be clocked from PCLK1/APB1.
     */
    uint32_t pclk1_freq = rcu_clock_freq_get(CK_APB1);

    switch (pclk1_freq) {

        case 25000000: // 25MHz APB1 clock

            switch (baud) {

                case 125000:
                    hcan1.prescaler = 20;
                    hcan1.time_segment_1 = CAN_BT_BS1_8TQ;
                    hcan1.time_segment_2 = CAN_BT_BS2_1TQ;
                    hcan1.resync_jump_width = CAN_BT_SJW_1TQ;
                    break;

                case 250000:
                    hcan1.prescaler = 10;
                    hcan1.time_segment_1 = CAN_BT_BS1_8TQ;
                    hcan1.time_segment_2 = CAN_BT_BS2_1TQ;
                    hcan1.resync_jump_width = CAN_BT_SJW_1TQ;
                    break;

                case 500000:
                    hcan1.prescaler = 5;
                    hcan1.time_segment_1 = CAN_BT_BS1_8TQ;
                    hcan1.time_segment_2 = CAN_BT_BS2_1TQ;
                    hcan1.resync_jump_width = CAN_BT_SJW_1TQ;
                    break;

                default:
                    /* Unsupported baud rate */
                    unknown_rate = 1;
                    break;
            }
            break;

        case 45000000: // 45MHz APB1 clock

            switch (baud) {

                case 125000:
                    hcan1.prescaler = 20;
                    hcan1.time_segment_1 = CAN_BT_BS1_15TQ;
                    hcan1.time_segment_2 = CAN_BT_BS2_2TQ;
                    hcan1.resync_jump_width = CAN_BT_SJW_1TQ;
                    break;

                case 250000:
                    hcan1.prescaler = 10;
                    hcan1.time_segment_1 = CAN_BT_BS1_15TQ;
                    hcan1.time_segment_2 = CAN_BT_BS2_2TQ;
                    hcan1.resync_jump_width = CAN_BT_SJW_1TQ;
                    break;

                case 500000:
                    hcan1.prescaler = 5;
                    hcan1.time_segment_1 = CAN_BT_BS1_15TQ;
                    hcan1.time_segment_2 = CAN_BT_BS2_2TQ;
                    hcan1.resync_jump_width = CAN_BT_SJW_1TQ;
                    break;

                case 1000000:
                    hcan1.prescaler = 3;
                    hcan1.time_segment_1 = CAN_BT_BS1_12TQ;
                    hcan1.time_segment_2 = CAN_BT_BS2_2TQ;
                    hcan1.resync_jump_width = CAN_BT_SJW_1TQ;
                    break;

                default:
                    /* Unsupported baud rate */
                    unknown_rate = 1;
                    break;
            }
            break;

        default:
            /* unsupported CAN peripheral clock frequency */
            unknown_rate = 1;
            break;
    }

    if (unknown_rate) {
//        printf("can_start(), error - unable to calculate bit timings\n");
        return(0);
    }

    can_msp_init();

    if (can_init(CAN0, &hcan1) != SUCCESS)
    {
        return(0);
    }

    /* Start the CAN peripheral (can_init() leaves initialization mode, this call
     * is the equivalent of the HAL_CAN_Start() sequence in the original driver). */
    if(can_working_mode_set(CAN0, CAN_MODE_NORMAL) != SUCCESS)
    {
        return(0);
    }

    /* Add the callback for received data */

    can_interrupt_enable(CAN0, CAN_INT_RFNE0);

    return true;
}

/**
* @brief CAN MSP Initialization
* This function configures the hardware resources used in this example
* (the GD32 library has no MSP callback, it is called from can_start() where
* HAL_CAN_Init() would call HAL_CAN_MspInit())
* @retval None
*/
static void can_msp_init (void)
{
    rcu_periph_clock_enable(RCU_GPIOA + GPIO_GET_INDEX(CAN_PORT));
    rcu_periph_clock_enable(RCU_CAN0);

    gpio_mode_set(CAN_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, (1 << CAN_RX_PIN)|(1 << CAN_TX_PIN));
    gpio_output_options_set(CAN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, (1 << CAN_RX_PIN)|(1 << CAN_TX_PIN));
    gpio_af_set(CAN_PORT, GPIO_AF_9, (1 << CAN_RX_PIN)|(1 << CAN_TX_PIN));

    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_CAN,
        .port = (void *)CAN_PORT,
        .pin = CAN_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "CAN"
    };

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_CAN,
        .port = (void *)CAN_PORT,
        .pin = CAN_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "CAN"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);
}

/* Former HAL_CAN_RxFifo0MsgPendingCallback() - the GD32 library has no interrupt
 * dispatcher, so the former HAL_CAN_IRQHandler() + callback pair is merged into
 * the interrupt handler below.
 */
static void can_rxfifo0_msg_pending_callback (void)
{
    canbus_message_t message;
    can_receive_message_struct RxHeader;

    while (can_receive_message_length_get(CAN0, CAN_FIFO0)) {

        can_message_receive(CAN0, CAN_FIFO0, &RxHeader); // releases the FIFO as well

        /* Attempt to add incoming message to the RX queue.
         *
         * Note: not currently checking for success, if there is no space available we
         * would just end up dropping messages somewhere else (i.e. in the CAN RX mailbox)..
         */
        message.id = RxHeader.rx_sfid;
        message.len = RxHeader.rx_dlen;
        memcpy(message.data, RxHeader.rx_data, sizeof(message.data));
        rx_enqueue(message, rx_callbacks[RxHeader.rx_fi]);
    }
}

void CAN0_RX0_IRQHandler(void)
{
    if(SET == can_interrupt_flag_get(CAN0, CAN_INT_FLAG_RFL0)) {
        can_interrupt_flag_clear(CAN0, CAN_INT_FLAG_RFL0);
        can_rxfifo0_msg_pending_callback();
    }
}

#endif // CAN_PORT
