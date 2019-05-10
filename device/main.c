/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * This project requires that a host that runs the
 * @ref gzll_host_m_ack_payload_example example is used as a counterpart for
 * receiving the data. This can be on either an nRF5x device or an nRF24Lxx device
 * running the \b gzll_host_m_ack_payload example in the nRFgo SDK.
 *
 * This example sends a packet and adds a new packet to the TX queue every time
 * it receives an ACK. Before adding a packet to the TX queue, the contents of
 * the GPIO Port BUTTONS is copied to the first payload byte (byte 0).
 * When an ACK is received, the contents of the first payload byte of
 * the ACK are output on GPIO Port LEDS.
 */

#include "nrf_gzll.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_error.h"
#include "nrf_gzll_error.h"

#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "user_define.h"
#include "nrf_drv_gpiote.h"
#include "counter.h"
#include "nrf_delay.h"

#include "bsp_uart.h"
#include "uart_protocol.h"

//#define DEVICE1 
#define ALEX_DEBUG
/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/
#ifdef DEVICE1

#define PIPE_NUMBER             2   /**< Pipe 0 is used in this example. */
#else
#define PIPE_NUMBER             1
#endif

#define TX_PAYLOAD_LENGTH       32//1   /**< 1-byte payload length is used when transmitting. */
#define MAX_TX_ATTEMPTS         5 /**< Maximum number of transmission attempts */
#define TIMESLOT_PERIOD         580
#define TIMESLOT_PER_CH         4
#define TIMESLOT_PER_CH_OUT_SYNC 12

static uint8_t                  m_data_payload[TX_PAYLOAD_LENGTH];                /**< Payload to send to Host. */
static uint8_t                  m_ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; /**< Placeholder for received ACK payloads from Host. */

#ifdef ALEX_DEBUG
T_sUartImuFrameFifo_TypeDef T_sUartImuFrameFifo;
uint32_t msgIdx = 0x00;
typedef struct
{
    uint32_t rtc_tick_cur;
    uint32_t rtc_tick_pre;
    uint32_t rtc_interval;
}sSyncRtc_TypeDef;
sSyncRtc_TypeDef sSyncRtc;

typedef struct
{
    uint32_t timer_tick_cur;
    uint32_t timer_tick_master;
    uint64_t timer_tick_ap;
}sSyncTimer_TypeDef;

sSyncTimer_TypeDef sSyncTime;

uint32_t base_address = 0xA94FDDA8;
uint32_t base_address0 = 0xaabb;

#endif

#if GZLL_TX_STATISTICS
static nrf_gzll_tx_statistics_t m_statistics;   /**< Struct containing transmission statistics. */
#endif

#if GZLL_PA_LNA_CONTROL

#define GZLL_PA_LNA_TIMER       CONCAT_2(NRF_TIMER, GZLL_PA_LNA_TIMER_NUM) /**< Convert timer number into timer struct. */

/**< PA/LNA structure configuration. */
static nrf_gzll_pa_lna_cfg_t m_pa_lna_cfg = {
    .lna_enabled        = GZLL_LNA_ENABLED,
    .pa_enabled         = GZLL_PA_ENABLED,
    .lna_active_high    = GZLL_LNA_ACTIVE_HIGH,
    .pa_active_high     = GZLL_PA_ACTIVE_HIGH,
    .lna_gpio_pin       = GZLL_PA_LNA_CRX_PIN,
    .pa_gpio_pin        = GZLL_PA_LNA_CTX_PIN,
    .pa_gpiote_channel  = GZLL_PA_LNA_TX_GPIOTE_CHAN,
    .lna_gpiote_channel = GZLL_PA_LNA_RX_GPIOTE_CHAN,
    .timer              = GZLL_PA_LNA_TIMER,
    .ppi_channels[0]    = GZLL_PA_LNA_PPI_CHAN_1,
    .ppi_channels[1]    = GZLL_PA_LNA_PPI_CHAN_2,
    .ppi_channels[2]    = GZLL_PA_LNA_PPI_CHAN_3,
    .ppi_channels[3]    = GZLL_PA_LNA_PPI_CHAN_4,
    .ramp_up_time       = GZLL_PA_LNA_RAMP_UP_TIME
};

static int32_t m_rssi_sum    = 0; /**< Variable used to calculate average RSSI. */
static int32_t m_packets_cnt = 0; /**< Transmitted packets counter. */
#endif

const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(1);


#ifdef ALEX_DEBUG
void imu_data_fifo_init(void)
{
    memset(T_sUartImuFrameFifo.imu_frame_bufer, 0x00, sizeof(T_sUartImuFrameFifo.imu_frame_bufer));
    T_sUartImuFrameFifo.imu_fifo_front = 0;
    T_sUartImuFrameFifo.imu_fifo_rear = 0;
    T_sUartImuFrameFifo.imu_fifo_size = 5;
}


int put_imu_data_fifo(byte *p_uart_imu_frame, byte imu_frame_len)
{
    int ret = -1;
    if (NULL == p_uart_imu_frame) {
#ifdef DEBUG_LOG
        NRF_LOG_INFO("the p_uart_imu_frame is null! \n");
#endif
        goto error;
    }

    if ((T_sUartImuFrameFifo.imu_fifo_rear + 1) % T_sUartImuFrameFifo.imu_fifo_size == T_sUartImuFrameFifo.imu_fifo_front) {  
#ifdef DEBUG_LOG
        NRF_LOG_INFO("%s\n","The T_sUartImuFrameFifo is full");  
#endif
        //fifo_discard_cnt++;
        goto error;  
    }  
    memcpy(T_sUartImuFrameFifo.imu_frame_bufer[T_sUartImuFrameFifo.imu_fifo_rear], p_uart_imu_frame, imu_frame_len);
    T_sUartImuFrameFifo.imu_fifo_rear = (T_sUartImuFrameFifo.imu_fifo_rear + 1) % T_sUartImuFrameFifo.imu_fifo_size;
    ret = 0;

error:  
    return ret;

}

int get_imu_data_fifo(byte *p_uart_imu_frame, byte imu_frame_len)
{
    int ret = -1;
    if ( NULL == p_uart_imu_frame ) {
#ifdef DEBUG_LOG
        NRF_LOG_INFO("the p_uart_imu_frame is null! \n");
#endif
        goto error;
    }
    if(T_sUartImuFrameFifo.imu_fifo_front == T_sUartImuFrameFifo.imu_fifo_rear) 
    {  
#ifdef DEBUG_LOG
        NRF_LOG_INFO("%s\n","The T_sUartImuFrameFifo is empty");  
#endif
        goto error;  
    }  

    memcpy(p_uart_imu_frame, T_sUartImuFrameFifo.imu_frame_bufer[T_sUartImuFrameFifo.imu_fifo_front], imu_frame_len);
    T_sUartImuFrameFifo.imu_fifo_front = (T_sUartImuFrameFifo.imu_fifo_front + 1) % T_sUartImuFrameFifo.imu_fifo_size;
    ret = 0;

error:
    return ret;
}

/**
 * @brief Function to put imu data into fifo.
 *
 * @return None.
 */
void uart_imu_int_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint64_t master_tick_val_tmp = 0x00;
    
    uint8_t tmp_payload[TX_PAYLOAD_LENGTH] = {0x00};
    
    msgIdx++;
    tmp_payload[0] = (byte)PIPE_NUMBER;
    uint32_encode(msgIdx, &tmp_payload[1]);
    
    if(0x00 != put_imu_data_fifo(tmp_payload, TX_PAYLOAD_LENGTH))
    {
//        NRF_LOG_INFO("fifo is full");                                                                        tImuFrameFifo is full");  
    }
}

void timestamp_print_int_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint64_t master_tick_val_tmp = 0x00;
    
    if(pin == 28)
    {
        nrf_gpio_pin_write(29,1);
        nrf_gpio_pin_write(29,0);
        sSyncTime.timer_tick_cur = nrfx_timer_capture(&TIMER_LED, NRF_TIMER_CC_CHANNEL1);
        master_tick_val_tmp = sSyncTime.timer_tick_cur+sSyncTime.timer_tick_master;
        
        if(master_tick_val_tmp > 0xffffffff)
        {
            master_tick_val_tmp = (0xffffffff-sSyncTime.timer_tick_master) + sSyncTime.timer_tick_cur;
        }

        NRF_LOG_INFO("P:%d,t:%x",PIPE_NUMBER,master_tick_val_tmp); 
    }
}
/**
 * @brief Function to init a input GPIO.
 *
 * @return None.
 */
void uart_imu_int_pin_init(void)
{
    ret_code_t err_code=NRF_SUCCESS;
	nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	config.pull=NRF_GPIO_PIN_NOPULL;
	err_code = nrf_drv_gpiote_in_init(UART_IMU_GPIO_PIN_NUMBER, &config, uart_imu_int_pin_event_handler);
	nrf_drv_gpiote_in_event_enable(UART_IMU_GPIO_PIN_NUMBER, true);
	

    nrf_drv_gpiote_in_config_t config1 = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	config1.pull=NRF_GPIO_PIN_NOPULL;
	err_code = nrf_drv_gpiote_in_init(TIMESTAMP_PRINT_PIN_NUMBER, &config1, timestamp_print_int_pin_event_handler);
	nrf_drv_gpiote_in_event_enable(TIMESTAMP_PRINT_PIN_NUMBER, true);
    
    //NRF_LOG_INFO("%x\n",err_code);  
	(void)(err_code);
}
#endif


/**
 * @brief Initialize the BSP modules.
 */
static void ui_init(void)
{
    uint32_t err_code;

    // Initialize application timer.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // BSP initialization.
//    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
//    APP_ERROR_CHECK(err_code);

    // Set up logger
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Gazell ACK payload example. Device mode.");
    NRF_LOG_FLUSH();

    //bsp_board_init(BSP_INIT_LEDS);
}


/*****************************************************************************/
/** @name Gazell callback function definitions  */
/*****************************************************************************/
/**
 * @brief TX success callback.
 *
 * @details If an ACK was received, another packet is sent.
 */
void  nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    bool     result_value         = false;
    uint32_t m_ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    if (tx_info.payload_received_in_ack)
    {
        sSyncRtc.rtc_tick_cur = counter_get();
        // Pop packet and write first byte of the payload to the GPIO port.
        result_value =
            nrf_gzll_fetch_packet_from_rx_fifo(pipe, m_ack_payload, &m_ack_payload_length);
        if (!result_value)
        {
            NRF_LOG_ERROR("RX fifo error ");
        }
        
        nrf_gpio_pin_write(31,1);
        nrf_gpio_pin_write(31,0);
        
        if((sSyncRtc.rtc_tick_cur - sSyncRtc.rtc_tick_pre) <= 22)
        {
            if(m_ack_payload[0] == 0xAA)
            {
                nrfx_timer_clear(&TIMER_LED);
                sSyncTime.timer_tick_master = uint32_decode(&m_ack_payload[1]);
                //NRF_LOG_INFO("m:%x",sSyncTime.timer_tick_master);
                sSyncTime.timer_tick_master += (2710);
            }
        }
    }

    // Load data payload into the TX queue.
    //    m_data_payload[0] = input_get();
    //  byte-0   byte1-byte4
    //  pipe      msgIdx
    if(get_imu_data_fifo(m_data_payload,32) != 0x00)
    {
        // Fifo empty, send emtpy packet
        memset(&m_data_payload[1],0x00,4);
    }
    // record rtc val of send time point
    sSyncRtc.rtc_tick_pre = counter_get();
    result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_data_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
    }

#if GZLL_PA_LNA_CONTROL
    m_rssi_sum += tx_info.rssi;
    m_packets_cnt++;
#endif
}


/**
 * @brief TX failed callback.
 *
 * @details If the transmission failed, send a new packet.
 *
 * @warning This callback does not occur by default since NRF_GZLL_DEFAULT_MAX_TX_ATTEMPTS
 * is 0 (inifinite retransmits).
 */
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    //NRF_LOG_ERROR("Gazell transmission failed");

    // Load data into TX queue.
    //m_data_payload[0] = input_get();
    if(get_imu_data_fifo(m_data_payload,32) != 0x00)
    {
        // Fifo empty, send emtpy packet
        memset(&m_data_payload[1],0x00,4);
    }
//    nrf_gpio_pin_write(31,1);
//    nrf_gpio_pin_write(31,0);
    bool result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_data_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
    }
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_disabled()
{
}

#if GZLL_PA_LNA_CONTROL
/**
 * @brief Function for configuring front end control in Gazell.
 */
static bool front_end_control_setup(void)
{
    bool result_value = true;

    // Configure pins controlling SKY66112 module.
    nrf_gpio_cfg_output(GZLL_PA_LNA_CHL_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_CPS_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_ANT_SEL_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_CSD_PIN);

    // Turn on front end module.
    nrf_gpio_pin_clear(GZLL_PA_LNA_CHL_PIN);
    nrf_gpio_pin_clear(GZLL_PA_LNA_CPS_PIN);
    nrf_gpio_pin_clear(GZLL_PA_LNA_ANT_SEL_PIN);
    nrf_gpio_pin_set(GZLL_PA_LNA_CSD_PIN);

    // PA/LNA configuration must be called after @ref nrf_gzll_init() and before @ref nrf_gzll_enable()
    result_value = nrf_gzll_set_pa_lna_cfg(&m_pa_lna_cfg);

    return result_value;
}
#endif


/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    static uint32_t i;
    bool result_value;
    uint32_t led_to_invert = ((i++) % LEDS_NUMBER);

    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            nrf_gpio_pin_write(31,1);
            nrf_gpio_pin_write(31,0);
            break;

        default:
            //Do nothing.
            break;
    }
}

uint32_t systick_cnt = 0x00;
static uint16_t uart_send_msg_cnt = 0;
/*****************************************************************************/
/**
 * @brief Main function.
 *
 * @return ANSI required int return type.
 */
/*****************************************************************************/
int main()
{
    
//#ifdef DEVICE1
//   static uint8_t channel_table[6]={ 25,63,33};
//#else
//   static uint8_t channel_table[6]={ 4, 42, 77};
//#endif
//    #ifdef DEVICE1
//   static uint8_t channel_table[6]={ 12,53,61};
//#else
//   static uint8_t channel_table[6]={ 10, 29, 54};
//#endif
  
#ifdef DEVICE1
   static uint8_t channel_table[6]={ 10,38,49};
#else
   static uint8_t channel_table[6]={ 34,47,65};
#endif
   
    uint32_t time_ms = 4; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
   
    // Set up the user interface (buttons and LEDs).
    ui_init();
    nrf_gpio_cfg_output(31);
    nrf_gpio_cfg_output(29);
    nrf_gpio_pin_write(31,0);
    nrf_gpio_pin_write(29,0);
   
    // Initialize Gazell.
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    nrf_gzll_set_datarate(NRF_GZLL_DATARATE_2MBIT);
    nrf_gzll_set_timeslot_period(TIMESLOT_PERIOD);
    nrf_gzll_set_channel_table((void *)&channel_table[0],3);
    nrf_gzll_set_timeslots_per_channel(TIMESLOT_PER_CH);
    nrf_gzll_set_device_channel_selection_policy(NRF_GZLL_DEVICE_CHANNEL_SELECTION_POLICY_USE_CURRENT);
    nrf_gzll_set_timeslots_per_channel_when_device_out_of_sync(TIMESLOT_PER_CH_OUT_SYNC);
    // Attempt sending every packet up to MAX_TX_ATTEMPTS times.
    nrf_gzll_set_max_tx_attempts(MAX_TX_ATTEMPTS);
    nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_4_DBM);

#if GZLL_PA_LNA_CONTROL
    // Initialize external PA/LNA control.
    result_value = front_end_control_setup();
    GAZELLE_ERROR_CODE_CHECK(result_value);
#endif

#if GZLL_TX_STATISTICS
    // Turn on transmission statistics gathering.
    result_value = nrf_gzll_tx_statistics_enable(&m_statistics);
    GAZELLE_ERROR_CODE_CHECK(result_value);
#endif

    // init a timer for time synchronization
    nrf_drv_timer_config_t timer_cfg = {
            .frequency          = (nrf_timer_frequency_t)4,\
            .mode               = (nrf_timer_mode_t)NRFX_TIMER_DEFAULT_CONFIG_MODE,          \
            .bit_width          = (nrf_timer_bit_width_t)3,\
            .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,                    \
            .p_context          = NULL 
    };
    err_code = nrf_drv_timer_init(&TIMER_LED, &timer_cfg, timer_led_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = 0xffffffff;//nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);
    nrf_drv_timer_extended_compare(
         &TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    
    nrf_drv_timer_enable(&TIMER_LED);
    
#ifdef ALEX_DEBUG
    imu_data_fifo_init();
    uart_imu_int_pin_init();
    counter_init();
    counter_start();
    
    // add uart interface
    bsp_uart0_init();
#endif
    
#ifdef NRG_GZLL_PAIRING_EN  
    if(!nrf_gzll_set_base_address_1(base_address))
    {
        NRF_LOG_INFO("Gzll base address 0 set failed");
    }
#endif    

    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);

    msgIdx++;
    m_data_payload[0] = (byte)PIPE_NUMBER;
    uint32_encode(msgIdx, &m_data_payload[1]);
    result_value = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, m_data_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
        NRF_LOG_FLUSH();
    }
    
    NRF_LOG_INFO("Gzll ack payload device example started.");
    
    while (true)
    {
        NRF_LOG_FLUSH();
        //__WFE();
        nrf_delay_ms(1);
        if(++uart_send_msg_cnt >= 50)
        {
            uart_send_msg_cnt = 0;
            vBEH_Uart_NewFrame_(UART_SEND_RAW_DATA);
        }

#if GZLL_PA_LNA_CONTROL
        if (m_packets_cnt >= 1000)
        {
            CRITICAL_REGION_ENTER();

            // Print info about average RSSI.
            NRF_LOG_INFO("Average RSSI: %d", (m_rssi_sum / m_packets_cnt));
            m_packets_cnt = 0;
            m_rssi_sum    = 0;

            CRITICAL_REGION_EXIT();
        }
#endif

#if GZLL_TX_STATISTICS
        if (m_statistics.packets_num >= 1000)
        {
            CRITICAL_REGION_ENTER();

            // Print all transmission statistics.
            NRF_LOG_RAW_INFO("\r\n");
            NRF_LOG_INFO("Total transmitted packets:   %4u",  m_statistics.packets_num);
            NRF_LOG_INFO("Total transmission time-outs: %03u\r\n", m_statistics.timeouts_num);

            for (uint8_t i = 0; i < nrf_gzll_get_channel_table_size(); i++)
            {
                NRF_LOG_INFO("Channel %u: %03u packets transmitted, %03u transmissions failed.",
                             i,
                             m_statistics.channel_packets[i],
                             m_statistics.channel_timeouts[i]);
            }

            CRITICAL_REGION_EXIT();

            // Reset statistics buffers.
            nrf_gzll_reset_tx_statistics();
        }
#endif
    }
}
