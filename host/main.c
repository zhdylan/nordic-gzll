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
 * This project requires that a device that runs the
 * @ref gzll_device_m_ack_payload_example is used as a counterpart for
 * receiving the data. This can be on either an nRF5x device or an nRF24Lxx device
 * running the \b gzll_device_m_ack_payload example in the nRFgo SDK.
 *
 * This example listens for a packet and sends an ACK
 * when a packet is received. The contents of the first payload byte of
 * the received packet is output on the GPIO Port BUTTONS.
 * The contents of GPIO Port LEDS are sent in the first payload byte (byte 0)
 * of the ACK packet.
 */
#include "nrf_gzll.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_error.h"
#include "nrf_gzll_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "counter.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_spis.h"
#include "nrf_drv_gpiote.h"

#include "user_config.h"

static uint8_t                  m_data_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; /**< Placeholder for data payload received from host. */
static uint8_t                  m_ack_payload[TX_PAYLOAD_LENGTH];                  /**< Payload to attach to ACK sent to device. */
const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(1);

sSyncTimer_TypeDef sSyncTimer = {0x00,0x00};
sRfPowerOnChannelList_TypeDef sRfPowerOnChannelList;

#ifdef PACKET_LOSS_DEBUG
bool msgReady[2];
uint8_t txId[2];
uint32_t msgIdx[2];
uint32_t curTmCnt[2];
#endif

#if GZLL_TX_STATISTICS
static nrf_gzll_tx_statistics_t m_statistics; /**< Struct containing transmission statistics. */
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
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);

    // Set up logger.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

//    NRF_LOG_INFO("Gazell ACK payload example. Host mode.");
//    NRF_LOG_FLUSH();

    bsp_board_init(BSP_INIT_LEDS);
}


/*****************************************************************************/
/** @name Gazell callback function definitions.  */
/*****************************************************************************/
/**
 * @brief RX data ready callback.
 *
 * @details If a data packet was received, the first byte is written to LEDS.
 */
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
    uint32_t data_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    // Pop packet and write first byte of the payload to the GPIO port.
    bool result_value = nrf_gzll_fetch_packet_from_rx_fifo(pipe,
                                                           m_data_payload,
                                                           &data_payload_length);
#ifdef PACKET_LOSS_DEBUG
    if((msgReady[0] == false) && (pipe==1) && (m_data_payload[0]==1))
#else
    if(/*(msgReady[0] == false) &&*/ (pipe==1) && (m_data_payload[0]==1))
#endif
    {
        nrf_gpio_pin_write(30,1);
        nrf_gpio_pin_write(30,0);
        
#ifdef PACKET_LOSS_DEBUG        
        curTmCnt[0] = counter_get();
        txId[0] = m_data_payload[0];
        msgIdx[0] = uint32_decode(&m_data_payload[1]);
        
        if(msgIdx[0] != 0x00)
        {
            msgReady[0] = true;
        }
#endif        
        sSyncTimer.timer_tick_cur0 = nrfx_timer_capture(&TIMER_LED, NRF_TIMER_CC_CHANNEL1);
        if((sSyncTimer.timer_tick_cur0 - sSyncTimer.timer_tick_pre0) >= TIME_SYNC_INTERVAL)
        {
            m_ack_payload[0]= 0xAA;
            uint32_encode(sSyncTimer.timer_tick_cur0, &m_ack_payload[1]);
            sSyncTimer.timer_tick_pre0 = sSyncTimer.timer_tick_cur0;
        }
    }
#ifdef PACKET_LOSS_DEBUG
    else if((msgReady[1] == false)&& (pipe==2) && (m_data_payload[0]==2))
#else
    else if(/*(msgReady[1] == false)&&*/ (pipe==2) && (m_data_payload[0]==2))
#endif    
    {
        nrf_gpio_pin_write(31,1);
        nrf_gpio_pin_write(31,0);
        
#ifdef PACKET_LOSS_DEBUG
        curTmCnt[1] = counter_get();
        txId[1] = m_data_payload[0];
        msgIdx[1] = uint32_decode(&m_data_payload[1]);
        if(msgIdx[1] != 0x00)
        {
            msgReady[1] = true;
        }
#endif       
        sSyncTimer.timer_tick_cur1 = nrfx_timer_capture(&TIMER_LED, NRF_TIMER_CC_CHANNEL1);
        if((sSyncTimer.timer_tick_cur1 - sSyncTimer.timer_tick_pre1) >= 10000000/2)
        {
            m_ack_payload[0]= 0xAA;
            uint32_encode(sSyncTimer.timer_tick_cur1, &m_ack_payload[1]);
            sSyncTimer.timer_tick_pre1 = sSyncTimer.timer_tick_cur1;
        }
    }
    
    result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_ack_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
    }
    m_ack_payload[0] = 0x00;

#if GZLL_PA_LNA_CONTROL
    m_rssi_sum += rx_info.rssi;
    m_packets_cnt++;
#endif
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    //TODO
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    //NRF_LOG_INFO("gzell tx failed");
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_disabled()
{
    NRF_LOG_INFO("gzell disabled");
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
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE1:
            //bsp_board_led_invert(led_to_invert);
            nrf_gpio_pin_write(29,1);
            nrf_gpio_pin_write(29,0);
            break;

        default:
            //Do nothing.
            break;
    }
}

void timestamp_print_int_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t systick = nrfx_timer_capture(&TIMER_LED, NRF_TIMER_CC_CHANNEL0);

    NRF_LOG_INFO("Host:%x",systick);  
}

#define MAXIMUM_CHANNELS_PER_REGION 20
#define MAXIMUM_CHANNEL_LIST_SIZE 3
#define REGION1_CHANNEL_LIST					{1, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 17, 18, 20, 23, 24, 25, 27, 29}
#define REGION2_CHANNEL_LIST					{35, 32, 36, 39, 41, 42, 44, 46,49, 50, 52, 53, 54, 55, 57, 59, 61, 62, 63, 65}
#define REGION3_CHANNEL_LIST					{67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86}


const uint8_t gca_available_chlist[MAXIMUM_CHANNEL_LIST_SIZE][MAXIMUM_CHANNELS_PER_REGION] = {
									REGION1_CHANNEL_LIST,
									REGION2_CHANNEL_LIST,				
									REGION3_CHANNEL_LIST};

void do_channel_list_generation_all(void)
{
    uint32_t packet;
	uint16_t samples, highest_rssi, rssi[MAXIMUM_CHANNELS_PER_REGION];
	uint8_t i, k, selected_ch;
	
	NRF_RADIO->PACKETPTR = (uint32_t)&packet;
	
	for(i = 0; i < MAXIMUM_CHANNEL_LIST_SIZE; i++){
		
		for(k = 0; k < MAXIMUM_CHANNELS_PER_REGION; k++){
			rssi[k] = 0;
		}
		
		highest_rssi = 0;
		
        for(k = 0; k < MAXIMUM_CHANNELS_PER_REGION; k++){
            for(samples = 0; samples < MAXIMUM_RSSI_SAMPLES; samples++){
				NRF_RADIO->FREQUENCY = gca_available_chlist[i][k];
				NRF_RADIO->EVENTS_READY = 0U;
				NRF_RADIO->TASKS_RXEN = 1U;
				while(NRF_RADIO->EVENTS_READY == 0U);
				NRF_RADIO->EVENTS_END = 0U;

				NRF_RADIO->TASKS_START = 1U;

				NRF_RADIO->EVENTS_RSSIEND = 0U;
				NRF_RADIO->TASKS_RSSISTART = 1U;
				while(NRF_RADIO->EVENTS_RSSIEND == 0U);

				rssi[k] += NRF_RADIO->RSSISAMPLE;

				NRF_RADIO->EVENTS_DISABLED = 0U;    
				NRF_RADIO->TASKS_DISABLE   = 1U;  // Disable the radio.

				while(NRF_RADIO->EVENTS_DISABLED == 0U)
				{
						// Do nothing.
				}
			}
            
            if(highest_rssi < rssi[k]){
                highest_rssi = rssi[k];
                selected_ch = gca_available_chlist[i][k];
            }
		}

        sRfPowerOnChannelList.chlist[i] = selected_ch;
	}
    
    NRF_LOG_INFO("%d,%d,%d",sRfPowerOnChannelList.chlist[0],sRfPowerOnChannelList.chlist[1],
    sRfPowerOnChannelList.chlist[2]);
    NRF_LOG_FLUSH();
}
uint32_t base_local_address = 0x00;
uint32_t base_address = 0x1234abcd;
uint8_t host_id[4]= {0x00};


void read_host_chip_id(byte *dst)
{
    byte loop;
    ulong dev_id = NRF_FICR->DEVICEID[1];
    
    for(loop = 0x00; loop < 4; loop++)
    {
        dst[loop] = (dev_id >> (loop * 8)) & 0xff;
    }
}

#define SPIS_INSTANCE 1 /**< SPIS instance index. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
static uint8_t       m_tx_buf[66] = {0x00};           /**< TX buffer. */
static uint8_t       m_rx_buf[66];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */
#define GPIO_SPIS_INT_PIN_NUMBER    41
static volatile bool spis_xfer_done; /**< Flag used to indicate that SPIS instance completed the transfer. */

/**
 * @brief SPIS user event handler.
 *
 * @param event
 */
void spis_event_handler(nrf_drv_spis_event_t event)
{

    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {
#if 0
        switch(SPI_FRAME_START)
        {
            case SPI_CONTROLLER_DATA:
                break;
            
            case SPI_CONTROL_DATA:
                if(SPI_FRAME_1ST_CMD == SPI_1ST_CMD_CTRL)
                {
                    process_control_packet(m_rx_buf, event.rx_amount);
                }
                else if(SPI_FRAME_1ST_CMD == SPI_1ST_CMD_DFU)
                {
                    process_dfu_packet(m_rx_buf, event.rx_amount);
                }
                break;
            default:
                break;
        }
#endif            
        //spis_xfer_done = true;
        //NRF_LOG_INFO(" Transfer completed. Received: %s",(uint32_t)m_rx_buf);
        
        APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));
    }
}

static void spis_init(void)
{
    NRF_LOG_INFO("SPIS example");

    nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spis_config.csn_pin               = APP_SPIS_CS_PIN;
    spis_config.miso_pin              = APP_SPIS_MISO_PIN;
    spis_config.mosi_pin              = APP_SPIS_MOSI_PIN;
    spis_config.sck_pin               = APP_SPIS_SCK_PIN;

    APP_ERROR_CHECK(nrf_drv_spis_init(&spis, &spis_config, spis_event_handler));    
    
    APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));

    nrf_gpio_cfg_output(GPIO_SPIS_INT_PIN_NUMBER);
    nrf_gpio_pin_write(GPIO_SPIS_INT_PIN_NUMBER,0);
}

/*****************************************************************************/
/**
 * @brief Main function.
 * @return ANSI required int return type.
 */
/*****************************************************************************/
int main()
{
    float tsMs; // timestamp in ms
 
    // Set up the user interface.
    //static uint8_t Test_chanel_table[6]={ 4, 25, 42, 63, 77,33};
    //static uint8_t Test_chanel_table[6] = {10,12,29,53,54,61};
    static uint8_t Test_chanel_table[6] = {10,34,38,47,49,65};
    
    uint32_t time_ms = 4; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
    
    ui_init();
    nrf_gpio_cfg_output(31);
    nrf_gpio_pin_write(31,0);
    nrf_gpio_cfg_output(30);
    nrf_gpio_pin_write(30,0);
//    nrf_gpio_cfg_output(29);
//    nrf_gpio_pin_write(29,0);
    
    do_channel_list_generation_all();
    
    // Initialize Gazell.
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_HOST);
    GAZELLE_ERROR_CODE_CHECK(result_value);
    
    nrf_gzll_set_datarate(NRF_GZLL_DATARATE_2MBIT);
    nrf_gzll_set_timeslot_period(TIMESLOT_PERIOD);    
    result_value = nrf_gzll_set_channel_table((void *)&Test_chanel_table[0],6);
    GAZELLE_ERROR_CODE_CHECK(result_value);
    nrf_gzll_set_timeslots_per_channel(TIMESLOT_PER_CH);
    nrf_gzll_set_device_channel_selection_policy(NRF_GZLL_DEVICE_CHANNEL_SELECTION_POLICY_USE_CURRENT);
    nrf_gzll_set_timeslots_per_channel_when_device_out_of_sync(TIMESLOT_PER_CH_OUT_SYNC);
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
    
    // pin0.29 configured for timestamp print
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	err_code = nrf_drv_gpiote_in_init(TIMESTAMP_PRINT_PIN_NUMBER, &config, timestamp_print_int_pin_event_handler);
	nrf_drv_gpiote_in_event_enable(TIMESTAMP_PRINT_PIN_NUMBER, true);
    
    //NRF_LOG_INFO("%x\n",err_code);  
	(void)(err_code);
    
    counter_init();
    counter_start();
    
#ifdef DEVICE_PAIRING_ENABLE    
    read_host_chip_id(host_id);
    base_address = uint32_decode(host_id);
    nrf_gzll_set_base_address_1(base_address);
#endif

    // Enable Gazell to start sending over the air.
    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);

    //NRF_LOG_INFO("Gzll ack payload host example started.");

    while (true)
    {
#ifndef PACKET_LOSS_DEBUG
        NRF_LOG_FLUSH();
#endif
        __WFE();
        
#ifdef PACKET_LOSS_DEBUG       
        if(msgReady[0])
        {
            tsMs = ( curTmCnt[0] * 0.12207);
            NRF_LOG_INFO("0, %06d, " NRF_LOG_FLOAT_MARKER, msgIdx[0],NRF_LOG_FLOAT(tsMs));
            msgReady[0] = false;
            NRF_LOG_FLUSH();
        }
        
        if(msgReady[1])
        {
            tsMs = ( curTmCnt[1] * 0.12207);
            NRF_LOG_INFO("1, %06d, " NRF_LOG_FLOAT_MARKER, msgIdx[1],NRF_LOG_FLOAT(tsMs));
            msgReady[1] = false;
            NRF_LOG_FLUSH();
        }
#endif        
#if GZLL_PA_LNA_CONTROL
        if (m_packets_cnt >= 1000)
        {
            CRITICAL_REGION_ENTER();

            // Print info about average RSSI.
            NRF_LOG_RAW_INFO("\r\n");
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
            NRF_LOG_INFO("Total received packets:   %4u", m_statistics.packets_num);
            NRF_LOG_INFO("Total CRC failures:       %03u\r\n", m_statistics.timeouts_num);

            for (uint8_t i = 0; i < nrf_gzll_get_channel_table_size(); i++)
            {
                NRF_LOG_INFO("Channel %u: %03u packets received, %03u CRC failures.",
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
