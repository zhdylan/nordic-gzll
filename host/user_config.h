/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
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
/**@cond To Make Doxygen skip documentation generation for this file.
 * @{
 */

#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#include <stdint.h>

/*****************************************************************************/
/** @name Configuration  */
/** @note #defines and typedefs*/
/*****************************************************************************/
/**@brief Pipe 0 is used in this example. */
#define PIPE_NUMBER
/**@brief 32-byte payload length is used when transmitting. */
#define TX_PAYLOAD_LENGTH           32
/**@brief  Timeslot period */
#define TIMESLOT_PERIOD             600
/**@brief   Timeslots per channel. */
#define TIMESLOT_PER_CH             2
/**@brief   Timeslot per channel when out of sync state. */
#define TIMESLOT_PER_CH_OUT_SYNC    12
/**@brief   Sync interval used 1s. */
#define TIME_SYNC_INTERVAL          5000000

/**@brief   Eenable packet loss debug. */
//#define PACKET_LOSS_DEBUG
/**@brief   Enable pairing feature */
//#define DEVICE_PAIRING_ENABLE

#define TIMESTAMP_PRINT_PIN_NUMBER  9

#define MAXiMUM_CHANNELS_COUNT      80
#define MAXIMUM_RSSI_SAMPLES		256

/**@brief   Data structure for time sync */
typedef struct
{
    uint32_t timer_tick_cur0;
    uint32_t timer_tick_pre0;
    uint32_t timer_tick_cur1;
    uint32_t timer_tick_pre1;
    uint64_t timer_tick_ap;
}sSyncTimer_TypeDef;

typedef struct
{
    uint16_t rssi[MAXiMUM_CHANNELS_COUNT];
    uint8_t rssi_index[MAXiMUM_CHANNELS_COUNT];
}sRssiArray_TypDef;

typedef struct
{
    uint8_t chlist[6];
}sRfPowerOnChannelList_TypeDef;

#define byte uint8_t
#define ulong uint32_t

#endif // __USER_CONFIG_H

/** @}
 *  @endcond
 */
