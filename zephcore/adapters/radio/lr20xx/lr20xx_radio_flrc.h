/*!
 * @file      lr20xx_radio_flrc.h
 *
 * @brief     Radio FLRC driver definition for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LR20XX_RADIO_FLRC_H
#define LR20XX_RADIO_FLRC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_status.h"
#include "lr20xx_radio_flrc_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*
 * Set FLRC modulation params. Not available on LR2022.
 * Applies lr20xx_workarounds_dcdc_configure automatically unless LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_CONFIGURE.
 */
lr20xx_status_t lr20xx_radio_flrc_set_modulation_params( const void*                           context,
                                                         const lr20xx_radio_flrc_mod_params_t* params );

/* Set FLRC packet params. Not available on LR2022. */
lr20xx_status_t lr20xx_radio_flrc_set_pkt_params( const void* context, const lr20xx_radio_flrc_pkt_params_t* params );

/*
 * Get FLRC Rx statistics. Not available on LR2022.
 * Stats reset on POR, retention-less sleep, or lr20xx_radio_common_reset_rx_stats.
 */
lr20xx_status_t lr20xx_radio_flrc_get_rx_stats( const void* context, lr20xx_radio_flrc_rx_stats_t* statistics );

/*
 * Get status of last FLRC received packet. Not available on LR2022.
 * rssi_sync/syncword_index available from SYNC_WORD_HEADER_VALID IRQ.
 * rssi_avg available from RX_DONE IRQ.
 */
lr20xx_status_t lr20xx_radio_flrc_get_pkt_status( const void* context, lr20xx_radio_flrc_pkt_status_t* pkt_status );

/* Set 2-byte short syncword at syncword_index. Not available on LR2022. */
lr20xx_status_t lr20xx_radio_flrc_set_short_syncword(
    const void* context, uint8_t syncword_index,
    const uint8_t short_syncword[LR20XX_RADIO_FLRC_SHORT_SYNCWORD_LENGTH] );

/* Set 4-byte syncword at syncword_index. Not available on LR2022. */
lr20xx_status_t lr20xx_radio_flrc_set_syncword( const void* context, uint8_t syncword_index,
                                                const uint8_t syncword[LR20XX_RADIO_FLRC_SYNCWORD_LENGTH] );

/* Compute FLRC time-on-air in microseconds. Not available on LR2022. */
uint32_t lr20xx_get_flrc_time_on_air_in_us( const lr20xx_radio_flrc_pkt_params_t* pkt_params,
                                            const lr20xx_radio_flrc_mod_params_t* mod_params );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_FLRC_H

/* --- EOF ------------------------------------------------------------------ */
