/*!
 * @file      lr20xx_radio_ook.h
 *
 * @brief     On-Off Keying radio driver definition for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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

#ifndef LR20XX_RADIO_OOK_H
#define LR20XX_RADIO_OOK_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_status.h"
#include "lr20xx_radio_ook_types.h"

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
 * Set OOK modulation params.
 * Applies lr20xx_workarounds_dcdc_configure automatically unless LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_CONFIGURE.
 */
lr20xx_status_t lr20xx_radio_ook_set_modulation_params( const void*                          context,
                                                        const lr20xx_radio_ook_mod_params_t* params );

/* Set OOK packet params. Note: explicit header + CRC disabled causes incorrect reception. */
lr20xx_status_t lr20xx_radio_ook_set_packet_params( const void* context, const lr20xx_radio_ook_pkt_params_t* params );

lr20xx_status_t lr20xx_radio_ook_set_crc_params( const void* context, uint32_t crc_polynomial, uint32_t crc_seed );

/*
 * Set OOK syncword. syncword is a 32-bit value stored MSB-first in 4 bytes; nb_bits LSBs are used.
 * bit_order controls OTA transmission order (LSBF or MSBF).
 * Example: syncword=0x0000AA67, nb_bits=12 → bits 0xA67 (12 LSBs).
 */
lr20xx_status_t lr20xx_radio_ook_set_syncword( const void*   context,
                                               const uint8_t syncword[LR20XX_RADIO_OOK_SYNCWORD_LENGTH],
                                               uint8_t nb_bits, lr20xx_radio_ook_syncword_bit_order_t bit_order );

lr20xx_status_t lr20xx_radio_ook_set_addresses( const void* context, uint8_t node_address, uint8_t broadcast_address );

/* Get OOK Rx statistics; reset on POR, retention-less sleep, or lr20xx_radio_common_reset_rx_stats */
lr20xx_status_t lr20xx_radio_ook_get_rx_statistics( const void* context, lr20xx_radio_ook_rx_statistics_t* statistics );

/*
 * Get status of last received OOK packet.
 * rssi_on/syncword available from SYNC_WORD_HEADER_VALID IRQ.
 * rssi_avg/addr_match available from RX_DONE IRQ.
 */
lr20xx_status_t lr20xx_radio_ook_get_packet_status( const void* context, lr20xx_radio_ook_packet_status_t* pkt_status );

lr20xx_status_t lr20xx_radio_ook_set_rx_detector( const void*                           context,
                                                  const lr20xx_radio_ook_rx_detector_t* rx_detector );
lr20xx_status_t lr20xx_radio_ook_set_whitening_params( const void*                                context,
                                                       const lr20xx_radio_ook_whitening_params_t* params );

uint32_t lr20xx_radio_ook_get_time_on_air_in_ms( const lr20xx_radio_ook_pkt_params_t* pkt_p,
                                                 const lr20xx_radio_ook_mod_params_t* mod_p,
                                                 uint8_t                              syncword_len_in_bit );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_OOK_H

/* --- EOF ------------------------------------------------------------------ */
