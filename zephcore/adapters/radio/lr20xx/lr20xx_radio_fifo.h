/*!
 * @file      lr20xx_radio_fifo.h
 *
 * @brief     Radio FiFo driver definition for LR20XX
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

#ifndef LR20XX_RADIO_FIFO_H
#define LR20XX_RADIO_FIFO_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_status.h"
#include "lr20xx_radio_fifo_types.h"

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

lr20xx_status_t lr20xx_radio_fifo_read_rx( const void* context, uint8_t* buffer, const uint16_t length );
lr20xx_status_t lr20xx_radio_fifo_write_tx( const void* context, const uint8_t* buffer, const uint16_t length );
lr20xx_status_t lr20xx_radio_fifo_clear_rx( const void* context );
lr20xx_status_t lr20xx_radio_fifo_clear_tx( const void* context );
lr20xx_status_t lr20xx_radio_fifo_get_rx_level( const void* context, uint16_t* fifo_level );
lr20xx_status_t lr20xx_radio_fifo_get_tx_level( const void* context, uint16_t* fifo_level );

/*
 * Configure FIFO threshold IRQs. Threshold IRQs fire on level crossing in the triggering direction;
 * clearing the IRQ flag does not re-fire until the threshold is crossed again.
 * rx_fifo_high_threshold: triggers THRESHOLD_HIGH when Rx level rises above this
 * rx_fifo_low_threshold:  triggers THRESHOLD_LOW  when Rx level falls below this
 * tx_fifo_high_threshold: triggers THRESHOLD_HIGH when Tx level rises above this
 * tx_fifo_low_threshold:  triggers THRESHOLD_LOW  when Tx level falls below this
 */
lr20xx_status_t lr20xx_radio_fifo_cfg_irq( const void* context, lr20xx_radio_fifo_flag_t rx_fifo_irq_enable,
                                           lr20xx_radio_fifo_flag_t tx_fifo_irq_enable, uint16_t rx_fifo_high_threshold,
                                           uint16_t tx_fifo_low_threshold, uint16_t rx_fifo_low_threshold,
                                           uint16_t tx_fifo_high_threshold );

lr20xx_status_t lr20xx_radio_fifo_clear_irq_flags( const void* context, lr20xx_radio_fifo_flag_t rx_fifo_flags_to_clear,
                                                   lr20xx_radio_fifo_flag_t tx_fifo_flags_to_clear );
lr20xx_status_t lr20xx_radio_fifo_get_irq( const void* context, lr20xx_radio_fifo_flag_t* rx_fifo_flags,
                                           lr20xx_radio_fifo_flag_t* tx_fifo_flags );
lr20xx_status_t lr20xx_radio_fifo_get_and_clear_irq_flags( const void* context, lr20xx_radio_fifo_flag_t* rx_fifo_flags,
                                                           lr20xx_radio_fifo_flag_t* tx_fifo_flags );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_FIFO_H

/* --- EOF ------------------------------------------------------------------ */
