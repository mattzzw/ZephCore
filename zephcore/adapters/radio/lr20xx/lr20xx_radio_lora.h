/*!
 * @file      lr20xx_radio_lora.h
 *
 * @brief     Radio LoRa driver definition for LR20XX
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

#ifndef LR20XX_RADIO_LORA_H
#define LR20XX_RADIO_LORA_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_status.h"
#include "lr20xx_radio_lora_types.h"

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
 * Set LoRa modulation params. Applies lr20xx_workarounds_dcdc_configure automatically unless
 * LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_CONFIGURE. For RTToF with fractional BW, also call
 * lr20xx_workarounds_rttof_results_deviation. See lr20xx_radio_lora_get_recommended_ppm_offset for PPM offset.
 */
lr20xx_status_t lr20xx_radio_lora_set_modulation_params( const void*                           context,
                                                         const lr20xx_radio_lora_mod_params_t* mod_params );

/*
 * Set LoRa packet params. In LR20XX_RADIO_LORA_PKT_EXPLICIT mode: pld_len_in_bytes=0 accepts all lengths;
 * pld_len_in_bytes>0 accepts [1:pld_len_in_bytes], rejecting 0 or >pld_len_in_bytes with LORA_HEADER_ERROR IRQ.
 */
lr20xx_status_t lr20xx_radio_lora_set_packet_params( const void*                           context,
                                                     const lr20xx_radio_lora_pkt_params_t* pkt_params );

/*
 * Configure preamble-absent Rx timeout in LoRa symbols. n_symbols=0 disables.
 * n_symbols>255 delegates to configure_timeout_by_mantissa_exponent_symbols via lr20xx_radio_convert_nb_symb_to_mant_exp.
 */
lr20xx_status_t lr20xx_radio_lora_configure_timeout_by_number_of_symbols( const void* context, uint16_t n_symbols );

/*
 * Configure preamble-absent Rx timeout via mantissa/exponent encoding: N = mant × 2^(2×exp+1).
 * mantissa in [0:31], exponent in [0:7]. N=0 disables.
 */
lr20xx_status_t lr20xx_radio_lora_configure_timeout_by_mantissa_exponent_symbols( const void* context, uint8_t mantissa,
                                                                                  uint8_t exponent );

/*
 * Compute mantissa/exponent [mant, exp] encoding for nb_symbol, finding the smallest N >= nb_symbol
 * where N = mant × 2^(2×exp+1). Returns the actual N value used.
 */
uint16_t lr20xx_radio_convert_nb_symb_to_mant_exp( const uint16_t nb_symbol, uint8_t* mant, uint8_t* exp );

/*
 * Set LoRa syncword. Default 0x12 (LoRaWAN private). 0x34 = LoRaWAN public.
 * Syncword is two 4-bit nibbles: syncword = ((block1 & 0x0F) << 4) | (block2 & 0x0F).
 * block1 must not be 0. With SX1276 compatibility disabled, blocks are 4-bit signed [-8:7];
 * with compatibility enabled they are 4-bit unsigned [0:15] matching SX127x.
 * Different syncwords reduce but do not guarantee rejection of foreign packets.
 */
lr20xx_status_t lr20xx_radio_lora_set_syncword( const void* context, uint8_t syncword );

lr20xx_status_t lr20xx_radio_lora_configure_cad_params( const void*                           context,
                                                        const lr20xx_radio_lora_cad_params_t* cad_params );

/*
 * Start CAD. Raises CAD_DONE IRQ; also raises CAD_DETECTED if signal found. Depending on cad_params exit_mode,
 * chip either returns to fallback or starts the configured exit operation (RX or TX), whose IRQ fires after CAD IRQ.
 */
lr20xx_status_t lr20xx_radio_lora_set_cad( const void* context );

/* Get Rx statistics. Reset on POR, retentionless sleep, or lr20xx_radio_common_reset_rx_stats */
lr20xx_status_t lr20xx_radio_lora_get_rx_statistics( const void*                        context,
                                                     lr20xx_radio_lora_rx_statistics_t* statistics );

/*
 * Get last received LoRa packet status. Valid after RX_DONE or CAD_DONE, until next set_packet_params call.
 * CRC/CR: from received header in EXPLICIT mode; from receiver config in IMPLICIT mode.
 */
lr20xx_status_t lr20xx_radio_lora_get_packet_status( const void*                        context,
                                                     lr20xx_radio_lora_packet_status_t* pkt_status );

/*
 * Configure LoRa address filtering. address_offset: payload byte offset of address field (header not counted).
 * address_length in [0:8]; 0 disables filtering.
 */
lr20xx_status_t lr20xx_radio_lora_set_address( const void* context, uint8_t address_offset, uint8_t address_length,
                                               const uint8_t* address );

/*
 * Configure LoRa intra-packet frequency hopping. For SX1276 compatibility, call
 * lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatibility_mode after this.
 */
lr20xx_status_t lr20xx_radio_lora_set_freq_hop( const void* context, const lr20xx_radio_lora_hopping_cfg_t* cfg );

/* Configure up to 3 CAD side detectors (additional SF detectors for CAD). n_side_detector_cad_configurations in [0:3] */
lr20xx_status_t lr20xx_radio_lora_configure_side_detector_cad(
    const void* context, const lr20xx_radio_lora_side_detector_cad_configuration_t* side_detector_cad_configurations,
    uint8_t n_side_detector_cad_configurations );

/*
 * Configure up to 3 LoRa side detectors (multi-SF Rx on same BW). n_side_detector_cfgs=0 or set_modulation_params
 * disables all. Constraints: Rx-SF < side-SF; CAD-SF > side-SF; BW>=500 limits to 2 (or 1 if main SF>=SF10);
 * all SFs distinct; max SF spread = 4. Demodulated SF readable via get_packet_status after RX_DONE.
 */
lr20xx_status_t lr20xx_radio_lora_configure_side_detectors(
    const void* context, const lr20xx_radio_lora_side_detector_cfg_t* side_detector_cfgs,
    uint8_t n_side_detector_cfgs );

/* Set syncwords for up to 3 side detectors. n_syncword in [0:3]; unconfigured syncwords use default */
lr20xx_status_t lr20xx_radio_lora_set_side_detector_syncwords( const void* context, const uint8_t* syncword,
                                                               uint8_t n_syncword );

/* Time-on-air numerator (divide by BW in Hz to get seconds) */
uint32_t lr20xx_radio_lora_get_time_on_air_numerator( const lr20xx_radio_lora_pkt_params_t* pkt_p,
                                                      const lr20xx_radio_lora_mod_params_t* mod_p );

uint32_t lr20xx_radio_lora_get_bw_in_hz( lr20xx_radio_lora_bw_t bw );
uint32_t lr20xx_radio_lora_get_time_on_air_in_ms( const lr20xx_radio_lora_pkt_params_t* pkt_p,
                                                  const lr20xx_radio_lora_mod_params_t* mod_p );

/*
 * Recommended ppm_offset for given SF/BW:
 *   NO_PPM: SF<11, or BW>=500, or BW=250+SF11
 *   PPM_1_4: SF11/12 at BW<=406 (SX128x compat), BW=250+SF12, or SF12 at other narrow BWs
 * See lr20xx_radio_lora_set_modulation_params.
 */
lr20xx_radio_lora_ppm_t lr20xx_radio_lora_get_recommended_ppm_offset( lr20xx_radio_lora_sf_t sf,
                                                                      lr20xx_radio_lora_bw_t bw );
#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_LORA_H

/* --- EOF ------------------------------------------------------------------ */
