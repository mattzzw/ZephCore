/*!
 * @file      lr20xx_workarounds.h
 *
 * @brief     System driver workarounds definition for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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

#ifndef LR20XX_WORKAROUNDS_H
#define LR20XX_WORKAROUNDS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "lr20xx_status.h"
#include "lr20xx_radio_fsk_common_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#ifndef LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_RESET
#define LR20XX_WORKAROUNDS_CONDITIONAL_APPLY_AUTOMATIC_DCDC_RESET( cont ) lr20xx_workarounds_dcdc_reset( cont )
#else
#define LR20XX_WORKAROUNDS_CONDITIONAL_APPLY_AUTOMATIC_DCDC_RESET( cont ) LR20XX_STATUS_OK
#endif  // LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_RESET

#ifndef LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_CONFIGURE
#define LR20XX_WORKAROUNDS_CONDITIONAL_APPLY_AUTOMATIC_DCDC_CONFIGURE( cont ) lr20xx_workarounds_dcdc_configure( cont )
#else
#define LR20XX_WORKAROUNDS_CONDITIONAL_APPLY_AUTOMATIC_DCDC_CONFIGURE( cont ) LR20XX_STATUS_OK
#endif  // LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_CONFIGURE

#ifndef LR20XX_WORKAROUND_DISABLE_AUTOMATIC_BLE_2MBPS_PREAMBLE_LENGTH
#define LR20XX_WORKAROUND_CONDITIONAL_APPLY_BLE_2MBPS_PREAMBLE_LENGTH( cont ) \
    lr20xx_workarounds_bluetooth_le_2mbps_preamble_length( cont )
#else
#define LR20XX_WORKAROUND_CONDITIONAL_APPLY_BLE_2MBPS_PREAMBLE_LENGTH( cont ) LR20XX_STATUS_OK
#endif  // LR20XX_WORKAROUND_DISABLE_AUTOMATIC_BLE_2MBPS_PREAMBLE_LENGTH

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

/* Call after lr20xx_radio_bluetooth_le_set_pkt_params when PHY is LE_CODED_125KB or LE_CODED_500KB */
lr20xx_status_t lr20xx_workarounds_bluetooth_le_phy_coded_syncwords( const void* context );

/* Call after lr20xx_radio_bluetooth_le_set_pkt_params when PHY is LE_CODED_125KB or LE_CODED_500KB */
lr20xx_status_t lr20xx_workarounds_bluetooth_le_phy_coded_frequency_drift( const void* context );

/* Persist BLE coded-PHY frequency drift register across sleep; slot in [0:31] */
lr20xx_status_t lr20xx_workarounds_bluetooth_le_phy_coded_frequency_drift_store_retention_mem( const void* context,
                                                                                               uint8_t     slot );

/*
 * Fix incorrect default preamble length for BLE 2Mbps mode.
 * Call after lr20xx_radio_bluetooth_le_set_modulation_params when PHY is LE_2M.
 * Applied automatically unless LR20XX_WORKAROUND_DISABLE_AUTOMATIC_BLE_2MBPS_PREAMBLE_LENGTH is defined.
 */
lr20xx_status_t lr20xx_workarounds_bluetooth_le_2mbps_preamble_length( const void* context );

/*
 * Enable SX1276 LoRa compatibility: SF6 implicit mode and syncword nibbles > 7 for all SF.
 * Call after lr20xx_radio_lora_set_modulation_params.
 */
lr20xx_status_t lr20xx_workarounds_lora_enable_sx1276_compatibility_mode( const void* context );

/* Disable SX1276 LoRa compatibility; may be called before or after lr20xx_radio_lora_set_modulation_params */
lr20xx_status_t lr20xx_workarounds_lora_disable_sx1276_compatibility_mode( const void* context );

/* Persist SX1276 LoRa compatibility register across sleep; slot in [0:31] */
lr20xx_status_t lr20xx_workarounds_lora_sx1276_compatibility_mode_store_retention_mem( const void* context,
                                                                                       uint8_t     slot );

/* Enable SX1276 freq-hopping compatibility; call after lr20xx_radio_lora_set_freq_hop */
lr20xx_status_t lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatibility_mode( const void* context );

/* Disable SX1276 freq-hopping compatibility */
lr20xx_status_t lr20xx_workarounds_lora_freq_hop_disable_sx1276_compatibility_mode( const void* context );

/* Persist SX1276 freq-hopping compatibility register across sleep; slot in [0:31] */
lr20xx_status_t lr20xx_workarounds_lora_freq_hop_sx1276_compatibility_mode_store_retention_mem( const void* context,
                                                                                                uint8_t     slot );

/*
 * Override OOK detection threshold. The default chip-computed value may be too conservative, raising PER.
 * Set to the noise floor (from lr20xx_radio_common_get_rssi_inst) if it exceeds the default.
 * Call after lr20xx_radio_ook_set_modulation_params. threshold_level_db in dBm.
 */
lr20xx_status_t lr20xx_workarounds_ook_set_detection_threshold_level( const void* context, int16_t threshold_level_db );

/*
 * Return default OOK detection threshold (dBm) for the given bandwidth.
 * Returns 0 for unknown bandwidth values.
 */
int16_t lr20xx_workarounds_ook_get_default_detection_threshold_level( lr20xx_radio_fsk_common_bw_t bw );

/*
 * Truncate internal PLL frequency to the nearest 122Hz multiple for RTToF accuracy.
 * Call after lr20xx_radio_common_set_rf_freq for RTToF ranging; adjusts RF freq by ≤122Hz.
 */
lr20xx_status_t lr20xx_workarounds_rttof_truncate_pll_freq_step( const void* context );

/*
 * Correct RTToF raw RSSI values using gain/offset read from hardware registers.
 * raw = -(rssi_dB * 2); rssi_dB = -(raw / 2).
 * rssi2_raw_fixed may be null for single-RSSI (normal) results.
 */
lr20xx_status_t lr20xx_workarounds_rttof_rssi_computation( const void* context, uint8_t rssi1_raw_value,
                                                           uint8_t rssi2_raw_value, uint8_t* rssi1_raw_fixed,
                                                           uint8_t* rssi2_raw_fixed );

/*
 * Reset DCDC switcher to default timing.
 * Required after lr20xx_radio_common_set_pkt_type when: sub-GHz RX + DCDC regulator mode.
 */
lr20xx_status_t lr20xx_workarounds_dcdc_reset( const void* context );

/*
 * Configure DCDC switcher timing based on current RX path.
 * Required after any of: fsk/flrc/ook/lora set_modulation_params, z_wave_set_params, set_rx_path;
 * when: sub-GHz RX + DCDC regulator mode.
 */
lr20xx_status_t lr20xx_workarounds_dcdc_configure( const void* context );

/* Persist DCDC switcher register across sleep; slot in [0:31] */
lr20xx_status_t lr20xx_workarounds_dcdc_store_retention_mem( const void* context, uint8_t slot );

/*
 * Reduce RTToF result deviation on fractional bandwidths (BW_812/406/203/101).
 * Call after lr20xx_radio_lora_set_modulation_params; reset by subsequent set_modulation_params.
 * is_manager: true for RTToF manager role, false for subordinate.
 */
lr20xx_status_t lr20xx_workarounds_rttof_results_deviation( const void* context, bool is_manager );

/* Persist RTToF deviation workaround registers (two slots) across sleep; slots in [0:31] */
lr20xx_status_t lr20xx_workarounds_rttof_results_deviation_store_retention_mem( const void* context, uint8_t slot_1,
                                                                                uint8_t slot_2 );

/* Enable RTToF extended-mode workaround; required when using LR20XX_RTTOF_MODE_EXTENDED */
lr20xx_status_t lr20xx_workarounds_rttof_extended_stuck_second_request_enable( const void* context );

/* Disable RTToF extended-mode workaround when switching back to LR20XX_RTTOF_MODE_NORMAL */
lr20xx_status_t lr20xx_workarounds_rttof_extended_stuck_second_request_disable( const void* context );

/* Persist RTToF extended-mode workaround register across sleep; slot in [0:31] */
lr20xx_status_t lr20xx_workarounds_rttof_extended_stuck_second_request_store_retention_mem( const void* context,
                                                                                            uint8_t     slot );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_WORKAROUNDS_H

/* --- EOF ------------------------------------------------------------------ */