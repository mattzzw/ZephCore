/*!
 * @file      lr20xx_radio_common.h
 *
 * @brief     Radio common driver definition for LR20XX
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

#ifndef LR20XX_RADIO_COMMON_H
#define LR20XX_RADIO_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr20xx_status.h"
#include "lr20xx_radio_common_types.h"
#include <stdint.h>
#include <stdbool.h>

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
 * Calibrate front-end (ADC offset, poly-phase filter, image) from raw calibration values.
 * Must be called when chip is not in Rx/Tx. Up to 3 values; 0 calibrates at next 4MHz multiple of current RF freq.
 * RF ops should stay within 50MHz of a calibrated frequency. Errors readable via lr20xx_system_get_errors.
 */
lr20xx_status_t lr20xx_radio_common_calibrate_front_end(
    const void* context, const lr20xx_radio_common_raw_front_end_calibration_value_t* front_end_calibration_values,
    uint8_t n_front_end_calibration_values );

/*
 * Helper: converts front_end_calibration_value_t structs to raw values and calls calibrate_front_end.
 * Each frequency is rounded up to the next 4MHz multiple.
 */
lr20xx_status_t lr20xx_radio_common_calibrate_front_end_helper(
    const void* context, const lr20xx_radio_common_front_end_calibration_value_t* front_end_calibration_structures,
    uint8_t n_front_end_calibration_structures );

/* Convert milliseconds to 32.768kHz RTC step count */
uint32_t lr20xx_radio_common_convert_time_in_ms_to_rtc_step( uint32_t time_in_ms );

lr20xx_status_t lr20xx_radio_common_set_rf_freq( const void* context, uint32_t freq_in_hz );

lr20xx_status_t lr20xx_radio_common_set_rx_path( const void* context, lr20xx_radio_common_rx_path_t rx_path,
                                                 lr20xx_radio_common_rx_path_boost_mode_t boost_mode );

/* Must be called before lr20xx_radio_common_set_tx_params */
lr20xx_status_t lr20xx_radio_common_set_pa_cfg( const void* context, const lr20xx_radio_common_pa_cfg_t* pa_cfg );

/*
 * Set TX output power (0.5dBm steps) and PA ramp time. Requires prior call to set_pa_cfg.
 * LF PA: power_half_dbm in [0xED, 0x2C] (-9.5 to +22dBm)
 * HF PA: power_half_dbm in [0xD9, 0x18] (-19.5 to +12dBm)
 */
lr20xx_status_t lr20xx_radio_common_set_tx_params( const void* context, const int8_t power_half_dbm,
                                                   const lr20xx_radio_common_ramp_time_t ramp_time );

/* Set RSSI calibration gain tables; NULL pointer skips that path */
lr20xx_status_t lr20xx_radio_common_set_rssi_calibration(
    const void* context, const lr20xx_radio_common_rssi_calibration_gain_table_t* rssi_cal_table_lf,
    const lr20xx_radio_common_rssi_calibration_gain_table_t* rssi_cal_table_hf );

/*
 * Set chip mode after leaving Tx/Rx (successful, timeout, or CAD).
 * Applied on: TX done, RX done (non-continuous), RX duty cycle done, CAD exit, timeout, auto-Tx/Rx transitions.
 */
lr20xx_status_t lr20xx_radio_common_set_rx_tx_fallback_mode( const void*                                context,
                                                             const lr20xx_radio_common_fallback_modes_t fallback_mode );

/*
 * Set packet type; must precede any modulation configuration.
 * Automatically applies lr20xx_workarounds_dcdc_reset unless LR20XX_WORKAROUNDS_DISABLE_AUTOMATIC_DCDC_RESET is defined.
 */
lr20xx_status_t lr20xx_radio_common_set_pkt_type( const void* context, lr20xx_radio_common_pkt_type_t pkt_type );

lr20xx_status_t lr20xx_radio_common_get_pkt_type( const void* context, lr20xx_radio_common_pkt_type_t* pkt_type );

/*
 * Configure when the Rx timeout timer stops:
 * - false: on LoRa header / GFSK syncword detection
 * - true:  on preamble detection
 */
lr20xx_status_t lr20xx_radio_common_set_rx_timeout_stop_event( const void* context,
                                                               const bool  is_stopped_on_preamble_detection );

lr20xx_status_t lr20xx_radio_common_reset_rx_stats( const void* context );

/*
 * Get instantaneous RSSI during active reception.
 * Full precision: RSSI_dBm = rssi_in_dbm - (half_dbm_count * 0.5); half_dbm_count may be NULL.
 */
lr20xx_status_t lr20xx_radio_common_get_rssi_inst( const void* context, int16_t* rssi_in_dbm, uint8_t* half_dbm_count );

lr20xx_status_t lr20xx_radio_common_set_rx( const void* context, const uint32_t timeout_in_ms );

/*
 * Start RX with RTC step timeout. timeout_in_ms = steps / 32.768; max 0xFFFFFE (~511s).
 * 0x000000 = single RX (wait for packet); 0xFFFFFF = continuous RX.
 */
lr20xx_status_t lr20xx_radio_common_set_rx_with_timeout_in_rtc_step( const void*    context,
                                                                     const uint32_t timeout_in_rtc_step );

/* Start RX using timeout pre-configured by set_default_rx_tx_timeout[_in_rtc_step] */
lr20xx_status_t lr20xx_radio_common_set_rx_with_default_timeout( const void* context );

lr20xx_status_t lr20xx_radio_common_set_tx( const void* context, const uint32_t timeout_in_ms );

/* Start TX with RTC step timeout. timeout=0 disables timeout. Max 0xFFFFFF (~511s). */
lr20xx_status_t lr20xx_radio_common_set_tx_with_timeout_in_rtc_step( const void*    context,
                                                                     const uint32_t timeout_in_rtc_step );

/* Start TX using timeout pre-configured by set_default_rx_tx_timeout[_in_rtc_step] */
lr20xx_status_t lr20xx_radio_common_set_tx_with_default_timeout( const void* context );

lr20xx_status_t lr20xx_radio_common_set_tx_test_mode( const void* context, lr20xx_radio_common_tx_test_mode_t mode );

/* Select PA; set_pa_cfg must be called first */
lr20xx_status_t lr20xx_radio_common_select_pa( const void* context, lr20xx_radio_common_pa_selection_t sel );

/* Converts ms to RTC steps and calls set_rx_duty_cycle_with_timing_in_rtc_step */
lr20xx_status_t lr20xx_radio_common_set_rx_duty_cycle( const void* context, const uint32_t rx_period_in_ms,
                                                       const uint32_t sleep_period_in_ms,
                                                       const lr20xx_radio_common_rx_duty_cycle_mode_t mode );

/*
 * Start RX duty cycle: Rx for rx_period, then sleep for sleep_period, repeat.
 * On activity detected: extend Rx timeout to (2*rx_period + sleep_period).
 * On packet received: return to fallback mode.
 * CAD mode (LoRa only): configure CAD params before calling.
 * To stop during sleep: call lr20xx_system_wakeup then lr20xx_system_set_standby_mode when BUSY goes low.
 */
lr20xx_status_t lr20xx_radio_common_set_rx_duty_cycle_with_timing_in_rtc_step(
    const void* context, const uint32_t rx_period_in_rtc_step, const uint32_t sleep_period_in_rtc_step,
    const lr20xx_radio_common_rx_duty_cycle_mode_t mode );

/*
 * Configure automatic Tx-after-Rx or Rx-after-Tx.
 * Set mode to Tx → auto-Rx fires; set mode to Rx → auto-Tx fires.
 * Between operations the chip enters fallback mode. Triggers once then auto-disables.
 * delay_in_tick must account for PA ramp, TCXO start, fallback mode switching.
 * Pass condition=LR20XX_RADIO_COMMON_AUTO_TX_RX_OFF to disable.
 */
lr20xx_status_t lr20xx_radio_common_configure_auto_tx_rx(
    const void* context, const lr20xx_radio_common_auto_tx_rx_configuration_t* configuration );

lr20xx_status_t lr20xx_radio_common_get_rx_packet_length( const void* context, uint16_t* pkt_len );

lr20xx_status_t lr20xx_radio_common_set_default_rx_tx_timeout( const void* context, uint32_t rx_timeout_in_ms,
                                                               uint32_t tx_timeout_in_ms );

/* Same special values apply as for set_rx/tx_with_timeout_in_rtc_step */
lr20xx_status_t lr20xx_radio_common_set_default_rx_tx_timeout_in_rtc_step( const void* context,
                                                                           uint32_t    rx_timeout_in_rtc_step,
                                                                           uint32_t    tx_timeout_in_rtc_step );

/* Arm a 32MHz timestamp on the given radio event; read with get_elapsed_time_in_tick */
lr20xx_status_t lr20xx_radio_common_set_timestamp_source( const void*                              context,
                                                          lr20xx_radio_common_timestamp_cfg_slot_t cfg_slot,
                                                          lr20xx_radio_common_timestamp_source_t   source );

/*
 * Read elapsed 32MHz ticks since the event configured in set_timestamp_source for cfg_slot.
 * Radio must not have entered sleep between the event and this call.
 */
lr20xx_status_t lr20xx_radio_common_get_elapsed_time_in_tick( const void*                              context,
                                                              lr20xx_radio_common_timestamp_cfg_slot_t cfg_slot,
                                                              uint32_t* elapsed_time_in_tick );

/* Start CCA (Clear Channel Assessment); duration in 32MHz steps */
lr20xx_status_t lr20xx_radio_common_set_cca( const void* context, const uint32_t duration );

lr20xx_status_t lr20xx_radio_common_get_cca_result( const void* context, lr20xx_radio_common_cca_res_t* cca_res );

lr20xx_status_t lr20xx_radio_common_set_agc_gain( const void* context, lr20xx_radio_common_gain_step_t gain );

/* Set non-LoRa CAD parameters; not applicable when packet type is LoRa */
lr20xx_status_t lr20xx_radio_common_set_cad_params( const void*                             context,
                                                    const lr20xx_radio_common_cad_params_t* params );

lr20xx_status_t lr20xx_radio_common_set_cad( const void* context );

/*
 * Get LQI of last detected packet. Valid from preamble detection until next Rx call.
 * Applies to FSK-based modes: FSK, BLE, OQPSK-15.4, Wi-SUN, Wireless M-Bus, Z-Wave.
 */
lr20xx_status_t lr20xx_radio_common_get_lqi( const void* context, lr20xx_radio_common_lqi_value_t* lqi );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_COMMON_H

/* --- EOF ------------------------------------------------------------------ */
