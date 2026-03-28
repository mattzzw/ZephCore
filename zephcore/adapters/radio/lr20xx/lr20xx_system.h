/*!
 * @file      lr20xx_system.h
 *
 * @brief     System driver definition for LR20XX
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

#ifndef LR20XX_SYSTEM_H
#define LR20XX_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_system_types.h"
#include "lr20xx_status.h"

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

lr20xx_status_t lr20xx_system_reset( const void* context );
lr20xx_status_t lr20xx_system_wakeup( const void* context );

/*
 * Return stat1, stat2, and irq_status. Any pointer may be NULL.
 * Implemented as a bare SPI read (NOP bytes on MOSI); does NOT execute the GetStatus command.
 * The LR20XX prefixes every SPI read response with stat1/stat2/irq_status automatically.
 * Reset status in stat2 is NOT cleared by this call — use lr20xx_system_clear_reset_status_info.
 */
lr20xx_status_t lr20xx_system_get_status( const void* context, lr20xx_system_stat1_t* stat1,
                                          lr20xx_system_stat2_t* stat2, lr20xx_system_irq_mask_t* irq_status );

lr20xx_status_t lr20xx_system_clear_reset_status_info( const void* context );

/*
 * Read firmware version. Expected: LR2021 = major 0x01 / minor 0x18; LR2022 = major 0x02 / minor 0x00.
 */
lr20xx_status_t lr20xx_system_get_version( const void* context, lr20xx_system_version_t* version );

/*
 * Return system error flags. Remediation: calibration errors → retry RC calibration;
 * XOSC errors → hardware issue, reset; PLL lock errors → run PLL calibration or change frequency.
 */
lr20xx_status_t lr20xx_system_get_errors( const void* context, lr20xx_system_errors_t* errors );
lr20xx_status_t lr20xx_system_clear_errors( const void* context );

/* Return number of available DIOs (valid range for lr20xx_system_dio_get_nth) */
uint8_t lr20xx_system_dio_get_count( void );

/* Return nth DIO enum value; returns false if nth >= dio_count */
bool lr20xx_system_dio_get_nth( uint8_t nth, lr20xx_system_dio_t* dio );

/*
 * Configure DIO function and drive mode. DIO state is re-evaluated on command.
 * drive applied on sleep entry (if func != NONE); reset to NONE (or PULL_UP for DIO5/6) on wake without retention.
 * TX/RX_TRIGGER uses the default timeout configured via set_default_rx_tx_timeout.
 * LF_CLK_OUT only valid on DIO7–DIO11.
 * DIO5/DIO6 must be set to FUNC_NONE if connected to external components that could toggle them during cold-start.
 */
lr20xx_status_t lr20xx_system_set_dio_function( const void* context, lr20xx_system_dio_t dio,
                                                lr20xx_system_dio_func_t func, lr20xx_system_dio_drive_t drive );

/* Set RF switch configuration for a DIO; DIO state re-evaluated on command */
lr20xx_status_t lr20xx_system_set_dio_rf_switch_cfg( const void* context, lr20xx_system_dio_t dio,
                                                     const lr20xx_system_dio_rf_switch_cfg_t rf_switch_cfg );

/*
 * Map IRQs to a DIO. Each IRQ can only be mapped to one DIO at a time (last write wins).
 * DIO state re-evaluated on command.
 */
lr20xx_status_t lr20xx_system_set_dio_irq_cfg( const void* context, lr20xx_system_dio_t dio,
                                               const lr20xx_system_irq_mask_t irq_cfg );

lr20xx_status_t lr20xx_system_clear_irq_status( const void* context, const lr20xx_system_irq_mask_t irqs_to_clear );

/* Atomically clear and return pending IRQ flags */
lr20xx_status_t lr20xx_system_get_and_clear_irq_status( const void* context, lr20xx_system_irq_mask_t* irq );

/*
 * Select LF clock source. When switching to LR20XX_SYSTEM_LFCLK_EXT, the external clock must already be running
 * and must remain running. Call lr20xx_system_calibrate after changing LF clock source.
 */
lr20xx_status_t lr20xx_system_cfg_lfclk( const void* context, const lr20xx_system_lfclk_cfg_t lfclock_cfg );

/*
 * Set HF clock output scaling on the DIO configured with LR20XX_SYSTEM_DIO_FUNC_HF_CLK_OUT.
 */
lr20xx_status_t lr20xx_system_cfg_clk_output( const void* context, lr20xx_system_hf_clk_scaling_t hf_clk_scaling );

/*
 * Configure TCXO supply voltage and start delay. start_delay_in_32mhz_step is a gating timeout in 32MHz ticks
 * (1 tick = 31.25ns); max value 0xFFFFFFFF. Set to 0 to disable TCXO mode.
 * If TCXO does not start within the delay, LR20XX_SYSTEM_ERRORS_HF_XOSC_START_MASK is set (check get_errors).
 * If 32MHz RC is uncalibrated, actual start time may be up to 2x the configured delay.
 */
lr20xx_status_t lr20xx_system_set_tcxo_mode( const void*                               context,
                                             const lr20xx_system_tcxo_supply_voltage_t supply_voltage,
                                             const uint32_t                            start_delay_in_32mhz_step );

/* Set regulator mode; controls whether DCDC is enabled in STANDBY_XOSC, FS, RX, TX modes */
lr20xx_status_t lr20xx_system_set_reg_mode( const void* context, const lr20xx_system_reg_mode_t reg_mode );

/*
 * Calibrate selected blocks (bitmask of lr20xx_system_calibration_e). Can be called from any mode.
 * Chip returns to STANDBY_RC on exit. Errors readable via lr20xx_system_get_errors.
 * Run at boot; re-run AAF_MASK if temperature changes by >20°C; MU_MASK needs boot calibration only.
 */
lr20xx_status_t lr20xx_system_calibrate( const void*                            context,
                                         const lr20xx_system_calibration_mask_t blocks_to_calibrate );

/*
 * Read supply voltage. RAW format: Vbat_V = (vbat/8192 × 5 - 1) × Vana (Vana typ. 1.35V; vbat is 13-bit).
 * UNIT format: result in mV.
 */
lr20xx_status_t lr20xx_system_get_vbat( const void* context, lr20xx_system_value_format_t format,
                                        lr20xx_system_meas_res_t res, uint16_t* vbat );

/*
 * Read internal junction temperature. RAW format: Temp_°C = (temp[12:0]/8192 × Vana - Vbe25) × 1000/VbeSlope + 25
 * (Vana typ. 1.35V, Vbe25 typ. 0.7295V, VbeSlope typ. -1.7mV/°C). UNIT format: °C in 13.5sb (integer + fractional).
 * Configure TCXO with set_tcxo_mode before calling if TCXO is used.
 */
lr20xx_status_t lr20xx_system_get_temp( const void* context, lr20xx_system_value_format_t format,
                                        lr20xx_system_meas_res_t res, lr20xx_system_temp_src_t src, uint16_t* temp );

/*
 * Read a 32-bit random number. Not suitable for cryptographic use. Radio must be in standby mode.
 */
lr20xx_status_t lr20xx_system_get_random_number( const void*                                   context,
                                                 lr20xx_system_random_entropy_source_bitmask_t source,
                                                 uint32_t*                                     random_number );

/* Enter sleep mode; sleep_time in LF clock steps (0 = sleep until wakeup pin) */
lr20xx_status_t lr20xx_system_set_sleep_mode( const void* context, const lr20xx_system_sleep_cfg_t* sleep_cfg,
                                              const uint32_t sleep_time );

lr20xx_status_t lr20xx_system_set_standby_mode( const void* context, const lr20xx_system_standby_mode_t standby_mode );
lr20xx_status_t lr20xx_system_set_fs_mode( const void* context );

/*
 * Add a register to the sleep retention list. slot in [0:31]; address must be word-aligned (only 3 LSBs significant).
 * Up to 32 additional registers beyond the hardware defaults can be retained across retentionless sleep.
 */
lr20xx_status_t lr20xx_system_add_register_to_retention_mem( const void* context, uint8_t slot, uint32_t address );

lr20xx_status_t lr20xx_system_set_lbd_cfg( const void* context, bool is_enabled, lr20xx_system_lbd_trim_t trim );

/*
 * Configure internal XTAL trim capacitors and post-ready wait time. XTA: 11.3pF + xta×0.47pF (max 47 steps = 33.39pF).
 * XTB: 10.1pF + xtb×0.47pF (max 47 steps = 32.19pF). wait_time_us is an additional delay after XTAL readiness.
 */
lr20xx_status_t lr20xx_system_configure_xosc( const void* context, uint8_t xta, uint8_t xtb, uint8_t wait_time_us );

/*
 * Configure TX heating compensation for XTAL 32MHz (not TCXO). Fails if TCXO is configured.
 * Set is_ntc_en if an external NTC temperature sensor is present.
 */
lr20xx_status_t lr20xx_system_set_temp_comp_cfg( const void* context, lr20xx_system_temp_comp_mode_t mode,
                                                 bool is_ntc_en );

/* Set NTC parameters: ntc_r_ratio is 10.9b resistance bias ratio at 25°C; ntc_beta in units of 2K */
lr20xx_status_t lr20xx_system_set_ntc_params( const void* context, uint16_t ntc_r_ratio, uint16_t ntc_beta,
                                              uint8_t delay );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_SYSTEM_H

/* --- EOF ------------------------------------------------------------------ */
