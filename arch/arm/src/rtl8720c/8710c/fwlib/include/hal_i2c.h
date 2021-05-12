/**************************************************************************//**
 * @file     hal_i2c.h
 * @brief    The HAL API implementation for the I2C device.
 * @version  V1.00
 * @date     2016-12-14
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/


#ifndef _HAL_I2C_H_
#define _HAL_I2C_H_

#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup hs_hal_i2c I2C
 * @ingroup 8710c_hal
 * @{
 * @brief The I2C HAL module of the AmebaZ2 platform.
 */

/*  Macros for hp i2c module system related configuration  */
/** @defgroup GROUP_I2C_MODULE_SYSTEM_CONFIGURATION I2C SYSTEM CONFIGURATION
 *  i2c system related configuration
 *  @{
 */
#define I2C_GDMA_MAX_BLOCK      16
//#define I2C_CACHE_RAM_VERSION   1
#undef I2C_CACHE_RAM_VERSION
/** @} */ // end of GROUP_I2C_MODULE_SYSTEM_CONFIGURATION


/** \brief Description of hal_i2c_read_dat_reg
 *
 *    hal_i2c_read_dat_reg is to read I2C data regsiter directly.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return uint8_t      I2C received data.
 */
static inline uint8_t hal_i2c_read_dat_reg (hal_i2c_adapter_t *phal_i2c_adapter)
{
    return (uint8_t)phal_i2c_adapter->init_dat.reg_base->dat_cmd_b.dat;
}

/** \brief Description of hal_i2c_get_rx_flr
 *
 *    hal_i2c_get_rx_flr is to read I2C RX FIFO level.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return uint32_t      I2C hardware RX FIFO level.
 */
static inline uint32_t hal_i2c_get_rx_flr (hal_i2c_adapter_t *phal_i2c_adapter)
{
    return (uint32_t)phal_i2c_adapter->init_dat.reg_base->rxflr_b.rxflr;
}

/** \brief Description of hal_i2c_get_tx_flr
 *
 *    hal_i2c_get_tx_flr is to read I2C TX FIFO level.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return uint32_t      I2C hardware TX FIFO level.
 */
static inline uint32_t hal_i2c_get_tx_flr (hal_i2c_adapter_t *phal_i2c_adapter)
{
    return (uint32_t)phal_i2c_adapter->init_dat.reg_base->txflr_b.txflr;
}

/** \brief Description of hal_i2c_slv_set_for_mst_rd_cmd
 *
 *    hal_i2c_slv_set_for_mst_rd_cmd is to set interrupt mask for Read Request interrupt which is from
 *    other masters.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return void
 */
static inline void hal_i2c_slv_set_for_mst_rd_cmd (hal_i2c_adapter_t *phal_i2c_adapter)
{
    phal_i2c_adapter->init_dat.reg_base->intr_msk_b.rd_req = I2CEnable;
}

/** \brief Description of hal_i2c_slv_clear_for_mst_rd_cmd
 *
 *    hal_i2c_slv_clear_for_mst_rd_cmd is to clear interrupt mask for Read Request interrupt which is from
 *    other masters.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return void
 */
static inline void hal_i2c_slv_clear_for_mst_rd_cmd (hal_i2c_adapter_t *phal_i2c_adapter)
{
    phal_i2c_adapter->init_dat.reg_base->intr_msk_b.rd_req = I2CDisable;
}

/** \brief Description of hal_i2c_mst_addr_retry_ctrl
 *
 *    hal_i2c_mst_addr_retry_ctrl is to enable/disable master address retry feature.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \param[in] uint8_t retry_en:           0: disable, 1: enable.
 *   \return void
 */
static inline void hal_i2c_mst_addr_retry_ctrl (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t retry_en)
{
    if (retry_en) {
        phal_i2c_adapter->mst_spe_func |= I2CAddressRetry;
    } else {
        phal_i2c_adapter->mst_spe_func &= (~I2CAddressRetry);
    }
}

/** \brief Description of hal_i2c_mst_restr_sw_ctrl
 *
 *    hal_i2c_mst_restr_sw_ctrl is to control software flag for RESTART generation.\n
 *    If RESTART software control is enabled, stop control (i2c_tx_info_t.mst_stop or i2c_rx_info_t.mst_stop) is \n
 *    ignored when executing the last data read/write operation.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \param[in] uint8_t restr_en:                RESTART is enable/disable.
 *   \return void
 */
static inline void hal_i2c_mst_restr_sw_ctrl (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t restr_en)
{
    if (restr_en) {
        phal_i2c_adapter->mst_spe_func |= I2CMasterRestart;
    } else {
        phal_i2c_adapter->mst_spe_func &= (~I2CMasterRestart);
    }
}

/** \brief Description of hal_i2c_set_slv_ack_addr
 *
 *    hal_i2c_set_slv_ack_addr is to set I2C slave ack address according to the given index.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:      pointer to I2C control adapter.
 *   \param[in] uint8_t addr_idx:                   address index.
 *   \param[in] uint32_t ack_addr:                  ack address.
 *   \return void
 */
static inline void hal_i2c_set_slv_ack_addr (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t addr_idx, uint32_t ack_addr)
{
    if (!addr_idx) {
        phal_i2c_adapter->init_dat.ack_addr0 = ack_addr;
        phal_i2c_adapter->init_dat.reg_base->sar = ack_addr;
    } else {
        phal_i2c_adapter->init_dat.ack_addr1 = ack_addr;
        phal_i2c_adapter->init_dat.reg_base->sar1 = ack_addr;
    }
}

uint8_t hal_i2c_timeout_chk (hal_i2c_adapter_t *phal_i2c_adapter, uint32_t start_cnt);
uint8_t hal_i2c_chk_mod (hal_i2c_adapter_t *phal_i2c_adapter);
uint8_t hal_i2c_pure_init (hal_i2c_adapter_t *phal_i2c_adapter);
uint8_t hal_i2c_pure_deinit (hal_i2c_adapter_t *phal_i2c_adapter);
uint8_t hal_i2c_en_ctrl (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable);
hal_status_t hal_i2c_set_clk (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_intr_ctrl (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t set_clr, uint16_t intr_msk);
hal_status_t hal_i2c_wr (hal_i2c_adapter_t *phal_i2c_adapter, const uint8_t *dat_buf, uint32_t dat_len, uint8_t ctrl);
void hal_i2c_mst_send_rdcmd (hal_i2c_adapter_t *phal_i2c_adapter, uint32_t cmd_len, uint8_t ctrl);
uint32_t hal_i2c_dma_ctrl (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable);
hal_status_t hal_i2c_mst_restr_ctrl (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t restr_en);
hal_status_t hal_i2c_mst_gc_sb_ctrl (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable, uint8_t gc_sb);
hal_status_t hal_i2c_slv_no_ack_ctrl (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t no_ack_en);
uint8_t hal_i2c_slv_no_ack_sts (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_slv_ack_gc_ctrl (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t slv_gc_en);
uint8_t hal_i2c_slv_ack_gc_sts (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_slv_to_slp (hal_i2c_adapter_t *phal_i2c_adapter);
uint8_t hal_i2c_slv_chk_mst_wr (hal_i2c_adapter_t *phal_i2c_adapter);
uint8_t hal_i2c_slv_chk_rd_req (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_power_init (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_power_deinit (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_init (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t scl_pin, uint8_t sda_pin);
hal_status_t hal_i2c_deinit (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_load_default (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t index);
hal_status_t hal_i2c_set_tar (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t mst_rw);
hal_status_t hal_i2c_send_dma_init (hal_i2c_adapter_t *phal_i2c_adapter, hal_gdma_adaptor_t *padc_gdma_tx_adaptor);
hal_status_t hal_i2c_send_dma_deinit (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_send (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_recv_dma_init (hal_i2c_adapter_t *phal_i2c_adapter, hal_gdma_adaptor_t *padc_gdma_rx_adaptor);
hal_status_t hal_i2c_recv_dma_deinit (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_receive (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_set_sar (hal_i2c_adapter_t *phal_i2c_adapter, uint8_t sar_idx, uint16_t slv_addr);
hal_status_t hal_i2c_slv_recv (hal_i2c_adapter_t *phal_i2c_adapter);
hal_status_t hal_i2c_slv_send (hal_i2c_adapter_t *phal_i2c_adapter);

/** @} */ /* End of group hs_hal_i2c */

#ifdef  __cplusplus
}
#endif

#endif  // end of "#define _HAL_I2C_H_"

