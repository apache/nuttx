/**************************************************************************//**
 * @file     rtl8710c_vendor_ctrl.h
 * @brief    Defines macros and data types for the Realtek SoC Vendor functions.
 * @version  v1.00
 * @date     2018/05/11
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
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

/**
 * @addtogroup hs_hal_vendor Vendor
 * @ingroup 8710c_hal
 * @{
 * @brief The HAL API for Vendor Functions Control.
 */

#ifndef RTL8710C_VDR_CTRL_H
#define RTL8710C_VDR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif


/**
  \brief  The data structure to export the Vendor Function Control API in ROM.
*/
typedef struct hal_vdr_func_stubs_s {
    void (*hal_vdr_s_jtag_key_write) (uint8_t *pkey);
    void (*hal_vdr_ns_jtag_key_write) (uint8_t *pkey);
 
    uint32_t reserved[8];  // reserved space for next ROM code version function table extending.
} hal_vdr_func_stubs_t, *phal_vdr_func_stubs_t;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_vendor_rom_func
 * @{
 */

void hal_vdr_s_jtag_key_write_rtl8710c (uint8_t *pkey);
void hal_vdr_ns_jtag_key_write_rtl8710c (uint8_t *pkey);

/** @} */ /* End of group hs_hal_vendor_rom_func */

#ifdef __cplusplus
}
#endif

#endif /* RTL8710C_VDR_CTRL_H */

/** @} */ /* End of group hs_hal_vendor */

