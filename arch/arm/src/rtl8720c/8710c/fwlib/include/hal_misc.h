/**************************************************************************//**
 * @file     hal_misc.h
 * @brief    The HAL Misc. API implementation.
 * @version  V1.00
 * @date     2017-03-24
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

#ifndef _HAL_MISC_H_
#define _HAL_MISC_H_
#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup hs_hal_misc Misc. Functions
 * @{
 */

extern const hal_misc_func_stubs_t hal_misc_stubs;

/**
  \brief  The data structure of the Co-Processor firmware image information (Start location & Image size)
*/
typedef struct cp_fw_info_s {
    uint32_t start_addr;
    uint32_t image_size;
} cp_fw_info_t;

/**
  \brief  The data structure for the collection of all Co-Processor firmware image information on AmebaPro.
*/
typedef struct sys_cp_fw_info_s {
    cp_fw_info_t wlan_fw;
} sys_cp_fw_info_t;

/**
 *  @brief Initial the HAL Misc. management adapter.
 *
 *  @param[in]  pmisc_adp  The Misc. entity.
 *  @returns    void
 */
__STATIC_INLINE
void hal_misc_init (hal_misc_adapter_t *pmisc_adp)
{
    hal_misc_stubs.hal_misc_init (pmisc_adp);
}

/**
 *  @brief Registers a handler function for the NMI interrupt which is
 *         not triggered by the watch dog timer timeout event.
 *         The NMI interrupt default is in secure region.
 *         To handle the NMI interrupt, we should make the AIRCR.BFHFNMINS = 1.
 *         We can do this by set SCB_AIRCR_BFHFNMINS_VAL = 1 at secure code build time.
 *
 *  @param[in]  handler  The interrupt handler.
 *  @param[in]  arg  The application data will be passed back to the application
 *                   with the callback function.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_misc_nmi_reg_irq (irq_handler_t handler, void *arg)
{
    hal_misc_stubs.hal_misc_nmi_reg_irq (handler, arg);
}

/** 
 *  @brief To enable wlan SDM Function.
 *
 *  @param[in] bypass_mode: 1: bypass mode, simple /4 path for 128k -> 32K; 
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_misc_sdm_32k_enable (u8 bypass_mode)
{
    hal_misc_stubs.hal_misc_sdm_32k_enable (bypass_mode);
}

/** 
 *  @brief To Read only, HW time loss calculation result.
 *
 *  @returns void
 */
__STATIC_INLINE
u32  hal_misc_read_sdm_32k_time_loss (void)
{
    return hal_misc_stubs.hal_misc_read_sdm_32k_time_loss ();
}

/** 
 *  @brief To set wlan SDM time loss mode Function.
 *
 *  @param[in] time_loss_set, 1:sw set src clk  higher than 128k clk; 0:sw set src clk  lower than 128k clk
 *  @param[in] ttime_loss_reg, set time loss regs
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_misc_set_sdm_32k_time_loss (u8 time_loss_set, u32 time_loss_reg)
{
    u32 i=0;
    hal_misc_stubs.hal_misc_set_sdm_32k_time_loss (time_loss_set, time_loss_reg);
    HAL_WRITE32 (0x40000000, 0x8C, 0xC0000000);
    for (i = 0; i < 1000000; i++) {
        if ((HAL_READ32 (0x40000000, 0x8C) & 0x8000000) != 0) {
            break;  // break the for loop
        } else {
            __NOP();
            __NOP();
            __NOP();
            __NOP();
        }
    }
    if (time_loss_set == 1) {
        HAL_WRITE32 (0x40000000, 0x8C, 0xC3000000);
    } else {
        HAL_WRITE32 (0x40000000, 0x8C, 0xC2000000);
    }
}

hal_status_t hal_misc_swd_pin_ctrl (BOOL en);
hal_status_t hal_misc_jtag_pin_ctrl (BOOL en);
void hal_misc_cpu_rst (void);

/** @} */ /* End of group hs_hal_misc */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_MISC_H_"

