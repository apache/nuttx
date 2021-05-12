/**************************************************************************//**
 * @file     hal_wdt.h
 * @brief    The Watchdog Timer HAL API implementation.
 * @version  V1.00
 * @date     2018-10-23
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

#ifndef _HAL_WDT_H_
#define _HAL_WDT_H_
#include "cmsis.h"
#include "hal_sys_ctrl.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup hs_hal_wdt Watchdog Timer Functions
 * @ingroup 8710c_hal
 * @{
 * @brief The WatchDog Timer HAL module of the AmebaZ2 platform.
 */

extern const hal_misc_func_stubs_t hal_misc_stubs;

/**
 *  @brief Enables the watch dog timer.
 *
 *  @returns    void
 */
void hal_misc_wdt_enable (void);

/**
 *  @brief Disables the watch dog timer.
 *
 *  @returns    void
 */
void hal_misc_wdt_disable (void);

/**
 *  @brief Refresh(reload) the watch dog timer counter.
 *         To prevents the watch dog timer timeout event occurred.
 *
 *  @returns    void
 */
void hal_misc_wdt_refresh (void);

/**
 *  @brief Force triggers a watch dog timer timeout to cause a system reset.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_misc_rst_by_wdt (void)
{
    rtl8710c_reset_reason_set(HAL_RESET_REASON_SOFTWARE);
    hal_misc_stubs.hal_misc_rst_by_wdt ();
}

/** 
 *  @brief Changes the watch dog timer timeout period.
 *
 *  @param[in]  time_us  The timeout period in micro-second.
 *
 *  @returns    void
 */
void hal_misc_wdt_set_timeout (uint32_t time_us);

/** 
 *  @brief Initials the watch dog timer and setup the timeout period.
 *         The system will be reset by the watch dog timeout event by default.
 *
 *  @param[in]  time_us  The timeout period in micro-second.
 *
 *  @returns    void
 */
void hal_misc_wdt_init (uint32_t time_us);

/**
 *  @brief Registers a handler for the watch dog timeout interrupt.
 *         The WDT timeout interrupt will trigger the NMI interrupt.
 *         However the NMI interrupt default is in secure region.
 *         To handle the NMI interrupt, we should make the AIRCR.BFHFNMINS = 1.
 *         We can do this by set SCB_AIRCR_BFHFNMINS_VAL = 1 at secure code build time.
 *
 *  @param[in]  handler  The interrupt handler.
 *  @param[in]  arg  The application data will be passed back to the application
 *                   with the callback function.
 *
 *  @returns    void
 */
void hal_misc_wdt_reg_irq (irq_handler_t handler, void *arg);

/** @} */ /* End of group hs_hal_wdt */

#ifdef  __cplusplus
}
#endif

#endif  // end of "#define _HAL_WDT_H_"

