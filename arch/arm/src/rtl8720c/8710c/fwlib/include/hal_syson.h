/**************************************************************************//**
 * @file     hal_syson.h
 * @brief    The HAL API implementation for the system wake up functions.
 * @version  V1.00
 * @date     2016-07-15
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

#ifndef _HAL_SYSON_H_
#define _HAL_SYSON_H_
#include "cmsis.h"

/**
 * @addtogroup hs_hal_syson
 * @{
 */

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 *  @brief Gets current CPU clock freq.
 *
 *  @returns The CPU clock frequency.
 */
uint32_t hal_syson_query_sys_clk (void);

/**
 *  @brief Set the System(CPU) clock rate.
 *
 *  @param[in] sys_clk The target system clock rate.
 *
 *  @return  void.
 */
void hal_syson_set_sys_clk (sys_clock_rate_t sys_clk);

/**
 *  @brief Registers an interrupt handler for the SysOn wake up event.
 *
 *  @param[in] event The Wake up event ID.
 *  @param[in] handler The Wake up event handler (callback function).
 *  @param[in] arg The argument of the wake up event handler (callback function).
 *
 *  @return  void
 */
void hal_syson_reg_wake_event_handler (sys_wake_event_t event, syson_irq_handler_t handler, uint32_t arg);

/**
 *  @brief Un-registers an interrupt handler for the SysOn wake up event.
 *
 *  @param[in] event The Wake up event ID.
 *
 *  @return  void
 */
void hal_syson_unreg_wake_event_handler (sys_wake_event_t event);

/**
 *  @brief Enables a SysOn wake up event.
 *
 *  @param[in] psyson_adp The SysOn adapter.
 *  @param[in] event The Wake up event ID.
 *
 *  @return  void
 */
void hal_syson_enable_wake_event (sys_wake_event_t event);

/**
 *  @brief Disables a SysOn wake up event.
 *
 *  @param[in] psyson_adp The SysOn adapter.
 *  @param[in] event The Wake up event ID.
 *
 *  @returns void
 */
void hal_syson_disable_wake_event (sys_wake_event_t event);

/**
 *  @brief Registers an interrupt handler for the SysOn function.
 *
 *  @param[in] psyson_adp The SysOn adapter.
 *  @param[in] handler The IRQ handle function.
 *
 *  @returns void
 */
void hal_syson_reg_irq (int_vector_t handler);

/**
 *  @brief Initials the System On domain management entity.
 *
 *  @param[in] psyson_adp The SysOn adapter.
 *
 *  @return  void.
 */
void hal_syson_init (hal_syson_adapter_t *psyson_adp);

#if !defined(CONFIG_BUILD_NONSECURE)

/** 
 *  @brief To Set the RAM code start function table address for fast booting.
 *
 *  @param[in] pstart_tbl The RAM code start function table address.
 *                        Disable the fast booting by give this value as NULL.
 *  @param[in] func_idx To assign the RAM code start function. Valid value is 0 ~ 3.
 *
 *  @return     HAL_OK: set the RAM code start function table addrsss OK.
 *  @return     HAL_ERR_PARA: the given start function table address is invalid.
 */
hal_status_t hal_sys_set_fast_boot (uint32_t pstart_tbl, uint32_t func_idx);

#endif  // end of "if !defined(CONFIG_BUILD_NONSECURE)"

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_SYSON_H_"

/** @} */ /* End of group hs_hal_syson */

