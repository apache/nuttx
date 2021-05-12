/**************************************************************************//**
 * @file     hal_lpi.h
 * @brief    This file implements the entry functions of the HAL Low Priority
 *           Interrupt API ROM functions.
 * 
 * @version  V1.00
 * @date     2017-07-07
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

#ifndef _HAL_LPI_H_
#define _HAL_LPI_H_
#include "cmsis.h"
  
#ifdef  __cplusplus
 extern "C"
 {
#endif
 
extern const hal_lpi_func_stubs_t hal_lpi_stubs;     // symbol from linker script
 
#if !defined(CONFIG_BUILD_SECURE)

 /** 
  *  @brief To initial the low priority interrupt handler.
  *
  *  @param plpi_hdl The low priority interrupt handler.
  *
  *  @returns void
  */
 __STATIC_INLINE  
 void hal_lpi_init (hal_lpi_int_t *plpi_hdl)
 {
     hal_lpi_stubs.hal_lpi_init (plpi_hdl);
 }

 /** 
  *  @brief To register a low priority interrupt handler.
  *
  *  @param int_id The interrupt ID. This is the IMR enable bit of the interrupt.
  *  @param int_trig_type The interrupt trigger type(edge or level).
  *  @param handler The interrupt handler
  *  @param arg The argument of the interrupt handler.
  *
  *  @returns void
  */
 __STATIC_INLINE
 void hal_lpi_handler_reg (lowpri_int_id_t int_id, low_pri_int_mode_t int_trig_type,
                                 irq_handler_t handler, void *arg)
 {
     hal_lpi_stubs.hal_lpi_handler_reg (int_id, int_trig_type, handler, arg);
 }
 
 /** 
  *  @brief To enable a low priority interrupt handler.
  *
  *  @param int_id The interrupt ID. This is the IMR enable bit of the interrupt.
  *
  *  @returns void
  */
 __STATIC_INLINE
 void hal_lpi_en (lowpri_int_id_t int_id)
 {
     hal_lpi_stubs.hal_lpi_en (int_id);
 }
 
 /** 
  *  @brief To disable a low priority interrupt handler.
  *
  *  @param int_id The interrupt ID. This is the IMR enable bit of the interrupt.
  *
  *  @returns void
  */
 __STATIC_INLINE
 void hal_lpi_dis (lowpri_int_id_t int_id)
 {
     hal_lpi_stubs.hal_lpi_dis (int_id);
 }
 
 /** 
  *  @brief To register a common interrupt handler for all 
  *         Low Priority Interrupt.
  *
  *  @param handler The IRQ handle function.
  *  @param priority The interrupt priority.
  *
  *  @returns void
  */ 
 __STATIC_INLINE
 void hal_lpi_reg_irq (uint32_t handler, uint8_t priority)
 {
     hal_lpi_stubs.hal_lpi_reg_irq (handler, priority);
 }

#endif  // #if !defined(CONFIG_BUILD_SECURE)

#ifdef  __cplusplus
 }
#endif
 
 
#endif  // end of "#define _HAL_LPI_H_"
 
