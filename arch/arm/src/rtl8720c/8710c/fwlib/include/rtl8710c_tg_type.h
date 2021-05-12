/**************************************************************************//**
 * @file      rtl8710c_tg_type.h
 * @brief
 * @version   V1.00
 * @date      2018-1-4 15:41:31
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

#ifndef _RTL8710C_TG_TYPE_H_
#define _RTL8710C_TG_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_TG_REG_TYPE

/**
 * @addtogroup hs_hal_tg_reg TG Registers.
 * @ingroup hs_hal_timer
 * @{
 */

/**
  \brief Union type to access tg_ists (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) Timer Group interrupt status register                      */
  
  struct {
    __IM  uint32_t ists       : 8;            /*!< [7..0] Each bit indicates the interrupt pending status of corresponding
                                                   timer: 0: the specifiied timer has no interrupt pending
                                                   1: the specifiied timer has pending interrupt                             */
  } b;                                        /*!< bit fields for tg_ists */
} tg_ists_t, *ptg_ists_t;

/**
  \brief Union type to access tg_raw_ists (@ 0x00000004).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000004) Timer Group raw interrupt status register                  */
  
  struct {
    __IM  uint32_t rists      : 8;            /*!< [7..0] Each bit indicates the interrupt pending status of corresponding
                                                   timer: 0: the specifiied timer has no interrupt pending
                                                   1: the specifiied timer has pending interrupt (pre-masing)                */
  } b;                                        /*!< bit fields for tg_raw_ists */
} tg_raw_ists_t, *ptg_raw_ists_t;

/**
  \brief Union type to access tg_tsel (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) Timer Group indirect read control register                 */
  
  struct {
    __IOM uint32_t tsel       : 3;            /*!< [2..0] The timer index in a timer group, to select the timer
                                                   for indirect read.                                                        */
    __IM  uint32_t            : 1;
    __IOM uint32_t sync_mode  : 1;            /*!< [4..4] Sync mode between Timer clock and APB clock 0: Freq.
                                                   of APB clock >> Freq. of Timer clock (about 10 times) 1:
                                                   others                                                                    */
    __IM  uint32_t            : 2;
    __IOM uint32_t poll       : 1;            /*!< [7..7] Set this bit to enable indirect read current value of
                                                   timer selected by tsel. This bit is cleared by HW while
                                                   finishing read and indicate REG_TIMER_TC is ready                         */
  } b;                                        /*!< bit fields for tg_tsel */
} tg_tsel_t, *ptg_tsel_t;

/**
  \brief Union type to access tg_tc (@ 0x0000000C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000000C) Timer Group indirect read timer counter register           */
  
  struct {
    __IM  uint32_t timer_tc   : 32;           /*!< [31..0] Current counter value of the specifiied timer by register
                                                   tg_tsel.                                                                  */
  } b;                                        /*!< bit fields for tg_tc */
} tg_tc_t, *ptg_tc_t;

/** @} */ /* End of group ls_hal_tg_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_TG_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_TG_TYPE_H_

