/**************************************************************************//**
 * @file      rtl8710c_tm_type.h
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

#ifndef _RTL8710C_TM_TYPE_H_
#define _RTL8710C_TM_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_TM_REG_TYPE

/**
 * @addtogroup hs_hal_tm_reg TM Registers.
 * @ingroup hs_hal_timer
 * @{
 */

/**
  \brief Union type to access tm_lc (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) Timer load count register                                  */
  
  struct {
    __IOM uint32_t lc         : 32;           /*!< [31..0] The counter value to be loaded into the timer counter
                                                   when timer is enabled or reset. Min = 0, when prescaler
                                                   register > 0; otherwise Min = 1.                                          */
  } b;                                        /*!< bit fields for tm_lc */
} tm_lc_t, *ptm_lc_t;

/**
  \brief Union type to access tm_tc (@ 0x00000004).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000004) Timer current counter register                             */
  
  struct {
    __IOM uint32_t tc         : 32;           /*!< [31..0] Current counter value.                                            */
  } b;                                        /*!< bit fields for tm_tc */
} tm_tc_t, *ptm_tc_t;

/**
  \brief Union type to access tm_pc (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) Timer prescaler counter register                           */
  
  struct {
    __IOM uint32_t pc         : 10;           /*!< [9..0] Pre-scaler counter will be increased every clock after
                                                   timer is enabled. When the prescaler counter is equal to
                                                   PR, the next clock increments (decrements) the TC and clears
                                                   the PC.                                                                   */
  } b;                                        /*!< bit fields for tm_pc */
} tm_pc_t, *ptm_pc_t;

/**
  \brief Union type to access tm_pr (@ 0x0000000C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000000C) Timer wescaler register                                   */
  
  struct {
    __IOM uint32_t pr         : 10;           /*!< [9..0] When the PC is equal to this value, the next clock increments
                                                   (decrements) the TC and clears the PC.                                    */
  } b;                                        /*!< bit fields for tm_pr */
} tm_pr_t, *ptm_pr_t;

/**
  \brief Union type to access tm_ctrl (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) Timer control register                                     */
  
  struct {
    __IOM uint32_t en         : 1;            /*!< [0..0] Timer enable control: 0: Disable 1: Enable                         */
    __IOM uint32_t mod        : 1;            /*!< [1..1] Timer operation mode: 0: timer mode (periodical mode);
                                                   it reloads LC to TC and repeat the process according to
                                                   the setting when the timeout event occurred. 1: counter
                                                   mode (one-shot mode); only one time process.                              */
    __IOM uint32_t imr        : 1;            /*!< [2..2] Timer Interrupt mask: 0: disable interrupt 1: enable
                                                   interrupt                                                                 */
    __IOM uint32_t cnt_mod    : 1;            /*!< [3..3] Timer counting mode: 0: Up-counter 1: Down-counter                 */
  } b;                                        /*!< bit fields for tm_ctrl */
} tm_ctrl_t, *ptm_ctrl_t;

/**
  \brief Union type to access tm_isr (@ 0x00000014).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000014) Timer0 interrupt status register                           */
  
  struct {
    __IOM uint32_t timeout    : 1;            /*!< [0..0] Timeout interrupt                                                  */
    __IOM uint32_t match_ev0  : 1;            /*!< [1..1] Counter value match event0 interrupt.                              */
    __IOM uint32_t match_ev1  : 1;            /*!< [2..2] Counter value match event1 interrupt.                              */
    __IOM uint32_t match_ev2  : 1;            /*!< [3..3] Counter value match event2 interrupt.                              */
    __IOM uint32_t match_ev3  : 1;            /*!< [4..4] Counter value match event3 interrupt.                              */
  } b;                                        /*!< bit fields for tm_isr */
} tm_isr_t, *ptm_isr_t;

/**
  \brief Union type to access tm_mectrl (@ 0x0000001C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000001C) Timer match event control register                         */
  
  struct {
    __IOM uint32_t me0_en     : 1;            /*!< [0..0] Counter value match event0 enable control: 0: Disable
                                                   1: Enable                                                                 */
    __IOM uint32_t me1_en     : 1;            /*!< [1..1] Counter value match event1 enable control: 0: Disable
                                                   1: Enable                                                                 */
    __IOM uint32_t me2_en     : 1;            /*!< [2..2] Counter value match event2 enable control: 0: Disable
                                                   1: Enable                                                                 */
    __IOM uint32_t me3_en     : 1;            /*!< [3..3] Counter value match event3 enable control: 0: Disable
                                                   1: Enable                                                                 */
  } b;                                        /*!< bit fields for tm_mectrl */
} tm_mectrl_t, *ptm_mectrl_t;

/**
  \brief Union type to access tm_me0 (@ 0x00000020).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000020) Timer match event0 counter register                        */
  
  struct {
    __IOM uint32_t me0        : 32;           /*!< [31..0] Timer counter value for metch event0.                             */
  } b;                                        /*!< bit fields for tm_me0 */
} tm_me0_t, *ptm_me0_t;

/**
  \brief Union type to access tm_me1 (@ 0x00000024).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000024) Timer match event1 counter register                        */
  
  struct {
    __IOM uint32_t me1        : 32;           /*!< [31..0] Timer counter value for metch event1.                             */
  } b;                                        /*!< bit fields for tm_me1 */
} tm_me1_t, *ptm_me1_t;

/**
  \brief Union type to access tm_me2 (@ 0x00000028).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000028) Timer match event2 counter register                        */
  
  struct {
    __IOM uint32_t me2        : 32;           /*!< [31..0] Timer counter value for metch event2.                             */
  } b;                                        /*!< bit fields for tm_me2 */
} tm_me2_t, *ptm_me2_t;

/**
  \brief Union type to access tm_me3 (@ 0x0000002C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000002C) Timer match event3 counter register                        */
  
  struct {
    __IOM uint32_t me3        : 32;           /*!< [31..0] Timer counter value for metch event3.                             */
  } b;                                        /*!< bit fields for tm_me3 */
} tm_me3_t, *ptm_me3_t;

/** @} */ /* End of group ls_hal_tm_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_TM_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_TM_TYPE_H_

