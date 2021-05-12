/**************************************************************************//**
 * @file      rtl8710c_pwm_comm_type.h
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

#ifndef _RTL8710C_PWM_COMM_TYPE_H_
#define _RTL8710C_PWM_COMM_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_PWM_COMM_REG_TYPE

/**
 * @addtogroup hs_hal_pwm_comm_reg PWM_COMM Registers.
 * @ingroup hs_hal_pwm
 * @{
 */

/**
  \brief Union type to access pwm_comm_enable_status (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) PWM enable status Register                                 */
  
  struct {
    __IM  uint32_t pwm_en_sts : 8;            /*!< [7..0] Each bit indicates the enabling status of corresponding
                                                   PWM: 0: the specific PWM is Disabled 1: the specific PWM
                                                   is Enabled                                                                */
  } b;                                        /*!< bit fields for pwm_comm_enable_status */
} pwm_comm_enable_status_t, *ppwm_comm_enable_status_t;

/**
  \brief Union type to access pwm_comm_enable_ctrl (@ 0x00000004).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000004) PWM enable Register                                        */
  
  struct {
    __IOM uint32_t pwm_en     : 8;            /*!< [7..0] Each bit is used to enablel the corresponding PWM function:
                                                   When wrote 0: No operation 1: the specific PWM is enabled
                                                   and the specific bit of REG_PWM_EN_STS is ONE.                            */
  } b;                                        /*!< bit fields for pwm_comm_enable_ctrl */
} pwm_comm_enable_ctrl_t, *ppwm_comm_enable_ctrl_t;

/**
  \brief Union type to access pwm_comm_disable_ctrl (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) PWM disable Register                                       */
  
  struct {
    __IOM uint32_t pwm_dis    : 8;            /*!< [7..0] Each bit is used to enablel the corresponding PWM function:
                                                   When wrote 0: No operation 1: the conrtolled PWM is disabled
                                                   and the specific bit of REG_PWM_EN_STS is ZERO.                           */
  } b;                                        /*!< bit fields for pwm_comm_disable_ctrl */
} pwm_comm_disable_ctrl_t, *ppwm_comm_disable_ctrl_t;

/**
  \brief Union type to access pwm_comm_int_status (@ 0x0000000C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000000C) Timer Group indirect read control register                 */
  
  struct {
    __IOM uint32_t duty_adj_dn_lim : 8;       /*!< [7..0] To indicate the interrupt pending status for PWM 0 ~
                                                   7 duty auto-adjustment reaches the down-limit. Write 1
                                                   clear. Bit 0 for PWM0, bit 1 for PWM1, ...bit7 for PWM7.                  */
    __IM  uint32_t            : 8;
    __IOM uint32_t duty_adj_up_lim : 8;       /*!< [23..16] To indicate the interrupt pending status for PWM 0
                                                   ~ 7 duty auto-adjustment reaches the up-limit. Write 1
                                                   clear. Bit 16 for PWM0, bit 17 for PWM1, ...bit23 for PWM7.               */
    __IOM uint32_t period_end : 8;            /*!< [31..24] To indicate the interrupt pending status for PWM 0
                                                   ~7 period end. Write 1 clear. Bit 24 for PWM0, bit 25 for
                                                   PWM1, ...bit31 for PWM7.                                                  */
  } b;                                        /*!< bit fields for pwm_comm_int_status */
} pwm_comm_int_status_t, *ppwm_comm_int_status_t;

/**
  \brief Union type to access pwm_comm_indread_idx (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) PWM Index of Indirect Read Register                        */
  
  struct {
    __IOM uint32_t pwm_sel    : 3;            /*!< [2..0] This field is used to assign the PWM index (0 ~ 7) for
                                                   the auto adjusted duty size indirect reading.                             */
    __IM  uint32_t            : 1;
    __IOM uint32_t sync_mode  : 1;            /*!< [4..4] Register sync mode selection for PWM clock and APB clock
                                                   0: Freq. of APB clock >> Freq. of PWM clock (about 10 times)
                                                   1: Freq. of PWM clock >= Freq. of APB clock                               */
    __IM  uint32_t            : 2;
    __IOM uint32_t pool       : 1;            /*!< [7..7] Set this bit to enable indirect read current value of
                                                   timer selected by BIT_PWM_INDREAD_IDX. This bit is cleared
                                                   by HW when reading is finished and indicates REG_PERI_PWM_INDREAD_DUTY
                                                   is ready.                                                                 */
  } b;                                        /*!< bit fields for pwm_comm_indread_idx */
} pwm_comm_indread_idx_t, *ppwm_comm_indread_idx_t;

/**
  \brief Union type to access pwm_comm_indread_duty (@ 0x00000014).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000014) PWM Auto Adjusted Duty Indirect Read Register              */
  
  struct {
    __IOM uint32_t pwm_duty   : 12;           /*!< [11..0] This field is used to read the auto adjusted duty size
                                                   of the PWM which is assigned by the pwm_sel. The SW should
                                                   assign the PWM to read by write the pwm_sel and then read
                                                   this field to get the current duty size of the specified
                                                   PWM.                                                                      */
  } b;                                        /*!< bit fields for pwm_comm_indread_duty */
} pwm_comm_indread_duty_t, *ppwm_comm_indread_duty_t;

/** @} */ /* End of group ls_hal_pwm_comm_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_PWM_COMM_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_PWM_COMM_TYPE_H_

