/**************************************************************************//**
 * @file      rtl8710c_pwm_type.h
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

#ifndef _RTL8710C_PWM_TYPE_H_
#define _RTL8710C_PWM_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_PWM_REG_TYPE

/**
 * @addtogroup hs_hal_pwm_reg PWM Registers.
 * @ingroup hs_hal_pwm
 * @{
 */

/**
  \brief Union type to access pwm_ctrl (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) PWM Control Register                                       */
  
  struct {
    __IOM uint32_t duty       : 12;           /*!< [11..0] The on-duty duration of PWM pulse. The time unit is
                                                   configured by the GTIMER which is specified by clk_sel
                                                   field. It can be written at any time, but HW will apply
                                                   the changing at PWM enable time or at the end of PWM period.              */
    __IOM uint32_t clk_sel    : 4;            /*!< [15..12] The tick source selection, G-timer 0 ~ 7 or SClk: 0:
                                                   GTIMER 0 1: GTIMER 1 ... 7: GTIMER 7 8: sclk                              */
    __IM  uint32_t cur_duty   : 12;           /*!< [27..16] The current duty size which is adjusted by the duty
                                                   auto-adjustment HW. The SW can read this field to know
                                                   the current duty size. However, the latest duty size may
                                                   not be sync to this field on time or the value may transition
                                                   while the SW is reading this field. So use in-direct read
                                                   method to read the duty size is more save and is suggested.               */
    __IOM uint32_t period_ie  : 1;            /*!< [28..28] This bit control the PWM period end interrupt enable
                                                   (1) or disable (0). When the PWM period end interrupt is
                                                   enabled, the PWM HW will assert the interrupt on every
                                                   PWM period end time. This interrupt status can be read
                                                   from pwm_int_status.                                                      */
    __IM  uint32_t run_sts    : 1;            /*!< [29..29] The PWM output running status: 0: PWM output is paused
                                                   1: the PWM output is running.                                             */
    __IOM uint32_t pause      : 1;            /*!< [30..30] The paus control: 0: Normal running 1: To pause the
                                                   PWM out at current PWM period end. Since the PWM output
                                                   is not stopped immediately, the software can poll the BIT_PERI_PWMx_RUNST
                                                   to know the PWM out is still running or is paused.                        */
    __IOM uint32_t ctrl_set   : 1;            /*!< [31..31] SW can change setting only at ctrl_set = 0, and set
                                                   ctrl_set = 1 after changing PWM Ctrl. HW will clear ctrl_set
                                                   bit after changing PWM Ctrl. PS. If this PWM is disabled
                                                   (PWMx_EN=0), the HW will not clear this bit. PS. if this
                                                   bit is asserted and the duty auto adjustment is enabled,
                                                   when the PWM is enabled the HW will updates its duty at
                                                   the 1st PWM period and then start the duty adjustment at
                                                   2nd PWM period.                                                           */
  } b;                                        /*!< bit fields for pwm_ctrl */
} pwm_ctrl_t, *ppwm_ctrl_t;

/**
  \brief Union type to access pwm_timing_ctrl (@ 0x00000004).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000004) PWM Timing Control Register                                */
  
  struct {
    __IOM uint32_t period     : 12;           /*!< [11..0] The period of PWM pulse. The time unit is configured
                                                   by the GTIMER which is specified by the clk_sel field.
                                                   It can be written at any time, but the HW will apply the
                                                   changing at PWM enable time or at the end of PWM period.                  */
    __IM  uint32_t            : 4;
    __IOM uint32_t duty_start : 12;           /*!< [27..16] The start of ON duration of PWM pulse in the period.
                                                   The time unit is the same as PWM period. It can be written
                                                   at any time, but HW change setting only at PWM enable and
                                                   the end of period.                                                        */
  } b;                                        /*!< bit fields for pwm_timing_ctrl */
} pwm_timing_ctrl_t, *ppwm_timing_ctrl_t;

/**
  \brief Union type to access pwm_auto_adj_ctrl (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) PWM Duty Auto Adjustment Control Register                  */
  
  struct {
    __IOM uint32_t duty_dec_step : 10;        /*!< [9..0] The Duty Decreasing Step size of the Duty Auto Adjustment.         */
    __IM  uint32_t            : 2;
    __IOM uint32_t duty_inc_step : 10;        /*!< [21..12] The Duty Increasing Step size of the Duty Auto Adjustment        */
    __IM  uint32_t            : 5;
    __IOM uint32_t duty_dn_lim_ie : 1;        /*!< [27..27] This bit is used to enable the Interrupt of the Duty
                                                   Auto Adjustment. 0: Disable 1: Enable If the Interrupt
                                                   is enabled, issue an interrupt when the Adjusted Duty reach
                                                   the Down Limit.                                                           */
    __IOM uint32_t duty_up_lim_ie : 1;        /*!< [28..28] This bit is used to enable the Interrupt of the Duty
                                                   Auto Adjustment. 0: Disable 1: Enable If the Interrupt
                                                   is enabled, issue an interrupt when the Adjusted Duty reach
                                                   the Up Limit.                                                             */
    __IOM uint32_t adj_loop_en : 1;           /*!< [29..29] This bit is used to enable the Duty Auto Adjustment
                                                   Loop mode. 0: Disable 1: Enable If the Loop Mode is enabled,
                                                   reverse the Duty Auto Adjustment direction when the adjusted
                                                   Duty reach the Up Limit or the Down Limit.                                */
    __IOM uint32_t adj_dir    : 1;            /*!< [30..30] This bit is used to set the Duty Ato Adjustment direction.
                                                   0: Decrease Duty 1: Increase Duty                                         */
    __IOM uint32_t adj_en     : 1;            /*!< [31..31] This bit is used to enable the Duty Ato Adjustment.
                                                   0: Disable 1: Enable PS. If this bit is 1, the duty of
                                                   the auto adjustment will override the duty setting in the
                                                   pwm_ctrl register.                                                        */
  } b;                                        /*!< bit fields for pwm_auto_adj_ctrl */
} pwm_auto_adj_ctrl_t, *ppwm_auto_adj_ctrl_t;

/**
  \brief Union type to access pwm_auto_adj_limit (@ 0x0000000C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000000C) PWM Duty Auto Adjustment Limit Register                    */
  
  struct {
    __IOM uint32_t duty_adj_dn_lim : 12;      /*!< [11..0] The down limit (in ticks) of the duty period for the
                                                   duty ratio auto adjustment                                                */
    __IM  uint32_t            : 4;
    __IOM uint32_t duty_adj_up_lim : 12;      /*!< [27..16] The up limit of the duty period for the duty ratio
                                                   auto adjustment                                                           */
  } b;                                        /*!< bit fields for pwm_auto_adj_limit */
} pwm_auto_adj_limit_t, *ppwm_auto_adj_limit_t;

/**
  \brief Union type to access pwm_auto_adj_cycle (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) PWM Duty Auto Adjustment Cycle Count Register              */
  
  struct {
    __IOM uint32_t adj_cycles : 12;           /*!< [11..0] The Cycle Count of the Duty Auto Adjustment. The Duty
                                                   size will be increased/decreased with a step size every
                                                   Cycle Count of PWM period                                                 */
  } b;                                        /*!< bit fields for pwm_auto_adj_cycle */
} pwm_auto_adj_cycle_t, *ppwm_auto_adj_cycle_t;

/** @} */ /* End of group ls_hal_pwm_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_PWM_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_PWM_TYPE_H_

