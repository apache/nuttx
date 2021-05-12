/**************************************************************************//**
 * @file     hal_power_mode.h
 * @brief    The HAL API implementation for the POWER MODE device.
 * @version  V1.00
 * @date     2018-10-02
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

#ifndef _HAL_POWER_MODE_H_
#define _HAL_POWER_MODE_H_
#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup hal_power_mode POWER MODE
 * @ingroup 8710_hal
 * @{
 * @brief The POWER MODE HAL module of the platform.
 */

//SLP
#define     SLP_STIMER    BIT0
#define     SLP_GTIMER    BIT1
#define     SLP_GPIO      BIT2
#define     SLP_PWM       BIT3
#define     SLP_UART      BIT4
#define     SLP_HSTIMER   BIT5
#define     SLP_WLAN      BIT6
#define     SLP_SDIO      BIT11
//#define     SLP_WL        BIT3

//DSTBY
#define     DSTBY_STIMER  BIT0
#define     DSTBY_GTIMER  BIT1
#define     DSTBY_GPIO    BIT2
#define     DSTBY_PWM     BIT3
#define     DSTBY_UART    BIT4
#define     DSTBY_HSTIMER BIT5
#define     DSTBY_WLAN    BIT6
#define     DSTBY_SDIO    BIT11
//#define     DSTBY_NFC     BIT3

//DS wake event
#define DS_STIMER   BIT0
#define DS_GPIO     BIT1

enum clk_idx{
    CLK_250K = 0,
    CLK_4M = 1,
};

void hal_sys_reg_irq(VOID);

/**
  * @brief The stubs functions table to exports POWER MODE HAL functions in ROM.
  */

//extern const hal_chg_func_stubs_t hal_chg_stubs;

/** 
 *  @brief The function for ls deep sleep mode.
 *         
 *  @param[in]  Option, To slect AON Timerand GPIO.
 *                - bit[1]: the GPIO Wake up event.
 *                - bit[0]: the AON Timer Wake up event.
 *  @param[in]  SDuration, wake up after SDuration value. Uint: us
 *  @param[in]  Clock, 1: 4MHz, 0: 250kHz.
 *
 *  @returns void
 */
void hal_DeepSleep (u8  Option, u32 SDuration, u8 Clock);

/** 
 *  @brief The function for hs sleep mode.
 *         
 *  @param[in]  Option, To slect HS Timer, GPIO and PWM...etc
 *                - bit[4]: the UART Wake up event.
 *                - bit[3]: the PWM Wake up event.
 *                - bit[2]: the GPIO Wake up event.
 *                - bit[1]: the GTimer Wake up event.
 *  @param[in]  SDuration, wake up after SDuration value. Uint: us
 *  @param[in]  Clock, 1: 4MHz, 0: 250kHz.
 *  @param[in]  GpioOption, Select GPIO pin as a wake up trigger.
 *
 *  @returns void
 */
void hal_SleepCG (u16 Option, u32 SDuration, u8 Clock, u8 GpioOption);

/** 
 *  @brief The function for hs sleep mode.
 *         
 *  @param[in]  Option, To slect HS Timer, GPIO and PWM...etc
 *                - bit[4]: the UART Wake up event.
 *                - bit[3]: the PWM Wake up event.
 *                - bit[2]: the GPIO Wake up event.
 *                - bit[1]: the GTimer Wake up event.
 *  @param[in]  SDuration, wake up after SDuration value. Uint: us
 *  @param[in]  Clock, 1: 4MHz, 0: 250kHz.
 *  @param[in]  GpioOption, Select GPIO pin as a wake up trigger.
 *
 *  @returns void
 */
void hal_SleepPG (u16 Option, u32 SDuration, u8 Clock, u8 GpioOption);

#ifdef  __cplusplus
}
#endif

/** @} */ /* End of group ls_hal_power_mode */

#endif  // end of "#define _HAL_POWER MODE_H_"
