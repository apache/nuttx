/**************************************************************************//**
 * @file     rtl8710c_peripheral_id.h
 * @brief    Define a identification ID for each peripheral device.
 * @version  V1.00
 * @date     2016-07-20
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
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


#ifndef _RTL8710C_PHERI_ID_H_
#define _RTL8710C_PHERI_ID_H_

#include "basic_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  \brief  Defines IO Pin peripheral function category.  
             0: SPIC-Flash/SDIO
             1: JTAG/Test-SPIC
             2: UART
             3: SPI/WL-LED/EXT-32K
             4: I2C/SIC
             5: PWM
             6: Wake/RFE-Ctrl
             7: GPIO
 */
enum  peripheral_func_cat_e {
    FUNC_FLASH      = 0x00,
    FUNC_SDIO       = 0x00,
    FUNC_JTAG       = 0x01,
    FUNC_TST_FLASH  = 0x01,
    FUNC_UART       = 0x02,
    FUNC_SPI        = 0x03,
    FUNC_WLED       = 0x03,
    FUNC_E32K       = 0x03,
    FUNC_I2C        = 0x04,
    FUNC_SIC        = 0x04,
    FUNC_PWM        = 0x05,
    FUNC_WAKE       = 0x06,
    FUNC_RFECTRL    = 0x06,
    FUNC_BT_LOG     = 0x06,
    FUNC_GPIO       = 0x07,
    FUNC_LPC        = 0x07
};

enum  peripheral_id_e {
  /* pinmux_sel = 0x00 */
  PID_FLASH         = (FUNC_FLASH << 4),
  PID_SDIO          = (FUNC_SDIO << 4)| (1 << 2),

  /* pinmux_sel = 0x01 */
  PID_JTAG          = (FUNC_JTAG << 4),

  /* pinmux_sel = 0x02 */
  PID_UART0         = (FUNC_UART << 4),
  PID_UART1         = (PID_UART0+1),
  PID_UART2         = (PID_UART0+2),
  PID_UART3         = (PID_UART0+3),

  /* pinmux_sel = 0x03 */
  PID_SPI0          = (FUNC_SPI << 4),
  PID_WLED0         = (FUNC_WLED << 4) | (1 << 2),
  PID_WLED1         = (PID_WLED0+1),

  /* pinmux_sel = 0x04 */
  PID_I2C0          = (FUNC_I2C << 4),
  PID_SIC           = (FUNC_SIC << 4) | (1 << 2),
  
  /* pinmux_sel = 0x05 */
  PID_PWM0          = (FUNC_PWM << 4),
  PID_PWM1          = (PID_PWM0+1),
  PID_PWM2          = (PID_PWM0+2),
  PID_PWM3          = (PID_PWM0+3),
  PID_PWM4          = (PID_PWM0+4),
  PID_PWM5          = (PID_PWM0+5),
  PID_PWM6          = (PID_PWM0+6),
  PID_PWM7          = (PID_PWM0+7),

  /* pinmux_sel = 0x06 */
  PID_BT_LOG        = (FUNC_BT_LOG << 4),
  PID_WAKE          = (FUNC_WAKE << 4) + 2,
  PID_REFCTRL       = (FUNC_RFECTRL << 4) + 4,
  
  /* pinmux_sel = 0x07 */
  PID_GPIO          = (FUNC_GPIO << 4),
  PID_LPC           = (FUNC_LPC << 4) | (1 << 2),


  PID_ERR           = 0xFF
};

typedef struct {
    uint8_t pin_name;
    uint8_t peripheral_id;
} hal_pin_map, *phal_pin_map;

#ifdef __cplusplus
}
#endif


#endif //_RTL8710C_PHERI_ID_H_

