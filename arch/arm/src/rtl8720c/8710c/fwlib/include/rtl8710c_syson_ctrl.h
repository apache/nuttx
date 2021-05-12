/**************************************************************************//**
 * @file     rtl8710c_syson_ctrl.h
 * @brief    Defines macros and data types for the System Power On dommin control.
 * @version  v1.00
 * @date     2017/11/17
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

/**
 * @addtogroup hs_hal_syson SysOn
 * @ingroup 8710c_hal
 * @{
 * @brief The HAL API for System Control.
 */

#ifndef RTL8710C_SYSON_CTRL_H
#define RTL8710C_SYSON_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  \brief  Define the Sys clock rate
*/
enum sys_clock_rate_e {
    SYS_CLK_100M        = 100000000,
    SYS_CLK_50M         = 50000000,
    SYS_CLK_25M         = 25000000
};
typedef uint32_t sys_clock_rate_t;

/**
  \brief  System clock source selection.
*/
enum  sys_clock_source_e {
    SYS_CLK_SRC_ANA  = 0,   ///< Clock source = ANA, 4M/250K
    SYS_CLK_SRC_PLL  = 1    ///< Clock source = PLL, 240M
};

/**
  \brief  System wake event selection.
*/
enum  sys_wake_event_e {
   SYS_WKEV_LS_GTIMER   = 0,    /*! Wake by LS G-Timer interrupt */
   SYS_WKEV_HS_GTIMER   = 1,    /*! Wake by HS G-Timer interrupt */
   SYS_WKEV_SWR_OCP     = 2,    /*! Wake by Switch Regulator Over Current Protector */
   SYS_WKEV_GPIO        = 4,    /*! Wake by GPIO Interrupt */
   SYS_WKEV_PWM         = 5,    /*! Wake by PWM Interrupt */
   SYS_WKEV_I2C_ADDR    = 6,    /*! Wake by I2C RX address match Interrupt */
   SYS_WKEV_WLAN        = 8,    /*! Wake by WLan Interrupt */
   SYS_WKEV_I2C         = 9,    /*! Wake by I2C Interrupt */
   SYS_WKEV_UART        = 11,   /*! Wake by UART Interrupt */
   SYS_WKEV_SDIO        = 14,   /*! Wake by SDIO device Interrupt */
   SYS_WKEV_BOR         = 30    /*! Wake by brown out reset */
};
typedef uint8_t sys_wake_event_t;

enum  ldoio_mode_e {
    LDOIO_BYPASS    = 0,    /*! bypass mode */
    LDOIO_LDO       = 1     /*! LDO mode */
};

/**
  \brief Union type to access syson_gpio_ctrl
*/
typedef union {
  __IOM uint32_t w;                           /*!< GPIO Control Register                                             */
  
  struct {
    __IOM uint32_t pinmux_sel_l : 3;    /*!< [2..0] 000:SPIC/SDIO
                                                    001: JTAG/Test-SPIC
                                                    010: UART
                                                    011: SPI/WL-LED/EXT-32K
                                                    100: I2C/SIC
                                                    101: PWM
                                                    110: Wake/RFE-Ctrl/BT_LOG
                                                    111: GPIO                                                  */
    __IM  uint32_t            : 3;
    __IOM uint32_t pull_ctrl_l : 2;     /*!< [7..6] 2b'00: high impedence; 2b'01: pull low; 2b'10: pull high;
                                                 2b'11: reserved                                                      */
    __IOM uint32_t shdn_n_l : 1;        /*!< [8..8] PAD enable, 1: enable PAD, 0: shutdown                             */
    __IOM uint32_t smt_en_l : 1;        /*!< [9..9] Enable GPIOA0 Schmitt trigger; 1: enable                          */
    __IOM uint32_t driving_l : 2;       /*!< [11..10] (E3,E2)=(10:9) 1.8V: 00: 2mA; 01: 4mA; 10: 6mA; 11:
                                                      8mA 3.3V: 00: 4mA; 01: 6mA; 10: 12mA; 11: 16mA                  */
    __IM  uint32_t            : 4;
    __IOM uint32_t pinmux_sel_h : 3;    /*!< [18..16] 000: Reserved 001: JTAG_TMS 010: UART1_OUT 011: BT_led
                                                 100: SIC_DA 101: PWM1 110: WL_UART_OUT 111: GPIO                     */
    __IM  uint32_t            : 3;
    __IOM uint32_t pull_ctrl_h : 2;     /*!< [23..22] 2b'00: high impedence; 2b'01: pull low; 2b'10: pull
                                                 high; 2b'11: reserved                                                */
    __IOM uint32_t shdn_n_h : 1;        /*!< [24..24] PAD enable, 1: enable PAD, 0: shutdown                           */
    __IOM uint32_t smt_en_h : 1;        /*!< [25..25] Enable GPIOA1 Schmitt trigger; 1: enable                        */
    __IOM uint32_t driving_h : 2;       /*!< [27..26] (E3,E2)=(10:9) 1.8V: 00: 2mA; 01: 4mA; 10: 6mA; 11:
                                                 8mA 3.3V: 00: 4mA; 01: 6mA; 10: 12mA; 11: 16mA                       */

  } b;                 /*!< bit fields for syson_gpio_ctrl */
} syson_gpio_ctrl_t, *psyson_gpio_ctrl_t;

/**
  \brief  SysOn IRQ(System Wake Event) handler type.
*/
typedef void (*syson_irq_handler_t)(uint32_t arg);

/**
  \brief  The data type for SysOn HAL entity manamgment.
*/
typedef struct hal_syson_adapter_s {
    uint32_t sys_wake_event;                /*!< the system wake event */
    syson_irq_handler_t ls_gtimer_handler;  /*!< the LS G-Timer event callback function */
    uint32_t ls_gtimer_arg;                 /*!< the argument of LS G-Timer event callback function */
    syson_irq_handler_t hs_gtimer_handler;  /*!< the HS G-Timer event callback function */
    uint32_t hs_gtimer_arg;                 /*!< the argument of HS G-Timer event callback function */
    syson_irq_handler_t swr_ocp_handler;    /*!< the Switch Regulator OCP event callback function */
    uint32_t swr_ocp_arg;                   /*!< the argument of Switch Regulator OCP event callback function */
    syson_irq_handler_t gpio_handler;       /*!< the GPIO event callback function */
    uint32_t gpio_arg;                      /*!< the argument of GPIO event callback function */
    syson_irq_handler_t pwm_handler;        /*!< the PWM event callback function */
    uint32_t pwm_arg;                       /*!< the argument of PWM event callback function */
    syson_irq_handler_t i2c_addr_handler;   /*!< the I2C RX address match event callback function */
    uint32_t i2c_addr_arg;                  /*!< the argument of I2C RX address match event callback function */
    syson_irq_handler_t wlan_handler;       /*!< the WLAN event callback function */
    uint32_t wlan_arg;                      /*!< the argument of PWM event callback function */
    syson_irq_handler_t i2c_handler;        /*!< the I2C interrupt event callback function */
    uint32_t i2c_arg;                       /*!< the argument of I2C interrupt event callback function */
    syson_irq_handler_t uart_handler;       /*!< the UART event callback function */
    uint32_t uart_arg;                      /*!< the argument of PWM event callback function */
    syson_irq_handler_t sdio_handler;       /*!< the SDIO device wake event callback function */
    uint32_t sdio_arg;                      /*!< the argument of SDIO device wake event callback function */
    syson_irq_handler_t bor_handler;        /*!< the brown out reset event callback function */
    uint32_t bor_arg;                       /*!< the argument of brown out reset wake event callback function */
} hal_syson_adapter_t, *phal_syson_adapter_t;

/// @cond DOXYGEN_ROM_HAL_API

#ifdef __cplusplus
}
#endif

#endif /* RTL8710C_SYSON_CTRL_H */

/** @} */ /* End of group hs_hal_syson */

