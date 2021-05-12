/**************************************************************************//**
 * @file     rtl8710c_pinmux.h
 * @brief    Define the IC pin name and IO port name
 * @version  V1.00
 * @date     2016-07-20
 *
 * @note
 *
 ******************************************************************************
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

#ifndef RTL8710C_PIN_MUX_H
#define RTL8710C_PIN_MUX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtl8710c_pin_name.h"

/**
  * @brief The data type of JTAG/SWD pins list.
  */
typedef struct hal_jtag_pin_s {
    union {
        uint8_t pin_tclk;
        uint8_t pin_swclk;
    };
    union {
        uint8_t pin_tms;
        uint8_t pin_swdio;
    };
    uint8_t pin_tdo;
    uint8_t pin_tdi;
    uint8_t pin_trst;
} hal_jtag_pin_t, *phal_jtag_pin_t;

/**
  * @brief The data type for IO pin configuration.
  */
typedef struct hal_pin_cfg_s {
    uint16_t pinmux_sel : 4;    /*!< [3..0] 000:SPIC/SDIO
                                                  001: JTAG/Test-SPIC
                                                  010: UART
                                                  011: SPI/WL-LED/EXT-32K
                                                  100: I2C/SIC
                                                  101: PWM
                                                  110: Wake/RFE-Ctrl
                                                  111: GPIO                                                  */
    uint16_t            : 2;
    uint16_t pull_ctrl : 2;     /*!< [7..6] 2b'00: high impedence; 2b'01: pull low; 2b'10: pull high;
                                               2b'11: reserved                                                      */
    uint16_t shdn_n : 1;        /*!< [8..8] PAD enable, 1: enable PAD, 0: shutdown                             */
    uint16_t smt_en : 1;        /*!< [9..9] Enable GPIOA0 Schmitt trigger; 1: enable                          */
    uint16_t driving : 2;       /*!< [11..10] (E3,E2)=(10:9) 1.8V: 00: 2mA; 01: 4mA; 10: 6mA; 11:
                                                    8mA 3.3V: 00: 4mA; 01: 6mA; 10: 12mA; 11: 16mA                  */
} hal_pin_cfg_t, *phal_pin_cfg_t;

typedef hal_status_t (*pin_register_t) (uint8_t pin_name, uint8_t periphl_id);
typedef hal_status_t (*pin_unregister_t) (uint8_t pin_name, uint8_t periphl_id);
typedef hal_status_t (*pin_validation_t) (uint8_t pin_name, uint8_t periphl_id);
typedef hal_status_t (*io_port_pwrup_t) (uint8_t pin_name, uint8_t periphl_id);
typedef hal_status_t (*io_port_pwrdn_t) (uint8_t pin_name, uint8_t periphl_id);
typedef hal_status_t (*pin_mux_cfg_t) (uint8_t pin_name, uint8_t periphl_id);

/**
  \brief  Pin management data structure to handle pin mux conflct, validate, power down management functions.
*/
typedef struct hal_pin_mux_mang_s {
    pin_register_t pin_reg_func;                ///< the callback function for pin register: usage record and conflict checking
    pin_unregister_t pin_unreg_func;            ///< the callback function for pin un-register
    pin_validation_t pin_validat_func;          ///< the callback function for pin validation checking
    pin_mux_cfg_t pin_mux_cfg_func;             ///< the callback function for pin mux configuration
    io_port_pwrup_t ioport_pwrup_ctrl_func;     ///< the callback function for GPIO pin power up control
    io_port_pwrdn_t ioport_pwrdn_ctrl_func;     ///< the callback function for GPIO pin power down control
    uint32_t pinmux_pwr_log[PORT_MAX_NUM];      ///< the bit map to record each pin is powered(enabled) or not
    uint32_t pinmux_reg_log[PORT_MAX_NUM];      ///< the bit map to record each pin is in using or not
    uint8_t *pinmux_reg_rec;                    ///< the array to record each pin is allocated to which peripheral
    uint32_t trap_pin_pwr_log;                  ///< the bit map to record trap pins are powered(enabled) or not
} hal_pin_mux_mang_t, *phal_pin_mux_mang_t;

void hal_pinmux_manager_init_rtl8710c (hal_pin_mux_mang_t *pinmux_manag);
hal_status_t hal_pin_register_rtl8710c (uint8_t pin_name, uint8_t periphl_id);
hal_status_t hal_pin_unregister_rtl8710c (uint8_t pin_name, uint8_t periphl_id);
hal_status_t hal_pin_mux_cfg_rtl8710c (uint8_t pin_name, uint8_t periphl_id);
hal_status_t hal_pin_pwrup_rtl8710c (uint8_t pin_name, uint8_t periphl_id);
hal_status_t hal_pin_pwrdwn_rtl8710c (uint8_t pin_name, uint8_t periphl_id);
hal_status_t hal_pinmux_register_rtl8710c (uint8_t pin_name, uint8_t periphl_id);
hal_status_t hal_pinmux_unregister_rtl8710c (uint8_t pin_name, uint8_t periphl_id);
hal_status_t hal_pin_get_cfg_rtl8710c (uint8_t pin_name, hal_pin_cfg_t *pin_cfg);

/**
  \brief  The data structure of the stubs function for the pin mux management HAL functions in ROM.
*/
typedef struct hal_pin_manag_func_stubs_s {
    phal_pin_mux_mang_t *pppin_manager;
    void (*hal_pinmux_manager_init) (hal_pin_mux_mang_t *pinmux_manag);
    hal_status_t (*hal_pin_register) (uint8_t pin_name, uint8_t periphl_id);
    hal_status_t (*hal_pin_unregister) (uint8_t pin_name, uint8_t periphl_id);
    hal_status_t (*hal_pin_mux_cfg) (uint8_t pin_name, uint8_t periphl_id);
    hal_status_t (*hal_pin_get_cfg) (uint8_t pin_name, hal_pin_cfg_t *pin_cfg);
    hal_status_t (*hal_pin_pwrup) (uint8_t pin_name, uint8_t periphl_id);
    hal_status_t (*hal_pin_pwrdwn) (uint8_t pin_name, uint8_t periphl_id);
    hal_status_t (*hal_pinmux_register) (uint8_t pin_name, uint8_t periphl_id);
    hal_status_t (*hal_pinmux_unregister) (uint8_t pin_name, uint8_t periphl_id);

    uint32_t reserved[8];  // reserved space for next ROM code version function table extending.
} hal_pin_manag_func_stubs_t, *phal_pin_manag_func_stubs_t;

#ifdef __cplusplus
}
#endif

#endif /* RTL8710C_PIN_MUX_H */


