/**************************************************************************//**
 * @file     hal_misc.c
 * @brief    This file implements the entry functions of the HAL Misc. ROM functions.
 *
 * @version  V1.00
 * @date     2017-05-24
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

#include "hal_misc.h"
#include "hal_pinmux.h"
#include "hal_gpio.h"
#include "hal_spic.h"
#include "hal_sys_ctrl.h"

extern hal_spic_adaptor_t *pglob_spic_adaptor;
extern void hal_flash_return_spi (phal_spic_adaptor_t phal_spic_adaptor);

/**
 * @addtogroup hs_hal_misc Misc. Functions
 * @{
 */

/**
  * @brief Pin mux selection table for JTAG pins
  */
const hal_jtag_pin_t jtag_pins = {
    .pin_tclk = PIN_A0,
    .pin_tms = PIN_A1,
    .pin_tdo = PIN_A2,
    .pin_tdi = PIN_A3,
    .pin_trst = PIN_A4
};

/**
 *  @brief Controls the enable of SWD pins.
 *
 *  @param[in]  en  Pin enable control:
 *                    - 0: Disable SWD pins.
 *                    - 1: Enable SWD pins.
 *
 *  @return     HAL_ERR_PARA:  Input pin mux selection is invalid.
 *  @return     HAL_ERR_CONFLICT: pin conflict with other peripheral device.
 *  @return     HAL_OK:  pin control OK.
 */
hal_status_t hal_misc_swd_pin_ctrl (BOOL en)
{
    hal_status_t ret;

    if (en) {
        ret = hal_pinmux_register(jtag_pins.pin_swclk, PID_JTAG);
        ret |= hal_pinmux_register(jtag_pins.pin_swdio, PID_JTAG);
        if (ret == HAL_OK) {
            hal_gpio_pull_ctrl(jtag_pins.pin_swclk, Pin_PullUp);
            hal_gpio_pull_ctrl(jtag_pins.pin_swdio, Pin_PullUp);
        }
    } else {
        ret = hal_pinmux_unregister(jtag_pins.pin_swclk, PID_JTAG);
        ret |= hal_pinmux_unregister(jtag_pins.pin_swdio, PID_JTAG);
        if (ret == HAL_OK) {
            hal_gpio_pull_ctrl(jtag_pins.pin_swclk, Pin_PullNone);
            hal_gpio_pull_ctrl(jtag_pins.pin_swdio, Pin_PullNone);
        }
    }

    return ret;
}

/**
 *  @brief Controls the enable of JTAG pins.
 *
 *  @param[in]  en  Pin enable control:
 *                    - 0: Disable JTAG pins.
 *                    - 1: Enable JTAG pins.
 *
 *  @return     HAL_ERR_PARA:  Input pin mux selection is invalid.
 *  @return     HAL_ERR_CONFLICT: pin conflict with other peripheral device.
 *  @return     HAL_OK:  pin control OK.
 */
hal_status_t hal_misc_jtag_pin_ctrl (BOOL en)
{
    hal_status_t ret;
    uint8_t *pin;
    uint32_t i;
    uint32_t j;

    pin = (uint8_t *)&jtag_pins;
    if (en) {
        for (i=0; i<sizeof(jtag_pins); i++) {
            ret = hal_pinmux_register(*(pin+i), PID_JTAG);
            if (ret == HAL_OK) {
                // also pull-high the pin
                hal_gpio_pull_ctrl(*(pin+i), Pin_PullUp);                
            } else {
                DBG_MISC_ERR("JTAG pin 0x%x reg err\r\n", *(pin+i));
                pin = (uint8_t *)&jtag_pins;
                // unregister all JTAG pins
                for (j=0; j<i; j++) {
                    hal_pinmux_unregister(*(pin+i), PID_JTAG);
                }
                break;
            }
        }
    } else {
        for (i=0; i<sizeof(jtag_pins); i++) {
            ret = hal_pinmux_unregister(*(pin+i), PID_JTAG);
            if (ret == HAL_OK) {
                // also pull-high the pin
                hal_gpio_pull_ctrl(*(pin+i), Pin_PullNone);                
            }
        }
    }

    return ret;
}

/**
 *  @brief Invokes a CPU reset. Compares to the system reset, it only reset the
 *         CPU part and the program will be restarted. All other peripheral keeps
 *         their state.
 *
 *  @returns    void
 */
void hal_misc_cpu_rst (void)
{
    rtl8710c_reset_reason_set(HAL_RESET_REASON_SOFTWARE);
#if defined(CONFIG_FLASH_XIP_EN) && (CONFIG_FLASH_XIP_EN == 1)
    if (pglob_spic_adaptor != NULL) {
        hal_flash_return_spi (pglob_spic_adaptor);
    }
#endif
    hal_misc_stubs.hal_misc_cpu_rst ();
}


/** @} */ /* End of group hs_hal_misc */

