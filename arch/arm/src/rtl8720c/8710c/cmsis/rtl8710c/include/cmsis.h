/**************************************************************************//**
 * @file     cmsis.h
 * @brief    The generic CMSIS include file.
 * @version  V1.00
 * @date     2016-7-20
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

#ifndef MBED_CMSIS_H
#define MBED_CMSIS_H

#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#define KM4_ITCM_ROM_BASE         (0x00000000UL)
#define KM4_DTCM_RAM_BASE         (0x10000000UL)

#define KM4_APB_BASE              (0x40000000UL)
#define KM4_SEC_APB_BASE          (0x50000000UL)

#define KM4_SYSON_BASE            (KM4_APB_BASE       + 0x000000UL)
#define KM4_WLAN_BASE             (KM4_APB_BASE       + 0x080000UL)
#define KM4_PSRAMPHY_BASE         (KM4_APB_BASE       + 0x402000UL)     // PSRAM/LPDDR1 Phy. Controller
#define KM4_PSRAM_BASE            (KM4_APB_BASE       + 0x600000UL)     // PSRAM Controller

#define KM4_PSRAM_MEM_BASE        (0x60000000UL)

#define KM4_IDAU_BASE             (KM4_SEC_APB_BASE   + 0x000000UL)     // Secure Cross-Bar controller (IDAU)

#include "cmsis_compiler.h"
#include "platform_conf.h"
#include "basic_types.h"
#include "rtl8710c.h"
#include "section_config.h"
#include "hal_api.h"
#include "diag.h"

#include "rtl8710c_irq.h"             /* The IRQ priority definition */
#include "rtl8710c_syson_ctrl.h"
#if (defined(ROM_REGION) && (ROM_REGION==1)) || (defined(CONFIG_BUILD_LIB) && (CONFIG_BUILD_LIB==1))
#include "rtl8710c_syson.h"
#include "rtl8710c_vendor.h"
#include "rtl8710c_crypto.h"
#include "rtl8710c_crypto_ctrl.h"
#endif

#include "rtl8710c_vendor_ctrl.h"
#include "rtl8710c_peri_id.h"
#include "rtl8710c_pin_name.h"
#include "rtl8710c_cache.h"
#include "rtl8710c_gdma.h"
#include "rtl8710c_uart.h"
#include "rtl8710c_timer.h"
#include "rtl8710c_pwm.h"
#include "rtl8710c_flash.h"
#include "rtl8710c_spic.h"
#include "rtl8710c_ssi.h"
#include "rtl8710c_gpio.h"
#include "rtl8710c_lpi.h"
#include "rtl8710c_sce.h"
#include "rtl8710c_misc.h"
#include "rtl8710c_pinmux.h"
#include "rtl8710c_i2c.h"
#include "rtl8710c_efuse.h"
#include "rtl8710c_sdio_dev.h"
#include <arm_cmse.h>   /* Use CMSE intrinsics */

#endif  // end of "#ifndef MBED_CMSIS_H"

