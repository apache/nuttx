/**************************************************************************//**
 * @file     platform_conf.h
 * @brief    The configuration for AmebaPro High Power(TM9) platform.
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

#ifndef _PLATFORM_CONF_H_
#define _PLATFORM_CONF_H_

/* System configuration */
#define RTL8710C                            1
#define CONFIG_FPGA                         0   // is on FPGA platform?

#define CHIP_A_CUT                          0
#define CHIP_B_CUT                          1
#define CHIP_C_CUT                          2
#define CHIP_D_CUT                          3

#if CONFIG_FPGA
// FPGA
#define XTAL                                (12000000U)      /* Oscillator frequency */
//#define XTAL                                (12500000U)      /* Oscillator frequency */
#define CONFIG_CPU_CLK                      (XTAL)
#define PLATFORM_SCLK                       (8000000)
#define CHIP_VER                            (CHIP_D_CUT)
#else
// ASIC
#define CONFIG_PLL_CLK                      (200000000U)
#define CONFIG_CPU_CLK                      (CONFIG_PLL_CLK/2)
#define PLATFORM_SCLK                       (40000000U)
#define CHIP_VER                            (CHIP_D_CUT)
#endif

/* ROM code version */
#define ROM_VER_MAIN                        (3)
#define ROM_VER_SUB                         (0)

/* Peripheral device configuration */
#define CONFIG_GTIMER_EN                    1
#define CONFIG_WDG_EN                       1
#define CONFIG_GDMA_EN                      1
#define CONFIG_GPIO_EN                      1
#define CONFIG_UART_EN                      1
#define CONFIG_I2C_EN                       1
#define CONFIG_SOC_PS_EN                    0
#define CONFIG_EFUSE_EN                     1
#define CONFIG_EFUSE_NSC_EN                 1
#define CONFIG_SPI_FLASH_EN                 1
#define CONFIG_PWM_EN                       1
#define CONFIG_SPI_EN                       1
#define CONFIG_SDIO_DEVICE_EN               1
#define CONFIG_WLAN_EN                      1
#define CONFIG_CRYPTO_EN                    1
#define CONFIG_PSRAM_EN                     1
#define CONFIG_SCE_EN                       1

#define CONFIG_DEBUG_LOG                    1
#define CONFIG_DEBUG_ERROR                  1
#define CONFIG_DEBUG_WARN                   1
#define CONFIG_DEBUG_INFO                   1

#define CONFIG_CMSIS_RTX_EN                 0   // No RTX RTOS
#define CONFIG_RTX_IN_ROM                   0   // is RTX in ROM ?

#define CONFIG_CMSIS_FREERTOS_EN            1

#define CONFIG_CMSIS_OS_EN                  (CONFIG_CMSIS_RTX_EN | CONFIG_CMSIS_FREERTOS_EN)

#if CONFIG_CMSIS_RTX_EN && CONFIG_CMSIS_FREERTOS_EN
#error "RTX & FreeRTOS cannot be enabled at same time!!"
#endif

#define IFX_XMC4XXX                         1

#define CONFIG_LIGHT_PRINTF                 1   // use lighter printf (use smaller stack size)
#define CONFIG_SYS_TIMER_ID                 0   // the G-Timer ID be used as free run system ticker
#define CONFIG_TIMER_SCLK_FREQ              (PLATFORM_SCLK)   // GTimer SCLK: 32K/ 26M / 40M
#if (CONFIG_TIMER_SCLK_FREQ == 32000)
#define CONFIG_SYS_TICK_TIME                (1000000/CONFIG_TIMER_SCLK_FREQ)   // in us
#else
#define CONFIG_SYS_TICK_TIME                (1)     // in us
#endif
#define STDIO_UART_TX_PIN                   (16)    // GPIO A_16
#define STDIO_UART_RX_PIN                   (15)    // GPIO A_15

/* Verifi each IP configuration */
#define CONFIG_VRF_MODE                      0

#define LOAD_FLAH_IMG_EN                    (1)     // is enable to load the image from the flash

#define CONFIG_FLASH_XIP_EN                 (1)     // is enable the Flash XIP (eXecute In Place)
#define CONFIG_EXRAM_PSRAM_EN               (1)     // is PSRAM memory present on this platform

#define CONFIG_FBOOT_AUTH                   (1)     /* is do IMG2 authentication on fast reboot.
                                                    // if the FW image is encrypted then should rise this flag */
#define HARD_FAULT_BACK_TRACE               (1)     // is enable the stack back trace for hard fault exception

#define RAM_START_JTAG_ENABLE               (1)     /* is enable the JTAG/SWD pins at RAM code start up 
                                                    // (only for the JTAG function is enabled) */
#if defined(CONFIG_BUILD_RAM) && (CONFIG_BUILD_RAM == 1)
#if defined(CONFIG_BUILD_SECURE)
#define __STACK_SIZE                        (0x1000)    // main stack memory size of secure region
#elif defined(CONFIG_BUILD_NONSECURE)
#define __STACK_SIZE                        (0x1000)    // main stack memory size of non-secure region
#else
#define __STACK_SIZE                        (0x1000)    // main stack memory size of ignore-secure
#endif
#endif  // end of "#if defined(CONFIG_BUILD_RAM) && (CONFIG_BUILD_RAM == 1)"

#endif  // end of "#define _PLATFORM_CONF_H_"

