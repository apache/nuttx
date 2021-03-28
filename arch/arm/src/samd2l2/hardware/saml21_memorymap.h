/****************************************************************************
 * arch/arm/src/samd2l2/hardware/saml21_memorymap.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML21_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML21_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Memory Map */

#define SAM_FLASH_BASE       0x00000000 /* Embedded FLASH memory space (<= 256KB) */
#define SAM_FLASHRWW_BASE    0x00400000 /* Embedded FLASH RWW memory space (<= 8KB) */
#define SAM_NVM_BASE         0x00800000 /* Readable NVM content */
#define SAM_SRAM_BASE        0x20000000 /* Embedded SRAM memory space (<= 32KB) */
#define SAM_LPSRAM_BASE      0x30000000 /* Embedded low-power SRAM memory space (<= 8KB) */
#define SAM_AHBA_BASE        0x40000000 /* AHB-APB Bridge A (64KB) */
#define SAM_AHBB_BASE        0x41000000 /* AHB-APB Bridge B (64KB) */
#define SAM_AHBC_BASE        0x42000000 /* AHB-APB Bridge C (64KB) */
#define SAM_AHBD_BASE        0x43000000 /* AHB-APB Bridge D (64KB) */
#define SAM_AHBE_BASE        0x44000000 /* AHB-APB Bridge D (64KB) */
#define SAM_IOBUS_BASE       0x60000000 /* IOBUS (O.5KB) */
#define SAM_SYSTEM_BASE      0x60000200 /* System */

/* Non-volatile memory */

#define SAM_NVMUSER_ROW      0x00804000 /* NVM user row */
#define SAM_NVMCALIB_AREA    0x00806020 /* NVM software calibration area */
#define SAM_NVM_SERIALNO     0x0080a00c /* Serial number */

/* AHB-APB Bridge A */

#define SAM_PM_BASE          0x40000000 /* Power Management */
#define SAM_MCLK_BASE        0x40000400 /* Main Clock */
#define SAM_RSTC_BASE        0x40000800 /* Reset controller */
#define SAM_OSCCTRL_BASE     0x40000c00 /* Oscillators Controller */
#define SAM_OSC32KCTRL_BASE  0x40001000 /* 32KHz Oscillators Controller */
#define SAM_SUPC_BASE        0x40001400 /* Supply Controller */
#define SAM_GCLK_BASE        0x40001800 /* Generic Clock Controller */
#define SAM_WDT_BASE         0x40001c00 /* Watchdog Timer */
#define SAM_RTC_BASE         0x40002000 /* Real-Time Counter */
#define SAM_EIC_BASE         0x40002400 /* External Interrupt Controller */
#define SAM_PORT_BASE        0x40002800 /* Ports */

/* AHB-APB Bridge B */

#define SAM_USB_BASE         0x41000000 /* Universal Serial Bus */
#define SAM_DSU_BASE         0x41002000 /* Device Service Unit */
#define SAM_NVMCTRL_BASE     0x41004000 /* Non-Volatile Memory Controller */
#define SAM_MTB_BASE         0x41006000 /* Micro trace buffer */

/* AHB-APB Bridge C */

#define SAM_SERCOM0_BASE     0x42000000 /* Serial Communication Interface 0 */
#define SAM_SERCOM1_BASE     0x42000400 /* Serial Communication Interface 1 */
#define SAM_SERCOM2_BASE     0x42000800 /* Serial Communication Interface 2 */
#define SAM_SERCOM3_BASE     0x42000c00 /* Serial Communication Interface 3 */
#define SAM_SERCOM4_BASE     0x42001000 /* Serial Communication Interface 4 */
#define SAM_TCC0_BASE        0x42001400 /* Timer/Counter Control 0 */
#define SAM_TCC1_BASE        0x42001800 /* Timer/Counter Control 1 */
#define SAM_TCC2_BASE        0x42001c00 /* Timer/Counter Control 2 */
#define SAM_TC0_BASE         0x42002000 /* Timer/Counter 0 */
#define SAM_TC1_BASE         0x42002400 /* Timer/Counter 1 */
#define SAM_TC2_BASE         0x42002800 /* Timer/Counter 2 */
#define SAM_TC3_BASE         0x42002c00 /* Timer/Counter 3 */
#define SAM_DAC_BASE         0x42003000 /* Digital-to-Analog Converter */
#define SAM_AES_BASE         0x42003400 /* Advanced Encryption Standard */
#define SAM_TRNG_BASE        0x42003800 /* True Random Number Generator */

/* AHB-APB Bridge D */

#define SAM_EVSYS_BASE       0x43000000 /* Event system */
#define SAM_SERCOM5_BASE     0x43000400 /* Serial Communication Interface 5 */
#define SAM_TC4_BASE         0x43000800 /* Timer/Counter 4 */
#define SAM_ADC_BASE         0x43000c00 /* Analog-to-Digital Converter */
#define SAM_AC_BASE          0x43001000 /* Analog Comparator */
#define SAM_PTC_BASE         0x43001400 /* Peripheral Touch Controller */
#define SAM_OPAMP_BASE       0x43001800 /* OpAmps */
#define SAM_CCL_BASE         0x43001c00 /* Configurable Custom Logic */

/* AHB-APB Bridge E */

#define SAM_PAC_BASE         0x44000000 /* Peripheral Access Controller */
#define SAM_DMAC_BASE        0x44000400 /* DMA Controller */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML21_MEMORYMAP_H */
