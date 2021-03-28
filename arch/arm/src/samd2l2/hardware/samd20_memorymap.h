/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd20_memorymap.h
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
 *   "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *   Datasheet", 42129J�SAM�12/2013
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD20_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD20_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Memory Map */

#define SAM_FLASH_BASE   0x00000000 /* Embedded FLASH memory space (<= 256KB) */
#define SAM_CALIB_BASE   0x00800000 /* Calibration and auxiliary space */
#define SAM_SRAM_BASE    0x20000000 /* Embedded SRAM memory space (<= 64KB) */
#define SAM_AHBA_BASE    0x40000000 /* AHB-APB Bridge A (64KB) */
#define SAM_AHBB_BASE    0x41000000 /* AHB-APB Bridge B (64KB) */
#define SAM_AHBC_BASE    0x42000000 /* AHB-APB Bridge C (64KB) */

/* Calibration and Auxiliary Space */

#define SAM_LOCKBIT_BASE 0x00802000 /* LOCKBIT Base Address */
#define SAM_AUX0_BASE    0x00804000 /* AUX0 offset address */
#define SAM_AUX1_BASE    0x00806000 /* AUX1 offset address */
#  define SAM_AUX1_AREA1 0x00806000 /* Area 1 offset address (reserved, 64 bits) */
#  define SAM_AUX1_AREA2 0x00806008 /* Area 2 Device configuration area (64 bits) */
#  define SAM_AUX1_AREA3 0x00806010 /* Area 3 offset address (reserved, 128 bits) */
#  define SAM_AUX1_AREA4 0x00806020 /* Area 4 Software calibration area (256 bits) */

#define SAM_NVMCALIB_AREA  SAM_AUX1_AREA4 /* Use same name of SAML21 */

/* AHB-APB Bridge A */

#define SAM_PAC0_BASE    0x40000000 /* Peripheral Access Controller 0 */
#define SAM_PM_BASE      0x40000400 /* Power Manager */
#define SAM_SYSCTRL_BASE 0x40000800 /* System Controller */
#define SAM_GCLK_BASE    0x40000c00 /* Generic Clock Controller */
#define SAM_WDT_BASE     0x40001000 /* Watchdog Timer */
#define SAM_RTC_BASE     0x40001400 /* Real-Time Counter */
#define SAM_EIC_BASE     0x40001800 /* External Interrupt Controller */

/* AHB-APB Bridge B */

#define SAM_PAC1_BASE    0x41000000 /* Peripheral Access Controller 1 */
#define SAM_DSU_BASE     0x41002000 /* Device Service Unit */
#define SAM_NVMCTRL_BASE 0x41004000 /* Non-Volatile Memory Controller */
#define SAM_PORT_BASE    0x41004400 /* Ports */

/* AHB-APB Bridge C */

#define SAM_PAC2_BASE    0x42000000 /* Peripheral Access Controller 2 */
#define SAM_EVSYS_BASE   0x42000400 /* Event System */
#define SAM_SERCOM0_BASE 0x42000800 /* Serial Communication Interface 0 */
#define SAM_SERCOM1_BASE 0x42000c00 /* Serial Communication Interface 1 */
#define SAM_SERCOM2_BASE 0x42001000 /* Serial Communication Interface 2 */
#define SAM_SERCOM3_BASE 0x42001400 /* Serial Communication Interface 3 */
#define SAM_SERCOM4_BASE 0x42001800 /* Serial Communication Interface 4 */
#define SAM_SERCOM5_BASE 0x42001c00 /* Serial Communication Interface 5 */
#define SAM_TC0_BASE     0x42002000 /* Timer/Counter 0 */
#define SAM_TC1_BASE     0x42002400 /* Timer/Counter 1 */
#define SAM_TC2_BASE     0x42002800 /* Timer/Counter 2 */
#define SAM_TC3_BASE     0x42002c00 /* Timer/Counter 3 */
#define SAM_TC4_BASE     0x42003000 /* Timer/Counter 4 */
#define SAM_TC5_BASE     0x42003400 /* Timer/Counter 5 */
#define SAM_TC6_BASE     0x42003800 /* Timer/Counter 6 */
#define SAM_TC7_BASE     0x42003c00 /* Timer/Counter 7 */
#define SAM_ADC_BASE     0x42004000 /* Analog-to-Digital Converter */
#define SAM_AC_BASE      0x42004400 /* Analog Comparator*/
#define SAM_DAC_BASE     0x42004800 /* Digital-to-Analog Converter */
#define SAM_PTC_BASE     0x42004c00 /* Peripheral Touch Controller */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD20_MEMORYMAP_H */
