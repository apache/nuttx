/****************************************************************************
 * arch/arm/src/stm32l5/hardware/stm32l5_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_PWR_H
#define __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32L5_PWR_CR1_OFFSET      0x0000  /* Power control register 1 */
#define STM32L5_PWR_CR2_OFFSET      0x0004  /* Power control register 2 */
#define STM32L5_PWR_CR3_OFFSET      0x0008  /* Power control register 3 */
#define STM32L5_PWR_CR4_OFFSET      0x000C  /* Power control register 4 */
#define STM32L5_PWR_SR1_OFFSET      0x0010  /* Power status register 1 */
#define STM32L5_PWR_SR2_OFFSET      0x0014  /* Power status register 2 */
#define STM32L5_PWR_SCR_OFFSET      0x0018  /* Power status clear register */
#define STM32L5_PWR_PUCRA_OFFSET    0x0020  /* Power Port A pull-up control register */
#define STM32L5_PWR_PDCRA_OFFSET    0x0024  /* Power Port A pull-down control register */
#define STM32L5_PWR_PUCRB_OFFSET    0x0028  /* Power Port B pull-up control register */
#define STM32L5_PWR_PDCRB_OFFSET    0x002C  /* Power Port B pull-down control register */
#define STM32L5_PWR_PUCRC_OFFSET    0x0030  /* Power Port C pull-up control register */
#define STM32L5_PWR_PDCRC_OFFSET    0x0034  /* Power Port C pull-down control register */
#define STM32L5_PWR_PUCRD_OFFSET    0x0038  /* Power Port D pull-up control register */
#define STM32L5_PWR_PDCRD_OFFSET    0x003C  /* Power Port D pull-down control register */
#define STM32L5_PWR_PUCRE_OFFSET    0x0040  /* Power Port E pull-up control register */
#define STM32L5_PWR_PDCRE_OFFSET    0x0044  /* Power Port E pull-down control register */
#define STM32L5_PWR_PUCRF_OFFSET    0x0048  /* Power Port F pull-up control register */
#define STM32L5_PWR_PDCRF_OFFSET    0x004C  /* Power Port F pull-down control register */
#define STM32L5_PWR_PUCRG_OFFSET    0x0050  /* Power Port G pull-up control register */
#define STM32L5_PWR_PDCRG_OFFSET    0x0054  /* Power Port G pull-down control register */
#define STM32L5_PWR_PUCRH_OFFSET    0x0058  /* Power Port H pull-up control register */
#define STM32L5_PWR_PDCRH_OFFSET    0x005C  /* Power Port H pull-down control register */
#define STM32L5_PWR_SECCFGR_OFFSET  0x0078  /* Power secure configuration register */
#define STM32L5_PWR_PRIVCFGR_OFFSET 0x0078  /* Power privilege configuration register */

/* Register Addresses *******************************************************/

#define STM32L5_PWR_CR1          (STM32L5_PWR_BASE + STM32L5_PWR_CR1_OFFSET)
#define STM32L5_PWR_CR2          (STM32L5_PWR_BASE + STM32L5_PWR_CR2_OFFSET)
#define STM32L5_PWR_CR3          (STM32L5_PWR_BASE + STM32L5_PWR_CR3_OFFSET)
#define STM32L5_PWR_CR4          (STM32L5_PWR_BASE + STM32L5_PWR_CR4_OFFSET)
#define STM32L5_PWR_SR1          (STM32L5_PWR_BASE + STM32L5_PWR_SR1_OFFSET)
#define STM32L5_PWR_SR2          (STM32L5_PWR_BASE + STM32L5_PWR_SR2_OFFSET)
#define STM32L5_PWR_SCR          (STM32L5_PWR_BASE + STM32L5_PWR_SCR_OFFSET)
#define STM32L5_PWR_PUCRA        (STM32L5_PWR_BASE + STM32L5_PWR_PUCRA_OFFSET)
#define STM32L5_PWR_PDCRA        (STM32L5_PWR_BASE + STM32L5_PWR_PDCRA_OFFSET)
#define STM32L5_PWR_PUCRB        (STM32L5_PWR_BASE + STM32L5_PWR_PUCRB_OFFSET)
#define STM32L5_PWR_PDCRB        (STM32L5_PWR_BASE + STM32L5_PWR_PDCRB_OFFSET)
#define STM32L5_PWR_PUCRC        (STM32L5_PWR_BASE + STM32L5_PWR_PUCRC_OFFSET)
#define STM32L5_PWR_PDCRC        (STM32L5_PWR_BASE + STM32L5_PWR_PDCRC_OFFSET)
#define STM32L5_PWR_PUCRD        (STM32L5_PWR_BASE + STM32L5_PWR_PUCRD_OFFSET)
#define STM32L5_PWR_PDCRD        (STM32L5_PWR_BASE + STM32L5_PWR_PDCRD_OFFSET)
#define STM32L5_PWR_PUCRE        (STM32L5_PWR_BASE + STM32L5_PWR_PUCRE_OFFSET)
#define STM32L5_PWR_PDCRE        (STM32L5_PWR_BASE + STM32L5_PWR_PDCRE_OFFSET)
#define STM32L5_PWR_PUCRF        (STM32L5_PWR_BASE + STM32L5_PWR_PUCRF_OFFSET)
#define STM32L5_PWR_PDCRF        (STM32L5_PWR_BASE + STM32L5_PWR_PDCRF_OFFSET)
#define STM32L5_PWR_PUCRG        (STM32L5_PWR_BASE + STM32L5_PWR_PUCRG_OFFSET)
#define STM32L5_PWR_PDCRG        (STM32L5_PWR_BASE + STM32L5_PWR_PDCRG_OFFSET)
#define STM32L5_PWR_PUCRH        (STM32L5_PWR_BASE + STM32L5_PWR_PUCRH_OFFSET)
#define STM32L5_PWR_PDCRH        (STM32L5_PWR_BASE + STM32L5_PWR_PDCRH_OFFSET)
#define STM32L5_PWR_SECCFGR      (STM32L5_PWR_BASE + STM32L5_PWR_SECCFGR_OFFSET)
#define STM32L5_PWR_PRIVCFGR     (STM32L5_PWR_BASE + STM32L5_PWR_PRIVCFGR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Power control register 1 */

#define PWR_CR1_LPMS_SHIFT       0
#define PWR_CR1_LPMS_MASK        (7 << PWR_CR1_LPMS_SHIFT) /* Bits 0-2: Low-power mode selection */
#  define PWR_CR1_LPMS_STOP0     (0 << PWR_CR1_LPMS_SHIFT) /* 000: Stop 0 mode */
#  define PWR_CR1_LPMS_STOP1     (1 << PWR_CR1_LPMS_SHIFT) /* 001:Stop 1 mode */
#  define PWR_CR1_LPMS_STOP2     (2 << PWR_CR1_LPMS_SHIFT) /* 010: Stop 2 mode */
#  define PWR_CR1_LPMS_STANDBY   (3 << PWR_CR1_LPMS_SHIFT) /* 011: Standby mode */
#  define PWR_CR1_LPMS_SHUTDOWN  (4 << PWR_CR1_LPMS_SHIFT) /* 1xx: Shutdown mode */
#define PWR_CR1_DBP              (1 <<  8)                 /* Bit  8: Disable Backup domain write protection */
#define PWR_CR1_VOS_SHIFT        9
#define PWR_CR1_VOS_MASK         (3 << PWR_CR1_VOS_SHIFT) /* Bits 9-10: Voltage scaling range selection */
#define PWR_CR1_VOS_RANGE0       (0 << PWR_CR1_VOS_SHIFT) /* 00: Range 0 */
#define PWR_CR1_VOS_RANGE1       (1 << PWR_CR1_VOS_SHIFT) /* 01: Range 1 */
#define PWR_CR1_VOS_RANGE2       (2 << PWR_CR1_VOS_SHIFT) /* 10: Range 2 */
#define PWR_CR1_LPR              (1 << 14)                /* Bit 14: Low-power run */

/* Power control register 2 */

#define PWR_CR2_PVDE             (1 <<  0)                /* Bit  0: Power voltage detector enable */
#define PWR_CR2_PLS_SHIFT        1
#define PWR_CR2_PLS_MASK         (7 << PWR_CR2_PLS_SHIFT) /* Bits 1-3: Power voltage detector level selection */
#  define PWR_CR2_PLS_2000mv     (0 << PWR_CR2_PLS_SHIFT) /* 000: VPVD0 around 2.0V */
#  define PWR_CR2_PLS_2200mv     (1 << PWR_CR2_PLS_SHIFT) /* 001: VPVD1 around 2.2V */
#  define PWR_CR2_PLS_2400mv     (2 << PWR_CR2_PLS_SHIFT) /* 010: VPVD2 around 2.4V */
#  define PWR_CR2_PLS_2500mv     (3 << PWR_CR2_PLS_SHIFT) /* 011: VPVD3 around 2.5V */
#  define PWR_CR2_PLS_2600mv     (4 << PWR_CR2_PLS_SHIFT) /* 100: VPVD4 around 2.6V */
#  define PWR_CR2_PLS_2800mv     (5 << PWR_CR2_PLS_SHIFT) /* 101: VPVD5 around 2.8V */
#  define PWR_CR2_PLS_2900mv     (6 << PWR_CR2_PLS_SHIFT) /* 110: VPVD6 around 2.9V */
#  define PWR_CR2_PLS_EXT        (7 << PWR_CR2_PLS_SHIFT) /* 111: External input analog voltage PVD_IN */
#define PWR_CR2_PVME1            (1 <<  4)                /* Bit  4: Peripheral voltage monitoring 1 enable (VDDUSB vs 1.2V) */
#define PWR_CR2_PVME2            (1 <<  5)                /* Bit  5: Peripheral voltage monitoring 2 enable (VDDIO2 vs 0.9V) */
#define PWR_CR2_PVME3            (1 <<  6)                /* Bit  6: Peripheral voltage monitoring 3 enable (VDDA vs 1.62V) */
#define PWR_CR2_PVME4            (1 <<  7)                /* Bit  7: Peripheral voltage monitoring 4 enable (VDDA vs 2.2V) */
#define PWR_CR2_IOSV             (1 <<  9)                /* Bit  9: VDDIO2 Independent I/Os supply valid */
#define PWR_CR2_USV              (1 << 10)                /* Bit 10: VDDUSB USB supply valid */

/* Power control register 3 */

#define PWR_CR3_EWUP1            (1 <<  0)                /* Bit  0: Enable Wakeup pin WKUP1 */
#define PWR_CR3_EWUP2            (1 <<  1)                /* Bit  1: Enable Wakeup pin WKUP2 */
#define PWR_CR3_EWUP3            (1 <<  2)                /* Bit  2: Enable Wakeup pin WKUP3 */
#define PWR_CR3_EWUP4            (1 <<  3)                /* Bit  3: Enable Wakeup pin WKUP4 */
#define PWR_CR3_EWUP5            (1 <<  4)                /* Bit  4: Enable Wakeup pin WKUP5 */
#define PWR_CR3_RRS_SHIFT        8
#define PWR_CR3_RRS_MASK         (3 << PWR_CR3_RRS_SHIFT) /* Bits 8-9: SRAM2 retention in Standby-mode */
#  define PWR_CR3_RRS_OFF        (0 << PWR_CR3_RRS_SHIFT) /* 00: SRAM2 is powered off in Standby-mode */
#  define PWR_CR3_RRS_ON         (1 << PWR_CR3_RRS_SHIFT) /* 01: SRAM2 is powered on in Standby-mode */
#  define PWR_CR3_RRS_4K_ON      (2 << PWE_CR3_RRS_SHIFT) /* 10: Upper 4KB of SRAM2 powered on in Standby-mode */
#define PWR_CR3_APC              (1 << 10)                /* Bit 10: Apply pull-up and pull-down configuration */
#define PWR_CR3_ULPMEN           (1 << 11)                /* Bit 11: Ultra-low-power mode enable */
#define PWR_CR3_UCPD_STBY        (1 << 13)                /* Bit 13: USB Type-C power delivery Standby-mode */
#define PWR_CR3_UCPD_DBDIS       (1 << 14)                /* Bit 14: USB Type-C power delivery dead battery disable */

/* Power control register 4 */

#define PWR_CR4_WP1              (1 <<  0)    /* Bit  0: Wakeup pin WKUP1 polarity */
#define PWR_CR4_WP2              (1 <<  1)    /* Bit  1: Wakeup pin WKUP2 polarity */
#define PWR_CR4_WP3              (1 <<  2)    /* Bit  2: Wakeup pin WKUP3 polarity */
#define PWR_CR4_WP4              (1 <<  3)    /* Bit  3: Wakeup pin WKUP4 polarity */
#define PWR_CR4_WP5              (1 <<  4)    /* Bit  4: Wakeup pin WKUP5 polarity */
#define PWR_CR4_VBE              (1 <<  8)    /* Bit  8: Vbat battery charging enable */
#define PWR_CR4_VBRS             (1 <<  9)    /* Bit  9: Vbat battery charging resistor selection */
#  define PWR_CR4_VBRS_5k        0            /*     0: 5k  resistor */
#  define PWR_CR4_VBRS_1k5       PWR_CR4_VBRS /*     1: 1k5 resistor */
#define PWR_CR4_SMPSBYP          (1 << 12)    /* Bit 12: SMPS Bypass mode */
#define PWR_CR4_EXTSMPSEN        (1 << 13)    /* Bit 13: Enable external SMPS mode */
#define PWR_CR4_SMPSFSTEN        (1 << 14)    /* Bit 14: Enable SMPS fast soft start */
#define PWR_CR4_SMPSLPEN         (1 << 15)    /* Bit 15: Enable SMPS low-power mode */

/* Power status register 1 */

#define PWR_SR1_WUF1             (1 <<  0) /* Bit  0: Wakeup flag 1 */
#define PWR_SR1_WUF2             (1 <<  1) /* Bit  1: Wakeup flag 2 */
#define PWR_SR1_WUF3             (1 <<  2) /* Bit  2: Wakeup flag 3 */
#define PWR_SR1_WUF4             (1 <<  3) /* Bit  3: Wakeup flag 4 */
#define PWR_SR1_WUF5             (1 <<  4) /* Bit  4: Wakeup flag 5 */
#define PWR_SR1_SBF              (1 <<  8) /* Bit  8: Standby flag */
#define PWR_SR1_SMPSBYPRDY       (1 << 12) /* Bit 12: SMPS BYPASS ready */
#define PWR_SR1_EXTSMPSRDY       (1 << 13) /* Bit 13: External SMPS mode ready */
#define PWR_SR1_SMPSHPRDY        (1 << 15) /* Bit 15: SMPS high-power mode ready */

/* Power status register 2 */

#define PWR_SR2_REGLPS           (1 <<  8) /* Bit  8: Low power regulator started */
#define PWR_SR2_REGLPF           (1 <<  9) /* Bit  9: Low power regulator flag */
#define PWR_SR2_VOSF             (1 << 10) /* Bit 10: Voltage scaling flag */
#define PWR_SR2_PVDO             (1 << 11) /* Bit 11: Power voltage detector output */
#define PWR_SR2_PVMO1            (1 << 12) /* Bit 12: Peripheral voltage monitoring output 1 (VDDUSB vs 1.2V) */
#define PWR_SR2_PVMO2            (1 << 13) /* Bit 13: Peripheral voltage monitoring output 2 (VDDIO2 vs 0.9V) */
#define PWR_SR2_PVMO3            (1 << 14) /* Bit 14: Peripheral voltage monitoring output 3 (VDDA vs 1.62V) */
#define PWR_SR2_PVMO4            (1 << 15) /* Bit 15: Peripheral voltage monitoring output 4 (VDDA vs 2.2V) */

/* Power status clear register */

#define PWR_SCR_CWUF1            (1 <<  0) /* Bit  0: Clear wakeup flag 1 */
#define PWR_SCR_CWUF2            (1 <<  1) /* Bit  1: Clear wakeup flag 2 */
#define PWR_SCR_CWUF3            (1 <<  2) /* Bit  2: Clear wakeup flag 3 */
#define PWR_SCR_CWUF4            (1 <<  3) /* Bit  3: Clear wakeup flag 4 */
#define PWR_SCR_CWUF5            (1 <<  4) /* Bit  4: Clear wakeup flag 5 */
#define PWR_SCR_CSBF             (1 <<  8) /* Bit  8: Clear standby flag */

/* Port X pull-up/down registers have one bit per port line, with a few
 * exceptions.
 */

/* Power secure configuration register */

#define PWR_SECCFGR_WUP1SEC      (1 <<  0) /* Bit  0: WKUP1 pin security */
#define PWR_SECCFGR_WUP2SEC      (1 <<  1) /* Bit  1: WKUP2 pin security */
#define PWR_SECCFGR_WUP3SEC      (1 <<  2) /* Bit  2: WKUP3 pin security */
#define PWR_SECCFGR_WUP4SEC      (1 <<  3) /* Bit  3: WKUP4 pin security */
#define PWR_SECCFGR_WUP5SEC      (1 <<  4) /* Bit  4: WKUP5 pin security */
#define PWR_SECCFGR_LPMSEC       (1 <<  8) /* Bit  8: Low-power mode security */
#define PWR_SECCFGR_VDMSEC       (1 <<  9) /* Bit  9: Voltage detection and monitoring security */
#define PWR_SECCFGR_VBSEC        (1 << 10) /* Bit 10: Voltage battery security */
#define PWR_SECCFGR_APCSEC       (1 << 11) /* Bit 11: APC security */

/* Power privilege configuration register */

#define PWR_PRIVCFGR_PRIV        (1 <<  0) /* Bit  0: Privilege protection */

#endif /* __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_PWR_H */
