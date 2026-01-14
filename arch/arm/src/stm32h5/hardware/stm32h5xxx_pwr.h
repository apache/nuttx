/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32h5xxx_pwr.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5XXX_PWR_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5XXX_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_PWR_PMCR_OFFSET     0x0000  /* Power Mode Control */
#define STM32_PWR_PMSR_OFFSET     0x0004  /* Power Mode Status */
#define STM32_PWR_VOSCR_OFFSET    0x0010  /* Voltage Scaling Control */
#define STM32_PWR_VOSSR_OFFSET    0x0014  /* Voltage Scaling Status */
#define STM32_PWR_BDCR_OFFSET     0x0020  /* Backup Domain Control */
#define STM32_PWR_DBPCR_OFFSET    0x0024  /* Backup Domain Protection Control */
#define STM32_PWR_BDSR_OFFSET     0x0028  /* Backup Domain Status */
#define STM32_PWR_UCPDR_OFFSET    0x002c  /* USB-C Power Delivery */
#define STM32_PWR_SCCR_OFFSET     0x0030  /* Supply Configuration Control */
#define STM32_PWR_VMCR_OFFSET     0x0034  /* Voltage Monitor Control */
#define STM32_PWR_USBSCR_OFFSET   0x0038  /* USB Supply Control */
#define STM32_PWR_VMSR_OFFSET     0x003c  /* Voltage Monitor Status */
#define STM32_PWR_WUSCR_OFFSET    0x0040  /* Wake-Up Status Clear */
#define STM32_PWR_WUSR_OFFSET     0x0044  /* Wake-Up Status */
#define STM32_PWR_WUCR_OFFSET     0x0048  /* Wake-Up Configuration */
#define STM32_PWR_IORETR_OFFSET   0x0050  /* I/O Retention */
#define STM32_PWR_SECCFGR_OFFSET  0x0100  /* Power secure configuration register */
#define STM32_PWR_PRIVCFGR_OFFSET 0x0104  /* Power privilege configuration register */

/* Register Addresses *******************************************************/

#define STM32_PWR_PMCR         (STM32_PWR_BASE + STM32_PWR_PMCR_OFFSET)
#define STM32_PWR_PMSR         (STM32_PWR_BASE + STM32_PWR_PMSR_OFFSET)
#define STM32_PWR_VOSCR        (STM32_PWR_BASE + STM32_PWR_VOSCR_OFFSET)
#define STM32_PWR_VOSSR        (STM32_PWR_BASE + STM32_PWR_VOSSR_OFFSET)
#define STM32_PWR_BDCR         (STM32_PWR_BASE + STM32_PWR_BDCR_OFFSET)
#define STM32_PWR_DBPCR        (STM32_PWR_BASE + STM32_PWR_DBPCR_OFFSET)
#define STM32_PWR_BDSR         (STM32_PWR_BASE + STM32_PWR_BDSR_OFFSET)
#define STM32_PWR_UCPDR        (STM32_PWR_BASE + STM32_PWR_UCPDR_OFFSET)
#define STM32_PWR_SCCR         (STM32_PWR_BASE + STM32_PWR_SCCR_OFFSET)
#define STM32_PWR_VMCR         (STM32_PWR_BASE + STM32_PWR_VMCR_OFFSET)
#define STM32_PWR_USBSCR       (STM32_PWR_BASE + STM32_PWR_USBSCR_OFFSET)
#define STM32_PWR_VMSR         (STM32_PWR_BASE + STM32_PWR_VMSR_OFFSET)
#define STM32_PWR_WUSCR        (STM32_PWR_BASE + STM32_PWR_WUSCR_OFFSET)
#define STM32_PWR_WUSR         (STM32_PWR_BASE + STM32_PWR_WUSR_OFFSET)
#define STM32_PWR_WUCR         (STM32_PWR_BASE + STM32_PWR_WUCR_OFFSET)
#define STM32_PWR_IORETR       (STM32_PWR_BASE + STM32_PWR_IORETR_OFFSET)
#define STM32_PWR_SECCFGR      (STM32_PWR_BASE + STM32_PWR_SECCFGR_OFFSET)
#define STM32_PWR_PRIVCFGR     (STM32_PWR_BASE + STM32_PWR_PRIVCFGR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Power Mode Control Register */

#define PWR_PMCR_LPMS           (1 << 0)  /* Low-Power Mode */
#define PWR_PMCR_SVOS_SHIFT     (2)       /* System Stop Mode Voltage Scaling */
#define PWR_PMCR_SVOS_MASK      (0x3 << PWR_PMCR_SVOS_SHIFT)
#  define PWR_PMCR_SVOS_SVOS5   (1 << PWR_PMCR_SVOS_SHIFT)
#  define PWR_PMCR_SVOS_SVOS4   (2 << PWR_PMCR_SVOS_SHIFT)
#  define PWR_PMCR_SVOS_SVOS3   (3 << PWR_PMCR_SVOS_SHIFT)
#define PWR_PMCR_CSSF           (1 << 7)  /* Clear Standby and Stop Flags */
#define PWR_PMCR_FLPS           (1 << 9)  /* Flash Memory Low-Power Mode in Stop Mode */
#define PWR_PMCR_BOOSTE         (1 << 12) /* Analog Switch Vboost Control */
#define PWR_PMCR_AVD_READY      (1 << 13) /* Analog Voltage Ready */
#define PWR_PMCR_ETHERNETSO     (1 << 16) /* Ethernet RAM shut-off in Stop Mode */
#define PWR_PMCR_SRAM3SO        (1 << 23) /* AHB SRAM3 shut-off in Stop mode */

#if defined(CONFIG_STM32H5_STM32H56X) || defined(CONFIG_STM32H5_STM32H7X)

#define PWR_PMCR_SRAM2_16SO     (1 << 24) /* AHB SRAM3 16-Kbyte shut-off in Stop mode **/
#define PWR_PMCR_SRAM2_48SO     (1 << 25) /* AHB SRAM2 48-Kbyte shut-off in Stop mode **/
#define PWR_PMCR_SRAM1SO        (1 << 26) /* AHB SRAM1 shut-off in Stop mode * */

#elif defined(CONFIG_STM32H5_STM32H2X) || defined(CONFIG_STM32H5_STM32H3X)

#define PWR_PMCR_SRAM2_16LSO    (1 << 24) /* AHB SRAM3 16-Kbyte Low shut-off in Stop mode **/
#define PWR_PMCR_SRAM2_16HSO    (1 << 25) /* AHB SRAM3 16-Kbyte High shut-off in Stop mode **/
#define PWR_PMCR_SRAM2_48SO     (1 << 26) /* AHB SRAM2 48-Kbyte shut-off in Stop mode **/
#define PWR_PMCR_SRAM1SO        (1 << 27) /* AHB SRAM1 shut-off in Stop mode */

#endif

/* Status Register */

#define PWR_PMSR_STOPF          (1 << 5) /* Stop Flag */
#define PWR_PMSR_SBF            (1 << 6) /* System Standby Flag */

/* Voltage Scaling Control Register */

#define PWR_VOSCR_VOS_SHIFT     (4)
#define PWR_VOSCR_VOS_MASK      (3 << PWR_VOSCR_VOS_SHIFT)
#  define PWR_VOSCR_VOS_RANGE3   (0 << PWR_VOSCR_VOS_SHIFT)
#  define PWR_VOSCR_VOS_RANGE2   (1 << PWR_VOSCR_VOS_SHIFT)
#  define PWR_VOSCR_VOS_RANGE1   (2 << PWR_VOSCR_VOS_SHIFT)
#  define PWR_VOSCR_VOS_RANGE0   (3 << PWR_VOSCR_VOS_SHIFT)

/* Voltage Scaling Status Register */

#define PWR_VOSSR_VOSRDY        (1 << 3)
#define PWR_VOSSR_ACTVOSRDY     (1 << 13)
#define PWR_VOSSR_ACTVOS_SHIFT  (14)
#define PWR_VOSSR_ACTVOS_MASK   (3 << PWR_VOSSR_ACTVOS_SHIFT)
#  define PWR_VOSSR_ACTVOS_VOS3 (0 << PWR_VOSSR_ACTVOS_SHIFT)
#  define PWR_VOSSR_ACTVOS_VOS2 (1 << PWR_VOSSR_ACTVOS_SHIFT)
#  define PWR_VOSSR_ACTVOS_VOS1 (2 << PWR_VOSSR_ACTVOS_SHIFT)
#  define PWR_VOSSR_ACTVOS_VOS0 (3 << PWR_VOSSR_ACTVOS_SHIFT)

/* Backup Domain Control Register */

#define PWR_BDCR_BREN           (1 << 0) /* Backup RAM Retention in Sby and VBAT modes */
#define PWR_BDCR_MONEN          (1 << 1) /* Backup Domain Voltage and Temperature Monitoring Enable */
#define PWR_BDCR_VBE            (1 << 8) /* Vbat Charging Enable */
#define PWR_BDCR_VBRS           (1 << 9) /* Vbat Charging Resistor Selection: 0=5k, 1=1.5k */

/* Backup Domain Protected Control Register */

#define PWR_DBPCR_DBP           (1 << 0)

/* Backup Domain Status Register */

#define PWR_BDSR_BRRDY          (1 << 16) /* Backup Regulator Ready */
#define PWR_BDSR_VBATL          (1 << 20) /* Vbat Level Monitoring vs Low Threshold */
#define PWR_BDSR_VBATH          (1 << 21) /* Vbat Level Monitoring vs High Threshold */
#define PWR_BDSR_TEMPL          (1 << 22) /* Temperature Level Monitoring vs Low Threshold */
#define PWR_BDSR_TEMPH          (1 << 23) /* Temperature Level Monitoring vs High Threshold */

/* USB Type-C Power Delivery Register */

#define PWR_UCPDR_UCPD_DBDIS    (1 << 0) /* USB-C Power Delivery and Dead Battery Disable */
#define PWR_UCPDR_UCPD_STBY     (1 << 1) /* USB-C and Power Delivery Standby Mode */

/* Supply Configuration Control Register */

#define PWR_SCCR_BYPASS         (1 << 0) /* Power Management Unit Bypass
                                          * 0: Normal Operation: Use Internal Regulator
                                          * 1: Bypassed: Use External Power (Monitoring still active)
                                          */
#define PWR_SCCR_LDOEN          (1 << 8) /* Low Dropout Regulator Enable */
#define PWR_SCCR_SMPSEN         (1 << 9) /* Switch-Mode Power Supply Enable */

/* Voltage Monitor Control Register */

#define PWR_VMCR_PVDE           (1 << 0) /* Programmable Voltage Detector (PVD) Enable */
#define PWR_VMCR_PLS_SHIFT      (1)
#define PWR_VMCR_PLS_MASK       (3 << PWR_VMCR_PLS_SHIFT)
#  define PWR_VMCR_PLS_PVD0     (0 << PWR_VMCR_PLS_SHIFT)
#  define PWR_VMCR_PLS_PVD1     (1 << PWR_VMCR_PLS_SHIFT)
#  define PWR_VMCR_PLS_PVD2     (2 << PWR_VMCR_PLS_SHIFT)
#  define PWR_VMCR_PLS_PVD3     (3 << PWR_VMCR_PLS_SHIFT)
#  define PWR_VMCR_PLS_PVD4     (4 << PWR_VMCR_PLS_SHIFT)
#  define PWR_VMCR_PLS_PVD5     (5 << PWR_VMCR_PLS_SHIFT)
#  define PWR_VMCR_PLS_PVD6     (6 << PWR_VMCR_PLS_SHIFT)
#  define PWR_VMCR_PLS_PVDIN    (7 << PWR_VMCR_PLS_SHIFT)
#define PWR_VMCR_AVDEN          (1 << 8) /* Peripheral Voltage Monitor on Vdda enable */
#define PWR_VMCR_ALS_SHIFT      (9)
#define PWR_VMCR_ALS_MASK       (3 << PWR_VMCR_ALS_SHIFT)
#  define PWR_VMCR_ALS_AVD0     (0 << PWR_VMCR_ALS_SHIFT)
#  define PWR_VMCR_ALS_AVD1     (1 << PWR_VMCR_ALS_SHIFT)
#  define PWR_VMCR_ALS_AVD2     (2 << PWR_VMCR_ALS_SHIFT)
#  define PWR_VMCR_ALS_AVD3     (3 << PWR_VMCR_ALS_SHIFT)

/* USB Supply Control Register */

#define PWR_USBSCR_USB33DEN     (1 << 24) /* Vddusb voltage level detector enable */
#define PWR_USBSCR_USB33SV      (1 << 25) /* Independent USB Supply Valid */

/* Voltage Monitor Status Register */

#define PWR_VMSR_AVDO           (1 << 19) /* Analog Voltage Detector Output on Vdda */
#define PWR_VMSR_VDDIO2RDY      (1 << 20) /* Voltage Detector Output on Vddio2 */
#define PWR_VMSR_PVDO           (1 << 22) /* Programmable Voltage Detect Output */
#define PWR_VMSR_USB33RDY       (1 << 24) /* Vddusb Ready */

/* Wake-up Status Clear Register */

#define PWR_WUSCR_CWUF1         (1 << 0) /* Clear wake-up pin flag for WUF1 */
#define PWR_WUSCR_CWUF2         (1 << 1) /* Clear wake-up pin flag for WUF2 */
#define PWR_WUSCR_CWUF3         (1 << 2) /* Clear wake-up pin flag for WUF3 */
#define PWR_WUSCR_CWUF4         (1 << 3) /* Clear wake-up pin flag for WUF4 */
#define PWR_WUSCR_CWUF5         (1 << 4) /* Clear wake-up pin flag for WUF5 */
#define PWR_WUSCR_CWUF6         (1 << 5) /* Clear wake-up pin flag for WUF6 */
#define PWR_WUSCR_CWUF7         (1 << 6) /* Clear wake-up pin flag for WUF7 */
#define PWR_WUSCR_CWUF8         (1 << 7) /* Clear wake-up pin flag for WUF8 */

/* Wake-up Status Register */

#define PWR_WUSR_WUF1           (1 << 0) /* Wake-up event received for WUF1 */
#define PWR_WUSR_WUF2           (1 << 1) /* Wake-up event received for WUF2 */
#define PWR_WUSR_WUF3           (1 << 2) /* Wake-up event received for WUF3 */
#define PWR_WUSR_WUF4           (1 << 3) /* Wake-up event received for WUF4 */
#define PWR_WUSR_WUF5           (1 << 4) /* Wake-up event received for WUF5 */
#define PWR_WUSR_WUF6           (1 << 5) /* Wake-up event received for WUF6 */
#define PWR_WUSR_WUF7           (1 << 6) /* Wake-up event received for WUF7 */
#define PWR_WUSR_WUF8           (1 << 7) /* Wake-up event received for WUF8 */

/* Wake-up Configuration Register */

#define PWR_WUCR_WUPEN(n)        (1 << (n-1))              /* Enable wake-up pin WUPn. n = 1..8 */
#define PWR_WUCR_WUPP(n)         (1 << (n+7))              /* Wake-up Pin Polarity bit for WUPn */
#define PWR_WUCR_MASK_WUPPUPD(n) (3 << (((n-1) * 2) + 16)) /* Wake-up pin pull configuration */
#define PWR_WUCR_NOPU_WUPPUPD(n) (0 << (((n-1) * 2) + 16))
#define PWR_WUCR_PU_WUPPUPD(n)   (1 << (((n-1) * 2) + 16))
#define PWR_WUCR_PD_WUPPUPD(n)   (2 << (((n-1) * 2) + 16))

/* IO Retention Register */

#define PWR_IORETR_IORETEN       (1 << 0)  /* IO Retention Enable */
#define PWR_IORETR_JTAGIORETEN   (1 << 16) /* IO Retention Enable for JTAG IOs */

/* Power secure configuration register */

#define PWR_SECCFGR_WUP_SEC(n)   (1 << (n-1)) /* WUP(n) Secure Protection */
#define PWR_SECCFGR_RETSEC       (1 << 11)    /*  Retention Secure Protection */
#define PWR_SECCFGR_LPMSEC       (1 << 12)    /*  Low-power Modes Secure Protection */
#define PWR_SECCFGR_SCMSEC       (1 << 13)    /*  Supply Configuration and Monitoring Secure Protection */
#define PWR_SECCFGR_VBSEC        (1 << 14)    /*  Backup Domain Secure Protection */
#define PWR_SECCFGR_VUSBSEC      (1 << 15)    /*  Voltage USB Secure Protection */

/* Power privilege configuration register */

#define PWR_PRIVCFGR_SPRIV        (1 <<  0) /* Bit  0: Power Secure Privilege protection */
#define PWR_PRIVCFGR_NSPRIV       (1 <<  1) /* Bit  0: Power Non-secure Privilege protection */

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5XXX_PWR_H */
