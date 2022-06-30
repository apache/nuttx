/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_PWR_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WB_PWR_CR1_OFFSET    0x0000  /* Power control register 1 */
#define STM32WB_PWR_CR2_OFFSET    0x0004  /* Power control register 2 */
#define STM32WB_PWR_CR3_OFFSET    0x0008  /* Power control register 3 */
#define STM32WB_PWR_CR4_OFFSET    0x000C  /* Power control register 4 */
#define STM32WB_PWR_SR1_OFFSET    0x0010  /* Power status register 1 */
#define STM32WB_PWR_SR2_OFFSET    0x0014  /* Power status register 2 */
#define STM32WB_PWR_SCR_OFFSET    0x0018  /* Power status clear register */
#define STM32WB_PWR_CR5_OFFSET    0x001C  /* Power control register 5 */
#define STM32WB_PWR_PUCRA_OFFSET  0x0020  /* Power Port A pull-up control register */
#define STM32WB_PWR_PDCRA_OFFSET  0x0024  /* Power Port A pull-down control register */
#define STM32WB_PWR_PUCRB_OFFSET  0x0028  /* Power Port B pull-up control register */
#define STM32WB_PWR_PDCRB_OFFSET  0x002C  /* Power Port B pull-down control register */
#define STM32WB_PWR_PUCRC_OFFSET  0x0030  /* Power Port C pull-up control register */
#define STM32WB_PWR_PDCRC_OFFSET  0x0034  /* Power Port C pull-down control register */
#define STM32WB_PWR_PUCRD_OFFSET  0x0038  /* Power Port D pull-up control register */
#define STM32WB_PWR_PDCRD_OFFSET  0x003C  /* Power Port D pull-down control register */
#define STM32WB_PWR_PUCRE_OFFSET  0x0040  /* Power Port E pull-up control register */
#define STM32WB_PWR_PUCRH_OFFSET  0x0058  /* Power Port H pull-up control register */
#define STM32WB_PWR_PDCRH_OFFSET  0x005C  /* Power Port H pull-down control register */
#define STM32WB_PWR_C2CR1_OFFSET  0x0080  /* CPU2 control register 1 */
#define STM32WB_PWR_C2CR3_OFFSET  0x0084  /* CPU2 control register 3 */
#define STM32WB_PWR_EXTSCR_OFFSET 0x0088  /* Extended status and status clear register */

/* Register Addresses *******************************************************/

#define STM32WB_PWR_CR1           (STM32WB_PWR_BASE + STM32WB_PWR_CR1_OFFSET)
#define STM32WB_PWR_CR2           (STM32WB_PWR_BASE + STM32WB_PWR_CR2_OFFSET)
#define STM32WB_PWR_CR3           (STM32WB_PWR_BASE + STM32WB_PWR_CR3_OFFSET)
#define STM32WB_PWR_CR4           (STM32WB_PWR_BASE + STM32WB_PWR_CR4_OFFSET)
#define STM32WB_PWR_SR1           (STM32WB_PWR_BASE + STM32WB_PWR_SR1_OFFSET)
#define STM32WB_PWR_SR2           (STM32WB_PWR_BASE + STM32WB_PWR_SR2_OFFSET)
#define STM32WB_PWR_SCR           (STM32WB_PWR_BASE + STM32WB_PWR_SCR_OFFSET)
#define STM32WB_PWR_CR5           (STM32WB_PWR_BASE + STM32WB_PWR_CR5_OFFSET)
#define STM32WB_PWR_PUCRA         (STM32WB_PWR_BASE + STM32WB_PWR_PUCRA_OFFSET)
#define STM32WB_PWR_PDCRA         (STM32WB_PWR_BASE + STM32WB_PWR_PDCRA_OFFSET)
#define STM32WB_PWR_PUCRB         (STM32WB_PWR_BASE + STM32WB_PWR_PUCRB_OFFSET)
#define STM32WB_PWR_PDCRB         (STM32WB_PWR_BASE + STM32WB_PWR_PDCRB_OFFSET)
#define STM32WB_PWR_PUCRC         (STM32WB_PWR_BASE + STM32WB_PWR_PUCRC_OFFSET)
#define STM32WB_PWR_PDCRC         (STM32WB_PWR_BASE + STM32WB_PWR_PDCRC_OFFSET)
#define STM32WB_PWR_PUCRD         (STM32WB_PWR_BASE + STM32WB_PWR_PUCRD_OFFSET)
#define STM32WB_PWR_PDCRD         (STM32WB_PWR_BASE + STM32WB_PWR_PDCRD_OFFSET)
#define STM32WB_PWR_PUCRE         (STM32WB_PWR_BASE + STM32WB_PWR_PUCRE_OFFSET)
#define STM32WB_PWR_PDCRE         (STM32WB_PWR_BASE + STM32WB_PWR_PDCRE_OFFSET)
#define STM32WB_PWR_PUCRH         (STM32WB_PWR_BASE + STM32WB_PWR_PUCRH_OFFSET)
#define STM32WB_PWR_PDCRH         (STM32WB_PWR_BASE + STM32WB_PWR_PDCRH_OFFSET)
#define STM32WB_PWR_C2CR1         (STM32WB_PWR_BASE + STM32WB_PWR_C2CR1_OFFSET)
#define STM32WB_PWR_C2CR3         (STM32WB_PWR_BASE + STM32WB_PWR_C2CR3_OFFSET)
#define STM32WB_PWR_EXTSCR        (STM32WB_PWR_BASE + STM32WB_PWR_EXTSCR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Power control register 1 */

#define PWR_CR1_LPMS_SHIFT        (0)
#define PWR_CR1_LPMS_MASK         (0x7 << PWR_CR1_LPMS_SHIFT) /* Bits 0-2: Low-power mode selection */
#  define PWR_CR1_LPMS_STOP0      (0x0 << PWR_CR1_LPMS_SHIFT) /* 000: Stop 0 mode */
#  define PWR_CR1_LPMS_STOP1      (0x1 << PWR_CR1_LPMS_SHIFT) /* 001: Stop 1 mode */
#  define PWR_CR1_LPMS_STOP2      (0x2 << PWR_CR1_LPMS_SHIFT) /* 010: Stop 2 mode */
#  define PWR_CR1_LPMS_STANDBY    (0x3 << PWR_CR1_LPMS_SHIFT) /* 011: Standby mode */
#  define PWR_CR1_LPMS_SHUTDOWN   (0x4 << PWR_CR1_LPMS_SHIFT) /* 1xx: Shutdown mode */

#define PWR_CR1_FPDR              (1 << 4)  /* Bit 4: Flash memory power down during LPRun */
#define PWR_CR1_FPDS              (1 << 5)  /* Bit 5: Flash memory power down during LPSleep */
#define PWR_CR1_DBP               (1 << 8)  /* Bit 8: Disable Backup domain write protection */
#define PWR_CR1_VOS_SHIFT         (9)
#  define PWR_CR1_VOS_MASK        (0x3 << PWR_CR1_VOS_SHIFT) /* Bits 9-10: Voltage scaling range selection */
#  define PWR_CR1_VOS_RANGE1      (0x1 << PWR_CR1_VOS_SHIFT) /* 01: Range 1 (1.2V, up to 64 MHz) */
#  define PWR_CR1_VOS_RANGE2      (0x2 << PWR_CR1_VOS_SHIFT) /* 10: Range 2 (1.0V, up to 16 MHz) */

#define PWR_CR1_LPR               (1 << 14) /* Bit 14: Low-power run */

/* Power control register 2 */

#define PWR_CR2_PVDE              (1 << 0)  /* Bit 0: Power voltage detector enable */
#define PWR_CR2_PLS_SHIFT         (1)
#define PWR_CR2_PLS_MASK          (0x7 << PWR_CR2_PLS_SHIFT) /* Bits 1-3: Power voltage detector level selection */
#  define PWR_CR2_PLS_2000mv      (0x0 << PWR_CR2_PLS_SHIFT) /* 000: VPVD0 around 2.0V */
#  define PWR_CR2_PLS_2200mv      (0x1 << PWR_CR2_PLS_SHIFT) /* 001: VPVD1 around 2.2V */
#  define PWR_CR2_PLS_2400mv      (0x2 << PWR_CR2_PLS_SHIFT) /* 010: VPVD2 around 2.4V */
#  define PWR_CR2_PLS_2500mv      (0x3 << PWR_CR2_PLS_SHIFT) /* 011: VPVD3 around 2.5V */
#  define PWR_CR2_PLS_2600mv      (0x4 << PWR_CR2_PLS_SHIFT) /* 100: VPVD4 around 2.6V */
#  define PWR_CR2_PLS_2800mv      (0x5 << PWR_CR2_PLS_SHIFT) /* 101: VPVD5 around 2.8V */
#  define PWR_CR2_PLS_2900mv      (0x6 << PWR_CR2_PLS_SHIFT) /* 110: VPVD6 around 2.9V */
#  define PWR_CR2_PLS_EXT         (0x7 << PWR_CR2_PLS_SHIFT) /* 111: External input analog voltage PVD_IN */

#define PWR_CR2_PVME1             (1 << 4)  /* Bit 4: Peripheral voltage monitoring 1 enable (VDDUSB vs 1.2V) */
#define PWR_CR2_PVME3             (1 << 6)  /* Bit 6: Peripheral voltage monitoring 3 enable (VDDA vs 1.62V) */
#define PWR_CR2_PVME4             (1 << 7)  /* Bit 7: Peripheral voltage monitoring 4 enable (VDDA vs 2.2V) */
#define PWR_CR2_USV               (1 << 10) /* Bit 10: VDDUSB supply validate */

/* Power control register 3 */

#define PWR_CR3_EWUP1             (1 << 0)  /* Bit 0: Enable Wakeup pin WKUP1 */
#define PWR_CR3_EWUP2             (1 << 1)  /* Bit 1: Enable Wakeup pin WKUP2 */
#define PWR_CR3_EWUP3             (1 << 2)  /* Bit 2: Enable Wakeup pin WKUP3 */
#define PWR_CR3_EWUP4             (1 << 3)  /* Bit 3: Enable Wakeup pin WKUP4 */
#define PWR_CR3_EWUP5             (1 << 4)  /* Bit 4: Enable Wakeup pin WKUP5 */
#define PWR_CR3_EBORHSMPSFB       (1 << 8)  /* Bit 8: Enable BORH and SMPS forced in Bypass interrupts */
#define PWR_CR3_RRS               (1 << 9)  /* Bit 9: SRAM2a retention in Standby mode */
#define PWR_CR3_APC               (1 << 10) /* Bit 10: Apply pull-up and pull-down configuration */
#define PWR_CR3_ECRPE             (1 << 11) /* Bit 11: Enable critical radio phase end of activity interrupt */
#define PWR_CR3_EBLEA             (1 << 12) /* Bit 12: Enable BLE end of activity interrupt */
#define PWR_CR3_E802A             (1 << 13) /* Bit 13: Enable 802.15.4 end of activity interrupt */
#define PWR_CR3_EC2H              (1 << 14) /* Bit 14: Enable CPU2 Hold interrupt */
#define PWR_CR3_EIWUL             (1 << 15) /* Bit 15: Enable internal wakeup line */

/* Power control register 4 */

#define PWR_CR4_WP1               (1 << 0)  /* Bit 0: Wakeup pin WKUP1 polarity */
#  define PWR_CR4_WP1_HI          (0 << 0)  /* 0: Detection on high level (rising edge) */
#  define PWR_CR4_WP1_LOW         (1 << 0)  /* 1: Detection on low level (failing edge) */

#define PWR_CR4_WP2               (1 << 1)  /* Bit 1: Wakeup pin WKUP2 polarity */
#  define PWR_CR4_WP2_HI          (0 << 1)  /* 0: Detection on high level (rising edge) */
#  define PWR_CR4_WP2_LOW         (1 << 1)  /* 1: Detection on low level (failing edge) */

#define PWR_CR4_WP3               (1 << 2)  /* Bit 2: Wakeup pin WKUP3 polarity */
#  define PWR_CR4_WP3_HI          (0 << 2)  /* 0: Detection on high level (rising edge) */
#  define PWR_CR4_WP3_LOW         (1 << 2)  /* 1: Detection on low level (failing edge) */

#define PWR_CR4_WP4               (1 << 3)  /* Bit 3: Wakeup pin WKUP4 polarity */
#  define PWR_CR4_WP4_HI          (0 << 3)  /* 0: Detection on high level (rising edge) */
#  define PWR_CR4_WP4_LOW         (1 << 3)  /* 1: Detection on low level (failing edge) */

#define PWR_CR4_WP5               (1 << 4)  /* Bit 4: Wakeup pin WKUP5 polarity */
#  define PWR_CR4_WP5_HI          (0 << 4)  /* 0: Detection on high level (rising edge) */
#  define PWR_CR4_WP5_LOW         (1 << 4)  /* 1: Detection on low level (failing edge) */

#define PWR_CR4_VBE               (1 << 8)  /* Bit 8: Vbat battery charging enable */
#define PWR_CR4_VBRS              (1 << 9)  /* Bit 9: Vbat battery charging resistor selection */
#  define PWR_CR4_VBRS_5k         (0 << 9)  /* 0: 5k  resistor */
#  define PWR_CR4_VBRS_1k5        (1 << 9)  /* 1: 1k5 resistor */

#define PWR_CR4_C2BOOT            (1 << 15) /* Bit 15: Boot CPU2 after reset or wakeup from Stop or Standby */

/* Power status register 1 */

#define PWR_SR1_WUF1              (1 << 0)  /* Bit 0: Wakeup flag 1 */
#define PWR_SR1_WUF2              (1 << 1)  /* Bit 1: Wakeup flag 2 */
#define PWR_SR1_WUF3              (1 << 2)  /* Bit 2: Wakeup flag 3 */
#define PWR_SR1_WUF4              (1 << 3)  /* Bit 3: Wakeup flag 4 */
#define PWR_SR1_WUF5              (1 << 4)  /* Bit 4: Wakeup flag 5 */
#define PWR_SR1_SBF               (1 << 8)  /* Bit 8: Standby flag */
#define PWR_SR1_WUFI              (1 << 15) /* Bit 15: Wakeup internal flag */

/* Power status register 2 */

#define PWR_SR2_SMPSBF            (1 <<  0) /* Bit 0: SMPS bypass mode flag */
#define PWR_SR2_SMPSF             (1 <<  1) /* Bit 1: SMPS ready flag */
#define PWR_SR2_REGLPS            (1 <<  8) /* Bit 8: Low power regulator started */
#define PWR_SR2_REGLPF            (1 <<  9) /* Bit 9: Low power regulator flag */
#  define PWR_SR2_REGLPF_MR       (0 <<  9) /* 0: Regulator is ready in main mode */
#  define PWR_SR2_REGLPF_LPR      (1 <<  9) /* 1: Regulator is in low-power mode */

#define PWR_SR2_VOSF              (1 << 10) /* Bit 10: Voltage scaling flag */
#  define PWR_SR2_VOSF_RDY        (0 << 10) /* 0: Regulator is ready in the selected voltage range */
#  define PWR_SR2_VOSF_NRDY       (1 << 10) /* 1: Regulator voltage is changing */

#define PWR_SR2_PVDO              (1 << 11) /* Bit 11: Power voltage detector output */
#  define PWR_SR2_PVDO_ABV        (0 << 11) /* 0: Voltage is above the selected PVD threshold */
#  define PWR_SR2_PVDO_BLW        (1 << 11) /* 1: Voltage is below the selected PVD threshold */

#define PWR_SR2_PVMO1             (1 << 12) /* Bit 12: VDDUSB voltage monitoring output (VDDUSB vs PVM1) */
#  define PWR_SR2_PVMO1_ABV       (0 << 12) /* 0: VDDUSB voltage is above PVM1 threshold (~ 1.2V) */
#  define PWR_SR2_PVMO1_BLW       (1 << 12) /* 1: VDDUSB voltage is below PVM1 threshold (~ 1.2V) */

#define PWR_SR2_PVMO3             (1 << 14) /* Bit 14: VDDA voltage monitoring output (VDDA vs PVM3) */
#  define PWR_SR2_PVMO3_ABV       (0 << 14) /* 0: VDDA voltage is above PVM3 threshold (~ 1.62V) */
#  define PWR_SR2_PVMO3_BLW       (1 << 14) /* 1: VDDA voltage is below PVM3 threshold (~ 1.62V) */

/* Power status clear register */

#define PWR_SCR_CWUF1             (1 << 0)  /* Bit 0: Clear wakeup flag 1 */
#define PWR_SCR_CWUF2             (1 << 1)  /* Bit 1: Clear wakeup flag 2 */
#define PWR_SCR_CWUF3             (1 << 2)  /* Bit 2: Clear wakeup flag 3 */
#define PWR_SCR_CWUF4             (1 << 3)  /* Bit 3: Clear wakeup flag 4 */
#define PWR_SCR_CWUF5             (1 << 4)  /* Bit 4: Clear wakeup flag 5 */
#define PWR_SCR_CSMPSFBF          (1 << 7)  /* Bit 7: Clear SMPS forced in Bypass interrupt flag */
#define PWR_SCR_CBORHF            (1 << 8)  /* Bit 8: Clear BORH interrupt flag */
#define PWR_SCR_CBLEWUF           (1 << 9)  /* Bit 9: Clear BLE wakeup interrupt flag */
#define PWR_SCR_C802WUF           (1 << 10) /* Bit 10: Clear 802.15.4 wakeup interrupt flag */
#define PWR_SCR_CCRPEF            (1 << 11) /* Bit 11: Critical radio phase end of activity interrupt flag */
#define PWR_SCR_CFBLEAF           (1 << 12) /* Bit 12: Clear BLE end of activity interrupt flag */
#define PWR_SCR_C802AF            (1 << 13) /* Bit 13: Clear 802.15.4 end of activity interrupt flag */
#define PWR_SCR_CC2HF             (1 << 14) /* Bit 14: Clear CPU2 Hold interrupt flag */

/* Power control register 5 */

#define PWR_CR5_SMPSVOS_SHIFT     (0)
#define PWR_CR5_SMPSVOS_MASK      (0xf << PWR_CR5_SMPSVOS_SHIFT) /* Bits 0-3: SMPS voltage output scaling */
#define PWR_CR5_SMPSSC_SHIFT      (4)
#define PWR_CR5_SMPSSC_MASK       (0x7 << PWR_CR5_SMPSSC_SHIFT)  /* Bits 4-6: SMPS supply startup current */

#define PWR_CR5_BORHC             (1 << 8)  /* Bit 8: BORH configuration selection */
#  define PWR_CR5_BORHC_SYSRST    (0 << 8)  /* 0: BORH generates a system reset */
#  define PWR_CR5_BORHC_SMPSFB    (1 << 8)  /* 1: BORH forces SMPS bypass mode */

#define PWR_CR5_SMPSEN            (1 << 15) /* Bit 15: SMPS enable */

/* Power Port A Pull Up Control Register (PUCRA) */

#define PWR_PUCRA_PU(y)           (1 << (y)) /* Port A Pull Up control bit, y = 0..15 */

/* Power Port A Pull Down Control Register (PDCRA) */

#define PWR_PDCRA_PD(y)           (1 << (y)) /* Port A Pull Down control bit, y = 0..15 */

/* Power Port B Pull Up Control Register (PUCRB) */

#define PWR_PUCRB_PU(y)           (1 << (y)) /* Port B Pull Up control bit, y = 0..15 */

/* Power Port B Pull Down Control Register (PDCRB) */

#define PWR_PDCRB_PD(y)           (1 << (y)) /* Port B Pull Down control bit, y = 0..15 */

/* Power Port C Pull Up Control Register (PUCRC) */

#define PWR_PUCRC_PU(y)           (1 << (y)) /* Port C Pull Up control bit, y = 0..15 */

/* Power Port C Pull Down Control Register (PDCRC) */

#define PWR_PDCRC_PD(y)           (1 << (y)) /* Port C Pull Down control bit, y = 0..15 */

/* Power Port D Pull Up Control Register (PUCRD) */

#define PWR_PUCRD_PU(y)           (1 << (y)) /* Port D Pull Up control bit, y = 0..15 */

/* Power Port D Pull Down Control Register (PDCRD) */

#define PWR_PDCRD_PD(y)           (1 << (y)) /* Port D Pull Down control bit, y = 0..15 */

/* Power Port E Pull Up Control Register (PUCRE) */

#define PWR_PUCRE_PU(y)           (1 << (y)) /* Port E Pull Up control bit, y = 0..4 */

/* Power Port E Pull Down Control Register (PDCRE) */

#define PWR_PDCRE_PD(y)           (1 << (y)) /* Port E Pull Down control bit, y = 0..4 */

/* Power Port H Pull Up Control Register (PUCRH) */

#define PWR_PUCRH_PU(y)           (1 << (y)) /* Port H Pull Up control bit, y = 0..1 or 3 */

/* Power Port H Pull Down Control Register (PDCRH) */

#define PWR_PDCRH_PD(y)           (1 << (y)) /* Port H Pull Down control bit, y = 0..1 or 3 */

/* CPU2 control register 1 */

#define PWR_C2CR1_LPMS_SHIFT      (0)
#define PWR_C2CR1_LPMS_MASK       (0x7 << PWR_C2CR1_LPMS_SHIFT) /* Bits 0-2: CPU2 Low-power mode selection */
#  define PWR_C2CR1_LPMS_STOP0    (0x0 << PWR_C2CR1_LPMS_SHIFT) /* 000: Stop 0 mode */
#  define PWR_C2CR1_LPMS_STOP1    (0x1 << PWR_C2CR1_LPMS_SHIFT) /* 001: Stop 1 mode */
#  define PWR_C2CR1_LPMS_STOP2    (0x2 << PWR_C2CR1_LPMS_SHIFT) /* 010: Stop 2 mode */
#  define PWR_C2CR1_LPMS_STANDBY  (0x3 << PWR_C2CR1_LPMS_SHIFT) /* 011: Standby mode */
#  define PWR_C2CR1_LPMS_SHUTDOWN (0x4 << PWR_C2CR1_LPMS_SHIFT) /* 1xx: Shutdown mode */

#define PWR_C2CR1_FPDR            (1 << 4)  /* Bit 4: CPU2 Flash memory power down during LPRun */
#define PWR_C2CR1_FPDS            (1 << 5)  /* Bit 5: CPU2 Flash memory power down during LPSleep */
#define PWR_C2CR1_BLEEWKUP        (1 << 14) /* Bit 14: BLE external wakeup */
#define PWR_C2CR1_800EWKUP        (1 << 15) /* Bit 15: 802.15.4 external wakeup */

/* CPU2 control register 3 */

#define PWR_C2CR3_EWUP1           (1 << 0)  /* Bit 0: CPU2 Enable Wakeup pin WKUP1 */
#define PWR_C2CR3_EWUP2           (1 << 1)  /* Bit 1: CPU2 Enable Wakeup pin WKUP2 */
#define PWR_C2CR3_EWUP3           (1 << 2)  /* Bit 2: CPU2 Enable Wakeup pin WKUP3 */
#define PWR_C2CR3_EWUP4           (1 << 3)  /* Bit 3: CPU2 Enable Wakeup pin WKUP4 */
#define PWR_C2CR3_EWUP5           (1 << 4)  /* Bit 4: CPU2 Enable Wakeup pin WKUP5 */
#define PWR_C2CR3_EBLEWUP         (1 << 9)  /* Bit 9: CPU2 Enable BLE host wakeup interrupt */
#define PWR_C2CR3_E802WUP         (1 << 10) /* Bit 10: CPU2 Enable 802.15.4 host wakeup interrupt */
#define PWR_C2CR3_APC             (1 << 12) /* Bit 12: CPU2 Apply pull-up and pull-down configuration */
#define PWR_C2CR3_EIWUL           (1 << 15) /* Bit 15: CPU2 Enable internal wakeup line */

/* Extended status and status clear register */

#define PWR_EXTSCR_C1CSSF         (1 << 0)  /* Bit 0: Clear CPU1 Stop Standby flags */
#define PWR_EXTSCR_C2CSSF         (1 << 1)  /* Bit 1: Clear CPU2 Stop Standby flags */
#define PWR_EXTSCR_CCRPF          (1 << 2)  /* Bit 2: Clear critical radio system phase */
#define PWR_EXTSCR_C1SBF          (1 << 8)  /* Bit 8: System Standby flag for CPU1 */
#define PWR_EXTSCR_C1STOPF        (1 << 9)  /* Bit 9: System Stop flag for CPU1 */
#define PWR_EXTSCR_C2SBF          (1 << 10) /* Bit 10: System Standby flag for CPU2 */
#define PWR_EXTSCR_C2STOPF        (1 << 11) /* Bit 11: System Stop flag for CPU2 */
#define PWR_EXTSCR_CRPF           (1 << 13) /* Bit 13: Critical radio system phase flag */
#define PWR_EXTSCR_C1DS           (1 << 14) /* Bit 14: CPU1 deepsleep mode */
#define PWR_EXTSCR_C2DS           (1 << 15) /* Bit 15: CPU2 deepsleep mode */

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_PWR_H */
