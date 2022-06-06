/****************************************************************************
 * arch/arm/src/stm32wl5/hardware/stm32wl5_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_PWR_H
#define __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WL5_PWR_CR1_OFFSET         0x0000  /* Power control register 1 */
#define STM32WL5_PWR_CR2_OFFSET         0x0004  /* Power control register 2 */
#define STM32WL5_PWR_CR3_OFFSET         0x0008  /* Power control register 3 */
#define STM32WL5_PWR_CR4_OFFSET         0x000C  /* Power control register 4 */
#define STM32WL5_PWR_SR1_OFFSET         0x0010  /* Power status register 1 */
#define STM32WL5_PWR_SR2_OFFSET         0x0014  /* Power status register 2 */
#define STM32WL5_PWR_SCR_OFFSET         0x0018  /* Power status clear register */
#define STM32WL5_PWR_CR5_OFFSET         0x001C  /* Power control register 5 */
#define STM32WL5_PWR_PUCRA_OFFSET       0x0020  /* Power Port A pull-up control register */
#define STM32WL5_PWR_PDCRA_OFFSET       0x0024  /* Power Port A pull-down control register */
#define STM32WL5_PWR_PUCRB_OFFSET       0x0028  /* Power Port B pull-up control register */
#define STM32WL5_PWR_PDCRB_OFFSET       0x002C  /* Power Port B pull-down control register */
#define STM32WL5_PWR_PUCRC_OFFSET       0x0030  /* Power Port C pull-up control register */
#define STM32WL5_PWR_PDCRC_OFFSET       0x0034  /* Power Port C pull-down control register */
#define STM32WL5_PWR_PUCRH_OFFSET       0x0058  /* Power Port H pull-up control register */
#define STM32WL5_PWR_PDCRH_OFFSET       0x005C  /* Power Port H pull-down control register */
#define STM32WL5_PWR_C2CR1_OFFSET       0x0080  /* Power control register 1 for cpu2 */
#define STM32WL5_PWR_C2CR3_OFFSET       0x0084  /* Power control register 3 for cpu2 */
#define STM32WL5_PWR_EXTSCR_OFFSET      0x0088  /* Power extended status */
#define STM32WL5_PWR_SECCFGR_OFFSET     0x0088  /* Power security configuration */
#define STM32WL5_PWR_SUBGHZSPICR_OFFSET 0x0088  /* Power sub-ghz spi radio control */
#define STM32WL5_PWR_RSSCMDR_OFFSET     0x0088  /* Power RSS command */

/* Register Addresses *******************************************************/

#define STM32WL5_PWR_CR1         (STM32WL5_PWR_BASE+STM32WL5_PWR_CR1_OFFSET)
#define STM32WL5_PWR_CR2         (STM32WL5_PWR_BASE+STM32WL5_PWR_CR2_OFFSET)
#define STM32WL5_PWR_CR3         (STM32WL5_PWR_BASE+STM32WL5_PWR_CR3_OFFSET)
#define STM32WL5_PWR_CR4         (STM32WL5_PWR_BASE+STM32WL5_PWR_CR4_OFFSET)
#define STM32WL5_PWR_SR1         (STM32WL5_PWR_BASE+STM32WL5_PWR_SR1_OFFSET)
#define STM32WL5_PWR_SR2         (STM32WL5_PWR_BASE+STM32WL5_PWR_SR2_OFFSET)
#define STM32WL5_PWR_SCR         (STM32WL5_PWR_BASE+STM32WL5_PWR_SCR_OFFSET)
#define STM32WL5_PWR_CR5         (STM32WL5_PWR_BASE+STM32WL5_PWR_CR5_OFFSET)
#define STM32WL5_PWR_PUCRA       (STM32WL5_PWR_BASE+STM32WL5_PWR_PUCRA_OFFSET)
#define STM32WL5_PWR_PDCRA       (STM32WL5_PWR_BASE+STM32WL5_PWR_PDCRA_OFFSET)
#define STM32WL5_PWR_PUCRB       (STM32WL5_PWR_BASE+STM32WL5_PWR_PUCRB_OFFSET)
#define STM32WL5_PWR_PDCRB       (STM32WL5_PWR_BASE+STM32WL5_PWR_PDCRB_OFFSET)
#define STM32WL5_PWR_PUCRC       (STM32WL5_PWR_BASE+STM32WL5_PWR_PUCRC_OFFSET)
#define STM32WL5_PWR_PDCRC       (STM32WL5_PWR_BASE+STM32WL5_PWR_PDCRC_OFFSET)
#define STM32WL5_PWR_PUCRH       (STM32WL5_PWR_BASE+STM32WL5_PWR_PUCRH_OFFSET)
#define STM32WL5_PWR_PDCRH       (STM32WL5_PWR_BASE+STM32WL5_PWR_PDCRH_OFFSET)
#define STM32WL5_PWR_C2CR1       (STM32WL5_PWR_BASE+STM32WL5_PWR_C2CR1_OFFSET)
#define STM32WL5_PWR_C2CR3       (STM32WL5_PWR_BASE+STM32WL5_PWR_C2CR3_OFFSET)
#define STM32WL5_PWR_EXTSCR      (STM32WL5_PWR_BASE+STM32WL5_PWR_EXTSCR_OFFSET)
#define STM32WL5_PWR_SECCFGR     (STM32WL5_PWR_BASE+STM32WL5_PWR_SECCFGR_OFFSET)
#define STM32WL5_PWR_SUBGHZSPICR (STM32WL5_PWR_BASE+STM32WL5_PWR_SUBGHZSPICR_OFFSET)
#define STM32WL5_PWR_RSSCMDR     (STM32WL5_PWR_BASE+STM32WL5_PWR_RSSCMDR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Power control register 1 */

#define PWR_CR1_LPMS_SHIFT       0
#define PWR_CR1_LPMS_MASK        (7 << PWR_CR1_LPMS_SHIFT) /* Bits 0-2: Low-power mode selection */
#  define PWR_CR1_LPMS_STOP1MR   (0 << PWR_CR1_LPMS_SHIFT) /* Stop 1 mode with main regulator (MR) */
#  define PWR_CR1_LPMS_STOP1LPR  (1 << PWR_CR1_LPMS_SHIFT) /* Stop 1 mode with low-power regulator (LPR) */
#  define PWR_CR1_LPMS_STOP2     (2 << PWR_CR1_LPMS_SHIFT) /* 010: Stop 2 mode */
#  define PWR_CR1_LPMS_STANDBY   (3 << PWR_CR1_LPMS_SHIFT) /* 011: Standby mode */
#  define PWR_CR1_LPMS_SHUTDOWN  (4 << PWR_CR1_LPMS_SHIFT) /* 1xx: Shutdown mode */

#define PWR_CR1_SUBGHZSPIPINSEL  (1 <<  3) /* Spi nss source select for radio */
#define PWR_CR1_FPDR             (1 <<  4) /* Flash in power-down mode when system in lprun mode */
#define PWR_CR1_FPDS             (1 <<  5) /* Flash in power-down mode when system in lpsleep mode (cpu1 only) */
                                           /* Bit 6-7: Reserved */
#define PWR_CR1_DBP              (1 <<  8) /* Disable Backup domain write protection */

#define PWR_CR1_VOS_SHIFT        9
#define PWR_CR1_VOS_MASK         (3 << PWR_CR1_VOS_SHIFT) /* Bits 9-10: Voltage scaling range selection */
#define PWR_CR1_VOS_RANGE1       (1 << PWR_CR1_VOS_SHIFT) /* 01: Range 1 */
#define PWR_CR1_VOS_RANGE2       (2 << PWR_CR1_VOS_SHIFT) /* 10: Range 2 */

#define PWR_CR1_LPR              (1 << 14) /* Bit 14: Low-power run */

/* Power control register 2 */

#define PWR_CR2_PVDE             (1 <<  0) /* Bit  0: Power voltage detector enable */
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

                                           /* Bit  4-5: Reserved */
#define PWR_CR2_PVME3            (1 <<  6) /* Bit  6: Peripheral voltage monitoring 3 enable (VDDA vs 1.62V) */

/* Power control register 3 */

#define PWR_CR3_EWUP1            (1 <<  0) /* Bit  0: Enable Wakeup pin WKUP1 */
#define PWR_CR3_EWUP2            (1 <<  1) /* Bit  1: Enable Wakeup pin WKUP2 */
#define PWR_CR3_EWUP3            (1 <<  2) /* Bit  2: Enable Wakeup pin WKUP3 */
#define PWR_CR3_ULPEN            (1 <<  7) /* Bit  7: Ultra low power mode enable */
#define PWR_CR3_EWPVD            (1 <<  8) /* Bit  8: PVD enabled while radio is active */
#define PWR_CR3_RRS              (1 <<  9) /* Bit  9: SRAM2 retention in Standby mode */
#define PWR_CR3_APC              (1 << 10) /* Bit 10: Apply pull-up and pull-down configuration */
#define PWR_CR3_EWRFBUSY         (1 << 11) /* Bit 11: Radio busy wakeup from standby for cpu1 */
#define PWR_CR3_EWRFIRQ          (1 << 13) /* Bit 13: Radio IRQ[2:0] wakeup for cpu1 */
#define PWR_CR3_EC2H             (1 << 14) /* Bit 14: Enable cpu2 interrupt to cpu1 */
#define PWR_CR3_EIWUL            (1 << 15) /* Bit 15: Enable internal wakeup line for cpu1 */

/* Power control register 4 */

#define PWR_CR4_WP1              (1 <<  0) /* Bit  0: Wakeup pin WKUP1 polarity */
#define PWR_CR4_WP2              (1 <<  1) /* Bit  1: Wakeup pin WKUP2 polarity */
#define PWR_CR4_WP3              (1 <<  2) /* Bit  2: Wakeup pin WKUP3 polarity */
#define PWR_CR4_VBE              (1 <<  8) /* Bit  8: Vbat battery charging enable */
#define PWR_CR4_VBRS             (1 <<  9) /* Bit  9: Vbat battery charging resistor selection */
#define PWR_CR4_WRFBUSYP         (1 << 11) /* Bit 11: Radio event detection on failling edge */
#define PWR_CR4_C2BOOT           (1 << 15) /* Bit 15: Boot cpu2 after reset if event is available */

#  define PWR_CR4_VBRS_5k        0            /*     0: 5k  resistor */
#  define PWR_CR4_VBRS_1k5       PWR_CR4_VBRS /*     1: 1k5 resistor */

/* Power control register 5 */

#define PWR_CR5_RFEOLEN          (1 << 14) /* Bit 14: Enable radio end-of-life detector */
#define PWR_CR5_SMPSEN           (1 << 15) /* Bit 15: Enable SMPS step-down converter */

/* Power status register 1 */

#define PWR_SR1_WUF1             (1 <<  0) /* Bit  0: Wakeup flag 1 */
#define PWR_SR1_WUF2             (1 <<  1) /* Bit  1: Wakeup flag 2 */
#define PWR_SR1_WUF3             (1 <<  2) /* Bit  2: Wakeup flag 3 */
#define PWR_SR1_WPVDF            (1 <<  8) /* Bit  8: Wakup PVD flag */
#define PWR_SR1_WRFBUSYF         (1 << 11) /* Bit 11: Radio busy wakup flag */
#define PWR_SR1_C2HF             (1 << 14) /* Bit 14: Cpu2 hold interrupt flag */
#define PWR_SR1_WUFI             (1 << 15) /* Bit 15: Wakeup internal flag */

/* Power status register 2 */

#define PVR_SR2_C2BOOTS          (1 <<  0) /* Bit  0: CPU2 booted from a C2BOOT request  */
#define PVR_SR2_RFBUSYS          (1 <<  1) /* Bit  1: Radio busy signal high (radio is busy) */
#define PVR_SR2_RFBUSYMS         (1 <<  2) /* Bit  2: Radio busy masked signal high */
#define PVR_SR2_SMPSRDY          (1 <<  3) /* Bit  3: SMPS step-down converter is ready */
#define PVR_SR2_LDORDY           (1 <<  4) /* Bit  4: LDO is ready */
#define PVR_SR2_RFEOLF           (1 <<  5) /* Bit  5: Supply voltage below radio end-of-life low level */
#define PVR_SR2_REGMRS           (1 <<  6) /* Bit  6: Main regulator supplied through LDO or SMPS */
#define PVR_SR2_FLASHRDY         (1 <<  7) /* Bit  7: Flash memory is ready */
#define PWR_SR2_REGLPS           (1 <<  8) /* Bit  8: Low power regulator started */
#define PWR_SR2_REGLPF           (1 <<  9) /* Bit  9: Low power regulator flag */
#define PWR_SR2_VOSF             (1 << 10) /* Bit 10: Voltage scaling flag */
#define PWR_SR2_PVDO             (1 << 11) /* Bit 11: Power voltage detector output */
#define PWR_SR2_PVMO3            (1 << 14) /* Bit 14: Peripheral voltage monitoring output 3 (VDDA vs 1.62V) */

/* Power status clear register */

#define PWR_SCR_CWUF1            (1 <<  0) /* Bit  0: Clear wakeup flag 1 */
#define PWR_SCR_CWUF2            (1 <<  1) /* Bit  1: Clear wakeup flag 2 */
#define PWR_SCR_CWUF3            (1 <<  2) /* Bit  2: Clear wakeup flag 3 */
#define PWRC_SCR_CWPVDF          (1 <<  8) /* Bit  8: Clear wakeup PVD */
#define PWRC_SCR_CWRFBUSYF       (1 << 11) /* Bit 11: Clear wakeup radio */
#define PWRC_SCR_CC2HF           (1 << 14) /* Bit 14: Clear wakeup cpu2 */

/* Port X pull-up/down registers have one bit per port line,
 * with a few exceptions
 */

#endif /* __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_PWR_H */
