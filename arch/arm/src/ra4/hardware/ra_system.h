/****************************************************************************
 * arch/arm/src/ra4/hardware/ra_system.h
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
#ifndef __ARCH_ARM_SRC_RA4M1_HARDWARE_RA4M1_SYSTEM_H
#define __ARCH_ARM_SRC_RA4M1_HARDWARE_RA4M1_SYSTEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/ra4/chip.h>
#include "ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_SYSTEM_VBTCR1_OFFSET            0x041f  /* VBATT Control Register 1 (8-bits) */
#define R_SYSTEM_VBTCR2_OFFSET            0x04b0  /* VBATT Control Register 2 (8-bits) */
#define R_SYSTEM_VBTSR_OFFSET             0x04b1  /* VBATT Status Register (8-bits) */
#define R_SYSTEM_VBTCMPCR_OFFSET          0x04b2  /* VBATT Comparator Control Register (8-bits) */
#define R_SYSTEM_VBTLVDICR_OFFSET         0x04b4  /* VBATT Pin Low Voltage Detect Interrupt Control Register (8-bits) */
#define R_SYSTEM_VBTWCTLR_OFFSET          0x04b6  /* VBATT Wakeup Function Control Register (8-bits) */
#define R_SYSTEM_VBTWCH0OTSR_OFFSET       0x04b8  /* VBATT Wakeup I/O 0 Output Trigger Select Register (8-bits) */
#define R_SYSTEM_VBTWCH1OTSR_OFFSET       0x04b9  /* VBATT Wakeup I/O 1 Output Trigger Select Register (8-bits) */
#define R_SYSTEM_VBTWCH2OTSR_OFFSET       0x04ba  /* VBATT Wakeup I/O 2 Output Trigger Select Register (8-bits) */
#define R_SYSTEM_VBTICTLR_OFFSET          0x04bb  /* VBATT Input Control Register (8-bits) */
#define R_SYSTEM_VBTOCTLR_OFFSET          0x04bc  /* VBATT Output Control Register (8-bits) */
#define R_SYSTEM_VBTWTER_OFFSET           0x04bd  /* VBATT Wakeup Trigger Source Enable Register (8-bits) */
#define R_SYSTEM_VBTWEGR_OFFSET           0x04be  /* VBATT Wakeup Trigger Source Edge Register (8-bits) */
#define R_SYSTEM_VBTWFR_OFFSET            0x04bf  /* VBATT Wakeup Trigger Source Flag Register (8-bits) */
#define R_SYSTEM_VBTBKR_OFFSET            0x0500  /* VBATT Backup Register (8-bits) */
#define R_SYSTEM_SCKDIVCR_OFFSET          0x0020  /* System Clock Division Control Register (32-bits) */
#define R_SYSTEM_SCKSCR_OFFSET            0x0026  /* System Clock Source Control Register (8-bits) */
#define R_SYSTEM_PLLCR_OFFSET             0x002a  /* PLL Control Register (8-bits) */
#define R_SYSTEM_PLLCCR2_OFFSET           0x002b  /* PLL Clock Control Register 2 (8-bits) */
#define R_SYSTEM_MEMWAIT_OFFSET           0x0031  /* Memory Wait Cycle Control Register (8-bits) */
#define R_SYSTEM_MOSCCR_OFFSET            0x0032  /* Main Clock Oscillator Control Register (8-bits) */
#define R_SYSTEM_HOCOCR_OFFSET            0x0036  /* High-Speed On-Chip Oscillator Control Register (8-bits) */
#define R_SYSTEM_MOCOCR_OFFSET            0x0038  /* Middle-Speed On-Chip Oscillator Control Register (8-bits) */
#define R_SYSTEM_SOSCCR_OFFSET            0x0480  /* Sub-Clock Oscillator Control Register (8-bits) */
#define R_SYSTEM_LOCOCR_OFFSET            0x0490  /* Low-Speed On-Chip Oscillator Control Register (8-bits) */
#define R_SYSTEM_OSCSF_OFFSET             0x003c  /* Oscillation Stabilization Flag Register (8-bits) */
#define R_SYSTEM_CKOCR_OFFSET             0x003e  /* Clock Out Control Register (8-bits) */
#define R_SYSTEM_TRCKCR_OFFSET            0x003f  /* Trace Clock Control Register (8-bits) */
#define R_SYSTEM_OSTDCR_OFFSET            0x0040  /* Oscillation Stop Detection Control Register (8-bits) */
#define R_SYSTEM_OSTDSR_OFFSET            0x0041  /* Oscillation Stop Detection Status Register (8-bits) */
#define R_SYSTEM_SLCDSCKCR_OFFSET         0x0050  /* Segment LCD Source Clock Control Register (8-bits) */
#define R_SYSTEM_MOCOUTCR_OFFSET          0x0061  /* MOCO User Trimming Control Register (8-bits) */
#define R_SYSTEM_HOCOUTCR_OFFSET          0x0062  /* HOCO User Trimming Control Register (8-bits) */
#define R_SYSTEM_MOSCWTCR_OFFSET          0x00a2  /* Main Clock Oscillator Wait Control Register (8-bits) */
#define R_SYSTEM_HOCOWTCR_OFFSET          0x00a5  /* High-Speed On-Chip Oscillator Wait Control Register (8-bits) */
#define R_SYSTEM_USBCKCR_OFFSET           0x00d0  /* USB Clock Control Register (8-bits) */
#define R_SYSTEM_MOMCR_OFFSET             0x0413  /* Main Clock Oscillator Mode Oscillation Control Register (8-bits) */
#define R_SYSTEM_SOMCR_OFFSET             0x0481  /* Sub-Clock Oscillator Mode Control Register (8-bits) */
#define R_SYSTEM_LOCOUTCR_OFFSET          0x0492  /* LOCO User Trimming Control Register (8-bits) */
#define R_SYSTEM_SBYCR_OFFSET             0x000c  /* Standby Control Register (16-bits) */
#define R_SYSTEM_MSTPCRA_OFFSET           0x001c  /* Module Stop Control Register A (32-bits) */
#define R_SYSTEM_SNZCR_OFFSET             0x0092  /* Snooze Control Register (8-bits) */
#define R_SYSTEM_SNZEDCR_OFFSET           0x0094  /* Snooze End Control Register (8-bits) */
#define R_SYSTEM_SNZREQCR_OFFSET          0x0098  /* Snooze Request Control Register (32-bits) */
#define R_SYSTEM_FLSTOP_OFFSET            0x009e  /* Flash Operation Control Register (8-bits) */
#define R_SYSTEM_OPCCR_OFFSET             0x00a0  /* Operating Power Control Register (8-bits) */
#define R_SYSTEM_SOPCCR_OFFSET            0x00aa  /* Sub Operating Power Control Register (8-bits) */
#define R_SYSTEM_SYOCDCR_OFFSET           0x040e  /* System Control OCD Control Register (8-bits) */
#define R_SYSTEM_LVCMPCR_OFFSET           0x0417  /* Voltage Monitor Circuit Control Register (8-bits) */
#define R_SYSTEM_LVDLVLR_OFFSET           0x0418  /* Voltage Detection Level Select Register (8-bits) */
#define R_SYSTEM_LVDCR0_OFFSET            0x041a  /* Voltage Monitor Circuit Control Register 0 (8-bits) */
#define R_SYSTEM_LVDCR1_OFFSET            0x00e0  /* Voltage Monitor Circuit Control Register 1 (8-bits) */
#define R_SYSTEM_LVDSR_OFFSET             0x00e1  /* Voltage Monitor Circuit Status Register (8-bits) */
#define R_SYSTEM_PRCR_OFFSET              0x03fe  /* Protect Register (16-bits) */
#define R_SYSTEM_RSTSR0_OFFSET            0x0410  /* Reset Status Register 0 (8-bits) */
#define R_SYSTEM_RSTSR2_OFFSET            0x0411  /* Reset Status Register 2 (8-bits) */
#define R_SYSTEM_RSTSR1_OFFSET            0x00c0  /* Reset Status Register 1 (16-bits) */
#define R_SYSTEM_BKRACR_OFFSET            0x00c6  /* Backup Register Access Control Register (8-bits) */

/* Register Addresses *******************************************************/

#  define R_SYSTEM_VBTCR1                (R_SYSTEM_BASE + R_SYSTEM_VBTCR1_OFFSET)
#  define R_SYSTEM_VBTCR2                (R_SYSTEM_BASE + R_SYSTEM_VBTCR2_OFFSET)
#  define R_SYSTEM_VBTSR                 (R_SYSTEM_BASE + R_SYSTEM_VBTSR_OFFSET)
#  define R_SYSTEM_VBTCMPCR              (R_SYSTEM_BASE + R_SYSTEM_VBTCMPCR_OFFSET)
#  define R_SYSTEM_VBTLVDICR             (R_SYSTEM_BASE + R_SYSTEM_VBTLVDICR_OFFSET)
#  define R_SYSTEM_VBTWCTLR              (R_SYSTEM_BASE + R_SYSTEM_VBTWCTLR_OFFSET)
#  define R_SYSTEM_VBTWCH0OTSR           (R_SYSTEM_BASE + R_SYSTEM_VBTWCH0OTSR_OFFSET)
#  define R_SYSTEM_VBTWCH1OTSR           (R_SYSTEM_BASE + R_SYSTEM_VBTWCH1OTSR_OFFSET)
#  define R_SYSTEM_VBTWCH2OTSR           (R_SYSTEM_BASE + R_SYSTEM_VBTWCH2OTSR_OFFSET)
#  define R_SYSTEM_VBTICTLR              (R_SYSTEM_BASE + R_SYSTEM_VBTICTLR_OFFSET)
#  define R_SYSTEM_VBTOCTLR              (R_SYSTEM_BASE + R_SYSTEM_VBTOCTLR_OFFSET)
#  define R_SYSTEM_VBTWTER               (R_SYSTEM_BASE + R_SYSTEM_VBTWTER_OFFSET)
#  define R_SYSTEM_VBTWEGR               (R_SYSTEM_BASE + R_SYSTEM_VBTWEGR_OFFSET)
#  define R_SYSTEM_VBTWFR                (R_SYSTEM_BASE + R_SYSTEM_VBTWFR_OFFSET)
#  define R_SYSTEM_VBTBKR(p)             (R_SYSTEM_BASE + R_SYSTEM_VBTBKR_OFFSET + p*0x0001)
#  define R_SYSTEM_SCKDIVCR              (R_SYSTEM_BASE + R_SYSTEM_SCKDIVCR_OFFSET)
#  define R_SYSTEM_SCKSCR                (R_SYSTEM_BASE + R_SYSTEM_SCKSCR_OFFSET)
#  define R_SYSTEM_PLLCR                 (R_SYSTEM_BASE + R_SYSTEM_PLLCR_OFFSET)
#  define R_SYSTEM_PLLCCR2               (R_SYSTEM_BASE + R_SYSTEM_PLLCCR2_OFFSET)
#  define R_SYSTEM_MEMWAIT               (R_SYSTEM_BASE + R_SYSTEM_MEMWAIT_OFFSET)
#  define R_SYSTEM_MOSCCR                (R_SYSTEM_BASE + R_SYSTEM_MOSCCR_OFFSET)
#  define R_SYSTEM_HOCOCR                (R_SYSTEM_BASE + R_SYSTEM_HOCOCR_OFFSET)
#  define R_SYSTEM_MOCOCR                (R_SYSTEM_BASE + R_SYSTEM_MOCOCR_OFFSET)
#  define R_SYSTEM_SOSCCR                (R_SYSTEM_BASE + R_SYSTEM_SOSCCR_OFFSET)
#  define R_SYSTEM_LOCOCR                (R_SYSTEM_BASE + R_SYSTEM_LOCOCR_OFFSET)
#  define R_SYSTEM_OSCSF                 (R_SYSTEM_BASE + R_SYSTEM_OSCSF_OFFSET)
#  define R_SYSTEM_CKOCR                 (R_SYSTEM_BASE + R_SYSTEM_CKOCR_OFFSET)
#  define R_SYSTEM_TRCKCR                (R_SYSTEM_BASE + R_SYSTEM_TRCKCR_OFFSET)
#  define R_SYSTEM_OSTDCR                (R_SYSTEM_BASE + R_SYSTEM_OSTDCR_OFFSET)
#  define R_SYSTEM_OSTDSR                (R_SYSTEM_BASE + R_SYSTEM_OSTDSR_OFFSET)
#  define R_SYSTEM_SLCDSCKCR             (R_SYSTEM_BASE + R_SYSTEM_SLCDSCKCR_OFFSET)
#  define R_SYSTEM_MOCOUTCR              (R_SYSTEM_BASE + R_SYSTEM_MOCOUTCR_OFFSET)
#  define R_SYSTEM_HOCOUTCR              (R_SYSTEM_BASE + R_SYSTEM_HOCOUTCR_OFFSET)
#  define R_SYSTEM_MOSCWTCR              (R_SYSTEM_BASE + R_SYSTEM_MOSCWTCR_OFFSET)
#  define R_SYSTEM_HOCOWTCR              (R_SYSTEM_BASE + R_SYSTEM_HOCOWTCR_OFFSET)
#  define R_SYSTEM_USBCKCR               (R_SYSTEM_BASE + R_SYSTEM_USBCKCR_OFFSET)
#  define R_SYSTEM_MOMCR                 (R_SYSTEM_BASE + R_SYSTEM_MOMCR_OFFSET)
#  define R_SYSTEM_SOMCR                 (R_SYSTEM_BASE + R_SYSTEM_SOMCR_OFFSET)
#  define R_SYSTEM_LOCOUTCR              (R_SYSTEM_BASE + R_SYSTEM_LOCOUTCR_OFFSET)
#  define R_SYSTEM_SBYCR                 (R_SYSTEM_BASE + R_SYSTEM_SBYCR_OFFSET)
#  define R_SYSTEM_MSTPCRA               (R_SYSTEM_BASE + R_SYSTEM_MSTPCRA_OFFSET)
#  define R_SYSTEM_SNZCR                 (R_SYSTEM_BASE + R_SYSTEM_SNZCR_OFFSET)
#  define R_SYSTEM_SNZEDCR               (R_SYSTEM_BASE + R_SYSTEM_SNZEDCR_OFFSET)
#  define R_SYSTEM_SNZREQCR              (R_SYSTEM_BASE + R_SYSTEM_SNZREQCR_OFFSET)
#  define R_SYSTEM_FLSTOP                (R_SYSTEM_BASE + R_SYSTEM_FLSTOP_OFFSET)
#  define R_SYSTEM_OPCCR                 (R_SYSTEM_BASE + R_SYSTEM_OPCCR_OFFSET)
#  define R_SYSTEM_SOPCCR                (R_SYSTEM_BASE + R_SYSTEM_SOPCCR_OFFSET)
#  define R_SYSTEM_SYOCDCR               (R_SYSTEM_BASE + R_SYSTEM_SYOCDCR_OFFSET)
#  define R_SYSTEM_LVCMPCR               (R_SYSTEM_BASE + R_SYSTEM_LVCMPCR_OFFSET)
#  define R_SYSTEM_LVDLVLR               (R_SYSTEM_BASE + R_SYSTEM_LVDLVLR_OFFSET)
#  define R_SYSTEM_LVDCR0                (R_SYSTEM_BASE + R_SYSTEM_LVDCR0_OFFSET)
#  define R_SYSTEM_LVDCR1                (R_SYSTEM_BASE + R_SYSTEM_LVDCR1_OFFSET)
#  define R_SYSTEM_LVDSR                 (R_SYSTEM_BASE + R_SYSTEM_LVDSR_OFFSET)
#  define R_SYSTEM_PRCR                  (R_SYSTEM_BASE + R_SYSTEM_PRCR_OFFSET)
#  define R_SYSTEM_RSTSR0                (R_SYSTEM_BASE + R_SYSTEM_RSTSR0_OFFSET)
#  define R_SYSTEM_RSTSR2                (R_SYSTEM_BASE + R_SYSTEM_RSTSR2_OFFSET)
#  define R_SYSTEM_RSTSR1                (R_SYSTEM_BASE + R_SYSTEM_RSTSR1_OFFSET)
#  define R_SYSTEM_BKRACR                (R_SYSTEM_BASE + R_SYSTEM_BKRACR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* VBATT Control Register 1 (8-bits) */

#define R_SYSTEM_VBTCR1_BPWSWSTP          (1 <<  0) /* 01: Battery Power supply Switch Stop */

/* VBATT Control Register 2 (8-bits) */

#define R_SYSTEM_VBTCR2_VBTLVDLVL_SHIFT   (6) /* 40: VBATT Pin Voltage Low Voltage Detect Level Select Bit */
#define R_SYSTEM_VBTCR2_VBTLVDLVL_MASK    (3)
#define R_SYSTEM_VBTCR2_VBTLVDEN          (1 <<  4) /* 10: VBATT Pin Low Voltage Detect Enable Bit */

/* VBATT Status Register (8-bits) */

#define R_SYSTEM_VBTSR_VBTRVLD            (1 <<  4) /* 10: VBATT_R Valid */
#define R_SYSTEM_VBTSR_VBTBLDF            (1 <<  1) /* 02: VBATT Battery Low voltage Detect Flag */
#define R_SYSTEM_VBTSR_VBTRDF             (1 <<  0) /* 01: VBAT_R Reset Detect Flag */

/* VBATT Comparator Control Register (8-bits) */

#define R_SYSTEM_VBTLVDICR_VBTLVDISEL     (1 <<  1) /* 02: Pin Low Voltage Detect Interrupt Select bit */
#define R_SYSTEM_VBTLVDICR_VBTLVDIE       (1 <<  0) /* 01: VBATT Pin Low Voltage Detect Interrupt Enable bit */

/* VBATT Pin Low Voltage Detect Interrupt Control Register (8-bits) */

#define R_SYSTEM_VBTCMPCR_VBTCMPE         (1 <<  0) /* 01: VBATT pin low voltage detect circuit output enable */

/* VBATT Wakeup Function Control Register (8-bits) */

#define R_SYSTEM_VBTWCTLR_VWEN            (1 <<  0) /* 01: VBATT wakeup enable */

/* VBATT Wakeup I/O 0 Output Trigger Select Register (8-bits) */

#define R_SYSTEM_VBTWCH0OTSR_CH0VRTCATE   (1 <<  4) /* 10: VBATWIO0 Output RTC Alarm Signal Enable */
#define R_SYSTEM_VBTWCH0OTSR_CH0VRTCTE    (1 <<  3) /* 08: VBATWIO0 Output RTC Periodic Signal Enable */
#define R_SYSTEM_VBTWCH0OTSR_CH0VCH2TE    (1 <<  2) /* 04: VBATWIO0 Output VBATWIO2 Trigger Enable */
#define R_SYSTEM_VBTWCH0OTSR_CH0VCH1TE    (1 <<  1) /* 02: VBATWIO0 Output VBATWIO1 Trigger Enable */

/* VBATT Wakeup I/O 1 Output Trigger Select Register (8-bits) */

#define R_SYSTEM_VBTWCH1OTSR_CH1VRTCATE   (1 <<  4) /* 10: VBATWIO1 Output RTC Alarm Signal Enable */
#define R_SYSTEM_VBTWCH1OTSR_CH1VRTCTE    (1 <<  3) /* 08: VBATWIO1 Output RTC Periodic Signal Enable */
#define R_SYSTEM_VBTWCH1OTSR_CH1VCH2TE    (1 <<  2) /* 04: VBATWIO1 Output VBATWIO2 Trigger Enable */
#define R_SYSTEM_VBTWCH1OTSR_CH1VCH0TE    (1 <<  0) /* 01: VBATWIO1 Output VBATWIO0 Trigger Enable */

/* VBATT Wakeup I/O 2 Output Trigger Select Register (8-bits) */

#define R_SYSTEM_VBTWCH2OTSR_CH2VRTCATE   (1 <<  4) /* 10: VBATWIO2 Output RTC Alarm Signal Enable */
#define R_SYSTEM_VBTWCH2OTSR_CH2VRTCTE    (1 <<  3) /* 08: VBATWIO2 Output RTC Periodic Signal Enable */
#define R_SYSTEM_VBTWCH2OTSR_CH2VCH1TE    (1 <<  1) /* 02: VBATWIO2 Output VBATWIO1 Trigger Enable */
#define R_SYSTEM_VBTWCH2OTSR_CH2VCH0TE    (1 <<  0) /* 01: VBATWIO2 Output VBATWIO0 Trigger Enable */

/* VBATT Input Control Register (8-bits) */

#define R_SYSTEM_VBTICTLR_VCH2INEN        (1 <<  2) /* 04: VBATT Wakeup I/O 2 Input Enable */
#define R_SYSTEM_VBTICTLR_VCH1INEN        (1 <<  1) /* 02: VBATT Wakeup I/O 1 Input Enable */
#define R_SYSTEM_VBTICTLR_VCH0INEN        (1 <<  0) /* 01: VBATT Wakeup I/O 0 Input Enable */

/* VBATT Output Control Register (8-bits) */

#define R_SYSTEM_VBTOCTLR_VOUT2LSEL       (1 <<  5) /* 20: VBATT Wakeup I/O 2 Output Level Selection */
#define R_SYSTEM_VBTOCTLR_VOUT1LSEL       (1 <<  4) /* 10: VBATT Wakeup I/O 1 Output Level Selection */
#define R_SYSTEM_VBTOCTLR_VOUT0LSEL       (1 <<  3) /* 08: VBATT Wakeup I/O 0 Output Level Selection */
#define R_SYSTEM_VBTOCTLR_VCH2OEN         (1 <<  2) /* 04: VBATT Wakeup I/O 2 Output Enable */
#define R_SYSTEM_VBTOCTLR_VCH1OEN         (1 <<  1) /* 02: VBATT Wakeup I/O 1 Output Enable */
#define R_SYSTEM_VBTOCTLR_VCH0OEN         (1 <<  0) /* 01: VBATT Wakeup I/O 0 Output Enable */

/* VBATT Wakeup Trigger Source Enable Register (8-bits) */

#define R_SYSTEM_VBTWTER_VRTCAE           (1 <<  4) /* 10: RTC Alarm Signal Enable */
#define R_SYSTEM_VBTWTER_VRTCIE           (1 <<  3) /* 08: RTC Periodic Signal Enable */
#define R_SYSTEM_VBTWTER_VCH2E            (1 <<  2) /* 04: VBATWIO2 Pin Enable */
#define R_SYSTEM_VBTWTER_VCH1E            (1 <<  1) /* 02: VBATWIO1 Pin Enable */
#define R_SYSTEM_VBTWTER_VCH0E            (1 <<  0) /* 01: VBATWIO0 Pin Enable */

/* VBATT Wakeup Trigger Source Edge Register (8-bits) */

#define R_SYSTEM_VBTWEGR_VCH2EG           (1 <<  2) /* 04: VBATWIO2 Wakeup Trigger Source Edge Select */
#define R_SYSTEM_VBTWEGR_VCH1EG           (1 <<  1) /* 02: VBATWIO1 Wakeup Trigger Source Edge Select */
#define R_SYSTEM_VBTWEGR_VCH0EG           (1 <<  0) /* 01: VBATWIO0 Wakeup Trigger Source Edge Select */

/* VBATT Wakeup Trigger Source Flag Register (8-bits) */

#define R_SYSTEM_VBTWFR_VRTCAF            (1 <<  4) /* 10: VBATT RTC-Alarm Wakeup Trigger Flag */
#define R_SYSTEM_VBTWFR_VRTCIF            (1 <<  3) /* 08: VBATT RTC-Interval Wakeup Trigger Flag */
#define R_SYSTEM_VBTWFR_VCH2F             (1 <<  2) /* 04: VBATWIO2 Wakeup Trigger Flag */
#define R_SYSTEM_VBTWFR_VCH1F             (1 <<  1) /* 02: VBATWIO1 Wakeup Trigger Flag */
#define R_SYSTEM_VBTWFR_VCH0F             (1 <<  0) /* 01: VBATWIO0 Wakeup Trigger Flag */

/* VBATT Backup Register (8-bits) */

#define R_SYSTEM_VBTBKR_SIZE              512
#define R_SYSTEM_VBTBKR_VBTBKR            (8 <<  0) /* 01: VBTBKR is a 512-byte readable/writable register to store data powered by VBATT. The value of this register is retained even when VCC is not powered but VBATT is powered. VBTBKR is initialized by VBATT selected voltage power-on-reset. */
#define R_SYSTEM_VBTBKR_VBTBKR_MASK       (0xff)

/* System Clock Division Control Register (32-bits) */

#define R_SYSTEM_SCKDIVCR_FCK_SHIFT             (28)                                   /* Bits 30-28: Flash IF Clock (FCLK) Select */
#define R_SYSTEM_SCKDIVCR_FCK_MASK              (0x07 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_1           (0 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)     /* CLK/1 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_2           (1 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)     /* CLK/2 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_4           (2 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)     /* CLK/4 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_8           (3 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)     /* CLK/8 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_16          (4 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)     /* CLK/16 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_32          (5 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)     /* CLK/32 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_64          (6 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)     /* CLK/64 */
#define R_SYSTEM_SCKDIVCR_ICK_SHIFT             (24)                                   /* Bits 26-24: System Clock (ICLK) Select */
#define R_SYSTEM_SCKDIVCR_ICK_MASK              (0x07 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_1           (0 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)     /* CLK/1 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_2           (1 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)     /* CLK/2 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_4           (2 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)     /* CLK/4 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_8           (3 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)     /* CLK/8 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_16          (4 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)     /* CLK/16 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_32          (5 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)     /* CLK/32 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_64          (6 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)     /* CLK/64 */
#define R_SYSTEM_SCKDIVCR_PCKA_SHIFT            (12)                                   /* Bits 14-10: Peripheral Module Clock A (PCLKA) Select */
#define R_SYSTEM_SCKDIVCR_PCKA_MASK             (0x07 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_1          (0 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)    /* CLK/1 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_2          (1 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)    /* CLK/2 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_4          (2 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)    /* CLK/4 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_8          (3 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)    /* CLK/8 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_16         (4 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)    /* CLK/16 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_32         (5 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)    /* CLK/32 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_64         (6 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)    /* CLK/64 */
#define R_SYSTEM_SCKDIVCR_PCKB_SHIFT            (8)                                    /* Bits 10-8: Peripheral Module Clock B (PCLKB) Select */
#define R_SYSTEM_SCKDIVCR_PCKB_MASK             (0x07 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_1          (0 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)    /* CLK/1 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_2          (1 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)    /* CLK/2 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_4          (2 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)    /* CLK/4 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_8          (3 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)    /* CLK/8 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_16         (4 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)    /* CLK/16 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_32         (5 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)    /* CLK/32 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_64         (6 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)    /* CLK/64 */
#define R_SYSTEM_SCKDIVCR_PCKC_SHIFT            (4)                                    /* Bits 6-4: Peripheral Module Clock C (PCLKC) Select */
#define R_SYSTEM_SCKDIVCR_PCKC_MASK             (0x07 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_1          (0 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)    /* CLK/1 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_2          (1 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)    /* CLK/2 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_4          (2 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)    /* CLK/4 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_8          (3 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)    /* CLK/8 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_16         (4 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)    /* CLK/16 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_32         (5 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)    /* CLK/32 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_64         (6 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)    /* CLK/64 */
#define R_SYSTEM_SCKDIVCR_PCKD_SHIFT            (0)                                    /* Bits 2-0: Peripheral Module Clock D (PCLKD) Select */
#define R_SYSTEM_SCKDIVCR_PCKD_MASK             (0x07 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_1          (0 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)    /* CLK/1 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_2          (1 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)    /* CLK/2 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_4          (2 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)    /* CLK/4 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_8          (3 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)    /* CLK/8 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_16         (4 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)    /* CLK/16 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_32         (5 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)    /* CLK/32 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_64         (6 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)    /* CLK/64 */

/* System Clock Source Control Register (8-bits) */

#define R_SYSTEM_SCKSCR_CKSEL_SHIFT       (0) /* 01: Clock Source Select Selecting the system clock source faster than 32MHz(system clock source > 32MHz ) is prohibit when SCKDIVCR.ICK[2:0] bits select the division-by-1 and MEMWAIT.MEMWAIT =0. */
#define R_SYSTEM_SCKSCR_CKSEL_MASK        (0x07 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)
# define R_SYSTEM_SCKSCR_CKSEL_HOCO       (0 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)
# define R_SYSTEM_SCKSCR_CKSEL_MOCO       (1 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)
# define R_SYSTEM_SCKSCR_CKSEL_LOCO       (2 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)
# define R_SYSTEM_SCKSCR_CKSEL_MOSC       (3 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)
# define R_SYSTEM_SCKSCR_CKSEL_SOSC       (4 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)
# define R_SYSTEM_SCKSCR_CKSEL_PLL        (5 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)

/* PLL Control Register (8-bits) */

#define R_SYSTEM_PLLCR_PLLSTP             (1 <<  0) /* 01: PLL Stop Control */

/* PLL Clock Control Register 2 (8-bits) */

#define R_SYSTEM_PLLCCR2                  (R_SYSTEM_BASE + R_SYSTEM_PLLCCR2_OFFSET)
#define R_SYSTEM_PLLCCR2_PLODIV_SHIFT     (6) /* 40: PLL Output Frequency Division Ratio Select */
#define R_SYSTEM_PLLCCR2_PLODIV_MASK      (0x03)
#define R_SYSTEM_PLLCCR2_PLLMUL_SHIFT     (0) /* 01: PLL Frequency Multiplication Factor Select */
#define R_SYSTEM_PLLCCR2_PLLMUL_MASK      (0x1f)

/* Memory Wait Cycle Control Register (8-bits) */

#define R_SYSTEM_MEMWAIT_MEMWAIT          (1 <<  0) /* 01: Memory Wait Cycle Select Note: Writing 0 to the MEMWAIT is prohibited when SCKDIVCR.ICK selects division by 1 and SCKSCR.CKSEL[2:0] bits select the system clock source that is faster than 32 MHz (ICLK > 32 MHz). */

/* Main Clock Oscillator Control Register (8-bits) */

#define R_SYSTEM_MOSCCR_MOSTP             (1 <<  0) /* 01: Main Clock Oscillator Stop Note: MOMCR register must be set before setting MOSTP to 0. */

/* High-Speed On-Chip Oscillator Control Register (8-bits) */

#define R_SYSTEM_HOCOCR_HCSTP             (1 <<  0) /* 01: HOCO Stop */

/* Middle-Speed On-Chip Oscillator Control Register (8-bits) */

#define R_SYSTEM_MOCOCR_MCSTP             (1 <<  0) /* 01: MOCO Stop */

/* Sub-Clock Oscillator Control Register (8-bits) */

#define R_SYSTEM_SOSCCR_SOSTP             (1 <<  0) /* 01: Sub-Clock Oscillator Stop */

/* Low-Speed On-Chip Oscillator Control Register (8-bits) */

#define R_SYSTEM_LOCOCR_LCSTP             (1 <<  0) /* 01: LOCO Stop */

/* Oscillation Stabilization Flag Register (8-bits) */

#define R_SYSTEM_OSCSF_PLLSF              (1 <<  5) /* 20: PLL Clock Oscillation Stabilization Flag */
#define R_SYSTEM_OSCSF_MOSCSF             (1 <<  3) /* 08: Main Clock Oscillation Stabilization Flag */
#define R_SYSTEM_OSCSF_HOCOSF             (1 <<  0) /* 01: HOCO Clock Oscillation Stabilization Flag NOTE: The HOCOSF bit value after a reset is 1 when the OFS1.HOCOEN bit is 0. It is 0 when the OFS1.HOCOEN bit is 1. */

/* Clock Out Control Register (8-bits) */

#define R_SYSTEM_CKOCR_CKOEN              (1 <<  7) /* 80: Clock out enable */
#define R_SYSTEM_CKOCR_CKODIV_SHIFT       (4)       /* 10: Clock out input frequency Division Select */
#define R_SYSTEM_CKOCR_CKODIV_MASK        (0x07)
#define R_SYSTEM_CKOCR_CKOSEL_SHIFT       (0)       /* 01: Clock out source select */
#define R_SYSTEM_CKOCR_CKOSEL_MASK        (0x07)

/* Trace Clock Control Register (8-bits) */

#define R_SYSTEM_TRCKCR_TRCKEN            (1 <<  7) /* 80: Trace Clock operating enable */
#define R_SYSTEM_TRCKCR_TRCK              (4 <<  0) /* 01: Trace Clock operating frequency select */
#define R_SYSTEM_TRCKCR_TRCK_MASK         (0x0f)

/* Oscillation Stop Detection Control Register (8-bits) */

#define R_SYSTEM_OSTDCR_OSTDE             (1 <<  7) /* 80: Oscillation Stop Detection Function Enable */
#define R_SYSTEM_OSTDCR_OSTDIE            (1 <<  0) /* 01: Oscillation Stop Detection Interrupt Enable */

/* Oscillation Stop Detection Status Register (8-bits) */

#define R_SYSTEM_OSTDSR_OSTDF             (1 <<  0) /* 01: Oscillation Stop Detection Flag */

/* Segment LCD Source Clock Control Register (8-bits) */

#define R_SYSTEM_SLCDSCKCR_LCDSCKEN           (1 <<  7)   /* 80: LCD Source Clock Out Enable */
#define R_SYSTEM_SLCDSCKCR_LCDSCKSEL_SHIFT    (0)         /* 01: LCD Source Clock (LCDSRCCLK) Select */
#define R_SYSTEM_SLCDSCKCR_LCDSCKSEL_MASK     (0x07)

/* MOCO User Trimming Control Register (8-bits) */

#define R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT         (0) /* 01: MOCO User Trimming 1000_0000 : -128 1000_0001 : -127 1000_0010 : -126 . . . 1111_1111 : -1 0000_0000 : Center Code 0000_0001 : +1 . . . 0111_1101 : +125 0111_1110 : +126 0111_1111 : +127 These bits are added to original MOCO trimming bits */
#define R_SYSTEM_MOCOUTCR_MOCOUTRM_MASK          (0xff)

/* HOCO User Trimming Control Register (8-bits) */

#define R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT         (0) /* 01: HOCO User Trimming 1000_0000 : -128 1000_0001 : -127 1000_0010 : -126 . . . 1111_1111 : -1 0000_0000 : Center Code 0000_0001 : +1 . . . 0111_1101 : +125 0111_1110 : +126 0111_1111 : +127 These bits are added to original HOCO trimming bits */
#define R_SYSTEM_HOCOUTCR_HOCOUTRM_MASK          (0xff)

/* Main Clock Oscillator Wait Control Register (8-bits) */

#define R_SYSTEM_MOSCWTCR_MSTS_SHIFT            (0) /* 01: Main clock oscillator wait time setting */
#define R_SYSTEM_MOSCWTCR_MSTS_MASK             (0x0f)

/* High-Speed On-Chip Oscillator Wait Control Register (8-bits) */

#define R_SYSTEM_HOCOWTCR_HSTS_SHIFT             (0) /* 01: HOCO wait time setting */
#define R_SYSTEM_HOCOWTCR_HSTS_MASK              (0x07)

/* USB Clock Control Register (8-bits) */

#define R_SYSTEM_USBCKCR_USBCLKSEL        (1 <<  0) /* 01: USB Clock Source Select */

/* Main Clock Oscillator Mode Oscillation Control Register (8-bits) */

#define R_SYSTEM_MOMCR_MOSEL              (1 <<  6) /* 40: Main Clock Oscillator Switching */
#define R_SYSTEM_MOMCR_MODRV1             (1 <<  3) /* 08: Main Clock Oscillator Drive Capability 1 Switching */

/* Sub-Clock Oscillator Mode Control Register (8-bits) */

#define R_SYSTEM_SOMCR_SODRV_SHIFT             (0) /* 01: Sub-Clock Oscillator Drive Capability Switching */
#define R_SYSTEM_SOMCR_SODRV_MASK              (0x03)

/* LOCO User Trimming Control Register (8-bits) */

#define R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT       (0) /* 01: LOCO User Trimming 1000_0000 : -128 1000_0001 : -127 1000_0010 : -126 . . . 1111_1111 : -1 0000_0000 : Center Code 0000_0001 : +1 . . . 0111_1101 : +125 0111_1110 : +126 0111_1111 : +127 These bits are added to original LOCO trimming bits */
#define R_SYSTEM_LOCOUTCR_LOCOUTRM_MASK        (0xff)

/* Standby Control Register (16-bits) */

#define R_SYSTEM_SBYCR_SSBY               (1 << 15) /* 8000: Software Standby */

/* Module Stop Control Register A (32-bits) */

#define R_SYSTEM_MSTPCRA_MSTPA22          (1 << 22) /* 400000: DMA Controller/Data Transfer Controller Module Stop */
#define R_SYSTEM_MSTPCRA_MSTPA6           (1 <<  6) /* 40: ECCRAM Module Stop */
#define R_SYSTEM_MSTPCRA_MSTPA0           (1 <<  0) /* 01: RAM0 Module Stop */

/* Snooze Control Register (8-bits) */

#define R_SYSTEM_SNZCR_SNZE               (1 <<  7) /* 80: Snooze Mode Enable */
#define R_SYSTEM_SNZCR_SNZDTCEN           (1 <<  1) /* 02: DTC Enable in Snooze Mode */
#define R_SYSTEM_SNZCR_RXDREQEN           (1 <<  0) /* 01: RXD0 Snooze Request Enable NOTE: Do not set to 1 other than in asynchronous mode. */

/* Snooze End Control Register (8-bits) */

#define R_SYSTEM_SNZEDCR_SCI0UMTED        (1 <<  7) /* 80: SCI0 Address Mismatch Snooze End Enable */
#define R_SYSTEM_SNZEDCR_AD0UMTED         (1 <<  4) /* 10: ADC140 Compare Mismatch Snooze End Enable */
#define R_SYSTEM_SNZEDCR_AD0MATED         (1 <<  3) /* 08: ADC140 Compare Match Snooze End Enable */
#define R_SYSTEM_SNZEDCR_DTCNZRED         (1 <<  2) /* 04: Not Last DTC Transmission Completion Snooze End Enable */
#define R_SYSTEM_SNZEDCR_DTCZRED          (1 <<  1) /* 02: Last DTC Transmission Completion Snooze End Enable */
#define R_SYSTEM_SNZEDCR_AGTUNFED         (1 <<  0) /* 01: AGT1 Underflow Snooze End Enable */

/* Snooze Request Control Register (32-bits) */

#define R_SYSTEM_SNZREQCR_SNZREQEN30      (1 << 30) /* 40000000: Snooze Request Enable 30 Enable AGT1 compare match B snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN29      (1 << 29) /* 20000000: Snooze Request Enable 29 Enable AGT1 compare match A snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN28      (1 << 28) /* 10000000: Snooze Request Enable 28 Enable AGT1 underflow snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN25      (1 << 25) /* 2000000: Snooze Request Enable 25 Enable RTC period snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN24      (1 << 24) /* 1000000: Snooze Request Enable 24 Enable RTC alarm snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN23      (1 << 23) /* 800000: Snooze Request Enable 23 Enable RTC alarm snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN17      (1 << 17) /* 20000: Snooze Request Enable 17 Enable KINT snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN15      (1 << 15) /* 8000: Snooze Request Enable 15 Enable IRQ15 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN14      (1 << 14) /* 4000: Snooze Request Enable 14 Enable IRQ14 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN12      (1 << 12) /* 1000: Snooze Request Enable 12 Enable IRQ12 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN11      (1 << 11) /* 800: Snooze Request Enable 11 Enable IRQ11 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN10      (1 << 10) /* 400: Snooze Request Enable 10 Enable IRQ10 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN9       (1 <<  9) /* 200: Snooze Request Enable 9 Enable IRQ9 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN8       (1 <<  8) /* 100: Snooze Request Enable 8 Enable IRQ8 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN7       (1 <<  7) /* 80: Snooze Request Enable 7 Enable IRQ7 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN6       (1 <<  6) /* 40: Snooze Request Enable 6 Enable IRQ6 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN5       (1 <<  5) /* 20: Snooze Request Enable 5 Enable IRQ5 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN4       (1 <<  4) /* 10: Snooze Request Enable 4 Enable IRQ4 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN3       (1 <<  3) /* 08: Snooze Request Enable 3 Enable IRQ3 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN2       (1 <<  2) /* 04: Snooze Request Enable 2 Enable IRQ2 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN1       (1 <<  1) /* 02: Snooze Request Enable 1 Enable IRQ1 pin snooze request */
#define R_SYSTEM_SNZREQCR_SNZREQEN0       (1 <<  0) /* 01: Snooze Request Enable 0 Enable IRQ0 pin snooze request */

/* Flash Operation Control Register (8-bits) */

#define R_SYSTEM_FLSTOP_FLSTPF            (1 <<  4) /* 10: Flash Memory Operation Status Flag */
#define R_SYSTEM_FLSTOP_FLSTOP            (1 <<  0) /* 01: Selecting ON/OFF of the Flash Memory Operation */

/* Operating Power Control Register (8-bits) */

#define R_SYSTEM_OPCCR_OPCMTSF            (1 <<  4) /* 10: Operating Power Control Mode Transition Status Flag */
#define R_SYSTEM_OPCCR_OPCM_SHIFT         (0)       /* 01: Operating Power Control Mode Select */
#define R_SYSTEM_OPCCR_OPCM_MASK          (0x03)

/* Sub Operating Power Control Register (8-bits) */

#define R_SYSTEM_SOPCCR_SOPCMTSF          (1 <<  4) /* 10: Sub Operating Power Control Mode Transition Status Flag */
#define R_SYSTEM_SOPCCR_SOPCM             (1 <<  0) /* 01: Sub Operating Power Control Mode Select */

/* System Control OCD Control Register (8-bits) */

#define R_SYSTEM_SYOCDCR_DBGEN            (1 <<  7) /* 80: Debugger Enable bit */

/* Voltage Monitor Circuit Control Register (8-bits) */

#define R_SYSTEM_LVCMPCR_LVD2E            (1 <<  6) /* 40: Voltage Detection 2 Enable */
#define R_SYSTEM_LVCMPCR_LVD1E            (1 <<  5) /* 20: Voltage Detection 1 Enable */

/* Voltage Detection Level Select Register (8-bits) */

#define R_SYSTEM_LVDLVLR_LVD2LVL_SHIFT         (5) /* 20: Voltage Detection 2 Level Select (Standard voltage during drop in voltage) */
#define R_SYSTEM_LVDLVLR_LVD2LVL_MASK          (0x07)
#define R_SYSTEM_LVDLVLR_LVD1LVL_SHIFT         (0) /* 01: Voltage Detection 1 Level Select (Standard voltage during drop in voltage) */
#define R_SYSTEM_LVDLVLR_LVD1LVL_MASK          (0x1f)
#define R_SYSTEM_LVDCR0_SIZE                   (2)

/* Voltage Monitor Circuit Control Register 0 (8-bits) */

#define R_SYSTEM_LVDCR0_RN                (1 <<  7) /* 80: Voltage Monitor Reset Negate Select */
#define R_SYSTEM_LVDCR0_RI                (1 <<  6) /* 40: Voltage Monitor Circuit Mode Select */
#define R_SYSTEM_LVDCR0_CMPE              (1 <<  2) /* 04: Voltage Monitor Circuit Comparison Result Output Enable */
#define R_SYSTEM_LVDCR0_RIE               (1 <<  0) /* 01: Voltage Monitor Interrupt/Reset Enable */
#define R_SYSTEM_LVDCR1_SIZE              (2)

/* Voltage Monitor Circuit Control Register 1 (8-bits) */

#define R_SYSTEM_LVDCR1_IRQSEL            (1 <<  2) /* 04: Voltage Monitor Interrupt Type Select */
#define R_SYSTEM_LVDCR1_IDTSEL_SHIFT      (0)       /* 01: Voltage Monitor Interrupt Generation Condition Select */
#define R_SYSTEM_LVDCR1_IDTSEL_MASK       (0x03)
#define R_SYSTEM_LVDSR_SIZE               (2)

/* Voltage Monitor Circuit Status Register (8-bits) */

#define R_SYSTEM_LVDSR_MON                (1 <<  1) /* 02: Voltage Monitor 1 Signal Monitor Flag */
#define R_SYSTEM_LVDSR_DET                (1 <<  0) /* 01: Voltage Monitor Voltage Change Detection Flag NOTE: Only 0 can be written to this bit. After writing 0 to this bit, it takes 2 system clock cycles for the bit to be read as 0. */

/* Protect Register (16-bits) */

#define R_SYSTEM_PRCR_PRKEY_SHIFT         (8)       /* 100: PRC Key Code */
#define R_SYSTEM_PRCR_PRKEY_MASK          (0xff)
#define R_SYSTEM_PRCR_PRKEY_VALUE         (0xA5)
#define R_SYSTEM_PRCR_PRC3                (1 <<  3) /* 08: Protect Bit 3 */
#define R_SYSTEM_PRCR_PRC1                (1 <<  1) /* 02: Protect Bit 1 */
#define R_SYSTEM_PRCR_PRC0                (1 <<  0) /* 01: Protect Bit 0 */

/* Reset Status Register 0 (8-bits) */

#define R_SYSTEM_RSTSR0_LVD2RF            (1 <<  3) /* 08: Voltage Monitor 2 Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written with 0 after the reset flag is read as 1. */
#define R_SYSTEM_RSTSR0_LVD1RF            (1 <<  2) /* 04: Voltage Monitor 1 Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written with 0 after the reset flag is read as 1. */
#define R_SYSTEM_RSTSR0_LVD0RF            (1 <<  1) /* 02: Voltage Monitor 0 Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written with 0 after the reset flag is read as 1. */
#define R_SYSTEM_RSTSR0_PORF              (1 <<  0) /* 01: Power-On Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written with 0 after the reset flag is read as 1. */

/* Reset Status Register 2 (8-bits) */

#define R_SYSTEM_RSTSR2_CWSF              (1 <<  0) /* 01: Cold/Warm Start Determination Flag Note: Only 1 can be written to set the flag. */

/* Reset Status Register 1 (16-bits) */

#define R_SYSTEM_RSTSR1_SPERF             (1 << 12) /* 1000: SP Error Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written as 0 after the reset flag is read as 1. */
#define R_SYSTEM_RSTSR1_BUSMRF            (1 << 11) /* 800: Bus Master MPU Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written as 0 after the reset flag is read as 1. */
#define R_SYSTEM_RSTSR1_BUSSRF            (1 << 10) /* 400: Bus Slave MPU Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written as 0 after the reset flag is read as 1. */
#define R_SYSTEM_RSTSR1_REERF             (1 <<  9) /* 200: RAM ECC Error Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written as 0 after the reset flag is read as 1. */
#define R_SYSTEM_RSTSR1_RPERF             (1 <<  8) /* 100: RAM Parity Error Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written as 0 after the reset flag is read as 1. */
#define R_SYSTEM_RSTSR1_SWRF              (1 <<  2) /* 04: Software Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written as 0 after the reset flag is read as 1. */
#define R_SYSTEM_RSTSR1_WDTRF             (1 <<  1) /* 02: Watchdog Timer Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written as 0 after the reset flag is read as 1. */
#define R_SYSTEM_RSTSR1_IWDTRF            (1 <<  0) /* 01: Independent Watchdog Timer Reset Detect Flag Note: Only 0 can be written to clear the flag. The reset flag must be written as 0 after the reset flag is read as 1. */

/* Backup Register Access Control Register (8-bits) */

#define R_SYSTEM_BKRACR_BKRACS_SHIFT            (0) /* 01: Backup Register Access Control Register */
#define R_SYSTEM_BKRACR_BKRACS_MASK             (0x07)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA4M1_HARDWARE_RA4M1_SYSTEM_H */
