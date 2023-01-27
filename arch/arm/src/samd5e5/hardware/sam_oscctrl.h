/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_oscctrl.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_OSCCTRL_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_OSCCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OSCCTRL register offsets *************************************************/

#define SAM_OSCCTRL_EVCTRL_OFFSET         0x0000  /* Event Control */
#define SAM_OSCCTRL_INTENCLR_OFFSET       0x0004  /* Interrupt enable clear */
#define SAM_OSCCTRL_INTENSET_OFFSET       0x0008  /* Interrupt enable set */
#define SAM_OSCCTRL_INTFLAG_OFFSET        0x000c  /* Interrupt flag status and clear */
#define SAM_OSCCTRL_STATUS_OFFSET         0x0010  /* Status */
#define SAM_OSCCTRL_XOSCCTRL0_OFFSET      0x0014  /* External multi-purpose crystal oscillator control 0 */
#define SAM_OSCCTRL_XOSCCTRL1_OFFSET      0x0018  /* External multi-purpose crystal oscillator control 1 */

#define SAM_OSCCTRL_DFLLCTRLA_OFFSET      0x001c  /* DFLL Control A */
#define SAM_OSCCTRL_DFLLCTRLB_OFFSET      0x0020  /* DFLL Control B */
#define SAM_OSCCTRL_DFLLVAL_OFFSET        0x0024  /* DFLL value */
#define SAM_OSCCTRL_DFLLMUL_OFFSET        0x0028  /* DFLL multiplier */
#define SAM_OSCCTRL_DFLLSYNC_OFFSET       0x002c  /* DFLL synchronization */

#define SAM_OSCCTRL_DPLL0_OFFSET          0x0030  /* DPLL0 base offset */
#define SAM_OSCCTRL_DPLL1_OFFSET          0x0044  /* DPLL1 base offset */
#  define SAM_OSCCTRL_DPLLCTRLA_OFFSET    0x0000  /* DPLLn control A */
#  define SAM_OSCCTRL_DPLLRATIO_OFFSET    0x0004  /* DPLLn ratio control */
#  define SAM_OSCCTRL_DPLLCTRLB_OFFSET    0x0008  /* DPLLn control B */
#  define SAM_OSCCTRL_DPLLSYNCBUSY_OFFSET 0x000c  /* DPLLn synchronization busy */
#  define SAM_OSCCTRL_DPLLSTATUS_OFFSET   0x0010  /* DPLLn status */

#define SAM_OSCCTRL_DPLL0CTRLA_OFFSET     0x0030  /* DPLL0 control A */
#define SAM_OSCCTRL_DPLL0RATIO_OFFSET     0x0034  /* DPLL0 ratio control */
#define SAM_OSCCTRL_DPLL0CTRLB_OFFSET     0x0038  /* DPLL0 control B */
#define SAM_OSCCTRL_DPLL0SYNCBUSY_OFFSET  0x003c  /* DPLL0 synchronization busy */
#define SAM_OSCCTRL_DPLL0STATUS_OFFSET    0x0040  /* DPLL0 status */

#define SAM_OSCCTRL_DPLL1CTRLA_OFFSET     0x0044  /* DPLL1 control A */
#define SAM_OSCCTRL_DPLL1RATIO_OFFSET     0x0048  /* DPLL1 ratio control */
#define SAM_OSCCTRL_DPLL1CTRLB_OFFSET     0x004c  /* DPLL1 control B */
#define SAM_OSCCTRL_DPLL1SYNCBUSY_OFFSET  0x0050  /* DPLL1 synchronization busy */
#define SAM_OSCCTRL_DPLL1STATUS_OFFSET    0x0054  /* DPLL1 status */

/* OSCCTRL register addresses ***********************************************/

#define SAM_OSCCTRL_EVCTRL                (SAM_OSCCTRL_BASE + SAM_OSCCTRL_EVCTRL_OFFSET)
#define SAM_OSCCTRL_INTENCLR              (SAM_OSCCTRL_BASE + SAM_OSCCTRL_INTENCLR_OFFSET)
#define SAM_OSCCTRL_INTENSET              (SAM_OSCCTRL_BASE + SAM_OSCCTRL_INTENSET_OFFSET)
#define SAM_OSCCTRL_INTFLAG               (SAM_OSCCTRL_BASE + SAM_OSCCTRL_INTFLAG_OFFSET)
#define SAM_OSCCTRL_STATUS                (SAM_OSCCTRL_BASE + SAM_OSCCTRL_STATUS_OFFSET)
#define SAM_OSCCTRL_XOSCCTRL0             (SAM_OSCCTRL_BASE + SAM_OSCCTRL_XOSCCTRL0_OFFSET)
#define SAM_OSCCTRL_XOSCCTRL1             (SAM_OSCCTRL_BASE + SAM_OSCCTRL_XOSCCTRL1_OFFSET)

#define SAM_OSCCTRL_DFLLCTRLA             (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DFLLCTRLA_OFFSET)
#define SAM_OSCCTRL_DFLLCTRLB             (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DFLLCTRLB_OFFSET)
#define SAM_OSCCTRL_DFLLVAL               (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DFLLVAL_OFFSET)
#define SAM_OSCCTRL_DFLLMUL               (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DFLLMUL_OFFSET)
#define SAM_OSCCTRL_DFLLSYNC              (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DFLLSYNC_OFFSET)

#define SAM_OSCCTRL_DPLL0_BASE            (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL0_OFFSET)
#define SAM_OSCCTRL_DPLL0CTRLA            (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL0CTRLA_OFFSET)
#define SAM_OSCCTRL_DPLL0RATIO            (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL0RATIO_OFFSET)
#define SAM_OSCCTRL_DPLL0CTRLB            (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL0CTRLB_OFFSET)
#define SAM_OSCCTRL_DPLLPRESC             (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLLPRESC_OFFSET)
#define SAM_OSCCTRL_DPLL0SYNCBUSY         (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL0SYNCBUSY_OFFSET)
#define SAM_OSCCTRL_DPLL0STATUS           (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL0STATUS_OFFSET)

#define SAM_OSCCTRL_DPLL1_BASE            (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL1_OFFSET)
#define SAM_OSCCTRL_DPLL1CTRLA            (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL1CTRLA_OFFSET)
#define SAM_OSCCTRL_DPLL1RATIO            (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL1RATIO_OFFSET)
#define SAM_OSCCTRL_DPLL1CTRLB            (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL1CTRLB_OFFSET)
#define SAM_OSCCTRL_DPLLPRESC             (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLLPRESC_OFFSET)
#define SAM_OSCCTRL_DPLL1SYNCBUSY         (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL1SYNCBUSY_OFFSET)
#define SAM_OSCCTRL_DPLL1STATUS           (SAM_OSCCTRL_BASE + SAM_OSCCTRL_DPLL1STATUS_OFFSET)

/* OSCCTRL register bit definitions *****************************************/

/* Event Control */

#define OSCCTRL_EVCTRL_CFDEO0             (1 << 0)  /* Bit 0:  Clock 0 failure detector event output enable */
#define OSCCTRL_EVCTRL_CFDEO1             (1 << 1)  /* Bit 1:  Clock 1 failure detector event output enable */

/* Interrupt enable clear, Interrupt enable set, Interrupt flag status and
 * clear, and Status registers.
 */

#define OSCCTRL_INT_XOSCRDY0              (1 << 0)  /* Bit 0:  XOSC 0 ready interrupt */
#define OSCCTRL_INT_XOSCRDY1              (1 << 1)  /* Bit 1:  XOSC 1 ready interrupt */
#define OSCCTRL_INT_XOSCFAIL0             (1 << 2)  /* Bit 2:  XOSC 0 clock failure interrupt */
#define OSCCTRL_INT_XOSCFAIL1             (1 << 3)  /* Bit 3:  XOSC 1 clock failure interrupt */
#define OSCCTRL_STATUS_XOSCCKSW0          (1 << 4)  /* Bit 4:  XOSC 1 clock failure interrupt (status) */
#define OSCCTRL_STATUS_XOSCCKSW1          (1 << 5)  /* Bit 5:  XOSC 1 clock failure interrupt (status) */
#define OSCCTRL_INT_DFLLRDY               (1 << 8)  /* Bit 8:  DFLL ready interrupt */
#define OSCCTRL_INT_DFLLOOB               (1 << 9)  /* Bit 9:  DFLL out of bounds interrupt */
#define OSCCTRL_INT_DFLLLCKF              (1 << 10) /* Bit 10: DFLL lock fine interrupt */
#define OSCCTRL_INT_DFLLLCKC              (1 << 11) /* Bit 11: DFLL lock coarse interrupt */
#define OSCCTRL_INT_DFLLRCS               (1 << 12) /* Bit 12: DFLL reference clock stopped interrupt */
#define OSCCTRL_INT_DPLL0LCKR             (1 << 16) /* Bit 16: DPLL0 lock rise interrupt */
#define OSCCTRL_INT_DPLL0LCKF             (1 << 17) /* Bit 17: DPLL0 lock fall interrupt */
#define OSCCTRL_INT_DPLL0LTO              (1 << 18) /* Bit 18: DPLL0 lock timeout */
#define OSCCTRL_INT_DPLL0DRTO             (1 << 19) /* Bit 19: DPLL0 loop divider ratio update complete */
#define OSCCTRL_INT_DPLL1LCKR             (1 << 24) /* Bit 24: DPLL1 lock rise interrupt */
#define OSCCTRL_INT_DPLL1LCKF             (1 << 25) /* Bit 25: DPLL1 lock fall interrupt */
#define OSCCTRL_INT_DPLL1LTO              (1 << 26) /* Bit 26: DPLL1 lock timeout */
#define OSCCTRL_INT_DPLL1DRTO             (1 << 27) /* Bit 27: DPLL1 loop divider ratio update complete */

#define OSCCTRL_INT_ALL                   (0x0f0f1f0f)

/* External multi-purpose crystal oscillator control register 0/1 */

#define OSCCTRL_XOSCCTRL_ENABLE           (1 << 1)  /* Bit 1:  Oscillator enable */
#define OSCCTRL_XOSCCTRL_XTALEN           (1 << 2)  /* Bit 2:  Crystal oscillator enable */
#define OSCCTRL_XOSCCTRL_RUNSTDBY         (1 << 6)  /* Bit 6:  Run in standby */
#define OSCCTRL_XOSCCTRL_ONDEMAND         (1 << 7)  /* Bit 7:  On demand control */
#define OSCCTRL_XOSCCTRL_LOWBUFGAIN       (1 << 8)  /* Bit 8:  Low buffer gain enable */
#define OSCCTRL_XOSCCTRL_IPTAT_SHIFT      (9)       /* Bits 9-10: Oscillator current reference */
#define OSCCTRL_XOSCCTRL_IPTAT_MASK       (3 << OSCCTRL_XOSCCTRL_IPTAT_SHIFT)
#  define OSCCTRL_XOSCCTRL_IPTAT(n)       ((uint32_t)(n) << OSCCTRL_XOSCCTRL_IPTAT_SHIFT)
#define OSCCTRL_XOSCCTRL_IMULT_SHIFT      (11)      /* Bits 11-14: Oscillator current reference */
#define OSCCTRL_XOSCCTRL_IMULT_MASK       (15 << OSCCTRL_XOSCCTRL_IMULT_SHIFT)
#  define OSCCTRL_XOSCCTRL_IMULT(n)       ((uint32_t)(n) << OSCCTRL_XOSCCTRL_IMULT_SHIFT)
#define OSCCTRL_XOSCCTRL_ENALC            (1 << 15) /* Bit 15: Automatic loop control enable */
#define OSCCTRL_XOSCCTRL_CFDEN            (1 << 16) /* Bit 16: Clock failure detector enable */
#define OSCCTRL_XOSCCTRL_SWBEN            (1 << 17) /* Bit 17: XOSC clock switch enable */
#define OSCCTRL_XOSCCTRL_STARTUP_SHIFT    (20)      /* Bits 20-23: Start-up time */
#define OSCCTRL_XOSCCTRL_STARTUP_MASK     (15 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)
#  define OSCCTRL_XOSCCTRL_STARTUP(n)     ((n) << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)
#  define OSCCTRL_XOSCCTRL_STARTUP_31US   (0 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 31µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_61US   (1 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 61µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_122US  (2 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 122µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_244US  (3 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 244µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_488US  (4 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 488µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_977US  (5 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 977µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_2MS    (6 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 1953µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_4MS    (7 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 3906µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_8MS    (8 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 7813µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_16MS   (9 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 15625µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_31MS   (10 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 31250µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_63MS   (11 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 62500µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_125MS  (12 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 125000µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_250MS  (13 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 250000µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_500MS  (14 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 500000µs */
#  define OSCCTRL_XOSCCTRL_STARTUP_1S     (15 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 1000000µs */

#define OSCCTRL_XOSCCTRL_CFDPRESC_SHIFT   (24)      /* Bits 24-27: Clock Failure Detector Prescaler */
#define OSCCTRL_XOSCCTRL_CFDPRESC_MASK    (15 << OSCCTRL_XOSCCTRL_CFDPRESC_SHIFT)
#  define OSCCTRL_XOSCCTRL_CFDPRESC(n)    ((uint32_t)(n) << OSCCTRL_XOSCCTRL_CFDPRESC_SHIFT)

/* DFLL control register A */

#define OSCCTRL_DFLLCTRLA_ENABLE          (1 << 1)  /* Bit 1:  DFLL enable */
#define OSCCTRL_DFLLCTRLA_RUNSTDBY        (1 << 6)  /* Bit 6:  Run in standby */
#define OSCCTRL_DFLLCTRLA_ONDEMAND        (1 << 7)  /* Bit 7:  On demand control */

/* DFLL control register B */

#define OSCCTRL_DFLLCTRLB_MODE            (1 << 0)  /* Bit 0:  Operating mode selection */
#define OSCCTRL_DFLLCTRLB_STABLE          (1 << 1)  /* Bit 1:  Stable DFLL frequency */
#define OSCCTRL_DFLLCTRLB_LLAW            (1 << 2)  /* Bit 2:  Lose lock after wake */
#define OSCCTRL_DFLLCTRLB_USBCRM          (1 << 3)  /* Bit 3:  USB clock recovery mode */
#define OSCCTRL_DFLLCTRLB_CCDIS           (1 << 4)  /* Bit 4:  Chill cycle disable */
#define OSCCTRL_DFLLCTRLB_QLDIS           (1 << 5)  /* Bit 5:  Quick Lock Disable */
#define OSCCTRL_DFLLCTRLB_BPLCKC          (1 << 6)  /* Bit 6:  Bypass coarse clock */
#define OSCCTRL_DFLLCTRLB_WAITLOCK        (1 << 7)  /* Bit 7:  Wait lock */

/* DFLL value register */

#define OSCCTRL_DFLLVAL_FINE_SHIFT        (0)       /* Bits 0-7: Fine value */
#define OSCCTRL_DFLLVAL_FINE_MASK         (0xff << OSCCTRL_DFLLVAL_FINE_SHIFT)
#  define OSCCTRL_DFLLVAL_FINE(n)         ((uint32_t)(n) << OSCCTRL_DFLLVAL_FINE_SHIFT)
#define OSCCTRL_DFLLVAL_COARSE_SHIFT      (10)      /* Bits 10-15: Coarse value */
#define OSCCTRL_DFLLVAL_COARSE_MASK       (0x3f << OSCCTRL_DFLLVAL_COARSE_SHIFT)
#  define OSCCTRL_DFLLVAL_COARSE(n)       ((uint32_t)(n) << OSCCTRL_DFLLVAL_COARSE_SHIFT)
#define OSCCTRL_DFLLVAL_DIFF_SHIFT        (16)      /* Bits 16-31: Multiplication ratio difference */
#define OSCCTRL_DFLLVAL_DIFF_MASK         (0xffff << OSCCTRL_DFLLVAL_DIFF_SHIFT)
#  define OSCCTRL_DFLLVAL_DIFF(n)         ((n) << OSCCTRL_DFLLVAL_DIFF_SHIFT)

/* DFLL multiplier register */

#define OSCCTRL_DFLLMUL_MUL_SHIFT         (0)       /* Bits 0-15: DFLL multiply factor */
#define OSCCTRL_DFLLMUL_MUL_MASK          (0xffff << OSCCTRL_DFLLMUL_MUL_SHIFT)
#  define OSCCTRL_DFLLMUL_MUL(n)          ((uint32_t)(n) << OSCCTRL_DFLLMUL_MUL_SHIFT)
#define OSCCTRL_DFLLMUL_FSTEP_SHIFT       (16)      /* Bits 16-23: Fine maximum step */
#define OSCCTRL_DFLLMUL_FSTEP_MASK        (0xff << OSCCTRL_DFLLMUL_FSTEP_SHIFT)
#  define OSCCTRL_DFLLMUL_FSTEP(n)        ((uint32_t)(n) << OSCCTRL_DFLLMUL_FSTEP_SHIFT)
#define OSCCTRL_DFLLMUL_CSTEP_SHIFT       (26)      /* Bits 26-31: Coarse maximum step */
#define OSCCTRL_DFLLMUL_CSTEP_MASK        (0x3f << OSCCTRL_DFLLMUL_CSTEP_SHIFT)
#  define OSCCTRL_DFLLMUL_CSTEP(n)        ((uint32_t)(n) << OSCCTRL_DFLLMUL_CSTEP_SHIFT)

/* DFLL synchronization register */

#define OSCCTRL_DFLLSYNC_ENABLE           (1 << 1)  /* Bit 1: ENABLE Synchronization Busy */
#define OSCCTRL_DFLLSYNC_DFLLCTRLB        (1 << 2)  /* Bit 2: DFLLCTRLB Synchronization Busy */
#define OSCCTRL_DFLLSYNC_DFLLVAL          (1 << 3)  /* Bit 3: DFLLVAL Synchronization Busy */
#define OSCCTRL_DFLLSYNC_DFLLMUL          (1 << 4)  /* Bit 4: DFLLMUL Synchronization Busy */

/* DPLL0/1 control A */

#define OSCCTRL_DPLLCTRLA_ENABLE          (1 << 1)  /* Bit 1:  DPLL enable */
#define OSCCTRL_DPLLCTRLA_RUNSTDBY        (1 << 6)  /* Bit 6:  Run in standby */
#define OSCCTRL_DPLLCTRLA_ONDEMAND        (1 << 7)  /* Bit 7:  On demand clock activation */

/* DPLL0/1 ratio control */

#define OSCCTRL_DPLLRATIO_LDR_SHIFT       (0)     /* Bits 0-11: Loop divider ratio */
#define OSCCTRL_DPLLRATIO_LDR_MASK        (0x1fff << OSCCTRL_DPLLRATIO_LDR_SHIFT)
#  define OSCCTRL_DPLLRATIO_LDR(n)        ((uint32_t)(n) << OSCCTRL_DPLLRATIO_LDR_SHIFT)
#define OSCCTRL_DPLLRATIO_LDRFRAC_SHIFT   (16)    /* Bits 16-20: Loop divider fractional part */
#define OSCCTRL_DPLLRATIO_LDRFRAC_MASK    (31 << OSCCTRL_DPLLRATIO_LDRFRAC_SHIFT)
#  define OSCCTRL_DPLLRATIO_LDRFRAC(n)    ((uint32_t)(n) << OSCCTRL_DPLLRATIO_LDRFRAC_SHIFT)

/* DPLL0/1 control B */

#define OSCCTRL_DPLLCTRLB_FILTER_SHIFT    (0)       /* Bits 0-3: Proportional integer filter selection */
#define OSCCTRL_DPLLCTRLB_FILTER_MASK     (15 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT)
#  define OSCCTRL_DPLLCTRLB_FILTER(n)     ((uint32_t)(n) << OSCCTRL_DPLLCTRLB_FILTER_SHIFT)
                                                                                 /* PLL BW    Damping */
#  define OSCCTRL_DPLLCTRLB_FILTER0       (0  << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 92.7 kHz  0.76 */
#  define OSCCTRL_DPLLCTRLB_FILTER1       (1  << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 131 kHz   1.08 */
#  define OSCCTRL_DPLLCTRLB_FILTER2       (2  << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 46.4 kHz  0.38 */
#  define OSCCTRL_DPLLCTRLB_FILTER3       (3  << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 65.6 kHz  0.54 */
#  define OSCCTRL_DPLLCTRLB_FILTER4       (4  << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 131 kHz   0.56 */
#  define OSCCTRL_DPLLCTRLB_FILTER5       (5  << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 185 kHz   0.79 */
#  define OSCCTRL_DPLLCTRLB_FILTER6       (6  << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 65.6 kHz  0.28 */
#  define OSCCTRL_DPLLCTRLB_FILTER7       (7  << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 92.7 kHz  0.39 */
#  define OSCCTRL_DPLLCTRLB_FILTER8       (8  << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 46.4 kHz  1.49 */
#  define OSCCTRL_DPLLCTRLB_FILTER9       (9  << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 65.6 kHz  2.11 */
#  define OSCCTRL_DPLLCTRLB_FILTER10      (10 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 23.2 kHz  0.75 */
#  define OSCCTRL_DPLLCTRLB_FILTER11      (11 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 32.8 kHz  1.06 */
#  define OSCCTRL_DPLLCTRLB_FILTER12      (12 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 65.6 kHz  1.07 */
#  define OSCCTRL_DPLLCTRLB_FILTER13      (13 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 92.7 kHz  1.51 */
#  define OSCCTRL_DPLLCTRLB_FILTER14      (14 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 32.8 kHz  0.53 */
#  define OSCCTRL_DPLLCTRLB_FILTER15      (15 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* 46.4 kHz  0.75 */

#define OSCCTRL_DPLLCTRLB_WUF             (1 << 4)  /* Bit 4: Wake up fast */
#define OSCCTRL_DPLLCTRLB_REFLCK_SHIFT    (5)       /* Bits 5-7: Reference clock selection */
#define OSCCTRL_DPLLCTRLB_REFLCK_MASK     (3 << OSCCTRL_DPLLCTRLB_REFLCK_SHIFT)
#  define OSCCTRL_DPLLCTRLB_REFLCK(n)     ((uint32_t)(n) << OSCCTRL_DPLLCTRLB_REFLCK_SHIFT)
#  define OSCCTRL_DPLLCTRLB_REFLCK_GCLK   (0 << OSCCTRL_DPLLCTRLB_REFLCK_SHIFT) /* Dedicated GCLK clock reference */
#  define OSCCTRL_DPLLCTRLB_REFLCK_XOSC32 (1 << OSCCTRL_DPLLCTRLB_REFLCK_SHIFT) /* XOSC32K clock reference (default) */
#  define OSCCTRL_DPLLCTRLB_REFLCK_XOSC0  (2 << OSCCTRL_DPLLCTRLB_REFLCK_SHIFT) /* XOSC0 clock reference */
#  define OSCCTRL_DPLLCTRLB_REFLCK_XOSC1  (3 << OSCCTRL_DPLLCTRLB_REFLCK_SHIFT) /* XOSC2 clock reference */

#define OSCCTRL_DPLLCTRLB_LTIME_SHIFT     (8)       /* Bits 8-10: Lock time */
#define OSCCTRL_DPLLCTRLB_LTIME_MASK      (7 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT)
#  define OSCCTRL_DPLLCTRLB_LTIME(n)      ((uint32_t)(n) << OSCCTRL_DPLLCTRLB_LTIME_SHIFT)
#  define OSCCTRL_DPLLCTRLB_LTIME_NONE    (0 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT) /* No time-out. Automatic lock */
#  define OSCCTRL_DPLLCTRLB_LTIME_800US   (4 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no locka within 800 us */
#  define OSCCTRL_DPLLCTRLB_LTIME_900US   (5 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no locka within 900 us */
#  define OSCCTRL_DPLLCTRLB_LTIME_1MS     (6 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no locka within 1MS */
#  define OSCCTRL_DPLLCTRLB_LTIME_1p1MS   (7 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no locka within 1.1MS */

#define OSCCTRL_DPLLCTRLB_LBYPASS         (1 << 11) /* Bit 11: Lock bypass */
#define OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT (12)      /* Bits 12-14: Sigma-Delta DCO Filter Selection */
#define OSCCTRL_DPLLCTRLB_DCOFILTER_MASK  (7 << OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT)
#  define OSCCTRL_DPLLCTRLB_DCOFILTER(n)  ((uint32_t)(n) << OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT)
                                                                                   /* Capa pF    BW MHz */
#  define OSCCTRL_DPLLCTRLB_DCOFILTER0    (0 << OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT) /* 0.5        3.21 */
#  define OSCCTRL_DPLLCTRLB_DCOFILTER1    (1 << OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT) /* 1          1.6 */
#  define OSCCTRL_DPLLCTRLB_DCOFILTER2    (2 << OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT) /* 1.5        1.1 */
#  define OSCCTRL_DPLLCTRLB_DCOFILTER3    (3 << OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT) /* 2          0.8 */
#  define OSCCTRL_DPLLCTRLB_DCOFILTER4    (4 << OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT) /* 2.5        0.64 */
#  define OSCCTRL_DPLLCTRLB_DCOFILTER5    (5 << OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT) /* 3          0.55 */
#  define OSCCTRL_DPLLCTRLB_DCOFILTER6    (6 << OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT) /* 3.5        0.45 */
#  define OSCCTRL_DPLLCTRLB_DCOFILTER7    (7 << OSCCTRL_DPLLCTRLB_DCOFILTER_SHIFT) /* 4          0.4 */

#define OSCCTRL_DPLLCTRLB_DCOEN           (1 << 15) /* Bit 15: DCO Filter Enable */
#define OSCCTRL_DPLLCTRLB_DIV_SHIFT       (16)      /* Bits 16-26: Clock divider */
#define OSCCTRL_DPLLCTRLB_DIV_MASK        (0x7ff << OSCCTRL_DPLLCTRLB_DIV_SHIFT)
#  define OSCCTRL_DPLLCTRLB_DIV(n)        ((uint32_t)(n) << OSCCTRL_DPLLCTRLB_DIV_SHIFT)

/* DPLL0/1 synchronization busy */

#define OSCCTRL_DPLLSYNCBUSY_ENABLE       (1 << 1)  /* Bit 1:  DPLL enable synchronization status */
#define OSCCTRL_DPLLSYNCBUSY_DPLLRATIO    (1 << 2)  /* Bit 2:  DPLL loop divider ratio synchronization status */

/* DPLL0/1 status */

#define OSCCTRL_DPLLSTATUS_LOCK           (1 << 0)  /* Bit 0:  DPLL lock status */
#define OSCCTRL_DPLLSTATUS_CLKRDY         (1 << 1)  /* Bit 1:  Output clock ready */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_OSCCTRL_H */
