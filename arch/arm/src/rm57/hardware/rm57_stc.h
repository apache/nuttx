/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_stc.h
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

/* Self-Test Controller (STC), SPNU562A chapter 10.
 *
 * STC1 runs LBIST on the two lockstep Cortex-R5F cores (segment 0, 125
 * intervals) and on the uSCU (segment 1, 3 intervals).  STC2 does the same
 * for the two N2HETs (57 intervals).  A CPU self-test always ends in a CPU
 * reset - not a system reset - so STCGSTAT/STCFSTAT/STCSCSCR survive it and
 * are how the reset handler finds out what just ran.
 *
 * Offsets follow TRM Table 10-8.  HALCoGen's HL_reg_stc.h names 0x0c
 * STCCADDR and 0x40 STCCADDR1; the TRM calls them STCCADDR1 (core 1) and
 * STCCADDR2 (core 2), which is what is used here.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_STC_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_STC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RM57_STC_GCR0_OFFSET        0x0000 /* Global Control Register 0 */
#define RM57_STC_GCR1_OFFSET        0x0004 /* Global Control Register 1 */

/* Timeout Counter Preload Register */
#define RM57_STC_TPR_OFFSET         0x0008
#define RM57_STC_CADDR1_OFFSET      0x000c /* Current ROM Address, core 1 */

/* Current Interval Count Register */
#define RM57_STC_CICR_OFFSET        0x0010
#define RM57_STC_GSTAT_OFFSET       0x0014 /* Global Status Register */
#define RM57_STC_FSTAT_OFFSET       0x0018 /* Fail Status Register */

/* Current MISR registers, n=0..3 */

#define RM57_STC_CORE1_MISR_OFFSET(n) (0x001c + ((n) << 2))
#define RM57_STC_CORE2_MISR_OFFSET(n) (0x002c + ((n) << 2))

#define RM57_STC_SCSCR_OFFSET       0x003c /* Signature Compare Self-Check */
#define RM57_STC_CADDR2_OFFSET      0x0040 /* Current ROM Address, core 2 */
#define RM57_STC_CLKDIV_OFFSET      0x0044 /* Clock Divider Register */

/* Segment First Preload Register */
#define RM57_STC_SEGPLR_OFFSET      0x0048

/* Register Addresses *******************************************************/

#define RM57_STC1_GCR0              (RM57_STC1_BASE + RM57_STC_GCR0_OFFSET)
#define RM57_STC1_GCR1              (RM57_STC1_BASE + RM57_STC_GCR1_OFFSET)
#define RM57_STC1_TPR               (RM57_STC1_BASE + RM57_STC_TPR_OFFSET)
#define RM57_STC1_CADDR1            (RM57_STC1_BASE + RM57_STC_CADDR1_OFFSET)
#define RM57_STC1_CICR              (RM57_STC1_BASE + RM57_STC_CICR_OFFSET)
#define RM57_STC1_GSTAT             (RM57_STC1_BASE + RM57_STC_GSTAT_OFFSET)
#define RM57_STC1_FSTAT             (RM57_STC1_BASE + RM57_STC_FSTAT_OFFSET)
#define RM57_STC1_SCSCR             (RM57_STC1_BASE + RM57_STC_SCSCR_OFFSET)
#define RM57_STC1_CADDR2            (RM57_STC1_BASE + RM57_STC_CADDR2_OFFSET)
#define RM57_STC1_CLKDIV            (RM57_STC1_BASE + RM57_STC_CLKDIV_OFFSET)
#define RM57_STC1_SEGPLR            (RM57_STC1_BASE + RM57_STC_SEGPLR_OFFSET)

#define RM57_STC2_GCR0              (RM57_STC2_BASE + RM57_STC_GCR0_OFFSET)
#define RM57_STC2_GCR1              (RM57_STC2_BASE + RM57_STC_GCR1_OFFSET)
#define RM57_STC2_TPR               (RM57_STC2_BASE + RM57_STC_TPR_OFFSET)
#define RM57_STC2_GSTAT             (RM57_STC2_BASE + RM57_STC_GSTAT_OFFSET)
#define RM57_STC2_FSTAT             (RM57_STC2_BASE + RM57_STC_FSTAT_OFFSET)
#define RM57_STC2_SCSCR             (RM57_STC2_BASE + RM57_STC_SCSCR_OFFSET)
#define RM57_STC2_CLKDIV            (RM57_STC2_BASE + RM57_STC_CLKDIV_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Global Control Register 0 */

#define STC_GCR0_RS_CNT_MASK        (3 << 0)  /* Bits 0-1: Restart/continue */

/* Continue from last interval */
#  define STC_GCR0_RS_CNT_CONTINUE  (0 << 0)
#  define STC_GCR0_RS_CNT_RESTART   (1 << 0)  /* Restart from interval 0 */

/* Bits 8-10: Capture idle cycle */
#define STC_GCR0_CAP_IDLE_SHIFT     (8)
#define STC_GCR0_CAP_IDLE_MASK      (7 << STC_GCR0_CAP_IDLE_SHIFT)
#define STC_GCR0_INTCOUNT_SHIFT     (16)      /* Bits 16-31: Interval count */
#define STC_GCR0_INTCOUNT_MASK      (0xffff << STC_GCR0_INTCOUNT_SHIFT)
#  define STC_GCR0_INTCOUNT(n)      ((uint32_t)(n) << STC_GCR0_INTCOUNT_SHIFT)

/* Global Control Register 1.  SEG0_CORE_SEL must be written before STC_ENA
 * and ignores further writes until the run completes.
 */

#define STC_GCR1_ENA_MASK           (0x0f << 0) /* Bits 0-3: Enable key */
#  define STC_GCR1_ENA_ENABLE       (0x0a << 0)
#  define STC_GCR1_ENA_DISABLE      (0x05 << 0)

/* Bits 8-11: Segment 0 cores */
#define STC_GCR1_CORE_SEL_MASK      (0x0f << 8)
#  define STC_GCR1_CORE_SEL_BOTH    (0x00 << 8) /* Both cores in parallel */
#  define STC_GCR1_CORE_SEL_CORE1   (0x05 << 8)
#  define STC_GCR1_CORE_SEL_CORE2   (0x0a << 8)

/* Timeout Counter Preload Register.  Errata STC#26: reverts to its reset
 * value at the end of every run, so it has to be rewritten each time.
 */

#define STC_TPR_MAX                 (0xffffffff)

/* Current Interval Count Register */

#define STC_CICR_CORE1_MASK         (0xffff << 0)
#define STC_CICR_CORE2_SHIFT        (16)
#define STC_CICR_CORE2_MASK         (0xffff << STC_CICR_CORE2_SHIFT)

/* Global Status Register (both flags W1C, reset only by power-on reset) */

#define STC_GSTAT_TEST_DONE         (1 << 0)  /* Bit 0: Run completed */
#define STC_GSTAT_TEST_FAIL         (1 << 1)  /* Bit 1: Run failed */

/* Bits 8-11: 0xa while a run is active */
#define STC_GSTAT_ST_ACTIVE_MASK    (0x0f << 8)
#  define STC_GSTAT_ST_ACTIVE       (0x0a << 8)

/* Fail Status Register (flags W1C, reset only by power-on reset) */

/* Bit 0: Core 1 MISR mismatch */
#define STC_FSTAT_CORE1_FAIL        (1 << 0)

/* Bit 1: Core 2 MISR mismatch */
#define STC_FSTAT_CORE2_FAIL        (1 << 1)
#define STC_FSTAT_TO_ERR            (1 << 2)  /* Bit 2: Timeout */
#define STC_FSTAT_FSEG_ID_SHIFT     (3)       /* Bits 3-4: Failed segment */
#define STC_FSTAT_FSEG_ID_MASK      (3 << STC_FSTAT_FSEG_ID_SHIFT)

/* Signature Compare Self-Check Register (reset by system reset, so it
 * survives the CPU reset that ends a run)
 */

#define STC_SCSCR_KEY_MASK          (0x0f << 0) /* Bits 0-3: Self-check key */
#  define STC_SCSCR_KEY_ENABLE      (0x0a << 0)
#  define STC_SCSCR_KEY_DISABLE     (0x05 << 0)

/* Bit 4: Insert stuck-at fault */
#define STC_SCSCR_FAULT_INS         (1 << 4)

/* Clock Divider Register.  Only honoured while the CLKDIV field of the
 * system module's STCLKDIV register (SYS2, 0xffffe108) is zero.
 */

/* Bits 16-18: Segment 1, /(n+1) */
#define STC_CLKDIV_CLKDIV1_SHIFT    (16)
#define STC_CLKDIV_CLKDIV1_MASK     (7 << STC_CLKDIV_CLKDIV1_SHIFT)
#  define STC_CLKDIV_CLKDIV1(n) ((uint32_t)(n) << STC_CLKDIV_CLKDIV1_SHIFT)

/* Bits 24-26: Segment 0, /(n+1) */
#define STC_CLKDIV_CLKDIV0_SHIFT    (24)
#define STC_CLKDIV_CLKDIV0_MASK     (7 << STC_CLKDIV_CLKDIV0_SHIFT)
#  define STC_CLKDIV_CLKDIV0(n) ((uint32_t)(n) << STC_CLKDIV_CLKDIV0_SHIFT)

/* Segment First Preload Register */

#define STC_SEGPLR_SEGID_MASK       (3 << 0)  /* Bits 0-1: First segment */

/* Interval counts (TRM Table 10-1), STCCLK ceiling (SPNS215C 6.5.5.2) */

#define STC1_SEG0_NINTERVALS        (125)
#define STC1_SEG1_NINTERVALS        (3)
#define STC2_NINTERVALS             (57)
#define STC_MAX_STCCLK              (110000000)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_STC_H */
