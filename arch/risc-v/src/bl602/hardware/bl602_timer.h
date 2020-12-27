/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_timer.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_TIMER_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_TIMER_TCCR_OFFSET     0x000000  /* TCCR */
#define BL602_TIMER_TMR2_0_OFFSET   0x000010  /* TMR2_0 */
#define BL602_TIMER_TMR2_1_OFFSET   0x000014  /* TMR2_1 */
#define BL602_TIMER_TMR2_2_OFFSET   0x000018  /* TMR2_2 */
#define BL602_TIMER_TMR3_0_OFFSET   0x00001c  /* TMR3_0 */
#define BL602_TIMER_TMR3_1_OFFSET   0x000020  /* TMR3_1 */
#define BL602_TIMER_TMR3_2_OFFSET   0x000024  /* TMR3_2 */
#define BL602_TIMER_TCR2_OFFSET     0x00002c  /* TCR2 */
#define BL602_TIMER_TCR3_OFFSET     0x000030  /* TCR3 */
#define BL602_TIMER_TMSR2_OFFSET    0x000038  /* TMSR2 */
#define BL602_TIMER_TMSR3_OFFSET    0x00003c  /* TMSR3 */
#define BL602_TIMER_TIER2_OFFSET    0x000044  /* TIER2 */
#define BL602_TIMER_TIER3_OFFSET    0x000048  /* TIER3 */
#define BL602_TIMER_TPLVR2_OFFSET   0x000050  /* TPLVR2 */
#define BL602_TIMER_TPLVR3_OFFSET   0x000054  /* TPLVR3 */
#define BL602_TIMER_TPLCR2_OFFSET   0x00005c  /* TPLCR2 */
#define BL602_TIMER_TPLCR3_OFFSET   0x000060  /* TPLCR3 */
#define BL602_TIMER_WMER_OFFSET     0x000064  /* WMER */
#define BL602_TIMER_WMR_OFFSET      0x000068  /* WMR */
#define BL602_TIMER_WVR_OFFSET      0x00006c  /* WVR */
#define BL602_TIMER_WSR_OFFSET      0x000070  /* WSR */
#define BL602_TIMER_TICR2_OFFSET    0x000078  /* TICR2 */
#define BL602_TIMER_TICR3_OFFSET    0x00007c  /* TICR3 */
#define BL602_TIMER_WICR_OFFSET     0x000080  /* WICR */
#define BL602_TIMER_TCER_OFFSET     0x000084  /* TCER */
#define BL602_TIMER_TCMR_OFFSET     0x000088  /* TCMR */
#define BL602_TIMER_TILR2_OFFSET    0x000090  /* TILR2 */
#define BL602_TIMER_TILR3_OFFSET    0x000094  /* TILR3 */
#define BL602_TIMER_WCR_OFFSET      0x000098  /* WCR */
#define BL602_TIMER_WFAR_OFFSET     0x00009c  /* WFAR */
#define BL602_TIMER_WSAR_OFFSET     0x0000a0  /* WSAR */
#define BL602_TIMER_TCVWR2_OFFSET   0x0000a8  /* TCVWR2 */
#define BL602_TIMER_TCVWR3_OFFSET   0x0000ac  /* TCVWR3 */
#define BL602_TIMER_TCVSYN2_OFFSET  0x0000b4  /* TCVSYN2 */
#define BL602_TIMER_TCVSYN3_OFFSET  0x0000b8  /* TCVSYN3 */
#define BL602_TIMER_TCDR_OFFSET     0x0000bc  /* TCDR */

/* Register definitions *****************************************************/

#define BL602_TIMER_TCCR     (BL602_TIMER_BASE + BL602_TIMER_TCCR_OFFSET)
#define BL602_TIMER_TMR2_0   (BL602_TIMER_BASE + BL602_TIMER_TMR2_0_OFFSET)
#define BL602_TIMER_TMR2_1   (BL602_TIMER_BASE + BL602_TIMER_TMR2_1_OFFSET)
#define BL602_TIMER_TMR2_2   (BL602_TIMER_BASE + BL602_TIMER_TMR2_2_OFFSET)
#define BL602_TIMER_TMR3_0   (BL602_TIMER_BASE + BL602_TIMER_TMR3_0_OFFSET)
#define BL602_TIMER_TMR3_1   (BL602_TIMER_BASE + BL602_TIMER_TMR3_1_OFFSET)
#define BL602_TIMER_TMR3_2   (BL602_TIMER_BASE + BL602_TIMER_TMR3_2_OFFSET)
#define BL602_TIMER_TCR2     (BL602_TIMER_BASE + BL602_TIMER_TCR2_OFFSET)
#define BL602_TIMER_TCR3     (BL602_TIMER_BASE + BL602_TIMER_TCR3_OFFSET)
#define BL602_TIMER_TMSR2    (BL602_TIMER_BASE + BL602_TIMER_TMSR2_OFFSET)
#define BL602_TIMER_TMSR3    (BL602_TIMER_BASE + BL602_TIMER_TMSR3_OFFSET)
#define BL602_TIMER_TIER2    (BL602_TIMER_BASE + BL602_TIMER_TIER2_OFFSET)
#define BL602_TIMER_TIER3    (BL602_TIMER_BASE + BL602_TIMER_TIER3_OFFSET)
#define BL602_TIMER_TPLVR2   (BL602_TIMER_BASE + BL602_TIMER_TPLVR2_OFFSET)
#define BL602_TIMER_TPLVR3   (BL602_TIMER_BASE + BL602_TIMER_TPLVR3_OFFSET)
#define BL602_TIMER_TPLCR2   (BL602_TIMER_BASE + BL602_TIMER_TPLCR2_OFFSET)
#define BL602_TIMER_TPLCR3   (BL602_TIMER_BASE + BL602_TIMER_TPLCR3_OFFSET)
#define BL602_TIMER_WMER     (BL602_TIMER_BASE + BL602_TIMER_WMER_OFFSET)
#define BL602_TIMER_WMR      (BL602_TIMER_BASE + BL602_TIMER_WMR_OFFSET)
#define BL602_TIMER_WVR      (BL602_TIMER_BASE + BL602_TIMER_WVR_OFFSET)
#define BL602_TIMER_WSR      (BL602_TIMER_BASE + BL602_TIMER_WSR_OFFSET)
#define BL602_TIMER_TICR2    (BL602_TIMER_BASE + BL602_TIMER_TICR2_OFFSET)
#define BL602_TIMER_TICR3    (BL602_TIMER_BASE + BL602_TIMER_TICR3_OFFSET)
#define BL602_TIMER_WICR     (BL602_TIMER_BASE + BL602_TIMER_WICR_OFFSET)
#define BL602_TIMER_TCER     (BL602_TIMER_BASE + BL602_TIMER_TCER_OFFSET)
#define BL602_TIMER_TCMR     (BL602_TIMER_BASE + BL602_TIMER_TCMR_OFFSET)
#define BL602_TIMER_TILR2    (BL602_TIMER_BASE + BL602_TIMER_TILR2_OFFSET)
#define BL602_TIMER_TILR3    (BL602_TIMER_BASE + BL602_TIMER_TILR3_OFFSET)
#define BL602_TIMER_WCR      (BL602_TIMER_BASE + BL602_TIMER_WCR_OFFSET)
#define BL602_TIMER_WFAR     (BL602_TIMER_BASE + BL602_TIMER_WFAR_OFFSET)
#define BL602_TIMER_WSAR     (BL602_TIMER_BASE + BL602_TIMER_WSAR_OFFSET)
#define BL602_TIMER_TCVWR2   (BL602_TIMER_BASE + BL602_TIMER_TCVWR2_OFFSET)
#define BL602_TIMER_TCVWR3   (BL602_TIMER_BASE + BL602_TIMER_TCVWR3_OFFSET)
#define BL602_TIMER_TCVSYN2  (BL602_TIMER_BASE + BL602_TIMER_TCVSYN2_OFFSET)
#define BL602_TIMER_TCVSYN3  (BL602_TIMER_BASE + BL602_TIMER_TCVSYN3_OFFSET)
#define BL602_TIMER_TCDR     (BL602_TIMER_BASE + BL602_TIMER_TCDR_OFFSET)

/* Register bit definitions *************************************************/

#define TIMER_TCCR_CS_WDT_SHIFT  (8)
#define TIMER_TCCR_CS_WDT_MASK   (0x03 << TIMER_TCCR_CS_WDT_SHIFT)
#define TIMER_TCCR_CS_2_SHIFT    (5)
#define TIMER_TCCR_CS_2_MASK     (0x03 << TIMER_TCCR_CS_2_SHIFT)
#define TIMER_TCCR_CS_1_SHIFT    (2)
#define TIMER_TCCR_CS_1_MASK     (0x03 << TIMER_TCCR_CS_1_SHIFT)

#define TIMER_TMSR2_TMSR_2       (1 << 2)
#define TIMER_TMSR2_TMSR_1       (1 << 1)
#define TIMER_TMSR2_TMSR_0       (1 << 0)

#define TIMER_TMSR3_TMSR_2       (1 << 2)
#define TIMER_TMSR3_TMSR_1       (1 << 1)
#define TIMER_TMSR3_TMSR_0       (1 << 0)

#define TIMER_TIER2_TIER_2       (1 << 2)
#define TIMER_TIER2_TIER_1       (1 << 1)
#define TIMER_TIER2_TIER_0       (1 << 0)

#define TIMER_TIER3_TIER_2       (1 << 2)
#define TIMER_TIER3_TIER_1       (1 << 1)
#define TIMER_TIER3_TIER_0       (1 << 0)

#define TIMER_TPLCR2_TPLCR_MASK  (0x03)

#define TIMER_TPLCR3_TPLCR_MASK  (0x03)

#define TIMER_WMER_WRIE          (1 << 1)
#define TIMER_WMER_WE            (1 << 0)

#define TIMER_WMR_WMR_MASK       (0xffff)

#define TIMER_WVR_WVR_MASK       (0xffff)

#define TIMER_WSR_WTS            (1 << 0)

#define TIMER_TICR2_TCLR_2       (1 << 2)
#define TIMER_TICR2_TCLR_1       (1 << 1)
#define TIMER_TICR2_TCLR_0       (1 << 0)

#define TIMER_TICR3_TCLR_2       (1 << 2)
#define TIMER_TICR3_TCLR_1       (1 << 1)
#define TIMER_TICR3_TCLR_0       (1 << 0)

#define TIMER_WICR_WICLR         (1 << 0)

#define TIMER_TCER_TIMER3_EN     (1 << 2)
#define TIMER_TCER_TIMER2_EN     (1 << 1)

#define TIMER_TCMR_TIMER3_MODE   (1 << 2)
#define TIMER_TCMR_TIMER2_MODE   (1 << 1)

#define TIMER_TILR2_TILR_2       (1 << 2)
#define TIMER_TILR2_TILR_1       (1 << 1)
#define TIMER_TILR2_TILR_0       (1 << 0)

#define TIMER_TILR3_TILR_2       (1 << 2)
#define TIMER_TILR3_TILR_1       (1 << 1)
#define TIMER_TILR3_TILR_0       (1 << 0)

#define TIMER_WCR_WCR            (1 << 0)

#define TIMER_WFAR_MASK          (0xffff)

#define TIMER_WSAR_MASK          (0xffff)

#define TIMER_TCDR_WCDR_SHIFT    (24)
#define TIMER_TCDR_WCDR_MASK     (0xff << TIMER_TCDR_WCDR_SHIFT)
#define TIMER_TCDR_TCDR3_SHIFT   (16)
#define TIMER_TCDR_TCDR3_MASK    (0xff << TIMER_TCDR_TCDR3_SHIFT)
#define TIMER_TCDR_TCDR2_SHIFT   (8)
#define TIMER_TCDR_TCDR2_MASK    (0xff << TIMER_TCDR_TCDR2_SHIFT)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_TIMER_H */
