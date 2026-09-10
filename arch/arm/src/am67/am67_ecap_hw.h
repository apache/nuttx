/****************************************************************************
 * arch/arm/src/am67/am67_ecap_hw.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_ECAP_HW_H
#define __ARCH_ARM_SRC_AM67_AM67_ECAP_HW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Base addresses (MAIN domain).  Registers are 32-bit and there is no clock
 * gate; liveness is proven via the PID readback below.
 */

#define AM67_ECAP0_BASE                 0x23100000
#define AM67_ECAP1_BASE                 0x23110000
#define AM67_ECAP2_BASE                 0x23120000

/* Peripheral ID.  Reset value not yet copper-verified; the driver only
 * treats 0 / 0xffffffff as a dead module.
 */

#define AM67_ECAP_PID_OFFSET            0x05c

/* Register offsets (all 32-bit) ********************************************
 *
 * In APWM mode CAP1/CAP2 are the active period/compare (APRD/ACMP) and
 * CAP3/CAP4 are their shadows.  TRM 12.4.2.4.1.2.
 */

#define AM67_ECAP_TSCNT_OFFSET          0x000   /* Time-base counter (TSCTR) */
#define AM67_ECAP_CNTPHS_OFFSET         0x004   /* Counter phase (sync load) */
#define AM67_ECAP_CAP1_OFFSET           0x008   /* CAP1 / APRD active period */
#define AM67_ECAP_CAP2_OFFSET           0x00c   /* CAP2 / ACMP active compare*/
#define AM67_ECAP_CAP3_OFFSET           0x010   /* CAP3 / APRD shadow        */
#define AM67_ECAP_CAP4_OFFSET           0x014   /* CAP4 / ACMP shadow        */
#define AM67_ECAP_ECCTL_OFFSET          0x028   /* Control (ECCTL1 + ECCTL2) */
#define AM67_ECAP_ECINT_EN_FLG_OFFSET   0x02c   /* Int enable + flags        */
#define AM67_ECAP_ECINT_CLR_FRC_OFFSET  0x030   /* Int clear + force         */

/* ECCTL - Control register *************************************************
 *
 * TI folds the classic C2000 ECCTL1 (low half) and ECCTL2 (high half) into
 * one 32-bit word.  The APWM path uses only CAP_APWM, APWMPOL, TSCNTSTP and
 * SYNCO_SEL; the capture-side fields are kept for completeness.
 */

/* Per-event edge polarity: 0 = rising, 1 = falling (capture mode only) */

#define AM67_ECAP_ECCTL_CAP1POL         (1u << 0)
#define AM67_ECAP_ECCTL_CAP2POL         (1u << 2)
#define AM67_ECAP_ECCTL_CAP3POL         (1u << 4)
#define AM67_ECAP_ECCTL_CAP4POL         (1u << 6)

/* Per-event counter reset (capture mode only): 0 = absolute, 1 = delta */

#define AM67_ECAP_ECCTL_CTRRST1         (1u << 1)
#define AM67_ECAP_ECCTL_CTRRST2         (1u << 3)
#define AM67_ECAP_ECCTL_CTRRST3         (1u << 5)
#define AM67_ECAP_ECCTL_CTRRST4         (1u << 7)

#define AM67_ECAP_ECCTL_CAPLDEN         (1u << 8)   /* Enable CAPx loading   */

#define AM67_ECAP_ECCTL_EVTFLTPS_SHIFT  (9)         /* Event filter prescale */
#define AM67_ECAP_ECCTL_EVTFLTPS_MASK   (0x1fu << 9)

#define AM67_ECAP_ECCTL_FREE_SOFT_SHIFT (14)        /* Emulation halt mode   */
#define AM67_ECAP_ECCTL_FREE_SOFT_MASK  (3u << 14)

#define AM67_ECAP_ECCTL_CONT_ONESHT     (1u << 16)  /* 0=continuous 1=oneshot*/

#define AM67_ECAP_ECCTL_STOPVALUE_SHIFT (17)        /* Wrap/stop after event */
#define AM67_ECAP_ECCTL_STOPVALUE_MASK  (3u << 17)
#define AM67_ECAP_ECCTL_STOPVALUE_EVT1  (0u << 17)
#define AM67_ECAP_ECCTL_STOPVALUE_EVT2  (1u << 17)
#define AM67_ECAP_ECCTL_STOPVALUE_EVT3  (2u << 17)
#define AM67_ECAP_ECCTL_STOPVALUE_EVT4  (3u << 17)

#define AM67_ECAP_ECCTL_REARM           (1u << 19)  /* Re-arm sequencer      */
#define AM67_ECAP_ECCTL_TSCNTSTP        (1u << 20)  /* 0=stop 1=run counter  */
#define AM67_ECAP_ECCTL_SYNCI_EN        (1u << 21)  /* Sync-in enable        */

#define AM67_ECAP_ECCTL_SYNCO_SEL_SHIFT (22)        /* 0=SYNCI 1=CTR=PRD 2/3=off */
#define AM67_ECAP_ECCTL_SYNCO_SEL_MASK  (3u << 22)
#define AM67_ECAP_ECCTL_SYNCO_SEL_DIS   (2u << 22)  /* Disable sync-out       */

#define AM67_ECAP_ECCTL_SWSYNC          (1u << 24)  /* Software sync         */
#define AM67_ECAP_ECCTL_CAP_APWM        (1u << 25)  /* 0=capture 1=APWM      */
#define AM67_ECAP_ECCTL_APWMPOL         (1u << 26)  /* 0=active-hi 1=active-lo*/

#define AM67_ECAP_ECCTL_FILTER_SHIFT    (27)
#define AM67_ECAP_ECCTL_FILTER_MASK     (0x1fu << 27)

/* ECINT_EN_FLG - Interrupt enable (low) + flags (high) *********************
 *
 * Unused by the basic APWM output; kept for a future period-callback.  In
 * APWM mode the valid sources are PRDEQ (CTR = PRD) and CMPEQ (CTR = CMP).
 */

/* Enable bits (write 1 to enable that source's interrupt) */

#define AM67_ECAP_INT_CEVT1_EN          (1u << 1)
#define AM67_ECAP_INT_CEVT2_EN          (1u << 2)
#define AM67_ECAP_INT_CEVT3_EN          (1u << 3)
#define AM67_ECAP_INT_CEVT4_EN          (1u << 4)
#define AM67_ECAP_INT_CNTOVF_EN         (1u << 5)
#define AM67_ECAP_INT_PRDEQ_EN          (1u << 6)   /* CTR = PRD (APWM)      */
#define AM67_ECAP_INT_CMPEQ_EN          (1u << 7)   /* CTR = CMP (APWM)      */

/* Flag bits (read-only status) */

#define AM67_ECAP_INT_FLG               (1u << 16)  /* Global interrupt flag */
#define AM67_ECAP_INT_CEVT1_FLG         (1u << 17)
#define AM67_ECAP_INT_CEVT2_FLG         (1u << 18)
#define AM67_ECAP_INT_CEVT3_FLG         (1u << 19)
#define AM67_ECAP_INT_CEVT4_FLG         (1u << 20)
#define AM67_ECAP_INT_CNTOVF_FLG        (1u << 21)
#define AM67_ECAP_INT_PRDEQ_FLG         (1u << 22)
#define AM67_ECAP_INT_CMPEQ_FLG         (1u << 23)

/* ECINT_CLR_FRC - Interrupt clear (low) + force (high) *********************
 *
 * Writing INT_CLR (bit 0) de-asserts the global interrupt and must be done
 * in any ISR before another can assert.
 */

#define AM67_ECAP_INT_CLR               (1u << 0)   /* Global interrupt clear*/
#define AM67_ECAP_INT_CEVT1_CLR         (1u << 1)
#define AM67_ECAP_INT_CEVT2_CLR         (1u << 2)
#define AM67_ECAP_INT_CEVT3_CLR         (1u << 3)
#define AM67_ECAP_INT_CEVT4_CLR         (1u << 4)
#define AM67_ECAP_INT_CNTOVF_CLR        (1u << 5)
#define AM67_ECAP_INT_PRDEQ_CLR         (1u << 6)
#define AM67_ECAP_INT_CMPEQ_CLR         (1u << 7)

/* Convenience mask: clear every event flag plus the global flag */

#define AM67_ECAP_INT_CLR_ALL           (AM67_ECAP_INT_CLR       | \
                                         AM67_ECAP_INT_CEVT1_CLR | \
                                         AM67_ECAP_INT_CEVT2_CLR | \
                                         AM67_ECAP_INT_CEVT3_CLR | \
                                         AM67_ECAP_INT_CEVT4_CLR | \
                                         AM67_ECAP_INT_CNTOVF_CLR | \
                                         AM67_ECAP_INT_PRDEQ_CLR | \
                                         AM67_ECAP_INT_CMPEQ_CLR)

#endif /* __ARCH_ARM_SRC_AM67_AM67_ECAP_HW_H */
