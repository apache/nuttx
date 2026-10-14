/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_ccm.h
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

/* CPU Compare Module for Cortex-R5F (CCM-R5F), SPNU562A chapter 13.
 *
 * Four independent diagnostics share one register layout, each with its
 * own status/key register pair:
 *
 *   1 - CPU output compare           (ESM group2 channel 2)
 *   2 - VIM output compare           (ESM group2 channel 25)
 *   3 - checker CPU inactivity       (ESM group2 channel 26)
 *   4 - power domain inactivity      (ESM group2 channel 28)
 *
 * Offsets follow TRM Table 13-7 and match HALCoGen's HL_reg_ccmr5.h.  Note
 * that the pairs are not evenly spaced: CCMPOLCNTRL sits between pair 3
 * and pair 4.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_CCM_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Status Register 1 (CPU compare) */
#define RM57_CCM_SR1_OFFSET         0x0000
#define RM57_CCM_KEYR1_OFFSET       0x0004 /* Key Register 1 */

/* Status Register 2 (VIM compare) */
#define RM57_CCM_SR2_OFFSET         0x0008
#define RM57_CCM_KEYR2_OFFSET       0x000c /* Key Register 2 */

/* Status Register 3 (inactivity) */
#define RM57_CCM_SR3_OFFSET         0x0010
#define RM57_CCM_KEYR3_OFFSET       0x0014 /* Key Register 3 */
#define RM57_CCM_POLCNTRL_OFFSET    0x0018 /* Polarity Control Register */

/* Status Register 4 (power domain) */
#define RM57_CCM_SR4_OFFSET         0x001c
#define RM57_CCM_KEYR4_OFFSET       0x0020 /* Key Register 4 */

/* Power Domain Status Register 0 */
#define RM57_CCM_PDSTAT0_OFFSET     0x0024

/* Register Addresses *******************************************************/

#define RM57_CCM_SR1                (RM57_CCM_BASE + RM57_CCM_SR1_OFFSET)
#define RM57_CCM_KEYR1              (RM57_CCM_BASE + RM57_CCM_KEYR1_OFFSET)
#define RM57_CCM_SR2                (RM57_CCM_BASE + RM57_CCM_SR2_OFFSET)
#define RM57_CCM_KEYR2              (RM57_CCM_BASE + RM57_CCM_KEYR2_OFFSET)
#define RM57_CCM_SR3                (RM57_CCM_BASE + RM57_CCM_SR3_OFFSET)
#define RM57_CCM_KEYR3              (RM57_CCM_BASE + RM57_CCM_KEYR3_OFFSET)
#define RM57_CCM_POLCNTRL           (RM57_CCM_BASE + RM57_CCM_POLCNTRL_OFFSET)
#define RM57_CCM_SR4                (RM57_CCM_BASE + RM57_CCM_SR4_OFFSET)
#define RM57_CCM_KEYR4              (RM57_CCM_BASE + RM57_CCM_KEYR4_OFFSET)
#define RM57_CCM_PDSTAT0            (RM57_CCM_BASE + RM57_CCM_PDSTAT0_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Status Registers 1-4 (identical layout) */

#define CCM_SR_STE                  (1 << 0)  /* Bit 0: Self-test error */

/* Bit 1: Error type, 1=mismatch test */
#define CCM_SR_STET                 (1 << 1)
#define CCM_SR_STC                  (1 << 8)  /* Bit 8: Self-test complete */

/* Bit 16: Compare error (W1C) */
#define CCM_SR_CMPE                 (1 << 16)

/* Key Registers 1-4 (identical layout).  Error forcing and self-test error
 * forcing last one clock cycle and then revert to CCM_KEYR_LOCKSTEP on
 * their own; self-test stays selected until software writes another key.
 */

#define CCM_KEYR_MKEY_MASK          (0x0f)    /* Bits 0-3: Mode key */
#define CCM_KEYR_LOCKSTEP           (0x00)    /* Active compare lockstep */
#define CCM_KEYR_SELFTEST           (0x06)    /* Self-test */
#define CCM_KEYR_ERRFORCE           (0x09)    /* Error forcing */
#define CCM_KEYR_STERRFORCE         (0x0f)    /* Self-test error forcing */

/* Polarity Control Register */

/* Bits 0-7: CPU1 signal invert */
#define CCM_POLCNTRL_INVERT_MASK    (0xff)

/* Power Domain Status Register 0 */

#define CCM_PDSTAT0_HTU2_MASK       (3 << 2)  /* Bits 2-3: HTU2 transaction */
#define CCM_PDSTAT0_DMM_MASK        (3 << 4)  /* Bits 4-5: DMM transaction */

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_CCM_H */
