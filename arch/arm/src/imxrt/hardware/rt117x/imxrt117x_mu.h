/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_mu.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_MU_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_MU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The i.MX RT117x Messaging Unit is the "v1" MU: 4 transmit and 4 receive
 * mailbox registers, one status and one control register per side.
 * MU-A is the CM7 side, MU-B is the CM4 side.
 */

#define IMXRT_MU_CHANNELS            4

/* Register Offsets *********************************************************/

#define IMXRT_MU_TR_OFFSET(n)        (0x0000 + ((n) << 2)) /* Transmit n */
#define IMXRT_MU_RR_OFFSET(n)        (0x0010 + ((n) << 2)) /* Receive n */
#define IMXRT_MU_SR_OFFSET           0x0020                /* Status */
#define IMXRT_MU_CR_OFFSET           0x0024                /* Control */

/* Register Addresses (CM7 side, MU-A) **************************************/

#define IMXRT_MUA_TR(n)              (IMXRT_MU_A + IMXRT_MU_TR_OFFSET(n))
#define IMXRT_MUA_RR(n)              (IMXRT_MU_A + IMXRT_MU_RR_OFFSET(n))
#define IMXRT_MUA_SR                 (IMXRT_MU_A + IMXRT_MU_SR_OFFSET)
#define IMXRT_MUA_CR                 (IMXRT_MU_A + IMXRT_MU_CR_OFFSET)

/* Status Register (SR) *****************************************************/

#define MU_SR_F_SHIFT                (0)       /* Bits 0-2: Other side flags */
#define MU_SR_F_MASK                 (7 << MU_SR_F_SHIFT)
#define MU_SR_EP                     (1 << 4)  /* Bit 4:  Event pending */
#define MU_SR_RS                     (1 << 7)  /* Bit 7:  Other side reset */
#define MU_SR_TE_SHIFT               (20)      /* Bits 20-23: TEn, TE0 is bit 23 */
#define MU_SR_TE(n)                  (1 << (23 - (n)))
#define MU_SR_TE_MASK                (15 << MU_SR_TE_SHIFT)
#define MU_SR_RF_SHIFT               (24)      /* Bits 24-27: RFn, RF0 is bit 27 */
#define MU_SR_RF(n)                  (1 << (27 - (n)))
#define MU_SR_RF_MASK                (15 << MU_SR_RF_SHIFT)
#define MU_SR_GIP_SHIFT              (28)      /* Bits 28-31: GIP0 is bit 31 */
#define MU_SR_GIP(n)                 (1 << (31 - (n)))
#define MU_SR_GIP_MASK               (15 << MU_SR_GIP_SHIFT)

/* Control Register (CR) ****************************************************/

#define MU_CR_F_SHIFT                (0)       /* Bits 0-2: Flags to other side */
#define MU_CR_F_MASK                 (7 << MU_CR_F_SHIFT)
#define MU_CR_MUR                    (1 << 5)  /* Bit 5:  MU reset */
#define MU_CR_GIR_SHIFT              (16)      /* Bits 16-19: GIR0 is bit 19 */
#define MU_CR_GIR(n)                 (1 << (19 - (n)))
#define MU_CR_GIR_MASK               (15 << MU_CR_GIR_SHIFT)
#define MU_CR_TIE_SHIFT              (20)      /* Bits 20-23: TIE0 is bit 23 */
#define MU_CR_TIE(n)                 (1 << (23 - (n)))
#define MU_CR_TIE_MASK               (15 << MU_CR_TIE_SHIFT)
#define MU_CR_RIE_SHIFT              (24)      /* Bits 24-27: RIE0 is bit 27 */
#define MU_CR_RIE(n)                 (1 << (27 - (n)))
#define MU_CR_RIE_MASK               (15 << MU_CR_RIE_SHIFT)
#define MU_CR_GIE_SHIFT              (28)      /* Bits 28-31: GIE0 is bit 31 */
#define MU_CR_GIE(n)                 (1 << (31 - (n)))
#define MU_CR_GIE_MASK               (15 << MU_CR_GIE_SHIFT)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_MU_H */
