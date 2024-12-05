/****************************************************************************
 * arch/arm/src/mx8mp/hardware/mx8mp_mu.h
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

#ifndef __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_MU_H
#define __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_MU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Messaging Unit Register Offsets ******************************************/

#define MU_TR_OFFSET       0x0000
#define MU_RR_OFFSET       0x0010
#define MU_SR_OFFSET       0x0020
#define MU_CR_OFFSET       0x0024

/* Messaging Unit Register Bit Definitions **********************************/

#define MU_SR_RF0_SHIFT    27
#define MU_SR_RF1_SHIFT    26
#define MU_SR_RF2_SHIFT    25
#define MU_SR_RF3_SHIFT    24
#define MU_SR_TE0_SHIFT    23
#define MU_SR_TE1_SHIFT    22
#define MU_SR_TE2_SHIFT    21
#define MU_SR_TE3_SHIFT    20

#define MU_CR_GIE_MASK     0xf0000000U
#define MU_CR_RIE_MASK     0x0f000000U
#define MU_CR_TIE_MASK     0x00f00000U
#define MU_CR_GIR_MASK     0x000f0000U
#define MU_CR_RIE0_SHIFT   27
#define MU_CR_RIE1_SHIFT   26
#define MU_CR_RIE2_SHIFT   25
#define MU_CR_RIE3_SHIFT   24

#endif /* __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_I2C_H */
