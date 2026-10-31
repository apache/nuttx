/****************************************************************************
 * arch/rh850/src/u2c/hardware/u2c_intc.h
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

#ifndef __ARCH_RH850_SRC_U2C_HARDWARE_U2C_INTC_H
#define __ARCH_RH850_SRC_U2C_HARDWARE_U2C_INTC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EIC registers: channels 0-31 in INTC1 of CPU0, the others in INTC2.  The
 * EIC of channel n is at base + 2 * n.
 */

#define U2C_INTC1_BASE          0xfffc4000
#define U2C_INTC2_BASE          0xfff80000
#define U2C_INTC1_CHANNELS      32

/* EIC low byte */

#define U2C_EIC_EIMK            (1 << 7)    /* Interrupt masked */
#define U2C_EIC_EITB            (1 << 6)    /* Table reference method */
#define U2C_EIC_EIP_MASK        0x0f        /* Priority, 0 is the highest */

/* EIC high byte */

#define U2C_EIC_HI_EIRF         (1 << 4)    /* Interrupt request flag */

#endif /* __ARCH_RH850_SRC_U2C_HARDWARE_U2C_INTC_H */
