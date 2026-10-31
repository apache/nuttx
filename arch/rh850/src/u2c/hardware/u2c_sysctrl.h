/****************************************************************************
 * arch/rh850/src/u2c/hardware/u2c_sysctrl.h
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

#ifndef __ARCH_RH850_SRC_U2C_HARDWARE_U2C_SYSCTRL_H
#define __ARCH_RH850_SRC_U2C_HARDWARE_U2C_SYSCTRL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock controller */

#define U2C_CKS_RLINC           0xff980140  /* CLK_RLIN3 source selection */
#define U2C_CKS_RLINS           0xff980148  /* CLK_RLIN3 source status */
#define U2C_CLKKCPROT1          0xff980700  /* Clock register write protection */

#define U2C_CKS_RLIN_HSB        1           /* CLK_RLIN3 from CLKC_HSB */
#define U2C_CKS_RLIN_MASK       0x7

/* Reset controller: module reset status, software system reset */

#define U2C_SWMRESS_RLIN3       0xff980964
#define U2C_SWMRESS_OSTM        0xff980a84
#define U2C_RESKCPROT0          0xff980f00  /* Reset register write protection */
#define U2C_SWSRESA             0xff98840c  /* Software system reset */

#define U2C_SWSRESA_TRIGGER     (1 << 0)    /* Generate a System Reset 2 */

/* Standby controller: module stop */

#define U2C_MSR_RLIN3           0xff981060
#define U2C_MSR_OSTM            0xff981180
#define U2C_MSRKCPROT           0xff981710  /* MSR write protection */

#define U2C_MSR_RLIN311         (1 << 11)
#define U2C_MSR_OSTM0           (1 << 0)

/* Write protection keys */

#define U2C_KCPROT_ENABLE       0xa5a5a501
#define U2C_KCPROT_DISABLE      0xa5a5a500

#endif /* __ARCH_RH850_SRC_U2C_HARDWARE_U2C_SYSCTRL_H */
