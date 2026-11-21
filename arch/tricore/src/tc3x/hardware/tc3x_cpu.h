/****************************************************************************
 * arch/tricore/src/tc3x/hardware/tc3x_cpu.h
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

#ifndef __ARCH_TRICORE_SRC_TC3X_HARDWARE_TC3X_CPU_H
#define __ARCH_TRICORE_SRC_TC3X_HARDWARE_TC3X_CPU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Start of the other cores: PC and SYSCON.BHALT of each CPU.  The CSFRs
 * of the CPUs are 0x20000 apart, but those of CPU5 are at 0xf88c0000.
 */

#define TC3X_CPU_CSFR(n)           ((n) < 5 ? 0xf8800000 + (n) * 0x20000 : \
                                    0xf88c0000)
#define TC3X_CPU_PC(n)             (TC3X_CPU_CSFR(n) + 0x1fe08)
#define TC3X_CPU_SYSCON(n)         (TC3X_CPU_CSFR(n) + 0x1fe14)
#define TC3X_SYSCON_BHALT          (1 << 24)

#endif /* __ARCH_TRICORE_SRC_TC3X_HARDWARE_TC3X_CPU_H */
