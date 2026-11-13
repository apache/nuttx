/****************************************************************************
 * arch/tricore/src/tc3xx/hardware/tc3xx_cpu.h
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

#ifndef __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_CPU_H
#define __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_CPU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CPU0 kernel reset status */

#define TC3XX_CPU0_KRST0           0xf880d000
#define TC3XX_KRST0_RSTSTAT_MASK   (0x3 << 1)

/* CPU1 start: PC and SYSCON.BHALT in the CPU1 CSFR space */

#define TC3XX_CPU1_PC              0xf883fe08
#define TC3XX_CPU1_SYSCON          0xf883fe14
#define TC3XX_SYSCON_BHALT         (1 << 24)

#endif /* __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_CPU_H */
