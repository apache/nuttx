/****************************************************************************
 * arch/tricore/src/tc4xx/hardware/tc4xx_cpu.h
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_CPU_H
#define __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_CPU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Start of the other cores: HRA PC and BOOTCON.BHALT of each CPU */

#define TC4XX_CPU_HRA_PC(n)        (0xf881fe08 + (n) * 0x40000)
#define TC4XX_CPU_HRA_BOOTCON(n)   (0xf881fe60 + (n) * 0x40000)
#define TC4XX_BOOTCON_BHALT        (1 << 0)

#endif /* __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_CPU_H */
