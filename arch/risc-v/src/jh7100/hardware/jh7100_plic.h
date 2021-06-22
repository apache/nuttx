/****************************************************************************
 * arch/risc-v/src/jh7100/hardware/jh7100_plic.h
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

#ifndef __ARCH_RISCV_SRC_jh7100_HARDWARE_JH7100_PLIC_H
#define __ARCH_RISCV_SRC_jh7100_HARDWARE_JH7100_PLIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define JH7100_PLIC_PRIORITY    (JH7100_PLIC_BASE + 0x000000) // Zero is invalid. Starts at 1.
#define JH7100_PLIC_IP0         (JH7100_PLIC_BASE + 0x001000) // Interrupt Pending.
#define JH7100_PLIC_IP1         (JH7100_PLIC_BASE + 0x001004)
#define JH7100_PLIC_MIE0        (JH7100_PLIC_BASE + 0x002000)
#define JH7100_PLIC_MIE1        (JH7100_PLIC_BASE + 0x002004)
#define JH7100_PLIC_SIE0        (JH7100_PLIC_BASE + 0x002080)
#define JH7100_PLIC_SIE1        (JH7100_PLIC_BASE + 0x002084)

/* This exists in D1, but not U74 */
#define JH7100_PLIC_MTHRESHOLD  (JH7100_PLIC_BASE + 0x200000)
#define JH7100_PLIC_MCLAIM      (JH7100_PLIC_BASE + 0x200004)
#define JH7100_PLIC_STHRESHOLD  (JH7100_PLIC_BASE + 0x201000)
#define JH7100_PLIC_SCLAIM      (JH7100_PLIC_BASE + 0x201004)

#endif /* __ARCH_RISCV_SRC_jh7100_HARDWARE_JH7100_PLIC_H */
