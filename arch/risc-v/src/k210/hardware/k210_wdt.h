/****************************************************************************
 * arch/risc-v/src/k210/hardware/k210_wdt.h
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

#ifndef __ARCH_RISCV_SRC_K210_HARDWARE_K210_WDT_H
#define __ARCH_RISCV_SRC_K210_HARDWARE_K210_WDT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define K210_WDT_CR_OFFSET          0x0000
#define K210_WDT_TORR_OFFSET        0x0004
#define K210_WDT_CCVR_OFFSET        0x0008
#define K210_WDT_CRR_OFFSET         0x000c
#define K210_WDT_STAT_OFFSET        0x0010
#define K210_WDT_EOI_OFFSET         0x0014
#define K210_WDT_PROT_LEVEL_OFFSET  0x001c

#define K210_WDT_CR(base)           ((base) + K210_WDT_CR_OFFSET)
#define K210_WDT_TORR(base)         ((base) + K210_WDT_TORR_OFFSET)
#define K210_WDT_CCVR(base)         ((base) + K210_WDT_CCVR_OFFSET)
#define K210_WDT_CRR(base)          ((base) + K210_WDT_CRR_OFFSET)
#define K210_WDT_STAT(base)         ((base) + K210_WDT_STAT_OFFSET)
#define K210_WDT_EOI(base)          ((base) + K210_WDT_EOI_OFFSET)

#define K210_WDT_CR_ENABLE          0x00000001u
#define K210_WDT_CR_RMOD_MASK       0x00000002u
#define K210_WDT_CR_RMOD_RESET      0x00000000u
#define K210_WDT_CR_RMOD_INTERRUPT  0x00000002u

#define K210_WDT_TORR_TOP(n)        (((n) << 4) | ((n) << 0))

#define K210_WDT_CRR_RESTART        0x00000076u

#endif /* __ARCH_RISCV_SRC_K210_HARDWARE_K210_WDT_H */
