/****************************************************************************
 * arch/tricore/src/tc3x/chip.h
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

#ifndef __ARCH_TRICORE_SRC_TC3X_CHIP_H
#define __ARCH_TRICORE_SRC_TC3X_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/tc3x_cpu.h"

/* UART0 pins: ASCLIN0 RXA on P14.1, TX on P14.0 (alternate output 2) */

#define UART_PIN_PORT              0xf003ae00  /* P14 */
#define UART_PIN_RX                1
#define UART_PIN_TX                0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Startup core synchronization: core ID bits to wait for and the timeout
 * in STM ticks (0: no timeout)
 */

#ifdef CONFIG_ARCH_CHIP_TC375
#  define TRICORE_CORE_SYNC_MASK   0x07
#else
#  define TRICORE_CORE_SYNC_MASK   0x5f
#endif
#define TRICORE_CORE_SYNC_TIMEOUT  100000

/* Other cores that tricore_start releases, CPU1 only as with the iLLD
 * startup code, and their PC and boot halt registers
 */

#define TRICORE_START_CORES        1
#define TRICORE_START_PC(n)        TC3X_CPU_PC(n)
#define TRICORE_START_HALT(n)      TC3X_CPU_SYSCON(n)
#define TRICORE_START_BHALT        TC3X_SYSCON_BHALT

/* Core index from CORE_ID; CPU5 has the core ID 6 on TC39x */

#define tricore_core_id()          tricore_mfcr(TRICORE_CPU_CORE_ID)
#define tricore_core_index()       (tricore_core_id() > 5 ? 5 : \
                                    tricore_core_id())

#endif /* __ARCH_TRICORE_SRC_TC3X_CHIP_H */
