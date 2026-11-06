/****************************************************************************
 * arch/tricore/src/tc3xx/chip.h
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

#ifndef __ARCH_TRICORE_SRC_TC3XX_CHIP_H
#define __ARCH_TRICORE_SRC_TC3XX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

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

#define TRICORE_CORE_SYNC_MASK     0x5f
#define TRICORE_CORE_SYNC_TIMEOUT  100000

/* Core index from CORE_ID; CPU5 has the core ID 6 on TC39x */

#define tricore_core_id()          tricore_mfcr(TRICORE_CPU_CORE_ID)
#define tricore_core_index()       (tricore_core_id() > 5 ? 5 : \
                                    tricore_core_id())

#endif /* __ARCH_TRICORE_SRC_TC3XX_CHIP_H */
