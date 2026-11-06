/****************************************************************************
 * arch/tricore/src/tc4xx/chip.h
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_CHIP_H
#define __ARCH_TRICORE_SRC_TC4XX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#define UART_PIN_RX IfxAsclin0_RXA_F_P14_1_IN     /* UART receive port pin  */
#define UART_PIN_TX IfxAsclin0_TX_F_P14_0_OUT     /* UART transmit port pin */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Startup core synchronization: core ID bits to wait for and the timeout
 * in STM ticks (0: no timeout)
 */

#define TRICORE_CORE_SYNC_MASK     0x01
#define TRICORE_CORE_SYNC_TIMEOUT  0

/* Core index from CORE_ID */

#define tricore_core_id()          (tricore_mfcr(TRICORE_CPU_CORE_ID) & 0x7)
#define tricore_core_index()       tricore_core_id()

#endif /* __ARCH_TRICORE_SRC_TC4XX_CHIP_H */
