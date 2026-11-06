/****************************************************************************
 * arch/tricore/src/tc3xx/hardware/tc3xx_asclin.h
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

#ifndef __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_ASCLIN_H
#define __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_ASCLIN_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ASCLIN0 (UART) and its register offsets */

#define TRICORE_ASCLIN0_BASE       0xf0000600

#define ASCLIN_CLC_OFFSET          0x000
#define ASCLIN_IOCR_OFFSET         0x004
#define ASCLIN_TXFIFOCON_OFFSET    0x00c
#define ASCLIN_RXFIFOCON_OFFSET    0x010
#define ASCLIN_BITCON_OFFSET       0x014
#define ASCLIN_FRAMECON_OFFSET     0x018
#define ASCLIN_DATCON_OFFSET       0x01c
#define ASCLIN_BRG_OFFSET          0x020
#define ASCLIN_FLAGSCLEAR_OFFSET   0x03c
#define ASCLIN_FLAGSENABLE_OFFSET  0x040
#define ASCLIN_TXDATA_OFFSET       0x044
#define ASCLIN_RXDATA_OFFSET       0x048
#define ASCLIN_CSR_OFFSET          0x04c

#endif /* __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_ASCLIN_H */
