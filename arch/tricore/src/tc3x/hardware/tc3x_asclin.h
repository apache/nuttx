/****************************************************************************
 * arch/tricore/src/tc3x/hardware/tc3x_asclin.h
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

#ifndef __ARCH_TRICORE_SRC_TC3X_HARDWARE_TC3X_ASCLIN_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define __ARCH_TRICORE_SRC_TC3X_HARDWARE_TC3X_ASCLIN_H

#define TRICORE_ASCLIN0_BASE        0xf0000600
#define TRICORE_ASCLIN_STRIDE       0x100
#define TRICORE_ASCLIN_BASE(n)      ((n) <= 9 ? (TRICORE_ASCLIN0_BASE + ((uint32_t)(n) * TRICORE_ASCLIN_STRIDE)) \
                                     : (0xf02c0a00 + (((uint32_t)(n) - 10) * TRICORE_ASCLIN_STRIDE)))

#define ASCLIN_CLC_OFFSET           0x00
#define ASCLIN_IOCR_OFFSET          0x04
#define ASCLIN_ID_OFFSET            0x08
#define ASCLIN_TXFIFOCON_OFFSET     0x0c
#define ASCLIN_RXFIFOCON_OFFSET     0x10
#define ASCLIN_BITCON_OFFSET        0x14
#define ASCLIN_FRAMECON_OFFSET      0x18
#define ASCLIN_DATCON_OFFSET        0x1c
#define ASCLIN_BRG_OFFSET           0x20
#define ASCLIN_BRD_OFFSET           0x24
#define ASCLIN_LINCON_OFFSET        0x28
#define ASCLIN_LINBTIMER_OFFSET     0x2c
#define ASCLIN_LINHTIMER_OFFSET     0x30
#define ASCLIN_FLAGS_OFFSET         0x34
#define ASCLIN_FLAGSSET_OFFSET      0x38
#define ASCLIN_FLAGSCLEAR_OFFSET    0x3c
#define ASCLIN_FLAGSENABLE_OFFSET   0x40
#define ASCLIN_TXDATA_OFFSET        0x44
#define ASCLIN_RXDATA_OFFSET        0x48
#define ASCLIN_CSR_OFFSET           0x4c
#define ASCLIN_RXDATAD_OFFSET       0x50

#define ASCLIN_OCS_OFFSET           0xe8
#define ASCLIN_KRSTCLR_OFFSET       0xec
#define ASCLIN_KRST1_OFFSET         0xf0
#define ASCLIN_KRST0_OFFSET         0xf4
#define ASCLIN_ACCEN1_OFFSET        0xf8
#define ASCLIN_ACCEN0_OFFSET        0xfc

#endif /* __ARCH_TRICORE_SRC_TC3X_HARDWARE_TC3X_ASCLIN_H */
