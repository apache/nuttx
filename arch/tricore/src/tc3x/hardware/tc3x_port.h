/****************************************************************************
 * arch/tricore/src/tc3x/hardware/tc3x_port.h
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

#ifndef __ARCH_TRICORE_SRC_TC3X_HARDWARE_TC3X_PORT_H
#define __ARCH_TRICORE_SRC_TC3X_HARDWARE_TC3X_PORT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AURIX_PORT_BASE         0xf003a000
#define AURIX_PORT_STRIDE       0x100
#define AURIX_PORT_ADDR(n)      (AURIX_PORT_BASE + \
                                 ((uint32_t)(n) * AURIX_PORT_STRIDE))

#define PORT_OUT_OFFSET         0x00
#define PORT_OMR_OFFSET         0x04
#define PORT_ID_OFFSET          0x08
#define PORT_IOCR_OFFSET        0x10
#define PORT_IOCR4_OFFSET       0x14
#define PORT_IOCR8_OFFSET       0x18
#define PORT_IOCR12_OFFSET      0x1c
#define PORT_IN_OFFSET          0x24
#define PORT_PDR0_OFFSET        0x40
#define PORT_PDR1_OFFSET        0x44
#define PORT_ESR_OFFSET         0x50
#define PORT_PDISC_OFFSET       0x60
#define PORT_PCSR_OFFSET        0x64
#define PORT_LPCR_OFFSET        0xa0
#define PORT_ACCEN1_OFFSET      0xf8
#define PORT_ACCEN0_OFFSET      0xfc

#define PORT_IOCR_REG(pin)      (PORT_IOCR_OFFSET + ((pin) / 4) * 4)
#define PORT_IOCR_PC_SHIFT(pin) (((pin) % 4) * 8 + 3)
#define PORT_IOCR_PC_MASK(pin)  (0x1f << PORT_IOCR_PC_SHIFT(pin))

#define PORT_PC_INPUT           0x00
#define PORT_PC_INPUT_PD        0x01
#define PORT_PC_INPUT_PU        0x02
#define PORT_PC_OUTPUT          0x10
#define PORT_PC_OUTPUT_OD       0x18
#define PORT_PC_OUTPUT_ALT(n)   (0x10 | ((n) & 0x07))

#define PORT_PDR_REG(pin)       (PORT_PDR0_OFFSET + ((pin) / 8) * 4)
#define PORT_PDR_SHIFT(pin)     (((pin) % 8) * 4)
#define PORT_PDR_MASK(pin)      (0x0f << PORT_PDR_SHIFT(pin))

#define PORT_PDR_VAL(pd, pl)    (((pl) << 2) | (pd))

#define PORT_OMR_SET(pin)       BIT(pin)
#define PORT_OMR_CLR(pin)       BIT((pin) + 16)

#endif /* __ARCH_TRICORE_SRC_TC3X_HARDWARE_TC3X_PORT_H */
