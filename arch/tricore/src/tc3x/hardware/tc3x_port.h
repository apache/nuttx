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

/* Registers of port n */

#define TC3X_PORT_BASE(n)          (0xf003a000 + (n) * 0x100)
#define TC3X_PORT_OMR_OFFSET       0x04        /* Output modification */
#define TC3X_PORT_IN_OFFSET        0x24        /* Input */

#define TC3X_PORT_OMR_SET(n)       (1 << (n))
#define TC3X_PORT_OMR_CLR(n)       (1 << ((n) + 16))

/* Port registers: one IOCR byte per pin, one PDR nibble per pin.  PDR is
 * protected by the CPU ENDINIT.
 */

#define TC3X_PORT_IOCR0_OFFSET     0x10
#define TC3X_PORT_PDR0_OFFSET      0x40

#define TC3X_PORT_IOCR(p, n)       ((p) + TC3X_PORT_IOCR0_OFFSET + \
                                    ((n) / 4) * 4)
#define TC3X_PORT_IOCR_SHIFT(n)    (((n) & 3) * 8)
#define TC3X_PORT_IOCR_MASK        0xff
#define TC3X_PORT_PDR(p, n)        ((p) + TC3X_PORT_PDR0_OFFSET + \
                                    ((n) / 8) * 4)
#define TC3X_PORT_PDR_SHIFT(n)     (((n) & 7) * 4)
#define TC3X_PORT_PDR_MASK         0xf

/* Port control code of a pin, bits 7:3 of its IOCR byte */

#define TC3X_PORT_PC_SHIFT         3
#define TC3X_PORT_PC_PULL_MASK     0x03        /* Input: 1 pull-down, 2 up */
#define TC3X_PORT_PC_OUTPUT        0x10        /* Output, alternate 0 to 7 */
#define TC3X_PORT_PC_OPENDRAIN     0x08
#define TC3X_PORT_PC_ALT_MAX       7

/* Pad nibble of a pin: pad level in bits 3:2, pad driver in bits 1:0 */

#define TC3X_PORT_PDR_PL_SHIFT     2

#endif /* __ARCH_TRICORE_SRC_TC3X_HARDWARE_TC3X_PORT_H */
