/****************************************************************************
 * arch/tricore/src/tc3xx/hardware/tc3xx_port.h
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

#ifndef __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_PORT_H
#define __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_PORT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Port registers: one IOCR byte per pin, one PDR nibble per pin.  PDR is
 * protected by the CPU ENDINIT.
 */

#define TC3XX_PORT_IOCR0_OFFSET    0x10
#define TC3XX_PORT_PDR0_OFFSET     0x40

#define TC3XX_PORT_IOCR(p, n)      ((p) + TC3XX_PORT_IOCR0_OFFSET + \
                                    ((n) / 4) * 4)
#define TC3XX_PORT_IOCR_SHIFT(n)   (((n) & 3) * 8)
#define TC3XX_PORT_IOCR_MASK       0xff
#define TC3XX_PORT_PDR(p, n)       ((p) + TC3XX_PORT_PDR0_OFFSET + \
                                    ((n) / 8) * 4)
#define TC3XX_PORT_PDR_SHIFT(n)    (((n) & 7) * 4)
#define TC3XX_PORT_PDR_MASK        0xf

#define TC3XX_PORT_IN_PULLUP       0x10        /* Input with pull-up */
#define TC3XX_PORT_OUT_ALT2        0x90        /* Push-pull, alternate 2 */
#define TC3XX_PORT_PAD_SPEED1      0           /* CMOS automotive speed 1 */

#endif /* __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_PORT_H */
