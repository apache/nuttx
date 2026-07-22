/****************************************************************************
 * arch/mips/src/jz4780/jz4780_ehci.h
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

#ifndef __ARCH_MIPS_SRC_JZ4780_JZ_EHCI_H
#define __ARCH_MIPS_SRC_JZ4780_JZ_EHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/ohci.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define JZ_UHPEHCI_VSECTION   0xb3490000

/* The JZ4780 supports 3 root hub ports */

#define JZ_EHCI_NRHPORT 3

/* Registers ****************************************************************/

/* Traditionally, NuttX specifies register locations using individual
 * register offsets from a base address.  That tradition is broken here and,
 * instead, register blocks are represented as structures.  This is done here
 * because, in principle, EHCI operational register address may not be known
 * at compile time; the operational registers lie at an offset specified in
 * the 'caplength' byte of the Host Controller Capability Registers.
 *
 * However, for the case of the JZ4780 EHCI, we know apriori that the value
 * of 'caplength' is 0x10.  We keep this structure, however, to facilitate
 * porting this driver to other environments where, perhaps, such knowledge
 * is not available.
 */

/* Host Controller Capability Registers */

#define HCCR ((struct ehci_hccr_s *)JZ_UHPEHCI_VSECTION)

/* Host Controller Operational Registers */

#define HCOR ((volatile struct ehci_hcor_s *)(JZ_UHPEHCI_VSECTION + 0x10))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_MIPS_SRC_JZ4780_JZ_EHCI_H */
