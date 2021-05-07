/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_ehci.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_EHCI_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_EHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/ohci.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SAMA5 supports 3 root hub ports */

#define SAM_EHCI_NRHPORT 3

/* Registers ****************************************************************/

/* Traditionally, NuttX specifies register locations using individual
 * register offsets from a base address.  That tradition is broken here and,
 * instead, register blocks are represented as structures.  This is done here
 * because, in principle, EHCI operational register address may not be known
 * at compile time; the operational registers lie at an offset specified in
 * the 'caplength' byte of the Host Controller Capability Registers.
 *
 * However, for the case of the SAMA5 EHCI, we know apriori that the value
 * of 'caplength' is 0x10.  We keep this structure, however, to facilitate
 * porting this driver to other environments where, perhaps, such knowledge
 * is not available.
 */

/* Host Controller Capability Registers */

#define HCCR ((struct ehci_hccr_s *)SAM_UHPEHCI_VSECTION)

/* Host Controller Operational Registers */

#define HCOR ((volatile struct ehci_hcor_s *)(SAM_UHPEHCI_VSECTION + 0x10))

/* USB2 Debug Port Register Interface.  These are not documented, but I was
 * able to find these by registers by peeking at EHCI memory.
 */

#define HDEBUG ((volatile struct ehci_debug_s *)(SAM_UHPEHCI_VSECTION + 0x90))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_EHCI_H */
