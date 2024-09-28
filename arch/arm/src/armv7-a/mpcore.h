/****************************************************************************
 * arch/arm/src/armv7-a/mpcore.h
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

/* Reference:
 *   Cortex™-A9 MPCore, Revision: r4p1, Technical Reference Manual, ARM DDI
 *   0407I (ID091612).
 */

#ifndef __ARCH_ARM_SRC_ARMV7_A_MPCORE_H
#define __ARCH_ARM_SRC_ARMV7_A_MPCORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"    /* For CHIP_MPCORE_VBASE */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPCore registers are memory mapped and accessed through a processor
 * specific private address space via the SCU.  The Cortex-A9 MCU chip.h
 * header file must provide the definition CHIP_MPCORE_VBASE to access this
 * the registers in this memory region.
 */

/* Peripheral Base Offsets **************************************************/

#ifndef MPCORE_SCU_OFFSET
#define MPCORE_SCU_OFFSET  0x0000 /* 0x0000-0x00fc SCU registers */
#endif
#ifndef MPCORE_ICC_OFFSET
#ifdef CONFIG_ARCH_CORTEXA7
#define MPCORE_ICC_OFFSET  0x2000 /* 0x0000-0x00FC Interrupt controller interface */
#else
#define MPCORE_ICC_OFFSET  0x0100 /* 0x0000-0x00FC Interrupt controller interface */
#endif
#endif
#ifndef MPCORE_GTM_OFFSET
#define MPCORE_GTM_OFFSET  0x0200 /* 0x0200-0x02ff Global timer */
#endif
                                  /* 0x0300-0x05ff Reserved */
#ifndef MPCORE_PTM_OFFSET
#define MPCORE_PTM_OFFSET  0x0600 /* 0x0600-0x06ff Private timers and watchdogs */
#endif
                                  /* 0x0700-0x07ff Reserved */
#ifndef MPCORE_ICD_OFFSET
#define MPCORE_ICD_OFFSET  0x1000 /* 0x1000-0x1fff Interrupt Distributor */
#endif

/* Peripheral Base Addresses ************************************************/

#define MPCORE_SCU_VBASE   (CHIP_MPCORE_VBASE+MPCORE_SCU_OFFSET)
#define MPCORE_ICC_VBASE   (CHIP_MPCORE_VBASE+MPCORE_ICC_OFFSET)
#define MPCORE_GTM_VBASE   (CHIP_MPCORE_VBASE+MPCORE_GTM_OFFSET)
#define MPCORE_PTM_VBASE   (CHIP_MPCORE_VBASE+MPCORE_PTM_OFFSET)
#define MPCORE_ICD_VBASE   (CHIP_MPCORE_VBASE+MPCORE_ICD_OFFSET)
#define MPCORE_V2M_VBASE   (CHIP_MPCORE_VBASE+MPCORE_V2M_OFFSET)

#endif /* __ARCH_ARM_SRC_ARMV7_A_MPCORE_H */
