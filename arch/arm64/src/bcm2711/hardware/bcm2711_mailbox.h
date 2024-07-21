/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_mailbox.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __ARCH_ARM64_SRC_BCM2711_MAILBOX_H
#define __ARCH_ARM64_SRC_BCM2711_MAILBOX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mailbox register offsets */

#define BCM_MBOX_SET00_OFFSET 0x80
#define BCM_MBOX_SET01_OFFSET 0x84
#define BCM_MBOX_SET02_OFFSET 0x88
#define BCM_MBOX_SET03_OFFSET 0x8c
#define BCM_MBOX_SET04_OFFSET 0x90
#define BCM_MBOX_SET05_OFFSET 0x94
#define BCM_MBOX_SET06_OFFSET 0x98
#define BCM_MBOX_SET07_OFFSET 0x9c
#define BCM_MBOX_SET08_OFFSET 0xa0
#define BCM_MBOX_SET09_OFFSET 0xa4
#define BCM_MBOX_SET10_OFFSET 0xa8
#define BCM_MBOX_SET11_OFFSET 0xac
#define BCM_MBOX_SET12_OFFSET 0xb0
#define BCM_MBOX_SET13_OFFSET 0xb4
#define BCM_MBOX_SET14_OFFSET 0xb8
#define BCM_MBOX_SET15_OFFSET 0xbc
#define BCM_MBOX_CLR00_OFFSET 0xc0
#define BCM_MBOX_CLR01_OFFSET 0xc4
#define BCM_MBOX_CLR02_OFFSET 0xc8
#define BCM_MBOX_CLR03_OFFSET 0xcc
#define BCM_MBOX_CLR04_OFFSET 0xd0
#define BCM_MBOX_CLR05_OFFSET 0xd4
#define BCM_MBOX_CLR06_OFFSET 0xd8
#define BCM_MBOX_CLR07_OFFSET 0xdc
#define BCM_MBOX_CLR08_OFFSET 0xe0
#define BCM_MBOX_CLR09_OFFSET 0xe4
#define BCM_MBOX_CLR10_OFFSET 0xe8
#define BCM_MBOX_CLR11_OFFSET 0xec
#define BCM_MBOX_CLR12_OFFSET 0xf0
#define BCM_MBOX_CLR13_OFFSET 0xf4
#define BCM_MBOX_CLR14_OFFSET 0xf8
#define BCM_MBOX_CLR15_OFFSET 0xfc

/* Mailbox register addresses */

#define _BCM_MBOX(offset) (BCM_ARMLOCAL_BASEADDR + (offset))

#define BCM_MBOX_SET00 _BCM_MBOX(BCM_MBOX_SET00_OFFSET)
#define BCM_MBOX_SET01 _BCM_MBOX(BCM_MBOX_SET01_OFFSET)
#define BCM_MBOX_SET02 _BCM_MBOX(BCM_MBOX_SET02_OFFSET)
#define BCM_MBOX_SET03 _BCM_MBOX(BCM_MBOX_SET03_OFFSET)
#define BCM_MBOX_SET04 _BCM_MBOX(BCM_MBOX_SET04_OFFSET)
#define BCM_MBOX_SET05 _BCM_MBOX(BCM_MBOX_SET05_OFFSET)
#define BCM_MBOX_SET06 _BCM_MBOX(BCM_MBOX_SET06_OFFSET)
#define BCM_MBOX_SET07 _BCM_MBOX(BCM_MBOX_SET07_OFFSET)
#define BCM_MBOX_SET08 _BCM_MBOX(BCM_MBOX_SET08_OFFSET)
#define BCM_MBOX_SET09 _BCM_MBOX(BCM_MBOX_SET09_OFFSET)
#define BCM_MBOX_SET10 _BCM_MBOX(BCM_MBOX_SET10_OFFSET)
#define BCM_MBOX_SET11 _BCM_MBOX(BCM_MBOX_SET11_OFFSET)
#define BCM_MBOX_SET12 _BCM_MBOX(BCM_MBOX_SET12_OFFSET)
#define BCM_MBOX_SET13 _BCM_MBOX(BCM_MBOX_SET13_OFFSET)
#define BCM_MBOX_SET14 _BCM_MBOX(BCM_MBOX_SET14_OFFSET)
#define BCM_MBOX_SET15 _BCM_MBOX(BCM_MBOX_SET15_OFFSET)
#define BCM_MBOX_CLR00 _BCM_MBOX(BCM_MBOX_CLR00_OFFSET)
#define BCM_MBOX_CLR01 _BCM_MBOX(BCM_MBOX_CLR01_OFFSET)
#define BCM_MBOX_CLR02 _BCM_MBOX(BCM_MBOX_CLR02_OFFSET)
#define BCM_MBOX_CLR03 _BCM_MBOX(BCM_MBOX_CLR03_OFFSET)
#define BCM_MBOX_CLR04 _BCM_MBOX(BCM_MBOX_CLR04_OFFSET)
#define BCM_MBOX_CLR05 _BCM_MBOX(BCM_MBOX_CLR05_OFFSET)
#define BCM_MBOX_CLR06 _BCM_MBOX(BCM_MBOX_CLR06_OFFSET)
#define BCM_MBOX_CLR07 _BCM_MBOX(BCM_MBOX_CLR07_OFFSET)
#define BCM_MBOX_CLR08 _BCM_MBOX(BCM_MBOX_CLR08_OFFSET)
#define BCM_MBOX_CLR09 _BCM_MBOX(BCM_MBOX_CLR09_OFFSET)
#define BCM_MBOX_CLR10 _BCM_MBOX(BCM_MBOX_CLR10_OFFSET)
#define BCM_MBOX_CLR11 _BCM_MBOX(BCM_MBOX_CLR11_OFFSET)
#define BCM_MBOX_CLR12 _BCM_MBOX(BCM_MBOX_CLR12_OFFSET)
#define BCM_MBOX_CLR13 _BCM_MBOX(BCM_MBOX_CLR13_OFFSET)
#define BCM_MBOX_CLR14 _BCM_MBOX(BCM_MBOX_CLR14_OFFSET)
#define BCM_MBOX_CLR15 _BCM_MBOX(BCM_MBOX_CLR15_OFFSET)

#endif /* __ARCH_ARM64_SRC_BCM2711_MAILBOX_H */
