/****************************************************************************
 * arch/arm/src/s32k1xx/chip.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_CHIP_H
#define __ARCH_ARM_SRC_S32K1XX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Include the memory map and the chip definitions file.
 * Other chip hardware files should then include this file for the proper
 * setup.
 */

#include <arch/irq.h>
#include "hardware/s32k1xx_memorymap.h"

/* The common ARMv6/7-M vector handling logic expects the following
 * definitions in this file.
 * ARMV6/7M_PERIPHERAL_INTERRUPTS provides the number of supported
 * external interrupts which, for this architecture, is provided in the
 * arch/s32k1xx/irq.h header file.
 */

#define ARMV6M_PERIPHERAL_INTERRUPTS S32K1XX_IRQ_NEXTINT
#define ARMV7M_PERIPHERAL_INTERRUPTS S32K1XX_IRQ_NEXTINT

/* Cache line sizes (in bytes)for the S32K14X */

#define ARMV7M_DCACHE_LINESIZE 16  /* 16 bytes (4 words) */
#define ARMV7M_ICACHE_LINESIZE 16  /* 16 bytes (4 words) */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_S32K1XX_CHIP_H */
