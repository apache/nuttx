/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_bsc.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_BSC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_BSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BSC Register Offsets *****************************************************/

#define SAM_BSC_CR_OFFSET    0x0000 /* Boot Sequence Configuration Register */

/* BSC Register Addresses ***************************************************/

#define SAM_BSC_CR           (SAM_BSC_VBASE+SAM_BSC_CR_OFFSET)

/* BSC Register Bit Definitions *********************************************/

/* Boot Sequence Configuration Register */

#define BSC_CR_BOOT_SHIFT    (0)    /* Bits 0-7: Boot Media Sequence */
#define BSC_CR_BOOT_MASK     (0xff << BSC_CR_BOOT_SHIFT)
#define BSC_CR_BOOTKEY_SHIFT (16)   /* Bits 16-31: Book key */
#define BSC_CR_BOOTKEY_MASK  (0xffff << BSC_CR_BOOTKEY_SHIFT)
#  define BSC_CR_BOOTKEY     (0x6683 << BSC_CR_BOOTKEY_SHIFT)

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_BSC_H */
