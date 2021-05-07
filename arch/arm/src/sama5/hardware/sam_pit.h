/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_pit.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_PIT_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_PIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PIT Register Offsets *****************************************************/

#define SAM_PIT_MR_OFFSET    0x0000 /* Mode Register */
#define SAM_PIT_SR_OFFSET    0x0004 /* Status Register */
#define SAM_PIT_PIVR_OFFSET  0x0008 /* Periodic Interval Value Register */
#define SAM_PIT_PIIR_OFFSET  0x000c /* Periodic Interval Image Register */

/* PIT Register Addresses ***************************************************/

#define SAM_PIT_MR          (SAM_PITC_VBASE+SAM_PIT_MR_OFFSET)
#define SAM_PIT_SR          (SAM_PITC_VBASE+SAM_PIT_SR_OFFSET)
#define SAM_PIT_PIVR        (SAM_PITC_VBASE+SAM_PIT_PIVR_OFFSET)
#define SAM_PIT_PIIR        (SAM_PITC_VBASE+SAM_PIT_PIIR_OFFSET)

/* PIT Register Bit Definitions *********************************************/

/* Mode Register */

#define PIT_MR_PIV_SHIFT    (0) /* Bits 0-19: Periodic Interval Value */
#define PIT_MR_PIV_MASK     (0x000fffff)
#  define PIT_MR_PIV(n)     (n)
#define PIT_MR_PITEN        (1 << 24) /* Bit 24: Period Interval Timer Enable */
#define PIT_MR_PITIEN       (1 << 25) /* Bit 25: Periodic Interval Timer Interrupt Enable */

/* Status Register */

#define PIT_SR_S            (1 << 0)  /* Bit 0: Periodic Interval Timer Status */

/* Periodic Interval Value Register */

/* Periodic Interval Image Register */

#define PIT_CPIV_SHIFT      (0) /* Bits 0-19: Current Periodic Interval Value */
#define PIT_CPIV_MASK       (0x000fffff)
#define PIT_PICNT_SHIFT     (20) /* Bits 20-31: Periodic Interval Counter */
#define PIT_PICNT_MASK      (0xfff << PIT_PIVR_PICNT_SHIFT)

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_PIT_H */
