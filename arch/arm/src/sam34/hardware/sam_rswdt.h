/****************************************************************************
 * arch/arm/src/sam34/hardware/sam_rswdt.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_RSWDT_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_RSWDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RSWDT register offsets ***************************************************/

#define SAM_RSWDT_CR_OFFSET         0x0000 /* Control Register */
#define SAM_RSWDT_MR_OFFSET         0x0004 /* Mode Register */
#define SAM_RSWDT_SR_OFFSET         0x0008 /* Status Register */

/* RSWDT register addresses *************************************************/

#define SAM_RSWDT_CR                (SAM_RSWDT_BASE+SAM_RSWDT_CR_OFFSET)
#define SAM_RSWDT_MR                (SAM_RSWDT_BASE+SAM_RSWDT_MR_OFFSET)
#define SAM_RSWDT_SR                (SAM_RSWDT_BASE+SAM_RSWDT_SR_OFFSET)

/* RSWDT register bit definitions *******************************************/

/* Watchdog Timer Control Register */

#define RSWDT_CR_WDRSTT             (1 << 0)   /* Bit 0:  Watchdog Rest */
#define RSWDT_CR_KEY_SHIFT          (24)       /* Bits 24-31:  Password */
#define RSWDT_CR_KEY_MASK           (0xff << RSWDT_CR_KEY_SHIFT)
#  define RSWDT_CR_KEY              (0xc4 << RSWDT_CR_KEY_SHIFT)

/* Watchdog Timer Mode Register */

#define RSWDT_MR_WDV_SHIFT          (0)       /* Bits 0-11:  Watchdog Counter Value */
#define RSWDT_MR_WDV_MASK           (0xfff << RSWDT_MR_WDV_SHIFT)
#  define RSWDT_MR_WDV(n)           ((uint32_t)(n) << RSWDT_MR_WDV_SHIFT)
#define RSWDT_MR_WDFIEN             (1 << 12) /* Bit 12: Watchdog Fault Interrupt Enable */
#define RSWDT_MR_WDRSTEN            (1 << 13) /* Bit 13: Watchdog Reset Enable */
#define RSWDT_MR_WDRPROC            (1 << 14) /* Bit 14: Watchdog Reset Processor */
#define RSWDT_MR_WDDIS              (1 << 15) /* Bit 15: Watchdog Disable */
#define RSWDT_MR_WDD_SHIFT          (16)      /* Bits 16-27:  Watchdog Delta Value */
#define RSWDT_MR_WDD_MASK           (0xfff << RSWDT_MR_WDD_SHIFT)
#  define RSWDT_MR_WDD(n)           ((uint32_t)(n) << RSWDT_MR_WDD_SHIFT)
#define RSWDT_MR_WDDBGHLT           (1 << 28) /* Bit 28: Watchdog Debug Halt */
#define RSWDT_MR_WDIDLEHLT          (1 << 29) /* Bit 29: Watchdog Idle Halt */

/* Watchdog Timer Status Register */

#define RSWDT_SR_WDUNF              (1 << 0)  /* Bit 0:  Watchdog Underflow */
#define RSWDT_SR_WDERR              (1 << 1)  /* Bit 1:  Watchdog Error */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_RSWDT_H */
