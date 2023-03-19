/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_rstc.h
 * Reset Controller (RSTC) definitions for the SAMA5
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_RSTC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_RSTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/sama5/chip.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RSTC register offsets ****************************************************/

#define SAM_RSTC_CR_OFFSET      0x00       /* Control Register */
#define SAM_RSTC_SR_OFFSET      0x04       /* Status Register */
#define SAM_RSTC_MR_OFFSET      0x08       /* Mode Register  */

/* RSTC register addresses **************************************************/

#define SAM_RSTC_CR             (SAM_RSTC_VBASE+SAM_RSTC_CR_OFFSET)
#define SAM_RSTC_SR             (SAM_RSTC_VBASE+SAM_RSTC_SR_OFFSET)
#define SAM_RSTC_MR             (SAM_RSTC_VBASE+SAM_RSTC_MR_OFFSET)

/* RSTC register bit definitions ********************************************/

/* Reset Controller Control Register */

#define RSTC_CR_PROCRST         (1 << 0)   /* Bit 0:  Processor Reset */
#if defined(CONFIG_ARCH_CHIP_SAMA5D3)
#  define RSTC_CR_PERRST        (1 << 2)   /* Bit 2:  Peripheral Reset */
#endif
#define RSTC_CR_EXTRST          (1 << 3)   /* Bit 3:  External Reset */
#define RSTC_CR_KEY_SHIFT       (24)       /* Bits 24-31:  Password */
#define RSTC_CR_KEY_MASK        (0xff << RSTC_CR_KEY_SHIFT)
#  define RSTC_CR_KEY           (0xa5 << RSTC_CR_KEY_SHIFT)

/* Reset Controller Status Register */

#define RSTC_SR_URSTS           (1 << 0)   /* Bit 0:  User Reset Status */
#define RSTC_SR_RSTTYP_SHIFT    (8)        /* Bits 8-10:  Reset Type */
#define RSTC_SR_RSTTYP_MASK     (7 << RSTC_SR_RSTTYP_SHIFT)
#  define RSTC_SR_RSTTYP_PWRUP  (0 << RSTC_SR_RSTTYP_SHIFT) /* General Reset */
#  define RSTC_SR_RSTTYP_BACKUP (1 << RSTC_SR_RSTTYP_SHIFT) /* Backup Reset */
#  define RSTC_SR_RSTTYP_WDOG   (2 << RSTC_SR_RSTTYP_SHIFT) /* Watchdog Reset */
#  define RSTC_SR_RSTTYP_SWRST  (3 << RSTC_SR_RSTTYP_SHIFT) /* Software Reset */
#  define RSTC_SR_RSTTYP_NRST   (4 << RSTC_SR_RSTTYP_SHIFT) /* User Reset NRST pin */

#define RSTC_SR_NRSTL           (1 << 16)  /* Bit 16:  NRST Pin Level */
#define RSTC_SR_SRCMP           (1 << 17)  /* Bit 17:  Software Reset Command in Progress */

/* Reset Controller Mode Register */

#define RSTC_MR_URSTEN          (1 << 0)   /* Bit 0:  User Reset Enable */
#define RSTC_MR_URSTIEN         (1 << 4)   /* Bit 4:  User Reset Interrupt Enable */
#define RSTC_MR_ERSTL_SHIFT     (8)        /* Bits 8-11:  External Reset Length */
#define RSTC_MR_ERSTL_MASK      (15 << RSTC_MR_ERSTL_SHIFT)
#  define RSTC_MR_ERSTL(n)      ((uint32_t)(n) << RSTC_MR_ERSTL_SHIFT)
#define RSTC_MR_KEY_SHIFT       (24)       /* Bits 24-31:  Password */
#define RSTC_MR_KEY_MASK        (0xff << RSTC_CR_KEY_SHIFT)
#  define RSTC_MR_KEY           (0xa5 << RSTC_CR_KEY_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_RSTC_H */
