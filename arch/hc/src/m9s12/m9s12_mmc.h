/****************************************************************************
 * arch/hc/src/m9s12/m9s12_mmc.h
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

#ifndef __ARCH_HC_SRC_M9S12_M9S12_MMC_H
#define __ARCH_HC_SRC_M9S12_M9S12_MMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Offsets relative to CORE1 */

#define HCS12_MMC_INITRM_OFFSET    0x0010 /* Internal RAM Position Register */
#define HCS12_MMC_INITRG_OFFSET    0x0011 /* Internal Registers Position Register */
#define HCS12_MMC_INITEE_OFFSET    0x0012 /* Internal EEPROM Position Register */
#define HCS12_MMC_MISC_OFFSET      0x0013 /* Miscellaneous System Control Register */
#define HCS12_MMC_MTST0_OFFSET     0x0014 /* Reserved Test Register 0 */
#define HCS12_MMC_MTST1_OFFSET     0x0017 /* Reserved Test Register 1 */

/* Offsets relative to CORE2 */

#define HCS12_MMC_MEMSIZ0_OFFSET   0x0000 /* Memory Size Register 0 */
#define HCS12_MMC_MEMSIZ1_OFFSET   0x0001 /* Memory Size Register 1 */

/* Offsets relative to CORE4 */

#define HCS12_MMC_PPAGE_OFFSET     0x0000 /* Program Page Index Register */

/* Register Addresses *******************************************************/

#define HCS12_MMC_INITRM           (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MMC_INITRM_OFFSET)
#define HCS12_MMC_INITRG           (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MMC_INITRG_OFFSET)
#define HCS12_MMC_INITEE           (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MMC_INITEE_OFFSET)
#define HCS12_MMC_MISC             (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MMC_MISC_OFFSET)
#define HCS12_MMC_MTST0            (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MMC_MTST0_OFFSET)
#define HCS12_MMC_MTST1            (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MMC_MTST1_OFFSET)
#define HCS12_MMC_MEMSIZ0          (HCS12_REG_BASE+HCS12_CORE2_BASE+HCS12_MMC_MEMSIZ0_OFFSET)
#define HCS12_MMC_MEMSIZ1          (HCS12_REG_BASE+HCS12_CORE2_BASE+HCS12_MMC_MEMSIZ1_OFFSET)
#define HCS12_MMC_PPAGE            (HCS12_REG_BASE+HCS12_CORE4_BASE+HCS12_MMC_PPAGE_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Internal RAM Position Register Bit-Field Definitions */

#define MMC_INITRM_RAMHAL        (1 << 0) /* Bit 0: RAM High-Align */
#define MMC_INITRM_RAM_SHIFT     (3)      /* Bits 3-7: Internal RAM Map Position */
#define MMC_INITRM_RAM_MASK      (0x1f << MMC_INITRM_RAM_SHIFT)
#define MMC_INITRM_RAM(addr)     (((addr) >> (11-MMC_INITRM_RAM_SHIFT)) & MMC_INITRM_RAM_MASK)

/* Internal Registers Position Register Bit-Field Definitions */

#define MMC_INITRG_REG_SHIFT     (3)      /* Bits 3-6: Internal RAM Map Position */
#define MMC_INITRG_REG_MASK      (0x0f << MMC_INITRG_REG_SHIFT)
#define MMC_INITRG_REG(addr)     (((addr) >> (11-MMC_INITRG_REG_SHIFT)) & MMC_INITRG_REG_SHIFT)

/* Internal EEPROM Position Register Bit-Field Definitions */

#define MMC_INITEE_EEON          (1 << 0)  /* Bit 0: Enable EEPROM */
#define MMC_INITEE_EE_SHIFT      (3)       /* Bits 3-7: Internal EEPROM Map Position */
#define MMC_INITEE_EE_MASK       (0x1f << MMC_INITEE_EE_SHIFT)
#define MMC_INITEE_EE(addr)      (((addr) >> (11-MMC_INITRG_REG_SHIFT)) & MMC_INITRG_REG_SHIFT)

/* Miscellaneous System Control Register Bit-Field Definitions */

#define MMC_MISC_ROMON           (1 << 0) /* Bit 0: Enable FLASH EEPROM or ROM */
#define MMC_MISC_ROMHM           (1 << 1) /* Bit 1: FLASH EEPROM or ROM Only in Second Half of Memory Map */
#define MMC_MISC_EXSTR_SHIFT     (2)      /* Bits 2-3: External Access Stretch Bits */
#define MMC_MISC_EXSTR_MASK      (3 << MMC_MISC_EXSTR_SHIFT)
#  define MISC_EXSTR_CLKS0       (0 << MMC_MISC_EXSTR_SHIFT)
#  define MISC_EXSTR_CLKS1       (1 << MMC_MISC_EXSTR_SHIFT)
#  define MISC_EXSTR_CLKS2       (2 << MMC_MISC_EXSTR_SHIFT)
#  define MISC_EXSTR_CLKS3       (3 << MMC_MISC_EXSTR_SHIFT)

/* Reserved Test Register 0/1 Bit-Field Definitions -- Not documented */

/* Memory Size Register 0 Bit-Field Definitions */

#define MMC_MEMSIZ0_RAMSW_SHIFT (0)       /* Bits 0-2: Allocated System RAM Memory Space */
#define MMC_MEMSIZ0_RAMSW_MASK  (7 << MMC_MEMSIZ0_RAMSW_SHIFT)
#  define MEMSIZ0_RAMSW_2KB     (0 << MMC_MEMSIZ0_RAMSW_SHIFT)
#  define MEMSIZ0_RAMSW_4KB     (1 << MMC_MEMSIZ0_RAMSW_SHIFT)
#  define MEMSIZ0_RAMSW_6KB     (2 << MMC_MEMSIZ0_RAMSW_SHIFT)
#  define MEMSIZ0_RAMSW_8KB     (3 << MMC_MEMSIZ0_RAMSW_SHIFT)
#  define MEMSIZ0_RAMSW_10KB    (4 << MMC_MEMSIZ0_RAMSW_SHIFT)
#  define MEMSIZ0_RAMSW_12KB    (5 << MMC_MEMSIZ0_RAMSW_SHIFT)
#  define MEMSIZ0_RAMSW_14KB    (6 << MMC_MEMSIZ0_RAMSW_SHIFT)
#  define MEMSIZ0_RAMSW_16KB    (7 << MMC_MEMSIZ0_RAMSW_SHIFT)
#define MMC_MEMSIZ0_EEPSW_SHIFT (4)       /* Bits 4-5: Allocated EEPROM Memory Space */
#define MMC_MEMSIZ0_EEPSW_MASK  (3 << MMC_MEMSIZ0_EEPSW_SHIFT)
#  define MEMSIZ0_EEPSW_OKB     (0 << MMC_MEMSIZ0_EEPSW_MASK)
#  define MEMSIZ0_EEPSW_2KB     (1 << MMC_MEMSIZ0_EEPSW_MASK)
#  define MEMSIZ0_EEPSW_4KB     (2 << MMC_MEMSIZ0_EEPSW_MASK)
#  define MEMSIZ0_EEPSW_5KB     (3 << MMC_MEMSIZ0_EEPSW_MASK)
#define MMC_MEMSIZ0_REGSW       (1 << 7)  /* Bits 7: Allocated System Register Space */

/* Memory Size Register 1 Bit-Field Definitions */

#define MMC_MEMSIZ1_PAGSW_SHIFT (0)       /* Bits 0-1: Allocated System RAM Memory Space */
#define MMC_MEMSIZ1_PAGSW_MASK  (3 << MMC_MEMSIZ1_PAGSW_SHIFT)
#  define MEMSIZ1_PAGSW_128KB   (0 << MMC_MEMSIZ1_PAGSW_SHIFT)
#  define MEMSIZ1_PAGSW_256KB   (1 << MMC_MEMSIZ1_PAGSW_SHIFT)
#  define MEMSIZ1_PAGSW_512KB   (2 << MMC_MEMSIZ1_PAGSW_SHIFT)
#  define MEMSIZ1_PAGSW_1MB     (3 << MMC_MEMSIZ1_PAGSW_SHIFT)
#define MMC_MEMSIZ1_ROMSW_SHIFT (6)       /* Bits 6-7: Allocated System RAM Memory Space */
#define MMC_MEMSIZ1_ROMSW_MASK  (3 << MMC_MEMSIZ1_ROMSW_SHIFT)
#  define MEMSIZ1_ROMSW_0KB     (0 << MMC_MEMSIZ1_ROMSW_SHIFT)
#  define MEMSIZ1_ROMSW_16KB    (1 << MMC_MEMSIZ1_ROMSW_SHIFT)
#  define MEMSIZ1_ROMSW_48KB    (2 << MMC_MEMSIZ1_ROMSW_SHIFT)
#  define MEMSIZ1_ROMSW_64KB    (3 << MMC_MEMSIZ1_ROMSW_SHIFT)

/* Program Page Index Register Bit-Field Definitions */

#define MMC_PPAGE_PIX_SHIFT     (0)       /* Bits 0-5 Program Page Index Bits */
#define MMC_PPAGE_PIX_MASK      (0x3f << MMC_PPAGE_PIX_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_HC_SRC_M9S12_M9S12_MMC_H */
