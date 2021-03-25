/****************************************************************************
 * arch/arm/src/tiva/hardware/lm/lm3s_flash.h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM3S_FLASH_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM3S_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FLASH dimensions *********************************************************/

#if defined(CONFIG_ARCH_CHIP_LM3S6965) || defined(CONFIG_ARCH_CHIP_LM4F120) || \
    defined(CONFIG_ARCH_CHIP_LM3S8962) || defined(CONFIG_ARCH_CHIP_LM3S9B96) || \
    defined(CONFIG_ARCH_CHIP_LM3S9B92) || \
    defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB) || defined(CONFIG_ARCH_CHIP_TM4C123GH6PM) || \
    defined(CONFIG_ARCH_CHIP_TM4C123AH6PM)

/* These parts all support a 1KiB erase page size and a total FLASH memory
 * size of 256Kib or 256 pages.
 */

#  define TIVA_FLASH_NPAGES        256
#  define TIVA_FLASH_PAGESIZE      1024
#endif

#define TIVA_FLASH_SIZE            (TIVA_FLASH_NPAGES * TIVA_FLASH_PAGESIZE)

/* FLASH register offsets ***************************************************/

/* The FMA, FMD, FMC, FCRIS, FCIM, and FCMISC registers are relative to the
 * Flash control base address of TIVA_FLASHCON_BASE.
 */

#define TIVA_FLASH_FMA_OFFSET      0x000 /* Flash memory address */
#define TIVA_FLASH_FMD_OFFSET      0x004 /* Flash memory data */
#define TIVA_FLASH_FMC_OFFSET      0x008 /* Flash memory control */
#define TIVA_FLASH_FCRIS_OFFSET    0x00c /* Flash controller raw interrupt status */
#define TIVA_FLASH_FCIM_OFFSET     0x010 /* Flash controller interrupt mask */
#define TIVA_FLASH_FCMISC_OFFSET   0x014 /* Flash controller masked interrupt status and clear */

/* The FMPREn, FMPPEn, USECRL, USER_DBG, and USER_REGn registers are relative
 * to the System Control base address of TIVA_SYSCON_BASE
 */

#define TIVA_FLASH_FMPRE_OFFSET    0x130 /* Flash memory protection read enable */
#define TIVA_FLASH_FMPPE_OFFSET    0x134 /* Flash memory protection program enable */
#define TIVA_FLASH_USECRL_OFFSET   0x140 /* USec Reload */
#define TIVA_FLASH_USERDBG_OFFSET  0x1d0 /* User Debug */
#define TIVA_FLASH_USERREG0_OFFSET 0x1e0 /* User Register 0 */
#define TIVA_FLASH_USERREG1_OFFSET 0x1e4 /* User Register 1 */
#define TIVA_FLASH_FMPRE0_OFFSET   0x200 /* Flash Memory Protection Read Enable 0 */
#define TIVA_FLASH_FMPRE1_OFFSET   0x204 /* Flash Memory Protection Read Enable 1 */
#define TIVA_FLASH_FMPRE2_OFFSET   0x208 /* Flash Memory Protection Read Enable 2 */
#define TIVA_FLASH_FMPRE3_OFFSET   0x20c /* Flash Memory Protection Read Enable 3 */
#define TIVA_FLASH_FMPPE0_OFFSET   0x400 /* Flash Memory Protection Program Enable 0 */
#define TIVA_FLASH_FMPPE1_OFFSET   0x404 /* Flash Memory Protection Program Enable 1 */
#define TIVA_FLASH_FMPPE2_OFFSET   0x408 /* Flash Memory Protection Program Enable 2 */
#define TIVA_FLASH_FMPPE3_OFFSET   0x40c /*  Flash Memory Protection Program Enable 3 */

/* FLASH register addresses *************************************************/

/* The FMA, FMD, FMC, FCRIS, FCIM, and FCMISC registers are relative to the
 * Flash control base address of TIVA_FLASHCON_BASE.
 */

#define TIVA_FLASH_FMA             (TIVA_FLASHCON_BASE + TIVA_FLASH_FMA_OFFSET)
#define TIVA_FLASH_FMD             (TIVA_FLASHCON_BASE + TIVA_FLASH_FMD_OFFSET)
#define TIVA_FLASH_FMC             (TIVA_FLASHCON_BASE + TIVA_FLASH_FMC_OFFSET)
#define TIVA_FLASH_FCRIS           (TIVA_FLASHCON_BASE + TIVA_FLASH_FCRIS_OFFSET)
#define TIVA_FLASH_FCIM            (TIVA_FLASHCON_BASE + TIVA_FLASH_FCIM_OFFSET)
#define TIVA_FLASH_FCMISC          (TIVA_FLASHCON_BASE + TIVA_FLASH_FCMISC_OFFSET)

/* The FMPREn, FMPPEn, USECRL, USER_DBG, and USER_REGn registers are relative
 * to the System Control base address of TIVA_SYSCON_BASE
 */

#define TIVA_FLASH_FMPRE           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE_OFFSET)
#define TIVA_FLASH_FMPPE           (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE_OFFSET)
#define TIVA_FLASH_USECRL          (TIVA_SYSCON_BASE + TIVA_FLASH_USECRL_OFFSET)
#define TIVA_FLASH_USERDBG         (TIVA_SYSCON_BASE + TIVA_FLASH_USERDBG_OFFSET)
#define TIVA_FLASH_USERREG0        (TIVA_SYSCON_BASE + TIVA_FLASH_USERREG0_OFFSET)
#define TIVA_FLASH_USERREG1        (TIVA_SYSCON_BASE + TIVA_FLASH_USERREG1_OFFSET)
#define TIVA_FLASH_FMPRE0          (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE0_OFFSET)
#define TIVA_FLASH_FMPRE1          (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE1_OFFSET)
#define TIVA_FLASH_FMPRE2          (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE2_OFFSET)
#define TIVA_FLASH_FMPRE3          (TIVA_SYSCON_BASE + TIVA_FLASH_FMPRE3_OFFSET)
#define TIVA_FLASH_FMPPE0          (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE0_OFFSET)
#define TIVA_FLASH_FMPPE1          (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE1_OFFSET)
#define TIVA_FLASH_FMPPE2          (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE2_OFFSET)
#define TIVA_FLASH_FMPPE3          (TIVA_SYSCON_BASE + TIVA_FLASH_FMPPE3_OFFSET)

/* FLASH register bit definitions *******************************************/

#define FLASH_FMA_OFFSET_SHIFT     0         /* Bits 17-0: Address Offset */
#define FLASH_FMA_OFFSET_MASK      (0x0003ffff << FLASH_FMA_OFFSET_SHIFT)

#define FLASH_FMC_WRITE            (1 << 0)  /* Write a Word into Flash Memory */
#define FLASH_FMC_ERASE            (1 << 1)  /* Erase a Page of Flash Memory */
#define FLASH_FMC_MERASE           (1 << 2)  /* Mass Erase Flash Memory */
#define FLASH_FMC_COMT             (1 << 3)  /* Commit Register Value */

/* This field contains a write key, which is used to minimize the incidence
 * of accidental flash writes. The value 0xA442 must be written into this
 * field for a write to occur. Writes to the FMC register without this WRKEY
 * value are ignored. A read of this field returns the value 0
 */
#define FLASH_FMC_WRKEY_SHIFT      16         /* Bits 16-31:  Flash Write Key */
#define FLASH_FMC_WRKEY_MASK       (0xffff << FLASH_FMC_WRKEY_SHIFT)
#define FLASH_FMC_WRKEY            (0xa442 << FLASH_FMC_WRKEY_SHIFT) /* Magic write key */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM3S_FLASH_H */
