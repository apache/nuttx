/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_bmx.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_BMX_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_BMX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PIC32MX_BMX_CON_OFFSET      0x0000 /* Configuration Register */
#define PIC32MX_BMX_CONCLR_OFFSET   0x0004 /* Configuration Clear Register */
#define PIC32MX_BMX_CONSET_OFFSET   0x0008 /* Configuration Set Register */
#define PIC32MX_BMX_CONINV_OFFSET   0x000c /* Configuration Invert Register */
#define PIC32MX_BMX_DKPBA_OFFSET    0x0010 /* Data RAM Kernel Program Base Address Register */
#define PIC32MX_BMX_DKPBACLR_OFFSET 0x0014 /* Data RAM Kernel Program Base Address Clear Register */
#define PIC32MX_BMX_DKPBASET_OFFSET 0x0018 /* Data RAM Kernel Program Base Address Set Register */
#define PIC32MX_BMX_DKPBAINV_OFFSET 0x001c /* Data RAM Kernel Program Base Address Invert Register */
#define PIC32MX_BMX_DUDBA_OFFSET    0x0020 /* Data RAM User Data Base Address Register */
#define PIC32MX_BMX_DUDBACLR_OFFSET 0x0024 /* Data RAM User Data Base Address Clear Register */
#define PIC32MX_BMX_DUDBASET_OFFSET 0x0028 /* Data RAM User Data Base Address Set Register */
#define PIC32MX_BMX_DUDBAINV_OFFSET 0x002c /* Data RAM User Data Base Address Invert Register */
#define PIC32MX_BMX_DUPBA_OFFSET    0x0030 /* Data RAM User Program Base Address Register */
#define PIC32MX_BMX_DUPBACLR_OFFSET 0x0034 /* Data RAM User Program Base Address Clear Register */
#define PIC32MX_BMX_DUPBASET_OFFSET 0x0038 /* Data RAM User Program Base Address Set Register */
#define PIC32MX_BMX_DUPBAINV_OFFSET 0x003c /* Data RAM User Program Base Address Invert Register */
#define PIC32MX_BMX_DRMSZ_OFFSET    0x0040 /* Data RAM Size Register */
#define PIC32MX_BMX_PUPBA_OFFSET    0x0050 /* Program Flash (PFM) User Program Base Address Register */
#define PIC32MX_BMX_PUPBACLR_OFFSET 0x0054 /* Program Flash (PFM) User Program Base Address Clear Register */
#define PIC32MX_BMX_PUPBASET_OFFSET 0x0058 /* Program Flash (PFM) User Program Base Address Set Register */
#define PIC32MX_BMX_PUPBINVA_OFFSET 0x005c /* Program Flash (PFM) User Program Base Address Invert Register */
#define PIC32MX_BMX_PFMSZ_OFFSET    0x0060 /* Program Flash Size Register */
#define PIC32MX_BMX_BOOTSZ_OFFSET   0x0070 /* Boot Flash Size Register */

/* Register Addresses *******************************************************/

#define PIC32MX_BMX_CON             (PIC32MX_BMX_K1BASE+PIC32MX_BMX_CON_OFFSET)
#define PIC32MX_BMX_CONCLR          (PIC32MX_BMX_K1BASE+PIC32MX_BMX_CONCLR_OFFSET)
#define PIC32MX_BMX_CONSET          (PIC32MX_BMX_K1BASE+PIC32MX_BMX_CONSET_OFFSET)
#define PIC32MX_BMX_CONINV          (PIC32MX_BMX_K1BASE+PIC32MX_BMX_CONINV_OFFSET)
#define PIC32MX_BMX_DKPBA           (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DKPBA_OFFSET)
#define PIC32MX_BMX_DKPBACLR        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DKPBACLR_OFFSET)
#define PIC32MX_BMX_DKPBASET        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DKPBASET_OFFSET)
#define PIC32MX_BMX_DKPBAINV        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DKPBAINV_OFFSET)
#define PIC32MX_BMX_DUDBA           (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DUDBA_OFFSET)
#define PIC32MX_BMX_DUDBACLR        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DUDBACLR_OFFSET)
#define PIC32MX_BMX_DUDBASET        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DUDBASET_OFFSET)
#define PIC32MX_BMX_DUDBAINV        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DUDBAINV_OFFSET)
#define PIC32MX_BMX_DUPBA           (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DUPBA_OFFSET)
#define PIC32MX_BMX_DUPBACLR        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DUPBACLR_OFFSET)
#define PIC32MX_BMX_DUPBASET        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DUPBASET_OFFSET)
#define PIC32MX_BMX_DUPBAINV        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DUPBAINV_OFFSET)
#define PIC32MX_BMX_DRMSZ           (PIC32MX_BMX_K1BASE+PIC32MX_BMX_DRMSZ_OFFSET)
#define PIC32MX_BMX_PUPBA           (PIC32MX_BMX_K1BASE+PIC32MX_BMX_PUPBA_OFFSET)
#define PIC32MX_BMX_PUPBACLR        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_PUPBACLR_OFFSET)
#define PIC32MX_BMX_PUPBASET        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_PUPBASET_OFFSET)
#define PIC32MX_BMX_PUPBINVA        (PIC32MX_BMX_K1BASE+PIC32MX_BMX_PUPBINVA_OFFSET)
#define PIC32MX_BMX_PFMSZ           (PIC32MX_BMX_K1BASE+PIC32MX_BMX_PFMSZ_OFFSET)
#define PIC32MX_BMX_BOOTSZ          (PIC32MX_BMX_K1BASE+PIC32MX_BMX_BOOTSZ_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Configuration Register */

#define BMX_CON_BMXARB_SHIFT        (0)       /* Bits 0-2: : Bus matrix arbitration mode */
#define BMX_CON_BMXARB_MASK         (7 << BMX_CON_BMXARB_SHIFT)
#  define BMX_CON_BMXARB(n)         ((n) << BMX_CON_BMXARB_SHIFT) /* Mode n, n=0,1,2 */

#define BMX_CON_BMXWSDRM            (1 << 6)  /* Bit 6:  CPU Instruction or data access from data RAM wait state */
#define BMX_CON_BMXERRIS            (1 << 16) /* Bit 16: Bus error from CPU instruction access */
#define BMX_CON_BMXERRDS            (1 << 17) /* Bit 17: Bus error from CPU data access */
#define BMX_CON_BMXERRDMA           (1 << 18) /* Bit 18: Bus error from DMA */
#define BMX_CON_BMXERRICD           (1 << 19) /* Bit 19: Enable bus error from ICD debug unit  */
#define BMX_CON_BMXERRIXI           (1 << 20) /* Bit 20: Enable bus error from IXI */
#define BMX_CON_BMXCHEDMA           (1 << 26) /* Bit 26: BMX PFM cacheability for DMA accesses */

/* Data RAM Kernel Program Base Address Register */

#define BMX_DKPBA_MASK              0x0000ffff /* Bits 0-15 */

/* Data RAM User Data Base Address Register */

#define BMX_DUDBA_MASK              0x0000ffff /* Bits 0-15 */

/* Data RAM User Program Base Address Register */

#define BMX_DUPBA_MASK              0x0000ffff /* Bits 0-15 */

/* Data RAM Size Register -- 32-bit size value */

/* Program Flash (PFM) User Program Base Address Register */

#define BMX_PUPBA_MASK              0x000fffff /* Bits 0-19 */

/* Program Flash Size Register -- 32-bit size value */

/* Boot Flash Size Register -- 32-bit size value */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_BMX_H */
