/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mz_prefetch.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PREFETCH_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PREFETCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mz_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Prefetch register offsets ************************************************/

#define PIC32MZ_PRECON_OFFSET   0x0000 /* Prefetch module control register */
#define PIC32MZ_PRESTAT_OFFSET  0x0000 /* Prefetch module status register */

/* Prefetch register addresses **********************************************/

#define PIC32MZ_PRECON          (PIC32MZ_PREFETCH_K1BASE+PIC32MZ_PRECON_OFFSET)
#define PIC32MZ_PRESTAT         (PIC32MZ_PREFETCH_K1BASE+PIC32MZ_PRESTAT_OFFSET)

/* Prefetch register bit field definitions **********************************/

/* Prefetch module control register */

#define PRECON_PFMWS_SHIFT      (0)       /* Bits 0-2: PFM Access Time */
#define PRECON_PFMWS_MASK       (7 << PRECON_PFMWS_SHIFT)
#  define PRECON_PFMWS(n)       ((uint32_t)(n) << PRECON_PFMWS_SHIFT) /* n wait states, n=0..7 */

#define PRECON_PREFEN_SHIFT     (4)       /* Bit 4-5: Predictive Prefetch Enable */
#define PRECON_PREFEN_MASK      (3 << PRECON_PREFEN_SHIFT)
#  define PRECON_PREFEN_DISABLE (0 << PRECON_PREFEN_SHIFT) /* Disable predictive prefetch */
#  define PRECON_PREFEN_CPUI    (1 << PRECON_PREFEN_SHIFT) /* Predictive prefetch CPU instructions */
#  define PRECON_PREFEN_CPUID   (2 << PRECON_PREFEN_SHIFT) /* Predictive prefetch CPU instructions and data */
#  define PRECON_PREFEN_ANY     (3 << PRECON_PREFEN_SHIFT) /* Predictive prefetch any address */

#define PRECON_PFMSECEN         (1 << 26) /* Bit 26: Flash SEC Interrupt Enable */

/* Prefetch module status register */

#define PRESTAT_PFMSECCNT_SHIFT (0)       /* Bits 0-7: Flash SEC Count bits */
#define PRESTAT_PFMSECCNT_MASK  (0xff << PRESTAT_PFMSECCNT_SHIFT)
#define PRESTAT_PFMSEC          (1 << 26) /* Bit 26: Flash Single-bit Error Corrected Status */
#define PRESTAT_PFMDED          (1 << 27) /* Bit 27: Flash Double-bit Error Detected Status */

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PREFETCH_H */
