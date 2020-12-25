/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_aux_smph.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible
 * BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AUX_SMPH_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AUX_SMPH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AUX SMPH Register Offsets ************************************************/

#define TIVA_AUX_SMPH_SMPH_OFFSET         (0x0000 + ((n) << 2)
#  define TIVA_AUX_SMPH_SMPH0_OFFSET      0x0000  /* Semaphore 0 */
#  define TIVA_AUX_SMPH_SMPH1_OFFSET      0x0004  /* Semaphore 1 */
#  define TIVA_AUX_SMPH_SMPH2_OFFSET      0x0008  /* Semaphore 2 */
#  define TIVA_AUX_SMPH_SMPH3_OFFSET      0x000c  /* Semaphore 3 */
#  define TIVA_AUX_SMPH_SMPH4_OFFSET      0x0010  /* Semaphore 4 */
#  define TIVA_AUX_SMPH_SMPH5_OFFSET      0x0014  /* Semaphore 5 */
#  define TIVA_AUX_SMPH_SMPH6_OFFSET      0x0018  /* Semaphore 6 */
#  define TIVA_AUX_SMPH_SMPH7_OFFSET      0x001c  /* Semaphore 7 */
#define TIVA_AUX_SMPH_AUTOTAKE_OFFSET     0x0020  /* Auto Take */

/* AUX SMPH Register Addresses **********************************************/

#define TIVA_AUX_SMPH_SMPH(n)             (TIVA_AUX_SMPH_BASE + TIVA_AUX_SMPH_SMPH_OFFSET(n))
#  define TIVA_AUX_SMPH_SMPH0             (TIVA_AUX_SMPH_BASE + TIVA_AUX_SMPH_SMPH0_OFFSET)
#  define TIVA_AUX_SMPH_SMPH1             (TIVA_AUX_SMPH_BASE + TIVA_AUX_SMPH_SMPH1_OFFSET)
#  define TIVA_AUX_SMPH_SMPH2             (TIVA_AUX_SMPH_BASE + TIVA_AUX_SMPH_SMPH2_OFFSET)
#  define TIVA_AUX_SMPH_SMPH3             (TIVA_AUX_SMPH_BASE + TIVA_AUX_SMPH_SMPH3_OFFSET)
#  define TIVA_AUX_SMPH_SMPH4             (TIVA_AUX_SMPH_BASE + TIVA_AUX_SMPH_SMPH4_OFFSET)
#  define TIVA_AUX_SMPH_SMPH5             (TIVA_AUX_SMPH_BASE + TIVA_AUX_SMPH_SMPH5_OFFSET)
#  define TIVA_AUX_SMPH_SMPH6             (TIVA_AUX_SMPH_BASE + TIVA_AUX_SMPH_SMPH6_OFFSET)
#  define TIVA_AUX_SMPH_SMPH7             (TIVA_AUX_SMPH_BASE + TIVA_AUX_SMPH_SMPH7_OFFSET)
#define TIVA_AUX_SMPH_AUTOTAKE            (TIVA_AUX_SMPH_BASE + TIVA_AUX_SMPH_AUTOTAKE_OFFSET)

/* AUX SMPH Register Bitfield Definitions **(********************************/

/* TIVA_AUX_SMPH_SMPH0-TIVA_AUX_SMPH_SMPH7 */

#define AUX_SMPH_SMPH_STAT                (1 << 0)  /* Bit 0:  Semaphore granted */
#  define AUX_SMPH_SMPH0_STAT             AUX_SMPH_SMPH_STAT
#  define AUX_SMPH_SMPH1_STAT             AUX_SMPH_SMPH_STAT
#  define AUX_SMPH_SMPH2_STAT             AUX_SMPH_SMPH_STAT
#  define AUX_SMPH_SMPH3_STAT             AUX_SMPH_SMPH_STAT
#  define AUX_SMPH_SMPH4_STAT             AUX_SMPH_SMPH_STAT
#  define AUX_SMPH_SMPH5_STAT             AUX_SMPH_SMPH_STAT
#  define AUX_SMPH_SMPH6_STAT             AUX_SMPH_SMPH_STAT
#  define AUX_SMPH_SMPH7_STAT             AUX_SMPH_SMPH_STAT

/* TIVA_AUX_SMPH_AUTOTAKE */

#define AUX_SMPH_AUTOTAKE_SMPH_ID_SHIFT   (0)       /* Bits 0-7: Write the semaphore
                                                     * ID 0-7 to SMPH_ID to request
                                                     * semaphore until AUX_EVCTL:EVSTAT3.
                                                     * AUX_SMPH_AUTOTAKE_DONE */
#define AUX_SMPH_AUTOTAKE_SMPH_ID_MASK    (7 << AUX_SMPH_AUTOTAKE_SMPH_ID_SHIFT)
#  define AUX_SMPH_AUTOTAKE_SMPH_ID(n)    ((uint32_t)(n) << AUX_SMPH_AUTOTAKE_SMPH_ID_SHIFT)

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AUX_SMPH_H */
