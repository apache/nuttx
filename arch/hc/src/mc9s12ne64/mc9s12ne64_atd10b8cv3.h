/************************************************************************************
 * arch/hc/src/mc9s12ne64/mc9s12ne64_atd10b8cv3.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_ATD10B8CV3_H
#define __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_ATD10B8CV3_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_ATD_CTL0_OFFSET           0x0000 /* ATD control register 0 */
#define HCS12_ATD_CTL1_OFFSET           0x0001 /* ATD control register 1 */
#define HCS12_ATD_CTL2_OFFSET           0x0002 /* ATD control register 2 */
#define HCS12_ATD_CTL3_OFFSET           0x0003 /* ATD control register 3 */
#define HCS12_ATD_CTL4_OFFSET           0x0004 /* ATD control register 4 */
#define HCS12_ATD_CTL5_OFFSET           0x0005 /* ATD control register 5 */
#define HCS12_ATD_STAT0_OFFSET          0x0006 /* ATD status register 0 */
#define HCS12_ATD_TEST0_OFFSET          0x0008 /* ATD test register 0 */
#define HCS12_ATD_TEST1_OFFSET          0x0009 /* ATD test register 1 */
#define HCS12_ATD_STAT1_OFFSET          0x000b /* ATD status register 1 */
#define HCS12_ATD_IEN_OFFSET            0x000d /* ATD Input enable register */
#define HCS12_ATD_PORTAD_OFFSET         0x000f /* Port data register */
                                               /* Left justified data */
#define HCS12_ATD_DLH_OFFSET(n)         (0x0010+(n))
#define HCS12_ATD_DLL_OFFSET(n)         (0x0011+(n))
#define HCS12_ATD_DL0H_OFFSET           0x0010 /* ATD conversion result register 0 (high) */
#define HCS12_ATD_DL0L_OFFSET           0x0011 /* ATD conversion result register 0 (low) */
#define HCS12_ATD_DL1H_OFFSET           0x0012 /* ATD conversion result register 1 (high) */
#define HCS12_ATD_DL1L_OFFSET           0x0013 /* ATD conversion result register 1 (low) */
#define HCS12_ATD_DL2H_OFFSET           0x0014 /* ATD conversion result register 2 (high) */
#define HCS12_ATD_DL2L_OFFSET           0x0015 /* ATD conversion result register 2 (low) */
#define HCS12_ATD_DL3H_OFFSET           0x0016 /* ATD conversion result register 3 (high) */
#define HCS12_ATD_DL3L_OFFSET           0x0017 /* ATD conversion result register 3 (low) */
#define HCS12_ATD_DL4H_OFFSET           0x0018 /* ATD conversion result register 4 (high) */
#define HCS12_ATD_DL4L_OFFSET           0x0019 /* ATD conversion result register 4 (low) */
#define HCS12_ATD_DL5H_OFFSET           0x001a /* ATD conversion result register 5 (high) */
#define HCS12_ATD_DL5L_OFFSET           0x001b /* ATD conversion result register 5 (low) */
#define HCS12_ATD_DL6H_OFFSET           0x001c /* ATD conversion result register 6 (high) */
#define HCS12_ATD_DL6L_OFFSET           0x001d /* ATD conversion result register 6 (low) */
#define HCS12_ATD_DL7H_OFFSET           0x001e /* ATD conversion result register 7 (high) */
#define HCS12_ATD_DL7L_OFFSET           0x001f /* ATD conversion result register 7 (low) */
                                               /* Right justified data */
#define HCS12_ATD_DRH_OFFSET(n)         (0x0020+(n))
#define HCS12_ATD_DRL_OFFSET(n)         (0x0021+(n))
#define HCS12_ATD_DR0H_OFFSET           0x0020 /* ATD conversion result register 0 (high) */
#define HCS12_ATD_DR0L_OFFSET           0x0021 /* ATD conversion result register 0 (low) */
#define HCS12_ATD_DR1H_OFFSET           0x0022 /* ATD conversion result register 1 (high) */
#define HCS12_ATD_DR1L_OFFSET           0x0023 /* ATD conversion result register 1 (low) */
#define HCS12_ATD_DR2H_OFFSET           0x0024 /* ATD conversion result register 2 (high) */
#define HCS12_ATD_DR2L_OFFSET           0x0025 /* ATD conversion result register 2 (low) */
#define HCS12_ATD_DR3H_OFFSET           0x0026 /* ATD conversion result register 3 (high) */
#define HCS12_ATD_DR3L_OFFSET           0x0027 /* ATD conversion result register 3 (low) */
#define HCS12_ATD_DR4H_OFFSET           0x0028 /* ATD conversion result register 4 (high) */
#define HCS12_ATD_DR4L_OFFSET           0x0029 /* ATD conversion result register 4 (low) */
#define HCS12_ATD_DR5H_OFFSET           0x002a /* ATD conversion result register 5 (high) */
#define HCS12_ATD_DR5L_OFFSET           0x002b /* ATD conversion result register 5 (low) */
#define HCS12_ATD_DR6H_OFFSET           0x002c /* ATD conversion result register 6 (high) */
#define HCS12_ATD_DR6L_OFFSET           0x002d /* ATD conversion result register 6 (low) */
#define HCS12_ATD_DR7H_OFFSET           0x002e /* ATD conversion result register 7 (high) */
#define HCS12_ATD_DR7L_OFFSET           0x002f /* ATD conversion result register 7 (low) */

/* Register Addresses ***************************************************************/

#define HCS12_ATD_CTL0                  (HCS12_ATD_BASE+HCS12_ATD_CTL0_OFFSET)
#define HCS12_ATD_CTL1                  (HCS12_ATD_BASE+HCS12_ATD_CTL1_OFFSET)
#define HCS12_ATD_CTL2                  (HCS12_ATD_BASE+HCS12_ATD_CTL2_OFFSET)
#define HCS12_ATD_CTL3                  (HCS12_ATD_BASE+HCS12_ATD_CTL3_OFFSET)
#define HCS12_ATD_CTL4                  (HCS12_ATD_BASE+HCS12_ATD_CTL4_OFFSET)
#define HCS12_ATD_CTL5                  (HCS12_ATD_BASE+HCS12_ATD_CTL5_OFFSET)
#define HCS12_ATD_STAT0                 (HCS12_ATD_BASE+HCS12_ATD_STAT0_OFFSET)
#define HCS12_ATD_TEST0                 (HCS12_ATD_BASE+HCS12_ATD_TEST0_OFFSET)
#define HCS12_ATD_TEST1                 (HCS12_ATD_BASE+HCS12_ATD_TEST1_OFFSET)
#define HCS12_ATD_STAT1                 (HCS12_ATD_BASE+HCS12_ATD_STAT1_OFFSET)
#define HCS12_ATD_IEN                   (HCS12_ATD_BASE+HCS12_ATD_IEN_OFFSET)
#define HCS12_ATD_PORTAD                (HCS12_ATD_BASE+HCS12_ATD_PORTAD_OFFSET)
#define HCS12_ATD_DLH(n)                (HCS12_ATD_BASE+HCS12_ATD_DLH_OFFSET(n))
#define HCS12_ATD_DLL(n)                (HCS12_ATD_BASE+HCS12_ATD_DLL_OFFSET(n))
#define HCS12_ATD_DL0H                  (HCS12_ATD_BASE+HCS12_ATD_DL0H_OFFSET)
#define HCS12_ATD_DL0L                  (HCS12_ATD_BASE+HCS12_ATD_DL0L_OFFSET)
#define HCS12_ATD_DL1H                  (HCS12_ATD_BASE+HCS12_ATD_DL1H_OFFSET)
#define HCS12_ATD_DL1L                  (HCS12_ATD_BASE+HCS12_ATD_DL1L_OFFSET)
#define HCS12_ATD_DL2H                  (HCS12_ATD_BASE+HCS12_ATD_DL2H_OFFSET)
#define HCS12_ATD_DL2L                  (HCS12_ATD_BASE+HCS12_ATD_DL2L_OFFSET)
#define HCS12_ATD_DL3H                  (HCS12_ATD_BASE+HCS12_ATD_DL3H_OFFSET)
#define HCS12_ATD_DL3L                  (HCS12_ATD_BASE+HCS12_ATD_DL3L_OFFSET)
#define HCS12_ATD_DL4H                  (HCS12_ATD_BASE+HCS12_ATD_DL4H_OFFSET)
#define HCS12_ATD_DL4L                  (HCS12_ATD_BASE+HCS12_ATD_DL4L_OFFSET)
#define HCS12_ATD_DL5H                  (HCS12_ATD_BASE+HCS12_ATD_DL5H_OFFSET)
#define HCS12_ATD_DL5L                  (HCS12_ATD_BASE+HCS12_ATD_DL5L_OFFSET)
#define HCS12_ATD_DL6H                  (HCS12_ATD_BASE+HCS12_ATD_DL6H_OFFSET)
#define HCS12_ATD_DL6L                  (HCS12_ATD_BASE+HCS12_ATD_DL6L_OFFSET)
#define HCS12_ATD_DL7H                  (HCS12_ATD_BASE+HCS12_ATD_DL7H_OFFSET)
#define HCS12_ATD_DL7L                  (HCS12_ATD_BASE+HCS12_ATD_DL7L_OFFSET)
#define HCS12_ATD_DRH(n)                (HCS12_ATD_BASE+HCS12_ATD_DRH_OFFSET(n))
#define HCS12_ATD_DRL(n)                (HCS12_ATD_BASE+HCS12_ATD_DRL_OFFSET(n))
#define HCS12_ATD_DR0H                  (HCS12_ATD_BASE+HCS12_ATD_DR0H_OFFSET)
#define HCS12_ATD_DR0L                  (HCS12_ATD_BASE+HCS12_ATD_DR0L_OFFSET)
#define HCS12_ATD_DR1H                  (HCS12_ATD_BASE+HCS12_ATD_DR1H_OFFSET)
#define HCS12_ATD_DR1L                  (HCS12_ATD_BASE+HCS12_ATD_DR1L_OFFSET)
#define HCS12_ATD_DR2H                  (HCS12_ATD_BASE+HCS12_ATD_DR2H_OFFSET)
#define HCS12_ATD_DR2L                  (HCS12_ATD_BASE+HCS12_ATD_DR2L_OFFSET)
#define HCS12_ATD_DR3H                  (HCS12_ATD_BASE+HCS12_ATD_DR3H_OFFSET)
#define HCS12_ATD_DR3L                  (HCS12_ATD_BASE+HCS12_ATD_DR3L_OFFSET)
#define HCS12_ATD_DR4H                  (HCS12_ATD_BASE+HCS12_ATD_DR4H_OFFSET)
#define HCS12_ATD_DR4L                  (HCS12_ATD_BASE+HCS12_ATD_DR4L_OFFSET)
#define HCS12_ATD_DR5H                  (HCS12_ATD_BASE+HCS12_ATD_DR5H_OFFSET)
#define HCS12_ATD_DR5L                  (HCS12_ATD_BASE+HCS12_ATD_DR5L_OFFSET)
#define HCS12_ATD_DR6H                  (HCS12_ATD_BASE+HCS12_ATD_DR6H_OFFSET)
#define HCS12_ATD_DR6L                  (HCS12_ATD_BASE+HCS12_ATD_DR6L_OFFSET)
#define HCS12_ATD_DR7H                  (HCS12_ATD_BASE+HCS12_ATD_DR7H_OFFSET)
#define HCS12_ATD_DR7L                  (HCS12_ATD_BASE+HCS12_ATD_DR7L_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* ATD control register 0 */
#define ATD_CTL0_

/* ATD control register 1 */
#define ATD_CTL1_

/* ATD control register 2 */
#define ATD_CTL2_

/* ATD control register 3 */
#define ATD_CTL3_

/* ATD control register 4 */
#define ATD_CTL4_

/* ATD control register 5 */
#define ATD_CTL5_

/* ATD status register 0 */
#define ATD_STAT0_

/* ATD test register 0 -- 8 MS bits of data written in special mode */
/* ATD test register 1 */

#define ATD_TEST1_SC                    (1 << 0)  /* Bit 0: Enable special conversions */
#define ATD_TEST1_LSU_MASK              0xc0      /* Bits 6-7: 2 LS bits of special mode data */

/* ATD status register 1 */

#define ATD_STAT1_CCF(n)                (1 << (n)) /* Bit n: Conversion complete flag channel n */
#define ATD_STAT1_CCF0                  (1 << 0)  /* Bit 0: Conversion complete flag channel 0 */
#define ATD_STAT1_CCF1                  (1 << 1)  /* Bit 1: Conversion complete flag channel 1 */
#define ATD_STAT1_CCF2                  (1 << 2)  /* Bit 2: Conversion complete flag channel 2 */
#define ATD_STAT1_CCF3                  (1 << 3)  /* Bit 3: Conversion complete flag channel 3 */
#define ATD_STAT1_CCF4                  (1 << 4)  /* Bit 4: Conversion complete flag channel 4 */
#define ATD_STAT1_CCF5                  (1 << 5)  /* Bit 5: Conversion complete flag channel 5 */
#define ATD_STAT1_CCF6                  (1 << 6)  /* Bit 6: Conversion complete flag channel 6 */
#define ATD_STAT1_CCF7                  (1 << 7)  /* Bit 7: Conversion complete flag channel 7 */

/* ATD Input enable register */

#define ATD_IEN(n)                     (1 << (n)) /* Bit n: ATD Digital Input Enable on channel n */
#define ATD_IEN0                       (1 << 0)  /* Bit 0: ATD Digital Input Enable on channel 0 */
#define ATD_IEN1                       (1 << 1)  /* Bit 1: ATD Digital Input Enable on channel 1 */
#define ATD_IEN2                       (1 << 2)  /* Bit 2: ATD Digital Input Enable on channel 2 */
#define ATD_IEN3                       (1 << 3)  /* Bit 3: ATD Digital Input Enable on channel 3 */
#define ATD_IEN4                       (1 << 4)  /* Bit 4: ATD Digital Input Enable on channel 4 */
#define ATD_IEN5                       (1 << 5)  /* Bit 5: ATD Digital Input Enable on channel 5 */
#define ATD_IEN6                       (1 << 6)  /* Bit 6: ATD Digital Input Enable on channel 6 */
#define ATD_IEN7                       (1 << 7)  /* Bit 7: ATD Digital Input Enable on channel 7 */

/* Port data register */

#define ATD_PORTAD(n)                     (1 << (n)) /* Bit n: A/D Channel n (ANn) Digital Input */
#define ATD_PORTAD0                       (1 << 0)  /* Bit 0: A/D Channel 0 (AN0) Digital Input */
#define ATD_PORTAD1                       (1 << 1)  /* Bit 1: A/D Channel 1 (AN1) Digital Input */
#define ATD_PORTAD2                       (1 << 2)  /* Bit 2: A/D Channel 2 (AN2) Digital Input */
#define ATD_PORTAD3                       (1 << 3)  /* Bit 3: A/D Channel 3 (AN3) Digital Input */
#define ATD_PORTAD4                       (1 << 4)  /* Bit 4: A/D Channel 4 (AN4) Digital Input */
#define ATD_PORTAD5                       (1 << 5)  /* Bit 5: A/D Channel 5 (AN5) Digital Input */
#define ATD_PORTAD6                       (1 << 6)  /* Bit 6: A/D Channel 6 (AN6) Digital Input */
#define ATD_PORTAD7                       (1 << 7)  /* Bit 7: A/D Channel 7 (AN7) Digital Input */

/* ATD conversion result register 0-7 (high/l;ow) */

#define ATD_DLL_MASK 0xc0  /* Bits 6-7: LS bits of left justified data */
#define ATD_DRH_MASK 0x03  /* Bits 0-1: MS bits of right justified data */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_ATD10B8CV3_H */
