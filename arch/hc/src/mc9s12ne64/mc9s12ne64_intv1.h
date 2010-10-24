/************************************************************************************
 * arch/hc/src/mc9s12ne64/mc9s12ne64_intv1.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_INTV1_H
#define __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_INTV1_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_INT_ITCR_OFFSET          0x0015 /* Interrupt Test Control Register */
#define HCS12_INT_ITEST_OFFSET         0x0016 /* Interrupt Test Registers */
#define HCS12_INT_HPRIO_OFFSET         0x001f /* Highest Priority Interrupt */

/* Register Addresses ***************************************************************/

#define HCS12_INT_ITCR                 (HCS12_CORE1_BASE+HCS12_INT_ITCR_OFFSET)
#define HCS12_INT_ITEST                (HCS12_CORE1_BASE+HCS12_INT_ITEST_OFFSET)
#define HCS12_INT_HPRIO                (HCS12_CORE1_BASE+HCS12_INT_HPRIO_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* nterrupt Test Control Register Bit-Field Definitions */

#define INT_ITCR_ADR_SHIFT             (0)       /* Bits 0-3: Test register select */
#define INT_ITCR_ADR_MASK              (15 << INT_ITCR_ADR_SHIFT)
#define INT_ITCR_WRTINT                (1 << 4)  /* Bit 4:  Write to the Interrupt Test Registers */

/* Interrupt Test Registers Bit-Field Definitions */

#define INT_ITEST_INT(n)               (1 << ((n) >> 1))
#define INT_ITEST_INT0                 (1 << 0)  /* Bit 0: Test vector 0xffx0 */
#define INT_ITEST_INT2                 (1 << 1)  /* Bit 1: Test vector 0xffx2 */
#define INT_ITEST_INT4                 (1 << 2)  /* Bit 2: Test vector 0xffx4 */
#define INT_ITEST_INT6                 (1 << 3)  /* Bit 3: Test vector 0xffx6 */
#define INT_ITEST_INT8                 (1 << 4)  /* Bit 4: Test vector 0xffx8 */
#define INT_ITEST_INTA                 (1 << 5)  /* Bit 5: Test vector 0xffxa */
#define INT_ITEST_INTC                 (1 << 6)  /* Bit 6: Test vector 0xffxc */
#define INT_ITEST_INTE                 (1 << 7)  /* Bit 7: Test vector 0xffxe */

/* Highest Priority Interrupt Bit-Field Definitions */
/* Holds the least of the highest priority interrupt vector address */

#define INT_HPRIO_MASK                 (0xfe)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_INTV1_H */
