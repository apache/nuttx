/****************************************************************************
 * arch/arm/src/armv8-m/dwt.h
 *
 *   Copyright (c) 2009 - 2013 ARM LIMITED
 *
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS
 *  AND CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Author: Pierre-noel Bouteville <pnb990@gmail.com>
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

#ifndef __ARCH_ARM_SRC_ARMV8_M_DWT_H
#define __ARCH_ARM_SRC_ARMV8_M_DWT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Data Watchpoint and Trace Register (DWT) Definitions *********************/

/* DWT Register Base Address ************************************************/

#define DWT_BASE                     (0xe0001000ul)

/* DWT Register Addresses ***************************************************/

#define DWT_CTRL                     (DWT_BASE + 0x0000)  /* Control Register */
#define DWT_CYCCNT                   (DWT_BASE + 0x0004)  /* Cycle Count Register */
#define DWT_CPICNT                   (DWT_BASE + 0x0008)  /* CPI Count Register */
#define DWT_EXCCNT                   (DWT_BASE + 0x000c)  /* Exception Overhead Count Register */
#define DWT_SLEEPCNT                 (DWT_BASE + 0x0010)  /* Sleep Count Register */
#define DWT_LSUCNT                   (DWT_BASE + 0x0014)  /* LSU Count Register */
#define DWT_FOLDCNT                  (DWT_BASE + 0x0018)  /* Folded-instruction Count Register */
#define DWT_PCSR                     (DWT_BASE + 0x001c)  /* Program Counter Sample Register */
#define DWT_COMP0                    (DWT_BASE + 0x0020)  /* Comparator Register 0 */
#define DWT_FUNCTION0                (DWT_BASE + 0x0028)  /* Function Register 0 */
#define DWT_MASK0                    (DWT_BASE + 0x002c)  /* Mask Register 0 */
#define DWT_COMP1                    (DWT_BASE + 0x0030)  /* Comparator Register 1 */
#define DWT_FUNCTION1                (DWT_BASE + 0x0038)  /* Function Register 1 */
#define DWT_MASK1                    (DWT_BASE + 0x003c)  /* Mask Register 1 */
#define DWT_COMP2                    (DWT_BASE + 0x0040)  /* Comparator Register 2 */
#define DWT_FUNCTION2                (DWT_BASE + 0x0048)  /* Function Register 2 */
#define DWT_MASK2                    (DWT_BASE + 0x004c)  /* Mask Register 2 */
#define DWT_COMP3                    (DWT_BASE + 0x0050)  /* Comparator Register 3 */
#define DWT_FUNCTION3                (DWT_BASE + 0x0058)  /* Function Register 3 */
#define DWT_MASK3                    (DWT_BASE + 0x005c)  /* Mask Register 3 */
#define DWT_COMP4                    (DWT_BASE + 0x0060)  /* Comparator Register 4 */
#define DWT_FUNCTION4                (DWT_BASE + 0x0068)  /* Function Register 4 */
#define DWT_MASK4                    (DWT_BASE + 0x006c)  /* Mask Register 4 */
#define DWT_COMP5                    (DWT_BASE + 0x0070)  /* Comparator Register 5 */
#define DWT_FUNCTION5                (DWT_BASE + 0x0078)  /* Function Register 5 */
#define DWT_MASK5                    (DWT_BASE + 0x007c)  /* Mask Register 5 */
#define DWT_COMP6                    (DWT_BASE + 0x0080)  /* Comparator Register 6 */
#define DWT_FUNCTION6                (DWT_BASE + 0x0088)  /* Function Register 6 */
#define DWT_MASK6                    (DWT_BASE + 0x008c)  /* Mask Register 6 */
#define DWT_COMP7                    (DWT_BASE + 0x0090)  /* Comparator Register 7 */
#define DWT_FUNCTION7                (DWT_BASE + 0x0098)  /* Function Register 7 */
#define DWT_MASK7                    (DWT_BASE + 0x009c)  /* Mask Register 7 */
#define DWT_COMP8                    (DWT_BASE + 0x00a0)  /* Comparator Register 8 */
#define DWT_FUNCTION8                (DWT_BASE + 0x00a8)  /* Function Register 8 */
#define DWT_MASK8                    (DWT_BASE + 0x00ac)  /* Mask Register 8 */
#define DWT_COMP9                    (DWT_BASE + 0x00b0)  /* Comparator Register 9 */
#define DWT_FUNCTION9                (DWT_BASE + 0x00b8)  /* Function Register 9 */
#define DWT_MASK9                    (DWT_BASE + 0x00bc)  /* Mask Register 9 */
#define DWT_COMP10                   (DWT_BASE + 0x00c0)  /* Comparator Register 10 */
#define DWT_FUNCTION10               (DWT_BASE + 0x00c8)  /* Function Register 10 */
#define DWT_MASK10                   (DWT_BASE + 0x00cc)  /* Mask Register 10 */
#define DWT_COMP11                   (DWT_BASE + 0x00d0)  /* Comparator Register 11 */
#define DWT_FUNCTION11               (DWT_BASE + 0x00d8)  /* Function Register 11 */
#define DWT_MASK11                   (DWT_BASE + 0x00dc)  /* Mask Register 11 */
#define DWT_COMP12                   (DWT_BASE + 0x00e0)  /* Comparator Register 12 */
#define DWT_FUNCTION12               (DWT_BASE + 0x00e8)  /* Function Register 12 */
#define DWT_MASK12                   (DWT_BASE + 0x00ec)  /* Mask Register 12 */
#define DWT_COMP13                   (DWT_BASE + 0x00f0)  /* Comparator Register 13 */
#define DWT_FUNCTION13               (DWT_BASE + 0x00f8)  /* Function Register 13 */
#define DWT_MASK13                   (DWT_BASE + 0x00fc)  /* Mask Register 13 */
#define DWT_COMP14                   (DWT_BASE + 0x0100)  /* Comparator Register 14 */
#define DWT_FUNCTION14               (DWT_BASE + 0x0108)  /* Function Register 14 */
#define DWT_MASK14                   (DWT_BASE + 0x010c)  /* Mask Register 14 */

/* DWT Register Bit Field Definitions ***************************************/

/* DWT CTRL */

#define DWT_CTRL_NUMCOMP_SHIFT        28
#define DWT_CTRL_NUMCOMP_MASK         (0xFul << DWT_CTRL_NUMCOMP_SHIFT)
#define DWT_CTRL_NOTRCPKT_SHIFT       27
#define DWT_CTRL_NOTRCPKT_MASK        (0x1ul << DWT_CTRL_NOTRCPKT_SHIFT)
#define DWT_CTRL_NOEXTTRIG_SHIFT      26
#define DWT_CTRL_NOEXTTRIG_MASK       (0x1ul << DWT_CTRL_NOEXTTRIG_SHIFT)
#define DWT_CTRL_NOCYCCNT_SHIFT       25
#define DWT_CTRL_NOCYCCNT_MASK        (0x1ul << DWT_CTRL_NOCYCCNT_SHIFT)
#define DWT_CTRL_NOPRFCNT_SHIFT       24
#define DWT_CTRL_NOPRFCNT_MASK        (0x1ul << DWT_CTRL_NOPRFCNT_SHIFT)
#define DWT_CTRL_CYCDISS_SHIFT        23
#define DWT_CTRL_CYCDISS_MASK         (0x1ul << DWT_CTRL_CYCDISS_SHIFT)
#define DWT_CTRL_CYCEVTENA_SHIFT      22
#define DWT_CTRL_CYCEVTENA_MASK       (0x1ul << DWT_CTRL_CYCEVTENA_SHIFT)
#define DWT_CTRL_FOLDEVTENA_SHIFT     21
#define DWT_CTRL_FOLDEVTENA_MASK      (0x1ul << DWT_CTRL_FOLDEVTENA_SHIFT)
#define DWT_CTRL_LSUEVTENA_SHIFT      20
#define DWT_CTRL_LSUEVTENA_MASK       (0x1ul << DWT_CTRL_LSUEVTENA_SHIFT)
#define DWT_CTRL_SLEEPEVTENA_SHIFT    19
#define DWT_CTRL_SLEEPEVTENA_MASK     (0x1ul << DWT_CTRL_SLEEPEVTENA_SHIFT)
#define DWT_CTRL_EXCEVTENA_SHIFT      18
#define DWT_CTRL_EXCEVTENA_MASK       (0x1ul << DWT_CTRL_EXCEVTENA_SHIFT)
#define DWT_CTRL_CPIEVTENA_SHIFT      17
#define DWT_CTRL_CPIEVTENA_MASK       (0x1ul << DWT_CTRL_CPIEVTENA_SHIFT)
#define DWT_CTRL_EXCTRCENA_SHIFT      16
#define DWT_CTRL_EXCTRCENA_MASK       (0x1ul << DWT_CTRL_EXCTRCENA_SHIFT)
#define DWT_CTRL_PCSAMPLENA_SHIFT     12
#define DWT_CTRL_PCSAMPLENA_MASK      (0x1ul << DWT_CTRL_PCSAMPLENA_SHIFT)
#define DWT_CTRL_SYNCTAP_SHIFT        10
#define DWT_CTRL_SYNCTAP_MASK         (0x3ul << DWT_CTRL_SYNCTAP_SHIFT)
#define DWT_CTRL_CYCTAP_SHIFT         9
#define DWT_CTRL_CYCTAP_MASK          (0x1ul << DWT_CTRL_CYCTAP_SHIFT)
#define DWT_CTRL_POSTINIT_SHIFT       5
#define DWT_CTRL_POSTINIT_MASK        (0xful << DWT_CTRL_POSTINIT_SHIFT)
#define DWT_CTRL_POSTPRESET_SHIFT     1
#define DWT_CTRL_POSTPRESET_MASK      (0xful << DWT_CTRL_POSTPRESET_SHIFT)
#define DWT_CTRL_CYCCNTENA_SHIFT      0
#define DWT_CTRL_CYCCNTENA_MASK       (0x1ul << DWT_CTRL_CYCCNTENA_SHIFT)

/* DWT CPICNT */

#define DWT_CPICNT_CPICNT_SHIFT       0
#define DWT_CPICNT_CPICNT_MASK        (0xfful << DWT_CPICNT_CPICNT_SHIFT)

/* DWT EXCCNT */

#define DWT_EXCCNT_EXCCNT_SHIFT       0
#define DWT_EXCCNT_EXCCNT_MASK        (0xfful << DWT_EXCCNT_EXCCNT_SHIFT)

/* DWT SLEEPCNT */

#define DWT_SLEEPCNT_SLEEPCNT_SHIFT   0
#define DWT_SLEEPCNT_SLEEPCNT_MASK    (0xfful << DWT_SLEEPCNT_SLEEPCNT_SHIFT)

/* DWT LSUCNT */

#define DWT_LSUCNT_LSUCNT_SHIFT       0
#define DWT_LSUCNT_LSUCNT_MASK        (0xfful << DWT_LSUCNT_LSUCNT_SHIFT)

/* DWT FOLDCNT */

#define DWT_FOLDCNT_FOLDCNT_SHIFT     0
#define DWT_FOLDCNT_FOLDCNT_MASK      (0xfful << DWT_FOLDCNT_FOLDCNT_SHIFT)

/* DWT MASK */

#define DWT_MASK_MASK_SHIFT           0
#define DWT_MASK_MASK_MASK            (0x1ful << DWT_MASK_MASK_SHIFT)

/* DWT FUNCTION */

#define DWT_FUNCTION_MATCHED_SHIFT    24
#define DWT_FUNCTION_MATCHED_MASK     (0x1ul << DWT_FUNCTION_MATCHED_SHIFT)
#define DWT_FUNCTION_DATAVADDR1_SHIFT 16
#define DWT_FUNCTION_DATAVADDR1_MASK  (0xful << DWT_FUNCTION_DATAVADDR1_SHIFT)
#define DWT_FUNCTION_DATAVADDR0_SHIFT 12
#define DWT_FUNCTION_DATAVADDR0_MASK  (0xful << DWT_FUNCTION_DATAVADDR0_SHIFT)
#define DWT_FUNCTION_DATAVSIZE_SHIFT  10
#define DWT_FUNCTION_DATAVSIZE_MASK   (0x3ul << DWT_FUNCTION_DATAVSIZE_SHIFT)
#define DWT_FUNCTION_LNK1ENA_SHIFT    9
#define DWT_FUNCTION_LNK1ENA_MASK     (0x1ul << DWT_FUNCTION_LNK1ENA_SHIFT)
#define DWT_FUNCTION_DATAVMATCH_SHIFT 8
#define DWT_FUNCTION_DATAVMATCH_MASK  (0x1ul << DWT_FUNCTION_DATAVMATCH_SHIFT)
#define DWT_FUNCTION_CYCMATCH_SHIFT   7
#define DWT_FUNCTION_CYCMATCH_MASK    0x1ul << DWT_FUNCTION_CYCMATCH_SHIFT)
#define DWT_FUNCTION_EMITRANGE_SHIFT  5
#define DWT_FUNCTION_EMITRANGE_MASK   (0x1ul << DWT_FUNCTION_EMITRANGE_SHIFT)
#define DWT_FUNCTION_FUNCTION_SHIFT   0
#define DWT_FUNCTION_FUNCTION_MASK    (0xful << DWT_FUNCTION_FUNCTION_SHIFT)

#endif /* __ARCH_ARM_SRC_ARMV8_M_DWT_H */
