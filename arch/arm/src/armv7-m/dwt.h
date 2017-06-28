/***********************************************************************************************
 * arch/arm/src/armv7-m/dwt.h
 *
 *   Copyright (c) 2009 - 2013 ARM LIMITED
 *
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *  *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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
 ***********************************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV7_M_DWT_H
#define __ARCH_ARM_SRC_ARMV7_M_DWT_H

/***********************************************************************************************
 * Pre-processor Definitions
 ***********************************************************************************************/

/* Data Watchpoint and Trace Register (DWT) Definitions ****************************************/
/* DWT Register Base Address *******************************************************************/

#define DWT_BASE                    (0xe0001000ul)

/* DWT Register Addresses **********************************************************************/

#define DWT_CTRL                    (DWT_BASE+0x0000)  /* Control Register */
#define DWT_CYCCNT                  (DWT_BASE+0x0004)  /* Cycle Count Register */
#define DWT_CPICNT                  (DWT_BASE+0x0008)  /* CPI Count Register */
#define DWT_EXCCNT                  (DWT_BASE+0x000c)  /* Exception Overhead Count Register */
#define DWT_SLEEPCNT                (DWT_BASE+0x0010)  /* Sleep Count Register */
#define DWT_LSUCNT                  (DWT_BASE+0x0014)  /* LSU Count Register */
#define DWT_FOLDCNT                 (DWT_BASE+0x0018)  /* Folded-instruction Count Register */
#define DWT_PCSR                    (DWT_BASE+0x001c)  /* Program Counter Sample Register */
#define DWT_COMP0                   (DWT_BASE+0x0020)  /* Comparator Register 0 */
#define DWT_MASK0                   (DWT_BASE+0x0024)  /* Mask Register 0 */
#define DWT_FUNCTION0               (DWT_BASE+0x0028)  /* Function Register 0 */
#define DWT_COMP1                   (DWT_BASE+0x0030)  /* Comparator Register 1 */
#define DWT_MASK1                   (DWT_BASE+0x0034)  /* Mask Register 1 */
#define DWT_FUNCTION1               (DWT_BASE+0x0038)  /* Function Register 1 */
#define DWT_COMP2                   (DWT_BASE+0x0040)  /* Comparator Register 2 */
#define DWT_MASK2                   (DWT_BASE+0x0044)  /* Mask Register 2 */
#define DWT_FUNCTION2               (DWT_BASE+0x0048)  /* Function Register 2 */
#define DWT_COMP3                   (DWT_BASE+0x0050)  /* Comparator Register 3 */
#define DWT_MASK3                   (DWT_BASE+0x0054)  /* Mask Register 3 */
#define DWT_FUNCTION3               (DWT_BASE+0x0058)  /* Function Register 3 */

/* DWT Register Bit Field Definitions **********************************************************/

#define DWT_CTRL_NUMCOMP_Pos        28                                     /* DWT CTRL: NUMCOMP Position */
#define DWT_CTRL_NUMCOMP_Msk        (0xFul << DWT_CTRL_NUMCOMP_Pos)        /* DWT CTRL: NUMCOMP Mask */
#define DWT_CTRL_NOTRCPKT_Pos       27                                     /* DWT CTRL: NOTRCPKT Position */
#define DWT_CTRL_NOTRCPKT_Msk       (0x1ul << DWT_CTRL_NOTRCPKT_Pos)       /* DWT CTRL: NOTRCPKT Mask */
#define DWT_CTRL_NOEXTTRIG_Pos      26                                     /* DWT CTRL: NOEXTTRIG Position */
#define DWT_CTRL_NOEXTTRIG_Msk      (0x1ul << DWT_CTRL_NOEXTTRIG_Pos)      /* DWT CTRL: NOEXTTRIG Mask */
#define DWT_CTRL_NOCYCCNT_Pos       25                                     /* DWT CTRL: NOCYCCNT Position */
#define DWT_CTRL_NOCYCCNT_Msk       (0x1ul << DWT_CTRL_NOCYCCNT_Pos)       /* DWT CTRL: NOCYCCNT Mask */
#define DWT_CTRL_NOPRFCNT_Pos       24                                     /* DWT CTRL: NOPRFCNT Position */
#define DWT_CTRL_NOPRFCNT_Msk       (0x1ul << DWT_CTRL_NOPRFCNT_Pos)       /* DWT CTRL: NOPRFCNT Mask */
#define DWT_CTRL_CYCEVTENA_Pos      22                                     /* DWT CTRL: CYCEVTENA Position */
#define DWT_CTRL_CYCEVTENA_Msk      (0x1ul << DWT_CTRL_CYCEVTENA_Pos)      /* DWT CTRL: CYCEVTENA Mask */
#define DWT_CTRL_FOLDEVTENA_Pos     21                                     /* DWT CTRL: FOLDEVTENA Position */
#define DWT_CTRL_FOLDEVTENA_Msk     (0x1ul << DWT_CTRL_FOLDEVTENA_Pos)     /* DWT CTRL: FOLDEVTENA Mask */
#define DWT_CTRL_LSUEVTENA_Pos      20                                     /* DWT CTRL: LSUEVTENA Position */
#define DWT_CTRL_LSUEVTENA_Msk      (0x1ul << DWT_CTRL_LSUEVTENA_Pos)      /* DWT CTRL: LSUEVTENA Mask */
#define DWT_CTRL_SLEEPEVTENA_Pos    19                                     /* DWT CTRL: SLEEPEVTENA Position */
#define DWT_CTRL_SLEEPEVTENA_Msk    (0x1ul << DWT_CTRL_SLEEPEVTENA_Pos)    /* DWT CTRL: SLEEPEVTENA Mask */
#define DWT_CTRL_EXCEVTENA_Pos      18                                     /* DWT CTRL: EXCEVTENA Position */
#define DWT_CTRL_EXCEVTENA_Msk      (0x1ul << DWT_CTRL_EXCEVTENA_Pos)      /* DWT CTRL: EXCEVTENA Mask */
#define DWT_CTRL_CPIEVTENA_Pos      17                                     /* DWT CTRL: CPIEVTENA Position */
#define DWT_CTRL_CPIEVTENA_Msk      (0x1ul << DWT_CTRL_CPIEVTENA_Pos)      /* DWT CTRL: CPIEVTENA Mask */
#define DWT_CTRL_EXCTRCENA_Pos      16                                     /* DWT CTRL: EXCTRCENA Position */
#define DWT_CTRL_EXCTRCENA_Msk      (0x1ul << DWT_CTRL_EXCTRCENA_Pos)      /* DWT CTRL: EXCTRCENA Mask */
#define DWT_CTRL_PCSAMPLENA_Pos     12                                     /* DWT CTRL: PCSAMPLENA Position */
#define DWT_CTRL_PCSAMPLENA_Msk     (0x1ul << DWT_CTRL_PCSAMPLENA_Pos)     /* DWT CTRL: PCSAMPLENA Mask */
#define DWT_CTRL_SYNCTAP_Pos        10                                     /* DWT CTRL: SYNCTAP Position */
#define DWT_CTRL_SYNCTAP_Msk        (0x3ul << DWT_CTRL_SYNCTAP_Pos)        /* DWT CTRL: SYNCTAP Mask */
#define DWT_CTRL_CYCTAP_Pos         9                                      /* DWT CTRL: CYCTAP Position */
#define DWT_CTRL_CYCTAP_Msk         (0x1ul << DWT_CTRL_CYCTAP_Pos)         /* DWT CTRL: CYCTAP Mask */
#define DWT_CTRL_POSTINIT_Pos       5                                      /* DWT CTRL: POSTINIT Position */
#define DWT_CTRL_POSTINIT_Msk       (0xful << DWT_CTRL_POSTINIT_Pos)       /* DWT CTRL: POSTINIT Mask */
#define DWT_CTRL_POSTPRESET_Pos     1                                      /* DWT CTRL: POSTPRESET Position */
#define DWT_CTRL_POSTPRESET_Msk     (0xful << DWT_CTRL_POSTPRESET_Pos)     /* DWT CTRL: POSTPRESET Mask */
#define DWT_CTRL_CYCCNTENA_Pos      0                                      /* DWT CTRL: CYCCNTENA Position */
#define DWT_CTRL_CYCCNTENA_Msk      (0x1ul << DWT_CTRL_CYCCNTENA_Pos)      /* DWT CTRL: CYCCNTENA Mask */

#define DWT_CPICNT_CPICNT_Pos       0                                      /* DWT CPICNT: CPICNT Position */
#define DWT_CPICNT_CPICNT_Msk       (0xfful << DWT_CPICNT_CPICNT_Pos)      /* DWT CPICNT: CPICNT Mask */

#define DWT_EXCCNT_EXCCNT_Pos       0                                      /* DWT EXCCNT: EXCCNT Position */
#define DWT_EXCCNT_EXCCNT_Msk       (0xfful << DWT_EXCCNT_EXCCNT_Pos)      /* DWT EXCCNT: EXCCNT Mask */

#define DWT_SLEEPCNT_SLEEPCNT_Pos   0                                      /* DWT SLEEPCNT: SLEEPCNT Position */
#define DWT_SLEEPCNT_SLEEPCNT_Msk   (0xfful << DWT_SLEEPCNT_SLEEPCNT_Pos)  /* DWT SLEEPCNT: SLEEPCNT Mask */

#define DWT_LSUCNT_LSUCNT_Pos       0                                      /* DWT LSUCNT: LSUCNT Position */
#define DWT_LSUCNT_LSUCNT_Msk       (0xfful << DWT_LSUCNT_LSUCNT_Pos)      /* DWT LSUCNT: LSUCNT Mask */

#define DWT_FOLDCNT_FOLDCNT_Pos     0                                      /* DWT FOLDCNT: FOLDCNT Position */
#define DWT_FOLDCNT_FOLDCNT_Msk     (0xfful << DWT_FOLDCNT_FOLDCNT_Pos)    /* DWT FOLDCNT: FOLDCNT Mask */

#define DWT_MASK_MASK_Pos           0                                      /* DWT MASK: MASK Position */
#define DWT_MASK_MASK_Msk           (0x1ful << DWT_MASK_MASK_Pos)          /* DWT MASK: MASK Mask */

#define DWT_FUNCTION_MATCHED_Pos    24                                     /* DWT FUNCTION: MATCHED Position */
#define DWT_FUNCTION_MATCHED_Msk    (0x1ul << DWT_FUNCTION_MATCHED_Pos)    /* DWT FUNCTION: MATCHED Mask */
#define DWT_FUNCTION_DATAVADDR1_Pos 16                                     /* DWT FUNCTION: DATAVADDR1 Position */
#define DWT_FUNCTION_DATAVADDR1_Msk (0xful << DWT_FUNCTION_DATAVADDR1_Pos) /* DWT FUNCTION: DATAVADDR1 Mask */
#define DWT_FUNCTION_DATAVADDR0_Pos 12                                     /* DWT FUNCTION: DATAVADDR0 Position */
#define DWT_FUNCTION_DATAVADDR0_Msk (0xful << DWT_FUNCTION_DATAVADDR0_Pos) /* DWT FUNCTION: DATAVADDR0 Mask */
#define DWT_FUNCTION_DATAVSIZE_Pos  10                                     /* DWT FUNCTION: DATAVSIZE Position */
#define DWT_FUNCTION_DATAVSIZE_Msk  (0x3ul << DWT_FUNCTION_DATAVSIZE_Pos)  /* DWT FUNCTION: DATAVSIZE Mask */
#define DWT_FUNCTION_LNK1ENA_Pos    9                                      /* DWT FUNCTION: LNK1ENA Position */
#define DWT_FUNCTION_LNK1ENA_Msk    (0x1ul << DWT_FUNCTION_LNK1ENA_Pos)    /* DWT FUNCTION: LNK1ENA Mask */
#define DWT_FUNCTION_DATAVMATCH_Pos 8                                      /* DWT FUNCTION: DATAVMATCH Position */
#define DWT_FUNCTION_DATAVMATCH_Msk (0x1ul << DWT_FUNCTION_DATAVMATCH_Pos) /* DWT FUNCTION: DATAVMATCH Mask */
#define DWT_FUNCTION_CYCMATCH_Pos   7                                      /* DWT FUNCTION: CYCMATCH Position */
#define DWT_FUNCTION_CYCMATCH_Msk   0x1ul << DWT_FUNCTION_CYCMATCH_Pos)    /* DWT FUNCTION: CYCMATCH Mask */
#define DWT_FUNCTION_EMITRANGE_Pos  5                                      /* DWT FUNCTION: EMITRANGE Position */
#define DWT_FUNCTION_EMITRANGE_Msk  (0x1ul << DWT_FUNCTION_EMITRANGE_Pos)  /* DWT FUNCTION: EMITRANGE Mask */
#define DWT_FUNCTION_FUNCTION_Pos   0                                      /* DWT FUNCTION: FUNCTION Position */
#define DWT_FUNCTION_FUNCTION_Msk   (0xful << DWT_FUNCTION_FUNCTION_Pos)   /* DWT FUNCTION: FUNCTION Mask */

#endif /* __ARCH_ARM_SRC_ARMV7_M_DWT_H */
