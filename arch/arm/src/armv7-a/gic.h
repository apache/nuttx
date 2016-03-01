/****************************************************************************
 * arch/arm/src/armv7-a/gic.h
 * Generic Interrupt Controller Definitions
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_ARMV7_A_GIC_H
#define __ARCH_ARM_SRC_ARMV7_A_GIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* GIC Register Base Addresses **********************************************/

/* GIC2 CPU interface / distributer register base address */

#define GIC2_CPU_PBASE             0x1e020000
#define GIC2_DISTR_PBASE           0x1e021000

/* GIC3 CPU interface / distributer register base address */

#define GIC3_CPU_PBASE             0x1e030000
#define GIC3_DISTR_PBASE           0x1e031000

/* GIC Register Offsets *****************************************************/
/* CPU Interface registers */

#define GIC_CPUCTRL_OFFSET         0x0000    /* CPU control */
#define GIC_PRIMASK_OFFSET         0x0004    /* Priority Mask */
#define GIC_BINARYPT_OFFSET        0x0008    /* Binary point */
#define GIC_INTACK_OFFSET          0x000c    /* Interrupt Acknowledge */
#define GIC_ENDINT_OFFSET          0x0010    /* End of interrupt */
#define GIC_RUNINT_OFFSET          0x0014    /* Running interrupt */
#define GIC_HIPEND_OFFSET          0x0018    /* Highest pending interrupt */

/* Distributor Registers */

#define GIC_DISTRCTRL_OFFSET       0x0000    /* Distributor control */
#define GIC_TYPE_OFFSET            0x0004    /* Controller type */
                                             /* 0x0008-0x00fc: Reserved */
#define GIC_SETENABLE_OFFSET(n)    (0x0100 + ((n) >> 3) & ~3)
#  define GIC_SETENABLE0_OFFSET    0x0100    /* Set-enable0 */
#  define GIC_SETENABLE1_OFFSET    0x0104    /* Set-enable1 */
#  define GIC_SETENABLE2_OFFSET    0x0108    /* Set-enable2 */
                                             /* 0x010c-0x017c: Reserved */
#define GIC_CLRENABLE_OFFSET(n)    (0x0180 + ((n) >> 3) & ~3)
#  define GIC_CLRENABLE0_OFFSET    0x0180    /* Clear-enable0 */
#  define GIC_CLRENABLE1_OFFSET    0x0184    /* Clear-enable1 */
#  define GIC_CLRENABLE2_OFFSET    0x0188    /* Clear-enable2 */
                                             /* 0x018c-0x01fc: Reserved */
#define GIC_SETPEND_OFFSET(n)      (0x0200 + ((n) >> 3) & ~3)
#  define GIC_SETPEND0_OFFSET      0x0200    /* Set-pending0 */
#  define GIC_SETPEND1_OFFSET      0x0204    /* Set-pending1 */
#  define GIC_SETPEND2_OFFSET      0x0208    /* Set-pending2 */
                                             /* 0x020c-0x027c: Reserved */
#define GIC_CLRPEND_OFFSET(n)      (0x0280 + ((n) >> 3) & ~3)
#  define GIC_CLRPEND0_OFFSET      0x0280    /* Clear-pending0 */
#  define GIC_CLRPEND1_OFFSET      0x0284    /* Clear-pending1 */
#  define GIC_CLRPEND2_OFFSET      0x0288    /* Clear-pending2 */
                                             /* 0x028c-0x02fc: Reserved */
#define GIC_ACTIVE_OFFSET(n)       (0x0300 + ((n) >> 3) & ~3)
#  define GIC_ACTIVE0_OFFSET       0x0300    /* Active0 */
#  define GIC_ACTIVE1_OFFSET       0x0304    /* Active1 */
#  define GIC_ACTIVE2_OFFSET       0x0308    /* Active2 */
                                             /* 0x030c-0x01fc: Reserved */
#define GIC_PRIORITY_OFFSET(n)     (0x400 + ((n) >> ~3))
#  define GIC_PRIORITY8_OFFSET     0x0420    /* Priority8  ID32-ID35 */
#  define GIC_PRIORITY9_OFFSET     0x0424    /* Priority9  ID36-ID39 */
#  define GIC_PRIORITY10_OFFSET    0x0428    /* Priority10 ID40-ID43 */
#  define GIC_PRIORITY11_OFFSET    0x042c    /* Priority11 ID44-ID47 */
#  define GIC_PRIORITY12_OFFSET    0x0430    /* Priority12 ID48-ID51 */
#  define GIC_PRIORITY13_OFFSET    0x0434    /* Priority13 ID52-ID55 */
#  define GIC_PRIORITY14_OFFSET    0x0438    /* Priority14 ID56-ID59 */
#  define GIC_PRIORITY15_OFFSET    0x043c    /* Priority15 ID60-ID63 */
#  define GIC_PRIORITY16_OFFSET    0x0440    /* Priority16 ID64-ID67 */
#  define GIC_PRIORITY17_OFFSET    0x0444    /* Priority17 ID68-ID71 */
#  define GIC_PRIORITY18_OFFSET    0x0448    /* Priority18 ID72-ID75 */
#  define GIC_PRIORITY19_OFFSET    0x044c    /* Priority19 ID76-ID79 */
#  define GIC_PRIORITY20_OFFSET    0x0450    /* Priority20 ID80-ID83 */
#  define GIC_PRIORITY21_OFFSET    0x0454    /* Priority21 ID84-ID87 */
#  define GIC_PRIORITY22_OFFSET    0x0458    /* Priority22 ID88-ID91 */
#  define GIC_PRIORITY23_OFFSET    0x045c    /* Priority23 ID92-ID95 */
                                             /* 0x0460-0x07fc: Reserved */
#define GIC_CPUTARGET_OFFSET(n)    (0x800 + ((n) >> ~3))
#  define GIC_CPUTARGET8_OFFSET    0x0820    /* CPU target8  ID32-ID35 */
#  define GIC_CPUTARGET9_OFFSET    0x0824    /* CPU target9  ID36-ID39 */
#  define GIC_CPUTARGET10_OFFSET   0x0828    /* CPU target10 ID40-ID43 */
#  define GIC_CPUTARGET11_OFFSET   0x082c    /* CPU target11 ID44-ID47 */
#  define GIC_CPUTARGET12_OFFSET   0x0830    /* CPU target12 ID48-ID51 */
#  define GIC_CPUTARGET13_OFFSET   0x0834    /* CPU target13 ID52-ID55 */
#  define GIC_CPUTARGET14_OFFSET   0x0838    /* CPU target14 ID56-ID59 */
#  define GIC_CPUTARGET15_OFFSET   0x083c    /* CPU target15 ID60-ID63 */
#  define GIC_CPUTARGET16_OFFSET   0x0840    /* CPU target16 ID64-ID67 */
#  define GIC_CPUTARGET17_OFFSET   0x0844    /* CPU target17 ID68-ID71 */
#  define GIC_CPUTARGET18_OFFSET   0x0848    /* CPU target18 ID72-ID75 */
#  define GIC_CPUTARGET19_OFFSET   0x084c    /* CPU target19 ID76-ID79 */
#  define GIC_CPUTARGET20_OFFSET   0x0850    /* CPU target20 ID80-ID83 */
#  define GIC_CPUTARGET21_OFFSET   0x0854    /* CPU target21 ID84-ID87 */
#  define GIC_CPUTARGET22_OFFSET   0x0858    /* CPU target22 ID88-ID91 */
#  define GIC_CPUTARGET23_OFFSET   0x085c    /* CPU target23 ID92-ID95 */
                                             /* 0x0860-0x0bfc: Reserved */
#define GIC_CONFIG2_OFFSET(n)      (0x0c00 + ((n) >> 2) & ~3)
#  define GIC_CONFIG2_OFFSET       0x0c08    /* Configuration2 ID32-ID47 */
#  define GIC_CONFIG3_OFFSET       0x0c0c    /* Configuration3 ID48-ID63 */
#  define GIC_CONFIG4_OFFSET       0x0c10    /* Configuration4 ID64-ID79 */
#  define GIC_CONFIG5_OFFSET       0x0c14    /* Configuration5 ID80-ID95 */
                                             /* 0x0c18-0x0efc: Reserved */
#define GIC_SWINT_OFFSET           0x0f00    /* Software interrupt */
                                             /* 0x0f04-0x0ffc: Reserved */

/* GIC Register Addresses ***************************************************/
/* CPU Interface registers */

#define GIC2_CPUCTRL               (GIC2_CPU_PBASE+GIC_CPUCTRL_OFFSET)
#define GIC2_PRIMASK               (GIC2_CPU_PBASE+GIC_PRIMASK_OFFSET)
#define GIC2_BINARYPT              (GIC2_CPU_PBASE+GIC_BINARYPT_OFFSET)
#define GIC2_INTACK                (GIC2_CPU_PBASE+GIC_INTACK_OFFSET)
#define GIC2_ENDINT                (GIC2_CPU_PBASE+GIC_ENDINT_OFFSET)
#define GIC2_RUNINT                (GIC2_CPU_PBASE+GIC_RUNINT_OFFSET)
#define GIC2_HIPEND                (GIC2_CPU_PBASE+GIC_HIPEND_OFFSET)

/* Distributor Registers */

#define GIC2_DISTRCTRL             (GIC2_DISTR_PBASE+GIC_DISTRCTRL_OFFSET)
#define GIC2_TYPE                  (GIC2_DISTR_PBASE+GIC_TYPE_OFFSET)
#define GIC2_SETENABLE(n)          (GIC2_DISTR_PBASE+GIC_SETENABLE_OFFSET(n))
#  define GIC2_SETENABLE0          (GIC2_DISTR_PBASE+GIC_SETENABLE0_OFFSET)
#  define GIC2_SETENABLE1          (GIC2_DISTR_PBASE+GIC_SETENABLE1_OFFSET)
#  define GIC2_SETENABLE2          (GIC2_DISTR_PBASE+GIC_SETENABLE2_OFFSET)
#define GIC2_CLRENABLE(n)          (GIC2_DISTR_PBASE+GIC_CLRENABLE_OFFSET(n))
#  define GIC2_CLRENABLE0          (GIC2_DISTR_PBASE+GIC_CLRENABLE0_OFFSET)
#  define GIC2_CLRENABLE1          (GIC2_DISTR_PBASE+GIC_CLRENABLE1_OFFSET)
#  define GIC2_CLRENABLE2          (GIC2_DISTR_PBASE+GIC_CLRENABLE2_OFFSET)
#define GIC2_SETPEND(n)            (GIC2_DISTR_PBASE+GIC_SETPEND_OFFSET(n))
#  define GIC2_SETPEND0            (GIC2_DISTR_PBASE+GIC_SETPEND0_OFFSET)
#  define GIC2_SETPEND1            (GIC2_DISTR_PBASE+GIC_SETPEND1_OFFSET)
#  define GIC2_SETPEND2            (GIC2_DISTR_PBASE+GIC_SETPEND2_OFFSET)
#define GIC2_CLRPEND(n)            (GIC2_DISTR_PBASE+GIC_CLRPEND_OFFSET(n))
#  define GIC2_CLRPEND0            (GIC2_DISTR_PBASE+GIC_CLRPEND0_OFFSET)
#  define GIC2_CLRPEND1            (GIC2_DISTR_PBASE+GIC_CLRPEND1_OFFSET)
#  define GIC2_CLRPEND2            (GIC2_DISTR_PBASE+GIC_CLRPEND2_OFFSET)
#define GIC2_ACTIVE(n)             (GIC2_DISTR_PBASE+GIC_ACTIVE_OFFSET(n))
#  define GIC2_ACTIVE0             (GIC2_DISTR_PBASE+GIC_ACTIVE0_OFFSET)
#  define GIC2_ACTIVE1             (GIC2_DISTR_PBASE+GIC_ACTIVE1_OFFSET)
#  define GIC2_ACTIVE2             (GIC2_DISTR_PBASE+GIC_ACTIVE2_OFFSET)
#define GIC2_PRIORITY(n)           (GIC2_DISTR_PBASE+GIC_PRIORITY_OFFSET(n))
#  define GIC2_PRIORITY8           (GIC2_DISTR_PBASE+GIC_PRIORITY8_OFFSET)
#  define GIC2_PRIORITY9           (GIC2_DISTR_PBASE+GIC_PRIORITY9_OFFSET)
#  define GIC2_PRIORITY10          (GIC2_DISTR_PBASE+GIC_PRIORITY10_OFFSET)
#  define GIC2_PRIORITY11          (GIC2_DISTR_PBASE+GIC_PRIORITY11_OFFSET)
#  define GIC2_PRIORITY12          (GIC2_DISTR_PBASE+GIC_PRIORITY12_OFFSET)
#  define GIC2_PRIORITY13          (GIC2_DISTR_PBASE+GIC_PRIORITY13_OFFSET)
#  define GIC2_PRIORITY14          (GIC2_DISTR_PBASE+GIC_PRIORITY14_OFFSET)
#  define GIC2_PRIORITY15          (GIC2_DISTR_PBASE+GIC_PRIORITY15_OFFSET)
#  define GIC2_PRIORITY16          (GIC2_DISTR_PBASE+GIC_PRIORITY16_OFFSET)
#  define GIC2_PRIORITY17          (GIC2_DISTR_PBASE+GIC_PRIORITY17_OFFSET)
#  define GIC2_PRIORITY18          (GIC2_DISTR_PBASE+GIC_PRIORITY18_OFFSET)
#  define GIC2_PRIORITY19          (GIC2_DISTR_PBASE+GIC_PRIORITY19_OFFSET)
#  define GIC2_PRIORITY20          (GIC2_DISTR_PBASE+GIC_PRIORITY20_OFFSET)
#  define GIC2_PRIORITY21          (GIC2_DISTR_PBASE+GIC_PRIORITY21_OFFSET)
#  define GIC2_PRIORITY22          (GIC2_DISTR_PBASE+GIC_PRIORITY22_OFFSET)
#  define GIC2_PRIORITY23          (GIC2_DISTR_PBASE+GIC_PRIORITY23_OFFSET)
#define GIC2_CPUTARGET(n)          (GIC2_DISTR_PBASE+GIC_CPUTARGET_OFFSET(n))
#  define GIC2_CPUTARGET8          (GIC2_DISTR_PBASE+GIC_CPUTARGET8_OFFSET)
#  define GIC2_CPUTARGET9          (GIC2_DISTR_PBASE+GIC_CPUTARGET9_OFFSET)
#  define GIC2_CPUTARGET10         (GIC2_DISTR_PBASE+GIC_CPUTARGET10_OFFSET)
#  define GIC2_CPUTARGET11         (GIC2_DISTR_PBASE+GIC_CPUTARGET11_OFFSET)
#  define GIC2_CPUTARGET12         (GIC2_DISTR_PBASE+GIC_CPUTARGET12_OFFSET)
#  define GIC2_CPUTARGET13         (GIC2_DISTR_PBASE+GIC_CPUTARGET13_OFFSET)
#  define GIC2_CPUTARGET14         (GIC2_DISTR_PBASE+GIC_CPUTARGET14_OFFSET)
#  define GIC2_CPUTARGET15         (GIC2_DISTR_PBASE+GIC_CPUTARGET15_OFFSET)
#  define GIC2_CPUTARGET16         (GIC2_DISTR_PBASE+GIC_CPUTARGET16_OFFSET)
#  define GIC2_CPUTARGET17         (GIC2_DISTR_PBASE+GIC_CPUTARGET17_OFFSET)
#  define GIC2_CPUTARGET18         (GIC2_DISTR_PBASE+GIC_CPUTARGET18_OFFSET)
#  define GIC2_CPUTARGET19         (GIC2_DISTR_PBASE+GIC_CPUTARGET19_OFFSET)
#  define GIC2_CPUTARGET20         (GIC2_DISTR_PBASE+GIC_CPUTARGET20_OFFSET)
#  define GIC2_CPUTARGET21         (GIC2_DISTR_PBASE+GIC_CPUTARGET21_OFFSET)
#  define GIC2_CPUTARGET22         (GIC2_DISTR_PBASE+GIC_CPUTARGET22_OFFSET)
#  define GIC2_CPUTARGET23         (GIC2_DISTR_PBASE+GIC_CPUTARGET23_OFFSET)
#define GIC2_CONFIG2(n)            (GIC2_DISTR_PBASE+GIC_CONFIG2_OFFSET(n))
#  define GIC2_CONFIG2             (GIC2_DISTR_PBASE+GIC_CONFIG2_OFFSET)
#  define GIC2_CONFIG3             (GIC2_DISTR_PBASE+GIC_CONFIG3_OFFSET)
#  define GIC2_CONFIG4             (GIC2_DISTR_PBASE+GIC_CONFIG4_OFFSET)
#  define GIC2_CONFIG5             (GIC2_DISTR_PBASE+GIC_CONFIG5_OFFSET)
#define GIC2_SWINT                 (GIC2_DISTR_PBASE+GIC_SWINT_OFFSET)

/* CPU Interface registers */

#define GIC3_CPUCTRL               (GIC3_CPU_PBASE+GIC_CPUCTRL_OFFSET)
#define GIC3_PRIMASK               (GIC3_CPU_PBASE+GIC_PRIMASK_OFFSET)
#define GIC3_BINARYPT              (GIC3_CPU_PBASE+GIC_BINARYPT_OFFSET)
#define GIC3_INTACK                (GIC3_CPU_PBASE+GIC_INTACK_OFFSET)
#define GIC3_ENDINT                (GIC3_CPU_PBASE+GIC_ENDINT_OFFSET)
#define GIC3_RUNINT                (GIC3_CPU_PBASE+GIC_RUNINT_OFFSET)
#define GIC3_HIPEND                (GIC3_CPU_PBASE+GIC_HIPEND_OFFSET)

/* Distributor Registers */

#define GIC3_DISTRCTRL             (GIC3_DISTR_PBASE+GIC_DISTRCTRL_OFFSET)
#define GIC3_TYPE                  (GIC3_DISTR_PBASE+GIC_TYPE_OFFSET)
#define GIC3_SETENABLE(n)          (GIC3_DISTR_PBASE+GIC_SETENABLE_OFFSET(n))
#  define GIC3_SETENABLE0          (GIC3_DISTR_PBASE+GIC_SETENABLE0_OFFSET)
#  define GIC3_SETENABLE1          (GIC3_DISTR_PBASE+GIC_SETENABLE1_OFFSET)
#  define GIC3_SETENABLE2          (GIC3_DISTR_PBASE+GIC_SETENABLE2_OFFSET)
#define GIC3_CLRENABLE(n)          (GIC3_DISTR_PBASE+GIC_CLRENABLE_OFFSET(n))
#  define GIC3_CLRENABLE0          (GIC3_DISTR_PBASE+GIC_CLRENABLE0_OFFSET)
#  define GIC3_CLRENABLE1          (GIC3_DISTR_PBASE+GIC_CLRENABLE1_OFFSET)
#  define GIC3_CLRENABLE2          (GIC3_DISTR_PBASE+GIC_CLRENABLE2_OFFSET)
#define GIC3_SETPEND(n)            (GIC3_DISTR_PBASE+GIC_SETPEND_OFFSET(n))
#  define GIC3_SETPEND0            (GIC3_DISTR_PBASE+GIC_SETPEND0_OFFSET)
#  define GIC3_SETPEND1            (GIC3_DISTR_PBASE+GIC_SETPEND1_OFFSET)
#  define GIC3_SETPEND2            (GIC3_DISTR_PBASE+GIC_SETPEND2_OFFSET)
#define GIC3_CLRPEND(n)            (GIC3_DISTR_PBASE+GIC_CLRPEND_OFFSET(n))
#  define GIC3_CLRPEND0            (GIC3_DISTR_PBASE+GIC_CLRPEND0_OFFSET)
#  define GIC3_CLRPEND1            (GIC3_DISTR_PBASE+GIC_CLRPEND1_OFFSET)
#  define GIC3_CLRPEND2            (GIC3_DISTR_PBASE+GIC_CLRPEND2_OFFSET)
#define GIC3_ACTIVE(n)             (GIC3_DISTR_PBASE+GIC_ACTIVE_OFFSET(n))
#  define GIC3_ACTIVE0             (GIC3_DISTR_PBASE+GIC_ACTIVE0_OFFSET)
#  define GIC3_ACTIVE1             (GIC3_DISTR_PBASE+GIC_ACTIVE1_OFFSET)
#  define GIC3_ACTIVE2             (GIC3_DISTR_PBASE+GIC_ACTIVE2_OFFSET)
#define GIC3_PRIORITY(n)           (GIC3_DISTR_PBASE+GIC_PRIORITY_OFFSET(n))
#  define GIC3_PRIORITY8           (GIC3_DISTR_PBASE+GIC_PRIORITY8_OFFSET)
#  define GIC3_PRIORITY9           (GIC3_DISTR_PBASE+GIC_PRIORITY9_OFFSET)
#  define GIC3_PRIORITY10          (GIC3_DISTR_PBASE+GIC_PRIORITY10_OFFSET)
#  define GIC3_PRIORITY11          (GIC3_DISTR_PBASE+GIC_PRIORITY11_OFFSET)
#  define GIC3_PRIORITY12          (GIC3_DISTR_PBASE+GIC_PRIORITY12_OFFSET)
#  define GIC3_PRIORITY13          (GIC3_DISTR_PBASE+GIC_PRIORITY13_OFFSET)
#  define GIC3_PRIORITY14          (GIC3_DISTR_PBASE+GIC_PRIORITY14_OFFSET)
#  define GIC3_PRIORITY15          (GIC3_DISTR_PBASE+GIC_PRIORITY15_OFFSET)
#  define GIC3_PRIORITY16          (GIC3_DISTR_PBASE+GIC_PRIORITY16_OFFSET)
#  define GIC3_PRIORITY17          (GIC3_DISTR_PBASE+GIC_PRIORITY17_OFFSET)
#  define GIC3_PRIORITY18          (GIC3_DISTR_PBASE+GIC_PRIORITY18_OFFSET)
#  define GIC3_PRIORITY19          (GIC3_DISTR_PBASE+GIC_PRIORITY19_OFFSET)
#  define GIC3_PRIORITY20          (GIC3_DISTR_PBASE+GIC_PRIORITY20_OFFSET)
#  define GIC3_PRIORITY21          (GIC3_DISTR_PBASE+GIC_PRIORITY21_OFFSET)
#  define GIC3_PRIORITY22          (GIC3_DISTR_PBASE+GIC_PRIORITY22_OFFSET)
#  define GIC3_PRIORITY23          (GIC3_DISTR_PBASE+GIC_PRIORITY23_OFFSET)
#define GIC3_CPUTARGET(n)          (GIC3_DISTR_PBASE+GIC_CPUTARGET_OFFSET(n))
#  define GIC3_CPUTARGET8          (GIC3_DISTR_PBASE+GIC_CPUTARGET8_OFFSET)
#  define GIC3_CPUTARGET9          (GIC3_DISTR_PBASE+GIC_CPUTARGET9_OFFSET)
#  define GIC3_CPUTARGET10         (GIC3_DISTR_PBASE+GIC_CPUTARGET10_OFFSET)
#  define GIC3_CPUTARGET11         (GIC3_DISTR_PBASE+GIC_CPUTARGET11_OFFSET)
#  define GIC3_CPUTARGET12         (GIC3_DISTR_PBASE+GIC_CPUTARGET12_OFFSET)
#  define GIC3_CPUTARGET13         (GIC3_DISTR_PBASE+GIC_CPUTARGET13_OFFSET)
#  define GIC3_CPUTARGET14         (GIC3_DISTR_PBASE+GIC_CPUTARGET14_OFFSET)
#  define GIC3_CPUTARGET15         (GIC3_DISTR_PBASE+GIC_CPUTARGET15_OFFSET)
#  define GIC3_CPUTARGET16         (GIC3_DISTR_PBASE+GIC_CPUTARGET16_OFFSET)
#  define GIC3_CPUTARGET17         (GIC3_DISTR_PBASE+GIC_CPUTARGET17_OFFSET)
#  define GIC3_CPUTARGET18         (GIC3_DISTR_PBASE+GIC_CPUTARGET18_OFFSET)
#  define GIC3_CPUTARGET19         (GIC3_DISTR_PBASE+GIC_CPUTARGET19_OFFSET)
#  define GIC3_CPUTARGET20         (GIC3_DISTR_PBASE+GIC_CPUTARGET20_OFFSET)
#  define GIC3_CPUTARGET21         (GIC3_DISTR_PBASE+GIC_CPUTARGET21_OFFSET)
#  define GIC3_CPUTARGET22         (GIC3_DISTR_PBASE+GIC_CPUTARGET22_OFFSET)
#  define GIC3_CPUTARGET23         (GIC3_DISTR_PBASE+GIC_CPUTARGET23_OFFSET)
#define GIC3_CONFIG2(n)            (GIC3_DISTR_PBASE+GIC_CONFIG2_OFFSET(n))
#  define GIC3_CONFIG2             (GIC3_DISTR_PBASE+GIC_CONFIG2_OFFSET)
#  define GIC3_CONFIG3             (GIC3_DISTR_PBASE+GIC_CONFIG3_OFFSET)
#  define GIC3_CONFIG4             (GIC3_DISTR_PBASE+GIC_CONFIG4_OFFSET)
#  define GIC3_CONFIG5             (GIC3_DISTR_PBASE+GIC_CONFIG5_OFFSET)
#define GIC3_SWINT                 (GIC3_DISTR_PBASE+GIC_SWINT_OFFSET)

/* GIC Register Bit Definitions *********************************************/

/* CPU Interface registers */
/* CPU Control Register */

#define GIC_CPUCTRL_ENABLE         (1 << 0)  /* Bit 0: Enable the CPU interface for this GIC */
                                             /* Bits 1-31: Reserved */
/* Priority Mask Register */
                                             /* Bits 0-3: Reserved */
#define GIC_PRIMASK_SHIFT          (4)       /* Bits 4-7: Priority mask */
#define GIC_PRIMASK_MASK           (15 << GIC_PRIMASK_SHIFT)
#  define GIC_PRIMASK_VALUE(n)     ((uint32_t)(n) << GIC_PRIMASK_SHIFT)
                                             /* Bits 8-31: Reserved */
/* Binary point Register */

#define GIC_BINARYPT_SHIFT         (0)       /* Bits 0-2: Binary point */
#define GIC_BINARYPT_MASK          (7 << GIC_BINARYPT_SHIFT)
#  define GIC_BINARYPT_ALL         (3 << GIC_BINARYPT_SHIFT) /* All priority bits are compared for pre-emption */
#  define GIC_BINARYPT_5_7         (4 << GIC_BINARYPT_SHIFT) /* Priority bits [7:5] compared for pre-emption */
#  define GIC_BINARYPT_6_7         (5 << GIC_BINARYPT_SHIFT) /* Priority bits [7:6] compared for pre-emption */
#  define GIC_BINARYPT_7           (6 << GIC_BINARYPT_SHIFT) /* Priority bit [7] compared for pre-emption */
#  define GIC_BINARYPT_NOPREMPT    (7 << GIC_BINARYPT_SHIFT) /* No pre-emption is performed */
                                             /* Bits 3-31: Reserved */
/* Interrupt Acknowledge Register */

#define GIC_INTACK_INTID_SHIFT     (0)       /* Bits 0-9: Interrupt ID */
#define GIC_INTACK_INTID_MASK      (0x3ff << GIC_INTACK_INTID_SHIFT)
#  define GIC_INTACK_INTID(n)      ((uint32_t)(n) << GIC_INTACK_INTID_SHIFT)
#define GIC_INTACK_CPUSRC_SHIFT    (10)      /* Bits 10-12: CPU source ID */
#define GIC_INTACK_CPUSRC_MASK     (7 << GIC_INTACK_CPUSRC_SHIFT)
#  define GIC_INTACK_CPUSRC(n)     ((uint32_t)(n) << GIC_INTACK_CPUSRC_SHIFT)
                                             /* Bits 13-31: Reserved */
/* End of Interrupt Register */

#define GIC_ENDINT_INTID_SHIFT     (0)       /* Bits 0-9: Interrupt ID */
#define GIC_ENDINT_INTID_MASK      (0x3ff << GIC_ENDINT_INTID_SHIFT)
#  define GIC_ENDINT_INTID(n)      ((uint32_t)(n) << GIC_ENDINT_INTID_SHIFT)
#define GIC_ENDINT_CPUSRC_SHIFT    (10)      /* Bits 10-12: CPU source ID */
#define GIC_ENDINT_CPUSRC_MASK     (7 << GIC_ENDINT_CPUSRC_SHIFT)
#  define GIC_ENDINT_CPUSRC(n)     ((uint32_t)(n) << GIC_ENDINT_CPUSRC_SHIFT)
                                             /* Bits 13-31: Reserved */
/* Running Interrupt Register */

                                             /* Bits 0-3: Reserved */
#define GIC_RUNINT_PRIO_SHIFT      (4)       /* Bits 4-7: Priority mask */
#define GIC_RUNINT_PRIO_MASK       (15 << GIC_RUNINT_PRIO_SHIFT)
#  define GIC_RUNINT_PRIO_VALUE(n) ((uint32_t)(n) << GIC_RUNINT_PRIO_SHIFT)
                                             /* Bits 8-31: Reserved */
/* Highest Pending Interrupt Register */

#define GIC_HIPEND_INTID_SHIFT     (0)       /* Bits 0-9: Interrupt ID */
#define GIC_HIPEND_INTID_MASK      (0x3ff << GIC_HIPEND_INTID_SHIFT)
#  define GIC_HIPEND_INTID(n)      ((uint32_t)(n) << GIC_HIPEND_INTID_SHIFT)
#define GIC_HIPEND_CPUSRC_SHIFT    (10)      /* Bits 10-12: CPU source ID */
#define GIC_HIPEND_CPUSRC_MASK     (7 << GIC_HIPEND_CPUSRC_SHIFT)
#  define GIC_HIPEND_CPUSRC(n)     ((uint32_t)(n) << GIC_HIPEND_CPUSRC_SHIFT)
                                             /* Bits 13-31: Reserved */

/* Distributor Registers */
/* Distributor Control Register */

#define GIC_DISTRCTRL_ENABLE       (1 << 0)  /* Bit 0: Enable the CPU interface for this GIC */
                                             /* Bits 1-31: Reserved */
/* Controller Type Register */

#define GIC_TYPE_IDLINES_SHIFT      (0)      /* Bits 0-4: ID lines number */
#define GIC_TYPE_IDLINES_MASK       (0x1f << GIC_TYPE_IDLINES_SHIFT)
#define GIC_TYPE_CPUNO_SHIFT        (5)      /* Bits 5-7: CPU number */
#define GIC_TYPE_CPUNO_MASK         (7 << GIC_TYPE_CPUNO_SHIFT)
                                             /* Bits 8-31: Reserved */
/* Set-Enable 0, 1, 2 */

#define GIC_SETENABLE0_INT(n)      (1 << (n))      /* Bit n: Interrupt n enable, n=0-31 */
#define GIC_SETENABLE1_INT(n)      (1 << ((n)-32)) /* Bit n: Interrupt n enable, n=32-63 */
#define GIC_SETENABLE3_INT(n)      (1 << ((n)-64)) /* Bit n: Interrupt n enable, n=64-95 */

/* Clear-Enable 0, 1, 2 */

#define GIC_CLRENABLE0_INT(n)      (1 << (n))      /* Bit n: Interrupt n clear, n=0-31 */
#define GIC_CLRENABLE1_INT(n)      (1 << ((n)-32)) /* Bit n: Interrupt n clear, n=32-63 */
#define GIC_CLRENABLE3_INT(n)      (1 << ((n)-64)) /* Bit n: Interrupt n clear, n=64-95 */

/* Set-Pending 0, 1, 2 */

#define GIC_SETPEND0_INT(n)      (1 << (n))      /* Bit n: Set interrupt n pending, n=0-31 */
#define GIC_SETPEND1_INT(n)      (1 << ((n)-32)) /* Bit n: Set interrupt n pending, n=32-63 */
#define GIC_SETPEND3_INT(n)      (1 << ((n)-64)) /* Bit n: Set interrupt n pending, n=64-95 */

/* Clear-Pending 0, 1, 2 */

#define GIC_CLRPEND0_INT(n)      (1 << (n))      /* Bit n: Clear interrupt n pending, n=0-31 */
#define GIC_CLRPEND1_INT(n)      (1 << ((n)-32)) /* Bit n: Clear interrupt n pending, n=32-63 */
#define GIC_CLRPEND3_INT(n)      (1 << ((n)-64)) /* Bit n: Clear interrupt n pending, n=64-95 */

/* Active 0, 1, 2 */

#define GIC_ACTIVE0_INT(n)         (1 << (n))      /* Bit n: Interrupt n active, n=0-31 */
#define GIC_ACTIVE1_INT(n)         (1 << ((n)-32)) /* Bit n: Interrupt n active, n=32-63 */
#define GIC_ACTIVE3_INT(n)         (1 << ((n)-64)) /* Bit n: Interrupt n active, n=64-95 */

/* Priority 8..23 */

#define GIC_PRIORITY_ID_SHIFT(n)   (((n) & 3) << 3)
#define GIC_PRIORITY_ID_MASK(n)    (15 << GIC_PRIORITY_ID_SHIFT(n))
#  define GIC_PRIORITY_ID(n,p)     ((uint32_t)(p) << GIC_PRIORITY_ID_SHIFT(n))

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY8_ID32_SHIFT   (0)       /* Bits 4-7: ID 32 priority */
#define GIC_PRIORITY8_ID32_MASK    (15 << GIC_PRIORITY8_ID32_SHIFT)
#  define GIC_PRIORITY8_ID32(n)    ((uint32_t)(n) << GIC_PRIORITY8_ID32_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY8_ID33_SHIFT   (12)      /* Bits 12-15: ID 33 priority */
#define GIC_PRIORITY8_ID33_MASK    (15 << GIC_PRIORITY8_ID33_SHIFT)
#  define GIC_PRIORITY8_ID33(n)    ((uint32_t)(n) << GIC_PRIORITY8_ID33_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY8_ID34_SHIFT   (20)      /* Bits 20-23: ID 34 priority */
#define GIC_PRIORITY8_ID34_MASK    (15 << GIC_PRIORITY8_ID34_SHIFT)
#  define GIC_PRIORITY8_ID34(n)    ((uint32_t)(n) << GIC_PRIORITY8_ID34_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY8_ID35_SHIFT   (28)      /* Bits 28-31: ID 35 priority */
#define GIC_PRIORITY8_ID35_MASK    (15 << GIC_PRIORITY8_ID35_SHIFT)
#  define GIC_PRIORITY8_ID35(n)    ((uint32_t)(n) << GIC_PRIORITY8_ID35_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY9_ID36_SHIFT   (0)       /* Bits 4-7: ID 36 priority */
#define GIC_PRIORITY9_ID36_MASK    (15 << GIC_PRIORITY9_ID36_SHIFT)
#  define GIC_PRIORITY9_ID36(n)    ((uint32_t)(n) << GIC_PRIORITY9_ID36_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY9_ID37_SHIFT   (12)      /* Bits 12-15: ID 37 priority */
#define GIC_PRIORITY9_ID37_MASK    (15 << GIC_PRIORITY9_ID37_SHIFT)
#  define GIC_PRIORITY9_ID37(n)    ((uint32_t)(n) << GIC_PRIORITY9_ID37_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY9_ID39_SHIFT   (20)      /* Bits 20-23: ID 38 priority */
#define GIC_PRIORITY9_ID39_MASK    (15 << GIC_PRIORITY9_ID39_SHIFT)
#  define GIC_PRIORITY9_ID38(n)    ((uint32_t)(n) << GIC_PRIORITY9_ID39_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY9_ID39_SHIFT   (28)      /* Bits 28-31: ID 39 priority */
#define GIC_PRIORITY9_ID39_MASK    (15 << GIC_PRIORITY9_ID39_SHIFT)
#  define GIC_PRIORITY9_ID39(n)    ((uint32_t)(n) << GIC_PRIORITY9_ID39_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY10_ID40_SHIFT  (0)       /* Bits 4-7: ID 40 priority */
#define GIC_PRIORITY10_ID40_MASK   (15 << GIC_PRIORITY10_ID40_SHIFT)
#  define GIC_PRIORITY10_ID40(n)   ((uint32_t)(n) << GIC_PRIORITY10_ID40_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY10_ID41_SHIFT  (12)      /* Bits 12-15: ID 41 priority */
#define GIC_PRIORITY10_ID41_MASK   (15 << GIC_PRIORITY10_ID41_SHIFT)
#  define GIC_PRIORITY10_ID41(n)   ((uint32_t)(n) << GIC_PRIORITY10_ID41_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY10_ID42_SHIFT  (20)      /* Bits 20-23: ID 42 priority */
#define GIC_PRIORITY10_ID42_MASK   (15 << GIC_PRIORITY10_ID42_SHIFT)
#  define GIC_PRIORITY10_ID42(n)   ((uint32_t)(n) << GIC_PRIORITY10_ID42_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY10_ID43_SHIFT  (28)      /* Bits 28-31: ID 43 priority */
#define GIC_PRIORITY10_ID43_MASK   (15 << GIC_PRIORITY10_ID43_SHIFT)
#  define GIC_PRIORITY10_ID43(n)   ((uint32_t)(n) << GIC_PRIORITY10_ID43_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY11_ID44_SHIFT  (0)       /* Bits 4-7: ID 44 priority */
#define GIC_PRIORITY11_ID44_MASK   (15 << GIC_PRIORITY11_ID44_SHIFT)
#  define GIC_PRIORITY11_ID44(n)   ((uint32_t)(n) << GIC_PRIORITY11_ID44_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY11_ID45_SHIFT  (12)      /* Bits 12-15: ID 45 priority */
#define GIC_PRIORITY11_ID45_MASK   (15 << GIC_PRIORITY11_ID45_SHIFT)
#  define GIC_PRIORITY11_ID45(n)   ((uint32_t)(n) << GIC_PRIORITY11_ID45_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY11_ID46_SHIFT  (20)      /* Bits 20-23: ID 46 priority */
#define GIC_PRIORITY11_ID46_MASK   (15 << GIC_PRIORITY11_ID46_SHIFT)
#  define GIC_PRIORITY11_ID46(n)   ((uint32_t)(n) << GIC_PRIORITY11_ID46_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY11_ID47_SHIFT  (28)      /* Bits 28-31: ID 47 priority */
#define GIC_PRIORITY11_ID47_MASK   (15 << GIC_PRIORITY11_ID47_SHIFT)
#  define GIC_PRIORITY11_ID47(n)   ((uint32_t)(n) << GIC_PRIORITY11_ID47_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY12_ID412_SHIFT (0)       /* Bits 4-7: ID 48 priority */
#define GIC_PRIORITY12_ID412_MASK  (15 << GIC_PRIORITY12_ID412_SHIFT)
#  define GIC_PRIORITY12_ID48(n)   ((uint32_t)(n) << GIC_PRIORITY12_ID412_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY12_ID49_SHIFT  (12)      /* Bits 12-15: ID 49 priority */
#define GIC_PRIORITY12_ID49_MASK   (15 << GIC_PRIORITY12_ID49_SHIFT)
#  define GIC_PRIORITY12_ID49(n)   ((uint32_t)(n) << GIC_PRIORITY12_ID49_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY12_ID50_SHIFT  (20)      /* Bits 20-23: ID 50 priority */
#define GIC_PRIORITY12_ID50_MASK   (15 << GIC_PRIORITY12_ID50_SHIFT)
#  define GIC_PRIORITY12_ID50(n)   ((uint32_t)(n) << GIC_PRIORITY12_ID50_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY12_ID51_SHIFT  (28)      /* Bits 28-31: ID 51 priority */
#define GIC_PRIORITY12_ID51_MASK   (15 << GIC_PRIORITY12_ID51_SHIFT)
#  define GIC_PRIORITY12_ID51(n)   ((uint32_t)(n) << GIC_PRIORITY12_ID51_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY13_ID52_SHIFT  (0)       /* Bits 4-7: ID 52 priority */
#define GIC_PRIORITY13_ID52_MASK   (15 << GIC_PRIORITY13_ID52_SHIFT)
#  define GIC_PRIORITY13_ID52(n)   ((uint32_t)(n) << GIC_PRIORITY13_ID52_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY13_ID53_SHIFT  (12)      /* Bits 12-15: ID 53 priority */
#define GIC_PRIORITY13_ID53_MASK   (15 << GIC_PRIORITY13_ID53_SHIFT)
#  define GIC_PRIORITY13_ID53(n)   ((uint32_t)(n) << GIC_PRIORITY13_ID53_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY13_ID54_SHIFT  (20)      /* Bits 20-23: ID 54 priority */
#define GIC_PRIORITY13_ID54_MASK   (15 << GIC_PRIORITY13_ID54_SHIFT)
#  define GIC_PRIORITY13_ID54(n)   ((uint32_t)(n) << GIC_PRIORITY13_ID54_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY13_ID55_SHIFT  (28)      /* Bits 28-31: ID 55 priority */
#define GIC_PRIORITY13_ID55_MASK   (15 << GIC_PRIORITY13_ID55_SHIFT)
#  define GIC_PRIORITY13_ID55(n)   ((uint32_t)(n) << GIC_PRIORITY13_ID55_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY14_ID56_SHIFT  (0)       /* Bits 4-7: ID 56 priority */
#define GIC_PRIORITY14_ID56_MASK   (15 << GIC_PRIORITY14_ID56_SHIFT)
#  define GIC_PRIORITY14_ID56(n)   ((uint32_t)(n) << GIC_PRIORITY14_ID56_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY14_ID57_SHIFT  (12)      /* Bits 12-15: ID 57 priority */
#define GIC_PRIORITY14_ID57_MASK   (15 << GIC_PRIORITY14_ID57_SHIFT)
#  define GIC_PRIORITY14_ID57(n)   ((uint32_t)(n) << GIC_PRIORITY14_ID57_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY14_ID514_SHIFT (20)      /* Bits 20-23: ID 58 priority */
#define GIC_PRIORITY14_ID514_MASK  (15 << GIC_PRIORITY14_ID514_SHIFT)
#  define GIC_PRIORITY14_ID58(n)   ((uint32_t)(n) << GIC_PRIORITY14_ID514_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY14_ID59_SHIFT  (28)      /* Bits 28-31: ID 59 priority */
#define GIC_PRIORITY14_ID59_MASK   (15 << GIC_PRIORITY14_ID59_SHIFT)
#  define GIC_PRIORITY14_ID59(n)   ((uint32_t)(n) << GIC_PRIORITY14_ID59_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY15_ID60_SHIFT  (0)       /* Bits 4-7: ID 60 priority */
#define GIC_PRIORITY15_ID60_MASK   (15 << GIC_PRIORITY15_ID60_SHIFT)
#  define GIC_PRIORITY15_ID60(n)   ((uint32_t)(n) << GIC_PRIORITY15_ID60_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY15_ID61_SHIFT  (12)      /* Bits 12-15: ID 61 priority */
#define GIC_PRIORITY15_ID61_MASK   (15 << GIC_PRIORITY15_ID61_SHIFT)
#  define GIC_PRIORITY15_ID61(n)   ((uint32_t)(n) << GIC_PRIORITY15_ID61_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY15_ID62_SHIFT  (20)      /* Bits 20-23: ID 62 priority */
#define GIC_PRIORITY15_ID62_MASK   (15 << GIC_PRIORITY15_ID62_SHIFT)
#  define GIC_PRIORITY15_ID62(n)   ((uint32_t)(n) << GIC_PRIORITY15_ID62_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY15_ID63_SHIFT  (28)      /* Bits 28-31: ID 63 priority */
#define GIC_PRIORITY15_ID63_MASK   (15 << GIC_PRIORITY15_ID63_SHIFT)
#  define GIC_PRIORITY15_ID63(n)   ((uint32_t)(n) << GIC_PRIORITY15_ID63_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY16_ID64_SHIFT  (0)       /* Bits 4-7: ID 64 priority */
#define GIC_PRIORITY16_ID64_MASK   (15 << GIC_PRIORITY16_ID64_SHIFT)
#  define GIC_PRIORITY16_ID64(n)   ((uint32_t)(n) << GIC_PRIORITY16_ID64_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY16_ID65_SHIFT  (12)      /* Bits 12-15: ID 65 priority */
#define GIC_PRIORITY16_ID65_MASK   (15 << GIC_PRIORITY16_ID65_SHIFT)
#  define GIC_PRIORITY16_ID65(n)   ((uint32_t)(n) << GIC_PRIORITY16_ID65_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY16_ID66_SHIFT  (20)      /* Bits 20-23: ID 66 priority */
#define GIC_PRIORITY16_ID66_MASK   (15 << GIC_PRIORITY16_ID66_SHIFT)
#  define GIC_PRIORITY16_ID66(n)   ((uint32_t)(n) << GIC_PRIORITY16_ID66_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY16_ID67_SHIFT  (28)      /* Bits 28-31: ID 67 priority */
#define GIC_PRIORITY16_ID67_MASK   (15 << GIC_PRIORITY16_ID67_SHIFT)
#  define GIC_PRIORITY16_ID67(n)   ((uint32_t)(n) << GIC_PRIORITY16_ID67_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY17_ID617_SHIFT (0)       /* Bits 4-7: ID 68 priority */
#define GIC_PRIORITY17_ID617_MASK  (15 << GIC_PRIORITY17_ID617_SHIFT)
#  define GIC_PRIORITY17_ID68(n)   ((uint32_t)(n) << GIC_PRIORITY17_ID617_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY17_ID69_SHIFT  (12)      /* Bits 12-15: ID 69 priority */
#define GIC_PRIORITY17_ID69_MASK   (15 << GIC_PRIORITY17_ID69_SHIFT)
#  define GIC_PRIORITY17_ID69(n)   ((uint32_t)(n) << GIC_PRIORITY17_ID69_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY17_ID70_SHIFT  (20)      /* Bits 20-23: ID 70 priority */
#define GIC_PRIORITY17_ID70_MASK   (15 << GIC_PRIORITY17_ID70_SHIFT)
#  define GIC_PRIORITY17_ID70(n)   ((uint32_t)(n) << GIC_PRIORITY17_ID70_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY17_ID71_SHIFT  (28)      /* Bits 28-31: ID 71 priority */
#define GIC_PRIORITY17_ID71_MASK   (15 << GIC_PRIORITY17_ID71_SHIFT)
#  define GIC_PRIORITY17_ID71(n)   ((uint32_t)(n) << GIC_PRIORITY17_ID71_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY18_ID72_SHIFT  (0)       /* Bits 4-7: ID 72 priority */
#define GIC_PRIORITY18_ID72_MASK   (15 << GIC_PRIORITY18_ID72_SHIFT)
#  define GIC_PRIORITY18_ID72(n)   ((uint32_t)(n) << GIC_PRIORITY18_ID72_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY18_ID73_SHIFT  (12)      /* Bits 12-15: ID 73 priority */
#define GIC_PRIORITY18_ID73_MASK   (15 << GIC_PRIORITY18_ID73_SHIFT)
#  define GIC_PRIORITY18_ID73(n)   ((uint32_t)(n) << GIC_PRIORITY18_ID73_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY18_ID74_SHIFT  (20)      /* Bits 20-23: ID 74 priority */
#define GIC_PRIORITY18_ID74_MASK   (15 << GIC_PRIORITY18_ID74_SHIFT)
#  define GIC_PRIORITY18_ID74(n)   ((uint32_t)(n) << GIC_PRIORITY18_ID74_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY18_ID75_SHIFT  (28)      /* Bits 28-31: ID 75 priority */
#define GIC_PRIORITY18_ID75_MASK   (15 << GIC_PRIORITY18_ID75_SHIFT)
#  define GIC_PRIORITY18_ID75(n)   ((uint32_t)(n) << GIC_PRIORITY18_ID75_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY19_ID76_SHIFT  (0)       /* Bits 4-7: ID 76 priority */
#define GIC_PRIORITY19_ID76_MASK   (15 << GIC_PRIORITY19_ID76_SHIFT)
#  define GIC_PRIORITY19_ID76(n)   ((uint32_t)(n) << GIC_PRIORITY19_ID76_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY19_ID77_SHIFT  (12)      /* Bits 12-15: ID 77 priority */
#define GIC_PRIORITY19_ID77_MASK   (15 << GIC_PRIORITY19_ID77_SHIFT)
#  define GIC_PRIORITY19_ID77(n)   ((uint32_t)(n) << GIC_PRIORITY19_ID77_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY19_ID719_SHIFT (20)      /* Bits 20-23: ID 78 priority */
#define GIC_PRIORITY19_ID719_MASK  (15 << GIC_PRIORITY19_ID719_SHIFT)
#  define GIC_PRIORITY19_ID78(n)   ((uint32_t)(n) << GIC_PRIORITY19_ID719_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY19_ID79_SHIFT  (28)      /* Bits 28-31: ID 78 priority */
#define GIC_PRIORITY19_ID79_MASK   (15 << GIC_PRIORITY19_ID79_SHIFT)
#  define GIC_PRIORITY19_ID79(n)   ((uint32_t)(n) << GIC_PRIORITY19_ID79_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY20_ID80_SHIFT  (0)       /* Bits 4-7: ID 80 priority */
#define GIC_PRIORITY20_ID80_MASK   (15 << GIC_PRIORITY20_ID80_SHIFT)
#  define GIC_PRIORITY20_ID80(n)   ((uint32_t)(n) << GIC_PRIORITY20_ID80_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY20_ID81_SHIFT  (12)      /* Bits 12-15: ID 81 priority */
#define GIC_PRIORITY20_ID81_MASK   (15 << GIC_PRIORITY20_ID81_SHIFT)
#  define GIC_PRIORITY20_ID81(n)   ((uint32_t)(n) << GIC_PRIORITY20_ID81_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY20_ID82_SHIFT  (20)      /* Bits 20-23: ID 82 priority */
#define GIC_PRIORITY20_ID82_MASK   (15 << GIC_PRIORITY20_ID82_SHIFT)
#  define GIC_PRIORITY20_ID82(n)   ((uint32_t)(n) << GIC_PRIORITY20_ID82_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY20_ID83_SHIFT  (28)      /* Bits 28-31: ID 83 priority */
#define GIC_PRIORITY20_ID83_MASK   (15 << GIC_PRIORITY20_ID83_SHIFT)
#  define GIC_PRIORITY20_ID83(n)   ((uint32_t)(n) << GIC_PRIORITY20_ID83_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY21_ID84_SHIFT  (0)       /* Bits 4-7: ID 84 priority */
#define GIC_PRIORITY21_ID84_MASK   (15 << GIC_PRIORITY21_ID84_SHIFT)
#  define GIC_PRIORITY21_ID84(n)   ((uint32_t)(n) << GIC_PRIORITY21_ID84_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY21_ID85_SHIFT  (12)      /* Bits 12-15: ID 85 priority */
#define GIC_PRIORITY21_ID85_MASK   (15 << GIC_PRIORITY21_ID85_SHIFT)
#  define GIC_PRIORITY21_ID85(n)   ((uint32_t)(n) << GIC_PRIORITY21_ID85_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY21_ID86_SHIFT  (20)      /* Bits 20-23: ID 86 priority */
#define GIC_PRIORITY21_ID86_MASK   (15 << GIC_PRIORITY21_ID86_SHIFT)
#  define GIC_PRIORITY21_ID86(n)   ((uint32_t)(n) << GIC_PRIORITY21_ID86_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY21_ID87_SHIFT  (28)      /* Bits 28-31: ID 87 priority */
#define GIC_PRIORITY21_ID87_MASK   (15 << GIC_PRIORITY21_ID87_SHIFT)
#  define GIC_PRIORITY21_ID87(n)   ((uint32_t)(n) << GIC_PRIORITY21_ID87_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY22_ID822_SHIFT (0)       /* Bits 4-7: ID 88 priority */
#define GIC_PRIORITY22_ID822_MASK  (15 << GIC_PRIORITY22_ID822_SHIFT)
#  define GIC_PRIORITY22_ID88(n)   ((uint32_t)(n) << GIC_PRIORITY22_ID822_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY22_ID89_SHIFT  (12)      /* Bits 12-15: ID 89 priority */
#define GIC_PRIORITY22_ID89_MASK   (15 << GIC_PRIORITY22_ID89_SHIFT)
#  define GIC_PRIORITY22_ID89(n)   ((uint32_t)(n) << GIC_PRIORITY22_ID89_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY22_ID90_SHIFT  (20)      /* Bits 20-23: ID 90 priority */
#define GIC_PRIORITY22_ID90_MASK   (15 << GIC_PRIORITY22_ID90_SHIFT)
#  define GIC_PRIORITY22_ID90(n)   ((uint32_t)(n) << GIC_PRIORITY22_ID90_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY22_ID91_SHIFT  (28)      /* Bits 28-31: ID 91 priority */
#define GIC_PRIORITY22_ID91_MASK   (15 << GIC_PRIORITY22_ID91_SHIFT)
#  define GIC_PRIORITY22_ID91(n)   ((uint32_t)(n) << GIC_PRIORITY22_ID91_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_PRIORITY23_ID92_SHIFT  (0)       /* Bits 4-7: ID 92 priority */
#define GIC_PRIORITY23_ID92_MASK   (15 << GIC_PRIORITY23_ID92_SHIFT)
#  define GIC_PRIORITY23_ID92(n)   ((uint32_t)(n) << GIC_PRIORITY23_ID92_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_PRIORITY23_ID93_SHIFT  (12)      /* Bits 12-15: ID 93 priority */
#define GIC_PRIORITY23_ID93_MASK   (15 << GIC_PRIORITY23_ID93_SHIFT)
#  define GIC_PRIORITY23_ID93(n)   ((uint32_t)(n) << GIC_PRIORITY23_ID93_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_PRIORITY23_ID94_SHIFT  (20)      /* Bits 20-23: ID 94 priority */
#define GIC_PRIORITY23_ID94_MASK   (15 << GIC_PRIORITY23_ID94_SHIFT)
#  define GIC_PRIORITY23_ID94(n)   ((uint32_t)(n) << GIC_PRIORITY23_ID94_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_PRIORITY23_ID95_SHIFT  (28)      /* Bits 28-31: ID 95 priority */
#define GIC_PRIORITY23_ID95_MASK   (15 << GIC_PRIORITY23_ID95_SHIFT)
#  define GIC_PRIORITY23_ID95(n)   ((uint32_t)(n) << GIC_PRIORITY23_ID95_SHIFT)

/* CPU targets 8...23 */

#define CPU0_TARGET                (1 << 0)
#define CPU1_TARGET                (1 << 1)
#define CPU2_TARGET                (1 << 2)
#define CPU3_TARGET                (1 << 3)

#define GIC_CPUTARGET_ID_SHIFT(n)  (((n) & 3) << 3)
#define GIC_CPUTARGET_ID_MASK(n)   (15 << GIC_CPUTARGET_ID_SHIFT(n))
#  define GIC_CPUTARGET_ID(n,t)    ((uint32_t)(t) << GIC_CPUTARGET_ID_SHIFT(n))

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET8_ID32_SHIFT  (0)       /* Bits 4-7: ID 32 CPU target */
#define GIC_CPUTARGET8_ID32_MASK   (15 << GIC_CPUTARGET8_ID32_SHIFT)
#  define GIC_CPUTARGET8_ID32(n)   ((uint32_t)(n) << GIC_CPUTARGET8_ID32_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET8_ID33_SHIFT  (12)      /* Bits 12-15: ID 33 CPU target */
#define GIC_CPUTARGET8_ID33_MASK   (15 << GIC_CPUTARGET8_ID33_SHIFT)
#  define GIC_CPUTARGET8_ID33(n)   ((uint32_t)(n) << GIC_CPUTARGET8_ID33_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET8_ID34_SHIFT  (20)      /* Bits 20-23: ID 34 CPU target */
#define GIC_CPUTARGET8_ID34_MASK   (15 << GIC_CPUTARGET8_ID34_SHIFT)
#  define GIC_CPUTARGET8_ID34(n)   ((uint32_t)(n) << GIC_CPUTARGET8_ID34_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET8_ID35_SHIFT  (28)      /* Bits 28-31: ID 35 CPU target */
#define GIC_CPUTARGET8_ID35_MASK   (15 << GIC_CPUTARGET8_ID35_SHIFT)
#  define GIC_CPUTARGET8_ID35(n)   ((uint32_t)(n) << GIC_CPUTARGET8_ID35_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET9_ID36_SHIFT  (0)       /* Bits 4-7: ID 36 CPU target */
#define GIC_CPUTARGET9_ID36_MASK   (15 << GIC_CPUTARGET9_ID36_SHIFT)
#  define GIC_CPUTARGET9_ID36(n)   ((uint32_t)(n) << GIC_CPUTARGET9_ID36_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET9_ID37_SHIFT  (12)      /* Bits 12-15: ID 37 CPU target */
#define GIC_CPUTARGET9_ID37_MASK   (15 << GIC_CPUTARGET9_ID37_SHIFT)
#  define GIC_CPUTARGET9_ID37(n)   ((uint32_t)(n) << GIC_CPUTARGET9_ID37_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET9_ID38_SHIFT  (20)      /* Bits 20-23: ID 38 CPU target */
#define GIC_CPUTARGET9_ID38_MASK   (15 << GIC_CPUTARGET9_ID38_SHIFT)
#  define GIC_CPUTARGET9_ID38(n)   ((uint32_t)(n) << GIC_CPUTARGET9_ID38_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET9_ID39_SHIFT  (28)      /* Bits 28-31: ID 39 CPU target */
#define GIC_CPUTARGET9_ID39_MASK   (15 << GIC_CPUTARGET9_ID39_SHIFT)
#  define GIC_CPUTARGET9_ID39(n)   ((uint32_t)(n) << GIC_CPUTARGET9_ID39_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET10_ID40_SHIFT (0)       /* Bits 4-7: ID 40 CPU target */
#define GIC_CPUTARGET10_ID40_MASK  (15 << GIC_CPUTARGET10_ID40_SHIFT)
#  define GIC_CPUTARGET10_ID40(n)  ((uint32_t)(n) << GIC_CPUTARGET10_ID40_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET10_ID41_SHIFT (12)      /* Bits 12-15: ID 41 CPU target */
#define GIC_CPUTARGET10_ID41_MASK  (15 << GIC_CPUTARGET10_ID41_SHIFT)
#  define GIC_CPUTARGET10_ID41(n)  ((uint32_t)(n) << GIC_CPUTARGET10_ID41_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET10_ID42_SHIFT (20)      /* Bits 20-23: ID 42 CPU target */
#define GIC_CPUTARGET10_ID42_MASK  (15 << GIC_CPUTARGET10_ID42_SHIFT)
#  define GIC_CPUTARGET10_ID42(n)  ((uint32_t)(n) << GIC_CPUTARGET10_ID42_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET10_ID43_SHIFT (28)      /* Bits 28-31: ID 43 CPU target */
#define GIC_CPUTARGET10_ID43_MASK  (15 << GIC_CPUTARGET10_ID43_SHIFT)
#  define GIC_CPUTARGET10_ID43(n)  ((uint32_t)(n) << GIC_CPUTARGET10_ID43_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET11_ID44_SHIFT (0)       /* Bits 4-7: ID 44 CPU target */
#define GIC_CPUTARGET11_ID44_MASK  (15 << GIC_CPUTARGET11_ID44_SHIFT)
#  define GIC_CPUTARGET11_ID44(n)  ((uint32_t)(n) << GIC_CPUTARGET11_ID44_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET11_ID45_SHIFT (12)      /* Bits 12-15: ID 45 CPU target */
#define GIC_CPUTARGET11_ID45_MASK  (15 << GIC_CPUTARGET11_ID45_SHIFT)
#  define GIC_CPUTARGET11_ID45(n)  ((uint32_t)(n) << GIC_CPUTARGET11_ID45_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET11_ID46_SHIFT (20)      /* Bits 20-23: ID 46 CPU target */
#define GIC_CPUTARGET11_ID46_MASK  (15 << GIC_CPUTARGET11_ID46_SHIFT)
#  define GIC_CPUTARGET11_ID46(n)  ((uint32_t)(n) << GIC_CPUTARGET11_ID46_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET11_ID47_SHIFT (28)      /* Bits 28-31: ID 47 CPU target */
#define GIC_CPUTARGET11_ID47_MASK  (15 << GIC_CPUTARGET11_ID47_SHIFT)
#  define GIC_CPUTARGET11_ID47(n)  ((uint32_t)(n) << GIC_CPUTARGET11_ID47_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET12_ID48_SHIFT (0)       /* Bits 4-7: ID 48 CPU target */
#define GIC_CPUTARGET12_ID48_MASK  (15 << GIC_CPUTARGET12_ID48_SHIFT)
#  define GIC_CPUTARGET12_ID48(n)  ((uint32_t)(n) << GIC_CPUTARGET12_ID48_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET12_ID49_SHIFT (12)      /* Bits 12-15: ID 49 CPU target */
#define GIC_CPUTARGET12_ID49_MASK  (15 << GIC_CPUTARGET12_ID49_SHIFT)
#  define GIC_CPUTARGET12_ID49(n)  ((uint32_t)(n) << GIC_CPUTARGET12_ID49_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET12_ID50_SHIFT (20)      /* Bits 20-23: ID 50 CPU target */
#define GIC_CPUTARGET12_ID50_MASK  (15 << GIC_CPUTARGET12_ID50_SHIFT)
#  define GIC_CPUTARGET12_ID50(n)  ((uint32_t)(n) << GIC_CPUTARGET12_ID50_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET12_ID51_SHIFT (28)      /* Bits 28-31: ID 51 CPU target */
#define GIC_CPUTARGET12_ID51_MASK  (15 << GIC_CPUTARGET12_ID51_SHIFT)
#  define GIC_CPUTARGET12_ID51(n)  ((uint32_t)(n) << GIC_CPUTARGET12_ID51_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET13_ID52_SHIFT (0)       /* Bits 4-7: ID 52 CPU target */
#define GIC_CPUTARGET13_ID52_MASK  (15 << GIC_CPUTARGET13_ID52_SHIFT)
#  define GIC_CPUTARGET13_ID52(n)  ((uint32_t)(n) << GIC_CPUTARGET13_ID52_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET13_ID53_SHIFT (12)      /* Bits 12-15: ID 53 CPU target */
#define GIC_CPUTARGET13_ID53_MASK  (15 << GIC_CPUTARGET13_ID53_SHIFT)
#  define GIC_CPUTARGET13_ID53(n)  ((uint32_t)(n) << GIC_CPUTARGET13_ID53_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET13_ID54_SHIFT (20)      /* Bits 20-23: ID 54 CPU target */
#define GIC_CPUTARGET13_ID54_MASK  (15 << GIC_CPUTARGET13_ID54_SHIFT)
#  define GIC_CPUTARGET13_ID54(n)  ((uint32_t)(n) << GIC_CPUTARGET13_ID54_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET13_ID55_SHIFT (28)      /* Bits 28-31: ID 55 CPU target */
#define GIC_CPUTARGET13_ID55_MASK  (15 << GIC_CPUTARGET13_ID55_SHIFT)
#  define GIC_CPUTARGET13_ID55(n)  ((uint32_t)(n) << GIC_CPUTARGET13_ID55_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET14_ID56_SHIFT (0)       /* Bits 4-7: ID 56 CPU target */
#define GIC_CPUTARGET14_ID56_MASK  (15 << GIC_CPUTARGET14_ID56_SHIFT)
#  define GIC_CPUTARGET14_ID56(n)  ((uint32_t)(n) << GIC_CPUTARGET14_ID56_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET14_ID57_SHIFT (12)      /* Bits 12-15: ID 57 CPU target */
#define GIC_CPUTARGET14_ID57_MASK  (15 << GIC_CPUTARGET14_ID57_SHIFT)
#  define GIC_CPUTARGET14_ID57(n)  ((uint32_t)(n) << GIC_CPUTARGET14_ID57_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET14_ID58_SHIFT (20)      /* Bits 20-23: ID 58 CPU target */
#define GIC_CPUTARGET14_ID58_MASK  (15 << GIC_CPUTARGET14_ID58_SHIFT)
#  define GIC_CPUTARGET14_ID58(n)  ((uint32_t)(n) << GIC_CPUTARGET14_ID58_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET14_ID59_SHIFT (28)      /* Bits 28-31: ID 59 CPU target */
#define GIC_CPUTARGET14_ID59_MASK  (15 << GIC_CPUTARGET14_ID59_SHIFT)
#  define GIC_CPUTARGET14_ID59(n)  ((uint32_t)(n) << GIC_CPUTARGET14_ID59_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET15_ID60_SHIFT (0)       /* Bits 4-7: ID 60 CPU target */
#define GIC_CPUTARGET15_ID60_MASK  (15 << GIC_CPUTARGET15_ID60_SHIFT)
#  define GIC_CPUTARGET15_ID60(n)  ((uint32_t)(n) << GIC_CPUTARGET15_ID60_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET15_ID61_SHIFT (12)      /* Bits 12-15: ID 61 CPU target */
#define GIC_CPUTARGET15_ID61_MASK  (15 << GIC_CPUTARGET15_ID61_SHIFT)
#  define GIC_CPUTARGET15_ID61(n)  ((uint32_t)(n) << GIC_CPUTARGET15_ID61_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET15_ID62_SHIFT (20)      /* Bits 20-23: ID 62 CPU target */
#define GIC_CPUTARGET15_ID62_MASK  (15 << GIC_CPUTARGET15_ID62_SHIFT)
#  define GIC_CPUTARGET15_ID62(n)  ((uint32_t)(n) << GIC_CPUTARGET15_ID62_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET15_ID63_SHIFT (28)      /* Bits 28-31: ID 63 CPU target */
#define GIC_CPUTARGET15_ID63_MASK  (15 << GIC_CPUTARGET15_ID63_SHIFT)
#  define GIC_CPUTARGET15_ID63(n)  ((uint32_t)(n) << GIC_CPUTARGET15_ID63_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET16_ID64_SHIFT (0)       /* Bits 4-7: ID 64 CPU target */
#define GIC_CPUTARGET16_ID64_MASK  (15 << GIC_CPUTARGET16_ID64_SHIFT)
#  define GIC_CPUTARGET16_ID64(n)  ((uint32_t)(n) << GIC_CPUTARGET16_ID64_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET16_ID65_SHIFT (12)      /* Bits 12-15: ID 65 CPU target */
#define GIC_CPUTARGET16_ID65_MASK  (15 << GIC_CPUTARGET16_ID65_SHIFT)
#  define GIC_CPUTARGET16_ID65(n)  ((uint32_t)(n) << GIC_CPUTARGET16_ID65_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET16_ID66_SHIFT (20)      /* Bits 20-23: ID 66 CPU target */
#define GIC_CPUTARGET16_ID66_MASK  (15 << GIC_CPUTARGET16_ID66_SHIFT)
#  define GIC_CPUTARGET16_ID66(n)  ((uint32_t)(n) << GIC_CPUTARGET16_ID66_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET16_ID67_SHIFT (28)      /* Bits 28-31: ID 67 CPU target */
#define GIC_CPUTARGET16_ID67_MASK  (15 << GIC_CPUTARGET16_ID67_SHIFT)
#  define GIC_CPUTARGET16_ID67(n)  ((uint32_t)(n) << GIC_CPUTARGET16_ID67_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET17_ID68_SHIFT (0)       /* Bits 4-7: ID 68 CPU target */
#define GIC_CPUTARGET17_ID68_MASK  (15 << GIC_CPUTARGET17_ID68_SHIFT)
#  define GIC_CPUTARGET17_ID68(n)  ((uint32_t)(n) << GIC_CPUTARGET17_ID68_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET17_ID69_SHIFT (12)      /* Bits 12-15: ID 69 CPU target */
#define GIC_CPUTARGET17_ID69_MASK  (15 << GIC_CPUTARGET17_ID69_SHIFT)
#  define GIC_CPUTARGET17_ID69(n)  ((uint32_t)(n) << GIC_CPUTARGET17_ID69_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET17_ID70_SHIFT (20)      /* Bits 20-23: ID 70 CPU target */
#define GIC_CPUTARGET17_ID70_MASK  (15 << GIC_CPUTARGET17_ID70_SHIFT)
#  define GIC_CPUTARGET17_ID70(n)  ((uint32_t)(n) << GIC_CPUTARGET17_ID70_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET17_ID71_SHIFT (28)      /* Bits 28-31: ID 71 CPU target */
#define GIC_CPUTARGET17_ID71_MASK  (15 << GIC_CPUTARGET17_ID71_SHIFT)
#  define GIC_CPUTARGET17_ID71(n)  ((uint32_t)(n) << GIC_CPUTARGET17_ID71_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET18_ID72_SHIFT (0)       /* Bits 4-7: ID 72 CPU target */
#define GIC_CPUTARGET18_ID72_MASK  (15 << GIC_CPUTARGET18_ID72_SHIFT)
#  define GIC_CPUTARGET18_ID72(n)  ((uint32_t)(n) << GIC_CPUTARGET18_ID72_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET18_ID73_SHIFT (12)      /* Bits 12-15: ID 73 CPU target */
#define GIC_CPUTARGET18_ID73_MASK  (15 << GIC_CPUTARGET18_ID73_SHIFT)
#  define GIC_CPUTARGET18_ID73(n)  ((uint32_t)(n) << GIC_CPUTARGET18_ID73_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET18_ID74_SHIFT (20)      /* Bits 20-23: ID 74 CPU target */
#define GIC_CPUTARGET18_ID74_MASK  (15 << GIC_CPUTARGET18_ID74_SHIFT)
#  define GIC_CPUTARGET18_ID74(n)  ((uint32_t)(n) << GIC_CPUTARGET18_ID74_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET18_ID75_SHIFT (28)      /* Bits 28-31: ID 75 CPU target */
#define GIC_CPUTARGET18_ID75_MASK  (15 << GIC_CPUTARGET18_ID75_SHIFT)
#  define GIC_CPUTARGET18_ID75(n)  ((uint32_t)(n) << GIC_CPUTARGET18_ID75_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET19_ID76_SHIFT (0)       /* Bits 4-7: ID 76 CPU target */
#define GIC_CPUTARGET19_ID76_MASK  (15 << GIC_CPUTARGET19_ID76_SHIFT)
#  define GIC_CPUTARGET19_ID76(n)  ((uint32_t)(n) << GIC_CPUTARGET19_ID76_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET19_ID77_SHIFT (12)      /* Bits 12-15: ID 77 CPU target */
#define GIC_CPUTARGET19_ID77_MASK  (15 << GIC_CPUTARGET19_ID77_SHIFT)
#  define GIC_CPUTARGET19_ID77(n)  ((uint32_t)(n) << GIC_CPUTARGET19_ID77_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET19_ID78_SHIFT (20)      /* Bits 20-23: ID 78 CPU target */
#define GIC_CPUTARGET19_ID78_MASK  (15 << GIC_CPUTARGET19_ID78_SHIFT)
#  define GIC_CPUTARGET19_ID78(n)  ((uint32_t)(n) << GIC_CPUTARGET19_ID78_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET19_ID79_SHIFT (28)      /* Bits 28-31: ID 79 CPU target */
#define GIC_CPUTARGET19_ID79_MASK  (15 << GIC_CPUTARGET19_ID79_SHIFT)
#  define GIC_CPUTARGET19_ID79(n)  ((uint32_t)(n) << GIC_CPUTARGET19_ID79_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET20_ID80_SHIFT (0)       /* Bits 4-7: ID 80 CPU target */
#define GIC_CPUTARGET20_ID80_MASK  (15 << GIC_CPUTARGET20_ID80_SHIFT)
#  define GIC_CPUTARGET20_ID80(n)  ((uint32_t)(n) << GIC_CPUTARGET20_ID80_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET20_ID81_SHIFT (12)      /* Bits 12-15: ID 81 CPU target */
#define GIC_CPUTARGET20_ID81_MASK  (15 << GIC_CPUTARGET20_ID81_SHIFT)
#  define GIC_CPUTARGET20_ID81(n)  ((uint32_t)(n) << GIC_CPUTARGET20_ID81_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET20_ID82_SHIFT (20)      /* Bits 20-23: ID 82 CPU target */
#define GIC_CPUTARGET20_ID82_MASK  (15 << GIC_CPUTARGET20_ID82_SHIFT)
#  define GIC_CPUTARGET20_ID82(n)  ((uint32_t)(n) << GIC_CPUTARGET20_ID82_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET20_ID83_SHIFT (28)      /* Bits 28-31: ID 83 CPU target */
#define GIC_CPUTARGET20_ID83_MASK  (15 << GIC_CPUTARGET20_ID83_SHIFT)
#  define GIC_CPUTARGET20_ID83(n)  ((uint32_t)(n) << GIC_CPUTARGET20_ID83_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET21_ID84_SHIFT (0)       /* Bits 4-7: ID 84 CPU target */
#define GIC_CPUTARGET21_ID84_MASK  (15 << GIC_CPUTARGET21_ID84_SHIFT)
#  define GIC_CPUTARGET21_ID84(n)  ((uint32_t)(n) << GIC_CPUTARGET21_ID84_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET21_ID85_SHIFT (12)      /* Bits 12-15: ID 85 CPU target */
#define GIC_CPUTARGET21_ID85_MASK  (15 << GIC_CPUTARGET21_ID85_SHIFT)
#  define GIC_CPUTARGET21_ID85(n)  ((uint32_t)(n) << GIC_CPUTARGET21_ID85_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET21_ID86_SHIFT (20)      /* Bits 20-23: ID 86 CPU target */
#define GIC_CPUTARGET21_ID86_MASK  (15 << GIC_CPUTARGET21_ID86_SHIFT)
#  define GIC_CPUTARGET21_ID86(n)  ((uint32_t)(n) << GIC_CPUTARGET21_ID86_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET21_ID87_SHIFT (28)      /* Bits 28-31: ID 87 CPU target */
#define GIC_CPUTARGET21_ID87_MASK  (15 << GIC_CPUTARGET21_ID87_SHIFT)
#  define GIC_CPUTARGET21_ID87(n)  ((uint32_t)(n) << GIC_CPUTARGET21_ID87_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET22_ID88_SHIFT (0)       /* Bits 4-7: ID 88 CPU target */
#define GIC_CPUTARGET22_ID88_MASK  (15 << GIC_CPUTARGET22_ID88_SHIFT)
#  define GIC_CPUTARGET22_ID88(n)  ((uint32_t)(n) << GIC_CPUTARGET22_ID88_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET22_ID89_SHIFT (12)      /* Bits 12-15: ID 89 CPU target */
#define GIC_CPUTARGET22_ID89_MASK  (15 << GIC_CPUTARGET22_ID89_SHIFT)
#  define GIC_CPUTARGET22_ID89(n)  ((uint32_t)(n) << GIC_CPUTARGET22_ID89_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET22_ID90_SHIFT (20)      /* Bits 20-23: ID 90 CPU target */
#define GIC_CPUTARGET22_ID90_MASK  (15 << GIC_CPUTARGET22_ID90_SHIFT)
#  define GIC_CPUTARGET22_ID90(n)  ((uint32_t)(n) << GIC_CPUTARGET22_ID90_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET22_ID91_SHIFT (28)      /* Bits 28-31: ID 91 CPU target */
#define GIC_CPUTARGET22_ID91_MASK  (15 << GIC_CPUTARGET22_ID91_SHIFT)
#  define GIC_CPUTARGET22_ID91(n)  ((uint32_t)(n) << GIC_CPUTARGET22_ID91_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_CPUTARGET23_ID92_SHIFT (0)       /* Bits 4-7: ID 92 CPU target */
#define GIC_CPUTARGET23_ID92_MASK  (15 << GIC_CPUTARGET23_ID92_SHIFT)
#  define GIC_CPUTARGET23_ID92(n)  ((uint32_t)(n) << GIC_CPUTARGET23_ID92_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_CPUTARGET23_ID93_SHIFT (12)      /* Bits 12-15: ID 93 CPU target */
#define GIC_CPUTARGET23_ID93_MASK  (15 << GIC_CPUTARGET23_ID93_SHIFT)
#  define GIC_CPUTARGET23_ID93(n)  ((uint32_t)(n) << GIC_CPUTARGET23_ID93_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_CPUTARGET23_ID94_SHIFT (20)      /* Bits 20-23: ID 94 CPU target */
#define GIC_CPUTARGET23_ID94_MASK  (15 << GIC_CPUTARGET23_ID94_SHIFT)
#  define GIC_CPUTARGET23_ID94(n)  ((uint32_t)(n) << GIC_CPUTARGET23_ID94_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_CPUTARGET23_ID95_SHIFT (28)      /* Bits 28-31: ID 95 CPU target */
#define GIC_CPUTARGET23_ID95_MASK  (15 << GIC_CPUTARGET23_ID95_SHIFT)
#  define GIC_CPUTARGET23_ID95(n)  ((uint32_t)(n) << GIC_CPUTARGET23_ID95_SHIFT)

/* Configuration 2, 3, 4, 5 */

#define INT_CONFIG_NN              0         /* Bit n: 0= N-N Model */
#define INT_CONFIG_1N              1         /* Bit n: 1= 1-N Model */
#define INT_CONFIG_LEVEL           0         /* Bit n+1: 0=Level sensitive */
#define INT_CONFIG_EDGE            2         /* Bit n+2: 1=Edge sensitive */
 
#define GIC_CONFIG_ID_SHIFT(n)     (((n) & 15) << 1)
#define GIC_CONFIG_ID_MASK(n)      (3 << GIC_CONFIG_ID_SHIFT(n))
#  define GIC_CONFIG_ID(n,c)       ((uint32_t)(c) << GIC_CONFIG_ID_SHIFT(n))

#define GIC_CONFIG2_ID32_SHIFT     (0)       /* Bits 0-1: ID 32 configuration */
#define GIC_CONFIG2_ID32_MASK      (3 << GIC_CONFIG2_ID32_SHIFT)
#  define GIC_CONFIG2_ID32(n)      ((uint32_t)(n) << GIC_CONFIG2_ID32_SHIFT)
#define GIC_CONFIG2_ID33_SHIFT     (2)       /* Bits 2-3: ID 33 configuration */
#define GIC_CONFIG2_ID33_MASK      (3 << GIC_CONFIG2_ID33_SHIFT)
#  define GIC_CONFIG2_ID33(n)      ((uint32_t)(n) << GIC_CONFIG2_ID33_SHIFT)
#define GIC_CONFIG2_ID34_SHIFT     (4)       /* Bits 4-5: ID 34 configuration */
#define GIC_CONFIG2_ID34_MASK      (3 << GIC_CONFIG2_ID34_SHIFT)
#  define GIC_CONFIG2_ID34(n)      ((uint32_t)(n) << GIC_CONFIG2_ID34_SHIFT)
#define GIC_CONFIG2_ID35_SHIFT     (6)       /* Bits 6-7: ID 35 configuration */
#define GIC_CONFIG2_ID35_MASK      (3 << GIC_CONFIG2_ID35_SHIFT)
#  define GIC_CONFIG2_ID35(n)      ((uint32_t)(n) << GIC_CONFIG2_ID35_SHIFT)
#define GIC_CONFIG2_ID36_SHIFT     (8)       /* Bits 8-9: ID 36 configuration */
#define GIC_CONFIG2_ID36_MASK      (3 << GIC_CONFIG2_ID36_SHIFT)
#  define GIC_CONFIG2_ID36(n)      ((uint32_t)(n) << GIC_CONFIG2_ID36_SHIFT)
#define GIC_CONFIG2_ID37_SHIFT     (10)      /* Bits 10-11: ID 37 configuration */
#define GIC_CONFIG2_ID37_MASK      (3 << GIC_CONFIG2_ID37_SHIFT)
#  define GIC_CONFIG2_ID37(n)      ((uint32_t)(n) << GIC_CONFIG2_ID37_SHIFT)
#define GIC_CONFIG2_ID38_SHIFT     (12)      /* Bits 12-13: ID 38 configuration */
#define GIC_CONFIG2_ID38_MASK      (3 << GIC_CONFIG2_ID38_SHIFT)
#  define GIC_CONFIG2_ID38(n)      ((uint32_t)(n) << GIC_CONFIG2_ID38_SHIFT)
#define GIC_CONFIG2_ID39_SHIFT     (14)      /* Bits 14-15: ID 39 configuration */
#define GIC_CONFIG2_ID39_MASK      (3 << GIC_CONFIG2_ID39_SHIFT)
#  define GIC_CONFIG2_ID39(n)      ((uint32_t)(n) << GIC_CONFIG2_ID39_SHIFT)
#define GIC_CONFIG2_ID40_SHIFT     (16)      /* Bits 16-17: ID 40 configuration */
#define GIC_CONFIG2_ID40_MASK      (3 << GIC_CONFIG2_ID40_SHIFT)
#  define GIC_CONFIG2_ID40(n)      ((uint32_t)(n) << GIC_CONFIG2_ID40_SHIFT)
#define GIC_CONFIG2_ID41_SHIFT     (18)      /* Bits 18-19: ID 41 configuration */
#define GIC_CONFIG2_ID41_MASK      (3 << GIC_CONFIG2_ID41_SHIFT)
#  define GIC_CONFIG2_ID41(n)      ((uint32_t)(n) << GIC_CONFIG2_ID41_SHIFT)
#define GIC_CONFIG2_ID42_SHIFT     (20)      /* Bits 20-21: ID 42 configuration */
#define GIC_CONFIG2_ID42_MASK      (3 << GIC_CONFIG2_ID42_SHIFT)
#  define GIC_CONFIG2_ID42(n)      ((uint32_t)(n) << GIC_CONFIG2_ID42_SHIFT)
#define GIC_CONFIG2_ID43_SHIFT     (22)      /* Bits 22-23: ID 43 configuration */
#define GIC_CONFIG2_ID43_MASK      (3 << GIC_CONFIG2_ID43_SHIFT)
#  define GIC_CONFIG2_ID43(n)      ((uint32_t)(n) << GIC_CONFIG2_ID43_SHIFT)
#define GIC_CONFIG2_ID44_SHIFT     (24)      /* Bits 24-25: ID 44 configuration */
#define GIC_CONFIG2_ID44_MASK      (3 << GIC_CONFIG2_ID44_SHIFT)
#  define GIC_CONFIG2_ID44(n)      ((uint32_t)(n) << GIC_CONFIG2_ID44_SHIFT)
#define GIC_CONFIG2_ID45_SHIFT     (26)      /* Bits 26-27: ID 45 configuration */
#define GIC_CONFIG2_ID45_MASK      (3 << GIC_CONFIG2_ID45_SHIFT)
#  define GIC_CONFIG2_ID45(n)      ((uint32_t)(n) << GIC_CONFIG2_ID45_SHIFT)
#define GIC_CONFIG2_ID46_SHIFT     (28)      /* Bits 28-29: ID 46 configuration */
#define GIC_CONFIG2_ID46_MASK      (3 << GIC_CONFIG2_ID46_SHIFT)
#  define GIC_CONFIG2_ID46(n)      ((uint32_t)(n) << GIC_CONFIG2_ID46_SHIFT)
#define GIC_CONFIG2_ID47_SHIFT     (30)      /* Bits 30-31: ID 47 configuration */
#define GIC_CONFIG2_ID47_MASK      (3 << GIC_CONFIG2_ID47_SHIFT)
#  define GIC_CONFIG2_ID47(n)      ((uint32_t)(n) << GIC_CONFIG2_ID47_SHIFT)

#define GIC_CONFIG3_ID48_SHIFT     (0)       /* Bits 0-1: ID 48 configuration */
#define GIC_CONFIG3_ID48_MASK      (3 << GIC_CONFIG3_ID48_SHIFT)
#  define GIC_CONFIG3_ID48(n)      ((uint32_t)(n) << GIC_CONFIG3_ID48_SHIFT)
#define GIC_CONFIG3_ID49_SHIFT     (2)       /* Bits 2-3: ID 49 configuration */
#define GIC_CONFIG3_ID49_MASK      (3 << GIC_CONFIG3_ID49_SHIFT)
#  define GIC_CONFIG3_ID49(n)      ((uint32_t)(n) << GIC_CONFIG3_ID49_SHIFT)
#define GIC_CONFIG3_ID50_SHIFT     (4)       /* Bits 4-5: ID 50 configuration */
#define GIC_CONFIG3_ID50_MASK      (3 << GIC_CONFIG3_ID50_SHIFT)
#  define GIC_CONFIG3_ID50(n)      ((uint32_t)(n) << GIC_CONFIG3_ID50_SHIFT)
#define GIC_CONFIG3_ID51_SHIFT     (6)       /* Bits 6-7: ID 51 configuration */
#define GIC_CONFIG3_ID51_MASK      (3 << GIC_CONFIG3_ID51_SHIFT)
#  define GIC_CONFIG3_ID51(n)      ((uint32_t)(n) << GIC_CONFIG3_ID51_SHIFT)
#define GIC_CONFIG3_ID52_SHIFT     (8)       /* Bits 8-9: ID 52 configuration */
#define GIC_CONFIG3_ID52_MASK      (3 << GIC_CONFIG3_ID52_SHIFT)
#  define GIC_CONFIG3_ID52(n)      ((uint32_t)(n) << GIC_CONFIG3_ID52_SHIFT)
#define GIC_CONFIG3_ID53_SHIFT     (10)      /* Bits 10-11: ID 53 configuration */
#define GIC_CONFIG3_ID53_MASK      (3 << GIC_CONFIG3_ID53_SHIFT)
#  define GIC_CONFIG3_ID53(n)      ((uint32_t)(n) << GIC_CONFIG3_ID53_SHIFT)
#define GIC_CONFIG3_ID54_SHIFT     (12)      /* Bits 12-13: ID 54 configuration */
#define GIC_CONFIG3_ID54_MASK      (3 << GIC_CONFIG3_ID54_SHIFT)
#  define GIC_CONFIG3_ID54(n)      ((uint32_t)(n) << GIC_CONFIG3_ID54_SHIFT)
#define GIC_CONFIG3_ID55_SHIFT     (14)      /* Bits 14-15: ID 55 configuration */
#define GIC_CONFIG3_ID55_MASK      (3 << GIC_CONFIG3_ID55_SHIFT)
#  define GIC_CONFIG3_ID55(n)      ((uint32_t)(n) << GIC_CONFIG3_ID55_SHIFT)
#define GIC_CONFIG3_ID56_SHIFT     (16)      /* Bits 16-17: ID 56 configuration */
#define GIC_CONFIG3_ID56_MASK      (3 << GIC_CONFIG3_ID56_SHIFT)
#  define GIC_CONFIG3_ID56(n)      ((uint32_t)(n) << GIC_CONFIG3_ID56_SHIFT)
#define GIC_CONFIG3_ID57_SHIFT     (18)      /* Bits 18-19: ID 57 configuration */
#define GIC_CONFIG3_ID57_MASK      (3 << GIC_CONFIG3_ID57_SHIFT)
#  define GIC_CONFIG3_ID57(n)      ((uint32_t)(n) << GIC_CONFIG3_ID57_SHIFT)
#define GIC_CONFIG3_ID58_SHIFT     (20)      /* Bits 20-21: ID 58 configuration */
#define GIC_CONFIG3_ID58_MASK      (3 << GIC_CONFIG3_ID58_SHIFT)
#  define GIC_CONFIG3_ID58(n)      ((uint32_t)(n) << GIC_CONFIG3_ID58_SHIFT)
#define GIC_CONFIG3_ID59_SHIFT     (22)      /* Bits 22-23: ID 59 configuration */
#define GIC_CONFIG3_ID59_MASK      (3 << GIC_CONFIG3_ID59_SHIFT)
#  define GIC_CONFIG3_ID59(n)      ((uint32_t)(n) << GIC_CONFIG3_ID59_SHIFT)
#define GIC_CONFIG3_ID60_SHIFT     (24)      /* Bits 24-25: ID 60 configuration */
#define GIC_CONFIG3_ID60_MASK      (3 << GIC_CONFIG3_ID60_SHIFT)
#  define GIC_CONFIG3_ID60(n)      ((uint32_t)(n) << GIC_CONFIG3_ID60_SHIFT)
#define GIC_CONFIG3_ID61_SHIFT     (26)      /* Bits 26-27: ID 61 configuration */
#define GIC_CONFIG3_ID61_MASK      (3 << GIC_CONFIG3_ID61_SHIFT)
#  define GIC_CONFIG3_ID61(n)      ((uint32_t)(n) << GIC_CONFIG3_ID61_SHIFT)
#define GIC_CONFIG3_ID62_SHIFT     (28)      /* Bits 28-29: ID 62 configuration */
#define GIC_CONFIG3_ID62_MASK      (3 << GIC_CONFIG3_ID62_SHIFT)
#  define GIC_CONFIG3_ID62(n)      ((uint32_t)(n) << GIC_CONFIG3_ID62_SHIFT)
#define GIC_CONFIG3_ID63_SHIFT     (30)      /* Bits 30-31: ID 63 configuration */
#define GIC_CONFIG3_ID63_MASK      (3 << GIC_CONFIG3_ID63_SHIFT)
#  define GIC_CONFIG3_ID63(n)      ((uint32_t)(n) << GIC_CONFIG3_ID63_SHIFT)

#define GIC_CONFIG4_ID64_SHIFT     (0)       /* Bits 0-1: ID 64 configuration */
#define GIC_CONFIG4_ID64_MASK      (3 << GIC_CONFIG4_ID64_SHIFT)
#  define GIC_CONFIG4_ID64(n)      ((uint32_t)(n) << GIC_CONFIG4_ID64_SHIFT)
#define GIC_CONFIG4_ID65_SHIFT     (2)       /* Bits 2-3: ID 65 configuration */
#define GIC_CONFIG4_ID65_MASK      (3 << GIC_CONFIG4_ID65_SHIFT)
#  define GIC_CONFIG4_ID65(n)      ((uint32_t)(n) << GIC_CONFIG4_ID65_SHIFT)
#define GIC_CONFIG4_ID66_SHIFT     (4)       /* Bits 4-5: ID 66 configuration */
#define GIC_CONFIG4_ID66_MASK      (3 << GIC_CONFIG4_ID66_SHIFT)
#  define GIC_CONFIG4_ID66(n)      ((uint32_t)(n) << GIC_CONFIG4_ID66_SHIFT)
#define GIC_CONFIG4_ID67_SHIFT     (6)       /* Bits 6-7: ID 67 configuration */
#define GIC_CONFIG4_ID67_MASK      (3 << GIC_CONFIG4_ID67_SHIFT)
#  define GIC_CONFIG4_ID67(n)      ((uint32_t)(n) << GIC_CONFIG4_ID67_SHIFT)
#define GIC_CONFIG4_ID68_SHIFT     (8)       /* Bits 8-9: ID 68 configuration */
#define GIC_CONFIG4_ID68_MASK      (3 << GIC_CONFIG4_ID68_SHIFT)
#  define GIC_CONFIG4_ID68(n)      ((uint32_t)(n) << GIC_CONFIG4_ID68_SHIFT)
#define GIC_CONFIG4_ID69_SHIFT     (10)      /* Bits 10-11: ID 69 configuration */
#define GIC_CONFIG4_ID69_MASK      (3 << GIC_CONFIG4_ID69_SHIFT)
#  define GIC_CONFIG4_ID69(n)      ((uint32_t)(n) << GIC_CONFIG4_ID69_SHIFT)
#define GIC_CONFIG4_ID70_SHIFT     (12)      /* Bits 12-13: ID 70 configuration */
#define GIC_CONFIG4_ID70_MASK      (3 << GIC_CONFIG4_ID70_SHIFT)
#  define GIC_CONFIG4_ID70(n)      ((uint32_t)(n) << GIC_CONFIG4_ID70_SHIFT)
#define GIC_CONFIG4_ID71_SHIFT     (14)      /* Bits 14-15: ID 71 configuration */
#define GIC_CONFIG4_ID71_MASK      (3 << GIC_CONFIG4_ID71_SHIFT)
#  define GIC_CONFIG4_ID71(n)      ((uint32_t)(n) << GIC_CONFIG4_ID71_SHIFT)
#define GIC_CONFIG4_ID72_SHIFT     (16)      /* Bits 16-17: ID 72 configuration */
#define GIC_CONFIG4_ID72_MASK      (3 << GIC_CONFIG4_ID72_SHIFT)
#  define GIC_CONFIG4_ID72(n)      ((uint32_t)(n) << GIC_CONFIG4_ID72_SHIFT)
#define GIC_CONFIG4_ID73_SHIFT     (18)      /* Bits 18-19: ID 73 configuration */
#define GIC_CONFIG4_ID73_MASK      (3 << GIC_CONFIG4_ID73_SHIFT)
#  define GIC_CONFIG4_ID73(n)      ((uint32_t)(n) << GIC_CONFIG4_ID73_SHIFT)
#define GIC_CONFIG4_ID74_SHIFT     (20)      /* Bits 20-21: ID 74 configuration */
#define GIC_CONFIG4_ID74_MASK      (3 << GIC_CONFIG4_ID74_SHIFT)
#  define GIC_CONFIG4_ID74(n)      ((uint32_t)(n) << GIC_CONFIG4_ID74_SHIFT)
#define GIC_CONFIG4_ID75_SHIFT     (22)      /* Bits 22-23: ID 75 configuration */
#define GIC_CONFIG4_ID75_MASK      (3 << GIC_CONFIG4_ID75_SHIFT)
#  define GIC_CONFIG4_ID75(n)      ((uint32_t)(n) << GIC_CONFIG4_ID75_SHIFT)
#define GIC_CONFIG4_ID76_SHIFT     (24)      /* Bits 24-25: ID 76 configuration */
#define GIC_CONFIG4_ID76_MASK      (3 << GIC_CONFIG4_ID76_SHIFT)
#  define GIC_CONFIG4_ID76(n)      ((uint32_t)(n) << GIC_CONFIG4_ID76_SHIFT)
#define GIC_CONFIG4_ID77_SHIFT     (26)      /* Bits 26-27: ID 77 configuration */
#define GIC_CONFIG4_ID77_MASK      (3 << GIC_CONFIG4_ID77_SHIFT)
#  define GIC_CONFIG4_ID77(n)      ((uint32_t)(n) << GIC_CONFIG4_ID77_SHIFT)
#define GIC_CONFIG4_ID78_SHIFT     (28)      /* Bits 28-29: ID 78 configuration */
#define GIC_CONFIG4_ID78_MASK      (3 << GIC_CONFIG4_ID78_SHIFT)
#  define GIC_CONFIG4_ID78(n)      ((uint32_t)(n) << GIC_CONFIG4_ID78_SHIFT)
#define GIC_CONFIG4_ID79_SHIFT     (30)      /* Bits 30-31: ID 79 configuration */
#define GIC_CONFIG4_ID79_MASK      (3 << GIC_CONFIG4_ID79_SHIFT)
#  define GIC_CONFIG4_ID79(n)      ((uint32_t)(n) << GIC_CONFIG4_ID79_SHIFT)

#define GIC_CONFIG5_ID80_SHIFT     (0)       /* Bits 0-1: ID 80 configuration */
#define GIC_CONFIG5_ID80_MASK      (3 << GIC_CONFIG5_ID80_SHIFT)
#  define GIC_CONFIG5_ID80(n)      ((uint32_t)(n) << GIC_CONFIG5_ID80_SHIFT)
#define GIC_CONFIG5_ID81_SHIFT     (2)       /* Bits 2-3: ID 81 configuration */
#define GIC_CONFIG5_ID81_MASK      (3 << GIC_CONFIG5_ID81_SHIFT)
#  define GIC_CONFIG5_ID81(n)      ((uint32_t)(n) << GIC_CONFIG5_ID81_SHIFT)
#define GIC_CONFIG5_ID82_SHIFT     (4)       /* Bits 4-5: ID 82 configuration */
#define GIC_CONFIG5_ID82_MASK      (3 << GIC_CONFIG5_ID82_SHIFT)
#  define GIC_CONFIG5_ID82(n)      ((uint32_t)(n) << GIC_CONFIG5_ID82_SHIFT)
#define GIC_CONFIG5_ID83_SHIFT     (6)       /* Bits 6-7: ID 83 configuration */
#define GIC_CONFIG5_ID83_MASK      (3 << GIC_CONFIG5_ID83_SHIFT)
#  define GIC_CONFIG5_ID83(n)      ((uint32_t)(n) << GIC_CONFIG5_ID83_SHIFT)
#define GIC_CONFIG5_ID84_SHIFT     (8)       /* Bits 8-9: ID 84 configuration */
#define GIC_CONFIG5_ID84_MASK      (3 << GIC_CONFIG5_ID84_SHIFT)
#  define GIC_CONFIG5_ID84(n)      ((uint32_t)(n) << GIC_CONFIG5_ID84_SHIFT)
#define GIC_CONFIG5_ID85_SHIFT     (10)      /* Bits 10-11: ID 85 configuration */
#define GIC_CONFIG5_ID85_MASK      (3 << GIC_CONFIG5_ID85_SHIFT)
#  define GIC_CONFIG5_ID85(n)      ((uint32_t)(n) << GIC_CONFIG5_ID85_SHIFT)
#define GIC_CONFIG5_ID86_SHIFT     (12)      /* Bits 12-13: ID 86 configuration */
#define GIC_CONFIG5_ID86_MASK      (3 << GIC_CONFIG5_ID86_SHIFT)
#  define GIC_CONFIG5_ID86(n)      ((uint32_t)(n) << GIC_CONFIG5_ID86_SHIFT)
#define GIC_CONFIG5_ID87_SHIFT     (14)      /* Bits 14-15: ID 87 configuration */
#define GIC_CONFIG5_ID87_MASK      (3 << GIC_CONFIG5_ID87_SHIFT)
#  define GIC_CONFIG5_ID87(n)      ((uint32_t)(n) << GIC_CONFIG5_ID87_SHIFT)
#define GIC_CONFIG5_ID88_SHIFT     (16)      /* Bits 16-17: ID 88 configuration */
#define GIC_CONFIG5_ID88_MASK      (3 << GIC_CONFIG5_ID88_SHIFT)
#  define GIC_CONFIG5_ID88(n)      ((uint32_t)(n) << GIC_CONFIG5_ID88_SHIFT)
#define GIC_CONFIG5_ID89_SHIFT     (18)      /* Bits 18-19: ID 89 configuration */
#define GIC_CONFIG5_ID89_MASK      (3 << GIC_CONFIG5_ID89_SHIFT)
#  define GIC_CONFIG5_ID89(n)      ((uint32_t)(n) << GIC_CONFIG5_ID89_SHIFT)
#define GIC_CONFIG5_ID90_SHIFT     (20)      /* Bits 20-21: ID 90 configuration */
#define GIC_CONFIG5_ID90_MASK      (3 << GIC_CONFIG5_ID90_SHIFT)
#  define GIC_CONFIG5_ID90(n)      ((uint32_t)(n) << GIC_CONFIG5_ID90_SHIFT)
#define GIC_CONFIG5_ID91_SHIFT     (22)      /* Bits 22-23: ID 91 configuration */
#define GIC_CONFIG5_ID91_MASK      (3 << GIC_CONFIG5_ID91_SHIFT)
#  define GIC_CONFIG5_ID91(n)      ((uint32_t)(n) << GIC_CONFIG5_ID91_SHIFT)
#define GIC_CONFIG5_ID92_SHIFT     (24)      /* Bits 24-25: ID 92 configuration */
#define GIC_CONFIG5_ID92_MASK      (3 << GIC_CONFIG5_ID92_SHIFT)
#  define GIC_CONFIG5_ID92(n)      ((uint32_t)(n) << GIC_CONFIG5_ID92_SHIFT)
#define GIC_CONFIG5_ID93_SHIFT     (26)      /* Bits 26-27: ID 93 configuration */
#define GIC_CONFIG5_ID93_MASK      (3 << GIC_CONFIG5_ID93_SHIFT)
#  define GIC_CONFIG5_ID93(n)      ((uint32_t)(n) << GIC_CONFIG5_ID93_SHIFT)
#define GIC_CONFIG5_ID94_SHIFT     (28)      /* Bits 28-29: ID 94 configuration */
#define GIC_CONFIG5_ID94_MASK      (3 << GIC_CONFIG5_ID94_SHIFT)
#  define GIC_CONFIG5_ID94(n)      ((uint32_t)(n) << GIC_CONFIG5_ID94_SHIFT)
#define GIC_CONFIG5_ID95_SHIFT     (30)      /* Bits 30-31: ID 95 configuration */
#define GIC_CONFIG5_ID95_MASK      (3 << GIC_CONFIG5_ID95_SHIFT)
#  define GIC_CONFIG5_ID95(n)      ((uint32_t)(n) << GIC_CONFIG5_ID95_SHIFT)

/* Software Interrupt Register */

#define GIC_SWINT_INTID_SHIFT      (0)       /* Bits 0-9: Interrupt ID */
#define GIC_SWINT_INTID_MASK       (0x3ff << GIC_SWINT_INTID_SHIFT)
#  define GIC_SWINT_INTID(n)       ((uint32_t)(n) << GIC_SWINT_INTID_SHIFT)
                                             /* Bits 10-15: Reserved */
#define GIC_SWINT_CPUTARGET_SHIFT  (16)      /* Bits 16-23: CPU target */
#define GIC_SWINT_CPUTARGET_MASK   (0xff << GIC_SWINT_CPUTARGET_SHIFT)
#  define GIC_SWINT_CPUTARGET(n)   ((uint32_t)(n) << GIC_SWINT_CPUTARGET_SHIFT)
                                             /* Bits 26-31: Reserved */
#define GIC_SWINT_TGTFILTER_SHIFT  (24)      /* Bits 24-25: Target filter */
#define GIC_SWINT_TGTFILTER_MASK   (3 << GIC_SWINT_TGTFILTER_SHIFT)
#  define GIC_SWINT_TGTFILTER_LIST   (0 << GIC_SWINT_TGTFILTER_SHIFT) /* Interrupt sent to CPUs CPU target list */
#  define GIC_SWINT_TGTFILTER_OTHER  (1 << GIC_SWINT_TGTFILTER_SHIFT) /* Interrupt is sent to all but requesting CPU */
#  define GIC_SWINT_TGTFILTER_THIS   (2 << GIC_SWINT_TGTFILTER_SHIFT) /* Interrupt is sent to requesting CPU only */

#endif /* __ARCH_ARM_SRC_ARMV7_A_GIC_H */
