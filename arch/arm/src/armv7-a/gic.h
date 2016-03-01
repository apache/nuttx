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
/* Interrupt Interface registers */

#define GIC_ICCICR_OFFSET          0x0000    /* CPU Interface Control Register */
#define GIC_ICCPMR_OFFSET          0x0004    /* Interrupt Priority Mask Register */
#define GIC_ICCBPR_OFFSET          0x0008    /* Binary point Register */
#define GIC_ICCIAR_OFFSET          0x000c    /* Interrupt Acknowledge */
#define GIC_ICCEOIR_OFFSET         0x0010    /* End of interrupt */
#define GIC_ICCRPR_OFFSET          0x0014    /* Running interrupt */
#define GIC_ICCHPIR_OFFSET         0x0018    /* Highest pending interrupt */
#define GIC_ICCABPR_OFFSET         0x001c    /* Aliased Non-secure Binary Point Register */
#define GIC_ICCIDR_OFFSET          0x00fc    /* CPU Interface Implementer ID Register */

/* Distributor Registers */

#define GIC_ICDDCR_OFFSET          0x0000    /* Distributor Control Register */
#define GIC_ICDICTR_OFFSET         0x0004    /* Interrupt Controller Type Register */
#define GIC_ICDIIDR_OFFSET         0x0008    /* Distributor Implementer ID Register */
                                             /* 0x000c-0x007c: Reserved */
#define GIC_ICDISR_OFFSET(n)
#  define GIC_ICDISR0_OFFSET       0x0080    /* Interrupt Security Register 0 */
#  define GIC_ICDISR1_OFFSET       0x0084    /* Interrupt Security Register 1 */
#  define GIC_ICDISR2_OFFSET       0x0088    /* Interrupt Security Register 2 */
#  define GIC_ICDISR3_OFFSET       0x008c    /* Interrupt Security Register 3 */
#  define GIC_ICDISR4_OFFSET       0x0090    /* Interrupt Security Register 4 */
#  define GIC_ICDISR5_OFFSET       0x0094    /* Interrupt Security Register 5 */
#  define GIC_ICDISR6_OFFSET       0x0095    /* Interrupt Security Register 6 */
#  define GIC_ICDISR7_OFFSET       0x009c    /* Interrupt Security Register 7 */
                                             /* 0x000c-0x00fc: Reserved */
#define GIC_ICDISER_OFFSET(n)      (0x0100 + ((n) >> 3) & ~3)
#  define GIC_ICDISER0_OFFSET      0x0100    /* Interrupt Set-enable0 */
#  define GIC_ICDISER1_OFFSET      0x0104    /* Interrupt Set-enable1 */
#  define GIC_ICDISER2_OFFSET      0x0108    /* Interrupt Set-enable2 */
                                             /* 0x010c-0x017c: Reserved */
#define GIC_ICDICER_OFFSET(n)      (0x0180 + ((n) >> 3) & ~3)
#  define GIC_ICDICER0_OFFSET      0x0180    /* Interrupt Clear-enable0 */
#  define GIC_ICDICER1_OFFSET      0x0184    /* Interrupt Clear-enable1 */
#  define GIC_ICDICER2_OFFSET      0x0188    /* Interrupt Clear-enable2 */
                                             /* 0x018c-0x01fc: Reserved */
#define GIC_ICDISPR_OFFSET(n)      (0x0200 + ((n) >> 3) & ~3)
#  define GIC_ICDISPR0_OFFSET      0x0200    /* Interrupt Set-pending0 */
#  define GIC_ICDISPR1_OFFSET      0x0204    /* Interrupt Set-pending1 */
#  define GIC_ICDISPR2_OFFSET      0x0208    /* Interrupt Set-pending2 */
                                             /* 0x020c-0x027c: Reserved */
#define GIC_ICDICPR_OFFSET(n)      (0x0280 + ((n) >> 3) & ~3)
#  define GIC_ICDICPR0_OFFSET      0x0280    /* Interrupt Clear-pending0 */
#  define GIC_ICDICPR1_OFFSET      0x0284    /* Interrupt Clear-pending1 */
#  define GIC_ICDICPR2_OFFSET      0x0288    /* Interrupt Clear-pending2 */
                                             /* 0x028c-0x02fc: Reserved */
#define GIC_ICDABR_OFFSET(n)       (0x0300 + ((n) >> 3) & ~3)
#  define GIC_ICDABR0_OFFSET       0x0300    /* Interrupt Active Bit0 */
#  define GIC_ICDABR1_OFFSET       0x0304    /* Interrupt Active Bit1 */
#  define GIC_ICDABR2_OFFSET       0x0308    /* Interrupt Active Bit2 */
                                             /* 0x030c-0x01fc: Reserved */
#define GIC_ICDIPTR_OFFSET(n)      (0x400 + ((n) ~3))
#  define GIC_ICDIPTR8_OFFSET      0x0420    /* Interrupt Priority Register8  ID32-ID35 */
#  define GIC_ICDIPTR9_OFFSET      0x0424    /* Interrupt Priority Register9  ID36-ID39 */
#  define GIC_ICDIPTR10_OFFSET     0x0428    /* Interrupt Priority Register10 ID40-ID43 */
#  define GIC_ICDIPTR11_OFFSET     0x042c    /* Interrupt Priority Register11 ID44-ID47 */
#  define GIC_ICDIPTR12_OFFSET     0x0430    /* Interrupt Priority Register12 ID48-ID51 */
#  define GIC_ICDIPTR13_OFFSET     0x0434    /* Interrupt Priority Register13 ID52-ID55 */
#  define GIC_ICDIPTR14_OFFSET     0x0438    /* Interrupt Priority Register14 ID56-ID59 */
#  define GIC_ICDIPTR15_OFFSET     0x043c    /* Interrupt Priority Register15 ID60-ID63 */
#  define GIC_ICDIPTR16_OFFSET     0x0440    /* Interrupt Priority Register16 ID64-ID67 */
#  define GIC_ICDIPTR17_OFFSET     0x0444    /* Interrupt Priority Register17 ID68-ID71 */
#  define GIC_ICDIPTR18_OFFSET     0x0448    /* Interrupt Priority Register18 ID72-ID75 */
#  define GIC_ICDIPTR19_OFFSET     0x044c    /* Interrupt Priority Register19 ID76-ID79 */
#  define GIC_ICDIPTR20_OFFSET     0x0450    /* Interrupt Priority Register20 ID80-ID83 */
#  define GIC_ICDIPTR21_OFFSET     0x0454    /* Interrupt Priority Register21 ID84-ID87 */
#  define GIC_ICDIPTR22_OFFSET     0x0458    /* Interrupt Priority Register22 ID88-ID91 */
#  define GIC_ICDIPTR23_OFFSET     0x045c    /* Interrupt Priority Register23 ID92-ID95 */
                                             /* 0x0460-0x07fc: Reserved */
#define GIC_ICDICTR_OFFSET(n)      (0x800 + ((n) >> ~3))
#  define GIC_ICDICTR8_OFFSET      0x0820    /* Interrupt CPU Target Register8  ID32-ID35 */
#  define GIC_ICDICTR9_OFFSET      0x0824    /* Interrupt CPU Target Register9  ID36-ID39 */
#  define GIC_ICDICTR10_OFFSET     0x0828    /* Interrupt CPU Target Register10 ID40-ID43 */
#  define GIC_ICDICTR11_OFFSET     0x082c    /* Interrupt CPU Target Register11 ID44-ID47 */
#  define GIC_ICDICTR12_OFFSET     0x0830    /* Interrupt CPU Target Register12 ID48-ID51 */
#  define GIC_ICDICTR13_OFFSET     0x0834    /* Interrupt CPU Target Register13 ID52-ID55 */
#  define GIC_ICDICTR14_OFFSET     0x0838    /* Interrupt CPU Target Register14 ID56-ID59 */
#  define GIC_ICDICTR15_OFFSET     0x083c    /* Interrupt CPU Target Register15 ID60-ID63 */
#  define GIC_ICDICTR16_OFFSET     0x0840    /* Interrupt CPU Target Register16 ID64-ID67 */
#  define GIC_ICDICTR17_OFFSET     0x0844    /* Interrupt CPU Target Register17 ID68-ID71 */
#  define GIC_ICDICTR18_OFFSET     0x0848    /* Interrupt CPU Target Register18 ID72-ID75 */
#  define GIC_ICDICTR19_OFFSET     0x084c    /* Interrupt CPU Target Register19 ID76-ID79 */
#  define GIC_ICDICTR20_OFFSET     0x0850    /* Interrupt CPU Target Register20 ID80-ID83 */
#  define GIC_ICDICTR21_OFFSET     0x0854    /* Interrupt CPU Target Register21 ID84-ID87 */
#  define GIC_ICDICTR22_OFFSET     0x0858    /* Interrupt CPU Target Register22 ID88-ID91 */
#  define GIC_ICDICTR23_OFFSET     0x085c    /* Interrupt CPU Target Register23 ID92-ID95 */
                                             /* 0x0860-0x0bfc: Reserved */
#define GIC_ICDICFR2_OFFSET(n)     (0x0c00 + ((n) >> 2) & ~3)
#  define GIC_ICDICFR2_OFFSET      0x0c08    /* Interrupt Configuration Register2 ID32-ID47 */
#  define GIC_ICDICFR3_OFFSET      0x0c0c    /* Interrupt Configuration Register3 ID48-ID63 */
#  define GIC_ICDICFR4_OFFSET      0x0c10    /* Interrupt Configuration Register4 ID64-ID79 */
#  define GIC_ICDICFR5_OFFSET      0x0c14    /* Interrupt Configuration Register5 ID80-ID95 */
                                             /* 0x0c18-0x0efc: Reserved */
#define GIC_ICDPPISR_OFFSET        0x0d00    /* PPI Status Register */
#define GIC_ICDPPSSR_OFFSET(n)     (0x0d04 + ((n) >> 3) & ~3)
#  define GIC_ICDPPSSR0_OFFSET     0x0d04    /* SPI Status Register0 ID0-ID31 */
#  define GIC_ICDPPSSR1_OFFSET     0x0d08    /* SPI Status Register1 ID32-ID63 */
#  define GIC_ICDPPSSR2_OFFSET     0x0d0c    /* SPI Status Register2 ID64-ID95 */
#  define GIC_ICDPPSSR3_OFFSET     0x0d10    /* SPI Status Register3 ID96-ID127 */
#  define GIC_ICDPPSSR4_OFFSET     0x0d14    /* SPI Status Register4 ID128-ID159 */
#  define GIC_ICDPPSSR5_OFFSET     0x0d18    /* SPI Status Register5 ID160-ID191 */
#  define GIC_ICDPPSSR6_OFFSET     0x0d1c    /* SPI Status Register6 ID192-ID223 */
#define GIC_ICDSGIR_OFFSET         0x0f00    /* Software Generated Interrupt Register */
                                             /* 0x0f04-0x0ffc: Reserved */

/* GIC Register Addresses ***************************************************/
/* Interrupt Interface registers */

#define GIC2_ICCICR                (GIC2_CPU_PBASE+GIC_ICCICR_OFFSET)
#define GIC2_ICCPMR                (GIC2_CPU_PBASE+GIC_ICCPMR_OFFSET)
#define GIC2_ICCBPR                (GIC2_CPU_PBASE+GIC_ICCBPR_OFFSET)
#define GIC2_ICCIAR                (GIC2_CPU_PBASE+GIC_ICCIAR_OFFSET)
#define GIC2_ICCEOIR               (GIC2_CPU_PBASE+GIC_ICCEOIR_OFFSET)
#define GIC2_ICCRPR                (GIC2_CPU_PBASE+GIC_ICCRPR_OFFSET)
#define GIC2_ICCHPIR               (GIC2_CPU_PBASE+GIC_ICCHPIR_OFFSET)
#define GIC2_ICCABPR               (GIC2_CPU_PBASE+GIC_ICCABPR_OFFSET)
#define GIC2_ICCIDR                (GIC2_CPU_PBASE+GIC_ICCIDR_OFFSET_

/* Distributor Registers */

#define GIC2_ICDDCR                (GIC2_DISTR_PBASE+GIC_ICDDCR_OFFSET)
#define GIC2_ICDICTR               (GIC2_DISTR_PBASE+GIC_ICDICTR_OFFSET)
#define GIC2_ICDIIDR               (GIC2_DISTR_PBASE+GIC_ICDIIDR_OFFSET)
#define GIC2_ICDISER(n)            (GIC2_DISTR_PBASE+GIC_ICDISER_OFFSET(n))
#  define GIC2_ICDISER0            (GIC2_DISTR_PBASE+GIC_ICDISER0_OFFSET)
#  define GIC2_ICDISER1            (GIC2_DISTR_PBASE+GIC_ICDISER1_OFFSET)
#  define GIC2_ICDISER2            (GIC2_DISTR_PBASE+GIC_ICDISER2_OFFSET)
#define GIC2_ICDICER(n)            (GIC2_DISTR_PBASE+GIC_ICDICER_OFFSET(n))
#  define GIC2_ICDICER0            (GIC2_DISTR_PBASE+GIC_ICDICER0_OFFSET)
#  define GIC2_ICDICER1            (GIC2_DISTR_PBASE+GIC_ICDICER1_OFFSET)
#  define GIC2_ICDICER2            (GIC2_DISTR_PBASE+GIC_ICDICER2_OFFSET)
#define GIC2_ICDISPR(n)            (GIC2_DISTR_PBASE+GIC_ICDISPR_OFFSET(n))
#  define GIC2_ICDISPR0            (GIC2_DISTR_PBASE+GIC_ICDISPR0_OFFSET)
#  define GIC2_ICDISPR1            (GIC2_DISTR_PBASE+GIC_ICDISPR1_OFFSET)
#  define GIC2_ICDISPR2            (GIC2_DISTR_PBASE+GIC_ICDISPR2_OFFSET)
#define GIC2_ICDICPR(n)            (GIC2_DISTR_PBASE+GIC_ICDICPR_OFFSET(n))
#  define GIC2_ICDICPR0            (GIC2_DISTR_PBASE+GIC_ICDICPR0_OFFSET)
#  define GIC2_ICDICPR1            (GIC2_DISTR_PBASE+GIC_ICDICPR1_OFFSET)
#  define GIC2_ICDICPR2            (GIC2_DISTR_PBASE+GIC_ICDICPR2_OFFSET)
#define GIC2_ICDABR(n)             (GIC2_DISTR_PBASE+GIC_ICDABR_OFFSET(n))
#  define GIC2_ICDABR0             (GIC2_DISTR_PBASE+GIC_ICDABR0_OFFSET)
#  define GIC2_ICDABR1             (GIC2_DISTR_PBASE+GIC_ICDABR1_OFFSET)
#  define GIC2_ICDABR2             (GIC2_DISTR_PBASE+GIC_ICDABR2_OFFSET)
#define GIC2_ICDIPTR(n)            (GIC2_DISTR_PBASE+GIC_ICDIPTR_OFFSET(n))
#  define GIC2_ICDIPTR8            (GIC2_DISTR_PBASE+GIC_ICDIPTR8_OFFSET)
#  define GIC2_ICDIPTR9            (GIC2_DISTR_PBASE+GIC_ICDIPTR9_OFFSET)
#  define GIC2_ICDIPTR10           (GIC2_DISTR_PBASE+GIC_ICDIPTR10_OFFSET)
#  define GIC2_ICDIPTR11           (GIC2_DISTR_PBASE+GIC_ICDIPTR11_OFFSET)
#  define GIC2_ICDIPTR12           (GIC2_DISTR_PBASE+GIC_ICDIPTR12_OFFSET)
#  define GIC2_ICDIPTR13           (GIC2_DISTR_PBASE+GIC_ICDIPTR13_OFFSET)
#  define GIC2_ICDIPTR14           (GIC2_DISTR_PBASE+GIC_ICDIPTR14_OFFSET)
#  define GIC2_ICDIPTR15           (GIC2_DISTR_PBASE+GIC_ICDIPTR15_OFFSET)
#  define GIC2_ICDIPTR16           (GIC2_DISTR_PBASE+GIC_ICDIPTR16_OFFSET)
#  define GIC2_ICDIPTR17           (GIC2_DISTR_PBASE+GIC_ICDIPTR17_OFFSET)
#  define GIC2_ICDIPTR18           (GIC2_DISTR_PBASE+GIC_ICDIPTR18_OFFSET)
#  define GIC2_ICDIPTR19           (GIC2_DISTR_PBASE+GIC_ICDIPTR19_OFFSET)
#  define GIC2_ICDIPTR20           (GIC2_DISTR_PBASE+GIC_ICDIPTR20_OFFSET)
#  define GIC2_ICDIPTR21           (GIC2_DISTR_PBASE+GIC_ICDIPTR21_OFFSET)
#  define GIC2_ICDIPTR22           (GIC2_DISTR_PBASE+GIC_ICDIPTR22_OFFSET)
#  define GIC2_ICDIPTR23           (GIC2_DISTR_PBASE+GIC_ICDIPTR23_OFFSET)
#define GIC2_ICDICTGR(n)           (GIC2_DISTR_PBASE+GIC_ICDICTGR_OFFSET(n))
#  define GIC2_ICDICTGR8           (GIC2_DISTR_PBASE+GIC_ICDICTGR8_OFFSET)
#  define GIC2_ICDICTGR9           (GIC2_DISTR_PBASE+GIC_ICDICTGR9_OFFSET)
#  define GIC2_ICDICTGR10          (GIC2_DISTR_PBASE+GIC_ICDICTGR10_OFFSET)
#  define GIC2_ICDICTGR11          (GIC2_DISTR_PBASE+GIC_ICDICTGR11_OFFSET)
#  define GIC2_ICDICTGR12          (GIC2_DISTR_PBASE+GIC_ICDICTGR12_OFFSET)
#  define GIC2_ICDICTGR13          (GIC2_DISTR_PBASE+GIC_ICDICTGR13_OFFSET)
#  define GIC2_ICDICTGR14          (GIC2_DISTR_PBASE+GIC_ICDICTGR14_OFFSET)
#  define GIC2_ICDICTGR15          (GIC2_DISTR_PBASE+GIC_ICDICTGR15_OFFSET)
#  define GIC2_ICDICTGR16          (GIC2_DISTR_PBASE+GIC_ICDICTGR16_OFFSET)
#  define GIC2_ICDICTGR17          (GIC2_DISTR_PBASE+GIC_ICDICTGR17_OFFSET)
#  define GIC2_ICDICTGR18          (GIC2_DISTR_PBASE+GIC_ICDICTGR18_OFFSET)
#  define GIC2_ICDICTGR19          (GIC2_DISTR_PBASE+GIC_ICDICTGR19_OFFSET)
#  define GIC2_ICDICTGR20          (GIC2_DISTR_PBASE+GIC_ICDICTGR20_OFFSET)
#  define GIC2_ICDICTGR21          (GIC2_DISTR_PBASE+GIC_ICDICTGR21_OFFSET)
#  define GIC2_ICDICTGR22          (GIC2_DISTR_PBASE+GIC_ICDICTGR22_OFFSET)
#  define GIC2_ICDICTGR23          (GIC2_DISTR_PBASE+GIC_ICDICTGR23_OFFSET)
#define GIC2_ICDICFR2(n)           (GIC2_DISTR_PBASE+GIC_ICDICFR2_OFFSET(n))
#  define GIC2_ICDICFR2            (GIC2_DISTR_PBASE+GIC_ICDICFR2_OFFSET)
#  define GIC2_ICDICFR3            (GIC2_DISTR_PBASE+GIC_ICDICFR3_OFFSET)
#  define GIC2_ICDICFR4            (GIC2_DISTR_PBASE+GIC_ICDICFR4_OFFSET)
#  define GIC2_ICDICFR5            (GIC2_DISTR_PBASE+GIC_ICDICFR5_OFFSET)
#define GIC2_ICDPPSSR(n)           (GIC2_DISTR_PBASE+GIC_ICDPPSSR_OFFSET(n))
#  define GIC2_ICDPPSSR0           (GIC2_DISTR_PBASE+GIC_ICDPPSSR0_OFFSET)
#  define GIC2_ICDPPSSR1           (GIC2_DISTR_PBASE+GIC_ICDPPSSR1_OFFSET)
#  define GIC2_ICDPPSSR2           (GIC2_DISTR_PBASE+GIC_ICDPPSSR2_OFFSET)
#  define GIC2_ICDPPSSR3           (GIC2_DISTR_PBASE+GIC_ICDPPSSR3_OFFSET)
#  define GIC2_ICDPPSSR4           (GIC2_DISTR_PBASE+GIC_ICDPPSSR4_OFFSET)
#  define GIC2_ICDPPSSR5           (GIC2_DISTR_PBASE+GIC_ICDPPSSR5_OFFSET)
#  define GIC2_ICDPPSSR6           (GIC2_DISTR_PBASE+GIC_ICDPPSSR6_OFFSET)
#define GIC2_ICDSGIR               (GIC2_DISTR_PBASE+GIC_ICDSGIR_OFFSET)

/* Interupt Interface registers */

#define GIC3_ICCICR                (GIC3_CPU_PBASE+GIC_ICCICR_OFFSET)
#define GIC3_ICCPMR                (GIC3_CPU_PBASE+GIC_ICCPMR_OFFSET)
#define GIC3_ICCBPR                (GIC3_CPU_PBASE+GIC_ICCBPR_OFFSET)
#define GIC3_ICCIAR                (GIC3_CPU_PBASE+GIC_ICCIAR_OFFSET)
#define GIC3_ICCEOIR               (GIC3_CPU_PBASE+GIC_ICCEOIR_OFFSET)
#define GIC3_ICCRPR                (GIC3_CPU_PBASE+GIC_ICCRPR_OFFSET)
#define GIC3_ICCHPIR               (GIC3_CPU_PBASE+GIC_ICCHPIR_OFFSET)
#define GIC3_ICCABPR               (GIC3_CPU_PBASE+GIC_ICCABPR_OFFSET)
#define GIC3_ICCIDR                (GIC3_CPU_PBASE+GIC_ICCIDR_OFFSET_

/* Distributor Registers */

#define GIC3_ICDDCR                (GIC3_DISTR_PBASE+GIC_ICDDCR_OFFSET)
#define GIC3_ICDICTR               (GIC3_DISTR_PBASE+GIC_ICDICTR_OFFSET)
#define GIC3_ICDIIDR               (GIC3_DISTR_PBASE+GIC_ICDIIDR_OFFSET)
#define GIC3_ICDISER(n)            (GIC3_DISTR_PBASE+GIC_ICDISER_OFFSET(n))
#  define GIC3_ICDISER0            (GIC3_DISTR_PBASE+GIC_ICDISER0_OFFSET)
#  define GIC3_ICDISER1            (GIC3_DISTR_PBASE+GIC_ICDISER1_OFFSET)
#  define GIC3_ICDISER2            (GIC3_DISTR_PBASE+GIC_ICDISER2_OFFSET)
#define GIC3_ICDICER(n)            (GIC3_DISTR_PBASE+GIC_ICDICER_OFFSET(n))
#  define GIC3_ICDICER0            (GIC3_DISTR_PBASE+GIC_ICDICER0_OFFSET)
#  define GIC3_ICDICER1            (GIC3_DISTR_PBASE+GIC_ICDICER1_OFFSET)
#  define GIC3_ICDICER2            (GIC3_DISTR_PBASE+GIC_ICDICER2_OFFSET)
#define GIC3_ICDISPR(n)            (GIC3_DISTR_PBASE+GIC_ICDISPR_OFFSET(n))
#  define GIC3_ICDISPR0            (GIC3_DISTR_PBASE+GIC_ICDISPR0_OFFSET)
#  define GIC3_ICDISPR1            (GIC3_DISTR_PBASE+GIC_ICDISPR1_OFFSET)
#  define GIC3_ICDISPR2            (GIC3_DISTR_PBASE+GIC_ICDISPR2_OFFSET)
#define GIC3_ICDICPR(n)            (GIC3_DISTR_PBASE+GIC_ICDICPR_OFFSET(n))
#  define GIC3_ICDICPR0            (GIC3_DISTR_PBASE+GIC_ICDICPR0_OFFSET)
#  define GIC3_ICDICPR1            (GIC3_DISTR_PBASE+GIC_ICDICPR1_OFFSET)
#  define GIC3_ICDICPR2            (GIC3_DISTR_PBASE+GIC_ICDICPR2_OFFSET)
#define GIC3_ICDABR(n)             (GIC3_DISTR_PBASE+GIC_ICDABR_OFFSET(n))
#  define GIC3_ICDABR0             (GIC3_DISTR_PBASE+GIC_ICDABR0_OFFSET)
#  define GIC3_ICDABR1             (GIC3_DISTR_PBASE+GIC_ICDABR1_OFFSET)
#  define GIC3_ICDABR2             (GIC3_DISTR_PBASE+GIC_ICDABR2_OFFSET)
#define GIC3_ICDIPTR(n)            (GIC3_DISTR_PBASE+GIC_ICDIPTR_OFFSET(n))
#  define GIC3_ICDIPTR8            (GIC3_DISTR_PBASE+GIC_ICDIPTR8_OFFSET)
#  define GIC3_ICDIPTR9            (GIC3_DISTR_PBASE+GIC_ICDIPTR9_OFFSET)
#  define GIC3_ICDIPTR10           (GIC3_DISTR_PBASE+GIC_ICDIPTR10_OFFSET)
#  define GIC3_ICDIPTR11           (GIC3_DISTR_PBASE+GIC_ICDIPTR11_OFFSET)
#  define GIC3_ICDIPTR12           (GIC3_DISTR_PBASE+GIC_ICDIPTR12_OFFSET)
#  define GIC3_ICDIPTR13           (GIC3_DISTR_PBASE+GIC_ICDIPTR13_OFFSET)
#  define GIC3_ICDIPTR14           (GIC3_DISTR_PBASE+GIC_ICDIPTR14_OFFSET)
#  define GIC3_ICDIPTR15           (GIC3_DISTR_PBASE+GIC_ICDIPTR15_OFFSET)
#  define GIC3_ICDIPTR16           (GIC3_DISTR_PBASE+GIC_ICDIPTR16_OFFSET)
#  define GIC3_ICDIPTR17           (GIC3_DISTR_PBASE+GIC_ICDIPTR17_OFFSET)
#  define GIC3_ICDIPTR18           (GIC3_DISTR_PBASE+GIC_ICDIPTR18_OFFSET)
#  define GIC3_ICDIPTR19           (GIC3_DISTR_PBASE+GIC_ICDIPTR19_OFFSET)
#  define GIC3_ICDIPTR20           (GIC3_DISTR_PBASE+GIC_ICDIPTR20_OFFSET)
#  define GIC3_ICDIPTR21           (GIC3_DISTR_PBASE+GIC_ICDIPTR21_OFFSET)
#  define GIC3_ICDIPTR22           (GIC3_DISTR_PBASE+GIC_ICDIPTR22_OFFSET)
#  define GIC3_ICDIPTR23           (GIC3_DISTR_PBASE+GIC_ICDIPTR23_OFFSET)
#define GIC3_ICDICTGR(n)           (GIC3_DISTR_PBASE+GIC_ICDICTGR_OFFSET(n))
#  define GIC3_ICDICTGR8           (GIC3_DISTR_PBASE+GIC_ICDICTGR8_OFFSET)
#  define GIC3_ICDICTGR9           (GIC3_DISTR_PBASE+GIC_ICDICTGR9_OFFSET)
#  define GIC3_ICDICTGR10          (GIC3_DISTR_PBASE+GIC_ICDICTGR10_OFFSET)
#  define GIC3_ICDICTGR11          (GIC3_DISTR_PBASE+GIC_ICDICTGR11_OFFSET)
#  define GIC3_ICDICTGR12          (GIC3_DISTR_PBASE+GIC_ICDICTGR12_OFFSET)
#  define GIC3_ICDICTGR13          (GIC3_DISTR_PBASE+GIC_ICDICTGR13_OFFSET)
#  define GIC3_ICDICTGR14          (GIC3_DISTR_PBASE+GIC_ICDICTGR14_OFFSET)
#  define GIC3_ICDICTGR15          (GIC3_DISTR_PBASE+GIC_ICDICTGR15_OFFSET)
#  define GIC3_ICDICTGR16          (GIC3_DISTR_PBASE+GIC_ICDICTGR16_OFFSET)
#  define GIC3_ICDICTGR17          (GIC3_DISTR_PBASE+GIC_ICDICTGR17_OFFSET)
#  define GIC3_ICDICTGR18          (GIC3_DISTR_PBASE+GIC_ICDICTGR18_OFFSET)
#  define GIC3_ICDICTGR19          (GIC3_DISTR_PBASE+GIC_ICDICTGR19_OFFSET)
#  define GIC3_ICDICTGR20          (GIC3_DISTR_PBASE+GIC_ICDICTGR20_OFFSET)
#  define GIC3_ICDICTGR21          (GIC3_DISTR_PBASE+GIC_ICDICTGR21_OFFSET)
#  define GIC3_ICDICTGR22          (GIC3_DISTR_PBASE+GIC_ICDICTGR22_OFFSET)
#  define GIC3_ICDICTGR23          (GIC3_DISTR_PBASE+GIC_ICDICTGR23_OFFSET)
#define GIC3_ICDICFR2(n)           (GIC3_DISTR_PBASE+GIC_ICDICFR2_OFFSET(n))
#  define GIC3_ICDICFR2            (GIC3_DISTR_PBASE+GIC_ICDICFR2_OFFSET)
#  define GIC3_ICDICFR3            (GIC3_DISTR_PBASE+GIC_ICDICFR3_OFFSET)
#  define GIC3_ICDICFR4            (GIC3_DISTR_PBASE+GIC_ICDICFR4_OFFSET)
#  define GIC3_ICDICFR5            (GIC3_DISTR_PBASE+GIC_ICDICFR5_OFFSET)
#define GIC3_ICDPPSSR(n)           (GIC3_DISTR_PBASE+GIC_ICDPPSSR_OFFSET(n))
#  define GIC3_ICDPPSSR0           (GIC3_DISTR_PBASE+GIC_ICDPPSSR0_OFFSET)
#  define GIC3_ICDPPSSR1           (GIC3_DISTR_PBASE+GIC_ICDPPSSR1_OFFSET)
#  define GIC3_ICDPPSSR2           (GIC3_DISTR_PBASE+GIC_ICDPPSSR2_OFFSET)
#  define GIC3_ICDPPSSR3           (GIC3_DISTR_PBASE+GIC_ICDPPSSR3_OFFSET)
#  define GIC3_ICDPPSSR4           (GIC3_DISTR_PBASE+GIC_ICDPPSSR4_OFFSET)
#  define GIC3_ICDPPSSR5           (GIC3_DISTR_PBASE+GIC_ICDPPSSR5_OFFSET)
#  define GIC3_ICDPPSSR6           (GIC3_DISTR_PBASE+GIC_ICDPPSSR6_OFFSET)
#define GIC3_ICDSGIR               (GIC3_DISTR_PBASE+GIC_ICDSGIR_OFFSET)

/* GIC Register Bit Definitions *********************************************/

/* Interrupt Interface registers */
/*  CPU Interface Control Register */

#define GIC_ICCICR_ENABLE          (1 << 0)  /* Bit 0: Enable the CPU interface for this GIC */
                                             /* Bits 1-31: Reserved */
/* Interrupt Priority Mask Register */
                                             /* Bits 0-3: Reserved */
#define GIC_ICCPMR_SHIFT           (4)       /* Bits 4-7: Priority mask */
#define GIC_ICCPMR_MASK            (15 << GIC_ICCPMR_SHIFT)
#  define GIC_ICCPMR_VALUE(n)      ((uint32_t)(n) << GIC_ICCPMR_SHIFT)
                                             /* Bits 8-31: Reserved */
/* Binary point Register */

#define GIC_ICCBPR_SHIFT           (0)       /* Bits 0-2: Binary point */
#define GIC_ICCBPR_MASK            (7 << GIC_ICCBPR_SHIFT)
#  define GIC_ICCBPR_ALL           (3 << GIC_ICCBPR_SHIFT) /* All priority bits are compared for pre-emption */
#  define GIC_ICCBPR_5_7           (4 << GIC_ICCBPR_SHIFT) /* Priority bits [7:5] compared for pre-emption */
#  define GIC_ICCBPR_6_7           (5 << GIC_ICCBPR_SHIFT) /* Priority bits [7:6] compared for pre-emption */
#  define GIC_ICCBPR_7             (6 << GIC_ICCBPR_SHIFT) /* Priority bit [7] compared for pre-emption */
#  define GIC_ICCBPR_NOPREMPT      (7 << GIC_ICCBPR_SHIFT) /* No pre-emption is performed */
                                             /* Bits 3-31: Reserved */
/* Interrupt Acknowledge Register */

#define GIC_ICCIAR_INTID_SHIFT     (0)       /* Bits 0-9: Interrupt ID */
#define GIC_ICCIAR_INTID_MASK      (0x3ff << GIC_ICCIAR_INTID_SHIFT)
#  define GIC_ICCIAR_INTID(n)      ((uint32_t)(n) << GIC_ICCIAR_INTID_SHIFT)
#define GIC_ICCIAR_CPUSRC_SHIFT    (10)      /* Bits 10-12: CPU source ID */
#define GIC_ICCIAR_CPUSRC_MASK     (7 << GIC_ICCIAR_CPUSRC_SHIFT)
#  define GIC_ICCIAR_CPUSRC(n)     ((uint32_t)(n) << GIC_ICCIAR_CPUSRC_SHIFT)
                                             /* Bits 13-31: Reserved */
/* End of Interrupt Register */

#define GIC_ICCEOIR_INTID_SHIFT    (0)       /* Bits 0-9: Interrupt ID */
#define GIC_ICCEOIR_INTID_MASK     (0x3ff << GIC_ICCEOIR_INTID_SHIFT)
#  define GIC_ICCEOIR_INTID(n)     ((uint32_t)(n) << GIC_ICCEOIR_INTID_SHIFT)
#define GIC_ICCEOIR_CPUSRC_SHIFT   (10)      /* Bits 10-12: CPU source ID */
#define GIC_ICCEOIR_CPUSRC_MASK    (7 << GIC_ICCEOIR_CPUSRC_SHIFT)
#  define GIC_ICCEOIR_CPUSRC(n)    ((uint32_t)(n) << GIC_ICCEOIR_CPUSRC_SHIFT)
                                             /* Bits 13-31: Reserved */
/* Running Interrupt Register */

                                             /* Bits 0-3: Reserved */
#define GIC_ICCRPR_PRIO_SHIFT      (4)       /* Bits 4-7: Priority mask */
#define GIC_ICCRPR_PRIO_MASK       (15 << GIC_ICCRPR_PRIO_SHIFT)
#  define GIC_ICCRPR_PRIO_VALUE(n) ((uint32_t)(n) << GIC_ICCRPR_PRIO_SHIFT)
                                             /* Bits 8-31: Reserved */
/* Highest Pending Interrupt Register */

#define GIC_ICCHPIR_INTID_SHIFT    (0)       /* Bits 0-9: Interrupt ID */
#define GIC_ICCHPIR_INTID_MASK     (0x3ff << GIC_ICCHPIR_INTID_SHIFT)
#  define GIC_ICCHPIR_INTID(n)     ((uint32_t)(n) << GIC_ICCHPIR_INTID_SHIFT)
#define GIC_ICCHPIR_CPUSRC_SHIFT   (10)      /* Bits 10-12: CPU source ID */
#define GIC_ICCHPIR_CPUSRC_MASK    (7 << GIC_ICCHPIR_CPUSRC_SHIFT)
#  define GIC_ICCHPIR_CPUSRC(n)    ((uint32_t)(n) << GIC_ICCHPIR_CPUSRC_SHIFT)
                                             /* Bits 13-31: Reserved */

/* Aliased Non-secure Binary Point Register */

#define GIC_ICCABPR_

/* CPU Interface Implementer ID Register */

#define GIC_ICCIDR_

/* Distributor Registers */
/* Distributor Control Register */

#define GIC_ICDDCR_ENABLE          (1 << 0)  /* Bit 0: Enable the CPU interface for this GIC */
                                             /* Bits 1-31: Reserved */
/* Interrupt Controller Type Register */

#define GIC_ICDICTR_IDLINES_SHIFT  (0)       /* Bits 0-4: ID lines number */
#define GIC_ICDICTR_IDLINES_MASK   (0x1f << GIC_ICDICTR_IDLINES_SHIFT)
#define GIC_ICDICTR_CPUNO_SHIFT    (5)       /* Bits 5-7: CPU number */
#define GIC_ICDICTR_CPUNO_MASK     (7 << GIC_ICDICTR_CPUNO_SHIFT)
                                             /* Bits 8-31: Reserved */
/* Distributor Implementer ID Register */

#define GIC_ICDIIDR_

/* Interrupt Set-Enable 0, 1, 2 */

#define GIC_ICDISER0_INT(n)        (1 << (n))      /* Bit n: Interrupt n enable, n=0-31 */
#define GIC_ICDISER1_INT(n)        (1 << ((n)-32)) /* Bit n: Interrupt n enable, n=32-63 */
#define GIC_ICDISER3_INT(n)        (1 << ((n)-64)) /* Bit n: Interrupt n enable, n=64-95 */

/* Interrupt Clear-Enable 0, 1, 2 */

#define GIC_ICDICER0_INT(n)        (1 << (n))      /* Bit n: Interrupt n clear, n=0-31 */
#define GIC_ICDICER1_INT(n)        (1 << ((n)-32)) /* Bit n: Interrupt n clear, n=32-63 */
#define GIC_ICDICER3_INT(n)        (1 << ((n)-64)) /* Bit n: Interrupt n clear, n=64-95 */

/* Interrupt Set-Pending 0, 1, 2 */

#define GIC_ICDISPR0_INT(n)        (1 << (n))      /* Bit n: Set interrupt n pending, n=0-31 */
#define GIC_ICDISPR1_INT(n)        (1 << ((n)-32)) /* Bit n: Set interrupt n pending, n=32-63 */
#define GIC_ICDISPR3_INT(n)        (1 << ((n)-64)) /* Bit n: Set interrupt n pending, n=64-95 */

/* Interrupt Clear-Pending 0, 1, 2 */

#define GIC_ICDICPR0_INT(n)        (1 << (n))      /* Bit n: Clear interrupt n pending, n=0-31 */
#define GIC_ICDICPR1_INT(n)        (1 << ((n)-32)) /* Bit n: Clear interrupt n pending, n=32-63 */
#define GIC_ICDICPR3_INT(n)        (1 << ((n)-64)) /* Bit n: Clear interrupt n pending, n=64-95 */

/* Interrupt Active Bit 0, 1, 2 */

#define GIC_ICDABR0_INT(n)         (1 << (n))      /* Bit n: Interrupt n active, n=0-31 */
#define GIC_ICDABR1_INT(n)         (1 << ((n)-32)) /* Bit n: Interrupt n active, n=32-63 */
#define GIC_ICDABR3_INT(n)         (1 << ((n)-64)) /* Bit n: Interrupt n active, n=64-95 */

/* Interrupt Priority Register 8..23 */

#define GIC_ICDIPTR_ID_SHIFT(n)    (((n) & 3) << 3)
#define GIC_ICDIPTR_ID_MASK(n)     (15 << GIC_ICDIPTR_ID_SHIFT(n))
#  define GIC_ICDIPTR_ID(n,p)      ((uint32_t)(p) << GIC_ICDIPTR_ID_SHIFT(n))

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR8_ID32_SHIFT    (0)       /* Bits 4-7: ID 32 priority */
#define GIC_ICDIPTR8_ID32_MASK     (15 << GIC_ICDIPTR8_ID32_SHIFT)
#  define GIC_ICDIPTR8_ID32(n)     ((uint32_t)(n) << GIC_ICDIPTR8_ID32_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR8_ID33_SHIFT    (12)      /* Bits 12-15: ID 33 priority */
#define GIC_ICDIPTR8_ID33_MASK     (15 << GIC_ICDIPTR8_ID33_SHIFT)
#  define GIC_ICDIPTR8_ID33(n)     ((uint32_t)(n) << GIC_ICDIPTR8_ID33_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR8_ID34_SHIFT    (20)      /* Bits 20-23: ID 34 priority */
#define GIC_ICDIPTR8_ID34_MASK     (15 << GIC_ICDIPTR8_ID34_SHIFT)
#  define GIC_ICDIPTR8_ID34(n)     ((uint32_t)(n) << GIC_ICDIPTR8_ID34_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR8_ID35_SHIFT    (28)      /* Bits 28-31: ID 35 priority */
#define GIC_ICDIPTR8_ID35_MASK     (15 << GIC_ICDIPTR8_ID35_SHIFT)
#  define GIC_ICDIPTR8_ID35(n)     ((uint32_t)(n) << GIC_ICDIPTR8_ID35_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR9_ID36_SHIFT    (0)       /* Bits 4-7: ID 36 priority */
#define GIC_ICDIPTR9_ID36_MASK     (15 << GIC_ICDIPTR9_ID36_SHIFT)
#  define GIC_ICDIPTR9_ID36(n)     ((uint32_t)(n) << GIC_ICDIPTR9_ID36_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR9_ID37_SHIFT    (12)      /* Bits 12-15: ID 37 priority */
#define GIC_ICDIPTR9_ID37_MASK     (15 << GIC_ICDIPTR9_ID37_SHIFT)
#  define GIC_ICDIPTR9_ID37(n)     ((uint32_t)(n) << GIC_ICDIPTR9_ID37_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR9_ID38_SHIFT    (20)      /* Bits 20-23: ID 38 priority */
#define GIC_ICDIPTR9_ID38_MASK     (15 << GIC_ICDIPTR9_ID38_SHIFT)
#  define GIC_ICDIPTR9_ID38(n)     ((uint32_t)(n) << GIC_ICDIPTR9_ID38_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR9_ID39_SHIFT    (28)      /* Bits 28-31: ID 39 priority */
#define GIC_ICDIPTR9_ID39_MASK     (15 << GIC_ICDIPTR9_ID39_SHIFT)
#  define GIC_ICDIPTR9_ID39(n)     ((uint32_t)(n) << GIC_ICDIPTR9_ID39_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR10_ID40_SHIFT   (0)       /* Bits 4-7: ID 40 priority */
#define GIC_ICDIPTR10_ID40_MASK    (15 << GIC_ICDIPTR10_ID40_SHIFT)
#  define GIC_ICDIPTR10_ID40(n)    ((uint32_t)(n) << GIC_ICDIPTR10_ID40_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR10_ID41_SHIFT   (12)      /* Bits 12-15: ID 41 priority */
#define GIC_ICDIPTR10_ID41_MASK    (15 << GIC_ICDIPTR10_ID41_SHIFT)
#  define GIC_ICDIPTR10_ID41(n)    ((uint32_t)(n) << GIC_ICDIPTR10_ID41_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR10_ID42_SHIFT   (20)      /* Bits 20-23: ID 42 priority */
#define GIC_ICDIPTR10_ID42_MASK    (15 << GIC_ICDIPTR10_ID42_SHIFT)
#  define GIC_ICDIPTR10_ID42(n)    ((uint32_t)(n) << GIC_ICDIPTR10_ID42_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR10_ID43_SHIFT   (28)      /* Bits 28-31: ID 43 priority */
#define GIC_ICDIPTR10_ID43_MASK    (15 << GIC_ICDIPTR10_ID43_SHIFT)
#  define GIC_ICDIPTR10_ID43(n)    ((uint32_t)(n) << GIC_ICDIPTR10_ID43_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR11_ID44_SHIFT   (0)       /* Bits 4-7: ID 44 priority */
#define GIC_ICDIPTR11_ID44_MASK    (15 << GIC_ICDIPTR11_ID44_SHIFT)
#  define GIC_ICDIPTR11_ID44(n)    ((uint32_t)(n) << GIC_ICDIPTR11_ID44_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR11_ID45_SHIFT   (12)      /* Bits 12-15: ID 45 priority */
#define GIC_ICDIPTR11_ID45_MASK    (15 << GIC_ICDIPTR11_ID45_SHIFT)
#  define GIC_ICDIPTR11_ID45(n)    ((uint32_t)(n) << GIC_ICDIPTR11_ID45_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR11_ID46_SHIFT   (20)      /* Bits 20-23: ID 46 priority */
#define GIC_ICDIPTR11_ID46_MASK    (15 << GIC_ICDIPTR11_ID46_SHIFT)
#  define GIC_ICDIPTR11_ID46(n)    ((uint32_t)(n) << GIC_ICDIPTR11_ID46_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR11_ID47_SHIFT   (28)      /* Bits 28-31: ID 47 priority */
#define GIC_ICDIPTR11_ID47_MASK    (15 << GIC_ICDIPTR11_ID47_SHIFT)
#  define GIC_ICDIPTR11_ID47(n)    ((uint32_t)(n) << GIC_ICDIPTR11_ID47_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR12_ID48_SHIFT   (0)       /* Bits 4-7: ID 48 priority */
#define GIC_ICDIPTR12_ID48_MASK    (15 << GIC_ICDIPTR12_ID48_SHIFT)
#  define GIC_ICDIPTR12_ID48(n)    ((uint32_t)(n) << GIC_ICDIPTR12_ID48_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR12_ID49_SHIFT   (12)      /* Bits 12-15: ID 49 priority */
#define GIC_ICDIPTR12_ID49_MASK    (15 << GIC_ICDIPTR12_ID49_SHIFT)
#  define GIC_ICDIPTR12_ID49(n)    ((uint32_t)(n) << GIC_ICDIPTR12_ID49_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR12_ID50_SHIFT   (20)      /* Bits 20-23: ID 50 priority */
#define GIC_ICDIPTR12_ID50_MASK    (15 << GIC_ICDIPTR12_ID50_SHIFT)
#  define GIC_ICDIPTR12_ID50(n)    ((uint32_t)(n) << GIC_ICDIPTR12_ID50_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR12_ID51_SHIFT   (28)      /* Bits 28-31: ID 51 priority */
#define GIC_ICDIPTR12_ID51_MASK    (15 << GIC_ICDIPTR12_ID51_SHIFT)
#  define GIC_ICDIPTR12_ID51(n)    ((uint32_t)(n) << GIC_ICDIPTR12_ID51_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR13_ID52_SHIFT   (0)       /* Bits 4-7: ID 52 priority */
#define GIC_ICDIPTR13_ID52_MASK    (15 << GIC_ICDIPTR13_ID52_SHIFT)
#  define GIC_ICDIPTR13_ID52(n)    ((uint32_t)(n) << GIC_ICDIPTR13_ID52_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR13_ID53_SHIFT   (12)      /* Bits 12-15: ID 53 priority */
#define GIC_ICDIPTR13_ID53_MASK    (15 << GIC_ICDIPTR13_ID53_SHIFT)
#  define GIC_ICDIPTR13_ID53(n)    ((uint32_t)(n) << GIC_ICDIPTR13_ID53_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR13_ID54_SHIFT   (20)      /* Bits 20-23: ID 54 priority */
#define GIC_ICDIPTR13_ID54_MASK    (15 << GIC_ICDIPTR13_ID54_SHIFT)
#  define GIC_ICDIPTR13_ID54(n)    ((uint32_t)(n) << GIC_ICDIPTR13_ID54_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR13_ID55_SHIFT   (28)      /* Bits 28-31: ID 55 priority */
#define GIC_ICDIPTR13_ID55_MASK    (15 << GIC_ICDIPTR13_ID55_SHIFT)
#  define GIC_ICDIPTR13_ID55(n)    ((uint32_t)(n) << GIC_ICDIPTR13_ID55_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR14_ID56_SHIFT   (0)       /* Bits 4-7: ID 56 priority */
#define GIC_ICDIPTR14_ID56_MASK    (15 << GIC_ICDIPTR14_ID56_SHIFT)
#  define GIC_ICDIPTR14_ID56(n)    ((uint32_t)(n) << GIC_ICDIPTR14_ID56_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR14_ID57_SHIFT   (12)      /* Bits 12-15: ID 57 priority */
#define GIC_ICDIPTR14_ID57_MASK    (15 << GIC_ICDIPTR14_ID57_SHIFT)
#  define GIC_ICDIPTR14_ID57(n)    ((uint32_t)(n) << GIC_ICDIPTR14_ID57_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR14_ID58_SHIFT   (20)      /* Bits 20-23: ID 58 priority */
#define GIC_ICDIPTR14_ID58_MASK    (15 << GIC_ICDIPTR14_ID58_SHIFT)
#  define GIC_ICDIPTR14_ID58(n)    ((uint32_t)(n) << GIC_ICDIPTR14_ID58_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR14_ID59_SHIFT   (28)      /* Bits 28-31: ID 59 priority */
#define GIC_ICDIPTR14_ID59_MASK    (15 << GIC_ICDIPTR14_ID59_SHIFT)
#  define GIC_ICDIPTR14_ID59(n)    ((uint32_t)(n) << GIC_ICDIPTR14_ID59_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR15_ID60_SHIFT   (0)       /* Bits 4-7: ID 60 priority */
#define GIC_ICDIPTR15_ID60_MASK    (15 << GIC_ICDIPTR15_ID60_SHIFT)
#  define GIC_ICDIPTR15_ID60(n)    ((uint32_t)(n) << GIC_ICDIPTR15_ID60_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR15_ID61_SHIFT   (12)      /* Bits 12-15: ID 61 priority */
#define GIC_ICDIPTR15_ID61_MASK    (15 << GIC_ICDIPTR15_ID61_SHIFT)
#  define GIC_ICDIPTR15_ID61(n)    ((uint32_t)(n) << GIC_ICDIPTR15_ID61_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR15_ID62_SHIFT   (20)      /* Bits 20-23: ID 62 priority */
#define GIC_ICDIPTR15_ID62_MASK    (15 << GIC_ICDIPTR15_ID62_SHIFT)
#  define GIC_ICDIPTR15_ID62(n)    ((uint32_t)(n) << GIC_ICDIPTR15_ID62_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR15_ID63_SHIFT   (28)      /* Bits 28-31: ID 63 priority */
#define GIC_ICDIPTR15_ID63_MASK    (15 << GIC_ICDIPTR15_ID63_SHIFT)
#  define GIC_ICDIPTR15_ID63(n)    ((uint32_t)(n) << GIC_ICDIPTR15_ID63_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR16_ID64_SHIFT   (0)       /* Bits 4-7: ID 64 priority */
#define GIC_ICDIPTR16_ID64_MASK    (15 << GIC_ICDIPTR16_ID64_SHIFT)
#  define GIC_ICDIPTR16_ID64(n)    ((uint32_t)(n) << GIC_ICDIPTR16_ID64_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR16_ID65_SHIFT   (12)      /* Bits 12-15: ID 65 priority */
#define GIC_ICDIPTR16_ID65_MASK    (15 << GIC_ICDIPTR16_ID65_SHIFT)
#  define GIC_ICDIPTR16_ID65(n)    ((uint32_t)(n) << GIC_ICDIPTR16_ID65_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR16_ID66_SHIFT   (20)      /* Bits 20-23: ID 66 priority */
#define GIC_ICDIPTR16_ID66_MASK    (15 << GIC_ICDIPTR16_ID66_SHIFT)
#  define GIC_ICDIPTR16_ID66(n)    ((uint32_t)(n) << GIC_ICDIPTR16_ID66_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR16_ID67_SHIFT   (28)      /* Bits 28-31: ID 67 priority */
#define GIC_ICDIPTR16_ID67_MASK    (15 << GIC_ICDIPTR16_ID67_SHIFT)
#  define GIC_ICDIPTR16_ID67(n)    ((uint32_t)(n) << GIC_ICDIPTR16_ID67_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR17_ID68_SHIFT   (0)       /* Bits 4-7: ID 68 priority */
#define GIC_ICDIPTR17_ID68_MASK    (15 << GIC_ICDIPTR17_ID68_SHIFT)
#  define GIC_ICDIPTR17_ID68(n)    ((uint32_t)(n) << GIC_ICDIPTR17_ID68_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR17_ID69_SHIFT   (12)      /* Bits 12-15: ID 69 priority */
#define GIC_ICDIPTR17_ID69_MASK    (15 << GIC_ICDIPTR17_ID69_SHIFT)
#  define GIC_ICDIPTR17_ID69(n)    ((uint32_t)(n) << GIC_ICDIPTR17_ID69_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR17_ID70_SHIFT   (20)      /* Bits 20-23: ID 70 priority */
#define GIC_ICDIPTR17_ID70_MASK    (15 << GIC_ICDIPTR17_ID70_SHIFT)
#  define GIC_ICDIPTR17_ID70(n)    ((uint32_t)(n) << GIC_ICDIPTR17_ID70_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR17_ID71_SHIFT   (28)      /* Bits 28-31: ID 71 priority */
#define GIC_ICDIPTR17_ID71_MASK    (15 << GIC_ICDIPTR17_ID71_SHIFT)
#  define GIC_ICDIPTR17_ID71(n)    ((uint32_t)(n) << GIC_ICDIPTR17_ID71_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR18_ID72_SHIFT   (0)       /* Bits 4-7: ID 72 priority */
#define GIC_ICDIPTR18_ID72_MASK    (15 << GIC_ICDIPTR18_ID72_SHIFT)
#  define GIC_ICDIPTR18_ID72(n)    ((uint32_t)(n) << GIC_ICDIPTR18_ID72_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR18_ID73_SHIFT   (12)      /* Bits 12-15: ID 73 priority */
#define GIC_ICDIPTR18_ID73_MASK    (15 << GIC_ICDIPTR18_ID73_SHIFT)
#  define GIC_ICDIPTR18_ID73(n)    ((uint32_t)(n) << GIC_ICDIPTR18_ID73_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR18_ID74_SHIFT   (20)      /* Bits 20-23: ID 74 priority */
#define GIC_ICDIPTR18_ID74_MASK    (15 << GIC_ICDIPTR18_ID74_SHIFT)
#  define GIC_ICDIPTR18_ID74(n)    ((uint32_t)(n) << GIC_ICDIPTR18_ID74_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR18_ID75_SHIFT   (28)      /* Bits 28-31: ID 75 priority */
#define GIC_ICDIPTR18_ID75_MASK    (15 << GIC_ICDIPTR18_ID75_SHIFT)
#  define GIC_ICDIPTR18_ID75(n)    ((uint32_t)(n) << GIC_ICDIPTR18_ID75_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR19_ID76_SHIFT   (0)       /* Bits 4-7: ID 76 priority */
#define GIC_ICDIPTR19_ID76_MASK    (15 << GIC_ICDIPTR19_ID76_SHIFT)
#  define GIC_ICDIPTR19_ID76(n)    ((uint32_t)(n) << GIC_ICDIPTR19_ID76_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR19_ID77_SHIFT   (12)      /* Bits 12-15: ID 77 priority */
#define GIC_ICDIPTR19_ID77_MASK    (15 << GIC_ICDIPTR19_ID77_SHIFT)
#  define GIC_ICDIPTR19_ID77(n)    ((uint32_t)(n) << GIC_ICDIPTR19_ID77_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR19_ID78_SHIFT   (20)      /* Bits 20-23: ID 78 priority */
#define GIC_ICDIPTR19_ID78_MASK    (15 << GIC_ICDIPTR19_ID78_SHIFT)
#  define GIC_ICDIPTR19_ID78(n)    ((uint32_t)(n) << GIC_ICDIPTR19_ID78_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR19_ID79_SHIFT   (28)      /* Bits 28-31: ID 78 priority */
#define GIC_ICDIPTR19_ID79_MASK    (15 << GIC_ICDIPTR19_ID79_SHIFT)
#  define GIC_ICDIPTR19_ID79(n)    ((uint32_t)(n) << GIC_ICDIPTR19_ID79_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR20_ID80_SHIFT   (0)       /* Bits 4-7: ID 80 priority */
#define GIC_ICDIPTR20_ID80_MASK    (15 << GIC_ICDIPTR20_ID80_SHIFT)
#  define GIC_ICDIPTR20_ID80(n)    ((uint32_t)(n) << GIC_ICDIPTR20_ID80_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR20_ID81_SHIFT   (12)      /* Bits 12-15: ID 81 priority */
#define GIC_ICDIPTR20_ID81_MASK    (15 << GIC_ICDIPTR20_ID81_SHIFT)
#  define GIC_ICDIPTR20_ID81(n)    ((uint32_t)(n) << GIC_ICDIPTR20_ID81_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR20_ID82_SHIFT   (20)      /* Bits 20-23: ID 82 priority */
#define GIC_ICDIPTR20_ID82_MASK    (15 << GIC_ICDIPTR20_ID82_SHIFT)
#  define GIC_ICDIPTR20_ID82(n)    ((uint32_t)(n) << GIC_ICDIPTR20_ID82_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR20_ID83_SHIFT   (28)      /* Bits 28-31: ID 83 priority */
#define GIC_ICDIPTR20_ID83_MASK    (15 << GIC_ICDIPTR20_ID83_SHIFT)
#  define GIC_ICDIPTR20_ID83(n)    ((uint32_t)(n) << GIC_ICDIPTR20_ID83_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR21_ID84_SHIFT   (0)       /* Bits 4-7: ID 84 priority */
#define GIC_ICDIPTR21_ID84_MASK    (15 << GIC_ICDIPTR21_ID84_SHIFT)
#  define GIC_ICDIPTR21_ID84(n)    ((uint32_t)(n) << GIC_ICDIPTR21_ID84_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR21_ID85_SHIFT   (12)      /* Bits 12-15: ID 85 priority */
#define GIC_ICDIPTR21_ID85_MASK    (15 << GIC_ICDIPTR21_ID85_SHIFT)
#  define GIC_ICDIPTR21_ID85(n)    ((uint32_t)(n) << GIC_ICDIPTR21_ID85_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR21_ID86_SHIFT   (20)      /* Bits 20-23: ID 86 priority */
#define GIC_ICDIPTR21_ID86_MASK    (15 << GIC_ICDIPTR21_ID86_SHIFT)
#  define GIC_ICDIPTR21_ID86(n)    ((uint32_t)(n) << GIC_ICDIPTR21_ID86_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR21_ID87_SHIFT   (28)      /* Bits 28-31: ID 87 priority */
#define GIC_ICDIPTR21_ID87_MASK    (15 << GIC_ICDIPTR21_ID87_SHIFT)
#  define GIC_ICDIPTR21_ID87(n)    ((uint32_t)(n) << GIC_ICDIPTR21_ID87_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR22_ID88_SHIFT   (0)       /* Bits 4-7: ID 88 priority */
#define GIC_ICDIPTR22_ID88_MASK    (15 << GIC_ICDIPTR22_ID88_SHIFT)
#  define GIC_ICDIPTR22_ID88(n)    ((uint32_t)(n) << GIC_ICDIPTR22_ID88_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR22_ID89_SHIFT   (12)      /* Bits 12-15: ID 89 priority */
#define GIC_ICDIPTR22_ID89_MASK    (15 << GIC_ICDIPTR22_ID89_SHIFT)
#  define GIC_ICDIPTR22_ID89(n)    ((uint32_t)(n) << GIC_ICDIPTR22_ID89_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR22_ID90_SHIFT   (20)      /* Bits 20-23: ID 90 priority */
#define GIC_ICDIPTR22_ID90_MASK    (15 << GIC_ICDIPTR22_ID90_SHIFT)
#  define GIC_ICDIPTR22_ID90(n)    ((uint32_t)(n) << GIC_ICDIPTR22_ID90_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR22_ID91_SHIFT   (28)      /* Bits 28-31: ID 91 priority */
#define GIC_ICDIPTR22_ID91_MASK    (15 << GIC_ICDIPTR22_ID91_SHIFT)
#  define GIC_ICDIPTR22_ID91(n)    ((uint32_t)(n) << GIC_ICDIPTR22_ID91_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDIPTR23_ID92_SHIFT   (0)       /* Bits 4-7: ID 92 priority */
#define GIC_ICDIPTR23_ID92_MASK    (15 << GIC_ICDIPTR23_ID92_SHIFT)
#  define GIC_ICDIPTR23_ID92(n)    ((uint32_t)(n) << GIC_ICDIPTR23_ID92_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDIPTR23_ID93_SHIFT   (12)      /* Bits 12-15: ID 93 priority */
#define GIC_ICDIPTR23_ID93_MASK    (15 << GIC_ICDIPTR23_ID93_SHIFT)
#  define GIC_ICDIPTR23_ID93(n)    ((uint32_t)(n) << GIC_ICDIPTR23_ID93_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDIPTR23_ID94_SHIFT   (20)      /* Bits 20-23: ID 94 priority */
#define GIC_ICDIPTR23_ID94_MASK    (15 << GIC_ICDIPTR23_ID94_SHIFT)
#  define GIC_ICDIPTR23_ID94(n)    ((uint32_t)(n) << GIC_ICDIPTR23_ID94_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDIPTR23_ID95_SHIFT   (28)      /* Bits 28-31: ID 95 priority */
#define GIC_ICDIPTR23_ID95_MASK    (15 << GIC_ICDIPTR23_ID95_SHIFT)
#  define GIC_ICDIPTR23_ID95(n)    ((uint32_t)(n) << GIC_ICDIPTR23_ID95_SHIFT)

/* Interrupt CPU Target Registers 8...23 */

#define CPU0_TARGET                (1 << 0)
#define CPU1_TARGET                (1 << 1)
#define CPU2_TARGET                (1 << 2)
#define CPU3_TARGET                (1 << 3)

#define GIC_ICDICTGR_ID_SHIFT(n)   (((n) & 3) << 3)
#define GIC_ICDICTGR_ID_MASK(n)    (15 << GIC_ICDICTGR_ID_SHIFT(n))
#  define GIC_ICDICTGR_ID(n,t)     ((uint32_t)(t) << GIC_ICDICTGR_ID_SHIFT(n))

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR8_ID32_SHIFT   (0)       /* Bits 4-7: ID 32 CPU target */
#define GIC_ICDICTGR8_ID32_MASK    (15 << GIC_ICDICTGR8_ID32_SHIFT)
#  define GIC_ICDICTGR8_ID32(n)    ((uint32_t)(n) << GIC_ICDICTGR8_ID32_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR8_ID33_SHIFT   (12)      /* Bits 12-15: ID 33 CPU target */
#define GIC_ICDICTGR8_ID33_MASK    (15 << GIC_ICDICTGR8_ID33_SHIFT)
#  define GIC_ICDICTGR8_ID33(n)    ((uint32_t)(n) << GIC_ICDICTGR8_ID33_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR8_ID34_SHIFT   (20)      /* Bits 20-23: ID 34 CPU target */
#define GIC_ICDICTGR8_ID34_MASK    (15 << GIC_ICDICTGR8_ID34_SHIFT)
#  define GIC_ICDICTGR8_ID34(n)    ((uint32_t)(n) << GIC_ICDICTGR8_ID34_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR8_ID35_SHIFT   (28)      /* Bits 28-31: ID 35 CPU target */
#define GIC_ICDICTGR8_ID35_MASK    (15 << GIC_ICDICTGR8_ID35_SHIFT)
#  define GIC_ICDICTGR8_ID35(n)    ((uint32_t)(n) << GIC_ICDICTGR8_ID35_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR9_ID36_SHIFT   (0)       /* Bits 4-7: ID 36 CPU target */
#define GIC_ICDICTGR9_ID36_MASK    (15 << GIC_ICDICTGR9_ID36_SHIFT)
#  define GIC_ICDICTGR9_ID36(n)    ((uint32_t)(n) << GIC_ICDICTGR9_ID36_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR9_ID37_SHIFT   (12)      /* Bits 12-15: ID 37 CPU target */
#define GIC_ICDICTGR9_ID37_MASK    (15 << GIC_ICDICTGR9_ID37_SHIFT)
#  define GIC_ICDICTGR9_ID37(n)    ((uint32_t)(n) << GIC_ICDICTGR9_ID37_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR9_ID38_SHIFT   (20)      /* Bits 20-23: ID 38 CPU target */
#define GIC_ICDICTGR9_ID38_MASK    (15 << GIC_ICDICTGR9_ID38_SHIFT)
#  define GIC_ICDICTGR9_ID38(n)    ((uint32_t)(n) << GIC_ICDICTGR9_ID38_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR9_ID39_SHIFT   (28)      /* Bits 28-31: ID 39 CPU target */
#define GIC_ICDICTGR9_ID39_MASK    (15 << GIC_ICDICTGR9_ID39_SHIFT)
#  define GIC_ICDICTGR9_ID39(n)    ((uint32_t)(n) << GIC_ICDICTGR9_ID39_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR10_ID40_SHIFT  (0)       /* Bits 4-7: ID 40 CPU target */
#define GIC_ICDICTGR10_ID40_MASK   (15 << GIC_ICDICTGR10_ID40_SHIFT)
#  define GIC_ICDICTGR10_ID40(n)   ((uint32_t)(n) << GIC_ICDICTGR10_ID40_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR10_ID41_SHIFT  (12)      /* Bits 12-15: ID 41 CPU target */
#define GIC_ICDICTGR10_ID41_MASK   (15 << GIC_ICDICTGR10_ID41_SHIFT)
#  define GIC_ICDICTGR10_ID41(n)   ((uint32_t)(n) << GIC_ICDICTGR10_ID41_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR10_ID42_SHIFT  (20)      /* Bits 20-23: ID 42 CPU target */
#define GIC_ICDICTGR10_ID42_MASK   (15 << GIC_ICDICTGR10_ID42_SHIFT)
#  define GIC_ICDICTGR10_ID42(n)   ((uint32_t)(n) << GIC_ICDICTGR10_ID42_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR10_ID43_SHIFT  (28)      /* Bits 28-31: ID 43 CPU target */
#define GIC_ICDICTGR10_ID43_MASK   (15 << GIC_ICDICTGR10_ID43_SHIFT)
#  define GIC_ICDICTGR10_ID43(n)   ((uint32_t)(n) << GIC_ICDICTGR10_ID43_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR11_ID44_SHIFT  (0)       /* Bits 4-7: ID 44 CPU target */
#define GIC_ICDICTGR11_ID44_MASK   (15 << GIC_ICDICTGR11_ID44_SHIFT)
#  define GIC_ICDICTGR11_ID44(n)   ((uint32_t)(n) << GIC_ICDICTGR11_ID44_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR11_ID45_SHIFT  (12)      /* Bits 12-15: ID 45 CPU target */
#define GIC_ICDICTGR11_ID45_MASK   (15 << GIC_ICDICTGR11_ID45_SHIFT)
#  define GIC_ICDICTGR11_ID45(n)   ((uint32_t)(n) << GIC_ICDICTGR11_ID45_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR11_ID46_SHIFT  (20)      /* Bits 20-23: ID 46 CPU target */
#define GIC_ICDICTGR11_ID46_MASK   (15 << GIC_ICDICTGR11_ID46_SHIFT)
#  define GIC_ICDICTGR11_ID46(n)   ((uint32_t)(n) << GIC_ICDICTGR11_ID46_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR11_ID47_SHIFT  (28)      /* Bits 28-31: ID 47 CPU target */
#define GIC_ICDICTGR11_ID47_MASK   (15 << GIC_ICDICTGR11_ID47_SHIFT)
#  define GIC_ICDICTGR11_ID47(n)   ((uint32_t)(n) << GIC_ICDICTGR11_ID47_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR12_ID48_SHIFT  (0)       /* Bits 4-7: ID 48 CPU target */
#define GIC_ICDICTGR12_ID48_MASK   (15 << GIC_ICDICTGR12_ID48_SHIFT)
#  define GIC_ICDICTGR12_ID48(n)   ((uint32_t)(n) << GIC_ICDICTGR12_ID48_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR12_ID49_SHIFT  (12)      /* Bits 12-15: ID 49 CPU target */
#define GIC_ICDICTGR12_ID49_MASK   (15 << GIC_ICDICTGR12_ID49_SHIFT)
#  define GIC_ICDICTGR12_ID49(n)   ((uint32_t)(n) << GIC_ICDICTGR12_ID49_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR12_ID50_SHIFT  (20)      /* Bits 20-23: ID 50 CPU target */
#define GIC_ICDICTGR12_ID50_MASK   (15 << GIC_ICDICTGR12_ID50_SHIFT)
#  define GIC_ICDICTGR12_ID50(n)   ((uint32_t)(n) << GIC_ICDICTGR12_ID50_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR12_ID51_SHIFT  (28)      /* Bits 28-31: ID 51 CPU target */
#define GIC_ICDICTGR12_ID51_MASK   (15 << GIC_ICDICTGR12_ID51_SHIFT)
#  define GIC_ICDICTGR12_ID51(n)   ((uint32_t)(n) << GIC_ICDICTGR12_ID51_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR13_ID52_SHIFT  (0)       /* Bits 4-7: ID 52 CPU target */
#define GIC_ICDICTGR13_ID52_MASK   (15 << GIC_ICDICTGR13_ID52_SHIFT)
#  define GIC_ICDICTGR13_ID52(n)   ((uint32_t)(n) << GIC_ICDICTGR13_ID52_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR13_ID53_SHIFT  (12)      /* Bits 12-15: ID 53 CPU target */
#define GIC_ICDICTGR13_ID53_MASK   (15 << GIC_ICDICTGR13_ID53_SHIFT)
#  define GIC_ICDICTGR13_ID53(n)   ((uint32_t)(n) << GIC_ICDICTGR13_ID53_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR13_ID54_SHIFT  (20)      /* Bits 20-23: ID 54 CPU target */
#define GIC_ICDICTGR13_ID54_MASK   (15 << GIC_ICDICTGR13_ID54_SHIFT)
#  define GIC_ICDICTGR13_ID54(n)   ((uint32_t)(n) << GIC_ICDICTGR13_ID54_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR13_ID55_SHIFT  (28)      /* Bits 28-31: ID 55 CPU target */
#define GIC_ICDICTGR13_ID55_MASK   (15 << GIC_ICDICTGR13_ID55_SHIFT)
#  define GIC_ICDICTGR13_ID55(n)   ((uint32_t)(n) << GIC_ICDICTGR13_ID55_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR14_ID56_SHIFT  (0)       /* Bits 4-7: ID 56 CPU target */
#define GIC_ICDICTGR14_ID56_MASK   (15 << GIC_ICDICTGR14_ID56_SHIFT)
#  define GIC_ICDICTGR14_ID56(n)   ((uint32_t)(n) << GIC_ICDICTGR14_ID56_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR14_ID57_SHIFT  (12)      /* Bits 12-15: ID 57 CPU target */
#define GIC_ICDICTGR14_ID57_MASK   (15 << GIC_ICDICTGR14_ID57_SHIFT)
#  define GIC_ICDICTGR14_ID57(n)   ((uint32_t)(n) << GIC_ICDICTGR14_ID57_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR14_ID58_SHIFT  (20)      /* Bits 20-23: ID 58 CPU target */
#define GIC_ICDICTGR14_ID58_MASK   (15 << GIC_ICDICTGR14_ID58_SHIFT)
#  define GIC_ICDICTGR14_ID58(n)   ((uint32_t)(n) << GIC_ICDICTGR14_ID58_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR14_ID59_SHIFT  (28)      /* Bits 28-31: ID 59 CPU target */
#define GIC_ICDICTGR14_ID59_MASK   (15 << GIC_ICDICTGR14_ID59_SHIFT)
#  define GIC_ICDICTGR14_ID59(n)   ((uint32_t)(n) << GIC_ICDICTGR14_ID59_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR15_ID60_SHIFT  (0)       /* Bits 4-7: ID 60 CPU target */
#define GIC_ICDICTGR15_ID60_MASK   (15 << GIC_ICDICTGR15_ID60_SHIFT)
#  define GIC_ICDICTGR15_ID60(n)   ((uint32_t)(n) << GIC_ICDICTGR15_ID60_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR15_ID61_SHIFT  (12)      /* Bits 12-15: ID 61 CPU target */
#define GIC_ICDICTGR15_ID61_MASK   (15 << GIC_ICDICTGR15_ID61_SHIFT)
#  define GIC_ICDICTGR15_ID61(n)   ((uint32_t)(n) << GIC_ICDICTGR15_ID61_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR15_ID62_SHIFT  (20)      /* Bits 20-23: ID 62 CPU target */
#define GIC_ICDICTGR15_ID62_MASK   (15 << GIC_ICDICTGR15_ID62_SHIFT)
#  define GIC_ICDICTGR15_ID62(n)   ((uint32_t)(n) << GIC_ICDICTGR15_ID62_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR15_ID63_SHIFT  (28)      /* Bits 28-31: ID 63 CPU target */
#define GIC_ICDICTGR15_ID63_MASK   (15 << GIC_ICDICTGR15_ID63_SHIFT)
#  define GIC_ICDICTGR15_ID63(n)   ((uint32_t)(n) << GIC_ICDICTGR15_ID63_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR16_ID64_SHIFT  (0)       /* Bits 4-7: ID 64 CPU target */
#define GIC_ICDICTGR16_ID64_MASK   (15 << GIC_ICDICTGR16_ID64_SHIFT)
#  define GIC_ICDICTGR16_ID64(n)   ((uint32_t)(n) << GIC_ICDICTGR16_ID64_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR16_ID65_SHIFT  (12)      /* Bits 12-15: ID 65 CPU target */
#define GIC_ICDICTGR16_ID65_MASK   (15 << GIC_ICDICTGR16_ID65_SHIFT)
#  define GIC_ICDICTGR16_ID65(n)   ((uint32_t)(n) << GIC_ICDICTGR16_ID65_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR16_ID66_SHIFT  (20)      /* Bits 20-23: ID 66 CPU target */
#define GIC_ICDICTGR16_ID66_MASK   (15 << GIC_ICDICTGR16_ID66_SHIFT)
#  define GIC_ICDICTGR16_ID66(n)   ((uint32_t)(n) << GIC_ICDICTGR16_ID66_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR16_ID67_SHIFT  (28)      /* Bits 28-31: ID 67 CPU target */
#define GIC_ICDICTGR16_ID67_MASK   (15 << GIC_ICDICTGR16_ID67_SHIFT)
#  define GIC_ICDICTGR16_ID67(n)   ((uint32_t)(n) << GIC_ICDICTGR16_ID67_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR17_ID68_SHIFT  (0)       /* Bits 4-7: ID 68 CPU target */
#define GIC_ICDICTGR17_ID68_MASK   (15 << GIC_ICDICTGR17_ID68_SHIFT)
#  define GIC_ICDICTGR17_ID68(n)   ((uint32_t)(n) << GIC_ICDICTGR17_ID68_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR17_ID69_SHIFT  (12)      /* Bits 12-15: ID 69 CPU target */
#define GIC_ICDICTGR17_ID69_MASK   (15 << GIC_ICDICTGR17_ID69_SHIFT)
#  define GIC_ICDICTGR17_ID69(n)   ((uint32_t)(n) << GIC_ICDICTGR17_ID69_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR17_ID70_SHIFT  (20)      /* Bits 20-23: ID 70 CPU target */
#define GIC_ICDICTGR17_ID70_MASK   (15 << GIC_ICDICTGR17_ID70_SHIFT)
#  define GIC_ICDICTGR17_ID70(n)   ((uint32_t)(n) << GIC_ICDICTGR17_ID70_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR17_ID71_SHIFT  (28)      /* Bits 28-31: ID 71 CPU target */
#define GIC_ICDICTGR17_ID71_MASK   (15 << GIC_ICDICTGR17_ID71_SHIFT)
#  define GIC_ICDICTGR17_ID71(n)   ((uint32_t)(n) << GIC_ICDICTGR17_ID71_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR18_ID72_SHIFT  (0)       /* Bits 4-7: ID 72 CPU target */
#define GIC_ICDICTGR18_ID72_MASK   (15 << GIC_ICDICTGR18_ID72_SHIFT)
#  define GIC_ICDICTGR18_ID72(n)   ((uint32_t)(n) << GIC_ICDICTGR18_ID72_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR18_ID73_SHIFT  (12)      /* Bits 12-15: ID 73 CPU target */
#define GIC_ICDICTGR18_ID73_MASK   (15 << GIC_ICDICTGR18_ID73_SHIFT)
#  define GIC_ICDICTGR18_ID73(n)   ((uint32_t)(n) << GIC_ICDICTGR18_ID73_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR18_ID74_SHIFT  (20)      /* Bits 20-23: ID 74 CPU target */
#define GIC_ICDICTGR18_ID74_MASK   (15 << GIC_ICDICTGR18_ID74_SHIFT)
#  define GIC_ICDICTGR18_ID74(n)   ((uint32_t)(n) << GIC_ICDICTGR18_ID74_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR18_ID75_SHIFT  (28)      /* Bits 28-31: ID 75 CPU target */
#define GIC_ICDICTGR18_ID75_MASK   (15 << GIC_ICDICTGR18_ID75_SHIFT)
#  define GIC_ICDICTGR18_ID75(n)   ((uint32_t)(n) << GIC_ICDICTGR18_ID75_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR19_ID76_SHIFT  (0)       /* Bits 4-7: ID 76 CPU target */
#define GIC_ICDICTGR19_ID76_MASK   (15 << GIC_ICDICTGR19_ID76_SHIFT)
#  define GIC_ICDICTGR19_ID76(n)   ((uint32_t)(n) << GIC_ICDICTGR19_ID76_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR19_ID77_SHIFT  (12)      /* Bits 12-15: ID 77 CPU target */
#define GIC_ICDICTGR19_ID77_MASK   (15 << GIC_ICDICTGR19_ID77_SHIFT)
#  define GIC_ICDICTGR19_ID77(n)   ((uint32_t)(n) << GIC_ICDICTGR19_ID77_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR19_ID78_SHIFT  (20)      /* Bits 20-23: ID 78 CPU target */
#define GIC_ICDICTGR19_ID78_MASK   (15 << GIC_ICDICTGR19_ID78_SHIFT)
#  define GIC_ICDICTGR19_ID78(n)   ((uint32_t)(n) << GIC_ICDICTGR19_ID78_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR19_ID79_SHIFT  (28)      /* Bits 28-31: ID 79 CPU target */
#define GIC_ICDICTGR19_ID79_MASK   (15 << GIC_ICDICTGR19_ID79_SHIFT)
#  define GIC_ICDICTGR19_ID79(n)   ((uint32_t)(n) << GIC_ICDICTGR19_ID79_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR20_ID80_SHIFT  (0)       /* Bits 4-7: ID 80 CPU target */
#define GIC_ICDICTGR20_ID80_MASK   (15 << GIC_ICDICTGR20_ID80_SHIFT)
#  define GIC_ICDICTGR20_ID80(n)   ((uint32_t)(n) << GIC_ICDICTGR20_ID80_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR20_ID81_SHIFT  (12)      /* Bits 12-15: ID 81 CPU target */
#define GIC_ICDICTGR20_ID81_MASK   (15 << GIC_ICDICTGR20_ID81_SHIFT)
#  define GIC_ICDICTGR20_ID81(n)   ((uint32_t)(n) << GIC_ICDICTGR20_ID81_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR20_ID82_SHIFT  (20)      /* Bits 20-23: ID 82 CPU target */
#define GIC_ICDICTGR20_ID82_MASK   (15 << GIC_ICDICTGR20_ID82_SHIFT)
#  define GIC_ICDICTGR20_ID82(n)   ((uint32_t)(n) << GIC_ICDICTGR20_ID82_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR20_ID83_SHIFT  (28)      /* Bits 28-31: ID 83 CPU target */
#define GIC_ICDICTGR20_ID83_MASK   (15 << GIC_ICDICTGR20_ID83_SHIFT)
#  define GIC_ICDICTGR20_ID83(n)   ((uint32_t)(n) << GIC_ICDICTGR20_ID83_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR21_ID84_SHIFT  (0)       /* Bits 4-7: ID 84 CPU target */
#define GIC_ICDICTGR21_ID84_MASK   (15 << GIC_ICDICTGR21_ID84_SHIFT)
#  define GIC_ICDICTGR21_ID84(n)   ((uint32_t)(n) << GIC_ICDICTGR21_ID84_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR21_ID85_SHIFT  (12)      /* Bits 12-15: ID 85 CPU target */
#define GIC_ICDICTGR21_ID85_MASK   (15 << GIC_ICDICTGR21_ID85_SHIFT)
#  define GIC_ICDICTGR21_ID85(n)   ((uint32_t)(n) << GIC_ICDICTGR21_ID85_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR21_ID86_SHIFT  (20)      /* Bits 20-23: ID 86 CPU target */
#define GIC_ICDICTGR21_ID86_MASK   (15 << GIC_ICDICTGR21_ID86_SHIFT)
#  define GIC_ICDICTGR21_ID86(n)   ((uint32_t)(n) << GIC_ICDICTGR21_ID86_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR21_ID87_SHIFT  (28)      /* Bits 28-31: ID 87 CPU target */
#define GIC_ICDICTGR21_ID87_MASK   (15 << GIC_ICDICTGR21_ID87_SHIFT)
#  define GIC_ICDICTGR21_ID87(n)   ((uint32_t)(n) << GIC_ICDICTGR21_ID87_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR22_ID88_SHIFT  (0)       /* Bits 4-7: ID 88 CPU target */
#define GIC_ICDICTGR22_ID88_MASK   (15 << GIC_ICDICTGR22_ID88_SHIFT)
#  define GIC_ICDICTGR22_ID88(n)   ((uint32_t)(n) << GIC_ICDICTGR22_ID88_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR22_ID89_SHIFT  (12)      /* Bits 12-15: ID 89 CPU target */
#define GIC_ICDICTGR22_ID89_MASK   (15 << GIC_ICDICTGR22_ID89_SHIFT)
#  define GIC_ICDICTGR22_ID89(n)   ((uint32_t)(n) << GIC_ICDICTGR22_ID89_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR22_ID90_SHIFT  (20)      /* Bits 20-23: ID 90 CPU target */
#define GIC_ICDICTGR22_ID90_MASK   (15 << GIC_ICDICTGR22_ID90_SHIFT)
#  define GIC_ICDICTGR22_ID90(n)   ((uint32_t)(n) << GIC_ICDICTGR22_ID90_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR22_ID91_SHIFT  (28)      /* Bits 28-31: ID 91 CPU target */
#define GIC_ICDICTGR22_ID91_MASK   (15 << GIC_ICDICTGR22_ID91_SHIFT)
#  define GIC_ICDICTGR22_ID91(n)   ((uint32_t)(n) << GIC_ICDICTGR22_ID91_SHIFT)

                                             /* Bits 0-3: Reserved */
#define GIC_ICDICTGR23_ID92_SHIFT  (0)       /* Bits 4-7: ID 92 CPU target */
#define GIC_ICDICTGR23_ID92_MASK   (15 << GIC_ICDICTGR23_ID92_SHIFT)
#  define GIC_ICDICTGR23_ID92(n)   ((uint32_t)(n) << GIC_ICDICTGR23_ID92_SHIFT)
                                             /* Bits 8-11: Reserved */
#define GIC_ICDICTGR23_ID93_SHIFT  (12)      /* Bits 12-15: ID 93 CPU target */
#define GIC_ICDICTGR23_ID93_MASK   (15 << GIC_ICDICTGR23_ID93_SHIFT)
#  define GIC_ICDICTGR23_ID93(n)   ((uint32_t)(n) << GIC_ICDICTGR23_ID93_SHIFT)
                                             /* Bits 16-19: Reserved */
#define GIC_ICDICTGR23_ID94_SHIFT  (20)      /* Bits 20-23: ID 94 CPU target */
#define GIC_ICDICTGR23_ID94_MASK   (15 << GIC_ICDICTGR23_ID94_SHIFT)
#  define GIC_ICDICTGR23_ID94(n)   ((uint32_t)(n) << GIC_ICDICTGR23_ID94_SHIFT)
                                             /* Bits 24-27: Reserved */
#define GIC_ICDICTGR23_ID95_SHIFT  (28)      /* Bits 28-31: ID 95 CPU target */
#define GIC_ICDICTGR23_ID95_MASK   (15 << GIC_ICDICTGR23_ID95_SHIFT)
#  define GIC_ICDICTGR23_ID95(n)   ((uint32_t)(n) << GIC_ICDICTGR23_ID95_SHIFT)

/* Interrupt Configuration Register 2, 3, 4, 5 */

#define INT_ICDICFR_NN             0         /* Bit n: 0= N-N Model */
#define INT_ICDICFR_1N             1         /* Bit n: 1= 1-N Model */
#define INT_ICDICFR_LEVEL          0         /* Bit n+1: 0=Level sensitive */
#define INT_ICDICFR_EDGE           2         /* Bit n+2: 1=Edge sensitive */
 
#define GIC_ICDICFR_ID_SHIFT(n)    (((n) & 15) << 1)
#define GIC_ICDICFR_ID_MASK(n)     (3 << GIC_ICDICFR_ID_SHIFT(n))
#  define GIC_ICDICFR_ID(n,c)      ((uint32_t)(c) << GIC_ICDICFR_ID_SHIFT(n))

#define GIC_ICDICFR2_ID32_SHIFT    (0)       /* Bits 0-1: ID 32 configuration */
#define GIC_ICDICFR2_ID32_MASK     (3 << GIC_ICDICFR2_ID32_SHIFT)
#  define GIC_ICDICFR2_ID32(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID32_SHIFT)
#define GIC_ICDICFR2_ID33_SHIFT    (2)       /* Bits 2-3: ID 33 configuration */
#define GIC_ICDICFR2_ID33_MASK     (3 << GIC_ICDICFR2_ID33_SHIFT)
#  define GIC_ICDICFR2_ID33(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID33_SHIFT)
#define GIC_ICDICFR2_ID34_SHIFT    (4)       /* Bits 4-5: ID 34 configuration */
#define GIC_ICDICFR2_ID34_MASK     (3 << GIC_ICDICFR2_ID34_SHIFT)
#  define GIC_ICDICFR2_ID34(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID34_SHIFT)
#define GIC_ICDICFR2_ID35_SHIFT    (6)       /* Bits 6-7: ID 35 configuration */
#define GIC_ICDICFR2_ID35_MASK     (3 << GIC_ICDICFR2_ID35_SHIFT)
#  define GIC_ICDICFR2_ID35(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID35_SHIFT)
#define GIC_ICDICFR2_ID36_SHIFT    (8)       /* Bits 8-9: ID 36 configuration */
#define GIC_ICDICFR2_ID36_MASK     (3 << GIC_ICDICFR2_ID36_SHIFT)
#  define GIC_ICDICFR2_ID36(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID36_SHIFT)
#define GIC_ICDICFR2_ID37_SHIFT    (10)      /* Bits 10-11: ID 37 configuration */
#define GIC_ICDICFR2_ID37_MASK     (3 << GIC_ICDICFR2_ID37_SHIFT)
#  define GIC_ICDICFR2_ID37(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID37_SHIFT)
#define GIC_ICDICFR2_ID38_SHIFT    (12)      /* Bits 12-13: ID 38 configuration */
#define GIC_ICDICFR2_ID38_MASK     (3 << GIC_ICDICFR2_ID38_SHIFT)
#  define GIC_ICDICFR2_ID38(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID38_SHIFT)
#define GIC_ICDICFR2_ID39_SHIFT    (14)      /* Bits 14-15: ID 39 configuration */
#define GIC_ICDICFR2_ID39_MASK     (3 << GIC_ICDICFR2_ID39_SHIFT)
#  define GIC_ICDICFR2_ID39(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID39_SHIFT)
#define GIC_ICDICFR2_ID40_SHIFT    (16)      /* Bits 16-17: ID 40 configuration */
#define GIC_ICDICFR2_ID40_MASK     (3 << GIC_ICDICFR2_ID40_SHIFT)
#  define GIC_ICDICFR2_ID40(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID40_SHIFT)
#define GIC_ICDICFR2_ID41_SHIFT    (18)      /* Bits 18-19: ID 41 configuration */
#define GIC_ICDICFR2_ID41_MASK     (3 << GIC_ICDICFR2_ID41_SHIFT)
#  define GIC_ICDICFR2_ID41(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID41_SHIFT)
#define GIC_ICDICFR2_ID42_SHIFT    (20)      /* Bits 20-21: ID 42 configuration */
#define GIC_ICDICFR2_ID42_MASK     (3 << GIC_ICDICFR2_ID42_SHIFT)
#  define GIC_ICDICFR2_ID42(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID42_SHIFT)
#define GIC_ICDICFR2_ID43_SHIFT    (22)      /* Bits 22-23: ID 43 configuration */
#define GIC_ICDICFR2_ID43_MASK     (3 << GIC_ICDICFR2_ID43_SHIFT)
#  define GIC_ICDICFR2_ID43(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID43_SHIFT)
#define GIC_ICDICFR2_ID44_SHIFT    (24)      /* Bits 24-25: ID 44 configuration */
#define GIC_ICDICFR2_ID44_MASK     (3 << GIC_ICDICFR2_ID44_SHIFT)
#  define GIC_ICDICFR2_ID44(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID44_SHIFT)
#define GIC_ICDICFR2_ID45_SHIFT    (26)      /* Bits 26-27: ID 45 configuration */
#define GIC_ICDICFR2_ID45_MASK     (3 << GIC_ICDICFR2_ID45_SHIFT)
#  define GIC_ICDICFR2_ID45(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID45_SHIFT)
#define GIC_ICDICFR2_ID46_SHIFT    (28)      /* Bits 28-29: ID 46 configuration */
#define GIC_ICDICFR2_ID46_MASK     (3 << GIC_ICDICFR2_ID46_SHIFT)
#  define GIC_ICDICFR2_ID46(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID46_SHIFT)
#define GIC_ICDICFR2_ID47_SHIFT    (30)      /* Bits 30-31: ID 47 configuration */
#define GIC_ICDICFR2_ID47_MASK     (3 << GIC_ICDICFR2_ID47_SHIFT)
#  define GIC_ICDICFR2_ID47(n)     ((uint32_t)(n) << GIC_ICDICFR2_ID47_SHIFT)

#define GIC_ICDICFR3_ID48_SHIFT    (0)       /* Bits 0-1: ID 48 configuration */
#define GIC_ICDICFR3_ID48_MASK     (3 << GIC_ICDICFR3_ID48_SHIFT)
#  define GIC_ICDICFR3_ID48(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID48_SHIFT)
#define GIC_ICDICFR3_ID49_SHIFT    (2)       /* Bits 2-3: ID 49 configuration */
#define GIC_ICDICFR3_ID49_MASK     (3 << GIC_ICDICFR3_ID49_SHIFT)
#  define GIC_ICDICFR3_ID49(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID49_SHIFT)
#define GIC_ICDICFR3_ID50_SHIFT    (4)       /* Bits 4-5: ID 50 configuration */
#define GIC_ICDICFR3_ID50_MASK     (3 << GIC_ICDICFR3_ID50_SHIFT)
#  define GIC_ICDICFR3_ID50(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID50_SHIFT)
#define GIC_ICDICFR3_ID51_SHIFT    (6)       /* Bits 6-7: ID 51 configuration */
#define GIC_ICDICFR3_ID51_MASK     (3 << GIC_ICDICFR3_ID51_SHIFT)
#  define GIC_ICDICFR3_ID51(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID51_SHIFT)
#define GIC_ICDICFR3_ID52_SHIFT    (8)       /* Bits 8-9: ID 52 configuration */
#define GIC_ICDICFR3_ID52_MASK     (3 << GIC_ICDICFR3_ID52_SHIFT)
#  define GIC_ICDICFR3_ID52(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID52_SHIFT)
#define GIC_ICDICFR3_ID53_SHIFT    (10)      /* Bits 10-11: ID 53 configuration */
#define GIC_ICDICFR3_ID53_MASK     (3 << GIC_ICDICFR3_ID53_SHIFT)
#  define GIC_ICDICFR3_ID53(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID53_SHIFT)
#define GIC_ICDICFR3_ID54_SHIFT    (12)      /* Bits 12-13: ID 54 configuration */
#define GIC_ICDICFR3_ID54_MASK     (3 << GIC_ICDICFR3_ID54_SHIFT)
#  define GIC_ICDICFR3_ID54(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID54_SHIFT)
#define GIC_ICDICFR3_ID55_SHIFT    (14)      /* Bits 14-15: ID 55 configuration */
#define GIC_ICDICFR3_ID55_MASK     (3 << GIC_ICDICFR3_ID55_SHIFT)
#  define GIC_ICDICFR3_ID55(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID55_SHIFT)
#define GIC_ICDICFR3_ID56_SHIFT    (16)      /* Bits 16-17: ID 56 configuration */
#define GIC_ICDICFR3_ID56_MASK     (3 << GIC_ICDICFR3_ID56_SHIFT)
#  define GIC_ICDICFR3_ID56(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID56_SHIFT)
#define GIC_ICDICFR3_ID57_SHIFT    (18)      /* Bits 18-19: ID 57 configuration */
#define GIC_ICDICFR3_ID57_MASK     (3 << GIC_ICDICFR3_ID57_SHIFT)
#  define GIC_ICDICFR3_ID57(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID57_SHIFT)
#define GIC_ICDICFR3_ID58_SHIFT    (20)      /* Bits 20-21: ID 58 configuration */
#define GIC_ICDICFR3_ID58_MASK     (3 << GIC_ICDICFR3_ID58_SHIFT)
#  define GIC_ICDICFR3_ID58(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID58_SHIFT)
#define GIC_ICDICFR3_ID59_SHIFT    (22)      /* Bits 22-23: ID 59 configuration */
#define GIC_ICDICFR3_ID59_MASK     (3 << GIC_ICDICFR3_ID59_SHIFT)
#  define GIC_ICDICFR3_ID59(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID59_SHIFT)
#define GIC_ICDICFR3_ID60_SHIFT    (24)      /* Bits 24-25: ID 60 configuration */
#define GIC_ICDICFR3_ID60_MASK     (3 << GIC_ICDICFR3_ID60_SHIFT)
#  define GIC_ICDICFR3_ID60(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID60_SHIFT)
#define GIC_ICDICFR3_ID61_SHIFT    (26)      /* Bits 26-27: ID 61 configuration */
#define GIC_ICDICFR3_ID61_MASK     (3 << GIC_ICDICFR3_ID61_SHIFT)
#  define GIC_ICDICFR3_ID61(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID61_SHIFT)
#define GIC_ICDICFR3_ID62_SHIFT    (28)      /* Bits 28-29: ID 62 configuration */
#define GIC_ICDICFR3_ID62_MASK     (3 << GIC_ICDICFR3_ID62_SHIFT)
#  define GIC_ICDICFR3_ID62(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID62_SHIFT)
#define GIC_ICDICFR3_ID63_SHIFT    (30)      /* Bits 30-31: ID 63 configuration */
#define GIC_ICDICFR3_ID63_MASK     (3 << GIC_ICDICFR3_ID63_SHIFT)
#  define GIC_ICDICFR3_ID63(n)     ((uint32_t)(n) << GIC_ICDICFR3_ID63_SHIFT)

#define GIC_ICDICFR4_ID64_SHIFT    (0)       /* Bits 0-1: ID 64 configuration */
#define GIC_ICDICFR4_ID64_MASK     (3 << GIC_ICDICFR4_ID64_SHIFT)
#  define GIC_ICDICFR4_ID64(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID64_SHIFT)
#define GIC_ICDICFR4_ID65_SHIFT    (2)       /* Bits 2-3: ID 65 configuration */
#define GIC_ICDICFR4_ID65_MASK     (3 << GIC_ICDICFR4_ID65_SHIFT)
#  define GIC_ICDICFR4_ID65(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID65_SHIFT)
#define GIC_ICDICFR4_ID66_SHIFT    (4)       /* Bits 4-5: ID 66 configuration */
#define GIC_ICDICFR4_ID66_MASK     (3 << GIC_ICDICFR4_ID66_SHIFT)
#  define GIC_ICDICFR4_ID66(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID66_SHIFT)
#define GIC_ICDICFR4_ID67_SHIFT    (6)       /* Bits 6-7: ID 67 configuration */
#define GIC_ICDICFR4_ID67_MASK     (3 << GIC_ICDICFR4_ID67_SHIFT)
#  define GIC_ICDICFR4_ID67(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID67_SHIFT)
#define GIC_ICDICFR4_ID68_SHIFT    (8)       /* Bits 8-9: ID 68 configuration */
#define GIC_ICDICFR4_ID68_MASK     (3 << GIC_ICDICFR4_ID68_SHIFT)
#  define GIC_ICDICFR4_ID68(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID68_SHIFT)
#define GIC_ICDICFR4_ID69_SHIFT    (10)      /* Bits 10-11: ID 69 configuration */
#define GIC_ICDICFR4_ID69_MASK     (3 << GIC_ICDICFR4_ID69_SHIFT)
#  define GIC_ICDICFR4_ID69(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID69_SHIFT)
#define GIC_ICDICFR4_ID70_SHIFT    (12)      /* Bits 12-13: ID 70 configuration */
#define GIC_ICDICFR4_ID70_MASK     (3 << GIC_ICDICFR4_ID70_SHIFT)
#  define GIC_ICDICFR4_ID70(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID70_SHIFT)
#define GIC_ICDICFR4_ID71_SHIFT    (14)      /* Bits 14-15: ID 71 configuration */
#define GIC_ICDICFR4_ID71_MASK     (3 << GIC_ICDICFR4_ID71_SHIFT)
#  define GIC_ICDICFR4_ID71(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID71_SHIFT)
#define GIC_ICDICFR4_ID72_SHIFT    (16)      /* Bits 16-17: ID 72 configuration */
#define GIC_ICDICFR4_ID72_MASK     (3 << GIC_ICDICFR4_ID72_SHIFT)
#  define GIC_ICDICFR4_ID72(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID72_SHIFT)
#define GIC_ICDICFR4_ID73_SHIFT    (18)      /* Bits 18-19: ID 73 configuration */
#define GIC_ICDICFR4_ID73_MASK     (3 << GIC_ICDICFR4_ID73_SHIFT)
#  define GIC_ICDICFR4_ID73(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID73_SHIFT)
#define GIC_ICDICFR4_ID74_SHIFT    (20)      /* Bits 20-21: ID 74 configuration */
#define GIC_ICDICFR4_ID74_MASK     (3 << GIC_ICDICFR4_ID74_SHIFT)
#  define GIC_ICDICFR4_ID74(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID74_SHIFT)
#define GIC_ICDICFR4_ID75_SHIFT    (22)      /* Bits 22-23: ID 75 configuration */
#define GIC_ICDICFR4_ID75_MASK     (3 << GIC_ICDICFR4_ID75_SHIFT)
#  define GIC_ICDICFR4_ID75(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID75_SHIFT)
#define GIC_ICDICFR4_ID76_SHIFT    (24)      /* Bits 24-25: ID 76 configuration */
#define GIC_ICDICFR4_ID76_MASK     (3 << GIC_ICDICFR4_ID76_SHIFT)
#  define GIC_ICDICFR4_ID76(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID76_SHIFT)
#define GIC_ICDICFR4_ID77_SHIFT    (26)      /* Bits 26-27: ID 77 configuration */
#define GIC_ICDICFR4_ID77_MASK     (3 << GIC_ICDICFR4_ID77_SHIFT)
#  define GIC_ICDICFR4_ID77(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID77_SHIFT)
#define GIC_ICDICFR4_ID78_SHIFT    (28)      /* Bits 28-29: ID 78 configuration */
#define GIC_ICDICFR4_ID78_MASK     (3 << GIC_ICDICFR4_ID78_SHIFT)
#  define GIC_ICDICFR4_ID78(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID78_SHIFT)
#define GIC_ICDICFR4_ID79_SHIFT    (30)      /* Bits 30-31: ID 79 configuration */
#define GIC_ICDICFR4_ID79_MASK     (3 << GIC_ICDICFR4_ID79_SHIFT)
#  define GIC_ICDICFR4_ID79(n)     ((uint32_t)(n) << GIC_ICDICFR4_ID79_SHIFT)

#define GIC_ICDICFR5_ID80_SHIFT    (0)       /* Bits 0-1: ID 80 configuration */
#define GIC_ICDICFR5_ID80_MASK     (3 << GIC_ICDICFR5_ID80_SHIFT)
#  define GIC_ICDICFR5_ID80(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID80_SHIFT)
#define GIC_ICDICFR5_ID81_SHIFT    (2)       /* Bits 2-3: ID 81 configuration */
#define GIC_ICDICFR5_ID81_MASK     (3 << GIC_ICDICFR5_ID81_SHIFT)
#  define GIC_ICDICFR5_ID81(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID81_SHIFT)
#define GIC_ICDICFR5_ID82_SHIFT    (4)       /* Bits 4-5: ID 82 configuration */
#define GIC_ICDICFR5_ID82_MASK     (3 << GIC_ICDICFR5_ID82_SHIFT)
#  define GIC_ICDICFR5_ID82(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID82_SHIFT)
#define GIC_ICDICFR5_ID83_SHIFT    (6)       /* Bits 6-7: ID 83 configuration */
#define GIC_ICDICFR5_ID83_MASK     (3 << GIC_ICDICFR5_ID83_SHIFT)
#  define GIC_ICDICFR5_ID83(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID83_SHIFT)
#define GIC_ICDICFR5_ID84_SHIFT    (8)       /* Bits 8-9: ID 84 configuration */
#define GIC_ICDICFR5_ID84_MASK     (3 << GIC_ICDICFR5_ID84_SHIFT)
#  define GIC_ICDICFR5_ID84(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID84_SHIFT)
#define GIC_ICDICFR5_ID85_SHIFT    (10)      /* Bits 10-11: ID 85 configuration */
#define GIC_ICDICFR5_ID85_MASK     (3 << GIC_ICDICFR5_ID85_SHIFT)
#  define GIC_ICDICFR5_ID85(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID85_SHIFT)
#define GIC_ICDICFR5_ID86_SHIFT    (12)      /* Bits 12-13: ID 86 configuration */
#define GIC_ICDICFR5_ID86_MASK     (3 << GIC_ICDICFR5_ID86_SHIFT)
#  define GIC_ICDICFR5_ID86(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID86_SHIFT)
#define GIC_ICDICFR5_ID87_SHIFT    (14)      /* Bits 14-15: ID 87 configuration */
#define GIC_ICDICFR5_ID87_MASK     (3 << GIC_ICDICFR5_ID87_SHIFT)
#  define GIC_ICDICFR5_ID87(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID87_SHIFT)
#define GIC_ICDICFR5_ID88_SHIFT    (16)      /* Bits 16-17: ID 88 configuration */
#define GIC_ICDICFR5_ID88_MASK     (3 << GIC_ICDICFR5_ID88_SHIFT)
#  define GIC_ICDICFR5_ID88(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID88_SHIFT)
#define GIC_ICDICFR5_ID89_SHIFT    (18)      /* Bits 18-19: ID 89 configuration */
#define GIC_ICDICFR5_ID89_MASK     (3 << GIC_ICDICFR5_ID89_SHIFT)
#  define GIC_ICDICFR5_ID89(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID89_SHIFT)
#define GIC_ICDICFR5_ID90_SHIFT    (20)      /* Bits 20-21: ID 90 configuration */
#define GIC_ICDICFR5_ID90_MASK     (3 << GIC_ICDICFR5_ID90_SHIFT)
#  define GIC_ICDICFR5_ID90(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID90_SHIFT)
#define GIC_ICDICFR5_ID91_SHIFT    (22)      /* Bits 22-23: ID 91 configuration */
#define GIC_ICDICFR5_ID91_MASK     (3 << GIC_ICDICFR5_ID91_SHIFT)
#  define GIC_ICDICFR5_ID91(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID91_SHIFT)
#define GIC_ICDICFR5_ID92_SHIFT    (24)      /* Bits 24-25: ID 92 configuration */
#define GIC_ICDICFR5_ID92_MASK     (3 << GIC_ICDICFR5_ID92_SHIFT)
#  define GIC_ICDICFR5_ID92(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID92_SHIFT)
#define GIC_ICDICFR5_ID93_SHIFT    (26)      /* Bits 26-27: ID 93 configuration */
#define GIC_ICDICFR5_ID93_MASK     (3 << GIC_ICDICFR5_ID93_SHIFT)
#  define GIC_ICDICFR5_ID93(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID93_SHIFT)
#define GIC_ICDICFR5_ID94_SHIFT    (28)      /* Bits 28-29: ID 94 configuration */
#define GIC_ICDICFR5_ID94_MASK     (3 << GIC_ICDICFR5_ID94_SHIFT)
#  define GIC_ICDICFR5_ID94(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID94_SHIFT)
#define GIC_ICDICFR5_ID95_SHIFT    (30)      /* Bits 30-31: ID 95 configuration */
#define GIC_ICDICFR5_ID95_MASK     (3 << GIC_ICDICFR5_ID95_SHIFT)
#  define GIC_ICDICFR5_ID95(n)     ((uint32_t)(n) << GIC_ICDICFR5_ID95_SHIFT)

/* PPI Status Register */

/* SPI Status Registers */

#define GIC_ICDPPSSR0_INT(n)       (1 << (n))      /* Bit n: Interrupt n status, n=0-31 */
#define GIC_ICDPPSSR1_INT(n)       (1 << ((n)-32)) /* Bit n: Interrupt n staut, n=32-63 */
#define GIC_ICDPPSSR2_INT(n)       (1 << ((n)-64)) /* Bit n: Interrupt n enable, n=64-95 */
#define GIC_ICDPPSSR3_INT(n)       (1 << ((n)-96)) /* Bit n: Interrupt n enable, n=69-127 */
#define GIC_ICDPPSSR4_INT(n)       (1 << ((n)-128)) /* Bit n: Interrupt n enable, n=128-159 */
#define GIC_ICDPPSSR5_INT(n)       (1 << ((n)-160)) /* Bit n: Interrupt n enable, n=160-191 */
#define GIC_ICDPPSSR6_INT(n)       (1 << ((n)-192)) /* Bit n: Interrupt n enable, n=192-223 */

/* Software Generated Interrupt Register */

#define GIC_ICDSGIR_INTID_SHIFT       (0)    /* Bits 0-9: Interrupt ID */
#define GIC_ICDSGIR_INTID_MASK        (0x3ff << GIC_ICDSGIR_INTID_SHIFT)
#  define GIC_ICDSGIR_INTID(n)        ((uint32_t)(n) << GIC_ICDSGIR_INTID_SHIFT)
                                             /* Bits 10-15: Reserved */
#define GIC_ICDSGIR_CPUTARGET_SHIFT   (16)   /* Bits 16-23: CPU target */
#define GIC_ICDSGIR_CPUTARGET_MASK    (0xff << GIC_ICDSGIR_CPUTARGET_SHIFT)
#  define GIC_ICDSGIR_CPUTARGET(n)    ((uint32_t)(n) << GIC_ICDSGIR_CPUTARGET_SHIFT)
                                             /* Bits 26-31: Reserved */
#define GIC_ICDSGIR_TGTFILTER_SHIFT   (24)   /* Bits 24-25: Target filter */
#define GIC_ICDSGIR_TGTFILTER_MASK    (3 << GIC_ICDSGIR_TGTFILTER_SHIFT)
#  define GIC_ICDSGIR_TGTFILTER_LIST  (0 << GIC_ICDSGIR_TGTFILTER_SHIFT) /* Interrupt sent to CPUs CPU target list */
#  define GIC_ICDSGIR_TGTFILTER_OTHER (1 << GIC_ICDSGIR_TGTFILTER_SHIFT) /* Interrupt is sent to all but requesting CPU */
#  define GIC_ICDSGIR_TGTFILTER_THIS  (2 << GIC_ICDSGIR_TGTFILTER_SHIFT) /* Interrupt is sent to requesting CPU only */

#endif /* __ARCH_ARM_SRC_ARMV7_A_GIC_H */
