/************************************************************************************
 * arch/sh/include/sh1/irq.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_SH_INCLUDE_SH1_IRQ_H
#define __ARCH_SH_INCLUDE_SH1_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* IRQ channels */

#define STR71X_IRQ_T0TIMI     (0)
#define STR71X_IRQ_FLASH      (1)
#define STR71X_IRQ_RCCU       (2)
#define STR71X_IRQ_RTC        (3)
#define STR71X_IRQ_WDG        (4)
#define STR71X_IRQ_XTI        (5)
#define STR71X_IRQ_USBHP      (6)
#define STR71X_IRQ_I2C0ITERR  (7)
#define STR71X_IRQ_I2C1ITERR  (8)
#define STR71X_IRQ_UART0      (9)
#define STR71X_IRQ_UART1     (10)
#define STR71X_IRQ_UART2     (11)
#define STR71X_IRQ_UART3     (12)
#define STR71X_IRQ_SPI0      (13)
#define STR71X_IRQ_SPI1      (14)
#define STR71X_IRQ_I2C0      (15)
#define STR71X_IRQ_I2C1      (16)
#define STR71X_IRQ_CAN       (17)
#define STR71X_IRQ_ADC       (18)
#define STR71X_IRQ_T1TIMI    (19)
#define STR71X_IRQ_T2TIMI    (20)
#define STR71X_IRQ_T3TIMI    (21)
#define STR71X_IRQ_HDLC      (25)
#define STR71X_IRQ_USBLP     (26)
#define STR71X_IRQ_T0TOI     (29)
#define STR71X_IRQ_T0OC1     (30)
#define STR71X_IRQ_T0OC2     (31)

#define STR71X_IRQ_SYSTIMER  STR71X_IRQ_T0TIMI
#define NR_IRQS               32

/* FIQ channels */

#define STR71X_FIQ_T0TIMI     (0X00000001)
#define STR71X_FIQ_WDG        (0X00000002)
#define STR71X_FIQ_WDGT0TIMIS (0X00000003)

/* Vector table offets **************************************************************/

/* Resets */

#define SH1_PWRONPC_VECOFFSET  (0*4)   /* 0: Power-on reset (hard, NMI high) PC*/
#define SH1_PWRONSP_VECOFFSET  (1*4)   /* 1: Power-on reset (hard, NMI high) SP */
#define SH1_MRESETPC_VECOFFSET (2*4)   /* 2: Power-on reset (hard, NMI high) PC*/
#define SH1_MRESETSP_VECOFFSET (3*4)   /* 3: Power-on reset (hard, NMI high) SP */

/* Illegal instructions / Address errors */

#define SH1_INVINSTR_VECOFFSET (4*4)   /* 4: General invalid instruction */
                                       /* 5: Reserved for system */
#define SH1_INVSLOT_VECOFFSET  (6*4)   /* 6: Invalid slot instruction */
                                       /* 7-8: Reserved for system */
#define SH1_BUSERR_VECOFFSET   (9*4)   /* 9: CPU bus error */
#define SH1_DMAERR_VECOFFSET   (10*4)  /* 10: DMA bus error */

/* NMI, user break */

#define SH1_NMI_VECOFFSET      (11*4)  /* 11: NMI */
#define SH1_USRBRK_VECOFFSET   (12*4)  /* 12: User break */
                                       /* 13-31: Reserved for system */
/* Trap instruction */

#define SH1_TRAP_VECOFFSET     (32*4)  /* 32-63: TRAPA instruction (user break) */
#define SH1_TRAP0_VECOFFSET    (32*4)  /* 32: TRAPA instruction (user break) */
#define SH1_TRAP1_VECOFFSET    (33*4)  /* 33: TRAPA instruction (user break) */
#define SH1_TRAP2_VECOFFSET    (34*4)  /* 34: TRAPA instruction (user break) */
#define SH1_TRAP3_VECOFFSET    (35*4)  /* 35: TRAPA instruction (user break) */
#define SH1_TRAP4_VECOFFSET    (36*4)  /* 36: TRAPA instruction (user break) */
#define SH1_TRAP5_VECOFFSET    (37*4)  /* 37: TRAPA instruction (user break) */
#define SH1_TRAP6_VECOFFSET    (38*4)  /* 38: TRAPA instruction (user break) */
#define SH1_TRAP7_VECOFFSET    (39*4)  /* 39: TRAPA instruction (user break) */
#define SH1_TRAP8_VECOFFSET    (40*4)  /* 40: TRAPA instruction (user break) */
#define SH1_TRAP9_VECOFFSET    (41*4)  /* 41: TRAPA instruction (user break) */
#define SH1_TRAP10_VECOFFSET   (42*4)  /* 42: TRAPA instruction (user break) */
#define SH1_TRAP11_VECOFFSET   (43*4)  /* 43: TRAPA instruction (user break) */
#define SH1_TRAP12_VECOFFSET   (44*4)  /* 44: TRAPA instruction (user break) */
#define SH1_TRAP13_VECOFFSET   (45*4)  /* 45: TRAPA instruction (user break) */
#define SH1_TRAP14_VECOFFSET   (46*4)  /* 46: TRAPA instruction (user break) */
#define SH1_TRAP15_VECOFFSET   (47*4)  /* 47: TRAPA instruction (user break) */
#define SH1_TRAP16_VECOFFSET   (48*4)  /* 48: TRAPA instruction (user break) */
#define SH1_TRAP17_VECOFFSET   (49*4)  /* 49: TRAPA instruction (user break) */
#define SH1_TRAP18_VECOFFSET   (50*4)  /* 50: TRAPA instruction (user break) */
#define SH1_TRAP19_VECOFFSET   (51*4)  /* 51: TRAPA instruction (user break) */
#define SH1_TRAP20_VECOFFSET   (52*4)  /* 52: TRAPA instruction (user break) */
#define SH1_TRAP21_VECOFFSET   (53*4)  /* 53: TRAPA instruction (user break) */
#define SH1_TRAP22_VECOFFSET   (54*4)  /* 54: TRAPA instruction (user break) */
#define SH1_TRAP23_VECOFFSET   (55*4)  /* 55: TRAPA instruction (user break) */
#define SH1_TRAP24_VECOFFSET   (56*4)  /* 56: TRAPA instruction (user break) */
#define SH1_TRAP25_VECOFFSET   (57*4)  /* 57: TRAPA instruction (user break) */
#define SH1_TRAP26_VECOFFSET   (58*4)  /* 58: TRAPA instruction (user break) */
#define SH1_TRAP27_VECOFFSET   (59*4)  /* 59: TRAPA instruction (user break) */
#define SH1_TRAP28_VECOFFSET   (60*4)  /* 60: TRAPA instruction (user break) */
#define SH1_TRAP29_VECOFFSET   (61*4)  /* 61: TRAPA instruction (user break) */
#define SH1_TRAP30_VECOFFSET   (62*4)  /* 62: TRAPA instruction (user break) */
#define SH1_TRAP31_VECOFFSET   (63*4)  /* 63: TRAPA instruction (user break) */

/* Interrupts */

#define SH1_IRQ_VECOFFSET      (64*4)  /* 64-71: IRQ0-7 */
#define SH1_IRQ0_VECOFFSET     (64*4)  /* 64: IRQ0 */
#define SH1_IRQ1_VECOFFSET     (65*4)  /* 65: IRQ1 */
#define SH1_IRQ2_VECOFFSET     (66*4)  /* 66: IRQ2 */
#define SH1_IRQ3_VECOFFSET     (67*4)  /* 67: IRQ3 */
#define SH1_IRQ4_VECOFFSET     (68*4)  /* 68: IRQ4 */
#define SH1_IRQ5_VECOFFSET     (69*4)  /* 69: IRQ5 */
#define SH1_IRQ6_VECOFFSET     (70*4)  /* 70: IRQ6 */
#define SH1_IRQ7_VECOFFSET     (71*4)  /* 71: IRQ7 */

/* On-chip modules -- The following may be unique to the 7032 */

#ifdef CONFIG_ARCH_SH7032

/* DMAC */

#define SH1_DMAC0_VECOFFSET    (72*4)  /* 72-73: DMAC0 */
#define SH1_DEI0_VECOFFSET     (72*4)  /* 72: DMAC0 DEI0 */
                                       /* 73: Reserved */
#define SH1_DMAC1_VECOFFSET    (74*4)  /* 74-75: DMAC1 */
#define SH1_DEI1_VECOFFSET     (74*4)  /* 74: DMAC1 DEI1 */
                                       /* 75: Reserved */
#define SH1_DMAC2_VECOFFSET    (76*4)  /* 76-77: DMAC2 */
#define SH1_DEI2_VECOFFSET     (76*4)  /* 76: DMAC2 DEI2 */
                                       /* 77: Reserved */
#define SH1_DMAC3_VECOFFSET    (78*4)  /* 78-79: DMAC3 */
#define SH1_DEI3_VECOFFSET     (78*4)  /* 78: DMAC3 DEI3 */
                                       /* 79: Reserved */
/* ITU */

#define SH1_IMIA0_VECOFFSET    (80*4)  /* 80: ITU0 IMIA0 */
#define SH1_IMIBO_VECOFFSET    (81*4)  /* 81:      IMIB0 */
#define SH1_OVI0_VECOFFSET     (82*4)  /* 82:      OVI0 */
                                       /* 83:      Reserved */
#define SH1_IMIA1_VECOFFSET    (84*4)  /* 84: ITU1 IMIA1 */
#define SH1_IMIB1_VECOFFSET    (85*4)  /* 85:      IMIB1 */
#define SH1_OVI1_VECOFFSET     (86*4)  /* 86:      OVI1 */
                                       /* 87:      Reserved */
#define SH1_IMIA2_VECOFFSET    (88*4)  /* 88: ITU2 IMIA2 */
#define SH1_IMIB2_VECOFFSET    (89*4)  /* 89:      IMIB2 */
#define SH1_OVI2_VECOFFSET     (90*4)  /* 90:      OVI2 */
                                       /* 91:      Reserved */
#define SH1_IMIA3_VECOFFSET    (92*4)  /* 92: ITU3 IMIA3 */
#define SH1_IMIB3_VECOFFSET    (93*4)  /* 93:      IMIB3 */
#define SH1_OVI3_VECOFFSET     (94*4)  /* 94:      OVI3 */
                                       /* 95:      Reserved */
#define SH1_IMIA4_VECOFFSET    (96*4)  /* 96: ITU4 IMIA4 */
#define SH1_IMIB4_VECOFFSET    (97*4)  /* 97:      IMIB4 */
#define SH1_OVI4_VECOFFSET     (98*4)  /* 98:      OVI4 */
                                       /* 99:      Reserved */
/* SCI */

#define SH1_ERI0_VECOFFET      (100*4) /* 100: SCI0 ERI0 */
#define SH1_RXI0_VECOFFET      (101*4) /* 101:      RxI0 */
#define SH1_TXI0_VECOFFET      (102*4) /* 102:      TxI0 */
#define SH1_TEI0_VECOFFET      (103*4) /* 103:      TEI0 */

#define SH1_ERI1_VECOFFET      (104*4) /* 104: SCI1 ERI1 */
#define SH1_RXI1_VECOFFET      (105*4) /* 105:      RxI1 */
#define SH1_TXI1_VECOFFET      (106*4) /* 106:      TxI1 */
#define SH1_TEI1_VECOFFET      (107*4) /* 107:      TEI1 */

#define SH1_PEI_VECOFFSET      (108*4) /* 108: Parity control unit PEI */
#define SH1_ADITI_VECOFFSET    (109*4) /* 109: A/D ITI */
                                       /* 110-111: Reserved */
#define SH1_WDTITI_VECOFFSET   (112*4) /* 112: WDT ITI */
#define SH1_CMI_VECOFFSET      (113*4) /* 113: REF CMI */
                                       /* 114-115: Reserved */
/* 116-255 reserved */
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/* Return the current interrupt enable state & disable IRQs */

static inline irqstate_t irqsave(void)
{
  irqstate_t flags;
  uint32 tmp;

  __asm__ __volatile__
    (
      "stc     sr, %1\n\t"
      "mov     %1, %0\n\t"
      "or      #0xf0, %0\n\t"
      "ldc     %0, sr\n\t"
      "mov     %1, %0\n\t"
      "and     #0xf0, %0"
      : "=&z" (flags), "=&r" (tmp)
      :
      : "memory"
    );
  return flags;
}

/* Disable IRQs */

static inline void irqdisable(void)
{
  unsigned long tmp;

  __asm__ __volatile__
    (
      "stc     sr, %0\n\t"
      "or      #0xf0, %0\n\t"
      "ldc     %0, sr"
      : "=&z" (tmp)
      :
      : "memory"
    );
}
/* Enable IRQs */

static inline void irqenable(void)
{
  uint32 tmp1;
  uint32 tmp2;

  __asm__ __volatile__
    (
      "stc     sr, %0\n\t"
      "and     %1, %0\n\t"
      "stc     r6_bank, %1\n\t"
      "or      %1, %0\n\t"
      "ldc     %0, sr"
      : "=&r" (tmp1), "=r" (tmp2)
      : "1" (~0x000000f0)
      : "memory"
    );
}

/* Restore saved IRQ state */

static inline void irqrestore(irqstate_t flags)
{
  if ((flags & 0x000000f0) != 0x000000f0)
    {
      irqenable();
    }
  else
    {
      irqdisable();
    }
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_SH_INCLUDE_SH1_IRQ_H */

