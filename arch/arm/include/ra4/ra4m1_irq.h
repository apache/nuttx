/****************************************************************************
 * arch/arm/include/ra4/ra4m1_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_RA_RA4M1_IRQ_H
#define __ARCH_ARM_INCLUDE_RA_RA4M1_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <arch/ra4/chip.h>
/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Total number of IRQ numbers */
#  define RA_IRQ_IELSR0       (RA_IRQ_FIRST + 0)   /* 0:  Event selected in the ICU.IELSR0 register */
#  define RA_IRQ_IELSR1       (RA_IRQ_FIRST + 1)   /* 1:  Event selected in the ICU.IELSR1 register */
#  define RA_IRQ_IELSR2       (RA_IRQ_FIRST + 2)   /* 2:  Event selected in the ICU.IELSR2 register */
#  define RA_IRQ_IELSR3       (RA_IRQ_FIRST + 3)   /* 3:  Event selected in the ICU.IELSR3 register */
#  define RA_IRQ_IELSR4       (RA_IRQ_FIRST + 4)   /* 4:  Event selected in the ICU.IELSR4 register */
#  define RA_IRQ_IELSR5       (RA_IRQ_FIRST + 5)   /* 5:  Event selected in the ICU.IELSR5 register */
#  define RA_IRQ_IELSR6       (RA_IRQ_FIRST + 6)   /* 6:  Event selected in the ICU.IELSR6 register */
#  define RA_IRQ_IELSR7       (RA_IRQ_FIRST + 7)   /* 7:  Event selected in the ICU.IELSR7 register */
#  define RA_IRQ_IELSR8       (RA_IRQ_FIRST + 8)   /* 8:  Event selected in the ICU.IELSR8 register */
#  define RA_IRQ_IELSR9       (RA_IRQ_FIRST + 9)   /* 9:  Event selected in the ICU.IELSR9 register */
#  define RA_IRQ_IELSR10      (RA_IRQ_FIRST + 10)  /* 10:  Event selected in the ICU.IELSR10 register */
#  define RA_IRQ_IELSR11      (RA_IRQ_FIRST + 11)  /* 11:  Event selected in the ICU.IELSR11 register */
#  define RA_IRQ_IELSR12      (RA_IRQ_FIRST + 12)  /* 12:  Event selected in the ICU.IELSR12 register */
#  define RA_IRQ_IELSR13      (RA_IRQ_FIRST + 13)  /* 13:  Event selected in the ICU.IELSR13 register */
#  define RA_IRQ_IELSR14      (RA_IRQ_FIRST + 14)  /* 14:  Event selected in the ICU.IELSR14 register */
#  define RA_IRQ_IELSR15      (RA_IRQ_FIRST + 15)  /* 15:  Event selected in the ICU.IELSR15 register */
#  define RA_IRQ_IELSR16      (RA_IRQ_FIRST + 16)  /* 16:  Event selected in the ICU.IELSR16 register */
#  define RA_IRQ_IELSR17      (RA_IRQ_FIRST + 17)  /* 17:  Event selected in the ICU.IELSR17 register */
#  define RA_IRQ_IELSR18      (RA_IRQ_FIRST + 18)  /* 18:  Event selected in the ICU.IELSR18 register */
#  define RA_IRQ_IELSR19      (RA_IRQ_FIRST + 19)  /* 19:  Event selected in the ICU.IELSR19 register */
#  define RA_IRQ_IELSR20      (RA_IRQ_FIRST + 20)  /* 20:  Event selected in the ICU.IELSR20 register */
#  define RA_IRQ_IELSR21      (RA_IRQ_FIRST + 21)  /* 21:  Event selected in the ICU.IELSR21 register */
#  define RA_IRQ_IELSR22      (RA_IRQ_FIRST + 22)  /* 22:  Event selected in the ICU.IELSR22 register */
#  define RA_IRQ_IELSR23      (RA_IRQ_FIRST + 23)  /* 23:  Event selected in the ICU.IELSR23 register */
#  define RA_IRQ_IELSR24      (RA_IRQ_FIRST + 24)  /* 24:  Event selected in the ICU.IELSR24 register */
#  define RA_IRQ_IELSR25      (RA_IRQ_FIRST + 25)  /* 25:  Event selected in the ICU.IELSR25 register */
#  define RA_IRQ_IELSR26      (RA_IRQ_FIRST + 26)  /* 26:  Event selected in the ICU.IELSR26 register */
#  define RA_IRQ_IELSR27      (RA_IRQ_FIRST + 27)  /* 27:  Event selected in the ICU.IELSR27 register */
#  define RA_IRQ_IELSR28      (RA_IRQ_FIRST + 28)  /* 28:  Event selected in the ICU.IELSR28 register */
#  define RA_IRQ_IELSR29      (RA_IRQ_FIRST + 29)  /* 29:  Event selected in the ICU.IELSR29 register */
#  define RA_IRQ_IELSR30      (RA_IRQ_FIRST + 30)  /* 30:  Event selected in the ICU.IELSR30 register */
#  define RA_IRQ_IELSR31      (RA_IRQ_FIRST + 31)  /* 31:  Event selected in the ICU.IELSR31 register */
#  define RA_IRQ_NEXTINT      (32)

#if (CONFIG_RA_SCI0_UART)
#define SCI0_RXI   (RA_IRQ_FIRST + __COUNTER__)  /* Receive data full */
#define SCI0_TXI   (RA_IRQ_FIRST + __COUNTER__)  /* Transmit data empty */
#define SCI0_TEI   (RA_IRQ_FIRST + __COUNTER__)  /* Transmit end */
#define SCI0_ERI   (RA_IRQ_FIRST + __COUNTER__)  /* Receive error */
#endif

#if (CONFIG_RA_SCI1_UART)
#define SCI1_RXI   (RA_IRQ_FIRST + __COUNTER__)  /* Receive data full */
#define SCI1_TXI   (RA_IRQ_FIRST + __COUNTER__)  /* Transmit data empty */
#define SCI1_TEI   (RA_IRQ_FIRST + __COUNTER__)  /* Transmit end */
#define SCI1_ERI   (RA_IRQ_FIRST + __COUNTER__)  /* Receive error */
#endif

#if (CONFIG_RA_SCI2_UART)
#define SCI2_RXI   (RA_IRQ_FIRST + __COUNTER__)  /* Receive data full */
#define SCI2_TXI   (RA_IRQ_FIRST + __COUNTER__)  /* Transmit data empty */
#define SCI2_TEI   (RA_IRQ_FIRST + __COUNTER__)  /* Transmit end */
#define SCI2_ERI   (RA_IRQ_FIRST + __COUNTER__)  /* Receive error */
#endif

#if (CONFIG_RA_SCI9_UART)
#define SCI9_RXI   (RA_IRQ_FIRST + __COUNTER__)  /* Receive data full */
#define SCI9_TXI   (RA_IRQ_FIRST + __COUNTER__)  /* Transmit data empty */
#define SCI9_TEI   (RA_IRQ_FIRST + __COUNTER__)  /* Transmit end */
#define SCI9_ERI   (RA_IRQ_FIRST + __COUNTER__)  /* Receive error */
#endif
/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
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
#endif

#endif /* __ARCH_ARM_INCLUDE_RA_RA_IRQ_H */
