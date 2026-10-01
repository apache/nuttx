/****************************************************************************
 * arch/arm/include/ra8m1/ra8m1_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_RA8M1_RA8M1_IRQ_H
#define __ARCH_ARM_INCLUDE_RA8M1_RA8M1_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <arch/ra8m1/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Vector number of the first external interrupt.  Defined here, ahead of
 * the event enumeration below that is based on it.
 */

#define RA_IRQ_FIRST          (16)

/* Total number of IRQ numbers.
 *
 * Every ICU IELSR slot is runtime-configurable -- any peripheral event
 * can be routed to any slot -- so RA_IRQ_IELSRn below are just the raw
 * vector numbers; RA_IRQ_NEXTINT is the total slot count.
 */

#define RA_IRQ_IELSR0 (RA_IRQ_FIRST + 0)    /* 0: Event selected in the ICU.IELSR0 register */
#define RA_IRQ_IELSR1 (RA_IRQ_FIRST + 1)    /* 1: Event selected in the ICU.IELSR1 register */
#define RA_IRQ_IELSR2 (RA_IRQ_FIRST + 2)    /* 2: Event selected in the ICU.IELSR2 register */
#define RA_IRQ_IELSR3 (RA_IRQ_FIRST + 3)    /* 3: Event selected in the ICU.IELSR3 register */
#define RA_IRQ_IELSR4 (RA_IRQ_FIRST + 4)    /* 4: Event selected in the ICU.IELSR4 register */
#define RA_IRQ_IELSR5 (RA_IRQ_FIRST + 5)    /* 5: Event selected in the ICU.IELSR5 register */
#define RA_IRQ_IELSR6 (RA_IRQ_FIRST + 6)    /* 6: Event selected in the ICU.IELSR6 register */
#define RA_IRQ_IELSR7 (RA_IRQ_FIRST + 7)    /* 7: Event selected in the ICU.IELSR7 register */
#define RA_IRQ_IELSR8 (RA_IRQ_FIRST + 8)    /* 8: Event selected in the ICU.IELSR8 register */
#define RA_IRQ_IELSR9 (RA_IRQ_FIRST + 9)    /* 9: Event selected in the ICU.IELSR9 register */
#define RA_IRQ_IELSR10 (RA_IRQ_FIRST + 10)  /* 10: Event selected in the ICU.IELSR10 register */
#define RA_IRQ_IELSR11 (RA_IRQ_FIRST + 11)  /* 11: Event selected in the ICU.IELSR11 register */
#define RA_IRQ_IELSR12 (RA_IRQ_FIRST + 12)  /* 12: Event selected in the ICU.IELSR12 register */
#define RA_IRQ_IELSR13 (RA_IRQ_FIRST + 13)  /* 13: Event selected in the ICU.IELSR13 register */
#define RA_IRQ_IELSR14 (RA_IRQ_FIRST + 14)  /* 14: Event selected in the ICU.IELSR14 register */
#define RA_IRQ_IELSR15 (RA_IRQ_FIRST + 15)  /* 15: Event selected in the ICU.IELSR15 register */
#define RA_IRQ_IELSR16 (RA_IRQ_FIRST + 16)  /* 16: Event selected in the ICU.IELSR16 register */
#define RA_IRQ_IELSR17 (RA_IRQ_FIRST + 17)  /* 17: Event selected in the ICU.IELSR17 register */
#define RA_IRQ_IELSR18 (RA_IRQ_FIRST + 18)  /* 18: Event selected in the ICU.IELSR18 register */
#define RA_IRQ_IELSR19 (RA_IRQ_FIRST + 19)  /* 19: Event selected in the ICU.IELSR19 register */
#define RA_IRQ_IELSR20 (RA_IRQ_FIRST + 20)  /* 20: Event selected in the ICU.IELSR20 register */
#define RA_IRQ_IELSR21 (RA_IRQ_FIRST + 21)  /* 21: Event selected in the ICU.IELSR21 register */
#define RA_IRQ_IELSR22 (RA_IRQ_FIRST + 22)  /* 22: Event selected in the ICU.IELSR22 register */
#define RA_IRQ_IELSR23 (RA_IRQ_FIRST + 23)  /* 23: Event selected in the ICU.IELSR23 register */
#define RA_IRQ_IELSR24 (RA_IRQ_FIRST + 24)  /* 24: Event selected in the ICU.IELSR24 register */
#define RA_IRQ_IELSR25 (RA_IRQ_FIRST + 25)  /* 25: Event selected in the ICU.IELSR25 register */
#define RA_IRQ_IELSR26 (RA_IRQ_FIRST + 26)  /* 26: Event selected in the ICU.IELSR26 register */
#define RA_IRQ_IELSR27 (RA_IRQ_FIRST + 27)  /* 27: Event selected in the ICU.IELSR27 register */
#define RA_IRQ_IELSR28 (RA_IRQ_FIRST + 28)  /* 28: Event selected in the ICU.IELSR28 register */
#define RA_IRQ_IELSR29 (RA_IRQ_FIRST + 29)  /* 29: Event selected in the ICU.IELSR29 register */
#define RA_IRQ_IELSR30 (RA_IRQ_FIRST + 30)  /* 30: Event selected in the ICU.IELSR30 register */
#define RA_IRQ_IELSR31 (RA_IRQ_FIRST + 31)  /* 31: Event selected in the ICU.IELSR31 register */
#define RA_IRQ_IELSR32 (RA_IRQ_FIRST + 32)  /* 32: Event selected in the ICU.IELSR32 register */
#define RA_IRQ_IELSR33 (RA_IRQ_FIRST + 33)  /* 33: Event selected in the ICU.IELSR33 register */
#define RA_IRQ_IELSR34 (RA_IRQ_FIRST + 34)  /* 34: Event selected in the ICU.IELSR34 register */
#define RA_IRQ_IELSR35 (RA_IRQ_FIRST + 35)  /* 35: Event selected in the ICU.IELSR35 register */
#define RA_IRQ_IELSR36 (RA_IRQ_FIRST + 36)  /* 36: Event selected in the ICU.IELSR36 register */
#define RA_IRQ_IELSR37 (RA_IRQ_FIRST + 37)  /* 37: Event selected in the ICU.IELSR37 register */
#define RA_IRQ_IELSR38 (RA_IRQ_FIRST + 38)  /* 38: Event selected in the ICU.IELSR38 register */
#define RA_IRQ_IELSR39 (RA_IRQ_FIRST + 39)  /* 39: Event selected in the ICU.IELSR39 register */
#define RA_IRQ_IELSR40 (RA_IRQ_FIRST + 40)  /* 40: Event selected in the ICU.IELSR40 register */
#define RA_IRQ_IELSR41 (RA_IRQ_FIRST + 41)  /* 41: Event selected in the ICU.IELSR41 register */
#define RA_IRQ_IELSR42 (RA_IRQ_FIRST + 42)  /* 42: Event selected in the ICU.IELSR42 register */
#define RA_IRQ_IELSR43 (RA_IRQ_FIRST + 43)  /* 43: Event selected in the ICU.IELSR43 register */
#define RA_IRQ_IELSR44 (RA_IRQ_FIRST + 44)  /* 44: Event selected in the ICU.IELSR44 register */
#define RA_IRQ_IELSR45 (RA_IRQ_FIRST + 45)  /* 45: Event selected in the ICU.IELSR45 register */
#define RA_IRQ_IELSR46 (RA_IRQ_FIRST + 46)  /* 46: Event selected in the ICU.IELSR46 register */
#define RA_IRQ_IELSR47 (RA_IRQ_FIRST + 47)  /* 47: Event selected in the ICU.IELSR47 register */
#define RA_IRQ_IELSR48 (RA_IRQ_FIRST + 48)  /* 48: Event selected in the ICU.IELSR48 register */
#define RA_IRQ_IELSR49 (RA_IRQ_FIRST + 49)  /* 49: Event selected in the ICU.IELSR49 register */
#define RA_IRQ_IELSR50 (RA_IRQ_FIRST + 50)  /* 50: Event selected in the ICU.IELSR50 register */
#define RA_IRQ_IELSR51 (RA_IRQ_FIRST + 51)  /* 51: Event selected in the ICU.IELSR51 register */
#define RA_IRQ_IELSR52 (RA_IRQ_FIRST + 52)  /* 52: Event selected in the ICU.IELSR52 register */
#define RA_IRQ_IELSR53 (RA_IRQ_FIRST + 53)  /* 53: Event selected in the ICU.IELSR53 register */
#define RA_IRQ_IELSR54 (RA_IRQ_FIRST + 54)  /* 54: Event selected in the ICU.IELSR54 register */
#define RA_IRQ_IELSR55 (RA_IRQ_FIRST + 55)  /* 55: Event selected in the ICU.IELSR55 register */
#define RA_IRQ_IELSR56 (RA_IRQ_FIRST + 56)  /* 56: Event selected in the ICU.IELSR56 register */
#define RA_IRQ_IELSR57 (RA_IRQ_FIRST + 57)  /* 57: Event selected in the ICU.IELSR57 register */
#define RA_IRQ_IELSR58 (RA_IRQ_FIRST + 58)  /* 58: Event selected in the ICU.IELSR58 register */
#define RA_IRQ_IELSR59 (RA_IRQ_FIRST + 59)  /* 59: Event selected in the ICU.IELSR59 register */
#define RA_IRQ_IELSR60 (RA_IRQ_FIRST + 60)  /* 60: Event selected in the ICU.IELSR60 register */
#define RA_IRQ_IELSR61 (RA_IRQ_FIRST + 61)  /* 61: Event selected in the ICU.IELSR61 register */
#define RA_IRQ_IELSR62 (RA_IRQ_FIRST + 62)  /* 62: Event selected in the ICU.IELSR62 register */
#define RA_IRQ_IELSR63 (RA_IRQ_FIRST + 63)  /* 63: Event selected in the ICU.IELSR63 register */
#define RA_IRQ_IELSR64 (RA_IRQ_FIRST + 64)  /* 64: Event selected in the ICU.IELSR64 register */
#define RA_IRQ_IELSR65 (RA_IRQ_FIRST + 65)  /* 65: Event selected in the ICU.IELSR65 register */
#define RA_IRQ_IELSR66 (RA_IRQ_FIRST + 66)  /* 66: Event selected in the ICU.IELSR66 register */
#define RA_IRQ_IELSR67 (RA_IRQ_FIRST + 67)  /* 67: Event selected in the ICU.IELSR67 register */
#define RA_IRQ_IELSR68 (RA_IRQ_FIRST + 68)  /* 68: Event selected in the ICU.IELSR68 register */
#define RA_IRQ_IELSR69 (RA_IRQ_FIRST + 69)  /* 69: Event selected in the ICU.IELSR69 register */
#define RA_IRQ_IELSR70 (RA_IRQ_FIRST + 70)  /* 70: Event selected in the ICU.IELSR70 register */
#define RA_IRQ_IELSR71 (RA_IRQ_FIRST + 71)  /* 71: Event selected in the ICU.IELSR71 register */
#define RA_IRQ_IELSR72 (RA_IRQ_FIRST + 72)  /* 72: Event selected in the ICU.IELSR72 register */
#define RA_IRQ_IELSR73 (RA_IRQ_FIRST + 73)  /* 73: Event selected in the ICU.IELSR73 register */
#define RA_IRQ_IELSR74 (RA_IRQ_FIRST + 74)  /* 74: Event selected in the ICU.IELSR74 register */
#define RA_IRQ_IELSR75 (RA_IRQ_FIRST + 75)  /* 75: Event selected in the ICU.IELSR75 register */
#define RA_IRQ_IELSR76 (RA_IRQ_FIRST + 76)  /* 76: Event selected in the ICU.IELSR76 register */
#define RA_IRQ_IELSR77 (RA_IRQ_FIRST + 77)  /* 77: Event selected in the ICU.IELSR77 register */
#define RA_IRQ_IELSR78 (RA_IRQ_FIRST + 78)  /* 78: Event selected in the ICU.IELSR78 register */
#define RA_IRQ_IELSR79 (RA_IRQ_FIRST + 79)  /* 79: Event selected in the ICU.IELSR79 register */
#define RA_IRQ_IELSR80 (RA_IRQ_FIRST + 80)  /* 80: Event selected in the ICU.IELSR80 register */
#define RA_IRQ_IELSR81 (RA_IRQ_FIRST + 81)  /* 81: Event selected in the ICU.IELSR81 register */
#define RA_IRQ_IELSR82 (RA_IRQ_FIRST + 82)  /* 82: Event selected in the ICU.IELSR82 register */
#define RA_IRQ_IELSR83 (RA_IRQ_FIRST + 83)  /* 83: Event selected in the ICU.IELSR83 register */
#define RA_IRQ_IELSR84 (RA_IRQ_FIRST + 84)  /* 84: Event selected in the ICU.IELSR84 register */
#define RA_IRQ_IELSR85 (RA_IRQ_FIRST + 85)  /* 85: Event selected in the ICU.IELSR85 register */
#define RA_IRQ_IELSR86 (RA_IRQ_FIRST + 86)  /* 86: Event selected in the ICU.IELSR86 register */
#define RA_IRQ_IELSR87 (RA_IRQ_FIRST + 87)  /* 87: Event selected in the ICU.IELSR87 register */
#define RA_IRQ_IELSR88 (RA_IRQ_FIRST + 88)  /* 88: Event selected in the ICU.IELSR88 register */
#define RA_IRQ_IELSR89 (RA_IRQ_FIRST + 89)  /* 89: Event selected in the ICU.IELSR89 register */
#define RA_IRQ_IELSR90 (RA_IRQ_FIRST + 90)  /* 90: Event selected in the ICU.IELSR90 register */
#define RA_IRQ_IELSR91 (RA_IRQ_FIRST + 91)  /* 91: Event selected in the ICU.IELSR91 register */
#define RA_IRQ_IELSR92 (RA_IRQ_FIRST + 92)  /* 92: Event selected in the ICU.IELSR92 register */
#define RA_IRQ_IELSR93 (RA_IRQ_FIRST + 93)  /* 93: Event selected in the ICU.IELSR93 register */
#define RA_IRQ_IELSR94 (RA_IRQ_FIRST + 94)  /* 94: Event selected in the ICU.IELSR94 register */
#define RA_IRQ_IELSR95 (RA_IRQ_FIRST + 95)  /* 95: Event selected in the ICU.IELSR95 register */

#define RA_IRQ_NEXTINT        (96)

/* Peripheral interrupt events.
 *
 * The ICU can route any peripheral event to any IELSR slot.  Each event
 * that an enabled peripheral needs is given a slot here, packed from
 * RA_IRQ_FIRST in the order below, so only the events a build uses take a
 * slot (the ICU has RA_IRQ_NEXTINT of them).
 *
 * ra_attach_icu() (ra_icu.c) writes the matching event number to
 * IELSR(<irq> - RA_IRQ_FIRST) for each of these.
 *
 * To give a peripheral another event (for example a GPT compare match),
 * add an enumerator under its own Kconfig option and route it in
 * ra_attach_icu().
 */

#ifndef __ASSEMBLY__
enum ra_icu_event_irq_e
{
  RA_IRQ_EVT_START = RA_IRQ_FIRST - 1,  /* Not an IRQ: first event = FIRST */

#ifdef CONFIG_RA_SCI0_UART
  SCI0_RXI,                             /* Receive data full */
  SCI0_TXI,                             /* Transmit data empty */
  SCI0_TEI,                             /* Transmit end */
  SCI0_ERI,                             /* Receive error */
#endif

#ifdef CONFIG_RA_SCI1_UART
  SCI1_RXI,                             /* Receive data full */
  SCI1_TXI,                             /* Transmit data empty */
  SCI1_TEI,                             /* Transmit end */
  SCI1_ERI,                             /* Receive error */
#endif

#ifdef CONFIG_RA_SCI2_UART
  SCI2_RXI,                             /* Receive data full */
  SCI2_TXI,                             /* Transmit data empty */
  SCI2_TEI,                             /* Transmit end */
  SCI2_ERI,                             /* Receive error */
#endif

#ifdef CONFIG_RA_SCI3_UART
  SCI3_RXI,                             /* Receive data full */
  SCI3_TXI,                             /* Transmit data empty */
  SCI3_TEI,                             /* Transmit end */
  SCI3_ERI,                             /* Receive error */
#endif

#ifdef CONFIG_RA_SCI4_UART
  SCI4_RXI,                             /* Receive data full */
  SCI4_TXI,                             /* Transmit data empty */
  SCI4_TEI,                             /* Transmit end */
  SCI4_ERI,                             /* Receive error */
#endif

#ifdef CONFIG_RA_SCI9_UART
  SCI9_RXI,                             /* Receive data full */
  SCI9_TXI,                             /* Transmit data empty */
  SCI9_TEI,                             /* Transmit end */
  SCI9_ERI,                             /* Receive error */
#endif

  /* GPT: only the counter overflow, which is the timer expiry */

#ifdef CONFIG_RA_GPT0_GPT
  GPT0_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT1_GPT
  GPT1_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT2_GPT
  GPT2_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT3_GPT
  GPT3_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT4_GPT
  GPT4_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT5_GPT
  GPT5_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT6_GPT
  GPT6_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT7_GPT
  GPT7_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT8_GPT
  GPT8_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT9_GPT
  GPT9_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT10_GPT
  GPT10_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT11_GPT
  GPT11_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT12_GPT
  GPT12_COUNTER_OVERFLOW,
#endif
#ifdef CONFIG_RA_GPT13_GPT
  GPT13_COUNTER_OVERFLOW,
#endif

  RA_IRQ_EVT_END                        /* Not an IRQ: one past the last */
};
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
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

#endif /* __ARCH_ARM_INCLUDE_RA8M1_RA8M1_IRQ_H */
