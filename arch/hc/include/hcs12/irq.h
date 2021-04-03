/************************************************************************************
 * arch/hc/include/hcs12/irq.h
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
 ************************************************************************************/

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_HC_INCLUDE_HCS12_IRQ_H
#define __ARCH_HC_INCLUDE_HCS12_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* CCR bit definitions */

#define HCS12_CCR_C (1 << 0) /* Bit 0: Carry/Borrow status bit */
#define HCS12_CCR_V (1 << 1) /* Bit 1: Two’s complement overflow status bit */
#define HCS12_CCR_Z (1 << 2) /* Bit 2: Zero status bit */
#define HCS12_CCR_N (1 << 3) /* Bit 3: Negative status bit */
#define HCS12_CCR_I (1 << 4) /* Bit 4: Maskable interrupt control bit */
#define HCS12_CCR_H (1 << 5) /* Bit 5: Half-carry status bit */
#define HCS12_CCR_X (1 << 6) /* Bit 6: Non-maskable interrupt control bit */
#define HCS12_CCR_S (1 << 7) /* Bit 7: STOP instruction control bit */

/************************************************************************************
 * Register state save strucure
 *   Low Address        <-- SP after state save
 *                [PPAGE]
 *                [soft regisers]
 *                XYH
 *                XYL
 *                ZH
 *                ZL
 *                TMPH
 *                TMPL
 *                FRAMEH
 *                FRAMEL
 *                SP    <-- SP after interrupt
 *                CCR
 *                B
 *                A
 *                XH
 *                XL
 *                YH
 *                YL
 *                PCH
 *   High Address PCL    <-- SP before interrupt
 *
 ************************************************************************************/

/* Byte offsets */

/* PPAGE register (only in banked mode) */

#ifndef CONFIG_HCS12_NONBANKED
#  define REG_PPAGE          0
#  define REG_FIRST_SOFTREG  1
#else
#  define REG_FIRST_SOFTREG  0
#endif

/* Soft registers (as configured) */

#if CONFIG_HCS12_MSOFTREGS > 2
#  error "Need to save more registers"
#elif CONFIG_HCS12_MSOFTREGS == 2
#  define REG_SOFTREG1       REG_FIRST_SOFTREG
#  define REG_SOFTREG2       (REG_FIRST_SOFTREG+2)
#  define REG_FIRST_HARDREG  (REG_FIRST_SOFTREG+4)
#elif CONFIG_HCS12_MSOFTREGS == 1
#  define REG_SOFTREG1       REG_FIRST_SOFTREG
#  define REG_FIRST_HARDREG  (REG_FIRST_SOFTREG+2)
#else
#  define REG_FIRST_HARDREG  REG_FIRST_SOFTREG
#endif

#define REG_XY               REG_FIRST_HARDREG
#define REG_Z                (REG_FIRST_HARDREG+2)
#  define REG_ZH             (REG_FIRST_HARDREG+2)
#  define REG_ZL             (REG_FIRST_HARDREG+3)
#define REG_TMP              (REG_FIRST_HARDREG+4)
#  define REG_TMPH           (REG_FIRST_HARDREG+4)
#  define REG_TMPL           (REG_FIRST_HARDREG+5)
#define REG_FRAME            (REG_FIRST_HARDREG+6)
#  define REG_FRAMEH         (REG_FIRST_HARDREG+6)
#  define REG_FRAMEL         (REG_FIRST_HARDREG+7)

/* Stack pointer before the interrupt */

#define REG_SP               (REG_FIRST_HARDREG+8)
#  define REG_SPH            (REG_FIRST_HARDREG+8)
#  define REG_SPL            (REG_FIRST_HARDREG+9)

/* On entry into an I- or X-interrupt, into an SWI, or into an undefined instruction
 * interrupt, the stack frame created by hardware looks like:
 *
 * Low Address       <-- SP after interrupt
 *              CCR
 *              B
 *              A
 *              XH
 *              XL
 *              YH
 *              YL
 *              PCH
 * High Address PCL  <-- SP before interrupt
 */

#define REG_CCR              (REG_FIRST_HARDREG+10)
#define REG_BA               (REG_FIRST_HARDREG+11)
#  define REG_B              (REG_FIRST_HARDREG+11)
#  define REG_A              (REG_FIRST_HARDREG+12)
#define REG_X                (REG_FIRST_HARDREG+13)
#  define REG_XH             (REG_FIRST_HARDREG+13)
#  define REG_XL             (REG_FIRST_HARDREG+14)
#define REG_Y                (REG_FIRST_HARDREG+15)
#  define REG_YH             (REG_FIRST_HARDREG+15)
#  define REG_YL             (REG_FIRST_HARDREG+16)
#define REG_PC               (REG_FIRST_HARDREG+17)
#  define REG_PCH            (REG_FIRST_HARDREG+17)
#  define REG_PCL            (REG_FIRST_HARDREG+18)

#define TOTALFRAME_SIZE      (REG_FIRST_HARDREG+17)
#define INTFRAME_SIZE        9
#define XCPTCONTEXT_REGS     TOTALFRAME_SIZE

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* This structure defines the way the registers are stored. */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  uint8_t regs[XCPTCONTEXT_REGS];
};

/************************************************************************************
 * Inline functions
 ************************************************************************************/

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: This function should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Enable/Disable interrupts */

#define ienable()  __asm("cli");
#define idisable() __asm("orcc #0x10")
#define xenable()  __asm("andcc #0xbf")
#define xdisable() __asm("orcc #0x40")

/* Get the current value of the CCR */

static inline irqstate_t up_getccr(void)
{
  irqstate_t ccr;
  __asm__
  (
    "\ttpa\n"
    "\tstaa %0\n"
    : "=m"(ccr) :
  );
  return ccr;
}

/* Save the current interrupt enable state & disable IRQs */

static inline irqstate_t up_irq_save(void)
{
  irqstate_t ccr;
  __asm__
  (
    "\ttpa\n"
    "\tstaa %0\n"
    "\torcc #0x50\n"
    : "=m"(ccr) :
  );
  return ccr;
}

/* Restore saved interrupt state */

static inline void up_irq_restore(irqstate_t flags)
{
  /* Should interrupts be enabled? */

  if ((flags & HCS12_CCR_I) == 0)
    {
      /* Yes.. unmask I- and Z-interrupts */

      __asm("andcc #0xaf");
    }
}

/* System call */

static inline void system_call3(unsigned int nbr, uintptr_t parm1,
                                uintptr_t parm2, uintptr_t parm3)
{
  /* To be provided */

  /* __asm("swi") */
}

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions Prototypes
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_HC_INCLUDE_HCS12_IRQ_H */
