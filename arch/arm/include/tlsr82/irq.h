/****************************************************************************
 * arch/arm/include/tlsr82/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_TLSR82_IRQ_H
#define __ARCH_ARM_INCLUDE_TLSR82_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Define this to prevent the arch irq.h include */

#define __ARCH_ARM_INCLUDE_ARMV6_M_IRQ_H

#define _REG_BASE_ADDR      0x00800000
#define _REG_ADDR8(a)       (*(volatile uint8_t *)(_REG_BASE_ADDR + (a)))
#define _REG_ADDR32(a)      (*(volatile uint32_t *)(_REG_BASE_ADDR + (a)))

#define _IRQ_MASK_REG       _REG_ADDR32(0x640)
#define _IRQ_EN_REG         _REG_ADDR8(0x643)

#define NR_TIMER0_IRQ       0
#define NR_TIMER1_IRQ       1
#define NR_TIMER2_IRQ       2
#define NR_USB_PWDN_IRQ     3
#define NR_DMA_IRQ          4
#define NR_DMA_FIFO_IRQ     5
#define NR_UART_IRQ         6
#define NR_MIX_CMD_IRQ      7

#define NR_EP0_SETUP_IRQ    8
#define NR_EP0_DATA_IRQ     9
#define NR_EP0_STA_IRQ      10
#define NR_SET_INTF_IRQ     11
#define NR_EP_DATA_IRQ      12
#define NR_RF_IRQ           13
#define NR_SW_PWM_IRQ       14
#define NR_PKE_IRQ          15

#define NR_USB_250US_IRQ    16
#define NR_USB_RST_IRQ      17
#define NR_GPIO_IRQ         18
#define NR_PM_IRQ           19
#define NR_SYSTEM_TIMER_IRQ 20
#define NR_GPIO_RISC0_IRQ   21
#define NR_GPIO_RISC1_IRQ   22

#define NR_IRQS             23

/* IRQ Stack Frame Format:
 *
 * Low Address  |
 *              |   regs --> 1  PC             (aka R15)
 *              |            1  SP             (aka R13)
 *              |            5  R12 ~ R8
 *              |            1  IRQ_STATE
 *              |            1  CPSR
 *              |            8  R7 ~ R0
 *              |            1  R14            (aka R14)
 * High Address v
 *
 * This results in the following set of indices that
 * can be used to access individual registers in the
 * xcp.regs array:
 */

#define REG_START           (0)
#define REG_R15             (0)
#define REG_R13             (1)
#define REG_R12             (2)
#define REG_R11             (3)
#define REG_R10             (4)
#define REG_R9              (5)
#define REG_R8              (6)
#define REG_IRQ_EN          (7)
#define REG_CPSR            (8)
#define REG_R7              (9)
#define REG_R6              (10)
#define REG_R5              (11)
#define REG_R4              (12)
#define REG_R3              (13)
#define REG_R2              (14)
#define REG_R1              (15)
#define REG_R0              (16)
#define REG_R14             (17)
#define REG_END             (18)

#define XCPTCONTEXT_REGS    (18)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

#define REG_FP              REG_R7
#define REG_IP              REG_R12
#define REG_SP              REG_R13
#define REG_LR              REG_R14
#define REG_PC              REG_R15

/* The PIC register is usually R10. It can be R9 is stack checking is enabled
 * or if the user changes it with -mpic-register on the GCC command line.
 */

#define REG_PIC             REG_R10

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct defines the way the registers are stored. We
 * need to save:
 *
 * Low Address  |
 *              |   regs --> 1  PC             (aka R15)
 *              |            1  SP             (aka R13)
 *              |            5  R12 ~ R8
 *              |            1  IRQ_STATE
 *              |            1  CPSR
 *              |            8  R7 ~ R0
 *              |            1  R14            (aka R14)
 * High Address v
 *
 * For a total of 18 (XCPTCONTEXT_REGS).
 *
 * Note: all the register are saved by software, hardware not
 * push register into the stack automatically when interrupt
 * occur.
 */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved register array pointer used during
   * signal processing.
   */

  uint32_t *saved_regs;

  /* Register save area */

  uint32_t *regs;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the
 * [get/set]_current_regs for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

extern volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: This function should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Save the current interrupt enable state & disable IRQs. */

static inline irqstate_t up_irq_save(void)
{
  irqstate_t r = _IRQ_EN_REG;
  _IRQ_EN_REG = 0;
  return r;
}

/* Restore saved IRQ & FIQ state */

static inline void up_irq_restore(irqstate_t flags)
{
  _IRQ_EN_REG = flags;
}

/* Enable IRQs and return the previous IRQ state */

static inline irqstate_t up_irq_enable(void)
{
  irqstate_t r = _IRQ_EN_REG;
  _IRQ_EN_REG = 1;
  return r;
}

static inline void up_irq_disable(void)
{
  up_irq_save();
}

static inline void up_disable_irq(int irq)
{
  _IRQ_MASK_REG &= ~(1 << irq);
}

static inline void up_enable_irq(int irq)
{
  _IRQ_MASK_REG |= (1 << irq);
}

static inline uint32_t getcontrol(void)
{
  return 0;
}

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return an index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int up_cpu_index(void) noinstrument_function;
#else
#  define up_cpu_index() 0
#endif /* CONFIG_SMP */

static inline_function uint32_t up_getsp(void)
{
  register uint32_t sp;

  __asm__ __volatile__
  (
    "tmov %0, sp\n"
    : "=r" (sp)
  );

  return sp;
}

noinstrument_function
static inline_function uint32_t *up_current_regs(void)
{
  return (uint32_t *)g_current_regs[up_cpu_index()];
}

noinstrument_function
static inline_function void up_set_current_regs(uint32_t *regs)
{
  g_current_regs[up_cpu_index()] = regs;
}

noinstrument_function
static inline_function bool up_interrupt_context(void)
{
#ifdef CONFIG_SMP
  irqstate_t flags = up_irq_save();
#endif

  bool ret = up_current_regs() != NULL;

#ifdef CONFIG_SMP
  up_irq_restore(flags);
#endif

  return ret;
}

#define up_switch_context(tcb, rtcb)                        \
  do {                                                      \
    if (!up_interrupt_context())                            \
      {                                                     \
        tc32_switchcontext(&rtcb->xcp.regs, tcb->xcp.regs); \
      }                                                     \
  } while (0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_TLSR82_IRQ_H */
