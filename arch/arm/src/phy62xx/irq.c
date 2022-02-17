/****************************************************************************
 * arch/arm/src/phy62xx/irq.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "nvic.h"
#include "arm_arch.h"
#include "arm_internal.h"
#include "jump_function.h"
#include "bus_dev.h"
/* #include "phy62xx_irq.h" */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | NVIC_SYSH_PRIORITY_DEFAULT)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy62xx_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void phy62xx_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();

  irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
  irqinfo("  ISER:       %08x ICER:   %08x\n",
          getreg32(ARMV6M_NVIC_ISER), getreg32(ARMV6M_NVIC_ICER));
  irqinfo("  ISPR:       %08x ICPR:   %08x\n",
          getreg32(ARMV6M_NVIC_ISPR), getreg32(ARMV6M_NVIC_ICPR));
  irqinfo("  IRQ PRIO:   %08x %08x %08x %08x\n",
          getreg32(ARMV6M_NVIC_IPR0), getreg32(ARMV6M_NVIC_IPR1),
          getreg32(ARMV6M_NVIC_IPR2), getreg32(ARMV6M_NVIC_IPR3));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(ARMV6M_NVIC_IPR4), getreg32(ARMV6M_NVIC_IPR5),
          getreg32(ARMV6M_NVIC_IPR6), getreg32(ARMV6M_NVIC_IPR7));

  irqinfo("SYSCON:\n");
  irqinfo("  CPUID:      %08x\n",
          getreg32(ARMV6M_SYSCON_CPUID));
  irqinfo("  ICSR:       %08x AIRCR:  %08x\n",
          getreg32(ARMV6M_SYSCON_ICSR), getreg32(ARMV6M_SYSCON_AIRCR));
  irqinfo("  SCR:        %08x CCR:    %08x\n",
          getreg32(ARMV6M_SYSCON_SCR), getreg32(ARMV6M_SYSCON_CCR));
  irqinfo("  SHPR2:      %08x SHPR3:  %08x\n",
          getreg32(ARMV6M_SYSCON_SHPR2), getreg32(ARMV6M_SYSCON_SHPR3));

  leave_critical_section(flags);
}

#else
#  define phy62xx_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: phy62xx_nmi, phy62xx_busfault, phy62xx_usagefault, phy62xx_pendsv,
 *       phy62xx_dbgmonitor, phy62xx_pendsv, phy62xx_reserved
 *
 * Description:
 *   Handlers for various exceptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int phy62xx_nmi(int irq, FAR void *context, FAR void *arg)
{
  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int phy62xx_pendsv(int irq, FAR void *context, FAR void *arg)
{
  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int phy62xx_reserved(int irq, FAR void *context, FAR void *arg)
{
  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: phy62xx_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.
 *
 ****************************************************************************/

static inline void phy62xx_clrpend(int irq)
{
  /* This will be called on each interrupt exit whether the interrupt can be
   * enambled or not.  So this assertion is necessarily lame.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);

  /* Check for an external interrupt */

  if (irq >= PHY62XX_IRQ_EXTINT && irq < (PHY62XX_IRQ_EXTINT + 32))
    {
      /* Set the appropriate bit in the ISER register to enable the
       * interrupt
       */

      putreg32((1 << (irq - PHY62XX_IRQ_EXTINT)), ARMV6M_NVIC_ICPR);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

extern void exception_common(void);
extern void exception_common_inline(void);

#define svc(code) asm volatile("svc %[immediate]"::[immediate]"I"(code))
#define SVC_CALL_WR 0

/* irqid 0~31 */

int irq_priority(int irqid, uint8_t priority)
{
#if 0
  uint32_t val = (uint32_t)(priority << 6);
  int idx = (irqid) / 4;
  int idx_mod = irqid % 4;
  uint32_t regaddr = ARMV6M_NVIC_IPR(idx);
  uint32_t regval = getreg32(regaddr);
  regval &= ~(0xfful << (idx_mod * 8));
  regval |= (val << (idx_mod * 8));
  putreg32(regval, regaddr);
#endif
  return 0;
}

void LL_IRQHandler1(void);
void TIM1_IRQHandler1(void);

#ifdef CONFIG_PHY6222_SDK
void TIM2_IRQHandler1(void);
#endif

void TIM3_IRQHandler1(void);

#ifdef CONFIG_PHY6222_SDK
void TIM5_IRQHandler1(void);
#endif

void up_irqinitialize(void)
{
  uint32_t regaddr;
  int i;

  /* Disable all interrupts */

  putreg32(0xffffffff, ARMV6M_NVIC_ICER);

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, ARMV6M_SYSCON_SHPR2);
  putreg32(DEFPRIORITY32, ARMV6M_SYSCON_SHPR3);

  /* Now set all of the interrupt lines to the default priority */

  for (i = 0; i < 8; i++)
    {
      regaddr = ARMV6M_NVIC_IPR(i);
      putreg32(DEFPRIORITY32, regaddr);
    }

  /* register jump table irq handler */

  JUMP_FUNCTION(NMI_HANDLER) = (uint32_t)&exception_common_inline;
  JUMP_FUNCTION(HARDFAULT_HANDLER) = (uint32_t)&exception_common;
  JUMP_FUNCTION(SVC_HANDLER) = (uint32_t)&exception_common_inline;
  JUMP_FUNCTION(PENDSV_HANDLER) = (uint32_t)&exception_common_inline;
  JUMP_FUNCTION(SYSTICK_HANDLER) = (uint32_t)&exception_common_inline;

  /* Vectors 16 - 47 external irq handler */

  JUMP_FUNCTION(V0_IRQ_HANDLER + 0) = (unsigned)&exception_common_inline,   /* 16+0  */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 1) = (unsigned)&exception_common_inline,   /* 16+1  */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 2) = (unsigned)&exception_common_inline,   /* 16+2  */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 3) = (unsigned)&exception_common_inline,   /* 16+3  */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 4) = (unsigned)&exception_common       ,   /* 16+4  */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 5) = (unsigned)&exception_common_inline,   /* 16+5  */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 6) = (unsigned)&exception_common_inline,   /* 16+6  */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 7) = (unsigned)&exception_common_inline,   /* 16+7  */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 8) = (unsigned)&exception_common_inline,   /* 16+8  */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 9) = (unsigned)&exception_common_inline,   /* 16+9  */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 10) = (unsigned)&exception_common_inline,  /* 16+10 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 11) = (unsigned)&exception_common       ,  /* 16+11 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 12) = (unsigned)&exception_common_inline,  /* 16+12 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 13) = (unsigned)&exception_common_inline,  /* 16+13 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 14) = (unsigned)&exception_common_inline,  /* 16+14 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 15) = (unsigned)&exception_common_inline,  /* 16+15 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 16) = (unsigned)&exception_common       ,  /* 16+16 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 17) = (unsigned)&exception_common_inline,  /* 16+17 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 18) = (unsigned)&exception_common       ,  /* 16+18 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 19) = (unsigned)&exception_common_inline,  /* 16+19 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 20) = (unsigned)&exception_common       ,  /* 16+20 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 21) = (unsigned)&exception_common       ,  /* 16+21 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 22) = (unsigned)&exception_common       ,  /* 16+22 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 23) = (unsigned)&exception_common       ,  /* 16+23 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 24) = (unsigned)&exception_common_inline,  /* 16+24 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 25) = (unsigned)&exception_common_inline,  /* 16+25 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 26) = (unsigned)&exception_common_inline,  /* 16+26 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 27) = (unsigned)&exception_common_inline,  /* 16+27 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 28) = (unsigned)&exception_common_inline,  /* 16+28 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 29) = (unsigned)&exception_common_inline,  /* 16+29 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 30) = (unsigned)&exception_common_inline,  /* 16+30 */
  JUMP_FUNCTION(V0_IRQ_HANDLER + 31) = (unsigned)&exception_common_inline,  /* 16+31 */

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(PHY62XX_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(PHY62XX_IRQ_HARDFAULT, arm_hardfault, NULL);

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(PHY62XX_IRQ_NMI, phy62xx_nmi, NULL);
  irq_attach(PHY62XX_IRQ_PENDSV, phy62xx_pendsv, NULL);
  irq_attach(PHY62XX_IRQ_RESERVED, phy62xx_reserved, NULL);
#endif

  phy62xx_dumpnvic("initial", NR_IRQS);

  /* Initialize logic to support a second level of interrupt decoding for
   * configured pin interrupts.
   */

#ifdef CONFIG_STM32F0L0G0_GPIOIRQ
  phy62xx_gpioirqinitialize();
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  irq_attach(PHY62XX_IRQ_BB_IRQn, (xcpt_t)LL_IRQHandler1, NULL);
  irq_attach(PHY62XX_IRQ_TIM1_IRQn, (xcpt_t)TIM1_IRQHandler1, NULL);

#ifdef CONFIG_PHY6222_SDK
  irq_attach(PHY62XX_IRQ_TIM2_IRQn, (xcpt_t)TIM2_IRQHandler1, NULL);
#endif

  irq_attach(PHY62XX_IRQ_TIM3_IRQn, (xcpt_t)TIM3_IRQHandler1, NULL);

#ifdef CONFIG_PHY6222_SDK
  irq_attach(PHY62XX_IRQ_TIM5_IRQn, (xcpt_t)TIM5_IRQHandler1, NULL);
#endif

  irq_priority((IRQn_Type)BB_IRQn,    IRQ_PRIO_REALTIME);
  irq_priority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     /* ll_EVT */

#ifdef CONFIG_PHY6222_SDK
  irq_priority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     /* OSAL_TICK */
#endif

  irq_priority((IRQn_Type)TIM3_IRQn,  IRQ_PRIO_APP);      /* OSAL_TICK */
  irq_priority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     /* LL_EXA_ADV */
  NVIC_EnableIRQ((IRQn_Type)BB_IRQn);
  NVIC_EnableIRQ((IRQn_Type)TIM1_IRQn);                   /* ll_EVT */

#ifdef CONFIG_PHY6222_SDK
  NVIC_EnableIRQ((IRQn_Type)TIM2_IRQn);
#endif

  NVIC_EnableIRQ((IRQn_Type)TIM3_IRQn);

#ifdef CONFIG_PHY6222_SDK
  NVIC_EnableIRQ((IRQn_Type)TIM5_IRQn);
#endif

  /* NVIC_EnableIRQ((IRQn_Type)TIM4_IRQn); */

  /* svc(SVC_CALL_WR); */

  up_irq_enable();

#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  DEBUGASSERT((unsigned)irq < NR_IRQS);

  /* Check for an external interrupt */

  if (irq >= PHY62XX_IRQ_EXTINT && irq < (PHY62XX_IRQ_EXTINT + 32))
    {
      /* Set the appropriate bit in the ICER register to disable the
       * interrupt
       */

      putreg32((1 << (irq - PHY62XX_IRQ_EXTINT)), ARMV6M_NVIC_ICER);
    }

  /* Handle processor exceptions.  Only SysTick can be disabled */

  else if (irq == PHY62XX_IRQ_SYSTICK)
    {
      modifyreg32(ARMV6M_SYSTICK_CSR, SYSTICK_CSR_ENABLE, 0);
    }

  phy62xx_dumpnvic("disable", irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  /* This will be called on each interrupt exit whether the interrupt can be
   * enabled or not.  So this assertion is necessarily lame.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= PHY62XX_IRQ_EXTINT && irq < (PHY62XX_IRQ_EXTINT + 32))
    {
      /* Set the appropriate bit in the ISER register to enable the
       * interrupt
       */

      putreg32((1 << (irq - PHY62XX_IRQ_EXTINT)), ARMV6M_NVIC_ISER);
    }

  /* Handle processor exceptions.  Only SysTick can be disabled */

  else if (irq == PHY62XX_IRQ_SYSTICK)
    {
      modifyreg32(ARMV6M_SYSTICK_CSR, 0, SYSTICK_CSR_ENABLE);
    }

  phy62xx_dumpnvic("enable", irq);
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
  phy62xx_clrpend(irq);
}
