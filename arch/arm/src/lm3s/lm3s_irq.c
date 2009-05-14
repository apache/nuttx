/****************************************************************************
 * arch/arm/src/lm3s/lm3s_irq.c
 * arch/arm/src/chip/lm3s_irq.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"
#include "lm3s_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 |\
   NVIC_SYSH_PRIORITY_DEFAULT << 16 |\
   NVIC_SYSH_PRIORITY_DEFAULT << 8  |\
   NVIC_SYSH_PRIORITY_DEFAULT)

/****************************************************************************
 * Public Data
 ****************************************************************************/

uint32 *current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lml3s_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int lml3s_irqinfo(int irq, uint32 *regaddr, uint32 *bit)
{
  DEBUGASSERT(irq >= LMSB_IRQ_MPU && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= LM3S_IRQ_INTERRUPTS)
    {
      if (irq < LM3S_IRQ_INTERRUPTS + 32)
        {
           *regaddr = NVIC_IRQ0_31_ENABLE;
           *bit     = 1 << (irq - LM3S_IRQ_INTERRUPTS);
        }
      else if (irq < NR_IRQS)
        {
           *regaddr = NVIC_IRQ32_63_ENABLE;
           *bit     = 1 << (irq - LM3S_IRQ_INTERRUPTS - 32);
        }
      else
        {
          return ERROR; /* Invalid interrupt */
        }
    }

  /* Handler processor exceptions.  Only a few can be disabled */

  else
    {
       *regaddr = NVIC_SYSHCON;
       if (irq == LMSB_IRQ_MPU)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == LMSB_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == LMSB_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == LMSB_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return ERROR; /* Invalid or unsupported exception */
        }
    }

  return OK;
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable all interrupts */

  putreg32(0, NVIC_IRQ0_31_ENABLE);
  putreg32(0, NVIC_IRQ32_63_ENABLE);

  /* Set all interrrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_IRQ0_3_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ12_15_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ16_19_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ20_23_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ24_27_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ28_31_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ32_35_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ36_39_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ40_43_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ44_47_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ48_51_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ52_55_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ56_59_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_IRQ60_63_PRIORITY);

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* Initialize support for GPIO interrupts if included in this build */

#ifndef CONFIG_LM3S_DISABLE_GPIO_IRQS
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (gpio_irqinitialize != NULL)
#endif
    {
      gpio_irqinitialize();
    }
#endif

  /* Attach the PendSV exception handler and set it to the minimum
   * prioirity.  The PendSV exception is used for performing
   * context switches.
   */

  irq_attach(LMSB_IRQ_PENDSV, lm3s_pendsv);
#ifdef CONFIG_ARCH_IRQPRIO
  up_prioritize_irq(LMSB_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG
  irq_attach(LMSB_IRQ_NMI, lm3s_nmi);
  irq_attach(LMSB_IRQ_HARDFAULT, lm3s_hardfault);
  irq_attach(LMSB_IRQ_MPU, lm3s_mpu);
  irq_attach(LMSB_IRQ_BUSFAULT, lm3s_busfault);
  irq_attach(LMSB_IRQ_USAGEFAULT, lm3s_usagefault);
  irq_attach(LMSB_IRQ_SVCALL, lm3s_svcall);
  irq_attach(LMSB_IRQ_DBGMONITOR, lm3s_dbgmonitor);
  irq_attach(LMSB_IRQ_RESERVED, lm3s_reserved);
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* Initialize FIQs */

#ifdef CONFIG_ARCH_FIQ
  up_fiqinitialize();
#endif

  /* And finally, enable interrupts */

  irqrestore(0);
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
  uint32 regaddr;
  uint32 regval;
  uint32 bit;

  if (lml3s_irqinfo(irq, &regaddr, &bit) == 0)
    {
      /* Clear the appropriate bit in the register to enable the interrupt */

      regval  = getreg32(regaddr);
      regval &= ~bit;
      putreg32(regval, regaddr);
    }
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
  uint32 regaddr;
  uint32 regval;
  uint32 bit;

  if (lml3s_irqinfo(irq, &regaddr, &bit) == 0)
    {
      /* Set the appropriate bit in the register to enable the interrupt */

      regval  = getreg32(regaddr);
      regval |= bit;
      putreg32(regval, regaddr);
    }
}

/****************************************************************************
 * Name: up_maskack_irq
 *
 * Description:
 *   Mask the IRQ and acknowledge it
 *
 ****************************************************************************/

void up_maskack_irq(int irq)
{
  up_disable_irq(irq);
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32 regaddr;
  uint32 regval;
  int shift;

  DEBUGASSERT(irq >= LMSB_IRQ_MPU && irq < NR_IRQS && (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < LM3S_IRQ_INTERRUPTS)
    {
      regaddr = NVIC_SYSH_PRIORITY(irq);
    }
  else
    {
      irq    -= LM3S_IRQ_INTERRUPTS;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);
  return OK;
}
#endif

/****************************************************************************
 * Name: lm3s_nmi, lm3s_hardfault, lm3s_mpu, lm3s_busfault, lm3s_usagefault,
 *       lm3s_svcall, lm3s_dbgmonitor, lm3s_pendsv, lm3s_reserved
 *
 * Description:
 *   Handlers for various execptions.  None are handler and all are fatal
 *   error conditions.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
int lm3s_nmi(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! NMI received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

int lm3s_hardfault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Hard fault received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

int lm3s_mpu(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! MPU interrupt received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

int lm3s_busfault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Bus fault recived\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

int lm3s_usagefault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Usage fault received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

int lm3s_svcall(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! SVCALL received\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

int lm3s_dbgmonitor(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Debug Monitor receieved\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}

int lm3s_reserved(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Reserved interrupt\n");
  PANIC(OSERR_UNEXPECTEDISR);
  return 0;
}
#endif
