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

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

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
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Clear, disable and configure all interrupts. */

# warning "Missing logic"

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* Attach all processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG
  irq_attach(LMSB_IRQ_NMI, lm3s_nmi);
  irq_attach(LMSB_IRQ_HARDFAULT, lm3s_hardfault);
  irq_attach(LMSB_IRQ_MPU, lm3s_mpu);
  irq_attach(LMSB_IRQ_BUSFAULT, lm3s_busfault);
  irq_attach(LMSB_IRQ_USAGEFAULT, lm3s_usagefault);
  irq_attach(LMSB_IRQ_SVCALL, lm3s_svcall);
  irq_attach(LMSB_IRQ_DBGMONITOR, lm3s_dbgmonitor);
  irq_attach(LMSB_IRQ_PENDSV, lm3s_pendsv);
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
# warning "Missing logic"
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
# warning "Missing logic"
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
# warning "Missing logic"
}


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

int lm3s_pendsv(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! PendSV received\n");
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