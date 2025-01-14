/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_irq.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "riscv_ipi.h"
#include "riscv_aia.h"
#include "chip.h"

#ifdef CONFIG_RPTUN
#include "qemu_rv_rptun.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_RPTUN
static int qemu_ipi_handler(int mcause, void *regs, void *args)
{
  /* Clear IPI (Inter-Processor-Interrupt) */

  riscv_ipi_clear(this_cpu());

#ifdef CONFIG_SMP
  riscv_pause_handler(mcause, regs, args);
#endif

  qemu_rptun_ipi();
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable Machine interrupts */

  up_irq_save();

  /* Disable all global interrupts */

#ifndef CONFIG_ARCH_RV_HAVE_APLIC
  putreg32(0x0, QEMU_RV_PLIC_ENABLE1);
  putreg32(0x0, QEMU_RV_PLIC_ENABLE2);
#else
  riscv_aplic_disable_irqs(QEMU_RV_APLIC_BASE, QEMU_RV_APLIC_NR_IRQ);
#endif

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color(g_intstackalloc, intstack_size);
#endif

  int id;

#ifdef CONFIG_ARCH_RV_HAVE_IMSIC
  /* Set default IRQ target hart index to 0 and EIID */

  for (id = 0; id < QEMU_RV_APLIC_NR_IRQ; id++)
    {
      riscv_aplic_configure_irq(QEMU_RV_APLIC_BASE, id + 1, 0, id + 1);
    }

  riscv_aplic_init_msi(QEMU_RV_APLIC_BASE, QEMU_RV_IMSIC_BASE, 0, 3, 0, 0);
#elif defined(CONFIG_ARCH_RV_HAVE_APLIC)
  /* Set default IRQ target hart index to 0 and priority */

  for (id = 0; id < QEMU_RV_APLIC_NR_IRQ; id++)
    {
      riscv_aplic_configure_irq(QEMU_RV_APLIC_BASE, id + 1,
                                RISCV_APLIC_DEFAULT_PRIORITY, 0);
    }

  riscv_aplic_init(QEMU_RV_APLIC_BASE, RISCV_APLIC_ENABLE_IDELIVERY,
                                       RISCV_APLIC_ENABLE_ITHRESHOLD);
#else
  /* Set priority for all global interrupts to 1 (lowest) */

  for (id = 1; id <= 52; id++)
    {
      putreg32(1, (uintptr_t)(QEMU_RV_PLIC_PRIORITY + 4 * id));
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  putreg32(0, QEMU_RV_PLIC_THRESHOLD);
#endif

  /* Attach the common interrupt handler */

  riscv_exception_attach();

#ifdef CONFIG_RPTUN
  /* Replace default IRQ_SOFT handler */

  irq_attach(RISCV_IRQ_SOFT, qemu_ipi_handler, NULL);
#endif

#if defined(CONFIG_SMP) || defined(CONFIG_RPTUN)
  /* Clear IPI for CPU0 */

  riscv_ipi_clear(0);

#ifdef CONFIG_ARCH_RV_USE_IMSIC_IPI
  riscv_imsic_local_eie_enable(RISCV_IMSIC_IPI_ID);
#else
  up_enable_irq(RISCV_IRQ_SOFT);
#endif
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

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
  int extirq;

  if (irq == RISCV_IRQ_SOFT)
    {
      /* Read m/sstatus & clear machine software interrupt enable in m/sie */

      CLEAR_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      /* Read m/sstatus & clear timer interrupt enable in m/sie */

      CLEAR_CSR(CSR_IE, IE_TIE);
    }
  else if (irq == RISCV_IRQ_EXT)
    {
      /* Read m/sstatus & clear external interrupt enable in m/sie */

      CLEAR_CSR(CSR_IE, IE_EIE);

#ifdef CONFIG_ARCH_RV_HAVE_IMSIC
      /* Set IMSIC irq threshold to 0 (permits all global interrupts) */

      riscv_imsic_csr_write(ISELECT_EITHRESHOLD,
                            RISCV_IMSIC_DISABLE_EITHRESHOLD);

      /* Enable irq delivery for IMSIC */

      riscv_imsic_csr_write(ISELECT_EIDELIVERY,
                            RISCV_IMSIC_DISABLE_EIDELIVERY);
#endif
    }
  else if (irq > RISCV_IRQ_EXT)
    {
      extirq = irq - RISCV_IRQ_EXT;

      /* Clear enable bit for the irq */

      if (0 <= extirq && extirq <= 63)
        {
#ifndef CONFIG_ARCH_RV_HAVE_APLIC
          modifyreg32(QEMU_RV_PLIC_ENABLE1 + (4 * (extirq / 32)),
                      1 << (extirq % 32), 0);
#else
          riscv_aplic_disable_irq(QEMU_RV_APLIC_BASE, extirq);
#endif
#ifdef CONFIG_ARCH_RV_HAVE_IMSIC
          riscv_imsic_local_eie_disable(extirq);
#endif
        }
      else
        {
          PANIC();
        }
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
  int extirq;

  if (irq == RISCV_IRQ_SOFT)
    {
      /* Read m/sstatus & set machine software interrupt enable in m/sie */

      SET_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      /* Read m/sstatus & set timer interrupt enable in m/sie */

      SET_CSR(CSR_IE, IE_TIE);
    }
  else if (irq == RISCV_IRQ_EXT)
    {
      /* Read m/sstatus & set external interrupt enable in m/sie */

      SET_CSR(CSR_IE, IE_EIE);

#ifdef CONFIG_ARCH_RV_HAVE_IMSIC
      /* Set IMSIC irq threshold to 0 (permits all global interrupts) */

      riscv_imsic_csr_write(ISELECT_EITHRESHOLD,
                            RISCV_IMSIC_ENABLE_EITHRESHOLD);

      /* Enable irq delivery for IMSIC */

      riscv_imsic_csr_write(ISELECT_EIDELIVERY,
                            RISCV_IMSIC_ENABLE_EIDELIVERY);
#endif
    }
  else if (irq > RISCV_IRQ_EXT)
    {
      extirq = irq - RISCV_IRQ_EXT;

      /* Set enable bit for the irq */

      if (0 <= extirq && extirq <= 63)
        {
#ifndef CONFIG_ARCH_RV_HAVE_APLIC
          modifyreg32(QEMU_RV_PLIC_ENABLE1 + (4 * (extirq / 32)),
                      0, 1 << (extirq % 32));
#else
          riscv_aplic_configure_irq(QEMU_RV_APLIC_BASE, extirq,
                                    RISCV_APLIC_SOURCECFG_SM_EDGE_RISE,
                                    up_cpu_index());
#ifdef CONFIG_ARCH_RV_HAVE_IMSIC
          riscv_imsic_local_eie_enable(extirq);
#endif
          riscv_aplic_enable_irq(QEMU_RV_APLIC_BASE, extirq);
#endif
        }
      else
        {
          PANIC();
        }
    }
}

irqstate_t up_irq_enable(void)
{
  irqstate_t oldstat;

  /* Enable external interrupts (mie/sie) */

  up_enable_irq(RISCV_IRQ_EXT);

  /* Read and enable global interrupts (M/SIE) in m/sstatus */

  oldstat = READ_AND_SET_CSR(CSR_STATUS, STATUS_IE);

  /* Enable APLIC irq */
#ifdef CONFIG_ARCH_RV_HAVE_IMSIC
  modifyreg32(QEMU_RV_APLIC_BASE + RISCV_APLIC_DOMAINCFG, 0x0,
              RISCV_APLIC_DOMAINCFG_IE | RISCV_APLIC_DOMAINCFG_DM);
#elif defined(CONFIG_ARCH_RV_HAVE_APLIC)
  modifyreg32(QEMU_RV_APLIC_BASE + RISCV_APLIC_DOMAINCFG, 0x0,
              RISCV_APLIC_DOMAINCFG_IE);
#endif

  return oldstat;
}
