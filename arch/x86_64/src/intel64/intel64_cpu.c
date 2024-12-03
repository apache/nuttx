/****************************************************************************
 * arch/x86_64/src/intel64/intel64_cpu.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include <arch/irq.h>
#include <arch/io.h>
#include <arch/acpi.h>

#include <stddef.h>
#include <assert.h>

#include <intel64_cpu.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IRQ_STACK_ALLOC (IRQ_STACK_SIZE * CONFIG_SMP_NCPUS)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern volatile uint32_t g_cpu_count;
static spinlock_t        g_ap_boot;

/* CPU private data */

struct intel64_cpu_s g_cpu_priv[CONFIG_SMP_NCPUS];

/* Allocate stack for interrupts and isr */

uint8_t  g_intstackalloc[IRQ_STACK_ALLOC] aligned_data(64);
uint8_t  g_isrstackalloc[IRQ_STACK_ALLOC] aligned_data(64);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_cpu_tss_get
 *
 * Description:
 *   Get TSS for a given CPU data.
 *
 ****************************************************************************/

static struct tss_s *x86_64_cpu_tss_get(uint8_t cpu)
{
  return (struct tss_s *)((uintptr_t)&g_ist64_low + X86_64_LOAD_OFFSET +
                          (X86_TSS_SIZE * cpu));
}

/****************************************************************************
 * Name: x86_64_cpu_tss_load
 *
 * Description:
 *  Load TSS for the current CPU
 *
 ****************************************************************************/

static void x86_64_cpu_tss_load(int cpu)
{
  uint16_t addr;

  /* Get offset for TSS */

  addr = X86_GDT_ISTL_SEL_NUM * 8 + 16 * cpu;

  __asm__ volatile ("mov %0, %%ax; ltr %%ax"
                    :: "m"(addr) : "memory", "rax");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_cpu_tss_init
 *
 * Description:
 *  Initialize the TSS
 *
 ****************************************************************************/

void x86_64_cpu_tss_init(int cpu)
{
  struct tss_s       *tss   = NULL;
  struct ist_s       *ist64 = NULL;
  struct gdt_entry_s  tss_l;
  uint64_t            tss_h;

  /* Get TSS - one per CPU */

  tss = x86_64_cpu_tss_get(cpu);

  /* Setup IST pointer */

  ist64 = &tss->ist;

  /* Reset local data */

  memset(&tss_l, 0, sizeof(tss_l));
  memset(&tss_h, 0, sizeof(tss_h));

  /* Segment limit = IST size - 1 */

  tss_l.limit_low = ((X86_IST_SIZE - 1) & 0xffff);

  /* Low address 1 */

  tss_l.base_low  = ((uintptr_t)ist64 & 0x00ffffff);

  /* Low address 2 */

  tss_l.base_high = (((uintptr_t)ist64 & 0xff000000) >> 24);
  tss_l.P = 1;

  /* Set type as IST */

  tss_l.AC = 1;
  tss_l.EX = 1;

  /* High address */

  tss_h = (((uintptr_t)ist64 >> 32) & 0xffffffff);

  g_gdt64[X86_GDT_ISTL_SEL_NUM + 2 * cpu] = tss_l;

  /* memcpy used to handle type punning compiler warning */

  memcpy((void *)&g_gdt64[X86_GDT_ISTH_SEL_NUM + 2 * cpu],
         &tss_h, sizeof(g_gdt64[0]));

  /* Stack for IRQ0-IRQ15 */

  ist64->IST1 = (uintptr_t)(g_isrstackalloc + ((cpu + 1) * IRQ_STACK_SIZE));

  /* Stack for the rest IRQ */

  ist64->IST2 = (uintptr_t)(g_intstackalloc + ((cpu + 1) * IRQ_STACK_SIZE));

  /* Now load TSS */

  x86_64_cpu_tss_load(cpu);
}

/****************************************************************************
 * Name: x86_64_cpu_tss_now_get
 *
 * Description:
 *   Get CPU TSS data associated with the current CPU
 *
 ****************************************************************************/

struct tss_s *x86_64_cpu_tss_now_get(void)
{
  uint64_t  *ist        = 0;
  uint16_t   seg        = 0;
  uint16_t   ist_offset = 0;
  uintptr_t  tss_addr   = 0;

  /* Get TSS associated with this CPU */

  __asm__ volatile ("str %%ax; mov %%ax, %0": "=rm"(seg)
                    :: "memory", "rax");

  /* This is BSP if TSS not configured yet */

  if (seg == 0)
    {
      return NULL;
    }

  /* Decode TSS */

  ist_offset = ((seg - X86_GDT_ISTL_SEL_NUM * 8) / 8);
  ist = (uint64_t *)((uintptr_t)&g_gdt64_ist_low + X86_64_LOAD_OFFSET);

  /* Low address */

  tss_addr = (ist[ist_offset] >> 16) & 0x00ffffff;
  tss_addr |= ((ist[ist_offset] >> 56) & 0xff) << 24;

  /* High address */

  tss_addr |= ist[ist_offset + 1] << 32;

  return (struct tss_s *)tss_addr;
}

/****************************************************************************
 * Name: x86_64_cpu_init
 *
 * Description:
 *   Initialize CPU data.
 *
 ****************************************************************************/

void x86_64_cpu_init(void)
{
  struct tss_s        *tss   = NULL;
  struct acpi_lapic_s *lapic = NULL;
  int                  i     = 0;
  int                  ret   = OK;

  /* Map logical CPU to Local APIC IDs */

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      ret = acpi_lapic_get(i, &lapic);
      if (ret == OK)
        {
          g_cpu_priv[i].loapic_id = lapic->apic_id;
          g_cpu_priv[i].id        = i;
          g_cpu_priv[i].ready     = false;
#ifdef CONFIG_LIB_SYSCALL
          g_cpu_priv[i].ustack    = NULL;
          g_cpu_priv[i].uvbase    = (uint64_t *)CONFIG_ARCH_TEXT_VBASE;
#endif
#ifdef CONFIG_ARCH_KERNEL_STACK
          g_cpu_priv[i].ktopstk   = NULL;
#endif

          /* Store private CPU in TSS */

          tss = x86_64_cpu_tss_get(i);
          tss->cpu = &g_cpu_priv[i];
        }
      else
        {
          /* We want to fail early when NCPUS doesn't match the number
           * of availalbe CPUs
           */

          /* Panic if not found */

          PANIC();
        }
    }

  /* Set BSP ready flag */

  g_cpu_priv[0].ready = true;
}

/****************************************************************************
 * Name: x86_64_lopaic_to_cpu
 *
 * Description:
 *   Get CPU index for a given Local APIC ID
 *
 ****************************************************************************/

uint8_t x86_64_loapic_to_cpu(uint8_t loapic)
{
  int i;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      if (g_cpu_priv[i].loapic_id == loapic)
        {
          return i;
        }
    }

  /* Panic if not found */

  PANIC();
}

/****************************************************************************
 * Name: x86_64_cpu_to_loapic
 *
 * Description:
 *   Get Local APIC ID for a given CPU index
 *
 ****************************************************************************/

uint8_t x86_64_cpu_to_loapic(uint8_t cpu)
{
  return g_cpu_priv[cpu].loapic_id;
}

/****************************************************************************
 * Name: x86_64_cpu_ready_set
 *
 * Description:
 *   Set CPU ready flag
 *
 ****************************************************************************/

void x86_64_cpu_ready_set(uint8_t cpu)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_ap_boot);

  if (!g_cpu_priv[cpu].ready)
    {
      g_cpu_priv[cpu].ready = true;
      g_cpu_count++;
    }

  spin_unlock_irqrestore(&g_ap_boot, flags);
}

/****************************************************************************
 * Name: x86_64_cpu_ready_get
 *
 * Description:
 *   Get CPU ready flag
 *
 ****************************************************************************/

bool x86_64_cpu_ready_get(uint8_t cpu)
{
  struct intel64_cpu_s *priv  = &g_cpu_priv[cpu];
  irqstate_t flags;
  bool ready;

  flags = spin_lock_irqsave(&g_ap_boot);
  ready = priv->ready;
  spin_unlock_irqrestore(&g_ap_boot, flags);

  return ready;
}

/****************************************************************************
 * Name: x86_64_cpu_count_get
 *
 * Description:
 *   Get CPU counter
 *
 ****************************************************************************/

uint8_t x86_64_cpu_count_get(void)
{
  irqstate_t flags;
  uint8_t count;

  flags = spin_lock_irqsave(&g_ap_boot);
  count = g_cpu_count;
  spin_unlock_irqrestore(&g_ap_boot, flags);

  return count;
}

/****************************************************************************
 * Name: x86_64_cpu_priv_set
 *
 * Description:
 *   Save CPU private data
 *
 ****************************************************************************/

void x86_64_cpu_priv_set(uint8_t cpu)
{
  /* Store private data pointer to GSBASE */

  write_gsbase((uintptr_t)&g_cpu_priv[cpu]);

#ifdef CONFIG_LIB_SYSCALL
  /* Configure SYSCALL instruction entry point */

  write_msr(MSR_LSTAR, (uintptr_t)x86_64_syscall_entry);

  /* Configure CS selection for SYSCALL (kernel) and SYSRET (userspace).
   *
   * Segment selection for SYSCALL works like this:
   *
   *          CS.Selector = IA32_STAR[47:32]
   *          SS.Selector := IA32_STAR[47:32] + 8
   *
   *       This require that we have to fill GDT with kernel code segment
   *       first and after that we can put kernel data segment.
   *
   * Segment selection for SYSRET has a really weird setup for 64-bit
   * operand size:
   *
   *           CS.Selector = IA32_STAR[63:48]+16
   *           SS.Selector = IA32_STAR[63:48]+8
   *
   *       This require that we have to fill GDT with user data segment
   *       first and after that we can put user code segment (differently
   *       than for kernel segments). Then this instruction needs to
   *       set CS segment for SYSRET at (USERDATA_SEL - 8) to work
   *       correctly.
   */

  write_msr(MSR_STAR, MSR_STAR_CSSYSCALL(X86_GDT_CODE_SEL) |
            MSR_STAR_CSSYSRET(X86_GDT_USERDATA_SEL - 8));

  /* Mask applied to RFLAGS when making a syscall */

  write_msr(MSR_FMASK, X86_64_RFLAGS_IF | X86_64_RFLAGS_DF);
#endif

#ifdef CONFIG_SMP
  /* Attach TLB shootdown handler */

  irq_attach(SMP_IPI_TLBSHOOTDOWN_IRQ, x86_64_tlb_handler, NULL);
#endif
}
