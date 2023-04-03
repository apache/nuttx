/****************************************************************************
 * arch/risc-v/src/espressif/esp_irq.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "riscv_internal.h"

#include "esp_irq.h"

#include "esp_attr.h"
#include "esp_bit_defs.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "riscv/interrupt.h"
#include "soc/soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP_DEFAULT_INT_THRESHOLD     1

#define IRQ_UNMAPPED                  0xff

/* Helper macros for working with cpuint_mapentry_t fields */

#define CPUINT_DISABLE(cpuint)        ((cpuint).cpuint_en = 0)
#define CPUINT_ENABLE(cpuint)         ((cpuint).cpuint_en = 1)
#define CPUINT_ASSIGN(cpuint,in_irq)  do                            \
                                        {                           \
                                          (cpuint).assigned = 1;    \
                                          (cpuint).cpuint_en = 1;   \
                                          (cpuint).irq = (in_irq);  \
                                        }                           \
                                      while(0)
#define CPUINT_GETIRQ(cpuint)         ((cpuint).irq)
#define CPUINT_FREE(cpuint)           ((cpuint).val = 0)
#define CPUINT_ISENABLED(cpuint)      ((cpuint).cpuint_en == 1)
#define CPUINT_ISASSIGNED(cpuint)     ((cpuint).assigned == 1)
#define CPUINT_ISFREE(cpuint)         (!CPUINT_ISASSIGNED(cpuint))

/* CPU interrupts can be detached from any interrupt source by setting the
 * map register to ETS_INVALID_INUM, which is an invalid CPU interrupt index.
 */

#define NO_CPUINT                     ETS_INVALID_INUM

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CPU interrupts to IRQ index mapping */

typedef union cpuint_mapentry_u
{
  struct
    {
      uint16_t assigned:1;
      uint16_t cpuint_en:1;
      uint16_t irq:10;
      uint16_t reserved0:4;
    };

  uint16_t val;
} cpuint_mapentry_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Map a CPU interrupt to the IRQ of the attached interrupt source */

static cpuint_mapentry_t g_cpuint_map[ESP_NCPUINTS];

/* Map an IRQ to a CPU interrupt to which the interrupt source is
 * attached to.
 */

static volatile uint8_t g_irq_map[NR_IRQS];

/* Bitsets for free, unallocated CPU interrupts available to peripheral
 * devices.
 */

static uint32_t g_cpuint_freelist = ESP_CPUINT_PERIPHSET;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_cpuint_alloc
 *
 * Description:
 *   Allocate a free CPU interrupt for a peripheral device. This function
 *   will not ignore all of the pre-allocated CPU interrupts for internal
 *   devices.
 *
 * Input Parameters:
 *   irq           - IRQ number.
 *
 * Returned Value:
 *   On success, a CPU interrupt number is returned.
 *   A negated errno is returned on failure.
 *
 ****************************************************************************/

static int esp_cpuint_alloc(int irq)
{
  uint32_t bitmask;
  uint32_t intset;
  int cpuint;

  /* Check if there are CPU interrupts with the requested properties
   * available.
   */

  intset = g_cpuint_freelist;
  if (intset != 0)
    {
      /* Skip over initial unavailable CPU interrupts quickly in groups
       * of 8 interrupt.
       */

      for (cpuint = 0, bitmask = 0xff;
           cpuint < ESP_NCPUINTS && (intset & bitmask) == 0;
           cpuint += 8, bitmask <<= 8);

      /* Search for an unallocated CPU interrupt number in the remaining
       * intset.
       */

      for (; cpuint < ESP_NCPUINTS; cpuint++)
        {
          /* If the bit corresponding to the CPU interrupt is '1', then
           * that CPU interrupt is available.
           */

          bitmask = BIT(cpuint);
          if ((intset & bitmask) != 0)
            {
              /* Got it! */

              g_cpuint_freelist &= ~bitmask;
              break;
            }
        }
    }

  if (cpuint == ESP_NCPUINTS)
    {
      /* No unallocated CPU interrupt found */

      return -ENOMEM;
    }

  DEBUGASSERT(CPUINT_ISFREE(g_cpuint_map[cpuint]));

  CPUINT_ASSIGN(g_cpuint_map[cpuint], irq);
  g_irq_map[irq] = cpuint;

  return cpuint;
}

/****************************************************************************
 * Name: esp_cpuint_free
 *
 * Description:
 *   Free a previously allocated CPU interrupt.
 *
 * Input Parameters:
 *   cpuint        - CPU interrupt to be freed.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_cpuint_free(int cpuint)
{
  uint32_t bitmask;

  DEBUGASSERT(cpuint >= 0 && cpuint < ESP_NCPUINTS);
  DEBUGASSERT(CPUINT_ISASSIGNED(g_cpuint_map[cpuint]));

  int irq = CPUINT_GETIRQ(g_cpuint_map[cpuint]);
  g_irq_map[irq] = IRQ_UNMAPPED;
  CPUINT_FREE(g_cpuint_map[cpuint]);

  /* Mark the CPU interrupt as available */

  bitmask = BIT(cpuint);

  DEBUGASSERT((g_cpuint_freelist & bitmask) == 0);

  g_cpuint_freelist |= bitmask;
}

/****************************************************************************
 * Name: esp_cpuint_initialize
 *
 * Description:
 *   Initialize CPU interrupts.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_cpuint_initialize(void)
{
  /* Unmap CPU interrupts from every interrupt source */

  for (int source = 0; source < ESP_NSOURCES; source++)
    {
      esp_rom_route_intr_matrix(PRO_CPU_NUM, source, NO_CPUINT);
    }

  /* Set CPU interrupt threshold level */

  esprv_intc_int_set_threshold(ESP_DEFAULT_INT_THRESHOLD);

  /* Indicate that no interrupt sources are assigned to CPU interrupts */

  memset(g_cpuint_map, 0, sizeof(g_cpuint_map));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Complete initialization of the interrupt system and enable normal,
 *   interrupt processing.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Indicate that no interrupt sources are assigned to CPU interrupts */

  for (int i = 0; i < NR_IRQS; i++)
    {
      g_irq_map[i] = IRQ_UNMAPPED;
    }

  /* Initialize CPU interrupts */

  esp_cpuint_initialize();

  /* Attach the common interrupt handler */

  riscv_exception_attach();

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the interrupt specified by 'irq'.
 *
 * Input Parameters:
 *   irq           - IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  int cpuint = g_irq_map[irq];

  irqinfo("irq=%d | cpuint=%d \n", irq, cpuint);

  DEBUGASSERT(cpuint >= 0 && cpuint < ESP_NCPUINTS);

  irqstate_t irqstate = enter_critical_section();

  CPUINT_ENABLE(g_cpuint_map[cpuint]);
  esprv_intc_int_enable(BIT(cpuint));

  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the interrupt specified by 'irq'.
 *
 * Input Parameters:
 *   irq           - IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  int cpuint = g_irq_map[irq];

  irqinfo("irq=%d | cpuint=%d \n", irq, cpuint);

  DEBUGASSERT(cpuint >= 0 && cpuint < ESP_NCPUINTS);

  irqstate_t irqstate = enter_critical_section();

  CPUINT_DISABLE(g_cpuint_map[cpuint]);
  esprv_intc_int_disable(BIT(cpuint));

  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name: esp_route_intr
 *
 * Description:
 *   Assign an interrupt source to a pre-allocated CPU interrupt.
 *
 * Input Parameters:
 *   source        - Interrupt source (see irq.h) to be assigned to a CPU
 *                   interrupt.
 *   cpuint        - Pre-allocated CPU interrupt to which the interrupt
 *                   source will be assigned.
 *   priority      - Interrupt priority.
 *   type          - Interrupt trigger type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_route_intr(int source, int cpuint, irq_priority_t priority,
                    irq_trigger_t type)
{
  /* Ensure the CPU interrupt is disabled */

  esprv_intc_int_disable(BIT(cpuint));

  /* Set the interrupt priority */

  esprv_intc_int_set_priority(cpuint, priority);

  /* Set the interrupt trigger type (Edge or Level) */

  if (type == ESP_IRQ_TRIGGER_EDGE)
    {
      esprv_intc_int_set_type(cpuint, INTR_TYPE_EDGE);
    }
  else
    {
      esprv_intc_int_set_type(cpuint, INTR_TYPE_LEVEL);
    }

  /* Route the interrupt source to the provided CPU interrupt */

  esp_rom_route_intr_matrix(PRO_CPU_NUM, source, cpuint);
}

/****************************************************************************
 * Name: esp_setup_irq
 *
 * Description:
 *   Configure an IRQ. It allocates a CPU interrupt of the given
 *   priority and type and attaches a given interrupt source to it.
 *
 * Input Parameters:
 *   source        - Interrupt source (see irq.h) to be assigned to
 *                   a CPU interrupt.
 *   priority      - Interrupt priority.
 *   type          - Interrupt trigger type.
 *
 * Returned Value:
 *   Allocated CPU interrupt.
 *
 ****************************************************************************/

int esp_setup_irq(int source, irq_priority_t priority, irq_trigger_t type)
{
  irqstate_t irqstate;
  int irq;
  int cpuint;

  irqinfo("source = %d\n", source);

  DEBUGASSERT(source >= 0 && source < ESP_NSOURCES);

  irqstate = enter_critical_section();

  /* Setting up an IRQ includes the following steps:
   *    1. Allocate a CPU interrupt.
   *    2. Map the CPU interrupt to the IRQ to ease searching later.
   *    3. Attach the interrupt source to the newly allocated CPU interrupt.
   */

  irq = ESP_SOURCE2IRQ(source);
  cpuint = esp_cpuint_alloc(irq);
  if (cpuint < 0)
    {
      _alert("Unable to allocate CPU interrupt for source=%d\n",
             priority, type);

      PANIC();
    }

  esp_route_intr(source, cpuint, priority, type);

  leave_critical_section(irqstate);

  return cpuint;
}

/****************************************************************************
 * Name: esp_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by esp_setup_irq.
 *   It detaches an interrupt source from a CPU interrupt and frees the
 *   CPU interrupt.
 *
 * Input Parameters:
 *   source        - Interrupt source (see irq.h) to be detached from the
 *                   CPU interrupt.
 *   cpuint        - CPU interrupt from which the interrupt source will
 *                   be detached.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_teardown_irq(int source, int cpuint)
{
  irqstate_t irqstate = enter_critical_section();

  /* Tearing down an IRQ includes the following steps:
   *   1. Free the previously allocated CPU interrupt.
   *   2. Unmap the IRQ from the IRQ-to-cpuint map.
   *   3. Detach the interrupt source from the CPU interrupt.
   */

  esp_cpuint_free(cpuint);

  DEBUGASSERT(source >= 0 && source < ESP_NSOURCES);

  esp_rom_route_intr_matrix(PRO_CPU_NUM, source, NO_CPUINT);

  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name: riscv_dispatch_irq
 *
 * Description:
 *   Process interrupt and its callback function.
 *
 * Input Parameters:
 *   mcause        - RISC-V "mcause" register.
 *   regs          - Saved registers reference.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR uintptr_t *riscv_dispatch_irq(uintptr_t mcause, uintptr_t *regs)
{
  int irq;
  bool is_irq = (RISCV_IRQ_BIT & mcause) != 0;
  bool is_edge = false;

  if (is_irq)
    {
      uint8_t cpuint = mcause & RISCV_IRQ_MASK;

      DEBUGASSERT(cpuint >= 0 && cpuint < ESP_NCPUINTS);
      DEBUGASSERT(CPUINT_ISENABLED(g_cpuint_map[cpuint]));
      DEBUGASSERT(CPUINT_ISASSIGNED(g_cpuint_map[cpuint]));

      irq = g_cpuint_map[cpuint].irq;

      is_edge = esprv_intc_int_get_type(cpuint) == INTR_TYPE_EDGE;
      if (is_edge)
        {
          /* Clear edge interrupts. */

          esp_cpu_intr_edge_ack(cpuint);
        }
    }
  else
    {
      /* It's exception */

      irq = mcause;
    }

  regs = riscv_doirq(irq, regs);

  return regs;
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable interrupts globally.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The interrupt state prior to enabling interrupts.
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t flags;

  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */

  flags = READ_AND_SET_CSR(mstatus, MSTATUS_MIE);
  return flags;
}
