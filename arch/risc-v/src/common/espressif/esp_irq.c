/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_irq.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "riscv_internal.h"

#include "esp_gpio.h"
#include "esp_irq.h"
#include "esp_rtc_gpio.h"

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
#define CPUINT_ISRESERVED(cpuint)     ((cpuint).reserved0 == 1)
#define CPUINT_ISFREE(cpuint)         (!CPUINT_ISASSIGNED(cpuint))

/* CPU interrupts can be detached from any interrupt source by setting the
 * map register to ETS_INVALID_INUM, which is an invalid CPU interrupt index.
 */

#define NO_CPUINT                     ETS_INVALID_INUM

/* Masks for interrupt type used on esp_setup_irq */

#define ESP_CPUINT_IRAM_MASK       (1 << 1)
#define ESP_CPUINT_TRIGGER_MASK    (1 << 0)

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

/* This bitmask has an 1 if the int should be disabled
 * when the flash is disabled.
 */

static uint32_t non_iram_int_mask[CONFIG_ESPRESSIF_NUM_CPUS];

/* This bitmask has 1 in it if the int was disabled
 * using esp_intr_noniram_disable.
 */

static uint32_t non_iram_int_disabled[CONFIG_ESPRESSIF_NUM_CPUS];
static bool non_iram_int_disabled_flag[CONFIG_ESPRESSIF_NUM_CPUS];

#ifdef CONFIG_ESPRESSIF_IRAM_ISR_DEBUG
/* The g_iram_count keeps track of how many times such an IRQ ran when the
 * non-IRAM interrupts were disabled.
 */

static uint64_t g_iram_count[NR_IRQS];
#endif

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
  int cpuint = ESP_NCPUINTS;

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

  esp_set_irq(irq, cpuint);

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

#ifdef CONFIG_ESPRESSIF_IRAM_ISR_DEBUG

/****************************************************************************
 * Name:  esp_iram_interrupt_record
 *
 * Description:
 *   This function keeps track of the IRQs that ran when non-IRAM interrupts
 *   are disabled and enables debugging of the IRAM-enabled interrupts.
 *
 * Input Parameters:
 *   irq - The IRQ associated with a CPU interrupt
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_irq_iram_interrupt_record(int irq)
{
  irqstate_t flags = enter_critical_section();

  g_iram_count[irq]++;

  leave_critical_section(flags);
}
#endif

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
  /* All CPU ints are non-IRAM interrupts at the beginning and should be
   * disabled during a SPI flash operation
   */

  for (int i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      non_iram_int_mask[i] = UINT32_MAX;
    }

  /* Indicate that no interrupt sources are assigned to CPU interrupts */

  for (int i = 0; i < NR_IRQS; i++)
    {
      g_irq_map[i] = IRQ_UNMAPPED;
    }

  /* Initialize CPU interrupts */

  esp_cpuint_initialize();

  /* Initialize GPIO interrupt support */

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
  esp_gpioirqinitialize();
#endif

  /* Initialize RTCIO interrupt support */

  esp_rtcioirqinitialize();

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

  /* Check if IRQ is initialized */

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

  /* Check if IRQ is initialized */

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

int esp_setup_irq(int source, irq_priority_t priority, int type)
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
             source);

      PANIC();
    }

  esp_route_intr(source, cpuint, priority, (type & ESP_CPUINT_TRIGGER_MASK));

  if ((type & ESP_CPUINT_IRAM_MASK) == ESP_IRQ_IRAM)
    {
      esp_irq_set_iram_isr(irq);
      irqinfo("source %d marked IRAM (irq=%d)\n", source, irq);
    }
  else
    {
      esp_irq_unset_iram_isr(irq);
    }

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

IRAM_ATTR void *riscv_dispatch_irq(uintreg_t mcause, uintreg_t *regs)
{
  int irq;
  bool is_irq = (RISCV_IRQ_BIT & mcause) != 0;
  bool is_edge = false;
  uint32_t cpu = esp_cpu_get_core_id();

  if (is_irq)
    {
      uint8_t cpuint = mcause & RISCV_IRQ_MASK;

      DEBUGASSERT(cpuint >= 0 && cpuint < ESP_NCPUINTS);
      DEBUGASSERT(CPUINT_ISENABLED(g_cpuint_map[cpuint]));
      DEBUGASSERT(CPUINT_ISASSIGNED(g_cpuint_map[cpuint]));

      irq = g_cpuint_map[cpuint].irq;

#ifdef CONFIG_ESPRESSIF_IRAM_ISR_DEBUG
      /* Check if non-IRAM interrupts are disabled */

      if (esp_irq_noniram_status(cpu) == 0)
        {
          /* Sum-up the IRAM-enabled counter associated with the IRQ */

          esp_irq_iram_interrupt_record(irq);
        }
#endif

      is_edge = esprv_intc_int_get_type(cpuint) == INTR_TYPE_LEVEL;
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

  flags = READ_AND_SET_CSR(CSR_MSTATUS, MSTATUS_MIE);
  return flags;
}

/****************************************************************************
 * Name: esp_intr_noniram_disable
 *
 * Description:
 *   Disable interrupts that aren't specifically marked as running from IRAM.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_intr_noniram_disable(void)
{
  uint32_t oldint;
  irqstate_t irqstate;
  uint32_t cpu;
  uint32_t non_iram_ints;

  irqstate = enter_critical_section();
  cpu = esp_cpu_get_core_id();
  non_iram_ints = non_iram_int_mask[cpu];

  if (non_iram_int_disabled_flag[cpu])
    {
      abort();
    }

  non_iram_int_disabled_flag[cpu] = true;
  oldint = esp_cpu_intr_get_enabled_mask();
  esp_cpu_intr_disable(non_iram_ints);

  /* Save disabled ints */

  non_iram_int_disabled[cpu] = oldint & non_iram_ints;
  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name: esp_intr_noniram_enable
 *
 * Description:
 *   Enable interrupts that aren't specifically marked as running from IRAM.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_intr_noniram_enable(void)
{
  irqstate_t irqstate;
  uint32_t cpu;
  int non_iram_ints;

  irqstate = enter_critical_section();
  cpu = esp_cpu_get_core_id();
  non_iram_ints = non_iram_int_disabled[cpu];

  if (!non_iram_int_disabled_flag[cpu])
    {
      abort();
    }

  non_iram_int_disabled_flag[cpu] = false;
  esp_cpu_intr_enable(non_iram_ints);
  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name:  esp_irq_noniram_status
 *
 * Description:
 *   Get the current status of non-IRAM interrupts on a specific CPU core
 *
 * Input Parameters:
 *   cpu - The CPU to check the non-IRAM interrupts state
 *
 * Returned Value:
 *   true if non-IRAM interrupts are enabled, false otherwise.
 *
 ****************************************************************************/

bool esp_irq_noniram_status(int cpu)
{
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  return !non_iram_int_disabled_flag[cpu];
}

/****************************************************************************
 * Name:  esp_get_irq
 *
 * Description:
 *   This function returns the IRQ associated with a CPU interrupt
 *
 * Input Parameters:
 *   cpuint - The CPU interrupt associated to the IRQ
 *
 * Returned Value:
 *   The IRQ associated with such CPU interrupt or CPUINT_UNASSIGNED if
 *   IRQ is not yet assigned to a CPU interrupt.
 *
 ****************************************************************************/

int esp_get_irq(int cpuint)
{
  return CPUINT_GETIRQ(g_cpuint_map[cpuint]);
}

/****************************************************************************
 * Name:  esp_set_irq
 *
 * Description:
 *   This function assigns a CPU interrupt to a specific IRQ number. It
 *   updates the mapping between IRQ numbers and CPU interrupts, allowing
 *   the system to correctly route hardware interrupts to the appropriate
 *   handlers. Please note that this function is intended to be used only
 *   when a CPU interrupt is already assigned to an IRQ number. Otherwise,
 *   please check esp_setup_irq.
 *
 * Input Parameters:
 *   irq    - The IRQ number to be associated with the CPU interrupt.
 *   cpuint - The CPU interrupt to be associated with the IRQ number.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_set_irq(int irq, int cpuint)
{
  CPUINT_ASSIGN(g_cpuint_map[cpuint], irq);
  g_irq_map[irq] = cpuint;
}

/****************************************************************************
 * Name:  esp_irq_set_iram_isr
 *
 * Description:
 *   Set the ISR associated to an IRQ as a IRAM-enabled ISR.
 *
 * Input Parameters:
 *   irq - The associated IRQ to set
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int esp_irq_set_iram_isr(int irq)
{
  uint32_t cpu = esp_cpu_get_core_id();
  int cpuint = g_irq_map[irq];

  if (cpuint == IRQ_UNMAPPED)
    {
      return -EINVAL;
    }

  non_iram_int_mask[cpu] &= ~(1 << cpuint);

  return OK;
}

/****************************************************************************
 * Name:  esp_irq_unset_iram_isr
 *
 * Description:
 *   Set the ISR associated to an IRQ as a non-IRAM ISR.
 *
 * Input Parameters:
 *   irq - The associated IRQ to set
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int esp_irq_unset_iram_isr(int irq)
{
  uint32_t cpu = esp_cpu_get_core_id();
  int cpuint = g_irq_map[irq];

  if (cpuint == IRQ_UNMAPPED)
    {
      return -EINVAL;
    }

  non_iram_int_mask[cpu] |= (1 << cpuint);

  return OK;
}

/****************************************************************************
 * Name:  esp_get_iram_interrupt_records
 *
 * Description:
 *   This function copies the vector that keeps track of the IRQs that ran
 *   when non-IRAM interrupts were disabled.
 *
 * Input Parameters:
 *
 *   irq_count - A previously allocated pointer to store the counter of the
 *               interrupts that ran when non-IRAM interrupts were disabled.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_IRAM_ISR_DEBUG
void esp_get_iram_interrupt_records(uint64_t *irq_count)
{
  irqstate_t flags = enter_critical_section();

  memcpy(irq_count, &g_iram_count, sizeof(uint64_t) * NR_IRQS);

  leave_critical_section(flags);
}
#endif
