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

#include "irq/irq.h"

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

#if SOC_INT_CLIC_SUPPORTED
#  include "hal/interrupt_clic_ll.h"
#  include "esp_private/interrupt_clic.h"
#endif // SOC_INT_CLIC_SUPPORTED

#include "esp_private/vectors_const.h"

#ifndef CONFIG_ARCH_CHIP_ESP32C3
#  include "soc/hp_system_reg.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SMP_NCPUS
#  define ESP_NCPUS                     CONFIG_SMP_NCPUS
#else
#  define ESP_NCPUS                     1
#endif

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE_DYNAMIC
#  ifndef CONFIG_ARCH_IRQ_TO_NDX
#    error "CONFIG_ARCH_IRQ_TO_NDX must be enabled for RISC-V-based \
            Espressif SoCs. Run 'make menuconfig' to select it."
#  endif
#  if CONFIG_ARCH_NUSER_INTERRUPTS != 17
#    error "CONFIG_ARCH_NUSER_INTERRUPTS must be 17 for RISC-V-based \
            Espressif SoCs. Run 'make menuconfig' to set it."
#  endif
#else
#  error "CONFIG_ARCH_MINIMAL_VECTORTABLE_DYNAMIC must be enabled for \
          RISC-V-based Espressif SoCs. Additionally, enable \
          CONFIG_ARCH_IRQ_TO_NDX and set CONFIG_ARCH_NUSER_INTERRUPTS to 17.\
          Run 'make menuconfig' to select and set these options."
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp_clear_handle(int cpu, int irq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Map an IRQ to a handle to which the interrupt source is attached to. */

static volatile intr_handle_t g_handle_map[CONFIG_SMP_NCPUS][NR_IRQS];

#ifdef CONFIG_ESPRESSIF_IRAM_ISR_DEBUG
/* The g_iram_count keeps track of how many times such an IRQ ran when the
 * non-IRAM interrupts were disabled.
 */

static uint64_t g_iram_count[CONFIG_SMP_NCPUS][NR_IRQS];
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  /* Set CPU interrupt threshold level */

  esprv_int_set_threshold(RVHAL_INTR_ENABLE_THRESH);
  rv_utils_intr_global_enable();
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
 *   cpu - The CPU associated with the CPU interrupt
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR static void esp_irq_iram_interrupt_record(int irq, int cpu)
{
  irqstate_t flags = enter_critical_section();

  g_iram_count[cpu][irq]++;

  leave_critical_section(flags);
}
#endif

IRAM_ATTR static void isr_adapter_func(void *arg)
{
  struct intr_adapter_from_nuttx *isr_adapter_args;

  isr_adapter_args = (struct intr_adapter_from_nuttx *)arg;

  isr_adapter_args->func(isr_adapter_args->irq,
                         isr_adapter_args->context,
                         isr_adapter_args->arg);
}

/****************************************************************************
 * Name: esp_isr_demultiplexing
 *
 * Description:
 *   Demultiplexing interrupt handler. All peripheral interrupts are
 *   dispatched through this single handler, which then calls the
 *   appropriate peripheral handler registered via esp_setup_irq.
 *
 * Input Parameters:
 *   irq     - The IRQ number (XTENSA_IRQ_DEMUX)
 *   context - Saved processor state
 *   arg     - Unused
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

IRAM_ATTR static int esp_isr_demultiplexing(int irq, void *context,
                                            void *arg)
{
  int cpuint = esp_get_cpuint(this_cpu(), irq);
  intr_handler_t handler;
  struct intr_adapter_from_nuttx *handler_arg;

  /* Validate cpuint - if invalid, the interrupt was not properly
   * registered via esp_setup_irq. This is a bug that needs to be fixed.
   */

  if (cpuint < 0 || cpuint >= SOC_CPU_INTR_NUM)
    {
      irqwarn("IRQ %d has invalid cpuint=%d (not registered)", irq, cpuint);
      return OK;
    }

  handler = (intr_handler_t)esp_cpu_intr_get_handler(cpuint);
  handler_arg = (struct intr_adapter_from_nuttx *)
                  esp_cpu_intr_get_handler_arg(cpuint);

  /* If the handler is the isr_adapter_func, then we need to set the irq
   * and context to the handler_arg. This is true for all interrupts set via
   * esp_setup_irq. Exceptions are the interrupts set directly by the
   * underlying hardware, like Wi-Fi.
   */

  if (handler == &isr_adapter_func)
    {
      handler_arg->irq = irq;
      handler_arg->context = context;
    }

  if (handler)
    {
      (*handler)(handler_arg);
    }
  else
    {
      /* Handler not found in _xt_interrupt_table.
       * This happens when irq_attach was used instead of esp_setup_irq
       * with a non-NULL handler. Peripheral handlers must be set via
       * esp_setup_irq() when using ARCH_MINIMAL_VECTORTABLE.
       */

      irqwarn("No handler for irq=%d cpuint=%d\n", irq, cpuint);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irq_to_ndx
 *
 * Description:
 *   Irq to ndx
 *
 ****************************************************************************/

int IRAM_ATTR up_irq_to_ndx(int irq)
{
  return irq < ESP_IRQ_FIRSTPERIPH ? irq : ESP_IRQ_DEMUX;
}

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
  int i;
  int j;

  /* Indicate that no interrupt sources are assigned to CPU interrupts */

  for (i = 0; i < NR_IRQS; i++)
    {
      for (j = 0; j < CONFIG_SMP_NCPUS; j++)
        {
          g_handle_map[j][i] = IRQ_UNMAPPED;
        }
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

  irq_attach(ESP_IRQ_DEMUX, esp_isr_demultiplexing, NULL);

  /* And finally, enable interrupts */

  riscv_color_intstack();
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
  esp_err_t ret;
  intr_handle_t intr_handle = esp_get_handle(this_cpu(), irq);

  if (intr_handle == IRQ_UNMAPPED)
    {
      irqwarn("IRQ %d not mapped to handle\n", irq);
      return;
    }

  ret = esp_intr_enable(intr_handle);
  if (ret != ESP_OK)
    {
      irqerr("Failed to enable interrupt %d\n", irq);
    }
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
  esp_err_t ret;
  intr_handle_t intr_handle = esp_get_handle(this_cpu(), irq);

  ret = esp_intr_disable(intr_handle);
  if (ret != ESP_OK)
    {
      irqerr("Failed to disable interrupt %d\n", irq);
    }
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
 *   Allocated CPU interrupt or a negated errno value on failure.
 *
 ****************************************************************************/

int esp_setup_irq(int source,
                  irq_priority_t priority,
                  int type,
                  xcpt_t handler,
                  void *arg)
{
  return esp_setup_irq_intrstatus(source, priority, type, 0, 0,
                                  handler, arg);
}

int esp_setup_irq_with_flags(int source,
                             int flags,
                             xcpt_t handler,
                             void *arg)
{
  return esp_setup_irq_with_flags_intrstatus(source, flags, 0, 0,
                                             handler, arg);
}

int esp_setup_irq_intrstatus(int source,
                             irq_priority_t priority,
                             int type,
                             uint32_t intrstatusreg,
                             uint32_t intrstatusmask,
                             xcpt_t handler,
                             void *arg)
{
  int flags;

  flags = (1 << priority);
  flags |= type == ESP_IRQ_TRIGGER_EDGE ? ESP_INTR_FLAG_EDGE : 0;
  flags |= ESP_INTR_FLAG_INTRDISABLED;

  return esp_setup_irq_with_flags_intrstatus(source,
                                             flags,
                                             intrstatusreg,
                                             intrstatusmask,
                                             handler,
                                             arg);
}

int esp_setup_irq_with_flags_intrstatus(int source,
                                        int flags,
                                        uint32_t intrstatusreg,
                                        uint32_t intrstatusmask,
                                        xcpt_t handler,
                                        void *arg)
{
  struct intr_adapter_from_nuttx *isr_adapter_args;
  esp_err_t ret;
  intr_handle_t ret_handle;
  int cpuint;
  int irq;

  irqinfo("source = %d\n", source);

  isr_adapter_args = kmm_calloc(1, sizeof(struct intr_adapter_from_nuttx));
  if (isr_adapter_args == NULL)
    {
      irqerr("Failed to kmm_calloc\n");
      return -EINVAL;
    }

  isr_adapter_args->func = handler;
  isr_adapter_args->arg = arg;

  ret = esp_intr_alloc_intrstatus(source,
                                  flags,
                                  intrstatusreg,
                                  intrstatusmask,
                                  isr_adapter_func,
                                  isr_adapter_args,
                                  &ret_handle);
  if (ret != ESP_OK)
    {
      irqerr("Failed to allocate interrupt for source %d\n", source);
      kmm_free(isr_adapter_args);
      return -EINVAL;
    }

  cpuint = esp_intr_get_intno(ret_handle);

  if (source < 0)
    {
      irq = source + ETS_INTERNAL_INTR_SOURCE_OFF;
    }
  else
    {
      irq = ESP_SOURCE2IRQ(source);
    }

  /* Store the handle. The handle already contains the CPU interrupt and
   * CPU information, so no additional mapping is needed.
   */

  esp_set_handle(this_cpu(), irq, ret_handle);

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
  esp_err_t ret;
  int cpu = this_cpu();
  int irq = ESP_SOURCE2IRQ(source);
  intr_handle_t intr_handle = esp_get_handle(cpu, irq);

  UNUSED(cpuint);

  if (intr_handle == IRQ_UNMAPPED)
    {
      irqwarn("No handle found for source %d\n", source);
      return;
    }

  ret = esp_intr_free(intr_handle);
  if (ret != ESP_OK)
    {
      irqerr("Failed to free interrupt %d\n", source);
    }

  esp_clear_handle(cpu, irq);
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
  int cpu = this_cpu();

  if (is_irq)
    {
      uint8_t cpuint = (mcause & VECTORS_MCAUSE_REASON_MASK) -
                       RV_EXTERNAL_INT_OFFSET;

      DEBUGASSERT(cpuint >= 0 && cpuint < ESP_NCPUINTS);

      irq = esp_cpuint_to_irq(cpuint, cpu);

      if (irq < 0)
        {
          /* No handle found for this CPU interrupt. This can happen
           * if the interrupt was triggered but not properly registered.
           */

          irqwarn("No IRQ found for cpuint=%d cpu=%d\n", cpuint, cpu);
          return regs;
        }

#ifdef CONFIG_ESPRESSIF_IRAM_ISR_DEBUG
      /* Check if non-IRAM interrupts are disabled */

      if (esp_intr_noniram_is_disabled(cpu))
        {
          /* Sum-up the IRAM-enabled counter associated with the IRQ */

          esp_irq_iram_interrupt_record(irq, this_cpu());
        }
#endif

      is_edge = esprv_int_get_type(cpuint) == INTR_TYPE_LEVEL;
      if (is_edge)
        {
          /* Clear edge interrupts. */

          esp_cpu_intr_edge_ack(cpuint);
        }
    }
  else
    {
      /* It's exception */

      irq = mcause &
            (VECTORS_MCAUSE_INTBIT_MASK | VECTORS_MCAUSE_REASON_MASK);
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
 * Name:  esp_set_handle
 *
 * Description:
 *   This function sets the handle associated with an IRQ
 *
 * Input Parameters:
 *   cpu - The CPU associated with the IRQ
 *   irq - The IRQ associated with a CPU interrupt
 *   handle - The handle to be associated with the IRQ
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int esp_set_handle(int cpu, int irq, intr_handle_t handle)
{
  intr_handle_t current_handle = g_handle_map[cpu][irq];

  if (current_handle != IRQ_UNMAPPED)
    {
      irqinfo("IRQ %d already has a handle\n", irq);
      return -EINVAL;
    }

  g_handle_map[cpu][irq] = handle;

  return OK;
}

/****************************************************************************
 * Name:  esp_get_handle
 *
 * Description:
 *   This function gets the handle associated with an IRQ
 *
 * Input Parameters:
 *   cpu - The CPU associated with the IRQ
 *   irq - The IRQ associated with a CPU interrupt
 *
 * Returned Value:
 *   The handle associated with the IRQ or IRQ_UNMAPPED if no handle is
 *   associated with the IRQ.
 *
 ****************************************************************************/

intr_handle_t esp_get_handle(int cpu, int irq)
{
  return g_handle_map[cpu][irq];
}

/****************************************************************************
 * Name:  esp_clear_handle
 *
 * Description:
 *   This function clears the handle associated with an IRQ
 *
 * Input Parameters:
 *   irq - The IRQ associated with a CPU interrupt
 *
 * Returned Value:
 *   The handle associated with the IRQ or IRQ_UNMAPPED if no handle is
 *   associated with the IRQ.
 *
 ****************************************************************************/

static void esp_clear_handle(int cpu, int irq)
{
  g_handle_map[cpu][irq] = IRQ_UNMAPPED;
}

/****************************************************************************
 * Name:  esp_get_cpuint
 *
 * Description:
 *   This function returns the CPU interrupt associated with an IRQ
 *
 * Input Parameters:
 *   cpu - The CPU associated with the IRQ
 *   irq - The IRQ associated with a CPU interrupt
 *
 * Returned Value:
 *   The CPU interrupt associated with such IRQ or a negated errno value on
 *   failure.
 *
 ****************************************************************************/

IRAM_ATTR int esp_get_cpuint(int cpu, int irq)
{
  intr_handle_t intr_handle = esp_get_handle(cpu, irq);

  if (intr_handle != IRQ_UNMAPPED && intr_handle != NULL)
    {
      return esp_intr_get_intno(intr_handle);
    }

  return -EINVAL;
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
 *   cpu -       The CPU to retrieve the interrupt records for
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_IRAM_ISR_DEBUG
void esp_get_iram_interrupt_records(uint64_t *irq_count, int cpu)
{
  irqstate_t flags = enter_critical_section();

  memcpy(irq_count, &g_iram_count[cpu], sizeof(uint64_t) * NR_IRQS);

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: esp_cpuint_to_irq
 *
 * Description:
 *   Find an IRQ associated with a given CPU interrupt by searching through
 *   g_handle_map. For shared interrupts, multiple IRQs may map to the same
 *   CPU interrupt - this function returns the first one found.
 *
 * Input Parameters:
 *   cpuint - The CPU interrupt number
 *   cpu    - The CPU core
 *
 * Returned Value:
 *   The IRQ number, or -1 if not found.
 *
 ****************************************************************************/

IRAM_ATTR int esp_cpuint_to_irq(int cpuint, int cpu)
{
  int irq;
  intr_handle_t handle;

  for (irq = 0; irq < NR_IRQS; irq++)
    {
      handle = g_handle_map[cpu][irq];
      if (handle != IRQ_UNMAPPED && handle != NULL)
        {
          if (esp_intr_get_intno(handle) == cpuint &&
              esp_intr_get_cpu(handle) == cpu)
            {
              return irq;
            }
        }
    }

  return -1;
}
