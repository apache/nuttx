/****************************************************************************
 * arch/risc-v/src/esp32c6/esp32c6_irq.c
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

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "riscv_internal.h"

#include "esp32c6.h"
#include "esp32c6_attr.h"

#include "esp32c6_irq.h"
#include "hardware/esp32c6_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C6_DEFAULT_INT_THRESHOLD   1

#define IRQ_UNMAPPED            0xff

/* CPU interrupts to peripheral mapping:
 *
 * Encoding: EPPPPPP
 *  E: CPU interrupt status (0 = Disabled, 1 = Enabled).
 *  P: Attached peripheral.
 */

#define CPUINT_UNASSIGNED       0x7f
#define CPUINT_GETEN(m)         (((m) & 0x80) >> 0x07)
#define CPUINT_GETIRQ(m)        ((m) & 0x7f)
#define CPUINT_ASSIGN(c)        (((c) & 0x7f) | 0x80)
#define CPUINT_DISABLE(m)       ((m) & 0x7f)
#define CPUINT_ENABLE(m)        ((m) | 0x80)

/* CPU interrupts can be detached from any peripheral source by setting the
 * map register to an internal CPU interrupt (1~31).
 */

#define NO_CPUINT               0

/* Priority range is 1-15 */

#define ESP32C6_MIN_PRIORITY    1
#define ESP32C6_MAX_PRIORITY    15
#define ESP32C6_PRIO_INDEX(p)   ((p) - ESP32C6_MIN_PRIORITY)

#define ESP32C6_WIFI_RESERVE_INT  (BIT(1))
#define ESP32C6_BLE_RESERVE_INT   (BIT(5) | BIT(8))
#define ESP32C6_DISABLED_INT      (BIT(6))
#define ESP32C6_INVALID_INT       (BIT(0) | BIT(3) | BIT(4) | BIT(7))

#define ESP32C6_RESERVE_INT       (ESP32C6_WIFI_RESERVE_INT | \
                                   ESP32C6_BLE_RESERVE_INT | \
                                   ESP32C6_DISABLED_INT | \
                                   ESP32C6_INVALID_INT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Maps a CPU interrupt to the IRQ of the attached peripheral interrupt */

static uint8_t g_cpu_intmap[ESP32C6_NCPUINTS];

static volatile uint8_t g_irqmap[NR_IRQS];

/* Bitsets for free, unallocated CPU interrupts available to peripheral
 * devices.
 */

static uint32_t g_cpu_freeints = ESP32C6_CPUINT_PERIPHSET &
                                 (~ESP32C6_RESERVE_INT);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32c6_getcpuint
 *
 * Description:
 *   Get a free CPU interrupt for a peripheral device.  This function will
 *   not ignore all of the pre-allocated CPU interrupts for internal
 *   devices.
 *
 * Returned Value:
 *   On success, a CPU interrupt number is returned.
 *   A negated errno is returned on failure.
 *
 ****************************************************************************/

static int esp32c6_getcpuint(void)
{
  uint32_t bitmask;
  uint32_t intset;
  int cpuint = 0;
  int ret = -ENOMEM;

  /* Check if there are CPU interrupts with the requested properties
   * available.
   */

  intset = g_cpu_freeints;
  if (intset != 0)
    {
      /* Skip over initial unavailable CPU interrupts quickly in groups
       * of 8 interrupt.
       */

      for (cpuint = 0, bitmask = 0xff;
           cpuint <= ESP32C6_CPUINT_MAX && (intset & bitmask) == 0;
           cpuint += 8, bitmask <<= 8);

      /* Search for an unallocated CPU interrupt number in the remaining
       * intset.
       */

      for (; cpuint <= ESP32C6_CPUINT_MAX; cpuint++)
        {
          /* If the bit corresponding to the CPU interrupt is '1', then
           * that CPU interrupt is available.
           */

          bitmask = 1ul << cpuint;
          if ((intset & bitmask) != 0)
            {
              /* Got it! */

              g_cpu_freeints &= ~bitmask;
              ret = cpuint;
              break;
            }
        }
    }

  /* Enable the CPU interrupt now.  The interrupt is still not attached
   * to any peripheral and thus has no effect.
   */

  if (ret >= 0)
    {
      setbits(1 << cpuint, PLIC_MXINT_ENABLE_REG);
    }

  return cpuint;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int periphid;

  /* Indicate that no peripheral interrupts are assigned to CPU interrupts */

  for (int i = 0; i < NR_IRQS; i++)
    {
      g_irqmap[i] = IRQ_UNMAPPED;
    }

  /* Clear all peripheral interrupts from "bootloader" */

  for (periphid = 0; periphid < ESP32C6_NPERIPHERALS; periphid++)
    {
      putreg32(0, DR_REG_INTERRUPT_MATRIX_BASE + periphid * 4);
    }

  /* Set CPU interrupt threshold level */

  putreg32(ESP32C6_DEFAULT_INT_THRESHOLD, PLIC_MXINT_THRESH_REG);

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
 *   Enable the interrupt specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  int cpuint = g_irqmap[irq];
  irqstate_t irqstate;

  irqinfo("irq=%d | cpuint=%d\n", irq, cpuint);

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32C6_CPUINT_MAX);

  irqstate = enter_critical_section();
  setbits(1 << cpuint, PLIC_MXINT_ENABLE_REG);
  SET_CSR(mie, 1 << cpuint);
  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the interrupt specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  int cpuint = g_irqmap[irq];

  irqinfo("irq=%d | cpuint=%d \n", irq, cpuint);

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32C6_CPUINT_MAX);

  if (cpuint == IRQ_UNMAPPED)
    {
      /* This interrupt is already disabled. */

      return;
    }
  else
    {
      irqstate_t irqstate;

      g_cpu_intmap[cpuint] = CPUINT_DISABLE(g_cpu_intmap[cpuint]);

      irqstate = enter_critical_section();
      CLEAR_CSR(mie, 1 << cpuint);
      resetbits(1 << cpuint, PLIC_MXINT_ENABLE_REG);
      leave_critical_section(irqstate);
    }
}

/****************************************************************************
 * Name: esp32c6_free_cpuint
 *
 * Description:
 *   Free CPU interrupt.
 *
 * Input Parameters:
 *   periphid - Peripheral ID.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c6_free_cpuint(uint8_t periphid)
{
  irqstate_t irqstate;
  uint8_t cpuint;

  DEBUGASSERT(periphid < ESP32C6_NPERIPHERALS);

  irqstate = enter_critical_section();

  /* Get the CPU interrupt ID mapped to this peripheral. */

  cpuint = getreg32(DR_REG_INTERRUPT_MATRIX_BASE + periphid * 4) & 0x1f;

  irqinfo("INFO: irq[%" PRIu8 "]=%" PRIu8 "\n", periphid, cpuint);

  if (cpuint != 0)
    {
      /* Undo the allocation process:
       *   1.  Unmap the peripheral from the CPU interrupt ID.
       *   2.  Reset the interrupt type.
       *   3.  Reset the interrupt priority.
       *   4.  Clear the CPU interrupt.
       */

      DEBUGASSERT(g_cpu_intmap[cpuint] != CPUINT_UNASSIGNED);

      g_cpu_intmap[cpuint] = CPUINT_UNASSIGNED;
      putreg32(0, DR_REG_INTERRUPT_MATRIX_BASE + periphid * 4);
      resetbits(1 << cpuint, PLIC_MXINT_TYPE_REG);
      putreg32(0, PLIC_MXINT0_PRI_REG + cpuint * 4);
      resetbits(1 << cpuint, PLIC_MXINT_ENABLE_REG);
    }

  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name:  esp32c6_cpuint_initialize
 *
 * Description:
 *   Initialize CPU interrupts
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp32c6_cpuint_initialize(void)
{
  /* Disable all CPU interrupts on this CPU */

  for (int i = 0; i < ESP32C6_NCPUINTS; i++)
    {
      putreg32(0, PLIC_MXINT0_PRI_REG + i * 4);
    }

  /* Detach all interrupts from peripheral sources */

  for (int i = 0; i < ESP32C6_NPERIPHERALS; i++)
    {
      putreg32(0, DR_REG_INTERRUPT_MATRIX_BASE + i * 4);
    }

  /* Indicate that no peripheral interrupts are assigned to CPU interrupts */

  memset(g_cpu_intmap, CPUINT_UNASSIGNED, ESP32C6_NCPUINTS);

  return OK;
}

/****************************************************************************
 * Name: esp32c6_bind_irq
 *
 * Description:
 *   Bind IRQ and resource with given parameters.
 *
 * Input Parameters:
 *   cpuint    - CPU interrupt ID
 *   periphid  - Peripheral ID
 *   prio      - Interrupt priority
 *   flags     - Interrupt flags
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c6_bind_irq(uint8_t cpuint, uint8_t periphid, uint8_t prio,
                      uint32_t flags)
{
  /* Disable the CPU interrupt. */

  resetbits(1 << cpuint, PLIC_MXINT_ENABLE_REG);

  /* Set the interrupt priority. */

  putreg32(prio, PLIC_MXINT0_PRI_REG + cpuint * 4);

  /* Set the interrupt type (Edge or Level). */

  if (flags & ESP32C6_INT_EDGE)
    {
      setbits(1 << cpuint, PLIC_MXINT_TYPE_REG);
    }
  else
    {
      resetbits(1 << cpuint, PLIC_MXINT_TYPE_REG);
    }

  /* Map the CPU interrupt ID to the peripheral. */

  putreg32(cpuint, DR_REG_INTERRUPT_MATRIX_BASE + periphid * 4);
}

/****************************************************************************
 * Name:  esp32c6_setup_irq
 *
 * Description:
 *   This function sets up the IRQ. It allocates a CPU interrupt of the given
 *   priority and type and attaches it to the given peripheral.
 *
 * Input Parameters:
 *   periphid - The peripheral number from irq.h to be assigned to
 *              a CPU interrupt.
 *   priority - Interrupt's priority (1 - 15).
 *   type     - Interrupt's type (level or edge).
 *
 * Returned Value:
 *   The allocated CPU interrupt on success, a negated errno value on
 *   failure.
 *
 ****************************************************************************/

int esp32c6_setup_irq(int periphid, int priority, int type)
{
  irqstate_t irqstate;
  int irq;
  int cpuint;

  irqinfo("periphid = %d\n", periphid);

  irqstate = enter_critical_section();

  /* Setting up an IRQ includes the following steps:
   *    1. Allocate a CPU interrupt.
   *    2. Attach that CPU interrupt to the peripheral.
   *    3. Map the CPU interrupt to the IRQ to ease searching later.
   */

  cpuint = esp32c6_getcpuint();
  if (cpuint < 0)
    {
      irqerr("Unable to allocate CPU interrupt for priority=%d and type=%d",
             priority, type);
      leave_critical_section(irqstate);

      return cpuint;
    }

  irq = ESP32C6_PERIPH2IRQ(periphid);

  DEBUGASSERT(periphid >= 0 && periphid < ESP32C6_NPERIPHERALS);
  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32C6_CPUINT_MAX);
  DEBUGASSERT(g_cpu_intmap[cpuint] == CPUINT_UNASSIGNED);

  g_cpu_intmap[cpuint] = CPUINT_ASSIGN(periphid + ESP32C6_IRQ_FIRSTPERIPH);
  g_irqmap[irq] = cpuint;

  esp32c6_bind_irq(cpuint, periphid, priority, type);

  leave_critical_section(irqstate);

  return cpuint;
}

/****************************************************************************
 * Name:  esp32c6_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by esp32c6_setup_irq.
 *   It detaches a peripheral interrupt from a CPU interrupt and frees the
 *   CPU interrupt.
 *
 * Input Parameters:
 *   periphid - The peripheral number from irq.h to be detached from the
 *              CPU interrupt.
 *   cpuint   - The CPU interrupt from which the peripheral interrupt will
 *              be detached.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c6_teardown_irq(int periphid, int cpuint)
{
  irqstate_t irqstate;
  uintptr_t regaddr;
  int irq;

  irqstate = enter_critical_section();

  /* Tearing down an IRQ includes the following steps:
   *   1. Free the previously allocated CPU interrupt.
   *   2. Detach the interrupt from the peripheral.
   *   3. Unmap the IRQ from the IRQ-to-cpuint map.
   */

  esp32c6_free_cpuint(cpuint);

  irq = ESP32C6_PERIPH2IRQ(periphid);

  DEBUGASSERT(periphid >= 0 && periphid < ESP32C6_NPERIPHERALS);

  DEBUGASSERT(g_cpu_intmap[cpuint] != CPUINT_UNASSIGNED);
  g_cpu_intmap[cpuint] = CPUINT_UNASSIGNED;
  g_irqmap[irq] = IRQ_UNMAPPED;
  regaddr = CORE_MAP_REGADDR(periphid);

  putreg32(NO_CPUINT, regaddr);

  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name: riscv_dispatch_irq
 *
 * Description:
 *   Process interrupt and its callback function.
 *
 * Input Parameters:
 *   mcause - RISC-V "mcause" register.
 *   regs   - Saved registers reference.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR uintptr_t *riscv_dispatch_irq(uintptr_t mcause, uintptr_t *regs)
{
  int irq;
  uint8_t cpuint = mcause & RISCV_IRQ_MASK;
  bool is_irq = (RISCV_IRQ_BIT & mcause) != 0;

  irqinfo("INFO: mcause=%08" PRIXPTR "\n", mcause);

  DEBUGASSERT(cpuint <= ESP32C6_CPUINT_MAX);

  irqinfo("INFO: cpuint=%" PRIu8 "\n", cpuint);

  if (is_irq)
    {
      /* Clear edge interrupts. */

      putreg32(1 << cpuint, PLIC_MXINT_CLEAR_REG);
      irq = CPUINT_GETIRQ(g_cpu_intmap[cpuint]);
    }
  else
    {
      /* It's exception */

      irq = mcause;
    }

  irqinfo("INFO: IRQ=%d\n", irq);

  regs = riscv_doirq(irq, regs);

  /* Toggle the bit back to zero. */

  if (is_irq)
    {
      putreg32(0, PLIC_MXINT_CLEAR_REG);
    }

  return regs;
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t flags;

  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */

  flags = READ_AND_SET_CSR(mstatus, MSTATUS_MIE);
  return flags;
}
