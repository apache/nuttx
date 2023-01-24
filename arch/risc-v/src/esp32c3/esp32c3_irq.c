/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_irq.c
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
#include "hardware/esp32c3_interrupt.h"
#include "rom/esp32c3_spiflash.h"

#include "esp32c3.h"
#include "esp32c3_attr.h"
#include "esp32c3_gpio.h"

#include "esp32c3_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C3_DEFAULT_INT_THRESHOLD   1

#define IRQ_UNMAPPED            0xff

/* CPU interrupts to peripheral mapping:
 *
 * Encoding: EPPPPPP
 *  E: CPU interrupt status (0 = Disabled, 1 = Enabled).
 *  P: Attached peripheral.
 */

#define CPUINT_UNASSIGNED       0x3f
#define CPUINT_GETEN(m)         (((m) & 0x40) >> 0x06)
#define CPUINT_GETIRQ(m)        ((m) & 0x3f)
#define CPUINT_ASSIGN(c)        (((c) & 0x3f) | 0x40)
#define CPUINT_DISABLE(m)       ((m) & 0x3f)
#define CPUINT_ENABLE(m)        ((m) | 0x40)

/* Mapping Peripheral IDs to map register addresses. */

#define CORE_MAP_REGADDR(n)     (DR_REG_INTERRUPT_BASE + ((n) << 2))

/* CPU interrupts can be detached from any peripheral source by setting the
 * map register to an internal CPU interrupt (1~31).
 */

#define NO_CPUINT               0

/* Priority range is 1-15 */

#define ESP32C3_MIN_PRIORITY    1
#define ESP32C3_MAX_PRIORITY    15
#define ESP32C3_PRIO_INDEX(p)   ((p) - ESP32C3_MIN_PRIORITY)

#ifdef CONFIG_ESP32C3_WIFI
#  define ESP32C3_WIFI_RESERVE_INT  ((1 << ESP32C3_CPUINT_MAC | \
                                      1 << ESP32C3_CPUINT_MAC_NMI))
#else
#  define ESP32C3_WIFI_RESERVE_INT  0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Maps a CPU interrupt to the IRQ of the attached peripheral interrupt */

static uint8_t g_cpu_intmap[ESP32C3_NCPUINTS];

static volatile uint8_t g_irqmap[NR_IRQS];

/* Bitsets for free, unallocated CPU interrupts available to peripheral
 * devices.
 */

static uint32_t g_cpu_freeints = ESP32C3_CPUINT_PERIPHSET &
                                 (~ESP32C3_WIFI_RESERVE_INT);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32c3_getcpuint
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

static int esp32c3_getcpuint(void)
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
           cpuint <= ESP32C3_CPUINT_MAX && (intset & bitmask) == 0;
           cpuint += 8, bitmask <<= 8);

      /* Search for an unallocated CPU interrupt number in the remaining
       * intset.
       */

      for (; cpuint <= ESP32C3_CPUINT_MAX; cpuint++)
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
      setbits(1 << cpuint, INTERRUPT_CPU_INT_ENABLE_REG);
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
  /* Indicate that no peripheral interrupts are assigned to CPU interrupts */

  for (int i = 0; i < NR_IRQS; i++)
    {
      g_irqmap[i] = IRQ_UNMAPPED;
    }

  /* Hard code special cases. */

#ifdef CONFIG_ESP32C3_WIFI
  g_irqmap[ESP32C3_IRQ_MAC_NMI] = ESP32C3_CPUINT_MAC_NMI;
  g_cpu_intmap[ESP32C3_CPUINT_MAC_NMI]  = CPUINT_ASSIGN(ESP32C3_IRQ_MAC_NMI);
#endif

#ifdef CONFIG_ESP32C3_BLE
  g_irqmap[ESP32C3_IRQ_BT_BB] = ESP32C3_CPUINT_BT_BB;
  g_cpu_intmap[ESP32C3_CPUINT_BT_BB] = CPUINT_ASSIGN(ESP32C3_IRQ_BT_BB);

  g_irqmap[ESP32C3_IRQ_RWBLE] = ESP32C3_CPUINT_RWBLE;
  g_cpu_intmap[ESP32C3_CPUINT_RWBLE] = CPUINT_ASSIGN(ESP32C3_IRQ_RWBLE);
#endif

  /* Initialize CPU interrupts */

  esp32c3_cpuint_initialize();

  /* Attach the common interrupt handler */

  riscv_exception_attach();

#ifdef CONFIG_ESP32C3_GPIO_IRQ
  /* Initialize GPIO interrupt support */

  esp32c3_gpioirqinitialize();
#endif

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

  irqinfo("irq=%d | cpuint=%d \n", irq, cpuint);

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32C3_CPUINT_MAX);

  irqstate = enter_critical_section();
  setbits(1 << cpuint, INTERRUPT_CPU_INT_ENABLE_REG);
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

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32C3_CPUINT_MAX);

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
      resetbits(1 << cpuint, INTERRUPT_CPU_INT_ENABLE_REG);
      leave_critical_section(irqstate);
    }
}

/****************************************************************************
 * Name: esp32c3_free_cpuint
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

void esp32c3_free_cpuint(uint8_t periphid)
{
  irqstate_t irqstate;
  uint8_t cpuint;

  DEBUGASSERT(periphid < ESP32C3_NPERIPHERALS);

  irqstate = enter_critical_section();

  /* Get the CPU interrupt ID mapped to this peripheral. */

  cpuint = getreg32(DR_REG_INTERRUPT_BASE + periphid * 4) & 0x1f;

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
      putreg32(0, DR_REG_INTERRUPT_BASE + periphid * 4);
      resetbits(1 << cpuint, INTERRUPT_CPU_INT_TYPE_REG);
      putreg32(0, INTERRUPT_CPU_INT_PRI_0_REG + cpuint * 4);
      resetbits(1 << cpuint, INTERRUPT_CPU_INT_ENABLE_REG);
    }

  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name:  esp32c3_cpuint_initialize
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

int esp32c3_cpuint_initialize(void)
{
  /* Disable all CPU interrupts on this CPU */

  for (int i = 0; i < ESP32C3_NCPUINTS; i++)
    {
      putreg32(0, INTERRUPT_CPU_INT_PRI_0_REG + i * 4);
    }

  /* Detach all interrupts from peripheral sources */

  for (int i = 0; i < ESP32C3_NPERIPHERALS; i++)
    {
      putreg32(0, DR_REG_INTERRUPT_BASE + i * 4);
    }

  /* Set CPU interrupt threshold level */

  putreg32(ESP32C3_DEFAULT_INT_THRESHOLD, INTERRUPT_CPU_INT_THRESH_REG);

  /* Indicate that no peripheral interrupts are assigned to CPU interrupts */

  memset(g_cpu_intmap, CPUINT_UNASSIGNED, ESP32C3_NCPUINTS);

  return OK;
}

/****************************************************************************
 * Name: esp32c3_bind_irq
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

void esp32c3_bind_irq(uint8_t cpuint, uint8_t periphid, uint8_t prio,
                      uint32_t flags)
{
  /* Disable the CPU interrupt. */

  resetbits(1 << cpuint, INTERRUPT_CPU_INT_ENABLE_REG);

  /* Set the interrupt priority. */

  putreg32(prio, INTERRUPT_CPU_INT_PRI_0_REG + cpuint * 4);

  /* Set the interrupt type (Edge or Level). */

  if (flags & ESP32C3_INT_EDGE)
    {
      setbits(1 << cpuint, INTERRUPT_CPU_INT_TYPE_REG);
    }
  else
    {
      resetbits(1 << cpuint, INTERRUPT_CPU_INT_TYPE_REG);
    }

  /* Map the CPU interrupt ID to the peripheral. */

  putreg32(cpuint, DR_REG_INTERRUPT_BASE + periphid * 4);
}

/****************************************************************************
 * Name:  esp32c3_setup_irq
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

int esp32c3_setup_irq(int periphid, int priority, int type)
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

  cpuint = esp32c3_getcpuint();
  if (cpuint < 0)
    {
      irqerr("Unable to allocate CPU interrupt for priority=%d and type=%d",
             priority, type);
      leave_critical_section(irqstate);

      return cpuint;
    }

  irq = ESP32C3_PERIPH2IRQ(periphid);

  DEBUGASSERT(periphid >= 0 && periphid < ESP32C3_NPERIPHERALS);
  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32C3_CPUINT_MAX);
  DEBUGASSERT(g_cpu_intmap[cpuint] == CPUINT_UNASSIGNED);

  g_cpu_intmap[cpuint] = CPUINT_ASSIGN(periphid + ESP32C3_IRQ_FIRSTPERIPH);
  g_irqmap[irq] = cpuint;

  esp32c3_bind_irq(cpuint, periphid, priority, type);

  leave_critical_section(irqstate);

  return cpuint;
}

/****************************************************************************
 * Name:  esp32c3_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by esp32s2_setup_irq.
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

void esp32c3_teardown_irq(int periphid, int cpuint)
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

  esp32c3_free_cpuint(cpuint);

  irq = ESP32C3_PERIPH2IRQ(periphid);

  DEBUGASSERT(periphid >= 0 && periphid < ESP32C3_NPERIPHERALS);

  DEBUGASSERT(g_cpu_intmap[cpuint] != CPUINT_UNASSIGNED);
  g_cpu_intmap[cpuint] = CPUINT_UNASSIGNED;
  g_irqmap[irq] = IRQ_UNMAPPED;
  regaddr = CORE_MAP_REGADDR(periphid);

  putreg32(NO_CPUINT, regaddr);

  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name: riscv_int_decode
 *
 * Description:
 *   Determine the peripheral that generated the interrupt and dispatch
 *   handling to the registered interrupt handler via riscv_irq_dispatch().
 *
 * Input Parameters:
 *   cpuints - Set of pending interrupts valid for this level
 *   regs    - Saves processor state on the stack
 *
 * Returned Value:
 *   Normally the same value as regs is returned.  But, in the event of an
 *   interrupt level context switch, the returned value will, instead point
 *   to the saved processor state in the TCB of the newly started task.
 *
 ****************************************************************************/

#if 0
uint32_t *riscv_int_decode(uint32_t cpuints, uint32_t *regs)
{
  uint32_t mask;
  int bit;

#ifdef CONFIG_ARCH_LEDS_CPU_ACTIVITY
  board_autoled_on(LED_CPU);
#endif

  /* Skip over zero bits, eight at a time */

  for (bit = 0, mask = 0xff;
       bit < ESP32C3_NCPUINTS && (cpuints & mask) == 0;
       bit += 8, mask <<= 8);

  /* Process each pending CPU interrupt */

  for (; bit < ESP32C3_NCPUINTS && cpuints != 0; bit++)
    {
      mask = 1 << bit;
      if ((cpuints & mask) != 0)
        {
          /* Extract the IRQ number from the mapping table */

          uint8_t irq = CPUINT_GETIRQ(g_cpu_intmap[bit]);

          DEBUGASSERT(CPUINT_GETEN(g_cpu_intmap[bit]));
          DEBUGASSERT(irq != CPUINT_UNASSIGNED);

          /* Clear software or edge-triggered interrupt */

          riscv_intclear(mask);

          /* Dispatch the CPU interrupt.
           *
           * NOTE that regs may be altered in the case of an interrupt
           * level context switch.
           */

          regs = riscv_dispatch_irq((int)irq, regs);

          /* Clear the bit in the pending interrupt so that perhaps
           * we can exit the look early.
           */

          cpuints &= ~mask;
        }
    }

  return regs;
}
#endif

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

#ifdef CONFIG_ESP32C3_EXCEPTION_ENABLE_CACHE
  if (!is_irq &&
      (mcause != RISCV_IRQ_ECALLM))
    {
      if (!spi_flash_cache_enabled())
        {
          spi_flash_enable_cache(0);
          _err("ERROR: Cache was disabled and re-enabled\n");
        }
    }
#endif

  irqinfo("INFO: mcause=%08" PRIXPTR "\n", mcause);

  DEBUGASSERT(cpuint <= ESP32C3_CPUINT_MAX);

  irqinfo("INFO: cpuint=%" PRIu8 "\n", cpuint);

  if (is_irq)
    {
      /* Clear edge interrupts. */

      putreg32(1 << cpuint, INTERRUPT_CPU_INT_CLEAR_REG);
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
      putreg32(0, INTERRUPT_CPU_INT_CLEAR_REG);
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
