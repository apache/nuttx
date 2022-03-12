/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_irq.c
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
#include <stdint.h>
#include <string.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/irq.h>
#include <arch/board/board.h>

#include "xtensa.h"
#ifdef CONFIG_ESP32S2_GPIO_IRQ
#include "esp32s2_gpio.h"
#endif
#include "esp32s2_irq.h"
#include "hardware/esp32s2_soc.h"
#include "hardware/esp32s2_system.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IRQ_UNMAPPED            0xff

/* CPU interrupts to peripheral mapping:
 *
 * Encoding: EPPPPPPP
 *  E: CPU interrupt status (0 = Disabled, 1 = Enabled).
 *  P: Attached peripheral.
 */

#define CPUINT_UNASSIGNED       0x7f
#define CPUINT_GETEN(m)         (((m) & 0x80) >> 0x07)
#define CPUINT_GETIRQ(m)        ((m) & 0x7f)
#define CPUINT_ASSIGN(c)        (((c) & 0x7f) | 0x80)
#define CPUINT_DISABLE(m)       ((m) & 0x7f)
#define CPUINT_ENABLE(m)        ((m) | 0x80)

/* Mapping Peripheral IDs to map register addresses. */

#define CORE_MAP_REGADDR(n)     (DR_REG_INTERRUPT_BASE + ((n) << 2))

/* CPU interrupts can be detached from any peripheral source by setting the
 * map register to an internal CPU interrupt (6, 7, 11, 15, 16, or 29).
 */

#define NO_CPUINT               ESP32S2_CPUINT_TIMER0

/* Priority range is 1-5 */

#define ESP32S2_MIN_PRIORITY    1
#define ESP32S2_MAX_PRIORITY    5
#define ESP32S2_PRIO_INDEX(p)   ((p) - ESP32S2_MIN_PRIORITY)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a reference to the current interrupt level
 * register storage structure.  It is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Maps a CPU interrupt to the IRQ of the attached peripheral interrupt */

static uint8_t g_cpu_intmap[ESP32S2_NCPUINTS];

static volatile uint8_t g_irqmap[NR_IRQS];

/* g_intenable is a shadow copy of the per-CPU INTENABLE register content */

static uint32_t g_intenable;

/* Bitsets for free, unallocated CPU interrupts available to peripheral
 * devices.
 */

static uint32_t g_cpu_freeints = ESP32S2_CPUINT_PERIPHSET;

/* Bitsets for each interrupt priority 1-5 */

static const uint32_t g_priority[5] =
{
  ESP32S2_INTPRI1_MASK,
  ESP32S2_INTPRI2_MASK,
  ESP32S2_INTPRI3_MASK,
  ESP32S2_INTPRI4_MASK,
  ESP32S2_INTPRI5_MASK
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32s2_getcpuint
 *
 * Description:
 *   Get a free CPU interrupt for a peripheral device.  This function will
 *   not ignore all of the pre-allocated CPU interrupts for internal
 *   devices.
 *
 * Input Parameters:
 *   intmask - mask of candidate CPU interrupts.  The CPU interrupt will be
 *             be allocated from free interrupts within this set
 *
 * Returned Value:
 *   On success, a CPU interrupt number is returned.
 *   A negated errno is returned on failure.
 *
 ****************************************************************************/

static int esp32s2_getcpuint(uint32_t intmask)
{
  uint32_t bitmask;
  uint32_t intset;
  int cpuint;
  int ret = -ENOMEM;

  /* Check if there are CPU interrupts with the requested properties
   * available.
   */

  intset = g_cpu_freeints & intmask;
  if (intset != 0)
    {
      /* Skip over initial unavailable CPU interrupts quickly in groups
       * of 8 interrupt.
       */

      for (cpuint = 0, bitmask = 0xff;
           cpuint <= ESP32S2_CPUINT_MAX && (intset & bitmask) == 0;
           cpuint += 8, bitmask <<= 8);

      /* Search for an unallocated CPU interrupt number in the remaining
       * intset.
       */

      for (; cpuint <= ESP32S2_CPUINT_MAX; cpuint++)
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
      xtensa_enable_cpuint(&g_intenable, 1ul << ret);
    }

  return ret;
}

/****************************************************************************
 * Name:  esp32s2_alloc_cpuint
 *
 * Description:
 *   Allocate a level CPU interrupt
 *
 * Input Parameters:
 *   priority - Priority of the CPU interrupt (1-5)
 *   type     - Interrupt type (level or edge).
 *
 * Returned Value:
 *   On success, the allocated CPU interrupt number is returned.
 *   A negated errno is returned on failure.  The only possible failure
 *   is that all CPU interrupts of the requested type have already been
 *   allocated.
 *
 ****************************************************************************/

static int esp32s2_alloc_cpuint(int priority, int type)
{
  uint32_t mask;

  DEBUGASSERT(priority >= ESP32S2_MIN_PRIORITY &&
              priority <= ESP32S2_MAX_PRIORITY);
  DEBUGASSERT(type == ESP32S2_CPUINT_LEVEL ||
              type == ESP32S2_CPUINT_EDGE);

  if (type == ESP32S2_CPUINT_LEVEL)
    {
      /* Check if there are any level CPU interrupts available at the
       * requested interrupt priority.
       */

      mask = g_priority[ESP32S2_PRIO_INDEX(priority)] &
              ESP32S2_CPUINT_LEVELSET;
    }
  else
    {
      /* Check if there are any edge CPU interrupts available at the
       * requested interrupt priority.
       */

      mask = g_priority[ESP32S2_PRIO_INDEX(priority)] &
              ESP32S2_CPUINT_EDGESET;
    }

  return esp32s2_getcpuint(mask);
}

/****************************************************************************
 * Name:  esp32s2_free_cpuint
 *
 * Description:
 *   Free a previously allocated CPU interrupt
 *
 * Input Parameters:
 *   The CPU interrupt number to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s2_free_cpuint(int cpuint)
{
  uint32_t bitmask;

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32S2_CPUINT_MAX);

  /* Mark the CPU interrupt as available */

  bitmask  = 1ul << cpuint;

  DEBUGASSERT((g_cpu_freeints & bitmask) == 0);

  g_cpu_freeints |= bitmask;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int i;
  for (i = 0; i < NR_IRQS; i++)
    {
      g_irqmap[i] = IRQ_UNMAPPED;
    }

  /* Hard code special cases. */

  g_irqmap[XTENSA_IRQ_TIMER0] = ESP32S2_CPUINT_TIMER0;

  g_irqmap[XTENSA_IRQ_SWINT]  = ESP32S2_CPUINT_SOFTWARE1;

  /* Initialize CPU interrupts */

  esp32s2_cpuint_initialize();

#ifdef CONFIG_ESP32S2_GPIO_IRQ
  /* Initialize GPIO interrupt support */

  esp32s2_gpioirqinitialize();
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts.  Also clears PS.EXCM */

  up_irq_enable();
#endif

  /* Attach the software interrupt */

  irq_attach(XTENSA_IRQ_SWINT, (xcpt_t)xtensa_swint, NULL);

  /* Enable the software CPU interrupt. */

  up_enable_irq(XTENSA_IRQ_SWINT);
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
  int cpuint = g_irqmap[irq];

  if (cpuint == IRQ_UNMAPPED)
    {
      /* This interrupt is already disabled. */

      return;
    }

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32S2_CPUINT_MAX);

  if (irq < XTENSA_NIRQ_INTERNAL)
    {
      /* This is an internal CPU interrupt, it cannot be disabled using
       * the Interrupt Matrix.
       */

      xtensa_disable_cpuint(&g_intenable, 1ul << cpuint);
    }
  else
    {
      /* A peripheral interrupt, use the Interrupt Matrix to disable it. */

      int periph = ESP32S2_IRQ2PERIPH(irq);
      uintptr_t regaddr = CORE_MAP_REGADDR(periph);

      DEBUGASSERT(periph >= 0 && periph < ESP32S2_NPERIPHERALS);

      g_cpu_intmap[cpuint] = CPUINT_DISABLE(g_cpu_intmap[cpuint]);
      putreg32(NO_CPUINT, regaddr);
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
  int cpuint = g_irqmap[irq];

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32S2_CPUINT_MAX);

  if (irq < XTENSA_NIRQ_INTERNAL)
    {
      /* Enable the CPU interrupt now for internal CPU. */

      xtensa_enable_cpuint(&g_intenable, (1ul << cpuint));
    }
  else
    {
      /* For peripheral interrupts, attach the interrupt to the peripheral;
       * the CPU interrupt was already enabled when allocated.
       */

      int periph = ESP32S2_IRQ2PERIPH(irq);
      uintptr_t regaddr = CORE_MAP_REGADDR(periph);

      DEBUGASSERT(periph >= 0 && periph < ESP32S2_NPERIPHERALS);

      g_cpu_intmap[cpuint] = CPUINT_ENABLE(g_cpu_intmap[cpuint]);
      putreg32(cpuint, regaddr);
    }
}

/****************************************************************************
 * Name:  esp32s2_cpuint_initialize
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

int esp32s2_cpuint_initialize(void)
{
  uintptr_t regaddr;
  int i;

  /* Disable all CPU interrupts on this CPU */

  xtensa_disable_all();

  /* Detach all interrupts from peripheral sources */

  for (i = 0; i < ESP32S2_NPERIPHERALS; i++)
    {
      regaddr = CORE_MAP_REGADDR(i);

      putreg32(NO_CPUINT, regaddr);
    }

  /* Indicate that no peripheral interrupts are assigned to CPU interrupts */

  memset(g_cpu_intmap, CPUINT_UNASSIGNED, ESP32S2_NCPUINTS);

  /* Special case the 6 internal interrupts.
   *
   *   CPU interrupt bit             IRQ number
   *   ----------------------------  ---------------------
   *   ESP32S2_CPUINT_TIMER0      6  XTENSA_IRQ_TIMER0  0
   *   ESP32S2_CPUINT_SOFTWARE0   7  Not yet defined
   *   ESP32S2_CPUINT_PROFILING  11  Not yet defined
   *   ESP32S2_CPUINT_TIMER1     15  XTENSA_IRQ_TIMER1  1
   *   ESP32S2_CPUINT_TIMER2     16  XTENSA_IRQ_TIMER2  2
   *   ESP32S2_CPUINT_SOFTWARE1  29  XTENSA_IRQ_SWINT   4
   */

  g_cpu_intmap[ESP32S2_CPUINT_TIMER0]    = CPUINT_ASSIGN(XTENSA_IRQ_TIMER0);
  g_cpu_intmap[ESP32S2_CPUINT_TIMER1]    = CPUINT_ASSIGN(XTENSA_IRQ_TIMER1);
  g_cpu_intmap[ESP32S2_CPUINT_TIMER2]    = CPUINT_ASSIGN(XTENSA_IRQ_TIMER2);
  g_cpu_intmap[ESP32S2_CPUINT_SOFTWARE1] = CPUINT_ASSIGN(XTENSA_IRQ_SWINT);

  return OK;
}

/****************************************************************************
 * Name:  esp32s2_setup_irq
 *
 * Description:
 *   This function sets up the IRQ. It allocates a CPU interrupt of the given
 *   priority and type and attaches it to the given peripheral.
 *
 * Input Parameters:
 *   periphid - The peripheral number from irq.h to be assigned to
 *              a CPU interrupt.
 *   priority - Interrupt's priority (1 - 5).
 *   type     - Interrupt's type (level or edge).
 *
 * Returned Value:
 *   The allocated CPU interrupt on success, a negated errno value on
 *   failure.
 *
 ****************************************************************************/

int esp32s2_setup_irq(int periphid, int priority, int type)
{
  irqstate_t irqstate;
  uintptr_t regaddr;
  int irq;
  int cpuint;

  irqstate = enter_critical_section();

  /* Setting up an IRQ includes the following steps:
   *    1. Allocate a CPU interrupt.
   *    2. Attach that CPU interrupt to the peripheral.
   *    3. Map the CPU interrupt to the IRQ to ease searching later.
   */

  cpuint = esp32s2_alloc_cpuint(priority, type);
  if (cpuint < 0)
    {
      irqerr("Unable to allocate CPU interrupt for priority=%d and type=%d",
             priority, type);
      leave_critical_section(irqstate);

      return cpuint;
    }

  irq = ESP32S2_PERIPH2IRQ(periphid);

  DEBUGASSERT(periphid >= 0 && periphid < ESP32S2_NPERIPHERALS);
  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32S2_CPUINT_MAX);
  DEBUGASSERT(g_cpu_intmap[cpuint] == CPUINT_UNASSIGNED);

  g_cpu_intmap[cpuint] = CPUINT_ASSIGN(periphid + XTENSA_IRQ_FIRSTPERIPH);
  g_irqmap[irq] = cpuint;
  regaddr = CORE_MAP_REGADDR(periphid);

  putreg32(cpuint, regaddr);

  leave_critical_section(irqstate);

  return cpuint;
}

/****************************************************************************
 * Name:  esp32s2_teardown_irq
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

void esp32s2_teardown_irq(int periphid, int cpuint)
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

  esp32s2_free_cpuint(cpuint);

  irq = ESP32S2_PERIPH2IRQ(periphid);

  DEBUGASSERT(periphid >= 0 && periphid < ESP32S2_NPERIPHERALS);

  DEBUGASSERT(g_cpu_intmap[cpuint] != CPUINT_UNASSIGNED);
  g_cpu_intmap[cpuint] = CPUINT_UNASSIGNED;
  g_irqmap[irq] = IRQ_UNMAPPED;
  regaddr = CORE_MAP_REGADDR(periphid);

  putreg32(NO_CPUINT, regaddr);

  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name: xtensa_int_decode
 *
 * Description:
 *   Determine the peripheral that generated the interrupt and dispatch
 *   handling to the registered interrupt handler via xtensa_irq_dispatch().
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

uint32_t *xtensa_int_decode(uint32_t cpuints, uint32_t *regs)
{
  uint32_t mask;
  int bit;

#ifdef CONFIG_ARCH_LEDS_CPU_ACTIVITY
  board_autoled_on(LED_CPU);
#endif

  /* Skip over zero bits, eight at a time */

  for (bit = 0, mask = 0xff;
       bit < ESP32S2_NCPUINTS && (cpuints & mask) == 0;
       bit += 8, mask <<= 8);

  /* Process each pending CPU interrupt */

  for (; bit < ESP32S2_NCPUINTS && cpuints != 0; bit++)
    {
      mask = 1 << bit;
      if ((cpuints & mask) != 0)
        {
          /* Extract the IRQ number from the mapping table */

          uint8_t irq = CPUINT_GETIRQ(g_cpu_intmap[bit]);

          DEBUGASSERT(CPUINT_GETEN(g_cpu_intmap[bit]));
          DEBUGASSERT(irq != CPUINT_UNASSIGNED);

          /* Clear software or edge-triggered interrupt */

           xtensa_intclear(mask);

          /* Dispatch the CPU interrupt.
           *
           * NOTE that regs may be altered in the case of an interrupt
           * level context switch.
           */

          regs = xtensa_irq_dispatch((int)irq, regs);

          /* Clear the bit in the pending interrupt so that perhaps
           * we can exit the look early.
           */

          cpuints &= ~mask;
        }
    }

  return regs;
}

