/****************************************************************************
 * arch/xtensa/src/esp32/esp32_irq.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "xtensa.h"

#include "hardware/esp32_dport.h"

#include "esp32_smp.h"
#include "esp32_gpio.h"

#include "esp32_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt stack definitions for SMP */

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
#  define INTSTACK_ALLOC (CONFIG_SMP_NCPUS * INTSTACK_SIZE)
#endif

#define IRQ_UNMAPPED      0xff
#define IRQ_GETCPU(m)     (((m) & 0x80) >> 0x07)
#define IRQ_GETCPUINT(m)  ((m) & 0x7f)
#define IRQ_MKMAP(c, i)   (((c) << 0x07) | (i))

/* Mapping Peripheral IDs to map register addresses. */

#define DPORT_PRO_MAP_REGADDR(n) (DR_REG_DPORT_BASE + 0x104 + ((n) << 2))
#define DPORT_APP_MAP_REGADDR(n) (DR_REG_DPORT_BASE + 0x218 + ((n) << 2))

/* CPU interrupts can be detached from any peripheral source by setting the
 * map register to an internal CPU interrupt (6, 7, 11, 15, 16, or 29).
 */

#define NO_CPUINT  ESP32_CPUINT_TIMER0

/* Priority range is 1-5 */

#define ESP32_MIN_PRIORITY     1
#define ESP32_MAX_PRIORITY     5
#define ESP32_PRIO_INDEX(p)    ((p) - ESP32_MIN_PRIORITY)

#ifdef CONFIG_ESP32_WIRELESS
#  define ESP32_WIRELESS_RESERVE_INT  (1 << ESP32_CPUINT_MAC)
#else
#  define ESP32_WIRELESS_RESERVE_INT  0
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a reference to the current interrupt level
 * register storage structure.  It is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

#ifdef CONFIG_SMP
/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];

#else

volatile uint32_t *g_current_regs[1];

#endif

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
/* In the SMP configuration, we will need custom interrupt stacks.
 * These definitions provide the aligned stack allocations.
 */

static uint32_t g_intstackalloc[INTSTACK_ALLOC >> 2];

/* These definitions provide the "top" of the push-down stacks. */

uintptr_t g_cpu_intstack_top[CONFIG_SMP_NCPUS] =
{
  (uintptr_t)g_intstackalloc + INTSTACK_SIZE,
#if CONFIG_SMP_NCPUS > 1
  (uintptr_t)g_intstackalloc + (2 * INTSTACK_SIZE),
#endif /* CONFIG_SMP_NCPUS > 1 */
};
#endif /* defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15 */

static volatile uint8_t g_irqmap[NR_IRQS];

/* Maps a CPU interrupt to the IRQ of the attached peripheral interrupt */

uint8_t g_cpu0_intmap[ESP32_NCPUINTS];
#ifdef CONFIG_SMP
uint8_t g_cpu1_intmap[ESP32_NCPUINTS];
#endif

/* g_intenable[] is a shadow copy of the per-CPU INTENABLE register
 * content.
 */

#ifdef CONFIG_SMP
uint32_t g_intenable[CONFIG_SMP_NCPUS];
#else
uint32_t g_intenable[1];
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Bitsets for free, unallocated CPU interrupts available to peripheral
 * devices.
 */

static uint32_t g_cpu0_freeints = ESP32_CPUINT_PERIPHSET &
                                  (~ESP32_WIRELESS_RESERVE_INT);
#ifdef CONFIG_SMP
static uint32_t g_cpu1_freeints = ESP32_CPUINT_PERIPHSET &
                                  (~ESP32_WIRELESS_RESERVE_INT);
#endif

/* Bitsets for each interrupt priority 1-5 */

static const uint32_t g_priority[5] =
{
  ESP32_INTPRI1_MASK,
  ESP32_INTPRI2_MASK,
  ESP32_INTPRI3_MASK,
  ESP32_INTPRI4_MASK,
  ESP32_INTPRI5_MASK
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_irq_dump
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void esp32_irq_dump(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();
#warning Missing logic
  leave_critical_section(flags);
}
#else
#  define esp32_irq_dump(msg, irq)
#endif

/****************************************************************************
 * Name: xtensa_attach_fromcpu1_interrupt
 ****************************************************************************/

#ifdef CONFIG_SMP
static inline void xtensa_attach_fromcpu1_interrupt(void)
{
  int cpuint;

  /* Connect all CPU peripheral source to allocated CPU interrupt */

  cpuint = esp32_setup_irq(0, ESP32_PERIPH_CPU_CPU1, 1, ESP32_CPUINT_LEVEL);
  DEBUGASSERT(cpuint >= 0);

  /* Attach the inter-CPU interrupt. */

  irq_attach(ESP32_IRQ_CPU_CPU1, (xcpt_t)esp32_fromcpu1_interrupt, NULL);

  /* Enable the inter 0 CPU interrupt. */

  up_enable_irq(ESP32_IRQ_CPU_CPU1);
}
#endif

/****************************************************************************
 * Name: xtensa_disable_all
 ****************************************************************************/

static inline void xtensa_disable_all(void)
{
  __asm__ __volatile__
  (
    "movi a2, 0\n"
    "xsr a2, INTENABLE\n"
    : : : "a2"
  );
}

/****************************************************************************
 * Name: esp32_intinfo
 *
 * Description:
 *    Return the CPU interrupt map of the given CPU and the register map
 *    of the given peripheral.
 *
 ****************************************************************************/

static void esp32_intinfo(int cpu, int periphid,
                          uintptr_t *regaddr, uint8_t **intmap)
{
#ifdef CONFIG_SMP
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  if (cpu != 0)
    {
      *regaddr = DPORT_APP_MAP_REGADDR(periphid);
      *intmap  = g_cpu1_intmap;
    }
  else
#endif
    {
      *regaddr = DPORT_PRO_MAP_REGADDR(periphid);
      *intmap  = g_cpu0_intmap;
    }
}

/****************************************************************************
 * Name:  esp32_getcpuint
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

static int esp32_getcpuint(uint32_t intmask)
{
  uint32_t *freeints;
  uint32_t bitmask;
  uint32_t intset;
  int cpuint;
  int ret = -ENOMEM;
  int cpu = 0;

  /* Check if there are CPU interrupts with the requested properties
   * available.
   */

  cpu = up_cpu_index();
#ifdef CONFIG_SMP
  if (cpu != 0)
    {
      freeints = &g_cpu1_freeints;
    }
  else
#endif
    {
      freeints = &g_cpu0_freeints;
    }

  intset = *freeints & intmask;
  if (intset != 0)
    {
      /* Skip over initial unavailable CPU interrupts quickly in groups
       * of 8 interrupt.
       */

      for (cpuint = 0, bitmask = 0xff;
           cpuint <= ESP32_CPUINT_MAX && (intset & bitmask) == 0;
           cpuint += 8, bitmask <<= 8);

      /* Search for an unallocated CPU interrupt number in the remaining
       * intset.
       */

      for (; cpuint <= ESP32_CPUINT_MAX; cpuint++)
        {
          /* If the bit corresponding to the CPU interrupt is '1', then
           * that CPU interrupt is available.
           */

          bitmask = (1ul << cpuint);
          if ((intset & bitmask) != 0)
            {
              /* Got it! */

              *freeints &= ~bitmask;
              ret = cpuint;
              break;
            }
        }
    }

  /* Make sure the CPU interrupt is disabled. */

  if (ret >= 0)
    {
      xtensa_disable_cpuint(&g_intenable[cpu], (1ul << ret));
    }

  return ret;
}

/****************************************************************************
 * Name:  esp32_alloc_cpuint
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

static int esp32_alloc_cpuint(int priority, int type)
{
  uint32_t mask;

  DEBUGASSERT(priority >= ESP32_MIN_PRIORITY &&
              priority <= ESP32_MAX_PRIORITY);
  DEBUGASSERT(type == ESP32_CPUINT_LEVEL ||
              type == ESP32_CPUINT_EDGE);

  if (type == ESP32_CPUINT_LEVEL)
    {
      /* Check if there are any level CPU interrupts available at the
       * requested interrupt priority.
       */

      mask = g_priority[ESP32_PRIO_INDEX(priority)] & ESP32_CPUINT_LEVELSET;
    }
  else
    {
      /* Check if there are any edge CPU interrupts available at the
       * requested interrupt priority.
       */

      mask = g_priority[ESP32_PRIO_INDEX(priority)] & ESP32_CPUINT_EDGESET;
    }

  return esp32_getcpuint(mask);
}

/****************************************************************************
 * Name:  esp32_free_cpuint
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

static void esp32_free_cpuint(int cpuint)
{
  uint32_t *freeints;
  uint32_t bitmask;

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);

  /* Mark the CPU interrupt as available */

  bitmask  = (1ul << cpuint);

#ifdef CONFIG_SMP
  if (up_cpu_index() != 0)
    {
      freeints = &g_cpu1_freeints;
    }
  else
#endif
    {
      freeints = &g_cpu0_freeints;
    }

  DEBUGASSERT((*freeints & bitmask) == 0);
  *freeints |= bitmask;
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

  g_irqmap[XTENSA_IRQ_TIMER0] = IRQ_MKMAP(0, ESP32_CPUINT_TIMER0);

#ifdef CONFIG_ESP32_WIRELESS
  g_irqmap[ESP32_IRQ_MAC] = IRQ_MKMAP(0, ESP32_CPUINT_MAC);
#endif

  /* Initialize CPU interrupts */

  esp32_cpuint_initialize();

  /* Attach and enable internal interrupts */

#ifdef CONFIG_SMP
  /* Attach and enable the inter-CPU interrupt */

  xtensa_attach_fromcpu1_interrupt();
#endif

  esp32_irq_dump("initial", NR_IRQS);

#ifdef CONFIG_ESP32_GPIO_IRQ
  /* Initialize GPIO interrupt support */

  esp32_gpioirqinitialize();
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts.  Also clears PS.EXCM */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name:  esp32_mapirq
 *
 * Description:
 *   Map the given IRQ to the CPU and allocated CPU interrupt.
 *
 * Input Parameters:
 *   irq      - IRQ number (see esp32/include/irq.h)
 *   cpu      - The CPU receiving the interrupt 0=PRO CPU 1=APP CPU
 *   cpuint   - The allocated CPU interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_mapirq(int irq, int cpu, int cpuint)
{
  DEBUGASSERT(irq >= 0 && irq < NR_IRQS);
  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);
#ifdef CONFIG_SMP
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);
#else
  DEBUGASSERT(cpu == 0);
#endif

  g_irqmap[irq] = IRQ_MKMAP(cpu, cpuint);
}

/****************************************************************************
 * Name:  esp32_unmapirq
 *
 * Description:
 *   Unmap the given IRQ.
 *
 * Input Parameters:
 *   irq      - IRQ number (see esp32/include/irq.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_unmapirq(int irq)
{
  DEBUGASSERT(irq >= 0 && irq < NR_IRQS);

  g_irqmap[irq] = IRQ_UNMAPPED;
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
  int cpu = up_cpu_index();
  int cpuint = IRQ_GETCPUINT(g_irqmap[irq]);

  if (g_irqmap[irq] == IRQ_UNMAPPED)
    {
      /* This interrupt is already disabled. */

      return;
    }

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);
#ifdef CONFIG_SMP
  DEBUGASSERT(cpu >= 0 && cpu <= CONFIG_SMP_NCPUS);
#else
  DEBUGASSERT(cpu == 0);
#endif

  xtensa_disable_cpuint(&g_intenable[cpu], (1ul << cpuint));
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
  int cpu = IRQ_GETCPU(g_irqmap[irq]);
  int cpuint = IRQ_GETCPUINT(g_irqmap[irq]);

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);
#ifdef CONFIG_SMP
  DEBUGASSERT(cpu >= 0 && cpu <= CONFIG_SMP_NCPUS);
#else
  DEBUGASSERT(cpu == 0);
#endif

  xtensa_enable_cpuint(&g_intenable[cpu], (1ul << cpuint));
}

/****************************************************************************
 * Name: xtensa_intstack_top
 *
 * Description:
 *   Return a pointer to the top of the correct interrupt stack for the
 *   given CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
uintptr_t xtensa_intstack_top(void)
{
  return g_cpu_intstack_top[up_cpu_index()];
}

/****************************************************************************
 * Name: xtensa_intstack_alloc
 *
 * Description:
 *   Return a pointer to the "alloc" the correct interrupt stack allocation
 *   for the current CPU.
 *
 ****************************************************************************/

uintptr_t xtensa_intstack_alloc(void)
{
  return g_cpu_intstack_top[up_cpu_index()] - INTSTACK_SIZE;
}
#endif

/****************************************************************************
 * Name:  esp32_cpuint_initialize
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

int esp32_cpuint_initialize(void)
{
  uintptr_t regaddr;
  uint8_t *intmap;
#ifdef CONFIG_SMP
  int cpu;
#endif
  int i;

#ifdef CONFIG_SMP
  /* Which CPU are we initializing */

  cpu = up_cpu_index();
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);
#endif

  /* Disable all CPU interrupts on this CPU */

  xtensa_disable_all();

  /* Detach all peripheral sources PRO CPU interrupts */

  for (i = 0; i < ESP32_NPERIPHERALS; i++)
    {
#ifdef CONFIG_SMP
      if (cpu != 0)
        {
          regaddr = DPORT_APP_MAP_REGADDR(i);
        }
      else
#endif
        {
          regaddr = DPORT_PRO_MAP_REGADDR(i);
        }

      putreg32(NO_CPUINT, regaddr);
    }

  /* Initialize CPU interrupt-to-IRQ mapping table */

#ifdef CONFIG_SMP
  if (cpu != 0)
    {
      intmap = g_cpu1_intmap;
    }
  else
#endif
    {
      intmap = g_cpu0_intmap;
    }

  /* Indicate that no peripheral interrupts are assigned to CPU interrupts */

  memset(intmap, CPUINT_UNASSIGNED, ESP32_NCPUINTS);

  /* Special case the 6 internal interrupts.
   *
   *   CPU interrupt bit           IRQ number
   *   --------------------------- ---------------------
   *   ESP32_CPUINT_MAC         0  ESP32_IRQ_MAC      4
   *   ESP32_CPUINT_TIMER0      6  XTENSA_IRQ_TIMER0  0
   *   ESP32_CPUINT_SOFTWARE0   7  Not yet defined
   *   ESP32_CPUINT_PROFILING  11  Not yet defined
   *   ESP32_CPUINT_TIMER1     15  XTENSA_IRQ_TIMER1  1
   *   ESP32_CPUINT_TIMER2     16  XTENSA_IRQ_TIMER2  2
   *   ESP32_CPUINT_SOFTWARE1  29  Not yet defined
   */

  intmap[ESP32_CPUINT_TIMER0] = XTENSA_IRQ_TIMER0;
  intmap[ESP32_CPUINT_TIMER1] = XTENSA_IRQ_TIMER1;
  intmap[ESP32_CPUINT_TIMER2] = XTENSA_IRQ_TIMER2;

  /* Reserve CPU interrupt for some special drivers */

#ifdef CONFIG_ESP32_WIRELESS
  intmap[ESP32_CPUINT_MAC]    = ESP32_IRQ_MAC;
#endif

  return OK;
}

/****************************************************************************
 * Name:  esp32_setup_irq
 *
 * Description:
 *   This function sets up the IRQ. It allocates a CPU interrupt of the given
 *   priority and type and attaches it to the given peripheral.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
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

int esp32_setup_irq(int cpu, int periphid, int priority, int type)
{
  irqstate_t irqstate;
  uintptr_t regaddr;
  uint8_t *intmap;
  int irq;
  int cpuint;

  irqstate = enter_critical_section();

  /* Setting up an IRQ includes the following steps:
   *    1. Allocate a CPU interrupt.
   *    2. Attach that CPU interrupt to the peripheral.
   *    3. Map the CPU interrupt to the IRQ to ease searching later.
   */

  cpuint = esp32_alloc_cpuint(priority, type);
  if (cpuint < 0)
    {
      irqerr("Unable to allocate CPU interrupt for priority=%d and type=%d",
             priority, type);
      leave_critical_section(irqstate);

      return cpuint;
    }

  irq = ESP32_PERIPH2IRQ(periphid);

  DEBUGASSERT(periphid >= 0 && periphid < ESP32_NPERIPHERALS);
  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);

  esp32_intinfo(cpu, periphid, &regaddr, &intmap);

  DEBUGASSERT(intmap[cpuint] == CPUINT_UNASSIGNED);

  intmap[cpuint] = periphid + XTENSA_IRQ_FIRSTPERIPH;
  esp32_mapirq(irq, cpu, cpuint);

  putreg32(cpuint, regaddr);

  leave_critical_section(irqstate);

  return cpuint;
}

/****************************************************************************
 * Name:  esp32_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by esp32_setup_irq.
 *   It detaches a peripheral interrupt from a CPU interrupt and frees the
 *   CPU interrupt.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   periphid - The peripheral number from irq.h to be detached from the
 *              CPU interrupt.
 *   cpuint   - The CPU interrupt from which the peripheral interrupt will
 *              be detached.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_teardown_irq(int cpu, int periphid, int cpuint)
{
  irqstate_t irqstate;
  uintptr_t regaddr;
  uint8_t *intmap;
  int irq;

  irqstate = enter_critical_section();

  /* Tearing down an IRQ includes the following steps:
   *   1. Free the previously allocated CPU interrupt.
   *   2. Detach the interrupt from the peripheral.
   *   3. Unmap the IRQ from the IRQ-to-cpuint map.
   */

  esp32_free_cpuint(cpuint);

  irq = ESP32_PERIPH2IRQ(periphid);

  DEBUGASSERT(periphid >= 0 && periphid < ESP32_NPERIPHERALS);

  esp32_intinfo(cpu, periphid, &regaddr, &intmap);

  DEBUGASSERT(intmap[cpuint] != CPUINT_UNASSIGNED);
  intmap[cpuint] = CPUINT_UNASSIGNED;
  esp32_unmapirq(irq);

  putreg32(NO_CPUINT, regaddr);

  leave_critical_section(irqstate);
}

