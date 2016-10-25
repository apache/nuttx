/****************************************************************************
 * arch/xtensa/src/esp32/esp32_irq.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "xtensa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32_INTSET(n)  ((1 << (n)) - 1)
#define ESP32_LEVEL_SET  ESP32_INTSET(ESP32_CPUINT_NLEVELPERIPHS)
#define ESP32_EDGE_SET   ESP32_INTSET(ESP32_CPUINT_NEDGEPERIPHS)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* g_intenable[] is a shadow copy of the per-CPU INTENABLE register
 * content.
 */

#ifdef CONFIG_SMP

static uint32_t g_intenable[CONFIG_SMP_NCPUS];

#else

static uint32_t g_intenable[1];

#endif

/* Bitsets for free, unallocated CPU interrupts */

static uint32_t g_level_ints = ESP32_LEVEL_SET;
static uint32_t g_edge_ints  = ESP32_EDGE_SET;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the CPU interrupt specified by 'cpuint'
 *
 ****************************************************************************/

void up_disable_irq(int cpuint)
{
#ifdef CONFIG_SMP
  int cpu;
#endif

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);

#ifdef CONFIG_SMP
  cpu = up_cpu_index();
  (void)xtensa_disable_cpuint(&g_intenable[cpu], (1ul << cpuint));
#else
  (void)xtensa_disable_cpuint(&g_intenable[0], (1ul << cpuint));
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Ensable the CPU interrupt specified by 'cpuint'
 *
 ****************************************************************************/

void up_enable_irq(int cpuint)
{
#ifdef CONFIG_SMP
  int cpu;
#endif

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);

#ifdef CONFIG_SMP
  cpu = up_cpu_index();
  (void)xtensa_enable_cpuint(&g_intenable[cpu], (1ul << cpuint));
#else
  (void)xtensa_enable_cpuint(&g_intenable[0], (1ul << cpuint));
#endif
}

/****************************************************************************
 * Name:  esp32_alloc_levelint
 *
 * Description:
 *   Allocate a level CPU interrupt
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, the allocated level-sensitive, CPU interrupt numbr is
 *   returned.  A negated errno is returned on failure.  The only possible
 *   failure is that all level-sensitive CPU interrupts have already been
 *   allocated.
 *
 ****************************************************************************/

int esp32_alloc_levelint(void)
{
  irqstate_t flags;
  uint32_t mask;
  int cpuint;
  int ret = -ENOMEM;

  /* Check if there are any level CPU interrupts available */

  flags = enter_critical_section();
  if ((g_level_ints & ESP32_LEVEL_SET) != 0)
    {
      /* Search for an unallocated CPU interrupt number in g_level_ints. */

      for (cpuint = 0; cpuint < ESP32_CPUINT_NLEVELPERIPHS; cpuint++)
        {
          /* If the bit corresponding to the CPU interrupt is '1', then
           * that CPU interrupt is available.
           */

          mask = (1ul << cpuint);
          if ((g_level_ints & mask) != 0)
            {
              /* Got it! */

              g_level_ints &= ~mask;
              ret = cpuint;
              break;
            }
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name:  esp32_free_levelint
 *
 * Description:
 *   Free a previoulsy allocated level CPU interrupt
 *
 * Input Parameters:
 *   The CPU interrupt number to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_free_levelint(int cpuint)
{
  irqstate_t flags;
  uint32_t mask;

  DEBUGASSERT(cpuint >= 0 && cpuint < ESP32_CPUINT_NLEVELPERIPHS);

  /* Mark the CPU interrupt as available */

  mask  = (1ul << cpuint);
  flags = enter_critical_section();
  DEBUGASSERT((g_level_ints & mask) == 0);
  g_level_ints |= mask;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name:  esp32_alloc_edgeint
 *
 * Description:
 *   Allocate an edge CPU interrupt
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, the allocated edge-sensitive, CPU interrupt numbr is
 *   returned.  A negated errno is returned on failure.  The only possible
 *   failure is that all edge-sensitive CPU interrupts have already been
 *   allocated.
 *
 ****************************************************************************/

int esp32_alloc_edgeint(void)
{
  irqstate_t flags;
  uint32_t mask;
  int cpuint;
  int ret = -ENOMEM;

  /* Check if there are any level CPU interrupts available */

  flags = enter_critical_section();
  if ((g_edge_ints & ESP32_EDGE_SET) != 0)
    {
      /* Search for an unallocated CPU interrupt number in g_edge_ints. */

      for (cpuint = 0; cpuint < ESP32_CPUINT_NEDGEPERIPHS; cpuint++)
        {
          /* If the bit corresponding to the CPU interrupt is '1', then
           * that CPU interrupt is available.
           */

          mask = (1ul << cpuint);
          if ((g_edge_ints & mask) != 0)
            {
              /* Got it! */

              g_edge_ints &= ~mask;
              ret = cpuint;
              break;
            }
        }
    }

  leave_critical_section(flags);
  return ret; 
}

/****************************************************************************
 * Name:  esp32_free_edgeint
 *
 * Description:
 *   Free a previoulsy allocated edge CPU interrupt
 *
 * Input Parameters:
 *   The CPU interrupt number to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_free_edgeint(int cpuint)
{
  irqstate_t flags;
  uint32_t mask;

  DEBUGASSERT(cpuint >= 0 && cpuint < ESP32_CPUINT_NEDGEPERIPHS);

  /* Mark the CPU interrupt as available */

  mask  = (1ul << cpuint);
  flags = enter_critical_section();
  DEBUGASSERT((g_edge_ints & mask) == 0);
  g_edge_ints |= mask;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name:  esp32_attach_peripheral
 *
 * Description:
 *   Attach a peripheral interupt to a CPU interrupt.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   periphid - The peripheral number from ira.h to be assigned.
 *   cpuint   - The CPU interrupt to receive the peripheral interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_attach_peripheral(int cpu, int periphid, int cpuint)
{
# warning Missing logic
}

/****************************************************************************
 * Name:  esp32_detach_peripheral
 *
 * Description:
 *   Detach a peripheral interupt from a CPU interrupt.
 *
 * Input Parameters:
 *   cpu    - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   cpuint - The CPU interrupt to receive the peripheral interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_detach_peripheral(int cpu, int cpuint)
{
# warning Missing logic
}
