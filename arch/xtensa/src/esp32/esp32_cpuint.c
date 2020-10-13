/****************************************************************************
 * arch/xtensa/src/esp32/esp32_cpuint.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "hardware/esp32_dport.h"
#include "esp32_cpuint.h"
#include "xtensa.h"

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mapping Peripheral IDs to map register addresses
 *
 * PERIPHERAL ID                  DPORT REGISTER OFFSET
 * MNEMONIC                   VAL PRO CPU APP CPU
 * -------------------------- --- ------- -------
 * ESP32_PERIPH_MAC            0  0x104   0x218
 * ESP32_PERIPH_MAC_NMI        1  0x108   0x21c
 * ESP32_PERIPH_BB             2  0x10c   0x220
 * ESP32_PERIPH_BB_MAC         3  0x110   0x224
 * ESP32_PERIPH_BT_BB          4  0x114   0x228
 * ESP32_PERIPH_BT_BB_NMI      5  0x118   0x22c
 * ESP32_PERIPH_RWBT_IRQ       6  0x11c   0x230
 * ESP32_PERIPH_RWBLE_IRQ      7  0x120   0x234
 * ESP32_PERIPH_RWBT_NMI       8  0x124   0x238
 * ESP32_PERIPH_RWBLE_NMI      9  0x128   0x23c
 * ESP32_PERIPH_SLC0           10 0x12c   0x240
 * ESP32_PERIPH_SLC1           11 0x130   0x244
 * ESP32_PERIPH_UHCI0          12 0x134   0x248
 * ESP32_PERIPH_UHCI1          13 0x138   0x24c
 * ESP32_PERIPH_TG_T0_LEVEL    14 0x13c   0x250
 * ESP32_PERIPH_TG_T1_LEVEL    15 0x140   0x254
 * ESP32_PERIPH_TG_WDT_LEVEL   16 0x144   0x258
 * ESP32_PERIPH_TG_LACT_LEVEL  17 0x148   0x25c
 * ESP32_PERIPH_TG1_T0_LEVEL   18 0x14c   0x260
 * ESP32_PERIPH_TG1_T1_LEVEL   19 0x150   0x264
 * ESP32_PERIPH_TG1_WDT_LEVEL  20 0x154   0x268
 * ESP32_PERIPH_G1_LACT_LEVEL  21 0x158   0x26c
 * ESP32_PERIPH_CPU_GPIO       22 0x15c   0x270
 * ESP32_PERIPH_CPU_NMI        23 0x160   0x274
 * ESP32_PERIPH_CPU_CPU0       24 0x164   0x278
 * ESP32_PERIPH_CPU_CPU1       25 0x168   0x27c
 * ESP32_PERIPH_CPU_CPU2       26 0x16c   0x280
 * ESP32_PERIPH_CPU_CPU3       27 0x170   0x284
 * ESP32_PERIPH_SPI0           28 0x174   0x288
 * ESP32_PERIPH_SPI1           29 0x178   0x28c
 * ESP32_PERIPH_SPI2           30 0x17c   0x290
 * ESP32_PERIPH_SPI3           31 0x180   0x294
 * ESP32_PERIPH_I2S0           32 0x184   0x298
 * ESP32_PERIPH_I2S1           33 0x188   0x29c
 * ESP32_PERIPH_UART           34 0x18c   0x2a0
 * ESP32_PERIPH_UART1          35 0x190   0x2a4
 * ESP32_PERIPH_UART2          36 0x194   0x2a8
 * ESP32_PERIPH_SDIO_HOST      37 0x198   0x2ac
 * ESP32_PERIPH_EMAC           38 0x19c   0x2b0
 * ESP32_PERIPH_PWM0           39 0x1a0   0x2b4
 * ESP32_PERIPH_PWM1           40 0x1a4   0x2b8
 * ESP32_PERIPH_PWM2           41 0x1a8   0x2bc
 * ESP32_PERIPH_PWM3           42 0x1ac   0x2c0
 * ESP32_PERIPH_LEDC           43 0x1b0   0x2c4
 * ESP32_PERIPH_EFUSE          44 0x1b4   0x2c8
 * ESP32_PERIPH_CAN            45 0x1b8   0x2cc
 * ESP32_PERIPH_RTC_CORE       46 0x1bc   0x2d0
 * ESP32_PERIPH_RMT            47 0x1c0   0x2d4
 * ESP32_PERIPH_PCNT           48 0x1c4   0x2d8
 * ESP32_PERIPH_I2C_EXT0       49 0x1c8   0x2dc
 * ESP32_PERIPH_I2C_EXT1       50 0x1cc   0x2e0
 * ESP32_PERIPH_RSA            51 0x1d0   0x2e4
 * ESP32_PERIPH_SPI1_DMA       52 0x1d4   0x2e8
 * ESP32_PERIPH_SPI2_DMA       53 0x1d8   0x2ec
 * ESP32_PERIPH_SPI3_DMA       54 0x1dc   0x2f0
 * ESP32_PERIPH_WDG            55 0x1e0   0x2f4
 * ESP32_PERIPH_TIMER1         56 0x1e4   0x2f8
 * ESP32_PERIPH_TIMER2         57 0x1e8   0x2fc
 * ESP32_PERIPH_TG_T0_EDGE     58 0x1ec   0x300
 * ESP32_PERIPH_TG_T1_EDGE     59 0x1f0   0x304
 * ESP32_PERIPH_TG_WDT_EDGE    60 0x1F4   0x308
 * ESP32_PERIPH_TG_LACT_EDGE   61 0x1F8   0x30c
 * ESP32_PERIPH_TG1_T0_EDGE    62 0x1fc   0x310
 * ESP32_PERIPH_TG1_T1_EDGE    63 0x200   0x314
 * ESP32_PERIPH_TG1_WDT_EDGE   64 0x204   0x318
 * ESP32_PERIPH_TG1_LACT_EDGE  65 0x208   0x31c
 * ESP32_PERIPH_MMU_IA         66 0x20c   0x320
 * ESP32_PERIPH_MPU_IA         67 0x210   0x324
 * ESP32_PERIPH_CACHE_IA       68 0x214   0x328
 */

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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Maps a CPU interrupt to the IRQ of the attached peripheral interrupt */

uint8_t g_cpu0_intmap[ESP32_NCPUINTS];
#ifdef CONFIG_SMP
uint8_t g_cpu1_intmap[ESP32_NCPUINTS];
#endif

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

/* Bitsets for free, unallocated CPU interrupts available to peripheral
 * devices.
 */

static uint32_t g_cpu0_freeints = EPS32_CPUINT_PERIPHSET;
#ifdef CONFIG_SMP
static uint32_t g_cpu1_freeints = EPS32_CPUINT_PERIPHSET;
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
 * Name:  esp32_alloc_cpuint
 *
 * Description:
 *   Allocate a CPU interrupt for a peripheral device.  This function will
 *   not allocate any of the pre-allocated CPU interrupts for internal
 *   devices.
 *
 * Input Parameters:
 *   intmask - mask of candidate CPU interrupts.  The CPU interrupt will be
 *             be allocated from free interrupts within this set
 *
 * Returned Value:
 *   On success, the allocated level-sensitive, CPU interrupt numbr is
 *   returned.  A negated errno is returned on failure.  The only possible
 *   failure is that all level-sensitive CPU interrupts have already been
 *   allocated.
 *
 ****************************************************************************/

static int esp32_alloc_cpuint(uint32_t intmask)
{
  irqstate_t flags;
  uint32_t *freeints;
  uint32_t bitmask;
  uint32_t intset;
  int cpuint;
  int ret = -ENOMEM;

  /* Check if there are CPU interrupts with the requested properties
   * available.
   */

  flags = enter_critical_section();

#ifdef CONFIG_SMP
  if (this_cpu() != 0)
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

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  return OK;
}

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
  xtensa_disable_cpuint(&g_intenable[cpu], (1ul << cpuint));
#else
  xtensa_disable_cpuint(&g_intenable[0], (1ul << cpuint));
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the CPU interrupt specified by 'cpuint'
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
  xtensa_enable_cpuint(&g_intenable[cpu], (1ul << cpuint));
#else
  xtensa_enable_cpuint(&g_intenable[0], (1ul << cpuint));
#endif
}

/****************************************************************************
 * Name:  esp32_alloc_levelint
 *
 * Description:
 *   Allocate a level CPU interrupt
 *
 * Input Parameters:
 *   priority - Priority of the CPU interrupt (1-5)
 *
 * Returned Value:
 *   On success, the allocated level-sensitive, CPU interrupt number is
 *   returned.  A negated errno is returned on failure.  The only possible
 *   failure is that all level-sensitive CPU interrupts have already been
 *   allocated.
 *
 ****************************************************************************/

int esp32_alloc_levelint(int priority)
{
  uint32_t intmask;

  DEBUGASSERT(priority >= ESP32_MIN_PRIORITY &&
              priority <= ESP32_MAX_PRIORITY);

  /* Check if there are any level CPU interrupts available at the requested
   * interrupt priority.
   */

  intmask = g_priority[ESP32_PRIO_INDEX(priority)] & EPS32_CPUINT_LEVELSET;
  return esp32_alloc_cpuint(intmask);
}

/****************************************************************************
 * Name:  esp32_alloc_edgeint
 *
 * Description:
 *   Allocate an edge CPU interrupt
 *
 * Input Parameters:
 *   priority - Priority of the CPU interrupt (1-5)
 *
 * Returned Value:
 *   On success, the allocated edge-sensitive, CPU interrupt numbr is
 *   returned.  A negated errno is returned on failure.  The only possible
 *   failure is that all edge-sensitive CPU interrupts have already been
 *   allocated.
 *
 ****************************************************************************/

int esp32_alloc_edgeint(int priority)
{
  uint32_t intmask;

  DEBUGASSERT(priority >= ESP32_MIN_PRIORITY &&
              priority <= ESP32_MAX_PRIORITY);

  /* Check if there are any edge CPU interrupts available at the requested
   * interrupt priority.
   */

  intmask = g_priority[ESP32_PRIO_INDEX(priority)] & EPS32_CPUINT_EDGESET;
  return esp32_alloc_cpuint(intmask);
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

void esp32_free_cpuint(int cpuint)
{
  irqstate_t flags;
  uint32_t *freeints;
  uint32_t bitmask;

  DEBUGASSERT(cpuint >= 0 && cpuint < ESP32_CPUINT_NEDGEPERIPHS);

  /* Mark the CPU interrupt as available */

  bitmask  = (1ul << cpuint);
  flags = enter_critical_section();

#ifdef CONFIG_SMP
  if (this_cpu() != 0)
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
  leave_critical_section(flags);
}

/****************************************************************************
 * Name:  esp32_attach_peripheral
 *
 * Description:
 *   Attach a peripheral interrupt to a CPU interrupt.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0=PRO CPU 1=APP CPU
 *   periphid - The peripheral number from irq.h to be assigned to
 *              a CPU interrupt.
 *   cpuint   - The CPU interrupt to receive the peripheral interrupt
 *              assignment.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_attach_peripheral(int cpu, int periphid, int cpuint)
{
  uintptr_t regaddr;
  uint8_t *intmap;

  DEBUGASSERT(periphid >= 0 && periphid < ESP32_NPERIPHERALS);
  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32_CPUINT_MAX);
#ifdef CONFIG_SMP
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  if (cpu != 0)
    {
      regaddr = DPORT_APP_MAP_REGADDR(periphid);
      intmap  = g_cpu1_intmap;
    }
  else
#endif
    {
      regaddr = DPORT_PRO_MAP_REGADDR(periphid);
      intmap  = g_cpu0_intmap;
    }

  DEBUGASSERT(intmap[cpuint] == CPUINT_UNASSIGNED);
  intmap[cpuint] = periphid + XTENSA_IRQ_FIRSTPERIPH;

  putreg32(cpuint, regaddr);
}

/****************************************************************************
 * Name:  esp32_detach_peripheral
 *
 * Description:
 *   Detach a peripheral interrupt from a CPU interrupt.
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

void esp32_detach_peripheral(int cpu, int periphid, int cpuint)
{
  uintptr_t regaddr;
  uint8_t *intmap;

  DEBUGASSERT(periphid >= 0 && periphid < ESP32_NPERIPHERALS);
#ifdef CONFIG_SMP
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  if (cpu != 0)
    {
      regaddr = DPORT_APP_MAP_REGADDR(periphid);
      intmap  = g_cpu1_intmap;
    }
  else
#endif
    {
      regaddr = DPORT_PRO_MAP_REGADDR(periphid);
      intmap  = g_cpu0_intmap;
    }

  DEBUGASSERT(intmap[cpuint] != CPUINT_UNASSIGNED);
  intmap[cpuint] = CPUINT_UNASSIGNED;

  putreg32(NO_CPUINT, regaddr);
}
