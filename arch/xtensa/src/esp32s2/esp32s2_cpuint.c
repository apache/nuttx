/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_cpuint.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "esp32s2_cpuint.h"
#include "hardware/esp32s2_interrupt.h"
#include "xtensa.h"

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mapping Peripheral IDs to map register addresses
 *
 * PERIPHERAL ID                                  INTERRUPT_PRO_X_MAP
 * MNEMONIC                                       REGISTER OFFSET
 * ESP32S2_PERIPH_MAC_INTR                        0x0000
 * ESP32S2_PERIPH_MAC_NMI                         0x0004
 * ESP32S2_PERIPH_PWR_INTR                        0x0008
 * ESP32S2_PERIPH_BB_INT                          0x000C
 * ESP32S2_PERIPH_BT_MAC_INT                      0x0010
 * ESP32S2_PERIPH_BT_BB_INT                       0x0014
 * ESP32S2_PERIPH_BT_BB_NMI                       0x0018
 * ESP32S2_PERIPH_RWBT_IRQ                        0x001C
 * ESP32S2_PERIPH_RWBLE_IRQ                       0x0020
 * ESP32S2_PERIPH_RWBT_NMI                        0x0024
 * ESP32S2_PERIPH_RWBLE_NMI                       0x0028
 * ESP32S2_PERIPH_SLC0_INTR                       0x002C
 * ESP32S2_PERIPH_SLC1_INTR                       0x0030
 * ESP32S2_PERIPH_UHCI0_INTR                      0x0034
 * ESP32S2_PERIPH_UHCI1_INTR                      0x0038
 * ESP32S2_PERIPH_TG_T0_LEVEL_INT                 0x003C
 * ESP32S2_PERIPH_TG_T1_LEVEL_INT                 0x0040
 * ESP32S2_PERIPH_TG_WDT_LEVEL_INT                0x0044
 * ESP32S2_PERIPH_TG_LACT_LEVEL_INT               0x0048
 * ESP32S2_PERIPH_TG1_T0_LEVEL_INT                0x004C
 * ESP32S2_PERIPH_TG1_T1_LEVEL_INT                0x0050
 * ESP32S2_PERIPH_TG1_WDT_LEVEL_INT               0x0054
 * ESP32S2_PERIPH_TG1_LACT_LEVEL_INT              0x0058
 * ESP32S2_PERIPH_GPIO_INTERRUPT_PRO              0x005C
 * ESP32S2_PERIPH_GPIO_INTERRUPT_PRO_NMI          0x0060
 * ESP32S2_PERIPH_GPIO_INTERRUPT_APP              0x0064
 * ESP32S2_PERIPH_GPIO_INTERRUPT_APP_NMI          0x0068
 * ESP32S2_PERIPH_DEDICATED_GPIO_IN_INTR          0x006C
 * ESP32S2_PERIPH_CPU_INTR_FROM_CPU_0             0x0070
 * ESP32S2_PERIPH_CPU_INTR_FROM_CPU_1             0x0074
 * ESP32S2_PERIPH_CPU_INTR_FROM_CPU_2             0x0078
 * ESP32S2_PERIPH_CPU_INTR_FROM_CPU_3             0x007C
 * ESP32S2_PERIPH_SPI_INTR_1                      0x0080
 * ESP32S2_PERIPH_SPI_INTR_2                      0x0084
 * ESP32S2_PERIPH_SPI_INTR_3                      0x0088
 * ESP32S2_PERIPH_I2S0_INT                        0x008C
 * ESP32S2_PERIPH_I2S1_INT                        0x0090
 * ESP32S2_PERIPH_UART_INT                        0x0094
 * ESP32S2_PERIPH_UART1_INT                       0x0098
 * ESP32S2_PERIPH_UART2_INT                       0x009C
 * ESP32S2_PERIPH_SDIO_HOST_INTERRUPT             0x00A0
 * ESP32S2_PERIPH_PWM0_INTR                       0x00A4
 * ESP32S2_PERIPH_PWM1_INTR                       0x00A8
 * ESP32S2_PERIPH_PWM2_INTR                       0x00AC
 * ESP32S2_PERIPH_PWM3_INTR                       0x00B0
 * ESP32S2_PERIPH_LEDC_INTR                       0x00B4
 * ESP32S2_PERIPH_EFUSE_INT                       0x00B8
 * ESP32S2_PERIPH_CAN_INT                         0x00BC
 * ESP32S2_PERIPH_USB_INT                         0x00C0
 * ESP32S2_PERIPH_RTC_CORE_INTR                   0x00C4
 * ESP32S2_PERIPH_RMT_INTR                        0x00C8
 * ESP32S2_PERIPH_PCNT_INTR                       0x00CC
 * ESP32S2_PERIPH_I2C_EXT0_INTR                   0x00D0
 * ESP32S2_PERIPH_I2C_EXT1_INTR                   0x00D4
 * ESP32S2_PERIPH_RSA_INTR                        0x00D8
 * ESP32S2_PERIPH_SHA_INTR                        0x00DC
 * ESP32S2_PERIPH_AES_INTR                        0x00E0
 * ESP32S2_PERIPH_SPI2_DMA_INT                    0x00E4
 * ESP32S2_PERIPH_SPI3_DMA_INT                    0x00E8
 * ESP32S2_PERIPH_WDG_INT                         0x00EC
 * ESP32S2_PERIPH_TIMER_INT1                      0x00F0
 * ESP32S2_PERIPH_TIMER_INT2                      0x00F4
 * ESP32S2_PERIPH_TG_T0_EDGE_INT                  0x00F8
 * ESP32S2_PERIPH_TG_T1_EDGE_INT                  0x00FC
 * ESP32S2_PERIPH_TG_WDT_EDGE_INT                 0x0100
 * ESP32S2_PERIPH_TG_LACT_EDGE_INT                0x0104
 * ESP32S2_PERIPH_TG1_T0_EDGE_INT                 0x0108
 * ESP32S2_PERIPH_TG1_T1_EDGE_INT                 0x010C
 * ESP32S2_PERIPH_TG1_WDT_EDGE_INT                0x0110
 * ESP32S2_PERIPH_TG1_LACT_EDGE_INT               0x0114
 * ESP32S2_PERIPH_CACHE_IA_INT                    0x0118
 * ESP32S2_PERIPH_SYSTIMER_TARGET0_INT            0x011C
 * ESP32S2_PERIPH_SYSTIMER_TARGET1_INT            0x0120
 * ESP32S2_PERIPH_SYSTIMER_TARGET2                0x0124
 * ESP32S2_PERIPH_ASSIST_DEBUG_INTR               0x0128
 * ESP32S2_PERIPH_PMS_PRO_IRAM0_ILG               0x012C
 * ESP32S2_PERIPH_PMS_PRO_DRAM0_ILG               0x0130
 * ESP32S2_PERIPH_PMS_PRO_DPORT_ILG               0x0134
 * ESP32S2_PERIPH_PMS_PRO_AHB_ILG                 0x0138
 * ESP32S2_PERIPH_PMS_PRO_CACHE_ILG               0x013C
 * ESP32S2_PERIPH_PMS_DMA_APB_I_ILG               0x0140
 * ESP32S2_PERIPH_PMS_DMA_RX_I_ILG                0x0144
 * ESP32S2_PERIPH_PMS_DMA_TX_I_ILG                0x0148
 * ESP32S2_PERIPH_SPI_MEM_REJECT_INTR             0x014C
 * ESP32S2_PERIPH_DMA_COPY_INTR                   0x0150
 * ESP32S2_PERIPH_SPI4_DMA_INT                    0x0154
 * ESP32S2_PERIPH_SPI_INTR_4                      0x0158
 * ESP32S2_PERIPH_DCACHE_PRELOAD_INT              0x015C
 * ESP32S2_PERIPH_ICACHE_PRELOAD_INT              0x0160
 * ESP32S2_PERIPH_APB_ADC_INT                     0x0164
 * ESP32S2_PERIPH_CRYPTO_DMA_INT                  0x0168
 * ESP32S2_PERIPH_CPU_PERI_ERROR_INT              0x016C
 * ESP32S2_PERIPH_APB_PERI_ERROR_INT              0x0170
 * ESP32S2_PERIPH_DCACHE_SYNC_INT                 0x0174
 * ESP32S2_PERIPH_ICACHE_SYNC_INT                 0x0178
 * ESP32S2_PERIPH_NMI                             0x0188
 */

#define INTERRUPT_PRO_X_MAP_REG(n) (DR_REG_INTERRUPT_BASE + ((n) << 2))

/* CPU interrupts can be detached from any peripheral source by setting the
 * map register to an internal CPU interrupt (6, 7, 11, 15, 16, or 29).
 */

#define NO_CPUINT  ESP32S2_CPUINT_TIMER0

/* Priority range is 1-5 */

#define ESP32S2_MIN_PRIORITY     1
#define ESP32S2_MAX_PRIORITY     5
#define ESP32S2_PRIO_INDEX(p)    ((p) - ESP32S2_MIN_PRIORITY)

#ifdef CONFIG_ESP32S2_WIRELESS
#  define ESP32S2_WIRELESS_RESERVE_INT  (1 << ESP32S2_CPUINT_MAC)
#else
#  define ESP32S2_WIRELESS_RESERVE_INT  0
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Maps a CPU interrupt to the IRQ of the attached peripheral interrupt */

uint8_t g_cpu0_intmap[ESP32S2_NCPUINTS];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* g_intenable[] is a shadow copy of the CPU INTENABLE register
 * content.
 */

static uint32_t g_intenable[1];

/* Bitsets for free, unallocated CPU interrupts available to peripheral
 * devices.
 */

static uint32_t g_cpu0_freeints = ESP32S2_CPUINT_PERIPHSET &
                                  (~ESP32S2_WIRELESS_RESERVE_INT);

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
 * Name:  esp32s2_alloc_cpuint
 *
 * Description:
 *   Allocate a CPU interrupt for a peripheral device.  This function will
 *   not allocate any of the pre-allocated CPU interrupts for internal
 *   devices. This current implementation is not supporting multiple
 *   peripheral interrupts maped to a single CPU interrupt.
 *
 * Input Parameters:
 *   intmask - Mask of candidate CPU interrupts.  The CPU interrupt will be
 *             be allocated from free interrupts within this set.
 *
 * Returned Value:
 *   On success, the first available CPU interrupt accordingly to the passed
 *   intmask. If no one is available return -ENOMEM.
 *
 ****************************************************************************/

static int esp32s2_alloc_cpuint(uint32_t intmask)
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

  freeints = &g_cpu0_freeints;

  intset = *freeints & intmask;
  if (intset != 0)
    {
      /* Skip over initial unavailable CPU interrupts quickly in groups
       * of 8 interrupt until find the first slot with the required
       * CPU interrupt set.
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

          bitmask = (1ul << cpuint);
          if ((intset & bitmask) != 0)
            {
              /* Got it!
               * Update the available CPU interrupts mask
               * and return the cpuint.
               */

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
 * Name:  esp32s2_cpuint_initialize
 *
 * Description:
 *   Initialize CPU interrupts.
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
  uint8_t *intmap;
  int i;

  /* Disable all CPU interrupts on this CPU */

  xtensa_disable_all();

  /* Detach all peripheral sources PRO CPU interrupts */

  for (i = 0; i < ESP32S2_NPERIPHERALS; i++)
    {
      regaddr = INTERRUPT_PRO_X_MAP_REG(i);

      putreg32(NO_CPUINT, regaddr);
    }

  /* Initialize CPU interrupt-to-IRQ mapping table */

  intmap = g_cpu0_intmap;

  /* Indicate that no peripheral interrupts are assigned to CPU interrupts */

  memset(intmap, CPUINT_UNASSIGNED, ESP32S2_NCPUINTS);

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

  intmap[ESP32S2_CPUINT_TIMER0]    = XTENSA_IRQ_TIMER0;
  intmap[ESP32S2_CPUINT_TIMER1]    = XTENSA_IRQ_TIMER1;
  intmap[ESP32S2_CPUINT_TIMER2]    = XTENSA_IRQ_TIMER2;
  intmap[ESP32S2_CPUINT_SOFTWARE1] = XTENSA_IRQ_SWINT;

  /* Reserve CPU interrupt for some special drivers */

#ifdef CONFIG_ESP32S2_WIRELESS
  intmap[ESP32S2_CPUINT_MAC]    = ESP32S2_IRQ_MAC;
#endif

  return OK;
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the CPU interrupt specified by 'cpuint'.
 *
 * Input Parameters:
 *  cpuint         - The CPU interrupt to disable.
 *
 ****************************************************************************/

void up_disable_irq(int cpuint)
{
  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32S2_CPUINT_MAX);

  xtensa_disable_cpuint(&g_intenable[0], (1ul << cpuint));
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the CPU interrupt specified by 'cpuint'.
 *
 * Input Parameters:
 *  cpuint         - The CPU interrupt to disable.
 *
 ****************************************************************************/

void up_enable_irq(int cpuint)
{
  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32S2_CPUINT_MAX);

  xtensa_enable_cpuint(&g_intenable[0], (1ul << cpuint));
}

/****************************************************************************
 * Name:  esp32s2_alloc_levelint
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

int esp32s2_alloc_levelint(int priority)
{
  uint32_t intmask;

  DEBUGASSERT(priority >= ESP32S2_MIN_PRIORITY &&
              priority <= ESP32S2_MAX_PRIORITY);

  /* Check if there are any level CPU interrupts available at the requested
   * interrupt priority.
   */

  intmask = g_priority[ESP32S2_PRIO_INDEX(priority)] &
            ESP32S2_CPUINT_LEVELSET;
  return esp32s2_alloc_cpuint(intmask);
}

/****************************************************************************
 * Name:  esp32s2_alloc_edgeint
 *
 * Description:
 *   Allocate an edge CPU interrupt
 *
 * Input Parameters:
 *   priority - Priority of the CPU interrupt (1-5)
 *
 * Returned Value:
 *   On success, the allocated edge-sensitive, CPU interrupt number is
 *   returned.  A negated errno is returned on failure.  The only possible
 *   failure is that all edge-sensitive CPU interrupts have already been
 *   allocated.
 *
 ****************************************************************************/

int esp32s2_alloc_edgeint(int priority)
{
  uint32_t intmask;

  DEBUGASSERT(priority >= ESP32S2_MIN_PRIORITY &&
              priority <= ESP32S2_MAX_PRIORITY);

  /* Check if there are any edge CPU interrupts available at the requested
   * interrupt priority.
   */

  intmask = g_priority[ESP32S2_PRIO_INDEX(priority)] &
            ESP32S2_CPUINT_EDGESET;
  return esp32s2_alloc_cpuint(intmask);
}

/****************************************************************************
 * Name:  esp32s2_free_cpuint
 *
 * Description:
 *   Free a previously allocated CPU interrupt by making it available in the
 *   g_cpu0_freeints.
 *
 * Input Parameters:
 *   cpuint         - The CPU interrupt number to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s2_free_cpuint(int cpuint)
{
  irqstate_t flags;
  uint32_t *freeints;
  uint32_t bitmask;

  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32S2_CPUINT_MAX);

  /* Mark the CPU interrupt as available */

  bitmask  = (1ul << cpuint);
  flags = enter_critical_section();

  freeints = &g_cpu0_freeints;

  DEBUGASSERT((*freeints & bitmask) == 0);
  *freeints |= bitmask;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name:  esp32s2_attach_peripheral
 *
 * Description:
 *   Attach a peripheral interrupt to a CPU interrupt.
 *   This function may be called after esp32s2_alloc_edgeint or
 *   esp32s2_alloc_levelint
 *
 * Input Parameters:
 *   periphid       - The peripheral number from irq.h to be assigned to
 *                    a CPU interrupt.
 *   cpuint         - The CPU interrupt to receive the peripheral interrupt
 *                    assignment. This value is returned by
 *                    esp32s2_alloc_edgeint or esp32s2_alloc_levelint.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s2_attach_peripheral(int periphid, int cpuint)
{
  uintptr_t regaddr;

  /* Get the map for CPU interrupts and IRQs */

  uint8_t *intmap = g_cpu0_intmap;

  DEBUGASSERT(periphid >= 0 && periphid < ESP32S2_NPERIPHERALS);
  DEBUGASSERT(cpuint >= 0 && cpuint <= ESP32S2_CPUINT_MAX);
  DEBUGASSERT(intmap[cpuint] == CPUINT_UNASSIGNED);

  /* Get the INTERRUPT_PRO_X_MAP_REG for that specific peripheral.
   * X stands for the peripheral source.
   * Fill the interruption map with the IRQ for the new CPU interrupt.
   * Allocate one peripheral interrupt to the CPU interrupt.
   */

  regaddr = INTERRUPT_PRO_X_MAP_REG(periphid);
  intmap[cpuint] = periphid + XTENSA_IRQ_FIRSTPERI;
  putreg32(cpuint, regaddr);
}

/****************************************************************************
 * Name:  esp32s2_detach_peripheral
 *
 * Description:
 *   Detach a peripheral interrupt from a CPU interrupt.
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

void esp32s2_detach_peripheral(int periphid, int cpuint)
{
  uintptr_t regaddr;
  uint8_t *intmap = g_cpu0_intmap;

  DEBUGASSERT(periphid >= 0 && periphid < ESP32S2_NPERIPHERALS);
  DEBUGASSERT(intmap[cpuint] != CPUINT_UNASSIGNED);

  /* Get the INTERRUPT_PRO_X_MAP_REG for that specific peripheral.
   * X stands for the peripheral source.
   * Unassign the IRQ from the CPU interrupt.
   * Deallocate the peripheral interrupt from the CPU interrupt.
   */

  regaddr = INTERRUPT_PRO_X_MAP_REG(periphid);
  intmap[cpuint] = CPUINT_UNASSIGNED;
  putreg32(NO_CPUINT, regaddr);
}
