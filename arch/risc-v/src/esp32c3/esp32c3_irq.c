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

#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include <arch/irq.h>
#include <arch/rv32im/mcause.h>

#include "riscv_internal.h"
#include "hardware/esp32c3_interrupt.h"

#include "esp32c3.h"
#include "esp32c3_attr.h"
#include "esp32c3_gpio.h"

#include "esp32c3_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C3_DEFAULT_INT_THRESHOLD   1

/* No peripheral assigned to this CPU interrupt */

#define CPUINT_UNASSIGNED 0xff

/* Wi-Fi reserved CPU interrupt bit */

#ifdef CONFIG_ESP32C3_WIRELESS
#  define CPUINT_WMAC_MAP (1 << ESP32C3_CPUINT_WMAC)
#else
#  define CPUINT_WMAC_MAP 0
#endif

/* Reserved CPU interrupt bits */

#define CPUINT_RESERVED_MAPS (CPUINT_WMAC_MAP)

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *g_current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

uint8_t g_cpuint_map[ESP32C3_CPUINT_MAX];

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

  memset(g_cpuint_map, CPUINT_UNASSIGNED, ESP32C3_CPUINT_MAX);

  /**
   * Initialize specific driver's CPU interrupt ID:
   *   Object  |  CPU INT  |  Pheripheral
   *           |           |
   *    Wi-Fi  |     1     |      1
   */

#ifdef CONFIG_ESP32C3_WIRELESS
  g_cpuint_map[ESP32C3_CPUINT_WMAC] = ESP32C3_PERIPH_WIFI_MAC_NMI;
#endif

  /* Clear all peripheral interrupts from "bootloader" */

  for (periphid = 0; periphid < ESP32C3_NPERIPHERALS; periphid++)
    {
      putreg32(0, DR_REG_INTERRUPT_BASE + periphid * 4);
    }

  /* Set CPU interrupt threshold level */

  putreg32(ESP32C3_DEFAULT_INT_THRESHOLD, INTERRUPT_CPU_INT_THRESH_REG);

  /* Attach the ECALL interrupt. */

  irq_attach(ESP32C3_IRQ_ECALL_M, riscv_swint, NULL);

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
 * Name: riscv_get_newintctx
 *
 * Description:
 *   Return initial mstatus when a task is created.
 *
 ****************************************************************************/

uint32_t riscv_get_newintctx(void)
{
  /* Set machine previous privilege mode to machine mode.
   * Also set machine previous interrupt enable
   */

  return (MSTATUS_MPPM | MSTATUS_MPIE);
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
  irqstate_t irqstate;

  DEBUGASSERT(cpuint >= ESP32C3_CPUINT_MIN && cpuint <= ESP32C3_CPUINT_MAX);

  irqstate = enter_critical_section();
  setbits(1 << cpuint, INTERRUPT_CPU_INT_ENABLE_REG);
  leave_critical_section(irqstate);
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
  irqstate_t irqstate;

  DEBUGASSERT(cpuint >= ESP32C3_CPUINT_MIN && cpuint <= ESP32C3_CPUINT_MAX);

  irqstate = enter_critical_section();
  resetbits(1 << cpuint, INTERRUPT_CPU_INT_ENABLE_REG);
  leave_critical_section(irqstate);
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
 * Name: esp32c3_request_irq
 *
 * Description:
 *   Request IRQ and resource with given parameters.
 *
 * Input Parameters:
 *   periphid  - Peripheral ID
 *   prio      - Interrupt priority
 *   flags     - Interrupt flags
 *
 * Returned Value:
 *   Allocated CPU interrupt on success, a negated error on failure.
 *
 ****************************************************************************/

int esp32c3_request_irq(uint8_t periphid, uint8_t prio, uint32_t flags)
{
  int ret;
  uint32_t regval;
  int cpuint;
  irqstate_t irqstate;

  DEBUGASSERT(periphid < ESP32C3_NPERIPHERALS);
  DEBUGASSERT((prio >= ESP32C3_INT_PRIO_MIN) &&
              (prio <= ESP32C3_INT_PRIO_MAX));

  irqstate = enter_critical_section();

  /* Skip over enabled interrupts.  NOTE: bit 0 is reserved. */

  regval = getreg32(INTERRUPT_CPU_INT_ENABLE_REG);

  /* Skip over reserved CPU interrupts */

  regval |= CPUINT_RESERVED_MAPS;

  for (cpuint = 1; cpuint <= ESP32C3_CPUINT_MAX; cpuint++)
    {
      if (!(regval & (1 << cpuint)))
        {
          break;
        }
    }

  irqinfo("INFO: cpuint=%d\n", cpuint);

  if (cpuint <= ESP32C3_CPUINT_MAX)
    {
      DEBUGASSERT(g_cpuint_map[cpuint] == CPUINT_UNASSIGNED);

      /* We have a free CPU interrupt.  We can continue with mapping the
       * peripheral.
       */

      /* Save the CPU interrupt ID.  We will return this value. */

      ret = cpuint;

      /* Update our CPU interrupt to Peripheral map. */

      g_cpuint_map[cpuint] = periphid;

      /* Configure IRQ */

      esp32c3_bind_irq(cpuint, periphid, prio, flags);
    }
  else
    {
      /* We couldn't find a free CPU interrupt. */

      ret = -ENOMEM;
    }

  leave_critical_section(irqstate);

  return ret;
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
  uint32_t cpuint;

  DEBUGASSERT(periphid < ESP32C3_NPERIPHERALS);

  irqstate = enter_critical_section();

  /* Get the CPU interrupt ID mapped to this peripheral. */

  cpuint = getreg32(DR_REG_INTERRUPT_BASE + periphid * 4) & 0x1f;
  irqinfo("INFO: irq[%d]=%08lx\n", periphid, cpuint);

  if (cpuint)
    {
      /* Undo the allocation process:
       *   1.  Unmap the peripheral from the CPU interrupt ID.
       *   2.  Reset the interrupt type.
       *   3.  Reset the interrupt priority.
       *   4.  Clear the CPU interrupt.
       */

      DEBUGASSERT(g_cpuint_map[cpuint] != CPUINT_UNASSIGNED);

      g_cpuint_map[cpuint] = CPUINT_UNASSIGNED;
      putreg32(0, DR_REG_INTERRUPT_BASE + periphid * 4);
      resetbits(1 << cpuint, INTERRUPT_CPU_INT_TYPE_REG);
      putreg32(0, INTERRUPT_CPU_INT_PRI_0_REG + cpuint * 4);
      resetbits(1 << cpuint, INTERRUPT_CPU_INT_ENABLE_REG);
    }

  leave_critical_section(irqstate);
}

/****************************************************************************
 * Name: esp32c3_dispatch_irq
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

IRAM_ATTR uint32_t *esp32c3_dispatch_irq(uint32_t mcause, uint32_t *regs)
{
  int cpuint;
  int irq;

  DEBUGASSERT(g_current_regs == NULL);
  g_current_regs = regs;

  irqinfo("INFO: mcause=%08lx\n", mcause);

  /* If the board supports LEDs, turn on an LED now to indicate that we are
   * processing an interrupt.
   */

  board_autoled_on(LED_INIRQ);

  if (MCAUSE_INTERRUPT & mcause)
    {
      cpuint = mcause & MCAUSE_INTERRUPT_MASK;

      DEBUGASSERT(cpuint <= ESP32C3_CPUINT_MAX);

      irqinfo("INFO: cpuint=%d\n", cpuint);

      /* Clear edge interrupts. */

      putreg32(1 << cpuint, INTERRUPT_CPU_INT_CLEAR_REG);

      irq = g_cpuint_map[cpuint] + ESP32C3_IRQ_FIRSTPERIPH;
      irq_dispatch(irq, regs);

      /* Toggle the bit back to zero. */

      resetbits(1 << cpuint, INTERRUPT_CPU_INT_CLEAR_REG);
    }
  else
    {
      if (mcause == MCAUSE_ECALL_M)
        {
          irq_dispatch(ESP32C3_IRQ_ECALL_M, regs);
        }
      else
        {
          riscv_exception(mcause, regs);
        }
    }

  regs = (uint32_t *)g_current_regs;
  g_current_regs = NULL;

  board_autoled_off(LED_INIRQ);

  return regs;
}
