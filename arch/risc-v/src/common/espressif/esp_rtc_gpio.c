/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_rtc_gpio.c
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
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "irq.h"
#include "riscv_internal.h"
#include "esp_irq.h"
#include "esp_rtc_gpio.h"
#include "soc/rtc_io_periph.h"
#include "hal/rtc_io_hal.h"
#include "soc/rtc_cntl_periph.h"
#include "soc/periph_defs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_ESP32C3_GENERIC
#ifdef CONFIG_ESPRESSIF_RTCIO_IRQ
static int g_rtcio_cpuint;
static uint32_t last_status;

#ifdef CONFIG_ARCH_CHIP_ESP32C3_GENERIC
static const int rtc_irq_reg_shift[ESP_NIRQ_RTCIO] =
{
  RTC_CNTL_SLP_WAKEUP_INT_ENA_S,
  RTC_CNTL_SLP_REJECT_INT_ENA_S,
  RTC_CNTL_WDT_INT_ENA_S,
  RTC_CNTL_BROWN_OUT_INT_ENA_S,
  RTC_CNTL_MAIN_TIMER_INT_ENA_S,
  RTC_CNTL_SWD_INT_ENA_S,
  RTC_CNTL_XTAL32K_DEAD_INT_ENA_S,
  RTC_CNTL_GLITCH_DET_INT_ENA_S,
  RTC_CNTL_BBPLL_CAL_INT_ENA_S
};
#define RTC_IRQ_REG_SHIFT(x)        rtc_irq_reg_shift[x]
#endif /* CONFIG_ARCH_CHIP_ESP32C3 */
#endif /* CONFIG_ESPRESSIF_RTCIO_IRQ */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtcio_dispatch
 *
 * Description:
 *   Second level dispatch for the RTC interrupt.
 *
 * Input Parameters:
 *   irq        - The IRQ number;
 *   reg_status - Pointer to a copy of the interrupt status register.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_RTCIO_IRQ
static void rtcio_dispatch(int irq, uint32_t *reg_status)
{
  uint32_t status = *reg_status;
  uint32_t mask;
  int i;

  /* Check each bit in the status register */

  for (i = 0; i < ESP_NIRQ_RTCIO && status != 0; i++)
    {
      /* Check if there is an interrupt pending for this type */

      mask = (UINT32_C(1) << RTC_IRQ_REG_SHIFT(i));
      if ((status & mask) != 0)
        {
          /* Yes... perform the second level dispatch. The IRQ context will
           * contain the contents of the status register.
           */

          irq_dispatch(irq + i, (void *)reg_status);

          /* Clear the bit in the status so that we might execute this loop
           * sooner.
           */

          status &= ~mask;
        }
    }
}

/****************************************************************************
 * Name: rtcio_interrupt
 *
 * Description:
 *   RTC interrupt handler.
 *
 * Input Parameters:
 *   irq     - The IRQ number;
 *   context - The interrupt context;
 *   args    - The arguments passed to the handler.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

static int rtcio_interrupt(int irq, void *context, void *arg)
{
  /* Read and clear the lower RTC interrupt status */

  last_status = getreg32(RTC_CNTL_INT_ST_REG);
  putreg32(last_status, RTC_CNTL_INT_CLR_REG);

  /* Dispatch pending interrupts in the RTC status register */

  rtcio_dispatch(ESP_FIRST_RTCIOIRQ, &last_status);

  return OK;
}
#endif /* CONFIG_ESPRESSIF_RTCIO_IRQ */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_rtcioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   RTC IRQs.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_RTCIO_IRQ
void esp_rtcioirqinitialize(void)
{
  /* Setup the RTCIO interrupt. */

  g_rtcio_cpuint = esp_setup_irq(ETS_RTC_CORE_INTR_SOURCE,
                                 1, ESP_IRQ_TRIGGER_LEVEL);
  DEBUGASSERT(g_rtcio_cpuint >= 0);

  /* Attach and enable the interrupt handler */

  DEBUGVERIFY(irq_attach(ESP_IRQ_RTC_CORE, rtcio_interrupt, NULL));
  up_enable_irq(ESP_IRQ_RTC_CORE);
}

/****************************************************************************
 * Name: esp_rtcioirqenable
 *
 * Description:
 *   Enable the interrupt for the specified RTC peripheral IRQ.
 *
 * Input Parameters:
 *   irq - The IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_rtcioirqenable(int irq)
{
  uintptr_t regaddr = RTC_CNTL_INT_ENA_REG;
  uint32_t regval;
  int bit;

  DEBUGASSERT(irq >= ESP_FIRST_RTCIOIRQ &&
              irq <= ESP_LAST_RTCIOIRQ);

  /* Convert the IRQ number to the corresponding bit */

  bit = RTC_IRQ_REG_SHIFT(irq - ESP_FIRST_RTCIOIRQ);

  /* Get the address of the GPIO PIN register for this pin */

  up_disable_irq(ESP_IRQ_RTC_CORE);

  regval = getreg32(regaddr) | (UINT32_C(1) << bit);
  putreg32(regval, regaddr);

  up_enable_irq(ESP_IRQ_RTC_CORE);
}

/****************************************************************************
 * Name: esp_rtcioirqdisable
 *
 * Description:
 *   Disable the interrupt for the specified RTC peripheral IRQ.
 *
 * Input Parameters:
 *   irq - The IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_rtcioirqdisable(int irq)
{
  uintptr_t regaddr = RTC_CNTL_INT_ENA_REG;
  uint32_t regval;
  int bit;

  DEBUGASSERT(irq >= ESP_FIRST_RTCIOIRQ &&
              irq <= ESP_LAST_RTCIOIRQ);

  /* Convert the IRQ number to the corresponding bit */

  bit = RTC_IRQ_REG_SHIFT(irq - ESP_FIRST_RTCIOIRQ);

  /* Disable IRQ */

  up_disable_irq(ESP_IRQ_RTC_CORE);

  regval = getreg32(regaddr) & (~(UINT32_C(1) << bit));
  putreg32(regval, regaddr);

  up_enable_irq(ESP_IRQ_RTC_CORE);
}
#endif /* CONFIG_ESPRESSIF_RTCIO_IRQ */
#endif /* CONFIG_ARCH_CHIP_ESP32C3_GENERIC */
