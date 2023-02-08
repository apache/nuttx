/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_rtc_gpio.c
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

#include "riscv_internal.h"
#include "esp32c3_irq.h"
#include "esp32c3_rtc_gpio.h"
#include "hardware/esp32c3_rtccntl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_RTCIO_IRQ
static int g_rtcio_cpuint;
static uint32_t last_status;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: is_valid_rtc_gpio
 *
 * Description:
 *   Determine if the specified rtcio_num is a valid RTC GPIO.
 *
 * Input Parameters:
 *   rtcio_num - RTC GPIO to be checked.
 *
 * Returned Value:
 *   True if valid. False otherwise.
 *
 ****************************************************************************/

static inline bool is_valid_rtc_gpio(uint32_t rtcio_num)
{
  return (rtcio_num < RTC_GPIO_NUMBER);
}

/****************************************************************************
 * Name: rtcio_dispatch
 *
 * Description:
 *   Second level dispatch for the RTC interrupt.
 *
 * Input Parameters:
 *   irq - The IRQ number;
 *   reg_status - Pointer to a copy of the interrupt status register.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_RTCIO_IRQ
static void rtcio_dispatch(int irq, uint32_t *reg_status)
{
  uint32_t status = *reg_status;
  uint32_t mask;
  int i;

  /* Check each bit in the status register */

  for (i = 0; i < ESP32C3_NIRQ_RTCIO && status != 0; i++)
    {
      /* Check if there is an interrupt pending for this type */

      mask = (UINT32_C(1) << rtc_irq_reg_shift[i]);
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
#endif

/****************************************************************************
 * Name: rtcio_interrupt
 *
 * Description:
 *   RTC interrupt handler.
 *
 * Input Parameters:
 *   irq - The IRQ number;
 *   context - The interrupt context;
 *   args - The arguments passed to the handler.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_RTCIO_IRQ
static int rtcio_interrupt(int irq, void *context, void *arg)
{
  /* Read and clear the lower RTC interrupt status */

  last_status = getreg32(RTC_CNTL_INT_ST_REG);
  putreg32(last_status, RTC_CNTL_INT_CLR_REG);

  /* Dispatch pending interrupts in the RTC status register */

  rtcio_dispatch(ESP32C3_FIRST_RTCIOIRQ, &last_status);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_rtcioirqinitialize
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

#ifdef CONFIG_ESP32C3_RTCIO_IRQ
void esp32c3_rtcioirqinitialize(void)
{
  /* Setup the RTCIO interrupt. */

  g_rtcio_cpuint = esp32c3_setup_irq(ESP32C3_PERIPH_RTC_CORE,
                                     1, ESP32C3_INT_LEVEL);
  DEBUGASSERT(g_rtcio_cpuint >= 0);

  /* Attach and enable the interrupt handler */

  DEBUGVERIFY(irq_attach(ESP32C3_IRQ_RTC_CORE, rtcio_interrupt, NULL));
  up_enable_irq(ESP32C3_IRQ_RTC_CORE);
}
#endif

/****************************************************************************
 * Name: esp32c3_rtcioirqenable
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

#ifdef CONFIG_ESP32C3_RTCIO_IRQ
void esp32c3_rtcioirqenable(int irq)
{
  uintptr_t regaddr = RTC_CNTL_INT_ENA_REG;
  uint32_t regval;
  int bit;

  DEBUGASSERT(irq >= ESP32C3_FIRST_RTCIOIRQ &&
              irq <= ESP32C3_LAST_RTCIOIRQ);

  /* Convert the IRQ number to the corresponding bit */

  bit = rtc_irq_reg_shift[irq - ESP32C3_FIRST_RTCIOIRQ];

  /* Get the address of the GPIO PIN register for this pin */

  up_disable_irq(ESP32C3_IRQ_RTC_CORE);

  regval = getreg32(regaddr) | (UINT32_C(1) << bit);
  putreg32(regval, regaddr);

  up_enable_irq(ESP32C3_IRQ_RTC_CORE);
}
#endif

/****************************************************************************
 * Name: esp32c3_rtcioirqdisable
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

#ifdef CONFIG_ESP32C3_RTCIO_IRQ
void esp32c3_rtcioirqdisable(int irq)
{
  uintptr_t regaddr = RTC_CNTL_INT_ENA_REG;
  uint32_t regval;
  int bit;

  DEBUGASSERT(irq >= ESP32C3_FIRST_RTCIOIRQ &&
              irq <= ESP32C3_LAST_RTCIOIRQ);

  /* Convert the IRQ number to the corresponding bit */

  bit = rtc_irq_reg_shift[irq - ESP32C3_FIRST_RTCIOIRQ];

  /* Disable IRQ */

  up_disable_irq(ESP32C3_IRQ_RTC_CORE);

  regval = getreg32(regaddr) & (~(UINT32_C(1) << bit));
  putreg32(regval, regaddr);

  up_enable_irq(ESP32C3_IRQ_RTC_CORE);
}
#endif