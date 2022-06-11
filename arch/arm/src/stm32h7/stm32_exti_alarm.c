/****************************************************************************
 * arch/arm/src/stm32h7/stm32_exti_alarm.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "stm32_exti.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to the ALARM EXTI */

static xcpt_t g_alarm_callback;
static void  *g_callback_arg;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_exti_alarm_isr
 *
 * Description:
 *   EXTI ALARM interrupt service routine/dispatcher
 *
 ****************************************************************************/

static int stm32_exti_alarm_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending EXTI interrupt */

  putreg32(EXTI_RTC_ALARM, STM32_EXTI_CPUPR1);

  /* And dispatch the interrupt to the handler */

  if (g_alarm_callback != NULL)
    {
      ret = g_alarm_callback(irq, context, g_callback_arg);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_exti_alarm
 *
 * Description:
 *   Sets/clears EXTI alarm interrupt.
 *
 * Input Parameters:
 *  - rising/falling edge: enables interrupt on rising/falling edge
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *  - arg:    Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int stm32_exti_alarm(bool risingedge, bool fallingedge, bool event,
                     xcpt_t func, void *arg)
{
  g_alarm_callback = func;
  g_callback_arg   = arg;

  /* Install external interrupt handlers (if not already attached) */

  if (func)
    {
      irq_attach(STM32_IRQ_RTCALARM, stm32_exti_alarm_isr, NULL);
      up_enable_irq(STM32_IRQ_RTCALARM);
    }
  else
    {
      up_disable_irq(STM32_IRQ_RTCALARM);
    }

  /* Configure rising/falling edges */

  modifyreg32(STM32_EXTI_RTSR1,
              risingedge ? 0 : EXTI_RTC_ALARM,
              risingedge ? EXTI_RTC_ALARM : 0);
  modifyreg32(STM32_EXTI_FTSR1,
              fallingedge ? 0 : EXTI_RTC_ALARM,
              fallingedge ? EXTI_RTC_ALARM : 0);

  /* Enable Events and Interrupts */

  modifyreg32(STM32_EXTI_CPUEMR1,
              event ? 0 : EXTI_RTC_ALARM,
              event ? EXTI_RTC_ALARM : 0);
  modifyreg32(STM32_EXTI_CPUIMR1,
              func ? 0 : EXTI_RTC_ALARM,
              func ? EXTI_RTC_ALARM : 0);

  return OK;
}
