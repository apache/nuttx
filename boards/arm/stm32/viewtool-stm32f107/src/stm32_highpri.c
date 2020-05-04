/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/src/stm32_highpri.c
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include <arch/irq.h>
#include <arch/armv7-m/nvicpri.h>

#include "arm_internal.h"
#include "ram_vectors.h"
#include "stm32_tim.h"

#include "viewtool_stm32f107.h"

#include <arch/board/board.h>

#ifdef CONFIG_VIEWTOOL_HIGHPRI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_CHIP_STM32F103VC
#  warning This only only been verified with CONFIG_ARCH_CHIP_STM32F103VC
#endif

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
#  error CONFIG_ARCH_HIPRI_INTERRUPT is required
#endif

#ifndef CONFIG_ARCH_RAMVECTORS
#  error CONFIG_ARCH_RAMVECTORS is required
#endif

#ifndef CONFIG_STM32_TIM6
#  error CONFIG_STM32_TIM6 is required
#endif

#ifndef CONFIG_VIEWTOOL_TIM6_FREQUENCY
#  warning CONFIG_VIEWTOOL_TIM6_FREQUENCY defaulting to STM32_APB1_TIM6_CLKIN
#  define CONFIG_VIEWTOOL_TIM6_FREQUENCY STM32_APB1_TIM6_CLKIN
#endif

#ifndef CONFIG_VIEWTOOL_TIM6_PERIOD
#  warning CONFIG_VIEWTOOL_TIM6_PERIOD defaulting to 1MS
#  define CONFIG_VIEWTOOL_TIM6_PERIOD (CONFIG_VIEWTOOL_TIM6_FREQUENCY / 1000)
#endif

#ifndef CONFIG_ARCH_IRQPRIO
#  error CONFIG_ARCH_IRQPRIO is required
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct highpri_s
{
  FAR struct stm32_tim_dev_s *dev;  /* TIM6 driver instance */
  volatile uint64_t basepri[16];
  volatile uint64_t handler;
  volatile uint64_t thread;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct highpri_s g_highpri;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tim6_handler
 *
 * Description:
 *   This is the handler for the high speed TIM6 interrupt.
 *
 ****************************************************************************/

void tim6_handler(void)
{
  uint8_t basepri;
  int index;

  /* Acknowledge the timer interrupt */

  STM32_TIM_ACKINT(g_highpri.dev, ATIM_SR_UIF);

  /* Increment the count associated with the current basepri */

  basepri = getbasepri();
  index   = ((basepri >> 4) & 15);
  g_highpri.basepri[index]++;

  /* Check if we are in an interrupt handle */

  if (up_interrupt_context())
    {
      g_highpri.handler++;
    }
  else
    {
      g_highpri.thread++;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: highpri_main
 *
 * Description:
 *   Main entry point in into the high priority interrupt test.
 *
 ****************************************************************************/

int highpri_main(int argc, char *argv[])
{
  FAR struct stm32_tim_dev_s *dev;
  uint64_t basepri[16];
  uint64_t handler;
  uint64_t thread;
  uint64_t total;
  uint32_t seconds;
  int prescaler;
  int ret;
  int i;

  printf("highpri_main: Started\n");

  /* Configure basic timer TIM6 and enable interrupts */

  dev = stm32_tim_init(6);
  if (!dev)
    {
      fprintf(stderr, "highpri_main: ERROR: stm32_tim_init(6) failed\n");
      return EXIT_FAILURE;
    }

  g_highpri.dev = dev;

  prescaler = STM32_TIM_SETCLOCK(dev, CONFIG_VIEWTOOL_TIM6_FREQUENCY);
  printf("TIM6 CLKIN=%d Hz, Frequency=%d Hz, prescaler=%d\n",
         STM32_APB1_TIM6_CLKIN, CONFIG_VIEWTOOL_TIM6_FREQUENCY, prescaler);

  STM32_TIM_SETPERIOD(dev, CONFIG_VIEWTOOL_TIM6_PERIOD);
  printf("TIM6 period=%d cyles; interrupt rate=%d Hz\n",
         CONFIG_VIEWTOOL_TIM6_PERIOD,
         CONFIG_VIEWTOOL_TIM6_FREQUENCY / CONFIG_VIEWTOOL_TIM6_PERIOD);

  /* Attach TIM6 ram vector */

  ret = arm_ramvec_attach(STM32_IRQ_TIM6, tim6_handler);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: arm_ramvec_attach failed: %d\n",
              ret);
      return EXIT_FAILURE;
    }

  /* Set the priority of the TIM6 interrupt vector */

  ret = up_prioritize_irq(STM32_IRQ_TIM6, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: up_prioritize_irq failed: %d\n",
              ret);
      return EXIT_FAILURE;
    }

  /* Enable the timer interrupt at the NVIC and at TIM6 */

  up_enable_irq(STM32_IRQ_TIM6);
  STM32_TIM_ENABLEINT(dev, ATIM_DIER_UIE);

  /* Monitor interrupts */

  seconds = 0;
  for (; ; )
    {
      /* Flush stdout and wait a bit */

      fflush(stdout);
      nxsig_sleep(1);
      seconds++;

      /* Sample counts so that they are not volatile.  Missing a count now
       * and then is a normal consequence of this design.
       */

      for (i = 0; i < 16; i++)
        {
          basepri[i] = g_highpri.basepri[i];
        }

      handler = g_highpri.handler;
      thread  = g_highpri.thread;

      /* Then print out what is happening */

      printf("Elapsed time: %d seconds\n\n", seconds);
      for (i = 0, total = 0; i < 16; i++)
        {
          total += basepri[i];
        }

      if (total > 0)
        {
          for (i = 0; i < 16; i++)
            {
              if (basepri[i] > 0)
                {
                  printf("  basepri[%02x]: %lld (%d%%)\n",
                         i << 4, basepri[i],
                         (int)((100 * basepri[i] + (total / 2)) / total));
                }
            }
        }

      total = handler + thread;
      if (total > 0)
        {
          printf("  Handler:     %lld (%d%%)\n",
                 handler,  (int)((100*handler + (total / 2)) / total));
          printf("  Thread:      %lld (%d%%)\n\n",
                 thread,   (int)((100*thread + (total / 2)) / total));
        }
    }

  return EXIT_SUCCESS;
}

#endif /* CONFIG_VIEWTOOL_HIGHPRI */
