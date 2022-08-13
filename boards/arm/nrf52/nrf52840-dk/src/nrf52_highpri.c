/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_highpri.c
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

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include <arch/irq.h>
#include <arch/armv7-m/nvicpri.h>

#include "arm_internal.h"
#include "ram_vectors.h"

#include "nrf52_tim.h"

#include <arch/board/board.h>

#ifdef CONFIG_NRF52840DK_HIGHPRI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
#  error CONFIG_ARCH_HIPRI_INTERRUPT is required
#endif

#ifndef CONFIG_ARCH_RAMVECTORS
#  error CONFIG_ARCH_RAMVECTORS is required
#endif

#ifndef CONFIG_ARCH_IRQPRIO
#  error CONFIG_ARCH_IRQPRIO is required
#endif

#ifndef CONFIG_NRF52_TIMER0
#  error CONFIG_NRF52_TIMER0 is required
#endif

/* Timer configuration */

#define NRF52_HIGHPRI_TIMER     (0)
#define NRF52_HIGHPRI_TIMER_IRQ (NRF52_IRQ_TIMER0)
#define NRF52_TIMER_PRESCALER   (NRF52_TIM_PRE_16000000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct highpri_s
{
  struct nrf52_tim_dev_s *dev;
  volatile uint64_t       basepri[16];
  volatile uint64_t       handler;
  volatile uint64_t       thread;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct highpri_s g_highpri;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_handler
 *
 * Description:
 *   This is the handler for the high speed TIMER0 interrupt.
 *
 ****************************************************************************/

void timer_handler(void)
{
  uint8_t basepri;
  int     index;
  int     ret;

  /* Verify interrupt source */

  ret = NRF52_TIM_CHECKINT(g_highpri.dev, NRF52_TIM_CC0);
  if (ret != 1)
    {
      DEBUGPANIC();
    }

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

  /* Clear event */

  NRF52_TIM_ACKINT(g_highpri.dev, NRF52_TIM_CC0);
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
  struct nrf52_tim_dev_s *tim = NULL;
  uint64_t basepri[16];
  uint64_t handler;
  uint64_t thread;
  uint64_t total;
  uint32_t seconds;
  int      ret;
  int      i;

  /* Initialzie TIMER */

  tim = nrf52_tim_init(NRF52_HIGHPRI_TIMER);
  if (tim == NULL)
    {
      printf("ERROR: failed to initialize TIMER%d instance\n",
             NRF52_HIGHPRI_TIMER);
      ret = EXIT_FAILURE;
      goto errout;
    }

  g_highpri.dev = tim;

  /* Configure TIMER mode and width */

  ret = NRF52_TIM_CONFIGURE(tim, NRF52_TIM_MODE_TIMER, NRF52_TIM_WIDTH_16B);
  if (ret < 0)
    {
      printf("ERROR: failed to configure timer %d\n", ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  /* Configure TIMER prescaler */

  ret = NRF52_TIM_SETPRE(tim, NRF52_TIMER_PRESCALER);
  if (ret < 0)
    {
      printf("ERROR: failed to set timer prescaler %d\n", ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  /* Set TIMER CC0 */

  ret = NRF52_TIM_SETCC(tim, NRF52_TIM_CC0, 0x01);
  if (ret < 0)
    {
      printf("ERROR: failed to set TIMER CC %d\n", ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  /* Enable IRQ for TIMER CC0 */

  ret = NRF52_TIM_ENABLEINT(tim, NRF52_TIM_INT_COMPARE0);
  if (ret < 0)
    {
      printf("ERROR: failed to enable TIMER0 CC IRQ %d\n", ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  /* Attach TIMER ram vector */

  ret = arm_ramvec_attach(NRF52_HIGHPRI_TIMER_IRQ, timer_handler);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: arm_ramvec_attach failed: %d\n",
              ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  /* Set the priority of the TIM6 interrupt vector */

  ret = up_prioritize_irq(NRF52_HIGHPRI_TIMER_IRQ, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      fprintf(stderr, "highpri_main: ERROR: up_prioritize_irq failed: %d\n",
              ret);
      ret = EXIT_FAILURE;
      goto errout;
    }

  /* Enable the timer interrupt at the NVIC and at TIMER */

  up_enable_irq(NRF52_HIGHPRI_TIMER_IRQ);
  NRF52_TIM_START(tim);

  seconds = 0;

  while (1)
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

      printf("Elapsed time: %" PRId32 " seconds\n\n", seconds);
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

  ret = EXIT_SUCCESS;

errout:
  return ret;
}

#endif /* CONFIG_NRF52840DK_HIGHPRI */
