/****************************************************************************
 * arch/avr/src/avrdx/avrdx_swint.c
 * AVR DA/DB software interrupt emulation
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
#include <stdint.h>
#include <nuttx/irq.h>
#include <assert.h>
#include <avr/io.h>

#include "avrdx.h"
#include "avrdx_swint.h"
#include "avrdx_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Verify configuration */

#ifdef CONFIG_AVRDX_SWINT_PORT_IDX
#  if ((CONFIG_AVRDX_SWINT_PORT_IDX < 0) || (CONFIG_AVRDX_SWINT_PORT_IDX > 6))
#    error Port idx must be within range of 0-6
#  endif
#else
#  error Port idx must be set
#endif
#ifdef CONFIG_AVRDX_SWINT_PIN_NUM
#  if ((CONFIG_AVRDX_SWINT_PIN_NUM < 0) || (CONFIG_AVRDX_SWINT_PIN_NUM > 7))
#    error Port idx must be within range of 0-6
#  endif
#else
#  error Port idx must be set
#endif

/* Convert pin number to its corresponding bitmask */

#define AVRDX_SWINT_PIN_BITMASK (1 << CONFIG_AVRDX_SWINT_PIN_NUM)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Struct that holds a handler that is to be executed by
 * the software interrupt
 */

struct avrdx_swint_isr_s
{
  xcpt_t handler; /* handler given by the user */
  FAR void *arg;  /* arg to be given to the handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This variable is a bitfield. For every software interrupt vector raised,
 * the corresponding bit is set to 1.
 */

static avrdx_swint_size_t g_swint_flags;

/* Handlers for each software interrupt vector. If set to NULL, corresponding
 * software interrupt vector is not assigned.
 */

struct avrdx_swint_isr_s g_swint_handlers[CONFIG_AVRDX_SWINT_MAX_VECTORS];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_swint_isr
 *
 * Description:
 *   ISR for software interrupt. Triggered by the interrupt source
 *   which pulls an output pin low. Reads raised software interrupt vectors
 *   and runs their handlers.
 *
 * Input Parameters:
 *   Arg - ignored
 *
 * Returned Value: void
 *
 * Assumptions/Limitations:
 *   Running in interrupt context
 *
 ****************************************************************************/

static int avrdx_swint_isr(int irq, void *context, void *arg)
{
  avrdx_swint_size_t raised_vectors;
  avrdx_swint_size_t j;
  irqstate_t irqflags;
  uint8_t i;

  /* In theory, interrupt handler may be interrupted by handler of higher
   * priority. Not supported directly in NuttX yet but let's be future-proof.
   * The higher priority interrupt may raise a software interrupt while
   * we're reading the flags.
   *
   * Disable interrupts, read flags and put the triggering pin back
   * to logical 1.
   */

  irqflags = enter_critical_section();

  raised_vectors = g_swint_flags;
  g_swint_flags = 0;
  AVRDX_PORT(CONFIG_AVRDX_SWINT_PORT_IDX).OUTSET = AVRDX_SWINT_PIN_BITMASK;

#ifndef CONFIG_AVRDX_SWINT_GPIO_MUX
  /* GPIO multiplexer takes care of resetting port's interrupt flags
   * register when in use. It is not, we need to care of that here.
   */

  AVRDX_PORT(CONFIG_AVRDX_SWINT_PORT_IDX).INTFLAGS = AVRDX_SWINT_PIN_BITMASK;
#endif

  leave_critical_section(irqflags);

  for (i = 0, j = 1; i < CONFIG_AVRDX_SWINT_MAX_VECTORS; i++, j <<= 1)
    {
      if ((raised_vectors & j) == 0)
        {
          continue;
        }

      if (g_swint_handlers[i].handler == NULL)
        {
          PANIC();
        }

      g_swint_handlers[i].handler(irq, context, g_swint_handlers[i].arg);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_swint_allocate
 *
 * Description:
 *   Allocates software interrupt vector for the caller and returns it,
 *   if any vector is available
 *
 * Input Parameters:
 *   Handler - to be executed in interrupt context when triggered
 *   Opaque arg argument - passed to the handler
 *
 * Returned Value:
 *   Software interrupt vector number to be used in avrdx_swint_raise
 *   or zero if no vectors are available. The number is single bit mask
 *   so it can be quickly OR'ed with others. Treat it as opaque value though.
 *
 ****************************************************************************/

avrdx_swint_size_t avrdx_swint_allocate(xcpt_t handler, void *arg)
{
  uint8_t i;
  irqstate_t irqflags;
  avrdx_swint_size_t ret;

  /* Unlikely to happen but two tasks may be doing this
   * at the same time.
   */

  irqflags = enter_critical_section();

  for (i = 0, ret = 1; i < CONFIG_AVRDX_SWINT_MAX_VECTORS; i++, ret <<= 1)
    {
      if (g_swint_handlers[i].handler == NULL)
        {
          break;
        }
    }

  if (i == CONFIG_AVRDX_SWINT_MAX_VECTORS)
    {
      return 0;
    }

  g_swint_handlers[i].handler = handler;
  g_swint_handlers[i].arg = arg;

  leave_critical_section(irqflags);

  return ret;
}

/****************************************************************************
 * Name: avrdx_swint_raise
 *
 * Description:
 *   Raises software interrupt with given vector number. Note that this
 *   function is designed to be runnable from code that runs outside
 *   of NuttX - this mainly means so called zero latency interrupts.
 *
 * Input Parameters:
 *   vector - number that identifies which software interrupt was raised
 *   (use return value from avrdx_swint_allocate.)
 *
 * Returned Value: void
 *
 ****************************************************************************/

void avrdx_swint_raise(avrdx_swint_size_t vector)
{
  uint8_t sreg;

  /* Need to disable interrupts but cannot use any function provided
   * by the operating system - enter_critical_section cannot be used
   * (we can never be sure about what it does under the hood.)
   */

  sreg = SREG;
  asm volatile("cli");

  g_swint_flags |= vector;
  AVRDX_PORT(CONFIG_AVRDX_SWINT_PORT_IDX).OUTCLR = AVRDX_SWINT_PIN_BITMASK;

  SREG = sreg;
}

/****************************************************************************
 * Name: avrdx_swint_initialize
 *
 * Description:
 *   Initialize hardware related to software interrupts.
 *
 ****************************************************************************/

int avrdx_swint_initialize(void)
{
  int ret = 0;
#ifndef CONFIG_AVRDX_SWINT_GPIO_MUX
  uint8_t vector_num;
  irqstate_t irqflags;
#endif

  /* First configure the pin as output and set it to logical one */

  AVRDX_PORT(CONFIG_AVRDX_SWINT_PORT_IDX).DIRSET = AVRDX_SWINT_PIN_BITMASK;
  AVRDX_PORT(CONFIG_AVRDX_SWINT_PORT_IDX).OUTSET = AVRDX_SWINT_PIN_BITMASK;

  /* Now configure the interrupt handler */

#ifdef CONFIG_AVRDX_SWINT_GPIO_MUX
  ret = avrdx_irq_attach_gpio_mux(
          CONFIG_AVRDX_SWINT_PORT_IDX,
          AVRDX_SWINT_PIN_BITMASK,
          avrdx_swint_isr,
          NULL,
          PORT_ISC_LEVEL_GC);

  if (ret != OK)
    {
      PANIC();
    }
#else
  vector_num = avrdx_gpio_irq_vectors[CONFIG_AVRDX_SWINT_PORT_IDX];

  /* Disabling interrupts here to be safe. This should only run on startup
   * but let's not risk that some event already happens during that time
   * that would interfere with PINCONFIG register
   */

  irqflags = enter_critical_section();
  AVRDX_PORT(CONFIG_AVRDX_SWINT_PORT_IDX).PINCONFIG = PORT_ISC_LEVEL_GC;
  AVRDX_PORT(CONFIG_AVRDX_SWINT_PORT_IDX).PINCTRLSET = \
      AVRDX_SWINT_PIN_BITMASK;
  leave_critical_section(irqflags);

  irq_attach(vector_num, avrdx_swint_isr, NULL);
#endif

  return ret;
}
