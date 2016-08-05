/****************************************************************************
 * configs/sim/src/sim_ioexpander.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#include <errno.h>
#include <debug.h>

#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "up_internal.h"
#include "sim.h"

#if defined(CONFIG_EXAMPLES_GPIO) && defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_gpio_initialize
 *
 * Description:
 *   Initialize simulated GPIO expander for use with /apps/examples/gpio
 *
 ****************************************************************************/

int sim_gpio_initialize(void)
{
  /* Get an instance of the simulated I/O expander */

  FAR struct ioexpander_dev_s *ioe = sim_ioexpander_initialize();
  if (ioe == NULL)
    {
      gpioerr("ERROR: sim_ioexpander_initialize failed\n");
      return -ENOMEM;
    }

  /* Register four pin drivers */

  /* Pin 0: an non-inverted, input pin */

  (void)IOEXP_SETDIRECTION(ioe, 0, IOEXPANDER_DIRECTION_IN);
  (void)IOEXP_SETOPTION(ioe, 0, IOEXPANDER_OPTION_INVERT,
                        (FAR void *)IOEXPANDER_VAL_NORMAL);
  (void)IOEXP_SETOPTION(ioe, 0, IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_DISABLE);
  (void)gpio_lower_half(ioe, 0, GPIO_INPUT_PIN, 0);

  /* Pin 1: an non-inverted, output pin */

  (void)IOEXP_SETDIRECTION(ioe, 1, IOEXPANDER_DIRECTION_OUT);
  (void)IOEXP_SETOPTION(ioe, 1, IOEXPANDER_OPTION_INVERT,
                        (FAR void *)IOEXPANDER_VAL_NORMAL);
  (void)IOEXP_SETOPTION(ioe, 2, IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_DISABLE);
  (void)gpio_lower_half(ioe, 1, GPIO_OUTPUT_PIN, 1);

  /* Pin 2: an non-inverted, edge interrupting pin */

  (void)IOEXP_SETDIRECTION(ioe, 2, IOEXPANDER_DIRECTION_IN);
  (void)IOEXP_SETOPTION(ioe, 2, IOEXPANDER_OPTION_INVERT,
                        (FAR void *)IOEXPANDER_VAL_NORMAL);
  (void)IOEXP_SETOPTION(ioe, 2, IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_BOTH);
  (void)gpio_lower_half(ioe, 2, GPIO_INTERRUPT_PIN, 2);

  /* Pin 3: a non-inverted, level interrupting pin */

  (void)IOEXP_SETDIRECTION(ioe, 3, IOEXPANDER_DIRECTION_IN);
  (void)IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INVERT,
                        (FAR void *)IOEXPANDER_VAL_NORMAL);
  (void)IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_HIGH);
  (void)gpio_lower_half(ioe, 3, GPIO_INTERRUPT_PIN, 3);

  return 0;
}
#endif /* CONFIG_EXAMPLES_GPIO && CONFIG_GPIO_LOWER_HALF */
