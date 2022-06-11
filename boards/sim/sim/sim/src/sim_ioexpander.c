/****************************************************************************
 * boards/sim/sim/sim/src/sim_ioexpander.c
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

#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/ioe_dummy.h>

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

  struct ioexpander_dev_s *ioe = ioe_dummy_initialize();
  if (ioe == NULL)
    {
      gpioerr("ERROR: ioe_dummy_initialize failed\n");
      return -ENOMEM;
    }

  /* Register four pin drivers */

  /* Pin 0: an non-inverted, input pin */

  IOEXP_SETDIRECTION(ioe, 0, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 0, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 0, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_DISABLE);
  gpio_lower_half(ioe, 0, GPIO_INPUT_PIN, 0);

  /* Pin 1: an non-inverted, output pin */

  IOEXP_SETDIRECTION(ioe, 1, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 1, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 2, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_DISABLE);
  gpio_lower_half(ioe, 1, GPIO_OUTPUT_PIN, 1);

  /* Pin 2: an non-inverted, edge interrupting pin */

  IOEXP_SETDIRECTION(ioe, 2, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 2, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 2, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_BOTH);
  gpio_lower_half(ioe, 2, GPIO_INTERRUPT_PIN, 2);

  /* Pin 3: a non-inverted, level interrupting pin */

  IOEXP_SETDIRECTION(ioe, 3, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_HIGH);
  gpio_lower_half(ioe, 3, GPIO_INTERRUPT_PIN, 3);

  return 0;
}
#endif /* CONFIG_EXAMPLES_GPIO && CONFIG_GPIO_LOWER_HALF */
