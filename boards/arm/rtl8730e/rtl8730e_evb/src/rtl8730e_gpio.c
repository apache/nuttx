/****************************************************************************
 * boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_gpio.c
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

#include <sys/param.h>
#include <syslog.h>

#include <nuttx/ioexpander/gpio.h>

#include "ameba_gpio.h"
#include "rtl8730e_evb.h"

#ifdef CONFIG_AMEBA_GPIO

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rtl8730e_gpio_s
{
  uint8_t pin;                  /* AMEBA_PA()/AMEBA_PB() pin encoding */
  enum gpio_pintype_e pintype;  /* Input, output or interrupt */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtl8730e_gpio_s g_gpio_pins[] =
{
  { AMEBA_PB(19), GPIO_OUTPUT_PIN    },  /* /dev/gpio0: output    */
  { AMEBA_PB(20), GPIO_INPUT_PIN     },  /* /dev/gpio1: input     */
  { AMEBA_PB(11), GPIO_INTERRUPT_PIN },  /* /dev/gpio2: interrupt */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8730e_gpio_initialize
 *
 * Description:
 *   Register the board's GPIO pins with the NuttX GPIO upper half.
 *
 ****************************************************************************/

int rtl8730e_gpio_initialize(void)
{
  int ret;
  size_t i;

  /* Initialise the ROM GPIO port-base lookup table.  lib_rom.a places
   * GPIO_PORTx in .sramdram.only.data which NuttX's linker script does not
   * copy, so the array is zero at boot.  Write the three port bases before
   * any ROM GPIO function is called.
   */

  extern void *GPIO_PORTx[3];

  GPIO_PORTx[0] = (void *)0x4200d000u;  /* GPIOA */
  GPIO_PORTx[1] = (void *)0x4200d400u;  /* GPIOB */
  GPIO_PORTx[2] = (void *)0x4200d800u;  /* GPIOC */

  for (i = 0; i < nitems(g_gpio_pins); i++)
    {
      ret = ameba_gpio_register(i, g_gpio_pins[i].pin,
                                g_gpio_pins[i].pintype);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: ameba_gpio_register(/dev/gpio%zu) failed: %d\n",
                 i, ret);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_AMEBA_GPIO */
