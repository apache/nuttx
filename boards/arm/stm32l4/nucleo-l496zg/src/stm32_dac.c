/*****************************************************************************
 * boards/arm/stm32l4/nucleo-l496zg/src/stm32_dac.c
 *
 *   Copyright (C) 2017 Haltian Ltd. All rights reserved.
 *   Authors: Juha Niskanen <juha.niskanen@haltian.com>
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

#include <nuttx/board.h>
#include <nuttx/analog/dac.h>
#include <arch/board/board.h>
#include "stm32l4_gpio.h"
#include "stm32l4_dac.h"
#include "nucleo-144.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32L4_DAC1
static struct dac_dev_s *g_dac1;
#endif

#ifdef CONFIG_STM32L4_DAC2
static struct dac_dev_s *g_dac2;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dac_setup
 ****************************************************************************/

int stm32_dac_setup(void)
{
  static bool initialized = false;

  if (!initialized)
    {
      int ret;

#ifdef CONFIG_STM32L4_DAC1
      g_dac1 = stm32l4_dacinitialize(0);
      if (g_dac1 == NULL)
        {
          aerr("ERROR: Failed to get DAC1 interface\n");
          return -ENODEV;
        }

      ret = dac_register("/dev/dac0", g_dac1);
      if (ret < 0)
        {
          aerr("ERROR: dac_register failed: %d\n", ret);
          return ret;
        }
#endif
#ifdef CONFIG_STM32L4_DAC2
      g_dac2 = stm32l4_dacinitialize(1);
      if (g_dac2 == NULL)
        {
          aerr("ERROR: Failed to get DAC2 interface\n");
          return -ENODEV;
        }

      ret = dac_register("/dev/dac1", g_dac2);
      if (ret < 0)
        {
          aerr("ERROR: dac_register failed: %d\n", ret);
          return ret;
        }
#endif
      UNUSED(ret);
      initialized = true;
    }

  return OK;
}
#endif /* CONFIG_DAC */
