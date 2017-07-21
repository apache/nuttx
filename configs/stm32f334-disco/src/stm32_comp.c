/****************************************************************************
 * configs/stm32f334-disco/src/stm32_comp.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/comp.h>

#include "stm32.h"

#if defined(CONFIG_COMP) && (defined(CONFIG_STM32_COMP2) || \
                             defined(CONFIG_STM32_COMP4) || \
                             defined(CONFIG_STM32_COMP6))

#ifdef CONFIG_STM32_COMP2
#  if defined(CONFIG_STM32_COMP4) || defined(CONFIG_STM32_COMP6)
#    error "Currently only one COMP device supported"
#  endif
#elif CONFIG_STM32_COMP4
#  if defined(CONFIG_STM32_COMP2) || defined(CONFIG_STM32_COMP6)
#    error "Currently only one COMP device supported"
#  endif
#elif CONFIG_STM32_COMP6
#  if defined(CONFIG_STM32_COMP2) || defined(CONFIG_STM32_COMP4)
#    error "Currently only one COMP device supported"
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_comp_setup
 *
 * Description:
 *   Initialize COMP
 *
 ****************************************************************************/

int stm32_comp_setup(void)
{
  static bool initialized = false;
  struct comp_dev_s* comp = NULL;
  int ret;

  if (!initialized)
    {
      /* Get the comparator interface */

#ifdef CONFIG_STM32_COMP2
      comp = stm32_compinitialize(2);
      if (comp == NULL)
        {
          aerr("ERROR: Failed to get COMP%d interface\n", 2);
          return -ENODEV;
        }
#endif

#ifdef CONFIG_STM32_COMP4
      comp = stm32_compinitialize(4);
      if (comp == NULL)
        {
          aerr("ERROR: Failed to get COMP%d interface\n", 4);
          return -ENODEV;
        }
#endif

#ifdef CONFIG_STM32_COMP6
      comp = stm32_compinitialize(6);
      if (comp == NULL)
        {
          aerr("ERROR: Failed to get COMP%d interface\n", 6);
          return -ENODEV;
        }
#endif

      /* Register the comparator character driver at /dev/comp0 */

      ret = comp_register("/dev/comp0", comp);
      if (ret < 0)
        {
          aerr("ERROR: comp_register failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_COMP && (CONFIG_STM32_COMP1 ||
        *                 CONFIG_STM32_COMP2
        *                 CONFIG_STM32_COMP6) */
