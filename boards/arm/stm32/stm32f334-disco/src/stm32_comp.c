/****************************************************************************
 * boards/arm/stm32/stm32f334-disco/src/stm32_comp.c
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
  struct comp_dev_s *comp = NULL;
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
