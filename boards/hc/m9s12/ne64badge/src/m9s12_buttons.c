/****************************************************************************
 * boards/hc/m9s12/ne64badge/src/m9s12_buttons.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "ne64badge.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_INPUT
#  define btnerr        _err
#  define btninfo(x...) _info
#  endif
#else
#  define btnerr(x...)
#  define btninfo(x...)
#endif

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_INPUT
#  define btn_dumpgpio(m) m9s12_dumpgpio(m)
#else
#  define btn_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  /* Configure all button GPIO lines */

  btn_dumpgpio("board_button_initialize() Entry)");

  hcs12_configgpio(NE64BADGE_BUTTON1);
  hcs12_configgpio(NE64BADGE_BUTTON2);

  btn_dumpgpio("board_button_initialize() Exit");

  return 2;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;

  if (hcs12_gpioread(NE64BADGE_BUTTON1))
    {
      ret |= BUTTON1;
    }

  if (hcs12_gpioread(NE64BADGE_BUTTON2))
    {
      ret |= BUTTON2;
    }

  return ret;
}

#endif /* CONFIG_ARCH_BUTTONS */
