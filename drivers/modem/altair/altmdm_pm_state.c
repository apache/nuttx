/****************************************************************************
 * drivers/modem/altair/altmdm_pm_state.c
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

#include <nuttx/irq.h>

#include <nuttx/modem/altmdm.h>
#include "altmdm_pm_state.h"

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_stateofmodem;

#ifdef CONFIG_MODEM_PM_PUTSTATE
static char *g_putstring[MODEM_PM_INTERNAL_STATE_MAX] =
{
  "SLEEP",
  "GOING_TO_WAKE",
  "WAKE",
  "GOING_TO_SLEEP"
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_pm_getstate
 *
 * Description:
 *   Get current modem state.
 *
 ****************************************************************************/

uint32_t altmdm_pm_getstate(void)
{
  uint32_t get_state;
  uint32_t internal_state;

  internal_state = altmdm_pm_getinternalstate();

  switch (internal_state)
    {
    case MODEM_PM_INTERNAL_STATE_SLEEP:
    case MODEM_PM_INTERNAL_STATE_GOING_TO_WAKE:
      get_state = MODEM_PM_STATE_SLEEP;
      break;

    case MODEM_PM_INTERNAL_STATE_WAKE:
    case MODEM_PM_INTERNAL_STATE_GOING_TO_SLEEP:
      get_state = MODEM_PM_STATE_WAKE;
      break;

    default:
      get_state = MODEM_PM_STATE_WAKE;
      break;
    }

  return get_state;
}

/****************************************************************************
 * Name: altmdm_pm_getinternalstate
 *
 * Description:
 *   Get internal modem state.
 *
 ****************************************************************************/

uint32_t altmdm_pm_getinternalstate(void)
{
  uint32_t get_state;
  irqstate_t flags;

  flags = enter_critical_section();

  get_state = g_stateofmodem;

  leave_critical_section(flags);

  return get_state;
}

/****************************************************************************
 * Name: altmdm_pm_setinternalstate
 *
 * Description:
 *   Set internal modem state.
 *
 ****************************************************************************/

void altmdm_pm_setinternalstate(uint32_t state)
{
  irqstate_t flags;
#ifdef CONFIG_MODEM_PM_PUTSTATE
  uint32_t prev_state;
#endif

  if (state < MODEM_PM_INTERNAL_STATE_MAX)
    {
      flags = enter_critical_section();
#ifdef CONFIG_MODEM_PM_PUTSTATE
      prev_state = g_stateofmodem;
#endif
      g_stateofmodem = state;

      leave_critical_section(flags);
#ifdef CONFIG_MODEM_PM_PUTSTATE
      m_err("MODEM State [%d:%s]-->[%d:%s]\n",
            prev_state, g_putstring[prev_state], state, g_putstring[state]);
#endif
    }
}
#endif
